// =============================================================================
// Gary_lidar.cpp
//
// LDROBOT D500 LiDAR driver implementation.
//
// The D500 streams 47-byte packets at ~10 Hz rotation:
//   [0]    header byte 1   = 0x54
//   [1]    header byte 2   = 0x2C   (version=0x2, length=12 points)
//   [2-3]  speed (uint16 LE) in deg/s
//   [4-5]  start angle (uint16 LE), units of 0.01°
//   [6-41] 12 measurement points, each 3 bytes: distance LE (2B) + intensity (1B)
//   [42-43] end angle (uint16 LE), units of 0.01°
//   [44-45] timestamp (uint16 LE) ms, rolls over at 30000
//   [46]   CRC-8
//
// We treat the 12 samples per packet as linearly distributed between start
// and end angles. Each sample gets written into the polar buffer at its
// rounded-to-integer bearing. Multiple samples mapping to the same bearing
// just overwrite — that's fine, we want the freshest reading.
// =============================================================================

#include "Gary_lidar.h"
#include <Arduino.h>
#include <HardwareSerial.h>

// ---- Pin and UART configuration ---------------------------------------------
// IMPORTANT — pin selection rationale:
//   IO18/IO19 are shared with the MM32 servo controller's PB3/PB4
//   programming/debug pins (K12 V1.1 schematic). UART2 traffic there DURING
//   MM32 BOOT can cause a partial servo-init failure (one leg's joint A
//   landing at the wrong angle) — that's why an earlier revision of this
//   driver avoided 18/19 in favor of GPIO32.
//
//   GPIO32/33 are taken by gary_face.h's LED matrix and GPIO34 by
//   hiwonder_board.h's sound_pin, and the physical LiDAR wiring is already
//   run to the 18/19 pair on hardware that's impractical to re-open.
//
//   CONFIRMED WORKING on GPIO18: a real test run showed ready=1, bytes=465,
//   pkts=9, bad=1, with plausible distance readings (F=539mm). Counts froze
//   mid-test, but that coincided with the robot tipping over — almost
//   certainly a jostled connector, not a wrong pin. (GPIO19 was tried as an
//   alternate guess before this confirmation came in; reverted back to 18.)
//
//   Legs have looked normal at power-on across every test so far, so the
//   MM32-boot-timing risk described above hasn't materialized in practice.
//
//   D500 is one-way (TX only — we receive), so we pass -1 for the TX pin
//   and the ESP32 UART driver leaves it unassigned.
static HardwareSerial LidarSerial(2);

static const int      LIDAR_RX_PIN = 18;     // ESP32 receives LiDAR TX here
static const int      LIDAR_TX_PIN = -1;     // unassigned; D500 has no Rx
static const uint32_t LIDAR_BAUD   = 230400;
// RX ring buffer. Was 1024, which was nowhere near enough.
//
// THE ARITHMETIC (measure, don't guess): the D500 spins at ~10Hz and emits
// ~375 packets/sec * 47 bytes = ~17.6 KB/sec. A 1024-byte buffer therefore
// fills solid in 58 MILLISECONDS. Gary's loop() was only coming round every
// 200-340ms, so the buffer overflowed three to six times BETWEEN EVERY DRAIN.
// Overflowed bytes are gone, and what survives is a stream with holes chewed
// out of the middle of packets -- which is where bad=2000 CRC failures came
// from. It was never a wiring or a CRC problem. It was a plumbing problem.
//
// 16KB buys ~900ms of headroom, so even a badly-stalled loop can't lose data.
static const int      LIDAR_RX_BUF = 16384;

// ---- Packet format ----------------------------------------------------------
static const uint8_t  PACKET_HEADER1 = 0x54;
static const uint8_t  PACKET_HEADER2 = 0x2C;
static const int      PACKET_SIZE    = 47;
static const int      POINTS_PER_PACKET = 12;

// ---- CRC-8 table (D500 datasheet) -------------------------------------------
static const uint8_t CRC_TABLE[256] = {
  0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
  0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
  0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
  0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
  0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
  0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
  0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
  0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
  0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
  0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
  0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
  0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
  0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
  0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
  0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
  0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8,
};

static uint8_t crc8(const uint8_t* data, int len) {
  uint8_t c = 0;
  for (int i = 0; i < len; i++) c = CRC_TABLE[(c ^ data[i]) & 0xff];
  return c;
}

// ---- Module state -----------------------------------------------------------

// Polar buffer: distance in millimeters at each integer compass bearing.
// 0 means "no data" or "LiDAR reported invalid reading here."
static uint16_t g_distance_at_angle[360] = {0};

// State machine for incoming packet bytes.
static uint8_t  g_packet_buf[PACKET_SIZE];
static int      g_packet_pos = 0;

// Diagnostics.
static uint32_t g_packets_received = 0;
static uint32_t g_bad_packets      = 0;
static uint32_t g_bytes_received   = 0;

// Full-rotation detection. We watch start_deg cross from large back to small.
static float    g_last_start_deg   = -1.0f;
static bool     g_first_rotation_done = false;

// ---- Packet parser ----------------------------------------------------------

static void parse_packet(const uint8_t* buf) {
  uint16_t start_raw = buf[4]  | (buf[5]  << 8);   // 0.01° units
  uint16_t end_raw   = buf[42] | (buf[43] << 8);   // 0.01° units
  float start_deg = start_raw * 0.01f;
  float end_deg   = end_raw   * 0.01f;

  // Span — handle wrap (end can be smaller than start if packet crosses 360°→0°)
  float span = end_deg - start_deg;
  if (span < 0.0f) span += 360.0f;

  // Per-sample angular step. 11 intervals between 12 samples — NOT 12.
  float step = (POINTS_PER_PACKET > 1) ? (span / (float)(POINTS_PER_PACKET - 1)) : 0.0f;

  for (int i = 0; i < POINTS_PER_PACKET; i++) {
    int off = 6 + i * 3;
    uint16_t dist_mm = buf[off] | (buf[off + 1] << 8);
    // intensity = buf[off + 2];  // not used yet, but available for filtering

    float ang = start_deg + step * (float)i;
    while (ang >= 360.0f) ang -= 360.0f;
    while (ang <    0.0f) ang += 360.0f;

    int bin = (int)(ang + 0.5f);  // nearest integer degree
    if (bin >= 360) bin = 0;
    if (bin <    0) bin = 0;

    g_distance_at_angle[bin] = dist_mm;   // 0 from LiDAR = invalid, preserved
  }

  // Detect full-rotation completion: start_deg dropped (wrapped back to ~0)
  if (g_last_start_deg >= 0.0f && start_deg < g_last_start_deg - 90.0f) {
    g_first_rotation_done = true;
  }
  g_last_start_deg = start_deg;
}

// ---- Byte-level state machine -----------------------------------------------

static void feed_byte(uint8_t b) {
  if (g_packet_pos == 0) {
    if (b == PACKET_HEADER1) g_packet_buf[g_packet_pos++] = b;
    return;
  }
  if (g_packet_pos == 1) {
    if (b == PACKET_HEADER2) {
      g_packet_buf[g_packet_pos++] = b;
    } else {
      g_packet_pos = 0;
      if (b == PACKET_HEADER1) g_packet_buf[g_packet_pos++] = b;
    }
    return;
  }

  g_packet_buf[g_packet_pos++] = b;

  if (g_packet_pos >= PACKET_SIZE) {
    // Full packet — verify CRC
    uint8_t expected = g_packet_buf[PACKET_SIZE - 1];
    uint8_t computed = crc8(g_packet_buf, PACKET_SIZE - 1);
    if (computed == expected) {
      parse_packet(g_packet_buf);
      g_packets_received++;
      g_packet_pos = 0;
    } else {
      g_bad_packets++;
      // Misalignment recovery: scan for next header start inside the buffer
      // rather than discarding all 47 bytes.
      int next_header = -1;
      for (int i = 1; i < PACKET_SIZE - 1; i++) {
        if (g_packet_buf[i] == PACKET_HEADER1 && g_packet_buf[i + 1] == PACKET_HEADER2) {
          next_header = i;
          break;
        }
      }
      if (next_header > 0) {
        int remain = PACKET_SIZE - next_header;
        memmove(g_packet_buf, g_packet_buf + next_header, remain);
        g_packet_pos = remain;
      } else {
        g_packet_pos = 0;
      }
    }
  }
}

// ---- Public API -------------------------------------------------------------

void gary_lidar_begin() {
  LidarSerial.setRxBufferSize(LIDAR_RX_BUF);
  LidarSerial.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  for (int i = 0; i < 360; i++) g_distance_at_angle[i] = 0;
  g_packet_pos = 0;
  g_packets_received = 0;
  g_bad_packets = 0;
  g_bytes_received = 0;
  g_last_start_deg = -1.0f;
  g_first_rotation_done = false;
}

// ---- The reader task --------------------------------------------------------
//
// WHY THIS EXISTS -- the single biggest bug in Gary's sensing stack.
//
// gary_lidar_update() used to be called only from loop(), once per iteration,
// and it read at most 400 bytes per call. Measured, loop() runs at 3-6 Hz. So
// the absolute ceiling on LiDAR ingest was:
//
//     400 bytes/call * ~5 calls/sec = 2,000 bytes/sec
//
// against an incoming stream of ~17,600 bytes/sec. Gary was discarding over 90%
// of his own LiDAR. Confirmed from his diag counters: 1,364 bytes/sec actually
// ingested, and gary_lidar_diag.py measured only ~138 of the 360 bearings being
// refreshed per second -- meaning a "360-degree scan" was in fact a MOSAIC
// smeared over ~2.6 seconds of walking. Every map was blurred by it, mover
// detection was comparing live returns against seconds-stale bearings, and
// frame-to-frame scan-matching saw no motion at all (which is what finally
// exposed this: the odometry regression came back all zeros).
//
// The mistake was structural: LiDAR ingest was chained to the main loop's rate.
// So cut the chain. This task runs on its own, wakes every 2ms, and drains the
// UART completely -- 500 wakeups/sec instead of 5. Whatever loop() is doing, and
// however slow it gets, the LiDAR keeps being read.
//
// CONCURRENCY, honestly: the task writes g_distance_at_angle[] while the main
// loop reads it. Deliberately unlocked. Each entry is a naturally-aligned 16-bit
// store, which is atomic on the ESP32, so a reader can never see a half-written
// distance -- the worst case is a ring whose bearings are from slightly
// different instants. That was ALREADY true (and vastly worse) before this
// change, and a mutex here would just hand the stall back to the LiDAR.
static TaskHandle_t g_lidar_task = nullptr;
static volatile bool g_task_running = false;

static void lidar_task(void* arg) {
  (void)arg;
  for (;;) {
    while (LidarSerial.available()) {
      g_bytes_received++;
      feed_byte((uint8_t)LidarSerial.read());
    }
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

void gary_lidar_start_task() {
  if (g_lidar_task) return;
  // Priority 2 = above the Arduino loopTask (1), so it preempts a busy loop
  // instead of politely waiting for it. Core 1 = the app core, alongside the
  // loop; core 0 is the radio core and BLE/WiFi already live there.
  BaseType_t ok = xTaskCreatePinnedToCore(
      lidar_task, "lidar", 4096, nullptr, 2, &g_lidar_task, 1);
  g_task_running = (ok == pdPASS);
}

void gary_lidar_update() {
  // Once the reader task is up it owns the UART and the packet state machine.
  // Draining from two contexts would interleave bytes into feed_byte() and
  // corrupt every packet, so the legacy call sites (loop(), gary_delay())
  // become no-ops. Left as a no-op rather than deleted so nothing breaks if a
  // call site is missed, and so this still works if the task ever fails to
  // start.
  if (g_task_running) return;

  while (LidarSerial.available()) {
    g_bytes_received++;
    feed_byte((uint8_t)LidarSerial.read());
  }
}

uint16_t gary_lidar_distance_at_angle(int deg) {
  while (deg >= 360) deg -= 360;
  while (deg <    0) deg += 360;
  return g_distance_at_angle[deg];
}

uint16_t gary_lidar_min_distance(int start_deg, int end_deg) {
  // Normalize bounds to [0, 360).
  while (start_deg >= 360) start_deg -= 360;
  while (start_deg <    0) start_deg += 360;
  while (end_deg   >= 360) end_deg   -= 360;
  while (end_deg   <    0) end_deg   += 360;

  uint16_t min_val = 0;
  // Iterate around the arc, handling wrap (start_deg > end_deg case).
  int d = start_deg;
  while (true) {
    uint16_t v = g_distance_at_angle[d];
    if (v > 0 && (min_val == 0 || v < min_val)) min_val = v;
    if (d == end_deg) break;
    d++;
    if (d >= 360) d = 0;
  }
  return min_val;
}

bool     gary_lidar_is_ready()         { return g_first_rotation_done; }
uint32_t gary_lidar_packets_received() { return g_packets_received; }
uint32_t gary_lidar_bad_packets()      { return g_bad_packets; }
uint32_t gary_lidar_bytes_received()   { return g_bytes_received; }
