// =============================================================================
// gary_v1.ino
//
// Gary — a MiniHexa K12 v1.0 robot personality firmware.
// Built on top of Hiwonder's open-source MiniHexa libraries.
//
// v1 SCOPE:
//   - Voice commands via WonderEcho (FACTORY firmware, NOT custom)
//   - R2-D2-style emotional buzzer sounds (NON-BLOCKING — Gary moves AND beeps)
//   - Free-roam with ultrasonic obstacle avoidance
//   - Emotional eye color state machine
//   - Sound-startle reactivity
//   - Self-balancing while idle
//   - Battery monitoring with low-batt alarm
//   - Tip detection
//   - "Come here" face-tracking (when camera enabled)
//
// VOICE NOTE: Gary uses the WonderEcho FACTORY firmware. To talk to Gary:
//   1. Wake: "HELLO-HI-WONDER" or "Hey Gary" → "I'm here"
//   2. Then say a factory command phrase. Examples:
//        "go straight" → forward
//        "stop"        → halt
//        "dance"       → dance routine
//        "execute action one" → custom Gary action 1
//   3. After ~5s of silence Gary drifts back to roaming.
//
// SOUND NOTE: Sounds are NON-BLOCKING. Triggering a sound returns immediately;
// the buzzer plays in the background while motion continues. To make this
// work, every delay() call in the action sequences uses gary_delay() which
// keeps pumping gary_sounds_update() during the wait. Don't use plain delay()
// inside action sequences.
//
// NOT IN v1 (deferred): touch sensor, IR sensors, VL53L0X cliff array,
//   D500 LiDAR, PS5 controller, Pi co-processor, person-specific recognition,
//   custom WonderEcho firmware with R2-D2 sound clips.
// =============================================================================

#include "hiwonder_robot.h"
#include "gary_config.h"
#include "gary_sounds.h"
#include "gary_face.h"
#include "gary_lidar.h"
#include "gary_knee_guard.h"
#include "pu_ble_client.h"
#include <Bluepad32.h>

// -----------------------------------------------------------------------------
// WiFi scan streaming -- lets gary_lidar_view.py / gary_lidar_map.py connect
// over the network ("python gary_lidar_map.py tcp:<Gary's IP>:3333") instead
// of a USB cable.
// -----------------------------------------------------------------------------
// FILL THESE IN with your own network's credentials before flashing. This is
// your robot's own firmware talking to your own router over your own local
// network -- nothing here leaves your network or goes anywhere external.
#include "wifi_secrets.h"   // WIFI_SSID / WIFI_PASSWORD live here (gitignored; see wifi_secrets.example.h)
#define WIFI_SCAN_PORT 3333

#include <WiFi.h>
#include <ArduinoOTA.h>   // wireless flashing (ENABLE_OTA, 2026-08-18)
WiFiServer wifi_scan_server(WIFI_SCAN_PORT);
WiFiClient wifi_scan_client;
bool wifi_connected = false;

void wifi_begin() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print(">>> WiFi connecting");
  uint32_t start = millis();
  // Give it up to 8s at boot, then move on regardless -- a WiFi hiccup
  // should never hold up the rest of Gary's boot sequence. Bluepad32 (BLE)
  // is already running by this point; WiFi + BLE share the same radio and
  // time-slice reasonably well on ESP32, but this is the first time this
  // sketch runs both together, so watch for controller lag/hiccups the
  // first few times you test it with WiFi active.
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    wifi_scan_server.begin();
    Serial.print(">>> WiFi connected. Scan server at tcp:");
    Serial.print(WiFi.localIP());
    Serial.print(":");
    Serial.println(WIFI_SCAN_PORT);
#if ENABLE_OTA
    ota_begin();   // wireless flashing — defined below the globals it touches
#endif
  } else {
    Serial.println(">>> WiFi connect FAILED (timed out) -- continuing without it. Scan data is still available over USB serial either way.");
  }
}

void wifi_pump() {
  if (!wifi_connected) return;
  // Accept a new client if one's waiting. Only one viewer at a time for
  // simplicity -- a fresh connection just replaces whatever was there.
  if (wifi_scan_server.hasClient()) {
    if (wifi_scan_client) wifi_scan_client.stop();
    wifi_scan_client = wifi_scan_server.available();
  }
}

void wifi_scan_broadcast(const char* line) {
  if (wifi_connected && wifi_scan_client && wifi_scan_client.connected()) {
    wifi_scan_client.print(line);
  }
}

// -----------------------------------------------------------------------------
// WiFi CSI (Channel State Information) sensing -- runs on Gary's own radio,
// piggybacking off the same STA connection already used for scan streaming
// (no second ESP32 needed). Triggered by voice command "execute action
// thirteen" (CMD_ACTION_13) -- see STATE_SENSING below.
//
// CSI measures how the WiFi signal between Gary and the router is being
// scattered by everything in the room -- a real (if noisy, research-grade)
// way to sense that something moved nearby, meant as an extra layer on top
// of the LiDAR map, NOT a replacement for LiDAR's precise geometry.
//
// IMPORTANT: this only works while GARY HIMSELF IS STANDING STILL. His own
// motion dominates the channel and completely swamps any signal from
// something else moving in the room -- that is exactly why this is gated
// behind an explicit stand-still state (STATE_SENSING) instead of running
// continuously in the background while he walks/roams.
// -----------------------------------------------------------------------------
#include "esp_wifi.h"

// Manual forward declarations -- see the long comment on csi_rx_cb() below
// for why these are spelled out explicitly instead of relying on Arduino's
// automatic prototype generation.
void csi_push_sample(uint32_t t_ms, int8_t rssi, float mean_amp, float variance, int len);
void csi_rx_cb(void *ctx, wifi_csi_info_t *info);
void csi_pump();
void csi_capture_begin();
void csi_capture_end();
void nodscan_emit_slice(float pitch_deg, float roll_deg);
bool nodscan_check_unstable(float commanded_pitch_deg, float commanded_roll_deg);
void run_nod_scan();
void loop_autoscan();
void autoscan_csi_drain();
bool autoscan_check_trip();
void csi_pump_autoscan_status();

#define CSI_CAPTURE_DURATION_MS 8000   // how long STATE_SENSING holds still + captures

volatile bool csi_capturing = false;
volatile uint32_t csi_packet_count = 0;

struct CsiSample {
  uint32_t t_ms;
  int8_t   rssi;
  float    mean_amp;
  float    variance;
  int      len;
};

// Small single-producer (WiFi task, via the CSI callback) / single-consumer
// (main loop) ring buffer. The callback must NOT do Serial.print itself --
// running Serial I/O from inside a WiFi-task callback is a known source of
// instability on ESP32 -- so it only computes a couple of cheap summary
// stats and queues them here; csi_pump() drains the queue from loop_sensing().
#define CSI_RING_SIZE 32
volatile CsiSample csi_ring[CSI_RING_SIZE];
volatile uint16_t csi_ring_head = 0;
volatile uint16_t csi_ring_tail = 0;

void csi_push_sample(uint32_t t_ms, int8_t rssi, float mean_amp, float variance, int len) {
  uint16_t next = (csi_ring_head + 1) % CSI_RING_SIZE;
  if (next == csi_ring_tail) return;  // ring full -- drop the sample rather than block the WiFi task
  csi_ring[csi_ring_head].t_ms     = t_ms;
  csi_ring[csi_ring_head].rssi     = rssi;
  csi_ring[csi_ring_head].mean_amp = mean_amp;
  csi_ring[csi_ring_head].variance = variance;
  csi_ring[csi_ring_head].len      = len;
  csi_ring_head = next;
}

void csi_rx_cb(void *ctx, wifi_csi_info_t *info) {
  if (!csi_capturing || !info || !info->buf || info->len <= 0) return;

  // Cheap summary computed right here: mean absolute amplitude and variance
  // across subcarriers. Rising variance/magnitude relative to a quiet-room
  // baseline is the same first-pass feature real CSI presence-detection
  // systems key off of -- we're not attempting full localization here.
  int8_t* buf = info->buf;
  int n = info->len;
  int32_t sum = 0;
  int32_t sum_sq = 0;
  for (int i = 0; i < n; i++) {
    int32_t v = buf[i];
    sum += abs(v);
    sum_sq += v * v;
  }
  float mean_amp = (float)sum / n;
  float mean_sq  = (float)sum_sq / n;
  float variance = mean_sq - (mean_amp * mean_amp);

  csi_packet_count++;
  csi_push_sample(millis(), info->rx_ctrl.rssi, mean_amp, variance, n);
}

void csi_pump() {
  // Safe to Serial.print here -- this runs from the main loop, not the
  // WiFi task. Tags each line "CSI:" so the Python side can tell it apart
  // from "SCAN:" lines sharing the same Serial/TCP stream.
  //
  // NOTE: csi_ring is `volatile` (written from the WiFi task's callback),
  // so it can't be copied via a plain struct assignment -- the compiler
  // has no volatile-qualified copy constructor for CsiSample. Read each
  // field out individually instead; built-in types support volatile reads
  // directly, no copy constructor needed.
  while (csi_ring_tail != csi_ring_head) {
    uint32_t t_ms    = csi_ring[csi_ring_tail].t_ms;
    int8_t   rssi    = csi_ring[csi_ring_tail].rssi;
    float    mean_amp = csi_ring[csi_ring_tail].mean_amp;
    float    variance = csi_ring[csi_ring_tail].variance;
    int      len      = csi_ring[csi_ring_tail].len;
    csi_ring_tail = (csi_ring_tail + 1) % CSI_RING_SIZE;

    char line[96];
    snprintf(line, sizeof(line), "CSI:%lu,%d,%.1f,%.1f,%d\n",
             (unsigned long)t_ms, (int)rssi, mean_amp, variance, len);
    Serial.print(line);
    wifi_scan_broadcast(line);
  }
}

void csi_capture_begin() {
  csi_packet_count = 0;
  csi_ring_head = 0;
  csi_ring_tail = 0;

  wifi_csi_config_t csi_config = {
    .lltf_en = true,
    .htltf_en = true,
    .stbc_htltf2_en = true,
    .ltf_merge_en = true,
    .channel_filter_en = false,
    .manu_scale = false,
    .shift = 0,
  };
  esp_wifi_set_csi_config(&csi_config);
  esp_wifi_set_csi_rx_cb(&csi_rx_cb, NULL);
  esp_wifi_set_csi(true);
  csi_capturing = true;
  Serial.println(">>> CSI capture started -- holding still");
}

void csi_capture_end() {
  csi_capturing = false;
  esp_wifi_set_csi(false);
  Serial.printf(">>> CSI capture done -- %lu packets captured\n", (unsigned long)csi_packet_count);
}

// -----------------------------------------------------------------------------
// AUTOSCAN -- STATE_AUTOSCAN's background CSI presence monitor. Reuses the
// same csi_capture_begin()/csi_ring plumbing as STATE_SENSING, but instead of
// one fixed 8s burst dumped straight to Serial, it runs indefinitely (until
// toggled off), keeps a slow-moving RSSI baseline, and watches a short recent
// window for the same "range widens / mean drops" signature validated across
// trials 6-9 in this environment. Serial output is throttled to a periodic
// status line plus trip/scan events -- printing every CSI: packet the way
// loop_sensing() does would flood the log over a watch that can run for
// minutes or hours.
// -----------------------------------------------------------------------------
#define AUTOSCAN_BASELINE_WINDOW      40    // samples folded into the slow baseline EMA
#define AUTOSCAN_MIN_BASELINE_SAMPLES 15    // don't evaluate trips until baseline has this many
#define AUTOSCAN_TRIP_WINDOW           8    // recent-sample window checked for a trip
#define AUTOSCAN_TRIP_RANGE_DB        6.0f  // recent min/max spread that counts as a trip (still ran ~1-2dB, active ran 8-12dB across trials 6-9)
#define AUTOSCAN_TRIP_DELTA_DB        4.0f  // recent mean vs baseline mean drop that also counts as a trip
#define AUTOSCAN_COOLDOWN_MS         4000   // hold off re-triggering immediately after a triggered scan
#define AUTOSCAN_STATUS_INTERVAL_MS  5000   // how often the periodic status line prints

enum AutoscanPhase { AUTOSCAN_WARMUP, AUTOSCAN_MONITOR, AUTOSCAN_COOLDOWN };
AutoscanPhase autoscan_phase = AUTOSCAN_WARMUP;
float    autoscan_baseline_rssi = 0.0f;
uint16_t autoscan_baseline_n = 0;
int8_t   autoscan_recent_rssi[AUTOSCAN_TRIP_WINDOW];
uint8_t  autoscan_recent_count = 0;
uint8_t  autoscan_recent_idx = 0;
uint32_t autoscan_cooldown_until_ms = 0;
uint32_t autoscan_triggered_count = 0;

// Drains the CSI ring into the rolling baseline + recent-window buffers.
// Doesn't print per-packet on Serial (that would flood a watch running for
// minutes/hours) -- see csi_pump_autoscan_status() for the periodic summary.
// It DOES throttle a lightweight "RSSI:<rssi>,<baseline>" line out over
// WiFi only (not Serial) so the live dashboard's strip chart gets a much
// livelier update rate than the 5s status line without spamming the log.
#define AUTOSCAN_RSSI_STREAM_MS 300
void autoscan_csi_drain() {
  static uint32_t last_rssi_stream_ms = 0;
  while (csi_ring_tail != csi_ring_head) {
    int8_t rssi = csi_ring[csi_ring_tail].rssi;
    csi_ring_tail = (csi_ring_tail + 1) % CSI_RING_SIZE;

    autoscan_recent_rssi[autoscan_recent_idx] = rssi;
    autoscan_recent_idx = (autoscan_recent_idx + 1) % AUTOSCAN_TRIP_WINDOW;
    if (autoscan_recent_count < AUTOSCAN_TRIP_WINDOW) autoscan_recent_count++;

    uint32_t now = millis();
    if (wifi_connected && now - last_rssi_stream_ms >= AUTOSCAN_RSSI_STREAM_MS) {
      last_rssi_stream_ms = now;
      static char rssi_buf[48];
      snprintf(rssi_buf, sizeof(rssi_buf), "RSSI:%d,%.1f\n", rssi, autoscan_baseline_rssi);
      wifi_scan_broadcast(rssi_buf);  // WiFi-only, deliberately not Serial.print'd
    }

    // Only fold samples into the slow baseline while calmly monitoring --
    // not during warmup's very first samples being weighted too heavily,
    // and not while cooling down from a trip (that would let the active-
    // phase RSSI drop get absorbed into "normal").
    if (autoscan_phase == AUTOSCAN_MONITOR || autoscan_phase == AUTOSCAN_WARMUP) {
      if (autoscan_baseline_n < AUTOSCAN_BASELINE_WINDOW) {
        autoscan_baseline_rssi = (autoscan_baseline_rssi * autoscan_baseline_n + rssi) / (autoscan_baseline_n + 1);
        autoscan_baseline_n++;
      } else {
        const float alpha = 1.0f / AUTOSCAN_BASELINE_WINDOW;
        autoscan_baseline_rssi = autoscan_baseline_rssi * (1.0f - alpha) + rssi * alpha;
      }
    }
  }
}

// True if the recent RSSI window looks like an active/motion signature
// relative to the current baseline -- either a wide min/max spread or a
// dropped mean, matching what trials 6-9 showed for "still" vs "active".
bool autoscan_check_trip() {
  if (autoscan_recent_count < AUTOSCAN_TRIP_WINDOW) return false;
  if (autoscan_baseline_n < AUTOSCAN_MIN_BASELINE_SAMPLES) return false;

  int8_t rmin = autoscan_recent_rssi[0], rmax = autoscan_recent_rssi[0];
  int32_t rsum = 0;
  for (int i = 0; i < AUTOSCAN_TRIP_WINDOW; i++) {
    int8_t v = autoscan_recent_rssi[i];
    if (v < rmin) rmin = v;
    if (v > rmax) rmax = v;
    rsum += v;
  }
  float recent_mean = (float)rsum / AUTOSCAN_TRIP_WINDOW;
  float range = (float)(rmax - rmin);
  float delta = autoscan_baseline_rssi - recent_mean;  // positive = recent is weaker (more negative dBm)

  if (range >= AUTOSCAN_TRIP_RANGE_DB || delta >= AUTOSCAN_TRIP_DELTA_DB) {
    // Broadcast over WiFi too, not just Serial -- the live dashboard reads
    // this over the same TCP stream as SCAN:/NOD: to flag trip events on
    // its RSSI chart in real time.
    static char trip_buf[128];
    snprintf(trip_buf, sizeof(trip_buf),
             ">>> AUTOSCAN: TRIP -- baseline=%.1fdBm recent_mean=%.1fdBm range=%.1fdB delta=%.1fdB\n",
             autoscan_baseline_rssi, recent_mean, range, delta);
    Serial.print(trip_buf);
    wifi_scan_broadcast(trip_buf);
    return true;
  }
  return false;
}

void csi_pump_autoscan_status() {
  static uint32_t last_status_ms = 0;
  if (millis() - last_status_ms > AUTOSCAN_STATUS_INTERVAL_MS) {
    last_status_ms = millis();
    const char* phase_name = autoscan_phase == AUTOSCAN_WARMUP  ? "WARMUP"
                            : autoscan_phase == AUTOSCAN_MONITOR ? "MONITOR"
                                                                  : "COOLDOWN";
    // Same WiFi broadcast treatment as the TRIP line above.
    static char status_buf[128];
    snprintf(status_buf, sizeof(status_buf),
             ">>> AUTOSCAN status | phase=%s baseline=%.1fdBm samples=%u triggered=%lu\n",
             phase_name, autoscan_baseline_rssi, autoscan_baseline_n,
             (unsigned long)autoscan_triggered_count);
    Serial.print(status_buf);
    wifi_scan_broadcast(status_buf);
  }
}


// -----------------------------------------------------------------------------
// Global robot object
// -----------------------------------------------------------------------------
Robot gary;

// -----------------------------------------------------------------------------
// State enumeration
// -----------------------------------------------------------------------------
enum GaryState {
  STATE_BOOT,
  STATE_IDLE,
  STATE_LISTENING,
  STATE_EXECUTING,
  STATE_ROAMING,
  STATE_COMING,
  STATE_STARTLED,
  STATE_ALARMED,
  STATE_CONTROLLER, // PS5 DualSense controller is in command
  STATE_SENSING,    // holding still, capturing WiFi CSI (see CMD_ACTION_13)
  STATE_NODSCAN,    // holding still, sweeping body pitch+roll to build a 3D LiDAR scan (see CMD_ACTION_14)
  STATE_AUTOSCAN,   // toggleable background watch: passive CSI presence monitor that
                     // auto-fires a NODSCAN when it trips (see CMD_ACTION_15)
  STATE_EXPLORE     // autonomous roam with the sim-trained reflex stack (see CMD_ACTION_16)
};

GaryState current_state = STATE_BOOT;
GaryState previous_state = STATE_BOOT;
uint32_t state_entered_ms = 0;
uint32_t last_voice_check_ms = 0;
uint32_t last_boredom_kick_ms = 0;
uint16_t last_sound_value = 0;
uint32_t startle_suppress_until_ms = 0;  // ignore sound-startle while his own voice/ack plays

// Coming-mode sub-state
enum ComingPhase { COMING_SEARCH, COMING_APPROACH, COMING_ARRIVED };
ComingPhase coming_phase = COMING_SEARCH;

// LISTENING two-tier timeout tracker.
bool listening_got_command = false;

// Movement targets.
Velocity_t vel = {0.0f, 0.0f, 0.0f};
Vector_t pos = {0.0f, 0.0f, 1.5f};
Euler_t att = {0.0f, 0.0f, 0.0f};

#if ENABLE_OTA
// --- ArduinoOTA (2026-08-18): wireless flashing — no more USB plug/unplug. ---
// In Arduino IDE: Tools -> Port -> "gary at <ip> (network)" appears once he's
// booted on WiFi; upload exactly like a serial port. Password in wifi_secrets.h.
// Motion is halted the moment an OTA upload starts (zero velocity, stand pose),
// because flashing pauses the main loop and we don't want a mid-gait freeze
// holding torque on a bent knee. Called from setup() after wifi_begin(); lives
// down here because it touches `gary` and `pos`, declared above.
void ota_begin() {
  if (!wifi_connected) return;
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() {
    Serial.println(">>> OTA: upload starting — halting motion");
    Velocity_t vz = {0.0f, 0.0f, 0.0f};
    Euler_t    na = {0.0f, 0.0f, 0.0f};
    gary.move(&vz, &pos, &na, 500);         // zero velocity at current stand pose
    gary.func_state = Robot::CRAWL;         // library default state
  });
  ArduinoOTA.onEnd([]() { Serial.println(">>> OTA: done, rebooting"); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf(">>> OTA: error %u\n", e); });
  ArduinoOTA.begin();
  Serial.printf(">>> OTA ready: hostname '%s' (network port in Arduino IDE)\n", OTA_HOSTNAME);
}
#endif

// Camera readings buffer
uint8_t camera_buf[4] = {0};

// Battery state
uint16_t last_battery_mv = 8400;

// =============================================================================
// HELPER FUNCTIONS
// =============================================================================

// Forward declarations
void return_to_listening();
const char* state_name(GaryState s);
void on_state_enter(GaryState s);
void transition_to(GaryState new_state);   // was missing -- the one GaryState-using
                                            // function without a manual prototype,
                                            // which is what let Arduino's ctags-based
                                            // auto-prototype generator hoist a garbled
                                            // copy above the enum and break the build

// -----------------------------------------------------------------------------
// gary_delay(ms) — replacement for delay() that keeps the buzzer track alive.
// Use this in action sequences where you want sound to keep playing during
// the wait. For super-short waits (<10ms) plain delay() is fine.
//
// Also pumps gary_lidar_update() -- found via the nod-scan point-cloud work
// that this was missing. gary_lidar_update() parses the LiDAR's UART stream
// and only gets called from the main loop() otherwise; any code (like
// run_nod_scan()) that blocks in gary_delay() for a few hundred ms was
// starving the parser, so gary_lidar_distance_at_angle() kept returning
// whatever buffer existed before the wait started -- e.g. nod-scan was
// emitting the SAME stale 360-point scan at every pitch step instead of a
// fresh one. Pumping it here fixes that for nod-scan and for any other
// gary_delay() call in the sketch that happens to run while LiDAR data
// matters.
// -----------------------------------------------------------------------------
void gary_delay(uint32_t ms) {
  uint32_t end_t = millis() + ms;
  while ((int32_t)(end_t - millis()) > 0) {
    gary_sounds_update();
    gary_lidar_update();
    delay(2);
  }
}

// -----------------------------------------------------------------------------
// 3D "nod scan" -- Gary has no dedicated tilting LiDAR gimbal, but he can
// pitch his WHOLE BODY via IK. Since the LiDAR is rigidly mounted to the
// body, pitching sweeps the LiDAR's 2D scan plane through a range of
// vertical angles -- capture one scan at each pitch step, tag it with that
// pitch, and there's enough information to reconstruct a 3D point cloud on
// the Python side. No new hardware needed, same trick as CMD_ACTION_13's
// CSI sensing (repurpose what's already onboard).
//
// The sweep range is well inside CLAMP_PITCH_DEG (currently 35 deg -- see
// gary_config.h, raised specifically to give this feature headroom) and
// stays far under IMU_TIP_THRESHOLD_DEG (45 deg), so it can never bump the
// pitch clamp or trip the tip alarm.
// -----------------------------------------------------------------------------
// v3 -- turns out v1's "tip-over" almost certainly wasn't one: Gary was
// watched directly and just did a quick bow-like dip, not a fall. The wild
// roll numbers (-166, then a sample flagged "ignoring bad sample" at 178,
// then bouncing through 140/39/-16 on successive reads) are a noise
// signature, not a body at rest on the ground -- a robot actually down
// holds a roughly constant reading, it doesn't oscillate like that. Most
// likely cause: the old code snapped from +25 deg pitch back to 0 in a
// single fast 500ms move at the very end -- a much bigger, faster attitude
// change than any of the 5-degree sweep steps -- and that's a plausible way
// to get a transient IMU glitch (vibration/current draw) that falsely
// tripped the tip-alarm's 3-consecutive-bad-reading check even though
// Gary himself was fine.
//
// So: sweep range restored to +/-25 deg (back to full vertical coverage,
// since the amplitude itself probably wasn't the real problem). Two actual
// fixes instead: the end-of-sweep return-to-level move is now slower and
// separate from the per-step move duration (the specific fast snap that's
// the likely noise source), and the live stability check now requires TWO
// consecutive bad readings (150ms apart) before aborting, instead of
// trusting a single sample -- matching the same debounce logic
// loop_alarmed() already uses for the real tip-alarm, so a lone noise spike
// can't cancel a perfectly fine scan.
#define NODSCAN_PITCH_MIN_DEG  -25.0f
#define NODSCAN_PITCH_MAX_DEG   25.0f
#define NODSCAN_PITCH_STEP_DEG   5.0f
#define NODSCAN_MOVE_MS          500    // move duration for each 5-degree sweep step
#define NODSCAN_SETTLE_MS        500    // extra hold time after each step's move before capturing
#define NODSCAN_RETURN_MS       1200    // slower, gentler move for the final return-to-level (was the likely noise source at 500ms)
#define NODSCAN_RETURN_SETTLE_MS 700    // extra settle after the slow return, before handing control back
#define NODSCAN_PITCH_FAN_HEIGHT  0.0f  // pass 1's height -- all the way down, for max stability
                                         // margin during the big +/-25 pitch lean. This is the
                                         // ORIGINAL value; a later revision (see v4 UPDATE below)
                                         // raised a single shared height constant to 1.0 to help
                                         // the roll sweep, which quietly cost pass 1 its stability
                                         // margin too and caused it to lose balance on its very
                                         // first move (pitch=-25) with no roll commanded at all --
                                         // now split into two constants so tuning one can't
                                         // regress the other.
// NODSCAN_ABORT_ROLL_DEG: how far off the COMMANDED pitch/roll is tolerated
// before this counts as instability. Was 18.0 -- three straight real tests
// aborted on pass 1's first move at a consistent, non-worsening ~16-17
// degree off-target reading (see the step-0 slowdown fix above, which should
// reduce this in the first place). 18 was overly tight relative to what this
// robot can actually recover from: the REAL tip-alarm elsewhere in this
// firmware (run_background_checks(), IMU_TIP_THRESHOLD_DEG) doesn't fire
// until 45 degrees, sustained for 3 consecutive readings. This nod-scan-only
// check was almost 2.5x more sensitive than the check that actually protects
// Gary from falling. Raised to 28 -- real margin above the ~16-17 observed,
// still well under the actual 45-degree fall threshold, so this check can
// keep doing its job (catching a genuine runaway) without tripping on a
// wobble that was never going anywhere near a real tip-over.
#define NODSCAN_ABORT_ROLL_DEG   28.0f
#define NODSCAN_ABORT_RECHECK_MS  150   // gap between the two confirmation samples

// -----------------------------------------------------------------------------
// v4 -- side-roll sweep. A pure pitch sweep only ever tilts the LiDAR's scan
// plane fore/aft, so the point cloud is a fan of rings all sharing the same
// "roll = level" cross-section -- it never sees what's directly to Gary's
// left/right beyond the ring's own azimuth spread. Sweeping roll as well
// (full pitch fan at roll=-15, again at roll=0, again at roll=+15) tilts
// that fan sideways too, filling in coverage a pitch-only sweep misses.
//
// Cost: this roughly triples scan time (~13s -> ~40s) since it's now a
// pitch x roll grid instead of a single pitch line. Dial NODSCAN_ROLL_STEP_DEG
// up (fewer roll levels) or NODSCAN_PITCH_STEP_DEG up (coarser fan) to trade
// coverage for speed.
//
// Every roll-LEVEL change (not every pitch step) is routed through a slow,
// controlled pitch=0 waypoint first -- jumping straight from
// (pitch=+25, roll=R) to (pitch=-25, roll=R') would be a much bigger, faster
// attitude change than any single 5-degree sweep step, which is exactly the
// kind of fast snap that caused a false tip-alarm trip on an earlier test
// (see the v3 comment above run_nod_scan()).
//
// UPDATE (first real hardware test, roll=-15): aborted partway through the
// pitch sweep at that roll level -- commanded roll=-15.0, IMU actually read
// ~+5 (roll1=5.1, roll2=5.5), a ~20 degree gap, well past
// NODSCAN_ABORT_ROLL_DEG. Nothing tipped -- the abort check did exactly its
// job and returned to level safely -- but it means -15/+15 roll combined
// with the full +/-25 pitch sweep at a near-flat-on-the-ground crouch simply
// isn't reachable as a PURE rotation: the legs on the "up" side run out of
// retraction travel with so little height to work with.
//
// UPDATE 2 (height compensation, second real test): raised body height with
// |roll| (NODSCAN_CROUCH_HEIGHT up to a taller value at the roll extremes)
// and re-tried the same min->max roll-level grid. Narrowed the gap a little
// (roll1/roll2 landed at 3.6/3.9 instead of 5.1/5.5, still commanded -15)
// but still aborted at the same spot. Two things this run also exposed: (a)
// the grid always tries roll=NODSCAN_ROLL_MIN_DEG (the negative extreme)
// FIRST, so both real tests aborted there before ever even attempting
// roll=0 or the positive extreme -- one side of the room was never actually
// scanned, not because it doesn't work, but because the loop never got
// there; (b) the grid's worst-case ask is a corner -- full pitch AND full
// roll AT THE SAME TIME (e.g. pitch=+25, roll=-15, a combined tilt of
// sqrt(25^2+15^2) ~= 29 degrees from level) -- well past what either axis
// needs alone.
//
// UPDATE 3 (circular sweep, replaces the rectangular grid): instead of a
// pitch x roll grid, pass 2 below traces a CIRCLE in pitch-roll space --
// pitch = R*cos(theta), roll = R*sin(theta) as theta sweeps a full loop.
// This is exactly the "ball tipped and rolled in place" motion: nod forward
// (theta=0) -> lean to one side (theta=90) -> nod back (theta=180) -> lean
// to the other side (theta=270) -> back to forward, as one continuous loop.
// The key property: total tilt magnitude is a CONSTANT R the entire way
// around, so it never asks for the "full pitch AND full roll" corner that
// the grid always needed at its hardest points -- and because it visits
// both roll directions within one uninterrupted loop instead of trying the
// negative extreme first and possibly dying there, an abort partway around
// still leaves you with real coverage of whatever side it got through,
// rather than always losing the same side. Pass 1 (the original,
// already-proven pitch-only fan at roll=0) stays as-is for full vertical
// resolution straight ahead; pass 2 is the new side coverage.
//
// UPDATE 4 (regression found, third real test): both attempts of the
// pass1+pass2 version above lost balance on pass 1's very FIRST step
// (pitch=-25, roll=0 -- roll was never even commanded away from level) --
// IMU read an uncommanded roll around -17 degrees both times. Root cause:
// UPDATE 2 raised a single shared height constant (was NODSCAN_CROUCH_HEIGHT)
// from 0.0 to 1.0 to help the roll sweep, but pass 1 was ALSO using that
// same constant for its baseline stance -- so the stability margin the
// original 0.0 height was specifically chosen for (see the comment that used
// to be on that define) got removed from pass 1 too, right when its biggest,
// fastest lean needed it most. Fix: split into two constants --
// NODSCAN_PITCH_FAN_HEIGHT (0.0, restored) for pass 1 and every
// neutral/return-to-level position, NODSCAN_HEIGHT_AT_MAX_TILT (2.5,
// unchanged) only for pass 2's circle. Tuning one pass's stance should never
// again be able to quietly change the other's.
// -----------------------------------------------------------------------------
#define NODSCAN_HEIGHT_AT_MAX_TILT      2.5f  // body height used during pass 2 (constant radius,
                                               // so no need to interpolate per-step like the old grid did)
#define NODSCAN_CIRCLE_RADIUS_DEG      20.0f  // combined pitch+roll tilt magnitude for pass 2 --
                                               // less than pass 1's proven 25 deg pitch max, on
                                               // purpose, since this is the first real test of the
                                               // circular motion; push it up once this comes back clean
#define NODSCAN_CIRCLE_STEPS              16  // points around the loop (22.5 deg apart)

// Emits "NOD:<pitch_deg>,<roll_deg>;<d0>,<d1>,...,<d359>" -- same one-buffer-
// then-send-to-both-sinks pattern as the SCAN: dump in loop(), so a WiFi
// viewer sees identical data to Serial. (Old logs had just "<pitch_deg>;..."
// with no roll -- nod_pointcloud.py parses both, defaulting roll to 0 for
// the old format.)
void nodscan_emit_slice(float pitch_deg, float roll_deg) {
  static char buf[2200];
  int pos = 0;
  pos += snprintf(buf + pos, sizeof(buf) - pos, "NOD:%.1f,%.1f;", pitch_deg, roll_deg);
  for (int i = 0; i < 360 && pos < (int)sizeof(buf) - 8; i++) {
    pos += snprintf(buf + pos, sizeof(buf) - pos, "%u", gary_lidar_distance_at_angle(i));
    if (i < 359) pos += snprintf(buf + pos, sizeof(buf) - pos, ",");
  }
  pos += snprintf(buf + pos, sizeof(buf) - pos, "\n");
  Serial.print(buf);
  wifi_scan_broadcast(buf);
}

// Returns true if pitch OR roll looks like real, sustained instability
// rather than a single noisy sample -- reads twice, NODSCAN_ABORT_RECHECK_MS
// apart, and only reports trouble if BOTH readings on the SAME axis are off
// by more than NODSCAN_ABORT_ROLL_DEG from what we actually commanded Gary
// to hold at this step. Originally roll-only (pitch was assumed safe since
// the pitch-only fan had already been validated alone); generalized once
// pass 2's circular sweep started commanding meaningful pitch and roll at
// the same time.
bool nodscan_check_unstable(float commanded_pitch_deg, float commanded_roll_deg) {
  float euler_data[3];
  gary.board.get_imu_euler(euler_data);
  float roll1  = euler_data[0];
  float pitch1 = euler_data[1];
  bool roll_bad1  = fabsf(roll1  - commanded_roll_deg)  > NODSCAN_ABORT_ROLL_DEG;
  bool pitch_bad1 = fabsf(pitch1 - commanded_pitch_deg) > NODSCAN_ABORT_ROLL_DEG;
  if (!roll_bad1 && !pitch_bad1) return false;   // clean first read, skip the recheck delay

  gary_delay(NODSCAN_ABORT_RECHECK_MS);
  gary.board.get_imu_euler(euler_data);
  float roll2  = euler_data[0];
  float pitch2 = euler_data[1];
  bool roll_bad2  = roll_bad1  && fabsf(roll2  - commanded_roll_deg)  > NODSCAN_ABORT_ROLL_DEG;
  bool pitch_bad2 = pitch_bad1 && fabsf(pitch2 - commanded_pitch_deg) > NODSCAN_ABORT_ROLL_DEG;
  if (!roll_bad2 && !pitch_bad2) return false;

  Serial.printf(">>> NODSCAN: ABORTING -- roll %.1f->%.1f (commanded %.1f) pitch %.1f->%.1f (commanded %.1f), over %.1f off-target\n",
                roll1, roll2, commanded_roll_deg, pitch1, pitch2, commanded_pitch_deg, NODSCAN_ABORT_ROLL_DEG);
  return true;
}

void run_nod_scan() {
  if (!gary_lidar_is_ready()) {
    Serial.println(">>> NODSCAN: LiDAR not ready yet, skipping scan");
    say(CLIP_UNSURE);
    return;
  }

  Serial.println(">>> NODSCAN: starting sweep (pass 1: pitch fan, pass 2: circle)");
  Velocity_t nv = {0.0f, 0.0f, 0.0f};
  Vector_t   np = {0.0f, 0.0f, NODSCAN_PITCH_FAN_HEIGHT};
  Euler_t    na = {0.0f, 0.0f, 0.0f};   // {pitch, roll, yaw} -- see process_controller()'s Euler_t ca assignment

  // ---- Pass 1: pitch-only fan at roll=0 -- unchanged from the original
  // nod-scan, already validated clean across multiple real tests. Gives full
  // vertical resolution straight ahead. ----
  int pitch_steps = (int)((NODSCAN_PITCH_MAX_DEG - NODSCAN_PITCH_MIN_DEG) / NODSCAN_PITCH_STEP_DEG + 0.5f) + 1;

  for (int i = 0; i < pitch_steps; i++) {
    float pitch = NODSCAN_PITCH_MIN_DEG + i * NODSCAN_PITCH_STEP_DEG;
    na.pitch = pitch;
    na.roll = 0.0f;

    // Step 0 is the entry move from level all the way to -25 -- a full-size
    // jump in one step, same category as pass 2's step-0 entry and the old
    // roll-level waypoints. Every other step is a 5-degree increment.
    // UPDATE (three real tests): this WAS the fast NODSCAN_MOVE_MS for every
    // step including this one, and every single test aborted on exactly this
    // first move with a consistent ~16-17 degree uncommanded roll -- the
    // same signature all three times, regardless of a height change that
    // shouldn't have mattered if height were the real cause. Slowing down
    // just this first move, the same way every other "big jump" in this
    // function already gets treated, is the fix actually aimed at the thing
    // that broke.
    uint16_t move_ms   = (i == 0) ? NODSCAN_RETURN_MS        : NODSCAN_MOVE_MS;
    uint16_t settle_ms = (i == 0) ? NODSCAN_RETURN_SETTLE_MS : NODSCAN_SETTLE_MS;
    gary.move(&nv, &np, &na, move_ms);
    gary_delay(move_ms + settle_ms);

    if (nodscan_check_unstable(pitch, 0.0f)) {
      na.pitch = 0.0f;
      gary.move(&nv, &np, &na, NODSCAN_RETURN_MS);
      gary_delay(NODSCAN_RETURN_MS + NODSCAN_RETURN_SETTLE_MS);
      say(CLIP_CONCERNED);
      return;
    }

    nodscan_emit_slice(pitch, 0.0f);
  }

  // Back to level before pass 2 -- same controlled, slower move used
  // everywhere else a bigger attitude change happens in one step.
  na.pitch = 0.0f;
  na.roll = 0.0f;
  gary.move(&nv, &np, &na, NODSCAN_RETURN_MS);
  gary_delay(NODSCAN_RETURN_MS + NODSCAN_RETURN_SETTLE_MS);

  // ---- Pass 2: circular pitch/roll sweep -- see the v4 UPDATE 3 comment
  // above nodscan_emit_slice() for the reasoning. Constant combined-tilt
  // radius the whole way around, so height only needs to be set once. ----
  Serial.println(">>> NODSCAN: pass 2 -- circular pitch/roll sweep");
  np.z = NODSCAN_HEIGHT_AT_MAX_TILT;

  for (int s = 0; s < NODSCAN_CIRCLE_STEPS; s++) {
    float theta = (2.0f * PI) * ((float)s / (float)NODSCAN_CIRCLE_STEPS);
    float pitch = NODSCAN_CIRCLE_RADIUS_DEG * cosf(theta);
    float roll  = NODSCAN_CIRCLE_RADIUS_DEG * sinf(theta);
    na.pitch = pitch;
    na.roll = roll;

    // Step 0 is the entry move from level (bigger combined jump: attitude
    // AND height both change at once) -- treat it like the other controlled
    // waypoints elsewhere in this function. Every step after that is a small
    // ~22.5 degree arc move, same idea as the 5-degree pass-1 steps.
    uint16_t move_ms   = (s == 0) ? NODSCAN_RETURN_MS        : NODSCAN_MOVE_MS;
    uint16_t settle_ms = (s == 0) ? NODSCAN_RETURN_SETTLE_MS : NODSCAN_SETTLE_MS;
    gary.move(&nv, &np, &na, move_ms);
    gary_delay(move_ms + settle_ms);

    if (nodscan_check_unstable(pitch, roll)) {
      na.pitch = 0.0f;
      na.roll = 0.0f;
      np.z = NODSCAN_PITCH_FAN_HEIGHT;
      gary.move(&nv, &np, &na, NODSCAN_RETURN_MS);
      gary_delay(NODSCAN_RETURN_MS + NODSCAN_RETURN_SETTLE_MS);
      say(CLIP_CONCERNED);
      return;
    }

    nodscan_emit_slice(pitch, roll);
  }

  // Return to level -- slower and separate from the per-step move duration,
  // since a big fast attitude change here is the likely source of the false
  // tip-alarm trip seen on the first real test.
  na.pitch = 0.0f;
  na.roll = 0.0f;
  np.z = NODSCAN_PITCH_FAN_HEIGHT;
  gary.move(&nv, &np, &na, NODSCAN_RETURN_MS);
  gary_delay(NODSCAN_RETURN_MS + NODSCAN_RETURN_SETTLE_MS);

  Serial.println(">>> NODSCAN: sweep complete");
}

void transition_to(GaryState new_state) {
  if (new_state == current_state) return;

  previous_state = current_state;
  current_state = new_state;
  state_entered_ms = millis();

  // AUTOSCAN keeps CSI capture running continuously in the background (not
  // just for one 8s burst like STATE_SENSING). Make sure it's actually
  // switched off whenever we leave AUTOSCAN for ANY reason -- toggle-off,
  // tip alarm, battery alarm, startle, controller takeover -- so it doesn't
  // keep holding the WiFi CSI hook after the state exits. Centralizing this
  // here (rather than in every possible exit path) means it can't be missed.
  if (previous_state == STATE_AUTOSCAN && new_state != STATE_AUTOSCAN) {
    csi_capture_end();
  }

#if ENABLE_DEBUG_SERIAL
  Serial.print("STATE: ");
  Serial.print(state_name(previous_state));
  Serial.print(" -> ");
  Serial.println(state_name(new_state));
#endif

  on_state_enter(new_state);
}

const char* state_name(GaryState s) {
  switch (s) {
    case STATE_BOOT:      return "BOOT";
    case STATE_IDLE:      return "IDLE";
    case STATE_LISTENING: return "LISTENING";
    case STATE_EXECUTING: return "EXECUTING";
    case STATE_ROAMING:   return "ROAMING";
    case STATE_COMING:    return "COMING";
    case STATE_STARTLED:  return "STARTLED";
    case STATE_ALARMED:   return "ALARMED";
    case STATE_CONTROLLER:return "CONTROLLER";
    case STATE_SENSING:   return "SENSING";
    case STATE_NODSCAN:   return "NODSCAN";
    case STATE_AUTOSCAN:  return "AUTOSCAN";
    case STATE_EXPLORE:   return "EXPLORE";
    default:              return "?";
  }
}

void set_eyes(const uint8_t rgb[3]) {
  uint8_t copy1[3] = { rgb[0], rgb[1], rgb[2] };
  uint8_t copy2[3] = { rgb[0], rgb[1], rgb[2] };
  gary.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, copy1, copy2);
}

void set_eyes_breathing(const uint8_t rgb[3]) {
  uint8_t copy1[3] = { rgb[0], rgb[1], rgb[2] };
  uint8_t copy2[3] = { rgb[0], rgb[1], rgb[2] };
  gary.sensor.set_ultrasound_rgb(RGB_WORK_BREATHING_MODE, copy1, copy2);
}

// -----------------------------------------------------------------------------
// say(clip_id) — non-blocking. Triggers a buzzer track and returns instantly.
// -----------------------------------------------------------------------------
void say(uint8_t clip_id) {
  switch (clip_id) {
    case CLIP_LOOK:                gary_chirp_happy();        break;
    case CLIP_SNAPPY:              gary_chirp_snappy();       break;
    case CLIP_UNSURE:              gary_unsure_descend();     break;
    case CLIP_PROCESSING:          gary_processing();         break;
    case CLIP_CHATTER:             gary_chatter_playful();    break;
    case CLIP_WHISTLING_EXCITED:   gary_whistle_excited();    break;
    case CLIP_PROUD:               gary_proud_fanfare();      break;
    case CLIP_SCREAM:              gary_scream();             break;
    case CLIP_SAD:                 gary_sad_descend();        break;
    case CLIP_WHISTLE:             gary_whistle();            break;
    case CLIP_PLAYFUL:             gary_chatter_playful();    break;
    case CLIP_VERY_EXCITED:        gary_whistle_excited();    break;
    case CLIP_LAUGHTER:            gary_laughter();           break;
    case CLIP_CONCERNED:           gary_concerned_low();      break;
    case CLIP_SHOCKED:             gary_shocked_blast();      break;
    case CLIP_DETERMINED:          gary_determined_growl();   break;
    case CLIP_WHISTLE_INQUISITIVE: gary_whistle_curious();    break;
    default: break;
  }
}


// -----------------------------------------------------------------------------
// Custom dance/show routines — replace gary.acting_cute() which made painful
// held buzzer tones during built-in routines. These use gary.move() directly,
// which is silent during motion, so we have full control of the audio.
// -----------------------------------------------------------------------------

void gary_dance_routine() {
  Velocity_t vel = {0.0f, 0.0f, 0.0f};
  Velocity_t zero = {0.0f, 0.0f, 0.0f};
  Vector_t pos = {0.0f, 0.0f, 1.5f};
  Euler_t att = {0.0f, 0.0f, 0.0f};

  // Beat 1: bouncy left-right rolls — Imperial cog rotating
  say(CLIP_LAUGHTER);
  anim_diagnostic();
  for (int i = 0; i < 2; i++) {
    att = {20.0f, 5.0f, 0.0f};
    gary.move(&zero, &pos, &att, 250);
    gary_delay(280);
    att = {-20.0f, -5.0f, 0.0f};
    gary.move(&zero, &pos, &att, 250);
    gary_delay(260);
  }
  gary_delay(50);  // extra time so cog animation is visible (no cog visible)

  // Beat 2: spin in place — Rebel starbird flapping (no its not)
  say(CLIP_VERY_EXCITED);
  anim_pulse_squares();
  vel = {0.0f, 0.0f, 10.0f};
  gary.move(&vel, &pos, &att, 250);
  gary_delay(245);
    vel = {0.0f, 0.0f, -10.0f};
  gary.move(&vel, &pos, &att, 250);
  gary_delay(245);
  gary_delay(50);  // extra time so starbird flap is visible

  // Beat 3: pitch + roll combo + yaw — control panel chaos
  say(CLIP_PLAYFUL);
  anim_control_panel();
  att = {20.0f, 10.0f, -10.0f};
  gary.move(&zero, &pos, &att, 250,3);
  gary_delay(280);
  att = {-20.0f, -10.0f, 10.0f};
  gary.move(&zero, &pos, &att, 325,3);
    anim_control_panel();
  att = {20.0f, 10.0f, -10.0f};
  gary.move(&zero, &pos, &att, 325,3);
  gary_delay(280);
  att = {-20.0f, -10.0f, 10.0f};
  gary.move(&zero, &pos, &att, 250,3);
  gary_delay(280);
  gary_delay(50);  // extra time so control panel is visible

  // Beat 4: BIG finish — hyperspace jump + scream
  say(CLIP_SCREAM);
  set_eyes(EYE_SABER_PURPLE);    // mace windu purple for finale flair
  anim_hyperspace();
  pos = {0.0f, 0.0f, 10.0f};         // stand tall
  att = {0.0f, 0.0f, 20.0f};
  gary.move(&zero, &pos, &att, 150);
  gary_delay(180);
  pos = {0.0f, 0.0f, 0.2f};         // crouch low
  gary.move(&zero, &pos, &att, 150);
  gary_delay(180);
  say(CLIP_VERY_EXCITED);
    pos = {0.0f, 0.0f, 10.0f};         // stand tall
  att = {0.0f, 0.0f, -20.0f};
  gary.move(&zero, &pos, &att, 200);
  gary_delay(210);
  say(CLIP_SCREAM);
    pos = {0.0f, 0.0f, 0.2f};         // crouch low
  gary.move(&zero, &pos, &att, 500);
  gary_delay(800);
  pos = {0.0f, 0.0f, 1.5f};         // back to neutral
  gary.move(&zero, &pos, &att, 1500);
  gary_delay(1800);
}

void gary_introduce_routine() {
  Velocity_t zero = {0.0f, 0.0f, 0.0f};
  Vector_t pos = {0.0f, 0.0f, 1.5f};
  Euler_t att = {0.0f, 0.0f, 0.0f};

  say(CLIP_PROUD);
  anim_imperial_cog();          // 
  // Stand tall
  pos = {0.0f, 0.0f, 20.0f};
  gary.move(&zero, &pos, &att, 100);
  gary_delay(150);
  // Slight bow
  att = {20.0f, 15.0f, -15.0f};
  pos = {0.0f, -10.0f, 0.5f};
  gary.move(&zero, &pos, &att, 1500);
  gary_delay(1550);
  // Rise back up
  att = {0.0f, 0.0f, 0.0f};
  pos = {0.0f, 0.0f, 1.7f};
  gary.move(&zero, &pos, &att, 1000);
  gary_delay(1000);
}

void gary_show_skill_routine() {
  Velocity_t zero = {0.0f, 0.0f, 0.0f};
  Vector_t pos = {0.0f, 0.0f, 1.5f};
  Euler_t att = {0.0f, 0.0f, 0.0f};

  say(CLIP_VERY_EXCITED);
  anim_hyperspace();
  // Quick rolls
  for (int i = 0; i < 3; i++) {
    att = {40.0f, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, 20.0f};
    gary.move(&zero, &pos, &att, 200);
    gary_delay(220);
    att = {-40.0f, 0.0f, 0.0f};
    pos = {0.0f, 0.0f, -20.0f};
    gary.move(&zero, &pos, &att, 200);
    gary_delay(220);
  }
  // Triumphant pose
  say(CLIP_PROUD);
  att = {0.0f, -20.0f, 0.0f};
  pos = {0.0f, 0.0f, 5.0f};
  gary.move(&zero, &pos, &att, 500);
  gary_delay(600);
    say(CLIP_PROUD);
  att = {0.0f, 20.0f, 0.0f};
  pos = {0.0f, 0.0f, 5.0f};
  gary.move(&zero, &pos, &att, 500);
  gary_delay(600);
  // Reset
  att = {0.0f, 0.0f, 0.0f};
  pos = {0.0f, 0.0f, 1.5f};
  gary.move(&zero, &pos, &att, 400);
  gary_delay(500);
}


// ============================================================================
// PS5 DualSense controller support via Bluepad32
//
// Tuning is based on Hiwonder's official remote.ino reference:
//   - Walking: call gary.move(&vel, &pos, &att) with NO duration argument
//     — the gait planner handles its own step phasing internally.
//   - Stationary pose-only: gary.move(&vel, &pos, &att, 200) for smooth blend.
//   - vel.vx / vel.vy: ±3.0 m/s
//   - vel.omega (yaw rate): ±2.5 rad/s stationary, ±0.2 while walking
//   - att.roll / att.pitch: degrees, scaled with body height
//
// CONVENTIONS:
//   Euler_t att = {pitch_deg, roll_deg, yaw_deg}
//   Vector_t pos = {x_cm, y_cm, z}
//   Velocity_t vel = {vx, vy, vz}
// ============================================================================

GamepadPtr myGamepad = nullptr;
bool controller_was_connected = false;
bool controller_muted = false;
bool ctrl_pu_mode = false;   // false = drive Gary, true = relay to PU
int pending_pu_button_cmd = 0;
uint32_t controller_disconnect_debounce_ms = 0;
const uint32_t CONTROLLER_DISCONNECT_HOLD_MS = 500;
uint32_t last_ctrl_status_ms = 0;
uint32_t last_pu_send_ms = 0;
const uint32_t PU_SEND_MIN_INTERVAL_MS = 60; // minimum ms between PU drive sends (was 250 -- now the lower-impact of the two throttles; see PU_MIN_SEND_INTERVAL_MS in pu_ble_client.cpp for the one that actually gates BLE write rate)
const float CTRL_TILT_INPUT_SCALE = 15000.0f;   // PS5 accel raw scale to normalized [-1,1]
const float CTRL_TILT_DEADZONE = 0.06f;

// ----------------------------------------------------------------------------
// MOVEMENT Tuning — values mirrored from Hiwonder remote.ino
// ----------------------------------------------------------------------------
const float CTRL_VX_MAX           = 5.0f;   // forward/back & strafe ceiling
const float CTRL_VY_MAX           = 7.0f;
const float CTRL_OMEGA_STATIONARY = 4.0f;   // yaw rate when not walking
const float CTRL_OMEGA_WALKING    = 0.3f;   // Hiwonder docs: "too high causes spin-in-place"   // tiny yaw blend while walking
const float CTRL_R_ROLL_MAX_DEG   = 30.0f;  // tuned to mid pos.z
const float CTRL_R_PITCH_MAX_DEG  = 15.0f;
const float CTRL_R1_LEAN_MAX_DEG  = 20.0f;
const float CTRL_R1_SHIFT_MAX_CM  = 5.0f;
const float CTRL_HEIGHT_BASE      = 1.5f;
const float CTRL_HEIGHT_RANGE     = 1.5f;   // 0..3 (Hiwonder's pos.z range)
const float CTRL_STICK_EXPONENT   = 1.3f;

// Cadence-from-stick: small deflection -> long step period (slow saunter),
// full deflection -> 400ms (library's floor when MOVING). Mapped linearly.
// Lib floor confirmed at line 624 of hiwonder_robot.cpp:
//   move_time = time > 400 ? time : 400;
const uint16_t CTRL_CADENCE_MIN_MS = 400;   // fast (library floor)
const uint16_t CTRL_CADENCE_MAX_MS = 1200;  // slow shuffle

// D-pad up/down hold-to-nudge body height. While held, height adjusts every
// HEIGHT_NUDGE_INTERVAL_MS by HEIGHT_NUDGE_STEP cm. Holding D-pad UP for ~2.5s
// sweeps from -3 to +3 (CLAMP_HEIGHT_MIN..MAX below).
const uint32_t HEIGHT_NUDGE_INTERVAL_MS = 50;
const float    HEIGHT_NUDGE_STEP        = 0.30f;

// Per-axis deadbands
const int CTRL_DEADBAND_LX = 30;
const int CTRL_DEADBAND_LY = 30;
const int CTRL_DEADBAND_RX = 40;
const int CTRL_DEADBAND_RY = 40;

// Hard safety clamps (final outputs to gary.move())
const float CLAMP_PITCH_DEG = 35.0f;   // was 30 -- raised for nod-scan headroom;
                                        // paired with the IMU_TIP_THRESHOLD_DEG bump
                                        // in gary_config.h so intentional tilt can't
                                        // trip the tip alarm at max pitch.
const float CLAMP_ROLL_DEG  = 30.0f;
const float CLAMP_HEIGHT_MIN = -3.0f;   // almost flat (library clamps at -4 internally)
const float CLAMP_HEIGHT_MAX =  3.0f;   // tall stance
const float CLAMP_BODY_Y_CM  = 3.0f;
const float CLAMP_VX = 7.0f;
const float CLAMP_VY = 7.0f;
const float CLAMP_OMEGA = 5.0f;

// Continuous targets
float ctrl_target_vy = 0.0f;       // forward/back
float ctrl_target_omega = 0.0f;    // yaw rate
float ctrl_target_vx = 0.0f;       // strafe
float ctrl_target_pitch = 0.0f;    // degrees
float ctrl_target_roll  = 0.0f;
float ctrl_target_height = CTRL_HEIGHT_BASE;
float ctrl_target_body_y = 0.0f;



int8_t ctrl_strafe_toggle_dir = 0;
uint32_t dpad_left_last_tap_ms = 0;
uint32_t dpad_right_last_tap_ms = 0;
uint32_t dpad_height_last_nudge_ms = 0;

struct ButtonEdge {
  bool cross, circle, triangle, square;
  bool l1, r1, l2, r2, l3, r3;
  bool select_b, start_b, ps_b, mute_b;
  bool tp_click;
  uint8_t dpad;
} prev_btn = {0};

uint8_t ctrl_gait_speed_idx = 1;
const float CTRL_GAIT_SPEEDS[3] = {0.15f, 1.0f, 2.0f};   // SLOW=sneak, NORMAL, FAST
const char* CTRL_SPEED_NAMES[3] = {"SLOW", "NORMAL", "FAST"};

bool ctrl_combat_mode = false;
bool ctrl_flashlight_on = false;

// -----------------------------------------------------------------------------
// SCANNER MODE
// -----------------------------------------------------------------------------
// The 360-point "SCAN:" dump is ~1.8KB per frame. Even at 921600 baud that's
// ~20ms of TX per frame, and it used to run unconditionally at 115200 (~150ms
// of blocking TX every 400ms) -- which starved the rest of loop(): the ASR was
// only getting polled ~1.6x/sec in controller mode, so spoken commands were
// routinely dropped on the floor.
//
// Scanner mode makes that traffic opt-in. OFF (default) = Gary is quiet on the
// wire and the loop runs fast, so voice/joystick stay responsive. ON = the full
// SCAN: + LiDAR-diag firehose for gary_lidar_map.py / gary_live_dashboard.py.
//
// Toggle: touchpad click on the DualSense (when NOT in PU relay mode), or send
// 's' over serial. Note the mapping tools will sit there seeing nothing until
// you switch it on -- that's expected, not a bug.
#define GARY_SERIAL_BAUD       921600
#define SCANNER_MODE_DEFAULT   0        // 1 = boot with the scan stream already on
#define SCAN_DUMP_INTERVAL_MS  200      // was 400; safe to halve at 921600

bool scanner_mode = SCANNER_MODE_DEFAULT;

void set_scanner_mode(bool on) {
  scanner_mode = on;
  Serial.printf("[SCAN] Scanner mode: %s\n", scanner_mode ? "ON (streaming SCAN: frames)" : "OFF (quiet -- voice/joystick priority)");
}

void toggle_scanner_mode() { set_scanner_mode(!scanner_mode); }

// Single-char console control, checked once per loop.
//   's' = toggle scanner mode   '1' = force ON   '0' = force OFF
//
// Read from BOTH the USB serial link AND the WiFi scan client. The WiFi half
// matters: wifi_pump() used to only ever accept a client and never read from
// it, so a viewer connected over tcp:<ip>:3333 had no way to turn scanner mode
// on -- it would just sit there receiving nothing forever. The Python tools now
// send '1' on connect (and '0' on exit), which lands here.
// ---- VOICE BISECT SWITCHES ---------------------------------------------------
// "Mister Gary" used to wake him and now doesn't. The boot banner listing other
// wake words is OUR OWN TEXT, not the module talking, so it proves nothing -- the
// user's memory that it worked is better evidence than a comment we typed.
//
// The wake word itself lives inside the CI1302 and we cannot change it. But we CAN
// change how hostile an environment it is trying to listen in, and two things I
// added this session made it hostile:
//
//   'i' -- the IMU timer. 100 I2C transactions/sec that did not exist before at
//          all (xTimerStart was commented out in the vendor lib; only balance mode
//          ever started it). Shares a bus with the ASR.
//   'd' -- the loop delay. Was 50ms, now 1ms. The ESP32 went from ~77% idle to
//          essentially pegged: more current draw, more switching noise, hotter
//          regulator. A wake word sitting near its confidence threshold does not
//          need much to be pushed under it.
//
// Toggle them live instead of reflashing twice. Say the wake word after each.
static bool  imu_enabled  = true;
static uint16_t loop_delay_ms = LOOP_DELAY_MS;

void handle_console_char(int c) {
  // '#<decimal><any non-digit>' injects a voice-command id from the serial
  // console (bench 2026-08-24: lets the tether run his whole command set
  // without shouting). Example: #1; -> CMD_FORWARD.
  static bool cmd_entry = false;
  static uint16_t cmd_val = 0;
  if (cmd_entry) {
    if (c >= '0' && c <= '9') { cmd_val = cmd_val * 10 + (c - '0'); return; }
    cmd_entry = false;
    Serial.printf(">>> CONSOLE CMD: %u\n", cmd_val);
    handle_voice_command((uint8_t)cmd_val);
    return;
  }
  switch (c) {
    case '#': cmd_entry = true; cmd_val = 0; break;
    case 's': case 'S': toggle_scanner_mode(); break;
    case '1': set_scanner_mode(true);  break;
    case '0': set_scanner_mode(false); break;

    case 'i': case 'I':
      imu_enabled = !imu_enabled;
      gary.board.imu_update(imu_enabled);
      Serial.printf(">>> IMU timer: %s  (heading/tilt %s)\n",
                    imu_enabled ? "ON" : "OFF",
                    imu_enabled ? "live" : "DEAD -- map will be wrong, this is a TEST");
      break;

    case 'e': case 'E':
      if (current_state == STATE_EXPLORE) {
        Serial.println(">>> EXPLORE: off (console)");
        transition_to(STATE_IDLE);
      } else {
        explore_begin();
        transition_to(STATE_EXPLORE);
      }
      break;
    case 'd': case 'D':
      // cycle 1 -> 10 -> 50 -> 1 ms
      loop_delay_ms = (loop_delay_ms == 1) ? 10 : (loop_delay_ms == 10 ? 50 : 1);
      Serial.printf(">>> LOOP_DELAY: %ums  (loop will run ~%s)\n", loop_delay_ms,
                    loop_delay_ms == 1 ? "250Hz, CPU pegged"
                  : loop_delay_ms == 10 ? "80Hz, CPU mostly idle"
                                        : "15Hz, like it was BEFORE this session");
      break;

    case '?':
      Serial.println(">>> KEYS: s=scanner  i=IMU timer on/off  d=loop delay 1/10/50ms");
      break;

    default: break;  // ignore stray newlines / other chars
  }
}

void poll_console() {
  while (Serial.available() > 0) handle_console_char(Serial.read());
  while (wifi_scan_client && wifi_scan_client.connected() && wifi_scan_client.available() > 0) {
    handle_console_char(wifi_scan_client.read());
  }
}

uint32_t ps_hold_start_ms = 0;
const uint16_t PS_HOLD_DISCONNECT_MS = 2000;

uint8_t ctrl_action_idx = 0;
const uint8_t CTRL_ACTION_COUNT = 7;
const uint8_t CTRL_ACTION_CMDS[CTRL_ACTION_COUNT] = {
  CMD_ACTION_12, CMD_ACTION_4, CMD_ACTION_5,
  CMD_SHAKE_HEAD, CMD_MARCH_PLACE, CMD_MARCH, CMD_ACTION_3
};

const uint16_t DOUBLE_TAP_WINDOW_MS = 400;
uint32_t triangle_last_press_ms = 0;
uint32_t square_last_press_ms = 0;
uint32_t circle_last_press_ms = 0;

const float CTRL_SMOOTHING = 0.40f;
float ctrl_smooth_vy = 0.0f;
float ctrl_smooth_omega = 0.0f;
float ctrl_smooth_vx = 0.0f;
float ctrl_smooth_pitch = 0.0f;
float ctrl_smooth_roll = 0.0f;
float ctrl_smooth_height = CTRL_HEIGHT_BASE;
float ctrl_smooth_body_y = 0.0f;

uint32_t last_debug_ms = 0;
const uint16_t DEBUG_INTERVAL_MS = 250;

// ----------------------------------------------------------------------------
// Bluepad32 callbacks
// ----------------------------------------------------------------------------

void onConnectedGamepad(GamepadPtr gp) {
  Serial.println("[CTRL] Controller connected");
  myGamepad = gp;
}

void onDisconnectedGamepad(GamepadPtr gp) {
  Serial.println("[CTRL] Controller disconnected");
  if (myGamepad == gp) myGamepad = nullptr;
}

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------

inline int apply_deadband(int v, int dz) {
  if (v >  dz) return v - dz;
  if (v < -dz) return v + dz;
  return 0;
}

inline float stick_raw(int v, int dz) {
  v = apply_deadband(v, dz);
  if (v == 0) return 0.0f;
  float f = (float)v / (512.0f - dz);
  if (f >  1.0f) f =  1.0f;
  if (f < -1.0f) f = -1.0f;
  return f;
}

inline float curve(float f) {
  float sign = (f < 0) ? -1.0f : 1.0f;
  return sign * powf(fabsf(f), CTRL_STICK_EXPONENT);
}

inline float clamp_f(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

void cycle_gait_speed() {
  ctrl_gait_speed_idx = (ctrl_gait_speed_idx + 1) % 3;
  Serial.printf("[CTRL] Speed: %s (%.2fx)\n",
                CTRL_SPEED_NAMES[ctrl_gait_speed_idx],
                CTRL_GAIT_SPEEDS[ctrl_gait_speed_idx]);
  say(CLIP_WHISTLE);
}

// ---------------------------------------------------------------------------
// GAIT PATTERN — TRIPOD / WAVE / RIPPLE
// Cycled by R3 click. Only TRIPOD is implemented today (uses gary.move() and
// the library's hardcoded tripod scheduler). WAVE and RIPPLE are stubs that
// log a warning and fall through to TRIPOD until we wire up our own per-leg
// phase scheduler in the next pass.
// ---------------------------------------------------------------------------
enum GaitPattern { GAIT_TRIPOD = 0, GAIT_WAVE = 1, GAIT_RIPPLE = 2 };
GaitPattern ctrl_gait_pattern = GAIT_TRIPOD;

const char* gait_pattern_name(int g) {
  switch (g) {
    case GAIT_TRIPOD: return "TRIPOD";
    case GAIT_WAVE:   return "WAVE";
    case GAIT_RIPPLE: return "RIPPLE";
    default:          return "?";
  }
}

void cycle_gait_pattern() {
  ctrl_gait_pattern = (GaitPattern)(((int)ctrl_gait_pattern + 1) % 3);
  Serial.printf("[CTRL] Gait pattern: %s\n", gait_pattern_name((int)ctrl_gait_pattern));
  switch (ctrl_gait_pattern) {
    case GAIT_TRIPOD: gary_chirp_snappy();    break;
    case GAIT_WAVE:   gary_whistle_curious(); break;
    case GAIT_RIPPLE: gary_chatter_playful(); break;
  }
}

// ---------------------------------------------------------------------------
// CUSTOM GAIT SCHEDULER — wave + ripple (bypasses the library's tripod)
//
// Mechanism:
//   1. Call gary.move(&v, &p, &a, t) — library populates each leg's
//      .amplitude (.x/.y) based on the commanded velocity.
//   2. Set gary.func_state = ACTION_GROUP — disables the library's internal
//      Robot::update() trajectory math (its switch on func_state hits the
//      empty ACTION_GROUP case and breaks out).
//   3. Compute our own per-leg target each tick using phase offsets and a
//      stance/swing window matched to the gait, then call legN.move(p, 0)
//      directly. The library's per-leg ikine() and servo dispatch still run.
//
// Trajectory shape (per leg):
//   STANCE: foot moves linearly backward through 2*amp over β cycle.
//           All grounded legs move at the same body-frame velocity, so the
//           body translates without skidding.
//   SWING:  foot moves linearly forward through 2*amp over (1-β) cycle, with
//           a half-sine z-arc for ground clearance.
//
//   Stride scale = 2β so body forward velocity matches across all gaits at
//   the same commanded vy. (Library's amp assumes β=0.5; we compensate.)
//
// Phase offsets:
//   TRIPOD: legs {1,3,5}=0, {2,4,6}=½  (matches library; we don't actually
//           use this — TRIPOD path stays on the library — but it's here for
//           reference and possible future unification)
//   WAVE:   metachronal, 1 leg in swing at a time. Order: 3→4→2→5→1→6
//           (back→back, mid→mid, front→front, alternating sides).
//   RIPPLE: 3 pairs of opposite legs, each pair offset by ⅓.
//           Pair (1,4)=0, (2,5)=⅓, (3,6)=⅔.
// ---------------------------------------------------------------------------

// Indexed [pattern][leg_idx-1]. Phase offset is fraction of cycle.
const float GAIT_PHASE_OFFSETS[3][6] = {
  {0.0f,    0.5f,    0.0f,    0.5f,    0.0f,    0.5f   },  // TRIPOD
  {4.0f/6,  2.0f/6,  0.0f/6,  1.0f/6,  3.0f/6,  5.0f/6 },  // WAVE (3,4,2,5,1,6)
  {0.0f/3,  1.0f/3,  2.0f/3,  0.0f/3,  1.0f/3,  2.0f/3 },  // RIPPLE
};

const float GAIT_STANCE_FRACTION[3] = {
  0.5f,        // TRIPOD: 3 legs always grounded
  5.0f/6,      // WAVE:   5 legs always grounded
  2.0f/3,      // RIPPLE: 4 legs always grounded
};

// Per-gait limits. Wave + ripple are inherently slower than tripod and the
// servos can't sustain fast cadence with the larger leg sweeps these gaits
// require. These caps prevent the "frantic" servo strain at full stick.
//   speed_cap  = multiplier on send_vy/vx/omega before passing to gary.move()
//   omega_cap  = additional multiplier on yaw rate (yaw is hardest in slow gaits)
//   cadence_ms = minimum step period; longer = slower cycles, smoother motion
//
// Wave/ripple speed caps were halved because of a touchdown-scoot problem:
// at touchdown, foot world-velocity ≈ +V_body (carried forward by the body
// during swing). The servo can't snap to v=0 instantly; foot scoots forward,
// pushing body backward. With 5-6 touchdowns per cycle, backward kicks were
// canceling forward propulsion (or beating it — "marches in place / moves
// backward"). Slowing the gait makes the scoot small enough that propulsion
// dominates.
const float    GAIT_SPEED_CAP[3]    = {1.0f, 0.15f, 0.30f};   // tripod/wave/ripple
const float    GAIT_OMEGA_CAP[3]    = {1.0f, 0.15f, 0.25f};
const uint16_t GAIT_MIN_CADENCE[3]  = {400,  1200,  800  };   // ms

const float CUSTOM_GAIT_LIFT_CM = 2.5f;   // base foot clearance during swing (scaled by height)

// Effective minimum cadence for a gait — tuned values when GAIT_TUNED, else the
// July table. EVERY consumer (controller, odometry, telemetry) must use this so
// they agree on the cycle time.
static uint16_t gait_min_cadence(int g) {
#if GAIT_TUNED
  if (g == GAIT_WAVE)   return GAIT_TUNED_CADENCE_WAVE;
  if (g == GAIT_RIPPLE) return GAIT_TUNED_CADENCE_RIP;
#endif
  return GAIT_MIN_CADENCE[g];
}

// BODY YAW -- the missing sixth degree of freedom (sim, 2026-08-22).
// He could pose pitch, roll, and shift dx/dy, but never YAW. Without it
// a "head shake" can only be a stepping turn: legs shuffling one at a
// time, which Bill correctly called "just moving its legs around,
// seemingly one at a time". A real wag rotates the BODY between planted
// feet. The mechanism is the same lever the gait's rotation already
// uses -- displace each neutral foot tangentially about the body centre
// by psi * r_hip -- but as a CONSTANT pose rather than a phase-modulated
// stride. Sim measured 23 deg peak-to-peak with the feet planted (the
// uniform-coxa-offset version managed 7 deg and skated 25 mm, because a
// coxa offset cannot represent a yaw: the hips orbit the centre, so the
// leg extension has to change too).
//
// Also the vehicle for Bill's CORNERING YAW LEAD ("it might even help
// in his cornering abilities... hell yah"), which the sim's race-mode
// evolution ADOPTED on merit from a zero start.
float gary_body_yaw_rad = 0.0f;

void run_custom_gait_scheduler(int gait, uint16_t move_time_ms, float lift_cm,
                               float omega_rad_s, float fwd_cm_s) {
  // Body-frame positions of each leg's hip in cm. Used to compute the
  // per-leg tangential amplitude contribution from yaw rate (omega) — the
  // library doesn't encode this in leg.amplitude for non-tripod gaits, so we
  // add it ourselves. Values match the leg origins in hiwonder_robot.h.
  static const float LEG_POS_X[6] = {  3.15f,  4.34f,  3.15f, -3.15f, -4.34f, -3.15f };
  static const float LEG_POS_Y[6] = { -5.65f,  0.00f,  5.65f,  5.65f,  0.00f, -5.65f };

  // Scale factor on omega-derived amplitude. 1.0 = "fully translate the
  // tangential velocity into a foot stride." Lower if turning is too aggressive,
  // higher if too weak. Foot stride is clamped below to leg's kinematic limits.
  const float OMEGA_AMP_SCALE = 1.0f;
  // PURE SPINS GET THE FULL STROKE (sim-certified 2026-08-22, Bill: "he
  // really should be turning-in-place a lot faster"). The 3 cm cap is
  // walking-turn conservatism -- while translating, rotation competes
  // with the stride for the same envelope. Standing still there is no
  // competition, and the sim measured 18 -> 56 deg/s (a 180 in ~3 s
  // instead of 10) with no loss of stability and no extra knee load.
  const float OMEGA_AMP_CLAMP_CM = (fabsf(fwd_cm_s) < 0.5f) ? 5.5f : 3.0f;

  static float    cycle_t = 0.0f;          // [0, 1) phase state — survives across calls
  static uint32_t last_call_ms = 0;
  static int      last_gait = GAIT_TRIPOD;
  uint32_t now = millis();

  // Reset phase when entering wave/ripple from idle, or when switching gaits.
  // Otherwise advance cycle_t by elapsed_time / current_move_time_ms — this
  // way, changes in move_time mid-cycle (which happen for ripple as stick
  // deflection varies) advance phase smoothly instead of jumping.
  // The 500ms reset threshold (was 200ms) tolerates occasional long loops
  // (Bluetooth, sensor reads, debug prints) without snapping all legs to
  // phase=0 mid-walk.
  bool reset = (last_call_ms == 0) || ((now - last_call_ms) > 500) || (gait != last_gait);
  if (reset) {
    cycle_t = 0.0f;
    last_gait = gait;
  } else {
    if (move_time_ms < 1) move_time_ms = 1;
    float dt_phase = (float)(now - last_call_ms) / (float)move_time_ms;
    // Cap per-update phase advance. If a loop occasionally takes 150ms+,
    // foot targets would otherwise jump 12%+ of cycle in one update — that's
    // the "slam / twitch / double-step" pattern. Capping causes a tiny phase
    // lag instead of a visible servo jolt; small loops are unaffected.
    if (dt_phase > 0.06f) dt_phase = 0.06f;
    cycle_t += dt_phase;
    while (cycle_t >= 1.0f) cycle_t -= 1.0f;
  }
  last_call_ms = now;

  const float* offsets = GAIT_PHASE_OFFSETS[gait];
  float beta           = GAIT_STANCE_FRACTION[gait];
  // stride_scale = 1.0 (was 2β). The 2β factor was matching wave/ripple body
  // velocity to tripod for the same amp. Hypothesis: that scaling pushed amps
  // past the kinematic envelope for some legs, producing the "moves backward
  // / marches in place" pathology. Using amp directly = slower base speed but
  // motion stays inside the leg's reachable space.
  float stride_scale   = 1.0f;

  RobotLeg* legs[6] = {&gary.leg1, &gary.leg2, &gary.leg3,
                       &gary.leg4, &gary.leg5, &gary.leg6};

  // KNEE GUARD (2026-08-18): if a knee has been pinned at its clamp for >400ms,
  // stop feeding it new targets — holding position ends the stall instead of
  // fighting it. The guard auto-clears 2s after the clipping stops.
  if (knee_guard_tripped()) return;

  // TARGET SLEW LIMIT (2026-08-18, bench-unverified): cap the per-update foot
  // displacement so no single update can slam a servo. move(target, 0) sends
  // instantaneous targets; at 1ms loops the steps are naturally tiny, but a
  // long loop pass (BT/LiDAR/debug burst) followed by a phase jump used to
  // produce one giant step = the "slam/twitch" + stall-current spike at the
  // servo. Caps chosen well above normal per-update motion (stance foot speed
  // ~= body speed, swing ~4x): they only clip the pathological steps.
  //   dt-scaled: 40 cm/s stance-ish floor, up to per-update cap of 0.8 cm.
  static float    slew_prev[6][3];
  static bool     slew_init = false;
  static uint32_t slew_last_ms = 0;
  float slew_dt = (now > slew_last_ms && !reset) ? (now - slew_last_ms) * 0.001f : 0.001f;
  slew_last_ms = now;
  float max_step_cm = 40.0f * slew_dt;              // 40 cm/s foot-speed ceiling
  if (max_step_cm > 0.8f) max_step_cm = 0.8f;       // absolute per-update cap
  if (reset || !slew_init) {
    for (int i = 0; i < 6; i++) {
      slew_prev[i][0] = legs[i]->result.x;
      slew_prev[i][1] = legs[i]->result.y;
      slew_prev[i][2] = legs[i]->result.z;
    }
    slew_init = true;
  }

  // Per-leg rotational amplitude contribution from omega. The library doesn't
  // encode this in leg.amplitude for our use case, so we compute it. T/2 because
  // amplitude is half-stroke; sign such that positive omega rotates CCW from above.
  float T_sec = (float)move_time_ms * 0.001f;
  float rot_factor = omega_rad_s * T_sec * 0.5f * OMEGA_AMP_SCALE;

  for (int i = 0; i < 6; i++) {
    float phase_t = cycle_t - offsets[i];
    phase_t = phase_t - floorf(phase_t);   // wrap to [0, 1)

    // Compute rotational amp contribution per leg from omega.
    float rot_amp_x = -LEG_POS_Y[i] * rot_factor;
    float rot_amp_y =  LEG_POS_X[i] * rot_factor;
    // Clamp so a runaway omega doesn't send feet to unreachable positions.
    if (rot_amp_x >  OMEGA_AMP_CLAMP_CM) rot_amp_x =  OMEGA_AMP_CLAMP_CM;
    if (rot_amp_x < -OMEGA_AMP_CLAMP_CM) rot_amp_x = -OMEGA_AMP_CLAMP_CM;
    if (rot_amp_y >  OMEGA_AMP_CLAMP_CM) rot_amp_y =  OMEGA_AMP_CLAMP_CM;
    if (rot_amp_y < -OMEGA_AMP_CLAMP_CM) rot_amp_y = -OMEGA_AMP_CLAMP_CM;

    // Combined amplitude = translation (from library) + rotation (ours).
    float amp_x = legs[i]->amplitude.x + rot_amp_x;
    float amp_y = legs[i]->amplitude.y + rot_amp_y;

    Vector_t target;
    target.x = legs[i]->result.x;
    target.y = legs[i]->result.y;
    target.z = legs[i]->result.z;
    // BODY YAW POSE: tangential displacement of the NEUTRAL foot about
    // the body centre. Feet stay put on the ground, so the body turns.
    if (fabsf(gary_body_yaw_rad) > 0.001f) {
      target.x += -LEG_POS_Y[i] * gary_body_yaw_rad;
      target.y +=  LEG_POS_X[i] * gary_body_yaw_rad;
    }

    if (phase_t < beta) {
      // STANCE: linear backward sweep — kinematically required for slip-free
      // body translation when multiple phased legs are simultaneously grounded.
      // (Sinusoidal/cosine stance was tried and produces visible jitter and
      // stick-slip friction artifacts: phased stance legs with different
      // velocities over-constrain the body, servos fight each other.)
      float k = 1.0f - 2.0f * phase_t / beta;
      target.x += amp_x * k * stride_scale;
      target.y += amp_y * k * stride_scale;
      // z grounded
    } else {
      float swing_t = (phase_t - beta) / (1.0f - beta);
      float k;
#if GAIT_TUNED
      // SWING (GAIT_TUNED): velocity-matched cubic Hermite, -1 -> +1 with the
      // endpoint slope equal to the stance sweep's slope. The foot lifts off
      // and touches down already moving WITH the ground — removes the X
      // velocity discontinuity at touchdown (the "scoot" that dragged the
      // knee servo through stall current at every footfall). Sim result:
      // tripod-equivalent knee saturation 17% -> 10% vs the cosine swing.
      {
        float m  = -2.0f * (1.0f - beta) / beta;         // endpoint slope (per unit k)
        float t2 = swing_t * swing_t, t3 = t2 * swing_t;
        k = (2*t3 - 3*t2 + 1) * (-1.0f)                  // h00 * start
          + (t3 - 2*t2 + swing_t) * m                    // h10 * start slope
          + (-2*t3 + 3*t2) * (1.0f)                      // h01 * end
          + (t3 - t2) * m;                               // h11 * end slope
      }
#else
      // SWING: cosine-eased X return + sin² Z arc.
      // Cosine X: foot velocity = 0 at swing endpoints (no jerky transition
      // at liftoff; touchdown still has the X velocity discontinuity into
      // linear stance — that's the price of slip-free stance).
      k = -cosf(PI * swing_t);   // -1 → +1
#endif
      target.x += amp_x * k * stride_scale;
      target.y += amp_y * k * stride_scale;
      // sin² Z: zero z-velocity at takeoff AND touchdown — kills the stomp.
      float s = sinf(PI * swing_t);
      target.z += lift_cm * s * s;
    }

    // Apply the slew limit per axis, then remember what we actually commanded.
    // (Vector_t fields are volatile float — no pointer tricks; do it longhand.)
    float tv[3] = { (float)target.x, (float)target.y, (float)target.z };
    for (int ax = 0; ax < 3; ax++) {
      float d = tv[ax] - slew_prev[i][ax];
      if (d >  max_step_cm) tv[ax] = slew_prev[i][ax] + max_step_cm;
      if (d < -max_step_cm) tv[ax] = slew_prev[i][ax] - max_step_cm;
      slew_prev[i][ax] = tv[ax];
    }
    target.x = tv[0]; target.y = tv[1]; target.z = tv[2];

    legs[i]->move(target, 0);
  }
}

void toggle_flashlight() {
  ctrl_flashlight_on = !ctrl_flashlight_on;
  Serial.printf("[CTRL] Flashlight: %s\n", ctrl_flashlight_on ? "ON" : "OFF");
  if (ctrl_flashlight_on) {
    uint8_t white[3] = {255, 255, 255};
    set_eyes(white);
  } else {
    set_eyes(EYE_SABER_BLUE);
  }
}

void cycle_action_group(int8_t direction) {
  if (direction > 0) ctrl_action_idx = (ctrl_action_idx + 1) % CTRL_ACTION_COUNT;
  else ctrl_action_idx = (ctrl_action_idx + CTRL_ACTION_COUNT - 1) % CTRL_ACTION_COUNT;
  Serial.printf("[CTRL] Action: %d\n", ctrl_action_idx + 1);
  say(CLIP_SNAPPY);
}

void controller_execute_action(uint8_t cmd) {
  Serial.printf("[CTRL] Executing CMD %d\n", cmd);
  handle_voice_command(cmd);
  if (myGamepad && myGamepad->isConnected()) {
    transition_to(STATE_CONTROLLER);
  }
}

void controller_execute_selected_action() {
  controller_execute_action(CTRL_ACTION_CMDS[ctrl_action_idx]);
}

void set_strafe(int8_t dir, bool hold_only) {
  if (hold_only) {
    if (ctrl_strafe_toggle_dir != 0 && ctrl_strafe_toggle_dir != dir) {
      ctrl_strafe_toggle_dir = 0;
      Serial.println("[CTRL] Strafe toggle CANCELED");
    }
    return;
  }
  if (dir == 0) {
    if (ctrl_strafe_toggle_dir != 0) Serial.println("[CTRL] Strafe OFF");
    ctrl_strafe_toggle_dir = 0;
  } else if (ctrl_strafe_toggle_dir == dir) {
    ctrl_strafe_toggle_dir = 0;
    Serial.println("[CTRL] Strafe OFF (re-toggle)");
  } else {
    ctrl_strafe_toggle_dir = dir;
    Serial.printf("[CTRL] Strafe %s\n", dir < 0 ? "LEFT" : "RIGHT");
    say(CLIP_SNAPPY);
  }
}

// ----------------------------------------------------------------------------
// Process one frame
// ----------------------------------------------------------------------------

void process_controller() {
  if (!myGamepad || !myGamepad->isConnected()) {
    if (controller_was_connected) {
      if (controller_disconnect_debounce_ms == 0) {
        controller_disconnect_debounce_ms = millis();
      } else if (millis() - controller_disconnect_debounce_ms >= CONTROLLER_DISCONNECT_HOLD_MS) {
        Serial.println("[CTRL] Connection lost, returning to voice mode");
        controller_was_connected = false;
        controller_disconnect_debounce_ms = 0;
        controller_muted = false;
        ctrl_combat_mode = false;
        ctrl_flashlight_on = false;
        ctrl_strafe_toggle_dir = 0;
        ctrl_gait_pattern = GAIT_TRIPOD;
        transition_to(STATE_IDLE);
      }
    }
    return;
  }
  controller_disconnect_debounce_ms = 0;
  controller_was_connected = true;

  if (millis() - last_ctrl_status_ms >= 2000) {
    last_ctrl_status_ms = millis();
    Serial.printf("[CTRL] status: pu_mode=%d connected=%d pu_ready=%d pending_btn=%d buttons=0x%08X misc=0x%08X AX=%d AY=%d RX=%d RY=%d accX=%d accY=%d accZ=%d\n",
                  ctrl_pu_mode ? 1 : 0,
                  myGamepad->isConnected() ? 1 : 0,
                  pu_ble_is_ready() ? 1 : 0,
                  pending_pu_button_cmd,
                  myGamepad->buttons(),
                  myGamepad->miscButtons(),
                  myGamepad->axisX(),
                  myGamepad->axisY(),
                  myGamepad->axisRX(),
                  myGamepad->axisRY(),
                  myGamepad->accelX(),
                  myGamepad->accelY(),
                  myGamepad->accelZ());
  }

  bool cross    = myGamepad->a();
  bool circle   = myGamepad->b();
  bool triangle = myGamepad->y();
  bool square   = myGamepad->x();
  bool l1       = myGamepad->l1();
  bool r1       = myGamepad->r1();

  uint32_t buttons = myGamepad->buttons();
  bool l2 = (buttons & BUTTON_TRIGGER_L) != 0;
  bool r2 = (buttons & BUTTON_TRIGGER_R) != 0;
  bool l3 = (buttons & BUTTON_THUMB_L) != 0;
  bool r3 = (buttons & BUTTON_THUMB_R) != 0;

  uint32_t misc = myGamepad->miscButtons();
  bool ps_b      = (misc & MISC_BUTTON_SYSTEM)  != 0;
  bool select_b  = (misc & MISC_BUTTON_BACK)    != 0;
  bool start_b   = (misc & MISC_BUTTON_START)   != 0;
  bool tp_click  = (misc & MISC_BUTTON_CAPTURE) != 0;
  bool mute_b = false;

  uint8_t dpad = myGamepad->dpad();
  if (cross || l3) {
  Serial.printf("[DIAG] cross=%d l3=%d buttons_raw=0x%08X axisX=%d axisY=%d\n",
                cross, l3, buttons, myGamepad->axisX(), myGamepad->axisY());
}

  // PS button: tap = no-op (gyro disabled), hold ~2s = disconnect
  if (ps_b && !prev_btn.ps_b) ps_hold_start_ms = millis();
  if (!ps_b && prev_btn.ps_b) {
    uint32_t held = millis() - ps_hold_start_ms;
    if (held >= PS_HOLD_DISCONNECT_MS) {
      Serial.println("[CTRL] PS held - disconnecting");
      myGamepad->disconnect();
      ps_hold_start_ms = 0;
    }
  }

  // Mute button (currently no-op until we identify the bit)
  if (mute_b && !prev_btn.mute_b) {
    controller_muted = !controller_muted;
    Serial.printf("[CTRL] %s\n", controller_muted ? "MUTED" : "UNMUTED");
    if (controller_muted) {
      ctrl_target_vy = 0; ctrl_target_omega = 0; ctrl_target_vx = 0;
      ctrl_target_roll = 0; ctrl_target_pitch = 0;
      ctrl_target_height = CTRL_HEIGHT_BASE; ctrl_target_body_y = 0;
      ctrl_strafe_toggle_dir = 0;
      say(CLIP_SAD);
    } else say(CLIP_PLAYFUL);
  }

  if (controller_muted) {
    prev_btn.ps_b = ps_b;
    prev_btn.mute_b = mute_b;
    return;
  }
  // ---- PU relay mode toggle (Start button) ----
if (start_b && !prev_btn.start_b) {
    ctrl_pu_mode = !ctrl_pu_mode;
    Serial.printf("[CTRL] PU relay mode: %s\n", ctrl_pu_mode ? "ON (driving PU)" : "OFF (driving Gary)");
    if (ctrl_pu_mode) {
      if (!pu_ble_is_ready()) {
        Serial.println("[CTRL] PU not ready, requesting BLE connect...");
        pu_ble_request_connect();
      } else {
        Serial.println("[CTRL] PU already ready");
      }
    }
}

if (ctrl_pu_mode) {
    if (pending_pu_button_cmd != 0) {
      if (pu_send_button(pending_pu_button_cmd)) {
        Serial.printf("[CTRL] PU resend button %d OK\n", pending_pu_button_cmd);
        pending_pu_button_cmd = 0;
      }
    }

    float lx_raw = stick_raw(myGamepad->axisX(), CTRL_DEADBAND_LX);
    float ly_raw = stick_raw(myGamepad->axisY(), CTRL_DEADBAND_LY);
    static float headPitchSmooth = 0;
    static float headRollSmooth = 0;
    // accelY() only carries tilt MAGNITUDE -- it's the axis gravity sits on when the
    // controller is level, so it stays ~8000-9200 regardless of pitch direction and
    // just shrinks as you rotate away from level either way. accelZ() is the axis that
    // actually swings sign with direction (confirmed from a live log: ~-8300 pitched
    // one way, ~+7800 pitched the other, both maxing near +-9000 -- 1g -- at a full
    // ~90 degree tilt, same scale accelY was already using).
    headPitchSmooth = headPitchSmooth * 0.85f + (myGamepad->accelZ() / 20000.0f) * 0.15f;
    headRollSmooth  = headRollSmooth  * 0.85f + (myGamepad->accelX() / 20000.0f) * 0.15f;

    // Driving via left stick only; gyro/accel are used for head-look only.
    float drive_turn = curve(lx_raw);
    float drive_speed = -curve(ly_raw);

    // Throttle drive sends to PU to avoid overwhelming the micro:bit GATT handler.
    uint32_t now = millis();
    bool drive_ok = true;
    if ((int32_t)(now - last_pu_send_ms) >= (int32_t)PU_SEND_MIN_INTERVAL_MS) {
      // Bill 2026-09-17 ("no driving PU"): the combined-axis packet
      // ("A<t>,<s>,<p>,<r>\n") is 21-24 bytes, but a BLE write-without-response
      // at PU's default 23-byte ATT MTU can only carry 20. Serial proved it:
      // EVERY send returned status=151 (GATT_CLIENT_VALUE_TOO_LONG), 955/955
      // errors, so PU received nothing -- no driving, and it idled itself to
      // sleep (its wake() only fires on the Speed/Turn handlers, which never
      // arrived). Back to the per-axis rotation: each T/S/P/R packet is 5-7
      // bytes (well under 20), PU parses them natively, and S/T wake it.
      drive_ok = pu_send_all_axes(drive_turn, drive_speed, headPitchSmooth, headRollSmooth);
      if (drive_ok) last_pu_send_ms = now;
    } else {
      // Skipped send due to throttle; consider this successful to avoid reconnect spam.
      drive_ok = true;
    }
    if (!drive_ok && !pu_ble_is_ready()) {
      Serial.println("[CTRL] PU drive send failed: reconnecting");
      pu_ble_request_connect();
    }

    if (cross    && !prev_btn.cross) {
      if (!pu_send_button(1)) {
        Serial.println("[CTRL] PU button Greet failed, queued");
        pending_pu_button_cmd = 1;
      }
    }
    if (triangle && !prev_btn.triangle) {
      if (!pu_send_button(2)) {
        Serial.println("[CTRL] PU button Dance failed, queued");
        pending_pu_button_cmd = 2;
      }
    }
    if (square   && !prev_btn.square) {
      if (!pu_send_button(3)) {
        Serial.println("[CTRL] PU button Explore failed, queued");
        pending_pu_button_cmd = 3;
      }
    }
    if (circle   && !prev_btn.circle) {
      if (!pu_send_button(4)) {
        Serial.println("[CTRL] PU button Rest failed, queued");
        pending_pu_button_cmd = 4;
      }
    }
    if (tp_click && !prev_btn.tp_click) {
      if (!pu_send_button(5)) {
        Serial.println("[CTRL] PU button Stop failed, queued");
        pending_pu_button_cmd = 5;
      }
    }

    prev_btn.cross = cross; prev_btn.circle = circle;
    prev_btn.triangle = triangle; prev_btn.square = square;
    prev_btn.l1 = l1; prev_btn.r1 = r1;
    prev_btn.l2 = l2; prev_btn.r2 = r2;
    prev_btn.l3 = l3; prev_btn.r3 = r3;
    prev_btn.select_b = select_b; prev_btn.start_b = start_b;
    prev_btn.ps_b = ps_b; prev_btn.mute_b = mute_b;
    prev_btn.tp_click = tp_click;
    prev_btn.dpad = dpad;
    return;
}

  if (l3 && !prev_btn.l3) cycle_gait_speed();
  if (r3 && !prev_btn.r3) cycle_gait_pattern();

  if (l2 && !prev_btn.l2) { controller_execute_action(CMD_LEFT_HOOK); return; }
  if (r2 && !prev_btn.r2) { controller_execute_action(CMD_RIGHT_HOOK); return; }

  if (l1 && r1 && !(prev_btn.l1 && prev_btn.r1)) {
    controller_execute_action(CMD_ACTION_6);
    return;
  }

  if (select_b && !prev_btn.select_b) toggle_flashlight();

  // Touchpad click = scanner mode toggle (Gary-driving mode only; in PU relay
  // mode the touchpad is already bound to the PU "Stop" button above).
  if (tp_click && !prev_btn.tp_click) toggle_scanner_mode();

  if (cross && !prev_btn.cross) {
    controller_execute_selected_action();
    return;
  }

  if (square && !prev_btn.square) {
    uint32_t now = millis();
    if (now - square_last_press_ms < DOUBLE_TAP_WINDOW_MS) {
      controller_execute_action(CMD_ACTION_8);
      square_last_press_ms = 0;
      return;
    } else square_last_press_ms = now;
  }
  if (square_last_press_ms != 0 && (millis() - square_last_press_ms) >= DOUBLE_TAP_WINDOW_MS) {
    square_last_press_ms = 0;
    controller_execute_action(CMD_DIVE_FWD);
    return;
  }

  if (circle && !prev_btn.circle) {
    uint32_t now = millis();
    if (now - circle_last_press_ms < DOUBLE_TAP_WINDOW_MS) {
      controller_execute_action(CMD_SPREAD);
      circle_last_press_ms = 0;
      return;
    } else circle_last_press_ms = now;
  }
  if (circle_last_press_ms != 0 && (millis() - circle_last_press_ms) >= DOUBLE_TAP_WINDOW_MS) {
    circle_last_press_ms = 0;
    controller_execute_action(CMD_DIVE_BACK);
    return;
  }

  if (triangle && !prev_btn.triangle) {
    uint32_t now = millis();
    if (now - triangle_last_press_ms < DOUBLE_TAP_WINDOW_MS) {
      controller_execute_action(CMD_DANCE);
      triangle_last_press_ms = 0;
      return;
    } else triangle_last_press_ms = now;
  }
  if (triangle_last_press_ms != 0 && (millis() - triangle_last_press_ms) >= DOUBLE_TAP_WINDOW_MS) {
    triangle_last_press_ms = 0;
    controller_execute_action(CMD_INTRODUCE);
    return;
  }

  bool dpad_up_now    = (dpad & DPAD_UP)    && !(prev_btn.dpad & DPAD_UP);
  bool dpad_down_now  = (dpad & DPAD_DOWN)  && !(prev_btn.dpad & DPAD_DOWN);
  bool dpad_left_now  = (dpad & DPAD_LEFT)  && !(prev_btn.dpad & DPAD_LEFT);
  bool dpad_right_now = (dpad & DPAD_RIGHT) && !(prev_btn.dpad & DPAD_RIGHT);

  // D-pad UP/DOWN: continuous body-height nudging while held.
  // (Old behavior — discrete STEP_OVER/SQUEEZE toggle — replaced.)
  // Hold duration: ~80ms per 0.25cm step => full -3..+3 sweep in ~2.5s.
  if ((dpad & DPAD_UP) || (dpad & DPAD_DOWN)) {
    if (millis() - dpad_height_last_nudge_ms >= HEIGHT_NUDGE_INTERVAL_MS) {
      dpad_height_last_nudge_ms = millis();
      if (dpad & DPAD_UP)   ctrl_target_height += HEIGHT_NUDGE_STEP;
      if (dpad & DPAD_DOWN) ctrl_target_height -= HEIGHT_NUDGE_STEP;
      ctrl_target_height = clamp_f(ctrl_target_height, CLAMP_HEIGHT_MIN, CLAMP_HEIGHT_MAX);
    }
  }

  if (dpad_left_now) {
    if (l1) {
      uint32_t now = millis();
      if (now - dpad_left_last_tap_ms < DOUBLE_TAP_WINDOW_MS) {
        set_strafe(-1, false);
        dpad_left_last_tap_ms = 0;
      } else dpad_left_last_tap_ms = now;
    } else cycle_action_group(-1);
  }
  if (dpad_right_now) {
    if (l1) {
      uint32_t now = millis();
      if (now - dpad_right_last_tap_ms < DOUBLE_TAP_WINDOW_MS) {
        set_strafe(+1, false);
        dpad_right_last_tap_ms = 0;
      } else dpad_right_last_tap_ms = now;
    } else cycle_action_group(+1);
  }

  // ---- Stick reads ----
  float lx_raw = stick_raw(myGamepad->axisX(),  CTRL_DEADBAND_LX);
  float ly_raw = stick_raw(myGamepad->axisY(),  CTRL_DEADBAND_LY);
  float rx_raw = stick_raw(myGamepad->axisRX(), CTRL_DEADBAND_RX);
  float ry_raw = stick_raw(myGamepad->axisRY(), CTRL_DEADBAND_RY);

  float lx_curved = curve(lx_raw);
  float ly_curved = curve(ly_raw);

  float speed_cap = CTRL_GAIT_SPEEDS[ctrl_gait_speed_idx];

  // L stick walking — using Hiwonder ranges
  ctrl_target_vy = -ly_curved * CTRL_VY_MAX * speed_cap;

  // Yaw rate — smoothly blend stationary (full omega) → walking (small omega)
  // based on STICK input (not smoothed vy) so releasing forward gives instant
  // turn-in-place response without waiting for vy to decay.
  float walking_amount = fabsf(ly_curved);  // 0 when stick X-only or centered
  if (walking_amount > 1.0f) walking_amount = 1.0f;
  float omega_max = CTRL_OMEGA_STATIONARY
                  + (CTRL_OMEGA_WALKING - CTRL_OMEGA_STATIONARY) * walking_amount;
  ctrl_target_omega = -lx_curved * omega_max * speed_cap;

  // R stick interpretation
  // (Removed: L1+R-stick used to drive height — that's now D-pad UP/DOWN.)
  if (r1) {
    ctrl_target_roll  = -curve(rx_raw) * CTRL_R1_LEAN_MAX_DEG;
    ctrl_target_body_y = -curve(ry_raw) * CTRL_R1_SHIFT_MAX_CM;
  } else if (l1) {
    // L1 alone is a strafe modifier (with D-pad) — don't move pose from R-stick
    // while it's held, so accidental stick wiggle doesn't tilt Gary.
    ctrl_target_roll  *= 0.7f;
    ctrl_target_pitch *= 0.7f;
    ctrl_target_body_y = 0.0f;
  } else {
    bool l_stopped = (fabsf(ctrl_target_vy) < 0.05f && fabsf(ctrl_target_omega) < 0.05f);
    if (l_stopped) {
      ctrl_target_roll  = -curve(rx_raw) * CTRL_R_ROLL_MAX_DEG;
      ctrl_target_pitch = -curve(ry_raw) * CTRL_R_PITCH_MAX_DEG;
    } else {
      ctrl_target_roll  *= 0.7f;
      ctrl_target_pitch *= 0.7f;
    }
    ctrl_target_body_y = 0.0f;
  }

  // Strafe
  ctrl_target_vx = 0.0f;
  if (l1 && (dpad & DPAD_LEFT)) {
    ctrl_target_vx = -CTRL_VX_MAX * speed_cap;
    set_strafe(-1, true);
  } else if (l1 && (dpad & DPAD_RIGHT)) {
    ctrl_target_vx =  CTRL_VX_MAX * speed_cap;
    set_strafe(+1, true);
  } else if (ctrl_strafe_toggle_dir != 0) {
    ctrl_target_vx = (float)ctrl_strafe_toggle_dir * CTRL_VX_MAX * speed_cap;
  }

  // Smoothing
  ctrl_smooth_vy     += CTRL_SMOOTHING * (ctrl_target_vy     - ctrl_smooth_vy);
  ctrl_smooth_omega  += CTRL_SMOOTHING * (ctrl_target_omega  - ctrl_smooth_omega);
  ctrl_smooth_vx     += CTRL_SMOOTHING * (ctrl_target_vx     - ctrl_smooth_vx);
  ctrl_smooth_pitch  += CTRL_SMOOTHING * (ctrl_target_pitch  - ctrl_smooth_pitch);
  ctrl_smooth_roll   += CTRL_SMOOTHING * (ctrl_target_roll   - ctrl_smooth_roll);
  ctrl_smooth_height += CTRL_SMOOTHING * (ctrl_target_height - ctrl_smooth_height);
  ctrl_smooth_body_y += CTRL_SMOOTHING * (ctrl_target_body_y - ctrl_smooth_body_y);

  // ---- HARD CLAMPS before issuing move ----
  float send_pitch  = clamp_f(ctrl_smooth_pitch,  -CLAMP_PITCH_DEG, CLAMP_PITCH_DEG);
  float send_roll   = clamp_f(ctrl_smooth_roll,   -CLAMP_ROLL_DEG,  CLAMP_ROLL_DEG);
  float send_height = clamp_f(ctrl_smooth_height, CLAMP_HEIGHT_MIN, CLAMP_HEIGHT_MAX);
  float send_body_y = clamp_f(ctrl_smooth_body_y, -CLAMP_BODY_Y_CM, CLAMP_BODY_Y_CM);
  float send_vx     = clamp_f(ctrl_smooth_vx,     -CLAMP_VX, CLAMP_VX);
  float send_vy     = clamp_f(ctrl_smooth_vy,     -CLAMP_VY, CLAMP_VY);
  float send_omega  = clamp_f(ctrl_smooth_omega,  -CLAMP_OMEGA, CLAMP_OMEGA);

  // ---- Cadence-from-stick ----
  // Library default is 600ms; floor 400ms when MOVING. Map demand magnitude
  // (the largest of vy/vx/omega normalized) to step period: 0 -> CADENCE_MAX_MS,
  // 1 -> CADENCE_MIN_MS. This gives variable cadence the prior bare-call missed.
  float vy_demand = fabsf(send_vy)    / CTRL_VY_MAX;
  float vx_demand = fabsf(send_vx)    / CTRL_VX_MAX;
  float om_demand = fabsf(send_omega) / CTRL_OMEGA_STATIONARY;
  float demand    = vy_demand;
  if (vx_demand > demand) demand = vx_demand;
  if (om_demand > demand) demand = om_demand;
  if (demand > 1.0f) demand = 1.0f;
  uint16_t move_time_ms = (uint16_t)(CTRL_CADENCE_MAX_MS
                                    + (CTRL_CADENCE_MIN_MS - CTRL_CADENCE_MAX_MS) * demand);

  // ---- Push to gary.move() — Hiwonder pattern ----
  Velocity_t cv = {send_vx, send_vy, send_omega};
  Vector_t   cp = {0.0f, send_body_y, send_height};
  Euler_t    ca = {send_pitch, send_roll, 0.0f};

  // Hysteresis on the moving flag. Stick deflection near the boundary used to
  // flicker `moving` true/false repeatedly, switching between custom scheduler
  // and library stationary mode every loop — that's the touch/release stutter
  // and the "fights itself at intermediate stick" wild-movement behavior.
  // Now: need a clear push (HI) to start moving, smaller value (LO) to keep going.
  static bool was_moving = false;
  const float MOVE_THRESH_HI = 0.15f;   // need this much demand to START moving
  const float MOVE_THRESH_LO = 0.04f;   // stays moving until below this
  float thresh = was_moving ? MOVE_THRESH_LO : MOVE_THRESH_HI;
  bool moving = fabsf(send_vy) > thresh
             || fabsf(send_omega) > thresh
             || fabsf(send_vx) > thresh;
  was_moving = moving;

  // ---- Odometry ------------------------------------------------------------
  // This used to be called WAY up at the top of the function, on the raw
  // send_vy/send_omega, before any of the decisions below. Two bugs came out of
  // that, and between them they were the reason the LiDAR-regressed fit only
  // reached R^2 ~0.5 -- odometry was describing a robot that wasn't there:
  //
  //   1. IT INTEGRATED WHILE THE LEGS WERE STANDING STILL. The `moving` gate
  //      above has hysteresis (start at 0.15, keep going down to 0.04), but
  //      ctrl_smooth_vy is a SMOOTHED value that ramps down through that whole
  //      band. So every time you released the stick, the legs stopped dead --
  //      and odometry kept cheerfully integrating the decaying velocity all the
  //      way to zero. Phantom motion, injected on every single start and stop.
  //
  //   2. IT USED A DIFFERENT VELOCITY THAN THE LEGS DID. Wave and ripple scale
  //      the command by GAIT_SPEED_CAP / GAIT_OMEGA_CAP before it reaches the
  //      legs; odometry never saw that. Harmless while you stay in TRIPOD, but
  //      it means the calibration would silently break the moment you switch
  //      gait -- the worst kind of bug, because nothing would look wrong.
  //
  // Fix: feed odometry EXACTLY what the legs get, and nothing when they get
  // nothing. If the feet aren't moving, neither is Gary.
  //
  //   3. AND THE DEEPEST ONE: send_vy IS THE WRONG VARIABLE ENTIRELY.
  //      A walking robot's speed is STRIDE / STEP_PERIOD. Neither of those is
  //      send_vy. The library converts velocity into a leg AMPLITUDE that
  //      SATURATES at 3.0cm (visible in the [CTRL] amp2/amp5 debug: it pins at
  //      2.99/-3.00 and stops), while move_time_ms independently slides the step
  //      PERIOD from 1200ms down to 400ms with stick demand. So past saturation,
  //      pushing the stick further doesn't lengthen his stride at all -- it just
  //      makes him step faster. Speed is therefore grossly NON-LINEAR in send_vy,
  //      and no linear coefficient on send_vy can ever describe it. That is why
  //      the LiDAR regression could fit forward motion only to R^2 0.86 and yaw
  //      to a hopeless 0.46, and why the gait-veer term flipped sign between two
  //      runs -- a coefficient that changes sign isn't being measured, it's being
  //      guessed at.
  //
  // So: integrate what the LEGS ACTUALLY DO, not what the stick asked for. This
  // is leg odometry, and it is the honest version. amplitude/period IS a speed,
  // it already accounts for saturation and cadence for free, and it keeps working
  // when you change gait.
  //
  // NOTE we read the amplitudes BEFORE gary.move() below overwrites them. That is
  // deliberate and correct: dt covers the interval that just ELAPSED, during which
  // the PREVIOUS tick's amplitude was the one in effect.
  //
  // The +/- combination of leg2 and leg5 that separates translation from rotation
  // depends on Hiwonder's per-leg frame conventions, which are mirrored left/right
  // and which I am not going to guess at. I don't have to: the LiDAR regression
  // fits a general linear map from these two numbers to real motion, so it will
  // discover the correct combination itself. Feeding it the raw legs is strictly
  // safer than feeding it my assumption about their signs.
  {
    uint16_t eff_T = move_time_ms;
    if (ctrl_gait_pattern == GAIT_WAVE || ctrl_gait_pattern == GAIT_RIPPLE) {
      int g = (int)ctrl_gait_pattern;
      if (eff_T < gait_min_cadence(g)) eff_T = gait_min_cadence(g);
    }
    if (eff_T < 1) eff_T = 1;
    float T_sec = eff_T / 1000.0f;

    float u_a = 0.0f, u_b = 0.0f;
    if (moving) {
      u_a = gary.leg2.amplitude.y / T_sec;   // cm of half-stroke per second
      u_b = gary.leg5.amplitude.y / T_sec;
    }
    odom_update(u_a, u_b);
  }

  if (moving) {
    switch (ctrl_gait_pattern) {
      case GAIT_WAVE:
      case GAIT_RIPPLE: {
        // Per-gait velocity & cadence limits — wave/ripple can't sustain
        // tripod-level speeds. Yaw is hardest in slow gaits so it gets a
        // tighter cap than translation.
        int g = (int)ctrl_gait_pattern;
#if GAIT_TUNED
        // Sim-trained caps/cadence/lift (see gary_config.h GAIT_TUNED block).
        // Caps derated by GAIT_TUNED_CAP_SCALE for the first bench runs; omega
        // caps were not trained and keep their July values.
        float speed_cap = (ctrl_gait_pattern == GAIT_WAVE ? GAIT_TUNED_SPEED_CAP_WAVE
                                                          : GAIT_TUNED_SPEED_CAP_RIP)
                          * GAIT_TUNED_CAP_SCALE;
        uint16_t cadence = (ctrl_gait_pattern == GAIT_WAVE) ? GAIT_TUNED_CADENCE_WAVE
                                                            : GAIT_TUNED_CADENCE_RIP;
        Velocity_t cv_g = {
          send_vx    * speed_cap,
          send_vy    * speed_cap,
          send_omega * GAIT_OMEGA_CAP[g]
        };
        uint16_t t_g = move_time_ms;
        if (t_g < cadence) t_g = cadence;

        // Trained lift is used directly (the optimizer searched lift jointly
        // with cadence/caps, so the height-adaptive scaling and per-gait trims
        // are bypassed here — revisit if crouched/tall gaits misbehave).
        float lift = (ctrl_gait_pattern == GAIT_WAVE) ? GAIT_TUNED_LIFT_WAVE
                                                      : GAIT_TUNED_LIFT_RIP;
#else
        Velocity_t cv_g = {
          send_vx    * GAIT_SPEED_CAP[g],
          send_vy    * GAIT_SPEED_CAP[g],
          send_omega * GAIT_OMEGA_CAP[g]
        };
        uint16_t t_g = move_time_ms;
        if (t_g < GAIT_MIN_CADENCE[g]) t_g = GAIT_MIN_CADENCE[g];

        // Dynamic lift: scale with body height so foot clearance grows when
        // standing tall. (Mirrors the library's _leg_lift formula on z >= 0.)
        float lift = CUSTOM_GAIT_LIFT_CM;
        if      (send_height >= 2.0f) lift = 3.2f;
        else if (send_height >= 1.0f) lift = 2.7f;
        else if (send_height >= 0.0f) lift = 2.2f;
        else                          lift = 1.6f;   // crouched: smaller lift

        // Per-gait lift trim — wave is the "stick-bug" gait, lifts barely
        // need to clear the surface; ripple a touch lower than tripod-equivalent.
        if      (ctrl_gait_pattern == GAIT_WAVE)   lift *= 0.50f;
        else if (ctrl_gait_pattern == GAIT_RIPPLE) lift *= 0.75f;
#endif

        // 1) Push velocity to library so it updates leg.amplitude.
        gary.move(&cv_g, &cp, &ca, t_g);
        // 2) Disable library's tripod scheduler.
        gary.func_state = Robot::ACTION_GROUP;
        // 3) Drive legs ourselves. Pass omega so the scheduler can compute
        //    per-leg rotational amplitude (library doesn't encode it in
        //    leg.amplitude for our use, so we add it ourselves).
        run_custom_gait_scheduler((int)ctrl_gait_pattern, t_g, lift, cv_g.omega,
                                  cv_g.vy);
        break;
      }
      case GAIT_TRIPOD:
      default:
        // Library handles tripod natively. gary.move() resets func_state=CRAWL.
        gary.move(&cv, &cp, &ca, move_time_ms);
        break;
    }
  } else {
    // Stationary: library handles pose blends. gary.move() will reset
    // func_state=CRAWL automatically, so the library scheduler resumes.
    gary.move(&cv, &cp, &ca, 200);
  }

  // Periodic Serial debug
  if (millis() - last_debug_ms >= DEBUG_INTERVAL_MS) {
    last_debug_ms = millis();
    // Show the ACTUAL cadence used by the gait (post-floor), not the raw
    // cadence-from-stick value, which is misleading for wave/ripple.
    uint16_t actual_cadence = move_time_ms;
    if (ctrl_gait_pattern == GAIT_WAVE   && actual_cadence < gait_min_cadence(GAIT_WAVE))   actual_cadence = gait_min_cadence(GAIT_WAVE);
    if (ctrl_gait_pattern == GAIT_RIPPLE && actual_cadence < gait_min_cadence(GAIT_RIPPLE)) actual_cadence = gait_min_cadence(GAIT_RIPPLE);

    Serial.printf("[CTRL] L(%d,%d) R(%d,%d) | vy=%.2f om=%.2f pitch=%.1f roll=%.1f h=%.2f t=%ums gait=%s spd=%s\n",
      myGamepad->axisX(), myGamepad->axisY(),
      myGamepad->axisRX(), myGamepad->axisRY(),
      send_vy, send_omega, send_pitch, send_roll, send_height,
      (unsigned)actual_cadence,
      gait_pattern_name((int)ctrl_gait_pattern),
      CTRL_SPEED_NAMES[ctrl_gait_speed_idx]);
    // Diagnostic: amp, result, AND result.z. If result.z isn't reasonable
    // (i.e., foot home isn't at ground level), legs are floating during
    // stance and that explains marches-in-place.
    Serial.printf("[CTRL]   amp2=(%.2f,%.2f) amp5=(%.2f,%.2f) r2=(%.2f,%.2f,z=%.2f) r5=(%.2f,%.2f,z=%.2f)\n",
      gary.leg2.amplitude.x, gary.leg2.amplitude.y,
      gary.leg5.amplitude.x, gary.leg5.amplitude.y,
      gary.leg2.result.x, gary.leg2.result.y, gary.leg2.result.z,
      gary.leg5.result.x, gary.leg5.result.y, gary.leg5.result.z);
  }

  // Save edge state
  prev_btn.cross = cross; prev_btn.circle = circle;
  prev_btn.triangle = triangle; prev_btn.square = square;
  prev_btn.l1 = l1; prev_btn.r1 = r1;
  prev_btn.l2 = l2; prev_btn.r2 = r2;
  prev_btn.l3 = l3; prev_btn.r3 = r3;
  prev_btn.select_b = select_b; prev_btn.start_b = start_b;
  prev_btn.ps_b = ps_b; prev_btn.mute_b = mute_b;
  prev_btn.tp_click = tp_click;
  prev_btn.dpad = dpad;
}

// -----------------------------------------------------------------------------
// Leg odometry, calibrated against the LiDAR
// -----------------------------------------------------------------------------
// This used to be two placeholder scale constants (ODOM_VY_SCALE = 1.0,
// ODOM_OMEGA_SCALE = 1.0) multiplying the JOYSTICK command. That was wrong three
// times over, and it took a LiDAR-based regression (gary_odom_fit.py) to see it:
//
//   1. It integrated even while the legs were standing still, because the
//      `moving` hysteresis gate stops the legs but the smoothed velocity keeps
//      ramping down through the deadband. Phantom motion on every stick release.
//
//   2. It used a different velocity than the legs did (wave/ripple scale the
//      command by GAIT_SPEED_CAP first).
//
//   3. THE REAL ONE: send_vy is simply not a speed. The library turns velocity
//      into a leg AMPLITUDE that SATURATES at 3.0cm, while move_time_ms
//      separately slides the step PERIOD from 1200ms down to 400ms. Past
//      saturation, more stick doesn't lengthen his stride at all -- it just makes
//      him step faster. Speed is stride/period. It is grossly non-linear in the
//      stick, and NO constant could ever have fixed that.
//
// So we now integrate what the LEGS actually do: each leg's stroke rate
// (amplitude / step-period), which IS a speed, and which handles saturation,
// cadence and gait changes for free.
//
// ODOM_M was fitted by gary_odom_fit.py: drive Gary around for two minutes while
// it scan-matches consecutive LiDAR rings to measure how far he ACTUALLY moved,
// then least-squares a linear map from the leg strokes to real body motion over
// ~250 samples. R^2: 0.78 forward, 0.53 sideways, 0.81 yaw.
//
// Change basis to the sum/difference of the two legs and it reads as a
// differential drive, and Gary's limp falls out as measurable numbers:
//
//     VEER : walking "straight", he yaws  -9.7 deg per METRE
//     ORBIT: spinning "in place", he slides +6.8 cm sideways per 90 deg
//     CREEP: spinning "in place", he drifts +1.3 cm forward  per 90 deg
//
// The -9.7 deg/m independently reproduces the 9-14 deg of unwanted rotation seen
// over a 1-2m "straight" leg in an earlier, completely separate test. Two methods,
// no shared data, same answer.
//
// THIS DOES NOT FIX THE LIMP. It stops the odometry being SURPRISED by it, which
// is what makes the map stitch together. He still walks crooked; that is a gait
// tuning job. Re-run gary_odom_fit.py after any gait change and watch VEER.
//
// STILL TRUE, STILL DEAD RECKONING: no loop closure, no scan-matching correction.
// The residual ~20% is real foot scrub that varies with speed/load/floor, which no
// linear model captures. But it is now UNBIASED noise rather than a systematic
// lie -- the map drifts like a random walk instead of curving away from reality.
// Also still only fed from process_controller(): the pose freezes during
// autonomous roaming (exec_walk/exec_turn don't call odom_update).
//
// Columns are the two leg stroke-rates (leg2, leg5), integrated over dt.
// Rows are body-frame dx (cm), dy (cm), dtheta (deg).
static float ODOM_M[3][2] = {
  {   +1.89145f,   -1.63644f },   // dx_cm      per (leg2_rate, leg5_rate)
  {   +0.69778f,   +0.63285f },   // dy_cm      per (leg2_rate, leg5_rate)
  {   +8.57150f,   +8.91489f },   // dtheta_deg per (leg2_rate, leg5_rate)
};

// ---- HEADING COMES FROM THE GYRO, NOT THE LEGS ------------------------------
// The leg model predicts yaw with R^2 0.81, which sounded fine until we looked at
// what it means: yaw RMS error of 10.5 DEGREES PER STEP. Heading then random-walks
// away at ~10*sqrt(N) degrees, so after a few hundred rings Gary has no idea which
// way he is facing -- and every ring gets stamped into the map at a different wrong
// angle. That is exactly the smeared concentric arcs the live map was drawing.
//
// This is not a calibration failure. The leg fit is UNBIASED (ratio 1.004/1.035).
// It is simply too NOISY in yaw ever to map with, and it always will be: turning a
// hexapod is mostly foot SCRUB, and scrub is chaotic. No amount of fitting fixes a
// variable that isn't predictable in principle.
//
// So use the right sensor. A gyro measures rotation DIRECTLY -- it does not have to
// infer it from what the feet were asked to do. Translation still comes from the
// legs (they're good at that: forward RMS is only 32mm), heading comes from the IMU.
// Legs for position, gyro for heading. That split is standard practice in wheeled
// and legged robotics alike, and it is the single biggest thing standing between
// Gary and a map that looks like a room.
//
// GYRO_YAW_SCALE absorbs the unit (dps vs rad/s) AND the sign convention, neither of
// which I am going to guess at. Calibrate it: flash, run
//     python gary_odom_fit.py --source <src> --identity
// and read the "yaw ratio" it prints. Multiply GYRO_YAW_SCALE by that number and
// reflash. If it comes back near -1, the gyro's +Z is Gary's clockwise. If near
// 57.3, the driver hands back rad/s. Either way one number fixes it.
static float GYRO_YAW_SCALE = 1.0f;

// Gyros drift. The board applies its own offset, but any residual bias integrates
// straight into heading error, so we keep re-zeroing it whenever the legs are
// commanded to stand still -- which, while mapping, is most of the time.
static float    gyro_bias_y = 0.0f;   // needed for the tilt projection below
static float    gyro_bias_z = 0.0f;
static bool     gyro_bias_ready = false;
static uint32_t gyro_bias_n = 0;

static float odom_x = 0.0f;         // cm, world frame
static float odom_y = 0.0f;         // cm, world frame
static float odom_theta_deg = 0.0f; // deg, world frame heading (0 = however Gary was facing at boot)
static uint32_t odom_last_update_ms = 0;

void odom_update(float leg2_rate, float leg5_rate) {
  uint32_t now = millis();
  if (odom_last_update_ms == 0) { odom_last_update_ms = now; return; }
  float dt = (now - odom_last_update_ms) / 1000.0f;
  odom_last_update_ms = now;
  // Was 0.5s. loop() only runs at ~4Hz, so dt is typically 0.25s and an ordinary
  // hitch brushed that cutoff -- silently DISCARDING real motion. 1.0s still
  // rejects the genuinely absurd gaps (returning from a multi-second voice action).
  if (dt <= 0.0f || dt > 1.0f) return;

  bool standing = (leg2_rate == 0.0f && leg5_rate == 0.0f);

  // ---- heading: gyro, PROJECTED ONTO THE WORLD VERTICAL ---------------------
  // gyro_z measures rotation about GARY'S z-axis. Heading is rotation about the
  // WORLD'S vertical. Those are the same axis only while he is level. Tilt him and
  // they diverge, and some of his pitch/roll rate leaks into yaw.
  //
  // This is not theoretical. In the pose trace, with his feet planted and only the
  // right stick moving, theta wandered 8 degrees purely from being tilted +/-18.
  // Since tilting is EXACTLY what he has to do to build a 3D map, an uncorrected
  // yaw would corrupt the very maps we added tilt in order to make.
  //
  // The standard projection of body rates onto the world yaw axis:
  //
  //     psi_dot = ( gy*sin(roll) + gz*cos(roll) ) / cos(pitch)
  //
  // The 1/cos(pitch) blows up as he approaches vertical, so it is clamped. He
  // physically cannot pitch past ~25 degrees without falling over, so the clamp
  // only ever fires on garbage.
  float g[3] = {0, 0, 0};
  gary.board.get_imu_gyro(g);
  float e[3] = {0, 0, 0};
  gary.board.get_imu_euler(e);        // e[0]=roll  e[1]=pitch  (degrees)

  float gy = g[1], gz = g[2];

  if (standing) {
    // Re-zero the bias while the feet are still. Exponential average so it tracks
    // thermal drift without a single jolt poisoning it.
    gyro_bias_y += 0.02f * (gy - gyro_bias_y);
    gyro_bias_z += 0.02f * (gz - gyro_bias_z);
    if (++gyro_bias_n > 50) gyro_bias_ready = true;
  }

  float dth;
  if (gyro_bias_ready) {
    float roll_r  = e[0] * (PI / 180.0f);
    float pitch_r = e[1] * (PI / 180.0f);
    float cp = cosf(pitch_r);
    if (fabsf(cp) < 0.5f) cp = (cp < 0.0f) ? -0.5f : 0.5f;   // clamp; see above
    float yaw_rate = ((gy - gyro_bias_y) * sinf(roll_r)
                    + (gz - gyro_bias_z) * cosf(roll_r)) / cp;
    dth = GYRO_YAW_SCALE * yaw_rate * dt;
  } else {
    // No bias estimate yet (first second or two after boot). Fall back to the leg
    // model rather than integrating an unknown offset.
    dth = ODOM_M[2][0] * (leg2_rate * dt) + ODOM_M[2][1] * (leg5_rate * dt);
  }

  // ---- translation: legs ---------------------------------------------------
  float a = leg2_rate * dt;
  float b = leg5_rate * dt;
  float dx_body = ODOM_M[0][0] * a + ODOM_M[0][1] * b;   // cm, forward
  float dy_body = ODOM_M[1][0] * a + ODOM_M[1][1] * b;   // cm, left

  // Midpoint integration: a step that both moves and turns lands between the two
  // headings, not at either end.
  float th0 = odom_theta_deg * (PI / 180.0f);
  odom_theta_deg += dth;
  while (odom_theta_deg > 180.0f)  odom_theta_deg -= 360.0f;
  while (odom_theta_deg < -180.0f) odom_theta_deg += 360.0f;
  float th_mid = (th0 + odom_theta_deg * (PI / 180.0f)) / 2.0f;

  float c = cosf(th_mid), sn = sinf(th_mid);
  odom_x += dx_body * c  - dy_body * sn;
  odom_y += dx_body * sn + dy_body * c;
}

// ---- VOICE DIAGNOSTIC --------------------------------------------------------
// Voice has been dead for a while and I have been assuming it was the slow loop
// starving the poll. That is a GUESS. Before optimising anything, establish the
// one fact that decides which bug this is:
//
//   Does the ASR module EVER return a command?
//
//   * polls/sec is healthy but cmd is NEVER seen  -> the module isn't hearing you,
//     or isn't on the bus. Polling faster cannot fix that. Look at the mic/I2C.
//   * cmd IS seen but the robot ignores it        -> the poll works and the bug is
//     downstream, in handle_voice_command / state handling.
//   * polls/sec is tiny                           -> THEN it is the loop rate.
//
// Every rec_recognition() call in this file now goes through here, so the counters
// cover all states, not just the one you happened to be in.
static uint32_t asr_polls = 0;
static uint32_t asr_hits  = 0;
static uint32_t asr_bad   = 0;   // 0xFF = failed I2C read, NOT a command
static uint8_t  asr_last  = 0;

static uint8_t asr_poll() {
  // RATE-LIMIT. The loop now runs at 220Hz, so this was hammering the ASR 220
  // times a second over an I2C bus that ALSO carries the IMU timer (100Hz) and 18
  // servos. That is almost certainly what the cmd=0xFF reads were: 0xFF is what an
  // I2C read looks like when nothing drives the bus low -- a COLLISION, not a word.
  //
  // Nothing here needs 220Hz. A spoken word takes half a second; the module latches
  // its result. 20Hz is 10x faster than it needs to be and stops us fighting the
  // IMU for the bus.
  //
  // Note the irony: I spent this whole session assuming voice was broken because the
  // loop was too SLOW. Then I fixed the loop, and it may have broken it further by
  // being too FAST.
  static uint32_t last_asr_ms = 0;
  uint32_t now = millis();
  if (now - last_asr_ms < 50) return CMD_NONE;   // 20Hz
  last_asr_ms = now;

  asr_polls++;
  uint8_t cmd = gary.sensor.asr.rec_recognition();
  if (cmd == 0xFF) {
    // Not a command -- a failed bus read. Count it separately so it can never be
    // mistaken for speech again.
    asr_bad++;
    return CMD_NONE;
  }
  if (cmd != CMD_NONE) {
    asr_hits++;
    asr_last = cmd;
    // The WonderEcho is now playing this command's acknowledgment clip; hold off
    // the sound-startle so his own voice doesn't scare him out of EXPLORE.
    startle_suppress_until_ms = now + VOICE_ACK_STARTLE_SUPPRESS_MS;
    Serial.printf(">>> ASR HIT | cmd=%u  (state=%s)\n", cmd, state_name(current_state));
  }
  return cmd;
}

void loop_controller() {
  // FIX: this used to only call process_controller() and never polled the
  // mic, so voice commands ("dance", "push-up", etc.) were silently dropped
  // the entire time the PS5 controller was connected -- every other loop_*
  // state (idle/listening/roaming/coming) polls rec_recognition() but this
  // one didn't. Mirror that pattern here.
  //
  // handle_voice_command() runs a blocking action sequence and ends by
  // calling return_to_listening(), which sets current_state = STATE_LISTENING
  // (not STATE_CONTROLLER). That's fine and self-healing: as long as the
  // gamepad is still connected, the very next loop() iteration's
  // "if (myGamepad->isConnected() && current_state != STATE_CONTROLLER)"
  // check (see loop()) flips it straight back into STATE_CONTROLLER, so
  // joystick control resumes automatically once the spoken action finishes.
  // While the action plays, joystick input is ignored (same as any other
  // voice-triggered action sequence elsewhere in this file).
#if ENABLE_VOICE
  uint8_t cmd = asr_poll();
  if (cmd != CMD_NONE) {
    Serial.printf("[CTRL-ASR] voice cmd=%u accepted in controller mode\n", cmd);
    handle_voice_command(cmd);
    return;
  }
#endif
  process_controller();
}

void stop_motion() {
  vel = {0.0f, 0.0f, 0.0f};
  gary.move(&vel, &pos, &att);
}

// -----------------------------------------------------------------------------
// Movement helpers
// -----------------------------------------------------------------------------

void exec_walk(float vy, uint16_t step_ms, uint8_t steps) {
  transition_to(STATE_EXECUTING);
  vel = {0.0f, vy, 0.0f};
  gary.move(&vel, &pos, &att, step_ms, steps);
  gary_delay(step_ms * steps + 100);
  return_to_listening();
}

void exec_turn(float yaw_rate, uint16_t step_ms, uint8_t steps) {
  transition_to(STATE_EXECUTING);
  vel = {0.0f, 0.0f, yaw_rate};
  gary.move(&vel, &pos, &att, step_ms, steps);
  gary_delay(step_ms * steps + 100);
  return_to_listening();
}

void exec_pose(Vector_t new_pos, uint16_t hold_ms) {
  transition_to(STATE_EXECUTING);
  Velocity_t zero = {0.0f, 0.0f, 0.0f};
  Euler_t zero_att = {0.0f, 0.0f, 0.0f};
  gary.move(&zero, &new_pos, &zero_att, hold_ms);
  gary_delay(hold_ms + 100);
  Vector_t standing = {0.0f, 0.0f, 1.5f};
  gary.move(&zero, &standing, &zero_att, 600);
  gary_delay(700);
  return_to_listening();
}

// =============================================================================
// STATE ENTRY HANDLERS
// =============================================================================

void on_state_enter(GaryState s) {
  switch (s) {
    case STATE_BOOT:
      set_eyes(EYE_SABER_BLUE);      // lightsaber blue at startup
      anim_lightsaber();             // saber ignition
      say(CLIP_WHISTLE);
      break;

    case STATE_IDLE:
      set_eyes_breathing(EYE_IDLE);
      anim_control_panel();          // dashboard lights — Gary is on standby
      stop_motion();
      break;

    case STATE_LISTENING:
      set_eyes_breathing(EYE_LISTENING);
      anim_sensor_grid();
      say(CLIP_LOOK);
      listening_got_command = false;
      break;

    case STATE_EXECUTING:
      set_eyes(EYE_IMPERIAL);
      anim_progress_bars();
      say(CLIP_SNAPPY);
      break;

    case STATE_ROAMING:
      set_eyes_breathing(EYE_TIE_GREEN);  // engine glow
      anim_radar_room();             // sweeping radar with persistent blips
      say(CLIP_CHATTER);
      last_boredom_kick_ms = millis();
      break;

    case STATE_COMING:
      set_eyes_breathing(EYE_REBEL);           // rebel orange — heading to base
      anim_radar_room();                 // X-Radar search
      say(CLIP_WHISTLING_EXCITED);
      coming_phase = COMING_SEARCH;
      break;

    case STATE_STARTLED:
      set_eyes(EYE_HYPERSPACE);      // hyperspace jump panic blue
      anim_hyperspace();             // panic radial streaks
      say(CLIP_SCREAM);
      // Backwards jump WHILE screaming — non-blocking sound makes this work
      vel = {0.0f, -5.0f, 0.0f};
      gary.move(&vel, &pos, &att, 300, 1);
      break;

    case STATE_ALARMED:
      set_eyes_breathing(EYE_SABER_RED);  // sith red — turn to dark side
      anim_death_star();             // Death Star charging — ominous
      say(CLIP_SAD);
      stop_motion();
      break;

    case STATE_CONTROLLER:
      if (ctrl_pu_mode) {
        uint8_t green[3] = {0x00, 0xFF, 0x80};
        set_eyes_breathing(green);
      } else {
        set_eyes_breathing(EYE_SABER_BLUE);
      }
      anim_target_hud();
      stop_motion();
      ctrl_target_vy = 0.0f;
      ctrl_target_omega = 0.0f;
      ctrl_target_vx = 0.0f;
      ctrl_target_roll = 0.0f;
      ctrl_target_pitch = 0.0f;
      ctrl_target_height = 1.5f;
      ctrl_target_body_y = 0.0f;
      ctrl_smooth_vy = 0.0f;
      ctrl_smooth_omega = 0.0f;
      ctrl_smooth_vx = 0.0f;
      ctrl_smooth_roll = 0.0f;
      ctrl_smooth_pitch = 0.0f;
      ctrl_smooth_height = 1.5f;
      ctrl_smooth_body_y = 0.0f;
      Serial.println("[CTRL] Entering controller mode");
      break;

    case STATE_SENSING:
      set_eyes_breathing(EYE_SABER_PURPLE);  // distinct color -- "listening" with the radio, not the mic
      anim_sensor_grid();
      say(CLIP_PROCESSING);
      stop_motion();
      csi_capture_begin();
      break;

    case STATE_NODSCAN:
      set_eyes_breathing(EYE_SABER_GREEN);  // distinct color -- deliberate, slow scanning motion
      anim_sensor_grid();
      say(CLIP_PROCESSING);
      stop_motion();
      // The actual pitch+roll sweep runs synchronously from the CMD_ACTION_14
      // handler (same pattern as CMD_ACTION_12's battery display) -- this
      // just sets the visual/audible "starting a scan" feedback. AUTOSCAN's
      // triggered scans call run_nod_scan() directly without visiting this
      // state, since they fire from inside STATE_AUTOSCAN.
      break;

    case STATE_AUTOSCAN:
      set_eyes_breathing(EYE_ALERT);   // distinct amber "on watch" color
      anim_radar_room();               // sweeping radar fits a background watch
      say(CLIP_DETERMINED);
      stop_motion();
      autoscan_phase = AUTOSCAN_WARMUP;
      autoscan_baseline_rssi = 0.0f;
      autoscan_baseline_n = 0;
      autoscan_recent_count = 0;
      autoscan_recent_idx = 0;
      autoscan_triggered_count = 0;
      csi_capture_begin();
      Serial.println(">>> AUTOSCAN: scanning mode ON -- establishing RSSI baseline");
      break;
  }
}

// =============================================================================
// VOICE COMMAND HANDLER
// =============================================================================

bool handle_voice_command(uint8_t cmd) {
  if (cmd == CMD_NONE) return false;

#if ENABLE_DEBUG_SERIAL
  Serial.print("VOICE CMD: 0x");
  if (cmd < 0x10) Serial.print("0");
  Serial.println(cmd, HEX);
#endif

  switch (cmd) {

    // ====================================================================
    //  MOVEMENT
    // ====================================================================
    case CMD_FORWARD:
      exec_walk(WALK_SPEED, STEP_TIME_MS, DEFAULT_STEPS);
      return true;

    case CMD_BACKWARD:
      // BACK + concerned chirp simultaneously
      say(CLIP_CONCERNED);
      exec_walk(-WALK_SPEED, STEP_TIME_MS, DEFAULT_STEPS);
      return true;

    case CMD_TURN_LEFT:
      exec_turn(TURN_SPEED, STEP_TIME_MS, DEFAULT_STEPS);
      return true;

    case CMD_TURN_RIGHT:
      exec_turn(-TURN_SPEED, STEP_TIME_MS, DEFAULT_STEPS);
      return true;

    case CMD_MARCH:
      // BENCH 2026-08-23 (Bill: "march makes him sorta back-away in a
      // turn"): this was wired to the dismissal walk-off turn. March
      // marches -- same routine as CMD_MARCH_PLACE.
      transition_to(STATE_EXECUTING);
      say(CLIP_DETERMINED);
      for (int i = 0; i < 4; i++) {
        vel = {0.0f, 0.5f, 0.0f};
        gary.move(&vel, &pos, &att, 300, 1);
        gary_delay(350);
      }
      return_to_listening();
      return true;

    case CMD_SPEED_UP: {
      transition_to(STATE_EXECUTING);
      vel = {0.0f, WALK_SPEED * 2.0f, 0.0f};
      gary.move(&vel, &pos, &att, STEP_TIME_MS, DEFAULT_STEPS);
      gary_delay(STEP_TIME_MS * DEFAULT_STEPS + 300);
      return_to_listening();
      return true;
    }

    case CMD_SLOW_DOWN: {
      transition_to(STATE_EXECUTING);
      vel = {0.0f, WALK_SPEED * 0.25f, 0.0f};
      gary.move(&vel, &pos, &att, STEP_TIME_MS, DEFAULT_STEPS);
      gary_delay(STEP_TIME_MS * DEFAULT_STEPS + 300);
      return_to_listening();
      return true;
    }

    case CMD_STOP:
      stop_motion();
      say(CLIP_SNAPPY);
      return_to_listening();
      return true;

    case CMD_ATTENTION:
      stop_motion();
      set_eyes(EYE_ALERT);
      say(CLIP_DETERMINED);
      return_to_listening();
      return true;

    case CMD_SIT_DOWN:
      exec_pose({0.0f, 0.0f, 0.0f}, 1500);
      return true;

    case CMD_RAISE_BODY:
      exec_pose({0.0f, 0.0f, 100.0f}, 3000);
      return true;

    // ====================================================================
    //  SPEECH / GREETING
    // ====================================================================
    case CMD_HELLO:
      transition_to(STATE_EXECUTING);
      say(CLIP_PLAYFUL);
      pos = {0.0f, 0.0f, .0f};
      gary.move(&vel, &pos, &att, 400);
      gary_delay(500);
      pos = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 400);
      gary_delay(500);
      return_to_listening();
      return true;

    case CMD_INTRODUCE:
      transition_to(STATE_EXECUTING);
      gary_introduce_routine();
      return_to_listening();
      return true;

    case CMD_SHOW_SKILL:
      transition_to(STATE_EXECUTING);
      gary_show_skill_routine();
      return_to_listening();
      return true;

    case CMD_SHAKE_HEAD:
      // THE REAL WAG: body yawing between PLANTED feet. The old version
      // rolled the chassis +/-20 deg, which reads as a lean, not a
      // shake -- and with no yaw DoF it was the closest thing available.
      transition_to(STATE_EXECUTING);
      say(CLIP_WHISTLE_INQUISITIVE);
      gary_head_shake(3, 0.30f, 550);
      return_to_listening();
      return true;

    case CMD_COMBAT:
      transition_to(STATE_ALARMED);
      say(CLIP_DETERMINED);
      pos = {0.0f, 0.0f, 0.5f};
      att = {20.0f, 0.0f, -5.0f};
      gary.move(&vel, &pos, &att, 400);
      gary_delay(500);
      vel = {0.0f, WALK_SPEED * 1.5f, 0.0f};
      gary.move(&vel, &pos, &att, 600, 1);
      gary_delay(700);
      pos = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 400);
      gary_delay(500);
      return_to_listening();
      return true;

    case CMD_SHAKE_OFF:
      transition_to(STATE_EXECUTING);
      say(CLIP_LAUGHTER);
      for (int i = 0; i < 3; i++) {
        att = {0.0f, 0.0f, 10.0f};
        gary.move(&vel, &pos, &att, 200);
        gary_delay(220);
        att = {0.0f, 0.0f, -10.0f};
        gary.move(&vel, &pos, &att, 200);
        gary_delay(220);
      }
      att = {0.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 200);
      gary_delay(220);
      return_to_listening();
      return true;

    // ====================================================================
    //  DANCE / ANIMATIONS
    // ====================================================================
    case CMD_DANCE:
      transition_to(STATE_EXECUTING);
      gary_dance_routine();
      return_to_listening();
      return true;

    case CMD_BOW:
      // BOW — pitch forward AND lower body so it reads as a deep, deliberate bow
      transition_to(STATE_EXECUTING);
      say(CLIP_PROUD);
      att = {40.0f, 0.0f, 0.0f};       // ~28° forward pitch (was 0.3 / ~17°)
      pos = {0.0f, -20.0f, 0.7f};       // also lower the body
      gary.move(&vel, &pos, &att, 1200);  // slower so legs can actually reach it
      gary_delay(1400);
      gary_delay(1500);                // hold the bow
      att = {0.0f, 0.0f, 0.0f};
      pos = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 1500);  // unhurried rise
      gary_delay(1100);
      return_to_listening();
      return true;

    case CMD_WAVE:
      // WAVE — rise tall, pitch back, big side-to-side roll. Big values + slow
      // durations so the IK can actually achieve the poses.
      transition_to(STATE_EXECUTING);
      say(CLIP_PLAYFUL);
      pos = {0.0f, 0.0f, 1.9f};         // stand taller
      att = {0.0f, -0.4f, 0.0f};        // pitch back ~23° (was -0.2 / ~11°)
      gary.move(&vel, &pos, &att, 800);
      gary_delay(900);
      for (int i = 0; i < 3; i++) {
        att = {0.5f, -0.4f, 0.0f};      // big roll right (~28°)
        gary.move(&vel, &pos, &att, 500);
        gary_delay(550);
        att = {-0.5f, -0.4f, 0.0f};     // big roll left
        gary.move(&vel, &pos, &att, 500);
        gary_delay(550);
      }
      att = {0.0f, 0.0f, 0.0f};
      pos = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 700);
      gary_delay(800);
      return_to_listening();
      return true;

    case CMD_MARCH_PLACE:
      transition_to(STATE_EXECUTING);
      say(CLIP_DETERMINED);                  // low growl masks any library squeak
      for (int i = 0; i < 4; i++) {
        vel = {0.0f, 0.5f, 0.0f};
        gary.move(&vel, &pos, &att, 300, 1);
        if (i == 1) say(CLIP_PLAYFUL);       // mid-routine chatter
        gary_delay(350);
      }
      return_to_listening();
      return true;

case CMD_STRUT:
  transition_to(STATE_EXECUTING);
  say(CLIP_PROUD);
  pos = {0.0f, 0.0f, 2.0f};         // head up
  
  for (int i = 0; i < 3; i++) {
    // Step 1: hip swing right + step forward
    att = {-15.0f, 0.0f, 30.0f};      // roll right ~14°
    vel = {0.0f, 2.0f, 0.0f};       // strong forward push
    gary.move(&vel, &pos, &att, 2000, 1);  // ONE long step
    gary_delay(1000);
    say(CLIP_PROUD);
    // Step 2: hip swing left + step forward
    att = {-10.0f, 0.0f, -30.0f};     // roll left
    gary.move(&vel, &pos, &att, 2000, 1);  // ONE long step
    gary_delay(1000);
  }
  
  // reset
  pos = {0.0f, 0.0f, 1.5f};
  att = {0.0f, 0.0f, 0.0f};
  vel = {0.0f, 0.0f, 0.0f};
  gary.move(&vel, &pos, &att, 400);
  gary_delay(500);
  return_to_listening();
  return true;

    // ====================================================================
    //  ACTION SLOTS
    // ====================================================================

    case CMD_ACTION_1:
      // Gary RUN — sustained sprint + excited screaming
      transition_to(STATE_EXECUTING);
      say(CLIP_VERY_EXCITED);
      vel = {0.0f, WALK_SPEED * 1.8f, 0.0f};
      gary.move(&vel, &pos, &att, STEP_TIME_MS, 4);
      gary_delay(STEP_TIME_MS * 4 + 100);
      return_to_listening();
      return true;

    case CMD_ACTION_2:
      transition_to(STATE_EXECUTING);
      say(CLIP_DETERMINED);
      vel = {0.0f, 0.0f, TURN_SPEED};
      gary.move(&vel, &pos, &att, DISMISSAL_TURN_MS, 1);
      gary_delay(DISMISSAL_TURN_MS + 200);
      vel = {0.0f, WALK_SPEED, 0.0f};
      gary.move(&vel, &pos, &att, STEP_TIME_MS, 3);
      gary_delay(STEP_TIME_MS * 3 + 100);
      return_to_listening();
      return true;

    case CMD_ACTION_3:
      // SAY_GOODBYE — sad chirp + back away
      transition_to(STATE_EXECUTING);
      say(CLIP_SAD);
      att = {0.2f, -0.2f, 0.0f};
      gary.move(&vel, &pos, &att, 500);
      gary_delay(600);
      att = {-0.2f, -0.2f, 0.0f};
      gary.move(&vel, &pos, &att, 500);
      gary_delay(600);
      att = {0.0f, 0.0f, 0.0f};
      vel = {0.0f, -WALK_SPEED * 0.8f, 0.0f};
      gary.move(&vel, &pos, &att, STEP_TIME_MS, 2);
      gary_delay(STEP_TIME_MS * 2 + 100);
      return_to_listening();
      return true;

    case CMD_ACTION_4:
      // SHOW DISTANCE — read ultrasonic sensor and display range as a bar graph
      transition_to(STATE_EXECUTING);
      say(CLIP_WHISTLE);
      {
        // Show distance for ~3 seconds, refreshing every 200ms
        uint32_t t0 = millis();
        while (millis() - t0 < 3000) {
          uint16_t cm = gary.sensor.get_distance();
          anim_distance(cm);
          gary_delay(200);
        }
      }
      return_to_listening();
      return true;

    case CMD_ACTION_5:
      transition_to(STATE_EXECUTING);
      say(CLIP_PROUD);
      for (int i = 0; i < 2; i++) {
        pos = {0.0f, 0.0f, 1.9f};
        gary.move(&vel, &pos, &att, 250);
        gary_delay(280);
        pos = {0.0f, 0.0f, 1.5f};
        gary.move(&vel, &pos, &att, 250);
        gary_delay(280);
      }
      return_to_listening();
      return true;

    case CMD_ACTION_6:
      transition_to(STATE_EXECUTING);
      say(CLIP_PLAYFUL);
      pos = {0.0f, 0.0f, 2.2f};
      att = {0.0f, -0.3f, 0.0f};
      gary.move(&vel, &pos, &att, 500);
      gary_delay(1500);
      att = {0.0f, 0.0f, 0.0f};
      pos = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 500);
      gary_delay(600);
      return_to_listening();
      return true;

    case CMD_ACTION_7:
      transition_to(STATE_EXECUTING);
      say(CLIP_LAUGHTER);
      vel = {0.0f, WALK_SPEED * 0.5f, 0.0f};
      pos = {0.0f, 0.0f, 1.2f};
      gary.move(&vel, &pos, &att, STEP_TIME_MS, 1);
      gary_delay(STEP_TIME_MS + 100);
      for (int i = 0; i < 3; i++) {
        att = {0.15f, 0.0f, 0.0f};
        gary.move(&vel, &pos, &att, 250);
        gary_delay(280);
        att = {-0.15f, 0.0f, 0.0f};
        gary.move(&vel, &pos, &att, 250);
        gary_delay(280);
      }
      att = {0.0f, 0.0f, 0.0f};
      pos = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 400);
      gary_delay(500);
      return_to_listening();
      return true;

    case CMD_ACTION_8:
      // SIC_EM — determined growl + aggressive crouch + lunge
      transition_to(STATE_ALARMED);
      say(CLIP_DETERMINED);
      pos = {0.0f, 0.0f, 1.0f};
      gary.move(&vel, &pos, &att, 300);
      gary_delay(350);
      vel = {0.0f, WALK_SPEED * 2.0f, 0.0f};
      gary.move(&vel, &pos, &att, 600, 2);
      gary_delay(1300);
      pos = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 400);
      gary_delay(500);
      return_to_listening();
      return true;

    case CMD_ACTION_9:
      // CHARGE — very excited yelp + sustained sprint
      transition_to(STATE_ALARMED);
      say(CLIP_VERY_EXCITED);
      vel = {0.0f, WALK_SPEED * 2.0f, 0.0f};
      gary.move(&vel, &pos, &att, 800, 3);
      gary_delay(800 * 3 + 100);
      return_to_listening();
      return true;

    case CMD_ACTION_10:
      stop_motion();
      say(CLIP_CONCERNED);
      gary_delay(400);
      vel = {0.0f, 0.0f, TURN_SPEED};
      gary.move(&vel, &pos, &att, DISMISSAL_TURN_MS, 1);
      gary_delay(DISMISSAL_TURN_MS + 200);
      // neutral stance before the handoff -- do not roam with this
      // action's leftover attitude (bench 2026-08-23)
      vel = {0.0f, 0.0f, 0.0f};
      pos = {0.0f, 0.0f, 0.0f};
      att = {0.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 400, 1);
      gary_delay(500);
      transition_to(STATE_ROAMING);
      return true;

    case CMD_ACTION_11: // COME HERE
#if ENABLE_CAMERA
      transition_to(STATE_COMING);
#else
      say(CLIP_WHISTLING_EXCITED);
      exec_walk(WALK_SPEED * 1.2f, STEP_TIME_MS, 3);
#endif
      return true;

    case CMD_ACTION_12:
      // SHOW BATTERY — read battery voltage and display as a fuel gauge
      transition_to(STATE_EXECUTING);
      say(CLIP_WHISTLE);
      {
        // Quick voltage read (Hiwonder library: gary.board.bat_voltage)
        gary.board.bat_voltage_update();
        uint16_t mv = gary.board.bat_voltage;
        last_battery_mv = mv;
        anim_battery(mv);
        gary_delay(3000);  // hold display 3 seconds
      }
      return_to_listening();
      return true;

    case CMD_ACTION_13:
      // SENSE AREA — hold still and capture WiFi CSI for CSI_CAPTURE_DURATION_MS.
      // loop_sensing() handles the timeout and returns to STATE_IDLE on its own.
      transition_to(STATE_SENSING);
      return true;

    case CMD_ACTION_14:
      // NOD SCAN — sweep body pitch+roll, capturing a tagged LiDAR slice at
      // each step to build a 3D point cloud. Runs synchronously (same
      // pattern as CMD_ACTION_12's battery display) -- takes ~40s now that
      // it covers roll as well as pitch (see NODSCAN_ROLL_* below).
      transition_to(STATE_NODSCAN);
      run_nod_scan();
      transition_to(STATE_IDLE);
      return true;

    case CMD_ACTION_15:
      // TOGGLE SCANNING MODE — "execute action fifteen" starts or stops
      // Gary's autonomous watch: background WiFi CSI presence monitoring
      // that automatically fires a NODSCAN whenever it trips, then resumes
      // watching. Say it once to start, say it again (or say "stop") to
      // turn it off and return to IDLE. See STATE_AUTOSCAN / loop_autoscan().
      if (current_state == STATE_AUTOSCAN) {
        say(CLIP_SAD);
        Serial.println(">>> AUTOSCAN: scanning mode OFF (voice toggle)");
        transition_to(STATE_IDLE);
      } else {
        transition_to(STATE_AUTOSCAN);
      }
      return true;

    case CMD_ACTION_16:
      // EXPLORE -- "execute action sixteen": autonomous roam using the reflex
      // stack trained in the MuJoCo twin (2026-08-19 graduation policy).
      // Say "stop" to end it. See loop_explore() / gary_config.h EXPLORE block.
      if (current_state == STATE_EXPLORE) {
        Serial.println(">>> EXPLORE: off (voice toggle)");
        transition_to(STATE_IDLE);
      } else {
        explore_begin();
        transition_to(STATE_EXPLORE);
      }
      return true;

    // ====================================================================
    //  ADDITIONAL FACTORY COMMANDS — body manipulation / lean / pose
    // ====================================================================

    case CMD_LEAN_FWD:
      // Pitch forward, hold, return
      transition_to(STATE_EXECUTING);
      say(CLIP_WHISTLE);
      att = {30.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 600);
      gary_delay(800);
      att = {0.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 500);
      gary_delay(600);
      return_to_listening();
      return true;

    case CMD_LEAN_BACK:
      transition_to(STATE_EXECUTING);
      say(CLIP_WHISTLE);
      att = {-300.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 600);
      gary_delay(800);
      att = {0.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 500);
      gary_delay(600);
      return_to_listening();
      return true;

    case CMD_LEAN_LEFT:
      transition_to(STATE_EXECUTING);
      say(CLIP_WHISTLE);
      att = {60.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 600);
      gary_delay(800);
      att = {0.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 500);
      gary_delay(600);
      return_to_listening();
      return true;

    case CMD_LEAN_RIGHT:
      transition_to(STATE_EXECUTING);
      say(CLIP_WHISTLE);
      att = {-60.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 600);
      gary_delay(800);
      att = {0.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 500);
      gary_delay(600);
      return_to_listening();
      return true;

    case CMD_GET_DOWN:
      // Lower further than sit — flatten body close to ground
      exec_pose({0.0f, 0.0f, 0.2f}, 1500);
      return true;

    case CMD_LOWER_BODY:
      // Crouch slightly — opposite of RAISE_BODY
      exec_pose({0.0f, 0.0f, 0.5f}, 1000);
      return true;

    case CMD_MAKE_SOUND:
      // Just chirp playfully and stay listening
      say(CLIP_PLAYFUL);
      return_to_listening();
      return true;

    case CMD_DIVE_FWD:
      // Lower + lurch forward like a dive
      transition_to(STATE_ALARMED);
      say(CLIP_DETERMINED);
      pos = {0.0f, 0.0f, 0.8f};
      att = {25.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 500);
      gary_delay(600);
      vel = {0.0f, WALK_SPEED * 4.0f, -5.0f};
      gary.move(&vel, &pos, &att, 600, 2);
      gary_delay(620);
      pos = {0.0f, 0.0f, 2.0f};
      gary.move(&vel, &pos, &att, 500);
      gary_delay(550);
      return_to_listening();
      return true;

    case CMD_DIVE_BACK:
      transition_to(STATE_ALARMED);
      say(CLIP_CONCERNED);
      pos = {0.0f, 0.0f, 0.8f};
      gary.move(&vel, &pos, &att, 150);
      gary_delay(300);
      vel = {0.0f, -WALK_SPEED * 3.0f, -2.0f};
      gary.move(&vel, &pos, &att, 600, 2);
      gary_delay(400);
      pos = {0.0f, 0.0f, 0.8f};
      gary.move(&vel, &pos, &att, 500);
      gary_delay(550);
      return_to_listening();
      return true;

    case CMD_SQUAT:
      // Lower than sit, hold longer
      att = {25.0f, 0.0f, 0.0f}; 
      exec_pose({0.0f, 0.0f, 0.4f},1800);
      return true;

    // ====================================================================
    //  MORE FACTORY ANIMATIONS
    // ====================================================================

    case CMD_PUSH_UP:
      // Body pumps up and down repeatedly
      transition_to(STATE_EXECUTING);
      say(CLIP_DETERMINED);
      for (int i = 0; i < 6; i++) {
        pos = {0.0f, 0.0f, 0.5f};
        att = {20.0f, 0.0f, 0.0f};
        gary.move(&vel, &pos, &att, 350);
        gary_delay(400);
        pos = {0.0f, 0.0f, 20.0f};
        att = {-20.0f, 0.0f, 0.0f};
        gary.move(&vel, &pos, &att, 350);
        gary_delay(400);
      }
      pos = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 800);
      gary_delay(900);
      return_to_listening();
      return true;

    case CMD_TWIST:
      // Yaw body left-right rapidly without translating
      transition_to(STATE_EXECUTING);
      say(CLIP_PLAYFUL);
      for (int i = 0; i < 3; i++) {
        att = {0.0f, 0.0f, 40.0f};
        gary.move(&vel, &pos, &att, 250);
        gary_delay(280);
        att = {0.0f, 0.0f, -40.0f};
        gary.move(&vel, &pos, &att, 250);
        gary_delay(280);
      }
      att = {0.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 200);
      gary_delay(220);
      return_to_listening();
      return true;

    case CMD_SIT_UP:
      // Rise from a sit — opposite of sit_down
      exec_pose({0.0f, 0.0f, 2.0f}, 800);
      return true;

    case CMD_SPREAD:
      // Hexapod can't really spread "hands" — interpret as wide stance pose:
      // pitch back + elevated body to look proud / stretched
      transition_to(STATE_EXECUTING);
      say(CLIP_PROUD);
      pos = {0.0f, 0.0f, 10.0f};
      att = {-45.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 700);
      gary_delay(1500);
      att = {0.0f, 0.0f, 0.0f};
      pos = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 500);
      gary_delay(600);
      return_to_listening();
      return true;

    case CMD_FALL_STAND:
      // Fall flat, pause, then rise dramatically
      transition_to(STATE_EXECUTING);
      say(CLIP_SHOCKED);
      pos = {0.0f, 0.0f, 0.2f};
      gary.move(&vel, &pos, &att, 50);
      gary_delay(900);
      gary_delay(800);  // lie there
      say(CLIP_DETERMINED);
      pos = {0.0f, 0.0f, 1.5f};
      att = {-15.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 1000);
      gary_delay(1100);
      return_to_listening();
      return true;

    case CMD_LEFT_HOOK:
      // Quick body lunge to the left + lean right (like throwing a hook)
      transition_to(STATE_ALARMED);
      say(CLIP_DETERMINED);
      att = {-10.0f, 30.0f, 20.0f};   // roll left + slight yaw
      gary.move(&vel, &pos, &att, 250);
      gary_delay(300);
      att = {-10.0f, -30.0f, -20.0f}; // recover with opposite roll
      gary.move(&vel, &pos, &att, 250);
      gary_delay(300);
      att = {10.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 300);
      gary_delay(350);
      return_to_listening();
      return true;

    case CMD_RIGHT_HOOK:
      transition_to(STATE_ALARMED);
      say(CLIP_DETERMINED);
      att = {-10.0f, -30.0f, -20.0f};
      gary.move(&vel, &pos, &att, 250);
      gary_delay(300);
      att = {-10.0f, 30.0f, 20.0f};
      gary.move(&vel, &pos, &att, 250);
      gary_delay(300);
      att = {10.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 300);
      gary_delay(350);
      return_to_listening();
      return true;

    default:
      say(CLIP_UNSURE);
      return false;
  }
}

void return_to_listening() {
  // BENCH 2026-08-23: the push-up left att roll -15 deg in place and he
  // sat tilted for three minutes of later commands. Every action ends in
  // the neutral stance.
  vel = {0.0f, 0.0f, 0.0f};
  pos = {0.0f, 0.0f, 1.5f};   // canonical standing height (0 left him low -- bench 2026-08-24)
  att = {0.0f, 0.0f, 0.0f};
  gary.move(&vel, &pos, &att, 400, 1);
  listening_got_command = true;
  state_entered_ms = millis();
  current_state = STATE_LISTENING;
  set_eyes(EYE_LISTENING);
}

// =============================================================================
// BACKGROUND CHECKS
// =============================================================================

bool run_background_checks() {

#if ENABLE_BATTERY_MONITOR
  static uint32_t last_battery_check = 0;
  static uint8_t consecutive_low_readings = 0;

  if (millis() - last_battery_check > 5000) {
    gary.board.bat_voltage_update();
    uint16_t reading = gary.board.bat_voltage;
    last_battery_check = millis();

    const uint16_t SANITY_FLOOR_MV = 5000;

    if (reading < SANITY_FLOOR_MV) {
#if ENABLE_DEBUG_SERIAL
      Serial.print(">>> BATTERY: ignoring bad sample = ");
      Serial.println(reading);
#endif
    } else {
      last_battery_mv = reading;
#if ENABLE_DEBUG_SERIAL
      Serial.print(">>> BATTERY: reading = ");
      Serial.print(reading);
      Serial.print("mV  consecutive_low=");
      Serial.println(consecutive_low_readings);
#endif

      if (last_battery_mv < BATTERY_LOW_MV) {
        consecutive_low_readings++;
        if (consecutive_low_readings >= 3 && current_state != STATE_ALARMED) {
          transition_to(STATE_ALARMED);
          return true;
        }
      } else {
        consecutive_low_readings = 0;
      }
    }
  }
#endif

  // Tip detection — with sanity filtering.
  //
  // Why filter: the IMU on the I2C bus has been observed returning wildly
  // implausible values (roll swinging ±150° while Gary is sitting still),
  // probably from voltage sag during high servo current draw. A single bad
  // sample shouldn't trip alarm; we require 3 consecutive over-threshold
  // readings AND we discard "implausible" samples (>175° roll or pitch,
  // which are physically impossible for an upright hexapod).
  float euler_data[3];
  gary.board.get_imu_euler(euler_data);
  float roll  = euler_data[0];
  float pitch = euler_data[1];

  static uint8_t consecutive_tip_readings = 0;
  const float IMU_INSANITY_DEG = 175.0f;  // beyond this = bad sample, ignore

  if (abs(roll) > IMU_INSANITY_DEG || abs(pitch) > IMU_INSANITY_DEG) {
    // Implausible reading — IMU glitched. Don't count it, don't reset counter.
#if ENABLE_DEBUG_SERIAL
    Serial.print(">>> IMU: ignoring bad sample roll=");
    Serial.print(roll);
    Serial.print(" pitch=");
    Serial.println(pitch);
#endif
  } else if (abs(roll) > IMU_TIP_THRESHOLD_DEG || abs(pitch) > IMU_TIP_THRESHOLD_DEG) {
    float _tg[3];
    gary.board.get_imu_gyro(_tg);
    float _gm = fabsf(_tg[0]) + fabsf(_tg[1]) + fabsf(_tg[2]);
    if (_gm < IMU_TIP_GYRO_QUIET_DPS) {
      consecutive_tip_readings++;   // extreme AND still = actually down
    }                               // gyro busy = being handled: hold the count
    if (consecutive_tip_readings >= IMU_TIP_CONSEC && current_state != STATE_ALARMED) {
#if ENABLE_DEBUG_SERIAL
      Serial.print("TIP DETECTED (3 consecutive): roll=");
      Serial.print(roll);
      Serial.print(" pitch=");
      Serial.println(pitch);
#endif
      transition_to(STATE_ALARMED);
      consecutive_tip_readings = 0;
      return true;
    }
  } else {
    consecutive_tip_readings = 0;
  }

#if ENABLE_DEBUG_SERIAL
  static uint32_t last_sensor_debug = 0;
  if (millis() - last_sensor_debug > 2000) {
    uint16_t debug_sound = gary.board.get_sound_val();
    uint16_t debug_dist = gary.sensor.get_distance();
    float _g[3] = {0, 0, 0};
    gary.board.get_imu_gyro(_g);
    Serial.printf(">>> GYRO | x=%.2f y=%.2f z=%.2f (z is yaw rate; MUST be nonzero when turning)\n",
                  _g[0], _g[1], _g[2]);
    Serial.print(">>> GARY SENSORS | sound=");
    Serial.print(debug_sound);
    Serial.print(" dist=");
    Serial.print(debug_dist);
    Serial.print("mm roll=");
    Serial.print(roll, 1);
    Serial.print(" pitch=");
    Serial.println(pitch, 1);
    last_sensor_debug = millis();
  }
#endif

#if ENABLE_SOUND_STARTLE
  static volatile uint8_t startle_enabled = ENABLE_SOUND_STARTLE;
  /* Skip startle checks while under controller mode so brief audio spikes
     or PU relay activity doesn't yank control away from the user. */
  if (startle_enabled && current_state != STATE_CONTROLLER) {
    uint16_t sound_value = gary.board.get_sound_val();
    int delta = (int)sound_value - (int)last_sound_value;

    if (sound_value >= SOUND_STARTLE_THRESHOLD
        && delta >= SOUND_STARTLE_DELTA
        && current_state != STATE_STARTLED
        && current_state != STATE_ALARMED
        && millis() > startle_suppress_until_ms  // ...nor his own WonderEcho voice/ack
        && !gary_sounds_is_playing()) {  // Don't startle from our own buzzer
      transition_to(STATE_STARTLED);
      last_sound_value = sound_value;
      return true;
    }
    last_sound_value = sound_value;
  }
#endif

  return false;
}

// =============================================================================
// PER-STATE LOOP HANDLERS
// =============================================================================

void loop_idle() {
#if ENABLE_BALANCE
  gary.balance(true);
#endif

#if ENABLE_VOICE
  uint8_t cmd = asr_poll();
  if (cmd != CMD_NONE) {
    transition_to(STATE_LISTENING);
    handle_voice_command(cmd);
    return;
  }
#endif

  // Stay-put mode: don't drift into ROAMING after a timeout.
  // To re-enable autonomous roaming, uncomment the block below:
  // if (millis() - state_entered_ms > IDLE_TO_ROAM_MS) {
  //   transition_to(STATE_ROAMING);
  // }
}

void loop_listening() {
#if ENABLE_VOICE
  uint8_t cmd = asr_poll();
  if (cmd != CMD_NONE) {
    handle_voice_command(cmd);
    return;
  }
#endif

  uint32_t timeout = listening_got_command
                     ? LISTENING_TIMEOUT_MS
                     : LISTENING_FIRST_TIMEOUT_MS;

  if (millis() - state_entered_ms > timeout) {
    say(CLIP_UNSURE);
    transition_to(STATE_IDLE);  // Stay put after silence (was STATE_ROAMING)
  }
}

void loop_executing() {
  transition_to(STATE_IDLE);
}

void loop_roaming() {
#if ENABLE_VOICE
  uint8_t cmd = asr_poll();
  if (cmd != CMD_NONE) {
    transition_to(STATE_LISTENING);
    handle_voice_command(cmd);
    return;
  }
#endif

  uint16_t dis = gary.sensor.get_distance();
  gary.avoid(dis);

#if ENABLE_BOREDOM_TIMER
  if (millis() - last_boredom_kick_ms > ROAM_BOREDOM_MS) {
    last_boredom_kick_ms = millis();
    float turn = ((float)(random(90) - 45)) * 0.02f;
    vel = {0.0f, 0.0f, turn};
    gary.move(&vel, &pos, &att, 500, 1);
    if (random(4) == 0) {
      say(random(2) ? CLIP_WHISTLE : CLIP_PLAYFUL);
    }
  }
#endif

  uint32_t cycle = (millis() / 100) % 60;
  if (cycle == 0)  set_eyes(EYE_ROAMING_A);
  if (cycle == 30) set_eyes(EYE_ROAMING_B);

  if (millis() - state_entered_ms > ROAM_TIMEOUT_MS) {
    transition_to(STATE_IDLE);
  }
}

void loop_coming() {
#if !ENABLE_CAMERA
  say(CLIP_UNSURE);
  transition_to(STATE_IDLE);
  return;
#endif

#if ENABLE_CAMERA
  switch (coming_phase) {

    case COMING_SEARCH: {
      gary.sensor.camera.face_data_receive(camera_buf, sizeof(camera_buf));

      if (camera_buf[0] != 0) {
        coming_phase = COMING_APPROACH;
        say(CLIP_WHISTLE_INQUISITIVE);
        return;
      }

      vel = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 300, 1);

      if (millis() - state_entered_ms > COMING_SEARCH_TIMEOUT_MS) {
        say(CLIP_SAD);
        transition_to(STATE_IDLE);
      }
      break;
    }

    case COMING_APPROACH: {
      gary.sensor.camera.face_data_receive(camera_buf, sizeof(camera_buf));

      if (camera_buf[0] == 0) {
        coming_phase = COMING_SEARCH;
        return;
      }

      uint8_t face_x = camera_buf[0];
      uint8_t face_w = camera_buf[2];
      float yaw_offset = ((float)face_x - 80.0f) / 80.0f * 0.5f;

      uint16_t dis = gary.sensor.get_distance();

      if (face_w >= FACE_SIZE_CLOSE || dis < DISTANCE_ARRIVED_MM) {
        coming_phase = COMING_ARRIVED;
        return;
      }

      vel = {0.0f, WALK_SPEED * 0.7f, yaw_offset};
      gary.move(&vel, &pos, &att, 400, 1);
      break;
    }

    case COMING_ARRIVED: {
      stop_motion();
      set_eyes(EYE_PROUD);
      gary_introduce_routine();   // proud "presented myself" routine, no library buzzer
      return_to_listening();
      break;
    }
  }

  uint8_t cmd = asr_poll();
  if (cmd == CMD_STOP) {
    transition_to(STATE_IDLE);
  }
#endif
}

void loop_startled() {
  if (millis() - state_entered_ms > STARTLED_DURATION_MS) {
    if (previous_state == STATE_LISTENING) {
      transition_to(STATE_LISTENING);
    } else {
      transition_to(STATE_IDLE);  // Stay-put: was STATE_ROAMING
    }
  }
}

void loop_sensing() {
  csi_pump();
  if (millis() - state_entered_ms > CSI_CAPTURE_DURATION_MS) {
    csi_capture_end();
    transition_to(STATE_IDLE);
  }
}

void loop_autoscan() {
#if ENABLE_VOICE
  uint8_t cmd = asr_poll();
  if (cmd != CMD_NONE) {
    // CMD_ACTION_15 toggles back off; CMD_STOP also works (falls through to
    // the generic "stop whatever you're doing" handling already in
    // handle_voice_command(), which calls transition_to() and gets the
    // AUTOSCAN-exit CSI cleanup for free).
    handle_voice_command(cmd);
    return;
  }
#endif

  switch (autoscan_phase) {
    case AUTOSCAN_WARMUP:
    case AUTOSCAN_MONITOR: {
      csi_pump_autoscan_status();
      autoscan_csi_drain();

      if (autoscan_phase == AUTOSCAN_WARMUP && autoscan_baseline_n >= AUTOSCAN_MIN_BASELINE_SAMPLES) {
        autoscan_phase = AUTOSCAN_MONITOR;
        Serial.println(">>> AUTOSCAN: baseline established, now watching");
      }

      if (autoscan_phase == AUTOSCAN_MONITOR && autoscan_check_trip()) {
        autoscan_triggered_count++;
        say(CLIP_WHISTLE_INQUISITIVE);
        csi_capture_end();   // stop CSI -- NODSCAN needs to move, CSI needs stillness
        // These bracketing lines are also broadcast over WiFi (not just
        // Serial) -- the live dashboard uses them to know which NOD: lines
        // in between came from a CSI-triggered scan vs. a manual one, and
        // colors that patch of the point cloud differently.
        {
          static char trig_buf[64];
          snprintf(trig_buf, sizeof(trig_buf), ">>> AUTOSCAN: triggered scan #%lu starting\n", (unsigned long)autoscan_triggered_count);
          Serial.print(trig_buf);
          wifi_scan_broadcast(trig_buf);
        }
        run_nod_scan();      // blocking, ~40s now that it covers roll too; tags its
                              // NOD: lines the same as a manual scan -- nothing in the
                              // point-cloud format marks this one as "triggered" beyond
                              // the AUTOSCAN markers bracketing it, which is enough for
                              // the Python side to line them up by timestamp
        {
          static const char resume_buf[] = ">>> AUTOSCAN: triggered scan complete, resuming watch\n";
          Serial.print(resume_buf);
          wifi_scan_broadcast(resume_buf);
        }
        csi_capture_begin(); // resume passive CSI
        autoscan_recent_count = 0;   // don't re-trip off stale pre-scan samples
        autoscan_recent_idx = 0;
        autoscan_cooldown_until_ms = millis() + AUTOSCAN_COOLDOWN_MS;
        autoscan_phase = AUTOSCAN_COOLDOWN;
      }
      break;
    }

    case AUTOSCAN_COOLDOWN: {
      autoscan_csi_drain();   // keep baseline fresh, but don't evaluate trips yet
      if (millis() > autoscan_cooldown_until_ms) {
        autoscan_phase = AUTOSCAN_MONITOR;
      }
      break;
    }
  }
}

void loop_alarmed() {
  float euler_data[3];
  gary.board.get_imu_euler(euler_data);
  float roll  = euler_data[0];
  float pitch = euler_data[1];

#if ENABLE_BATTERY_MONITOR
  bool battery_ok = (last_battery_mv >= BATTERY_LOW_MV + 200);
#else
  bool battery_ok = true;
#endif

  // Require 3 consecutive level samples to recover, with insanity filter.
  // Prevents recovery from getting stuck if IMU glitches one direction or the
  // other while Gary is actually upright again.
  const float IMU_INSANITY_DEG = 175.0f;
  bool insane = (abs(roll) > IMU_INSANITY_DEG || abs(pitch) > IMU_INSANITY_DEG);
  bool level_now = (!insane
                   && abs(roll) < IMU_TIP_THRESHOLD_DEG / 2
                   && abs(pitch) < IMU_TIP_THRESHOLD_DEG / 2);

  static uint8_t consecutive_level_readings = 0;
  if (level_now) {
    consecutive_level_readings++;
  } else if (!insane) {
    // Real not-level reading resets the counter. Insane samples are ignored.
    consecutive_level_readings = 0;
  }

  if (consecutive_level_readings >= 3 && battery_ok) {
    consecutive_level_readings = 0;
    transition_to(STATE_IDLE);
  }

  static uint32_t last_chirp = 0;
  if (millis() - last_chirp > 5000) {
    say(CLIP_SAD);
    last_chirp = millis();
  }
}

// =============================================================================
// SETUP / LOOP
// =============================================================================

void setup() {
  Serial.begin(GARY_SERIAL_BAUD);
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  pu_ble_init();
  Serial.println("[CTRL] Bluepad32 ready - press PS button to pair DualSense");
  delay(500);


#if ENABLE_DEBUG_SERIAL
  Serial.println();
  Serial.println("=== GARY v1 BOOTING ===");
  Serial.print(">>> COMPILED FLAGS: SOUND_STARTLE=");
  Serial.print(ENABLE_SOUND_STARTLE);
  Serial.print(" BATTERY=");
  Serial.print(ENABLE_BATTERY_MONITOR);
  Serial.print(" VOICE=");
  Serial.println(ENABLE_VOICE);
  Serial.println(">>> Voice: factory firmware (CI1302 V00729)");
  Serial.println(">>> Wake: \"HELLO-HI-WONDER\" or \"Hey Gary\" -> \"I'm here\"");
  Serial.println(">>> Sounds: NON-BLOCKING (motion + buzzer simultaneous)");
#endif

  randomSeed(analogRead(34));

  gary.begin();
  delay(1000);

    // MAARTAS SOUND SWITCH // 

  ledcWriteTone(LEDC_CHANNEL_0, 0);
  ledcDetachPin(BUZZER_PIN);   // disconnect pin from PWM
  pinMode(BUZZER_PIN, INPUT);  // float the pin so nothing can drive it

  gary_sounds_begin();
  gary_face_begin();
  gary_lidar_begin();
  gary_lidar_start_task();   // LiDAR now reads itself; see Gary_lidar.cpp

  // ---- TURN THE GYRO ON. It has been OFF this whole time. --------------------
  // HW_Board::begin() creates the IMU timer and then does NOT start it:
  //     imu_timer = xTimerCreate("imu_update_timer", ...);
  //     // xTimerStart(imu_timer, 0);      <-- commented out in the vendor lib
  // The ONLY caller of imu_update(true) is Robot::balance(true), so unless you
  // are in balance mode the timer never fires, _imu_update() never runs, and
  // gyro_cali / euler stay {0,0,0} forever.
  //
  // That is why every SCAN frame reported theta=0.0 while Gary spun around the
  // room, and why the map "warped based on where the front of Gary pointed":
  // with heading frozen at zero, every ring gets stamped into the world as if he
  // were facing north, so the ROOM rotates around HIM instead of the other way
  // round. It is also why roll/pitch printed as exactly 0.0 while the raw
  // accelerometer values in [CTRL] status were visibly changing.
  //
  // The gyro was never noisy or miscalibrated. It was switched off.
  gary.board.imu_update(true);   // starts the 100Hz IMU timer
  face_scroll_text("GARY");  // boot greeting

#if ENABLE_DEBUG_SERIAL
  Serial.println(">>> GARY: standing up");
#endif
  Velocity_t stand_vel = {0.0f, 0.0f, 0.0f};
  Vector_t stand_pos = {0.0f, 0.0f, 1.5f};
  Euler_t stand_att = {0.0f, 0.0f, 0.0f};
  gary.move(&stand_vel, &stand_pos, &stand_att, 1000);
  delay(1500);

#if ENABLE_DEBUG_SERIAL
  // ---- Startup joint diagnostic ----
  // Reads the deviation table (per-joint calibration trim) and the runtime
  // state of all 18 joints. If one joint is mechanically wrong at boot, look
  // for an outlier in either:
  //   - dev: deviation way out of family with same-joint-letter peers
  //   - duty: PWM commanded substantially different from peers
  //   - angle: commanded angle different from peers
  // Same-letter joints across legs should land within a few units of each
  // other after the symmetric standup pose.
  {
    int8_t devs[18] = {0};
    gary.read_deviation(devs);

    RobotLeg* legs[6] = {&gary.leg1, &gary.leg2, &gary.leg3,
                         &gary.leg4, &gary.leg5, &gary.leg6};
    const char* leg_names[6] = {"FR", "MR", "BR", "BL", "ML", "FL"};

    Serial.println();
    Serial.println(">>> JOINT DIAGNOSTIC (after standup, before any motion)");
    Serial.println("Leg     | A (hip)              | B (femur)            | C (tibia)");
    Serial.println("--------+----------------------+----------------------+----------------------");
    for (int i = 0; i < 6; i++) {
      RobotJoint& ja = legs[i]->joint_a;
      RobotJoint& jb = legs[i]->joint_b;
      RobotJoint& jc = legs[i]->joint_c;
      Serial.printf("Leg%d %s | id=%02u dev=%+3d ang=%+5.1f | id=%02u dev=%+3d ang=%+5.1f | id=%02u dev=%+3d ang=%+5.1f\n",
        i + 1, leg_names[i],
        (unsigned)ja.id, (int)ja.deviation, ja.angle,
        (unsigned)jb.id, (int)jb.deviation, jb.angle,
        (unsigned)jc.id, (int)jc.deviation, jc.angle);
    }

    // Also dump the raw devs[] array in case its order differs from what we
    // think (the library might index it by servo id rather than by leg).
    Serial.print(">>> Raw deviations array (18 entries): ");
    for (int i = 0; i < 18; i++) {
      Serial.printf("%+d ", (int)devs[i]);
    }
    Serial.println();

    // Per-column min/max of deviation, so outliers in any one column pop out.
    int dev_a_min = 127, dev_a_max = -128;
    int dev_b_min = 127, dev_b_max = -128;
    int dev_c_min = 127, dev_c_max = -128;
    for (int i = 0; i < 6; i++) {
      int da = legs[i]->joint_a.deviation;
      int db = legs[i]->joint_b.deviation;
      int dc = legs[i]->joint_c.deviation;
      if (da < dev_a_min) dev_a_min = da;
      if (da > dev_a_max) dev_a_max = da;
      if (db < dev_b_min) dev_b_min = db;
      if (db > dev_b_max) dev_b_max = db;
      if (dc < dev_c_min) dev_c_min = dc;
      if (dc > dev_c_max) dev_c_max = dc;
    }
    Serial.printf(">>> Per-joint-letter dev spread: A=[%+d..%+d]  B=[%+d..%+d]  C=[%+d..%+d]\n",
      dev_a_min, dev_a_max, dev_b_min, dev_b_max, dev_c_min, dev_c_max);
    Serial.println();
  }
#endif

  set_eyes(EYE_BOOT);
  delay(500);

  // Boot into IDLE — Gary stays put until commanded.
  // To revert to autonomous roaming, change to STATE_ROAMING and re-enable
  // the IDLE_TO_ROAM_MS timeout in loop_idle().
  transition_to(STATE_IDLE);

#if ENABLE_DEBUG_SERIAL
  Serial.println("=== GARY v1 READY ===");
  Serial.printf(">>> Serial: %d baud (set your monitor/Python to match)\n", GARY_SERIAL_BAUD);
  Serial.printf(">>> Scanner mode: %s -- touchpad click, or 's' here, to toggle\n",
                scanner_mode ? "ON" : "OFF (no SCAN: stream; voice/joystick get full loop)");
#endif

  wifi_begin();
}


// =============================================================================
// STATE_EXPLORE — the sim-trained reflex stack (2026-08-19 graduation policy),
// ported to the REAL sensor suite. See sim/train_course_sensor.py and
// notes/STATUS-2026-08-18.md for how each reflex was earned.
//
// Sensor mapping (sim -> hardware):
//   climb trigger   sonar shortfall (down-tilted)   ->  forward sonar < EXP_CLIMB_TRIG_MM.
//                   NOTE: the forward sonar CANNOT see obstacles below its own
//                   height (~40 mm). September mod: a ~10 deg down-tilt shim on
//                   the sonar mount ports the sim's shortfall/overshoot (cliff!)
//                   math verbatim -- see EXPLORE_SONAR_TILTED.
//   roof vs wall    nod rays vs ring                ->  D500 front arc blocked
//                   + sonar CLEAR  = something at mast height only -> CROUCH.
//                   front arc + sonar BOTH blocked  = wall -> retreat + pivot.
//   climb persist   IMU pitch                       ->  same IMU, verbatim.
//   crouch release  distance traveled               ->  commanded-speed integration.
//   heading hold    sim yaw                         ->  IMU-integrated yaw is NOT
//                   ported yet (gyro yaw scale never calibrated, see odometry
//                   notes) -- v1 holds heading open-loop and relies on the
//                   retreat/pivot reflexes. Bench item.
//
// SAFETY: knee guard is global and armed. All speeds ship derated by
// EXP_DERATE. First live run: doorway + one obstacle, eyes on him.
// =============================================================================

static uint8_t  exp_mode = 0;                   // 0 cruise, 1 climb, 2 crouch
static uint32_t exp_last_tick_ms = 0;
static uint32_t exp_climb_until_ms = 0;
static uint32_t exp_crouch_until_ms = 0;
static uint32_t exp_retreat_until_ms = 0;
static uint32_t exp_turn_cool_until_ms = 0;
static uint32_t exp_pivot_until_ms = 0;
static int8_t   exp_pivot_dir = 1;
static uint32_t exp_stall_ref_ms = 0;
static uint16_t exp_stall_ref_sonar = 0;

void explore_begin() {
  exp_mode = 0;
  exp_last_tick_ms = millis();
  exp_climb_until_ms = exp_crouch_until_ms = 0;
  exp_retreat_until_ms = exp_pivot_until_ms = 0;
  exp_turn_cool_until_ms = millis() + 2000;     // no hazard turns in the first 2 s
  exp_stall_ref_ms = millis();
  exp_stall_ref_sonar = gary.sensor.get_distance();
  Serial.println(">>> EXPLORE: on -- reflex stack live (say 'stop' or press 'e' to end)");
}

// THE PUSHING DOCTRINE (Bill, watching the sim navigator 2026-08-22:
// "he should realize when he's pushing objects. as in, if he's moving,
// but its staying the same distance... that's associated with his
// odometry i figure." -- exactly right, and it ports to hardware
// unchanged because it needs only the forward sonar and leg odometry,
// both of which this robot already has and both of which are honest.)
//
//   range PINNED while odometry ACCUMULATES  = I am pushing a MOVABLE
//   clear bearings GROWING while pushing     = MY push is opening it
//   range OPENS PAST the threshold           = it is passable: go
//
// A wall does not move. A door does. Without this he treats a yielding
// door as a wall that will not go away, and oscillates in front of it.
static uint16_t exp_push_ref_mm = 0;      // sonar when the push began
static float    exp_push_ref_x = 0.0f;    // odometry when the push began
static float    exp_push_ref_y = 0.0f;
static uint32_t exp_push_until = 0;       // "I am pushing" latch
static uint32_t exp_push_start_ms = 0;    // when this push began (wall budget)
static uint8_t  exp_push_clear = 0;       // clear-bearing count at start

// THE WAG -- a real head shake: feet planted, body yawing side to side.
// This is what CMD_SHAKE_HEAD should have been doing all along.
void gary_head_shake(uint8_t shakes, float amp_rad, uint16_t period_ms) {
  uint32_t t0 = millis();
  uint32_t dur = (uint32_t)shakes * period_ms;
  while (millis() - t0 < dur) {
    float ph = (float)(millis() - t0) / (float)period_ms;
    gary_body_yaw_rad = amp_rad * sinf(2.0f * PI * ph);
    Velocity_t v = {0.0f, 0.0f, 0.0f};
    Vector_t   p = {0.0f, 0.0f, 1.5f};
    Euler_t    a = {0.0f, 0.0f, 0.0f};
    gary.move(&v, &p, &a, 60);
    gary.func_state = Robot::ACTION_GROUP;
    run_custom_gait_scheduler(GAIT_TRIPOD, 60, 0.0f, 0.0f, 0.0f);
    gary_delay(20);
  }
  gary_body_yaw_rad = 0.0f;
}

// THE DEFEAT PERFORMANCE (Bill's choreography, sim-certified in the
// breach drill): "look it up and down first, then admit defeat by
// lowering and shaking his head back-and-forth in shame. and definitely
// making the defeat noise." Then a short, dignified walk-off -- he does
// not look back.
void gary_defeat_performance() {
  Velocity_t v = {0.0f, 0.0f, 0.0f};
  Vector_t   p = {0.0f, 0.0f, 1.5f};
  Euler_t    a = {0.0f, 0.0f, 0.0f};
  Serial.println(">>> DEFEAT: looking it up and down");
  a = {0.0f, -17.0f, 0.0f};  gary.move(&v, &p, &a, 900);  gary_delay(1000);
  a = {0.0f,   8.0f, 0.0f};  gary.move(&v, &p, &a, 700);  gary_delay(800);
  a = {0.0f,   0.0f, 0.0f};  gary.move(&v, &p, &a, 500);  gary_delay(500);
  Serial.println(">>> DEFEAT: the sag, and the noise");
  p = {0.0f, -12.0f, 0.9f};  a = {0.0f, 7.0f, 0.0f};
  gary.move(&v, &p, &a, 700);
  gary_sad_descend();                       // THE defeat noise
  gary_delay(900);
  gary_head_shake(2, 0.30f, 550);           // quick -- Bill: "real
                                            // quick, and then move on"
  Serial.println(">>> DEFEAT: walking it off");
  p = {0.0f, 0.0f, 1.5f}; a = {0.0f, 0.0f, 0.0f};
  gary.move(&v, &p, &a, 600);
  gary_delay(600);
}

// ---- BREACH FSM state (Layer 1 door-working; constants in gary_config.h) ----
enum BreachPhase { BR_NONE = 0, BR_SQUARE, BR_SHOVE, BR_LOOK, BR_KNOCK, BR_SHAKE, BR_TURN, BR_KICK, BR_DONE };
static BreachPhase breach_ph = BR_NONE;
static uint32_t breach_t0 = 0, breach_cool_until = 0;
static uint16_t breach_ref_mm = 0, breach_best_mm = 0;
static int8_t   breach_turn_dir = 1;

void loop_explore() {
  uint32_t now = millis();
  if (now - exp_last_tick_ms < EXP_TICK_MS) return;
  float dt = (now - exp_last_tick_ms) * 0.001f;
  exp_last_tick_ms = now;

#if ENABLE_VOICE
  // He was DEAF while exploring (ASR 0.0 polls/sec -- 'say stop to end'
  // was a lie). Poll voice here too. Bill 2026-09-17: a command spoken while
  // exploring used to be announced-and-DROPPED -- the roam kept driving
  // underneath it, so the only way to run anything was to say "stop" first,
  // then the command. Now a command PREEMPTS the roam: pause, run it to
  // completion, then RESUME exploring on its own. Only "stop" (or "action
  // sixteen", the explicit explore toggle) ends explore; and if the command
  // itself switched Gary into another autonomous mode, that mode wins.
  {
    uint8_t vcmd = asr_poll();
    if (vcmd == CMD_STOP || vcmd == CMD_ACTION_16) {
      Serial.println(">>> EXPLORE: off (voice)");
      stop_motion();
      transition_to(STATE_IDLE);
      return;
    } else if (vcmd != CMD_NONE) {
      // PREEMPT. Stop the roam so the action isn't fighting the gait, then
      // run the spoken command synchronously. handle_voice_command() blocks
      // through the whole action and leaves Gary in LISTENING (one-shot
      // actions: walk/turn/dance/pose/battery) or IDLE (nod scan) -- OR in a
      // new persistent mode (roam/come/sense/autoscan) if that is what was
      // asked. This also fixes the nod-scan smear: action fourteen now runs
      // STATIONARY (we stop_motion first) instead of walking during the sweep.
      Serial.printf(">>> EXPLORE: preempt for cmd=%u -- run it, then resume roam\n", vcmd);
      stop_motion();
      handle_voice_command(vcmd);
      if (current_state == STATE_LISTENING || current_state == STATE_IDLE) {
        // transient one-shot -> back to exploring
        Serial.println(">>> EXPLORE: command done -- resuming roam");
        explore_begin();
        transition_to(STATE_EXPLORE);
      } else {
        Serial.printf(">>> EXPLORE: command switched to %s -- explore yields\n",
                      state_name(current_state));
      }
      return;
    }
  }
#endif

  // ---- senses -------------------------------------------------------------
  uint16_t sonar_raw = gary.sensor.get_distance();         // forward
  // BENCH 2026-08-24 (the oven): at point-blank range the sonar returns
  // 0 / 65535 (no echo). That is BLIND, not "clear at 5000mm" -- the old
  // junk guard turned nose-on-the-oven into open road.
  bool sonar_valid = (sonar_raw != 0 && sonar_raw <= 5000);
  uint16_t sonar_mm = sonar_valid ? sonar_raw : 5000;
  uint16_t ring_front = gary_lidar_min_distance(350, 10);  // 20 deg dead ahead
  uint16_t ring_wide  = gary_lidar_min_distance(330, 30);  // 60 deg forward arc
  uint16_t ring_left  = gary_lidar_min_distance(60, 120);  // frame chirality: VERIFY
  uint16_t ring_right = gary_lidar_min_distance(240, 300); //   on bench, swap if mirrored
  uint16_t ring_rear  = gary_lidar_min_distance(170, 190);
  if (!gary_lidar_is_ready()) { ring_front = ring_wide = ring_left = ring_right = ring_rear = 0xFFFF; }
  // LD19 "no data in the arc" comes back as 0 (absorbing/oblique surfaces).
  // 0 is INVALID, not a 0mm contact -- unmapped it fakes CONTACT walls,
  // fakes "boxed" (retreat skipped), and resets the stall timer.
  if (ring_front == 0) ring_front = 0xFFFF;
  if (ring_wide  == 0) ring_wide  = 0xFFFF;
  if (ring_left  == 0) ring_left  = 0xFFFF;
  if (ring_right == 0) ring_right = 0xFFFF;
  if (ring_rear  == 0) ring_rear  = 0xFFFF;
  // LIDAR STALENESS WATCHDOG (bench 2026-08-24: the wire went SILENT
  // mid-session -- bytes counter frozen -- and he navigated a stale
  // snapshot for minutes: phantom overheads, stale pivot sides). A quiet
  // wire means BLIND, never last-known; and we try to bring it back.
  static uint32_t lid_last_bytes = 0, lid_last_adv_ms = 0, lid_reinit_ms = 0;
  uint32_t lid_bytes = gary_lidar_bytes_received();
  if (lid_bytes != lid_last_bytes) { lid_last_bytes = lid_bytes; lid_last_adv_ms = now; }
  if (gary_lidar_is_ready() && now - lid_last_adv_ms > 700) {
    ring_front = ring_wide = ring_left = ring_right = ring_rear = 0xFFFF;
    if (now - lid_reinit_ms > 3000) {
      lid_reinit_ms = now;
      Serial.println(">>> LIDAR STALE: no bytes on the wire -- reinit, flying on sonar only");
      gary_lidar_begin();     // reinstalls the UART; ready re-latches after one full rotation
    }
  }
  float e[3];
  gary.board.get_imu_euler(e);                             // e[0]=roll e[1]=pitch (deg)
  float roll = e[0], pitch = e[1];

  // ---- fusion (the graduated logic) ----------------------------------------
  bool ring_blocked = (ring_front < EXP_DUCK_RING_MM) || (ring_wide < EXP_DUCK_RING_MM);
  // CONTACT: the lidar says the nose is ON something. Nothing the sonar
  // thinks overrides this (bench 2026-08-24: ringF held 43-54mm at the
  // oven for 100 s while the blind sonar kept the DUCK walking forward).
  bool front_contact = (ring_front < EXP_CONTACT_RING_MM);
  bool sonar_clear  = sonar_valid && (sonar_mm > EXP_SONAR_CLEAR_MM);   // blind is NOT clear
  bool overhead     = ring_blocked && sonar_clear && !front_contact;    // mast-height only -> duck
  bool wall         = (ring_blocked && !sonar_clear) || front_contact;  // low AND high -> wall
  if (sonar_mm < EXP_CLIMB_TRIG_MM && !overhead && !front_contact)  // solid thing at body height
    exp_climb_until_ms = now + EXP_CLIMB_HOLD_MS;                    //  (contact is a wall, not a climb)
  if (fabsf(pitch) > 5.0f)                                  // inclined = ON an obstacle:
    exp_climb_until_ms = max(exp_climb_until_ms, now + 1000);  // hold climb until level
  if (overhead) {
    // distance-based release (the tall-guy doorway fix): stay low for
    // EXP_DUCK_TRAVEL_MM of commanded travel past the last overhead sighting.
    float v_cm_s = WALK_SPEED * EXP_DUCK_SPEED * EXP_DERATE;
    uint32_t hold = (uint32_t)(EXP_DUCK_TRAVEL_MM / max(v_cm_s * 10.0f, 4.0f) * 1000.0f);
    exp_crouch_until_ms = now + min(hold, (uint32_t)15000);
  }
  bool crouching = now < exp_crouch_until_ms;
  bool climbing  = (now < exp_climb_until_ms) && !crouching;

  // ---- THE PUSHING DETECTOR ------------------------------------------------
  // (see the doctrine note above loop_explore)
  bool pushing = false;
  if (sonar_mm < 550 && exp_mode != 2) {
    float _dx = odom_x - exp_push_ref_x, _dy = odom_y - exp_push_ref_y;
    float _trav = sqrtf(_dx * _dx + _dy * _dy);          // cm since ref
    if (exp_push_ref_mm == 0 ||
        abs((int)sonar_mm - (int)exp_push_ref_mm) > 60) {
      exp_push_ref_mm = sonar_mm;                        // range changed:
      exp_push_ref_x = odom_x; exp_push_ref_y = odom_y;  // new reference
      exp_push_clear = 0;
    } else if (_trav > ((now < exp_push_until) ? 2.5f : 12.0f)) {
      // travelled 12 cm and the range never moved -> the thing ahead is
      // moving WITH me. It is a door, not a wall. (12 cm is the INITIAL
      // evidence bar; an already-latched push re-proves itself every
      // 2.5 cm -- 1.35 s even at CLIMB speed -- so the latch outlives the
      // gap between refreshes with a blind tick to spare.)
      if (now > exp_push_until + 3000)
        Serial.printf(">>> EXPLORE: range pinned at %u mm while I moved "
                      "%.0f cm -- I'm PUSHING something movable\n",
                      sonar_mm, _trav);
      if (exp_push_until == 0) exp_push_start_ms = now;   // a new push
      exp_push_until = now + 2000;    // must exceed the worst re-proof gap
      exp_push_ref_x = odom_x; exp_push_ref_y = odom_y;
      // THE WALL BUDGET (bench 2026-08-23: leg odometry keeps counting
      // while a wall wins, so a wall passes the movable test forever).
      // The doctrine's missing half: a MOVABLE opens the range. Pinned
      // range through EXP_PUSH_WALL_MS of pushing = a wall: drop the
      // latch and let the wall turn fire.
      if (now - exp_push_start_ms > EXP_PUSH_WALL_MS &&
          abs((int)sonar_mm - (int)exp_push_ref_mm) <= 60) {
        exp_push_until = 0;
        exp_push_ref_mm = 0;
        Serial.println(">>> PUSH: range never opened -- that is a WALL, turning");
      }
      // WIDENING RECOGNITION: count ring bearings now seeing PAST the
      // pushed thing. Rising = my push is creating the opening.
      uint8_t _cl = 0;
      for (int _b = -2; _b <= 2; _b++) {
        uint16_t _r = gary_lidar_min_distance((_b * 15 - 5 + 360) % 360,
                                              (_b * 15 + 5 + 360) % 360);
        if (_r != 0xFFFF && _r != 0 && _r > sonar_mm + 250) _cl++;
      }
      if (_cl > exp_push_clear) {
        Serial.printf(">>> EXPLORE: the opening is WIDENING (%u -> %u clear "
                      "bearings) -- MY push is creating it\n",
                      exp_push_clear, _cl);
        exp_push_clear = _cl;
        gary_chirp_snappy();           // "got it"
      }
    }
  } else if (sonar_valid) {
    // a BLIND tick is NOT evidence the reference changed: wiping it here
    // discarded the accrued travel and killed live push latches (~56% of
    // single mid-push no-echo ticks at CLIMB speed). Only a VALID far
    // reading resets the push reference.
    exp_push_ref_mm = 0;
    exp_push_clear = 0;
  }
  pushing = (now < exp_push_until);   // the latch survives blind/far ticks
  if (!pushing) exp_push_until = 0;   // an EXPIRED latch is no latch
  if (pushing && sonar_valid && sonar_mm > 600) {
    // PUSH COMPLETE: it swung clear. Stop shoving, walk through.
    Serial.println(">>> EXPLORE: PUSHED IT OPEN -- the way is passable, "
                   "going through");
    gary_chirp_happy();
    exp_push_until = 0;
    exp_push_ref_mm = 0;
    pushing = false;
  }

  // ---- BREACH FSM (Layer 1: purposeful door-working, replaces the door trial
  //      when BREACH_ENABLE). Owns the tick while active: does its own motion
  //      and returns. Square up -> ONE committed shove -> if it's a wall:
  //      look it up and down, probe-knock, head-shake, turn away + bounded
  //      mule-kick, then move on. Reactive: any range opening -> drive through.
#if BREACH_ENABLE
  {
    uint16_t fwd_mm = sonar_valid ? sonar_mm : 5000;
    if (ring_front != 0xFFFF && ring_front < fwd_mm) fwd_mm = ring_front;
    bool blocked_now = front_contact || (sonar_valid && sonar_mm < 250 && ring_blocked);

    if (breach_ph == BR_NONE && blocked_now && now > breach_cool_until && !crouching
        && now > exp_pivot_until_ms && now > exp_retreat_until_ms) {
      breach_ph = BR_SQUARE; breach_t0 = now;
      breach_ref_mm = fwd_mm; breach_best_mm = fwd_mm;
      breach_turn_dir = (ring_left > ring_right) ? 1 : -1;
      gary_query();
      Serial.println(">>> BREACH: blocked -- squaring up to work it");
    }

    if (breach_ph != BR_NONE) {
      if (fwd_mm > breach_best_mm) breach_best_mm = fwd_mm;
      // REACTIVE: the way opened -> stop the theatrics and go through
      if (breach_ph != BR_KICK && breach_ph != BR_TURN
          && fwd_mm > breach_ref_mm + BREACH_OPEN_MM && fwd_mm > BREACH_MIN_OPEN_MM) {
        Serial.printf(">>> BREACH: it OPENED (%u -> %u mm) -- going through\n", breach_ref_mm, fwd_mm);
        gary_chirp_happy();
        breach_ph = BR_NONE; breach_cool_until = now + 1200;
      }
    }

    if (breach_ph != BR_NONE) {
      float bv = 0.0f, bomega = 0.0f, bpitch = 0.0f, byaw = 0.0f, bheight = 0.0f;
      const float blift = EXP_CRUISE_LIFT;
      uint32_t pt = now - breach_t0;
      switch (breach_ph) {
        case BR_SQUARE:
          if (pt > 500) { breach_ph = BR_SHOVE; breach_t0 = now; breach_ref_mm = fwd_mm;
                          gary_determined_growl(); Serial.println(">>> BREACH: one committed shove"); }
          break;
        case BR_SHOVE:
          bv = WALK_SPEED * BREACH_SHOVE_SPD * EXP_DERATE;
          if (pt > BREACH_SHOVE_MS) { breach_ph = BR_LOOK; breach_t0 = now;
                          gary_concerned_low(); Serial.println(">>> BREACH: wouldn't give -- looking it up and down"); }
          break;
        case BR_LOOK: {
          float hs = pt * 0.001f;
          bpitch = (hs < 1.1f) ? ( BREACH_LOOK_PITCH * sinf(PI * hs / 1.1f))
                               : (-0.45f * BREACH_LOOK_PITCH * sinf(PI * (hs - 1.1f) / 0.5f));
          if (pt > BREACH_LOOK_MS) { bpitch = 0.0f; breach_ph = BR_KNOCK; breach_t0 = now;
                          Serial.println(">>> BREACH: probe-knock"); }
          break; }
        case BR_KNOCK:
          bv = (pt < BREACH_KNOCK_MS) ? (WALK_SPEED * BREACH_SHOVE_SPD * EXP_DERATE) : 0.0f;
          if (pt > BREACH_KNOCK_END_MS) { breach_ph = BR_SHAKE; breach_t0 = now; }
          break;
        case BR_SHAKE:
          byaw = 0.35f * sinf(2.0f * PI * (pt * 0.001f) / 0.55f);
          bheight = -1.0f;
          if (pt > BREACH_SHAKE_MS) { breach_ph = BR_TURN; breach_t0 = now;
                          Serial.println(">>> BREACH: giving up on it -- turning away"); }
          break;
        case BR_TURN:
          bomega = EXP_PIVOT_OMEGA * breach_turn_dir;
          byaw = clamp_f(0.15f * breach_turn_dir, -0.30f, 0.30f);
          if (pt > BREACH_TURN_MS) { breach_ph = BR_KICK; breach_t0 = now;
                          gary_chirp_snappy(); Serial.println(">>> BREACH: mule-kick -- parting shot"); }
          break;
        case BR_KICK:
          bv = -WALK_SPEED * BREACH_KICK_SPD * EXP_DERATE;
          if (pt > BREACH_KICK_MS) { breach_ph = BR_DONE; breach_t0 = now; }
          break;
        case BR_DONE:
        default:
          breach_ph = BR_NONE;
          breach_cool_until = now + BREACH_COOL_MS;
          exp_pivot_dir = breach_turn_dir;
          exp_retreat_until_ms = now;
          exp_pivot_until_ms = now + EXP_PIVOT_MS;         // walk it off via the normal pivot
          exp_turn_cool_until_ms = exp_pivot_until_ms + 3000;
          Serial.println(">>> BREACH: moving on");
          break;
      }

      if (breach_ph != BR_NONE) {
        // contact invariant: don't grind forward into contact EXCEPT during the
        // deliberate shove/knock (driving into it is the whole point there)
        if (front_contact && bv > 0.0f && breach_ph != BR_SHOVE && breach_ph != BR_KNOCK)
          bv = 0.0f;
        gary_body_yaw_rad = byaw;
        Velocity_t bcv = { 0.0f, bv, bomega };
        Vector_t   bcp = { 0.0f, 0.0f,
                           clamp_f(1.5f + bheight, CLAMP_HEIGHT_MIN + 1.5f, CLAMP_HEIGHT_MAX + 1.5f) };
        Euler_t    bca = { clamp_f(bpitch, -CLAMP_PITCH_DEG, CLAMP_PITCH_DEG), 0.0f, 0.0f };
        uint16_t bt = gait_min_cadence(GAIT_TRIPOD);
        gary.move(&bcv, &bcp, &bca, bt);
        gary.func_state = Robot::ACTION_GROUP;
        run_custom_gait_scheduler(GAIT_TRIPOD, bt, blift, bcv.omega, bcv.vy);
        static uint32_t br_last = 0;
        if (now - br_last > 500) { br_last = now;
          Serial.printf(">>> BREACH | ph=%d | fwd=%u ref=%u best=%u | v=%.2f om=%.2f pitch=%.1f\n",
                        (int)breach_ph, fwd_mm, breach_ref_mm, breach_best_mm, bv, bomega, bpitch);
        }
        return;   // breach owns the robot this tick
      }
    }
  }
#endif

  // ---- THE DOOR TRIAL -------------------------------------------------------
  // (bench 2026-08-24: at close range the contact invariant froze him,
  // frozen legs accrue no odometry, so the 12cm push evidence could never
  // accumulate -> approach / freeze / turn / skitter at every door. The
  // doctrine per Bill: walk up, ONE deliberate bounded shove, judge by
  // whether the range opens, then commit -- through it or away from it.)
  static uint32_t exp_trial_until = 0, exp_trial_cool_until = 0;
  static uint16_t exp_trial_ref = 0;
  bool front_blocked_close = front_contact ||
       (sonar_valid && sonar_mm < 250 && ring_blocked);
  if (!BREACH_ENABLE && exp_trial_until == 0 && now > exp_trial_cool_until
      && now > exp_turn_cool_until_ms && now > exp_pivot_until_ms
      && front_blocked_close && !pushing && !crouching) {
    exp_trial_until = now + EXP_PUSH_WALL_MS;
    exp_trial_ref = sonar_valid ? sonar_mm : 0;
    Serial.printf(">>> EXPLORE: blocked (%s) -- ONE shove, then I judge\n",
                  sonar_valid ? "sonar+ring" : "ring, sonar blind");
  }
  if (exp_trial_until != 0) {
    if (exp_trial_ref == 0 && sonar_valid) exp_trial_ref = sonar_mm;
    bool trial_opened = (sonar_valid && exp_trial_ref != 0 &&
                         sonar_mm > exp_trial_ref + 150);
    if (trial_opened) {
      Serial.println(">>> EXPLORE: it YIELDED -- through we go");
      gary_chirp_happy();
      exp_trial_until = 0;
      exp_trial_cool_until = now + 2000;
    } else if (now >= exp_trial_until) {
      Serial.println(">>> EXPLORE: it did not budge -- WALL, turning away");
      exp_trial_until = 0;
      exp_trial_cool_until = now + 8000;
      exp_retreat_until_ms = now + EXP_RETREAT_MS;
      if (ring_rear < 120) exp_retreat_until_ms = now;
      exp_pivot_dir = (ring_left > ring_right) ? 1 : -1;
      exp_pivot_until_ms = exp_retreat_until_ms + EXP_PIVOT_MS;
      exp_turn_cool_until_ms = exp_pivot_until_ms + 4000;
      gary_query();
    } else {
      pushing = true;   // the licensed shove: contact invariant and wall turn stand down
    }
  }

  // ---- hazard: retreat, then pivot toward the open side --------------------
  // (a yielding door is NOT a wall: while pushing, keep driving)
  // NOTE a crouch no longer suppresses the wall turn -- that gate was the
  // other half of the oven deadlock (garbage sonar refreshed the crouch,
  // the crouch muzzled the wall). He turns at whatever height he holds.
  static uint8_t exp_wall_streak = 0;
  if (wall) { if (exp_wall_streak < 250) exp_wall_streak++; }
  else exp_wall_streak = 0;
  // A frontal block with VALID sonar at standoff range is not turned from
  // any more -- he closes in and runs the door trial instead. The turn
  // fires when close/blind/oblique, or when a trial just failed.
  if (exp_wall_streak >= 3 && !pushing && now > exp_turn_cool_until_ms
      && (front_blocked_close || !sonar_valid || now < exp_trial_cool_until)) {
    exp_retreat_until_ms = now + EXP_RETREAT_MS;
    if (ring_rear < 120) exp_retreat_until_ms = now;       // boxed: skip the retreat
    exp_pivot_dir = (ring_left > ring_right) ? 1 : -1;     // spin toward open space
    exp_pivot_until_ms = exp_retreat_until_ms + EXP_PIVOT_MS;
    exp_turn_cool_until_ms = exp_pivot_until_ms + 4000;
    gary_query();                                          // "huh?" at the wall
    char _sw[8]; snprintf(_sw, sizeof(_sw), "%u", sonar_mm);
    Serial.printf(">>> EXPLORE: wall (ring %u / sonar %s) -- retreat then pivot %s\n",
                  ring_front, sonar_valid ? _sw : "BLIND",
                  exp_pivot_dir > 0 ? "LEFT" : "RIGHT");
  }

  // ---- posture + speed ------------------------------------------------------
  float spd, lift, height;
  if (crouching)      { spd = EXP_DUCK_SPEED;  lift = EXP_CRUISE_LIFT; height = EXP_DUCK_HEIGHT; }
  else if (climbing)  { spd = EXP_CLIMB_SPEED; lift = EXP_CLIMB_LIFT;  height = EXP_CLIMB_HEIGHT; }
  else                { spd = EXP_CRUISE_SPEED; lift = EXP_CRUISE_LIFT; height = EXP_CRUISE_HEIGHT; }
  exp_mode = crouching ? 2 : (climbing ? 1 : 0);
  // his VOICE at the real moments (2026-08-21, Bill: "turn his sound back
  // on" -- the 19-track April library was muted by neglect: wired into the
  // scheduler but almost never triggered). Edge-triggered on transitions.
  static uint8_t exp_mode_prev = 0;
  if (exp_mode != exp_mode_prev) {
    if (exp_mode == 1)      gary_determined_growl();       // committing to a climb
    else if (exp_mode == 2) gary_chirp_snappy();           // "got it -- ducking"
    exp_mode_prev = exp_mode;
  }

  // unstick (sim-earned): stalled = sonar unchanged too long while trying to move.
  uint16_t stall_prox = sonar_mm;      // nearest believable forward range:
  if (ring_front != 0xFFFF && ring_front < stall_prox) stall_prox = ring_front;
  if (abs((int)stall_prox - (int)exp_stall_ref_sonar) > 40) {
    exp_stall_ref_sonar = stall_prox; exp_stall_ref_ms = now;
  }
  bool _stalled_now = (now - exp_stall_ref_ms > EXP_STUCK_MS && now > exp_pivot_until_ms);
  static bool _stalled_prev = false;
  static uint8_t _stall_streak = 0;
  if (_stalled_now && !_stalled_prev) {
    _stall_streak++;
    if (_stall_streak % 3 == 0) gary_defeat_performance(); // the WHOLE thing:
    // look it up and down, sag, the sad tone, a quick wag, walk it off.
    // Three stalls in a row IS a defeat, and he has earned the gesture.
    else                        gary_concerned_low();      //   (third strike)
    if (_stall_streak >= 2) {
      // STALL ESCALATION (bench 2026-08-24: a clothes pile eats BOTH
      // sensors -- ultrasound AND infrared -- and he rocked against it
      // forever). Two rock cycles with zero range progress = there IS
      // something there; treat it as a wall and leave.
      exp_retreat_until_ms = now + EXP_RETREAT_MS;
      if (ring_rear < 120) exp_retreat_until_ms = now;
      exp_pivot_dir = (ring_left > ring_right) ? 1 : -1;
      exp_pivot_until_ms = exp_retreat_until_ms + EXP_PIVOT_MS;
      exp_turn_cool_until_ms = exp_pivot_until_ms + 4000;
      exp_stall_ref_ms = now;
      Serial.println(">>> EXPLORE: stalled twice, zero progress -- something unseen is there, turning away");
    }
  }
  _stalled_prev = _stalled_now;
  if (_stalled_now) {
    uint32_t ph = ((now - exp_stall_ref_ms - EXP_STUCK_MS) / 1500) % 2;
    bool inclined = fabsf(pitch) > 5.0f;
    if (ph == 0 || inclined) {                              // push-up surge; NEVER
      height = crouching ? EXP_DUCK_HEIGHT : 2.0f;          // reverse while inclined
      lift = EXP_CLIMB_LIFT; spd = 0.6f;
    } else {
      spd = -0.45f;
    }
  }

  if (pushing) {                    // lean in and keep walking: the
    spd = fmaxf(spd, 0.55f);        // door opens as he goes
    lift = EXP_CRUISE_LIFT;
  }
  float vy = WALK_SPEED * spd * EXP_DERATE;
  float omega = 0.0f;
  if (now < exp_retreat_until_ms) { vy = -WALK_SPEED * 0.5f * EXP_DERATE; }
  else if (now < exp_pivot_until_ms) { vy = 0.0f; omega = EXP_PIVOT_OMEGA * exp_pivot_dir; }
  else if (front_contact && !pushing && vy > 0.0f) { vy = 0.0f; }
  // ^ THE INVARIANT (bench 2026-08-24): forward is NEVER commanded into
  //   lidar contact unless deliberately pushing -- covers the post-pivot
  //   cooldown window where the wall turn cannot re-fire yet.

  // CORNERING YAW LEAD (Bill: "the YAW... might even help in his
  // cornering abilities. hell yah, i think it will."). Pose the body
  // into the turn while the gait carves the track, so the nose -- and
  // the whole LiDAR ring -- looks THROUGH the corner early. Sim
  // evolution adopted this on merit in race mode (0.0 -> 0.149) but
  // rejected it while exploring, where pose-yawing the ring costs the
  // map more than the cornering buys: so it is applied only to the
  // deliberate pivot, never to ordinary cruising.
  gary_body_yaw_rad = (now < exp_pivot_until_ms)
                    ? clamp_f(0.15f * exp_pivot_dir, -0.30f, 0.30f) : 0.0f;

  // ---- motion out (same plumbing as the controller's wave/ripple path) ------
  Velocity_t cv = { 0.0f, vy, omega };
  Vector_t   cp = { 0.0f, 0.0f, clamp_f(1.5f + height, CLAMP_HEIGHT_MIN + 1.5f, CLAMP_HEIGHT_MAX + 1.5f) };
  Euler_t    ca = { 0.0f, 0.0f, 0.0f };
  uint16_t t_g = gait_min_cadence(GAIT_TRIPOD);
  gary.move(&cv, &cp, &ca, t_g);
  gary.func_state = Robot::ACTION_GROUP;
  run_custom_gait_scheduler(GAIT_TRIPOD, t_g, lift, cv.omega, cv.vy);

  // throttled telemetry
  static uint32_t exp_last_print = 0;
  if (now - exp_last_print > 2000) {
    exp_last_print = now;
    char _sn[8];
    if (sonar_valid) snprintf(_sn, sizeof(_sn), "%u", sonar_mm);
    else             strcpy(_sn, "BLIND");
    Serial.printf(">>> EXPLORE | %s%s | sonar=%s ringF=%u L=%u R=%u pitch=%+.1f | v=%.2f h=%+.1f lift=%.1f\n",
                  exp_mode == 2 ? "DUCK" : exp_mode == 1 ? "CLIMB" : "cruise",
                  front_contact ? "+CONTACT" : "",
                  _sn, ring_front, ring_left, ring_right, pitch, vy, height, lift);
  }
}

// ---- loop profiler ----------------------------------------------------------
// loop() is running at ~4Hz. That is disastrous and it is not just a LiDAR problem:
// the gait controller updates at loop rate (a walker being steered 4 times a second
// will absolutely wander), and so does the ASR poll -- which is why voice commands
// stopped responding again. Rather than guess which call is eating the time, MEASURE.
// Prints the worst offender every 2s. Delete this block once the culprit is fixed.
#define PROF(name, call) do { uint32_t _t0 = micros(); call; prof_acc[name] += (micros() - _t0); } while (0)
enum { P_BP32, P_SOUND, P_FACE, P_LIDAR, P_WIFI, P_CONSOLE, P_STATE, P_SCAN, P_BG, P_N };
static const char* PROF_NAMES[P_N] = {"BP32","sounds","face","lidar","wifi","console","state","scandump","bgchk"};
static uint32_t prof_acc[P_N] = {0};
static uint32_t prof_loops = 0;
static uint32_t prof_last_ms = 0;

static void prof_report() {
  uint32_t now = millis();
  if (now - prof_last_ms < 2000) return;
  float secs = (now - prof_last_ms) / 1000.0f;
  Serial.printf(">>> ASR | %.1f polls/sec | hits=%lu badreads=%lu last=%u  %s\n",
                asr_polls / secs, (unsigned long)asr_hits, (unsigned long)asr_bad, asr_last,
                asr_hits ? "" : "<-- NEVER heard a command");
  asr_polls = 0;
  Serial.printf(">>> LOOP | %.1f Hz | ", prof_loops / secs);
  for (int i = 0; i < P_N; i++) {
    Serial.printf("%s=%.1fms ", PROF_NAMES[i],
                  prof_loops ? (prof_acc[i] / 1000.0f / prof_loops) : 0.0f);
    prof_acc[i] = 0;
  }
  Serial.println();
  prof_loops = 0;
  prof_last_ms = now;
  knee_guard_report();   // prints only if a knee clipped since last report
}

void loop() {
  prof_loops++;
  knee_guard_update();   // stall-watchdog housekeeping (auto-clears the trip latch)
#if ENABLE_OTA
  if (wifi_connected) ArduinoOTA.handle();   // wireless flashing (cheap when idle)
#endif
  // Pump Bluepad32 first - keeps connection alive and reads new packets
  PROF(P_BP32, BP32.update());

  // Auto-transition to controller mode when DualSense connects
  if (myGamepad && myGamepad->isConnected() && current_state != STATE_CONTROLLER) {
    transition_to(STATE_CONTROLLER);
  }

  // Pump the sound state machine every loop iteration
  PROF(P_SOUND,   gary_sounds_update());
  // The LED matrix costs 12ms per call -- by far the most expensive named thing in
  // the loop. It's a bit-banged display and there is no reason to redraw it at the
  // full loop rate; 25Hz is smoother than the eye needs. Rate-limiting it hands
  // those 12ms back to the gait and the voice poll.
  {
    static uint32_t last_face_ms = 0;
    if (millis() - last_face_ms >= 40) {
      last_face_ms = millis();
      PROF(P_FACE, gary_face_update());
    }
  }
  PROF(P_LIDAR,   gary_lidar_update());
  PROF(P_WIFI,    wifi_pump());
  PROF(P_CONSOLE, poll_console());   // 's' toggles scanner, '1'/'0' force it
  prof_report();

#if ENABLE_DEBUG_SERIAL
  static uint32_t last_heartbeat = 0;
  if (millis() - last_heartbeat > 2000) {
    Serial.print(">>> GARY HB | state=");
    Serial.print(state_name(current_state));
    Serial.print(" | t=");
    Serial.print(millis());
    Serial.print(" | bat=");
    Serial.print(last_battery_mv);
    Serial.println("mV");
    last_heartbeat = millis();
  }

  static uint32_t last_lidar_diag = 0;
  if (scanner_mode && millis() - last_lidar_diag > 2000) {
    last_lidar_diag = millis();
    // Quick LiDAR status: ready flag, packet rate, and the cardinal-direction
    // distances. Forward = 0°, right = 90°, back = 180°, left = 270° (in
    // LiDAR frame; physical orientation depends on mount).
    // bytes=... added to tell apart "nothing arriving on the wire at all"
    // (bytes stuck at 0 -- power/wiring/pin, a hardware problem) from "bytes
    // arriving but never forming a valid packet" (bytes climbing, pkts not --
    // baud/framing/noise). See gary_lidar.h for the full diagnosis note.
    Serial.printf(">>> LIDAR | ready=%d bytes=%lu pkts=%lu bad=%lu | F=%umm R=%umm B=%umm L=%umm\n",
                  gary_lidar_is_ready() ? 1 : 0,
                  (unsigned long)gary_lidar_bytes_received(),
                  (unsigned long)gary_lidar_packets_received(),
                  (unsigned long)gary_lidar_bad_packets(),
                  gary_lidar_distance_at_angle(0),
                  gary_lidar_distance_at_angle(90),
                  gary_lidar_distance_at_angle(180),
                  gary_lidar_distance_at_angle(270));
  }

  // ---- Full-scan + pose dump for the live map viewer (gary_lidar_map.py) --
  // "SCAN:<x_cm>,<y_cm>,<theta_deg>;<d0>,<d1>,...,<d359>\n" -- pose from the
  // dead-reckoning odometry above, then one distance per degree (0..359).
  //
  // Gated on scanner_mode: this is the single biggest consumer of loop time
  // (~1.8KB/frame), and when it ran unconditionally at 115200 it blocked
  // ~150ms every 400ms, starving the ASR poll. Now it only runs when you've
  // explicitly asked for the scan stream (touchpad click / 's' on serial),
  // and at 921600 baud the same frame costs ~20ms instead of ~150ms -- cheap
  // enough to halve the interval to 200ms for a smoother live map.
  //
  // Only active when ready==1 (i.e. after the first full rotation), so the
  // Python side never has to special-case an all-zero warm-up frame.
  static uint32_t last_scan_dump = 0;
  if (scanner_mode && gary_lidar_is_ready() && millis() - last_scan_dump > SCAN_DUMP_INTERVAL_MS) {
    last_scan_dump = millis();
    // Built into one buffer (rather than piecemeal Serial.print calls) so
    // the exact same bytes can go to both Serial and the WiFi client without
    // duplicating the assembly logic. ~1.8KB max (360 pts * ~5 chars + pose
    // prefix) comfortably fits the 2200-byte buffer.
    static char scan_buf[2200];
    int pos = 0;
    // The LiDAR is BOLTED TO GARY'S BODY. When he pitches or rolls, the scan
    // plane tilts with him -- the geometry is genuinely 3D. The live map was flat
    // for a dumb reason: this line only ever carried the POSE and threw the TILT
    // away, so the dashboard had no idea the plane wasn't level and drew
    // everything at z=0. (NOD: has the opposite bug -- it carries tilt but no
    // pose.) Each format discarded exactly what the other one kept.
    //
    // Now we send both. Drive him around while working the right stick and the
    // live map becomes a real point cloud instead of a floor plan.
    //
    // euler.x = roll, euler.y = pitch (degrees). These come from _imu_update(),
    // the same timer that was switched off -- so before the imu_update(true) fix
    // above, these would have been zeros anyway.
    float _e[3] = {0, 0, 0};
    gary.board.get_imu_euler(_e);
    pos += snprintf(scan_buf + pos, sizeof(scan_buf) - pos, "SCAN:%.1f,%.1f,%.1f,%.1f,%.1f;",
                    odom_x, odom_y, odom_theta_deg, _e[1], _e[0]);
    for (int i = 0; i < 360 && pos < (int)sizeof(scan_buf) - 8; i++) {
      pos += snprintf(scan_buf + pos, sizeof(scan_buf) - pos, "%u", gary_lidar_distance_at_angle(i));
      if (i < 359) pos += snprintf(scan_buf + pos, sizeof(scan_buf) - pos, ",");
    }
    pos += snprintf(scan_buf + pos, sizeof(scan_buf) - pos, "\n");
    uint32_t _tscan = micros();
    Serial.print(scan_buf);
    wifi_scan_broadcast(scan_buf);  // no-op if WiFi/client isn't connected
    prof_acc[P_SCAN] += (micros() - _tscan);   // was never actually timed before
  }
#endif

  // -----------------------------------------------------------------------
  // NOTE (fix, this version): GARY_v3_nodscan_v3.ino and _v4.ino as they
  // existed in the outputs folder were byte-identical and both cut off
  // right here mid-statement -- missing the tip/battery/startle check call
  // and the entire per-state dispatch switch below, so neither would have
  // compiled as saved. Reconstructed from GARY_v3_nodscan_v2.ino /
  // GARY_v3_final.ino (both complete) and extended with the STATE_AUTOSCAN
  // case. If your actual flashed firmware already has its own fix for this,
  // diff before flashing this file over it.
  // -----------------------------------------------------------------------
  uint32_t _tbg = micros();
  bool _bg = run_background_checks();
  prof_acc[P_BG] += (micros() - _tbg);
  if (_bg) {
    delay(loop_delay_ms);
    return;
  }

  uint32_t _tstate = micros();
  switch (current_state) {
    case STATE_BOOT:      transition_to(STATE_ROAMING); break;
    case STATE_IDLE:      loop_idle();      break;
    case STATE_LISTENING: loop_listening(); break;
    case STATE_EXECUTING: loop_executing(); break;
    case STATE_ROAMING:   loop_roaming();   break;
    case STATE_COMING:    loop_coming();    break;
    case STATE_STARTLED:  loop_startled();  break;
    case STATE_ALARMED:   loop_alarmed();   break;
    case STATE_CONTROLLER:loop_controller();break;
    case STATE_SENSING:   loop_sensing();   break;
    case STATE_NODSCAN:   break;  // sweep runs synchronously in the CMD_ACTION_14 handler
    case STATE_AUTOSCAN:  loop_autoscan();  break;
    case STATE_EXPLORE:   loop_explore();   break;
  }
  prof_acc[P_STATE] += (micros() - _tstate);   // loop_controller() lives in here

  delay(loop_delay_ms);   // runtime-adjustable; press 'd' -- see handle_console_char
}