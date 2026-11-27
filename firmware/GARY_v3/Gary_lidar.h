// =============================================================================
// Gary_lidar.h
//
// LDROBOT D500 LiDAR driver. Talks to the LiDAR over ESP32's UART2 (GPIO18 =
// ESP32 RX, receiving LiDAR's TX data line). UART2 TX is left unassigned
// because the D500 is one-way (it transmits, doesn't listen).
//
// Pin history — read before "helpfully" changing LIDAR_RX_PIN:
//   - IO18/IO19 are shared with the MM32 servo-controller's PB3/PB4
//     programming/debug pins (hiwonder_board.h's MM32 link). UART2 traffic
//     there DURING MM32 BOOT can cause a partial servo-init failure (one
//     leg's joint A landing at the wrong angle). This is why an earlier
//     revision of this driver avoided 18/19 in favor of GPIO32.
//   - GPIO32/33 are TAKEN (gary_face.h's LED matrix, DIN/CLK) and GPIO34 is
//     TAKEN (hiwonder_board.h's sound_pin), and the physical LiDAR wiring
//     is already run to the 18/19 pair on hardware that's hard to re-open.
//   - CONFIRMED WORKING on GPIO18: a real test run showed ready=1, bytes=465,
//     pkts=9, bad=1, with plausible distance readings (F=539mm). The counts
//     froze mid-test, but that coincided with the robot tipping over
//     (TIP DETECTED, roll swinging to -94/-109°) — almost certainly a wire
//     getting jostled by the fall, not a wrong pin. GPIO19 was tried as an
//     alternate guess before this confirmation came in; go back to 18.
//   - THE TELL (if this is ever revisited): if the MM32-boot-timing bet
//     were wrong, you'd see it immediately at power-on — one leg sitting at
//     a visibly wrong angle before Gary does anything else. Legs have been
//     fine across every test so far, so this risk hasn't materialized.
//     GPIO35 (free, input-only, ADC1_CHANNEL_7) remains the fallback if it
//     ever does, but requires physically moving the wire.
//
// Parses the 47-byte D500 packet format on the fly and maintains a 360-entry
// polar distance buffer indexed by integer degree.
//
// Mechanics:
//   - begin() opens UART2 at 230400 baud on pin 18 (RX-only, TX unassigned).
//   - update() must be called every loop iteration to drain the UART buffer
//     and feed bytes into the packet state machine.
//   - distance_at_angle(deg) returns the most recent measurement at that
//     compass bearing in millimeters, or 0 if there's no data yet.
//
// 0° is whichever direction the LiDAR was physically pointing at startup;
// when you mount it on Gary, decide which body direction "LiDAR 0°" should
// correspond to and add a fixed offset before calling these accessors.
// =============================================================================

#ifndef GARY_LIDAR_H
#define GARY_LIDAR_H

#include <stdint.h>

void     gary_lidar_begin();

// Start the dedicated LiDAR reader task. Call this once, right after
// gary_lidar_begin(). It decouples UART draining from the main loop's rate --
// which matters enormously: loop() runs at 3-6 Hz, the D500 streams ~17.6 KB/s,
// and the old loop-driven reader could physically only absorb ~2 KB/s of it.
// Over 90% of the LiDAR was being thrown away. See the long comment in
// Gary_lidar.cpp for the full autopsy.
void     gary_lidar_start_task();

// Drain the UART by hand. Becomes a NO-OP once gary_lidar_start_task() has run
// (the task owns the UART then, and draining from two places would interleave
// bytes and corrupt every packet). Kept so the old loop()/gary_delay() call
// sites remain harmless, and as a fallback if the task fails to start.
void     gary_lidar_update();

// Most recent distance reading at the given compass bearing (rounded to
// the nearest integer degree). Returns 0 if no measurement has arrived
// at that bearing yet, OR if the LiDAR reported an invalid reading there
// (out of range, no return, etc.). Both cases mean "no data" — callers
// should treat 0 as "ignore this slot."
uint16_t gary_lidar_distance_at_angle(int deg);

// Minimum non-zero distance within an arc. Handles wrap (e.g., start=350,
// end=10 = the 20° arc straight ahead if LiDAR-0° is forward).
// Returns 0 if no valid readings in the arc.
uint16_t gary_lidar_min_distance(int start_deg, int end_deg);

// True once at least one full rotation has been observed since boot. Until
// this returns true, large swaths of distance_at_angle() will return 0.
bool     gary_lidar_is_ready();

// Diagnostic counters.
uint32_t gary_lidar_packets_received();
uint32_t gary_lidar_bad_packets();

// Raw bytes seen on the UART2 RX line since gary_lidar_begin() was called,
// regardless of whether they ever formed a valid packet. Use this to tell
// apart failure modes:
//   - bytes stuck at 0            -> nothing reaching GPIO18. Check wiring/
//                                    connector for a loose joint, especially
//                                    after any fall/impact.
//   - bytes climbing but frozen   -> was working, then stopped. Likely a
//     partway through a test         connector got jostled (e.g. by a tip-
//                                    over) rather than a firmware issue.
//   - bytes and pkts both         -> working as intended.
//     climbing together
uint32_t gary_lidar_bytes_received();

#endif // GARY_LIDAR_H
