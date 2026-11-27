// =============================================================================
// gary_lidar.h
//
// LDROBOT D500 LiDAR driver. Talks to the LiDAR over ESP32's UART2 (GPIO32 =
// ESP32 RX, receiving LiDAR's TX data line). UART2 TX is left unassigned
// because the D500 is one-way (it transmits, doesn't listen).
//
// IO32 is intentional — NOT IO18/IO19, which look free at the header but are
// actually shared with the MM32 servo-controller's programming lines and
// disrupt servo init when driven by UART traffic.
//
// Parses the 47-byte D500 packet format on the fly and maintains a 360-entry
// polar distance buffer indexed by integer degree.
//
// Mechanics:
//   - begin() opens UART2 at 230400 baud on pin 18.
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

#endif // GARY_LIDAR_H
