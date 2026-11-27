// gary_knee_guard.h — knee (tibia, joint_c) servo protection. 2026-08-18.
//
// WHY: hiwonder_robot.cpp clamps the FEMUR duty to 857–2143 µs but leaves the KNEE
// unclamped on a 280° servo whose zero point (c = 0) already sits at 230° — only 50°
// from the 2500 µs mechanical end stop. Any IK output with c > +50° pins the knee
// against its end stop and holds it there under load. Hiwonder's own docs: stall =
// burnout. (Servo = HPD-0127 with a slip clutch — the clutch saves the gears, not the
// motor: sustained commanded-past-limit still cooks windings/driver and wears the clutch,
// which is how Gary lost two knees anyway.) Gary has already lost two knee servos this way (April IK-rewrite lock-ups,
// then the July wave/ripple scheduler). See notes/CAD-GEOMETRY-2026-08-18.md.
//
// WHAT:
//   1. knee_guard_clamp(id, duty)  — hard duty clamp, called from RobotLeg::move()
//      right where the femur clamp already lives. Counts + timestamps clips per leg.
//   2. stall watchdog — if one knee is *continuously* clipped for KNEE_GUARD_STALL_MS
//      the guard latches "tripped". The custom gait scheduler checks this and holds
//      position (stops fighting); the main loop prints a throttled report.
//   3. knee_guard_report() — one-line telemetry for the 2 s profiler cadence.
//
// LIMITS ARE BENCH-UNVERIFIED (hardware in Norway). 600/2400 µs only shaves the last
// ~14° before each mechanical stop — it cannot break normal motion, it only prevents
// end-stop pinning. Bench task: find the true interference angles (blade vs femur
// bracket) with the DIAG sketch and tighten these.
#pragma once
#include <Arduino.h>

#define KNEE_GUARD_MIN_US     600    // bench-tighten later
#define KNEE_GUARD_MAX_US     2400
#define KNEE_GUARD_STALL_MS   400    // continuous clip longer than this = stall risk
#define KNEE_GUARD_LEGS       6

// Called with the raw commanded duty; returns the safe duty. leg_id is 1..6.
uint16_t knee_guard_clamp(uint8_t leg_id, uint16_t duty);

// True once any knee has been continuously clipped past KNEE_GUARD_STALL_MS.
// Latched; cleared automatically KNEE_GUARD_COOLDOWN_MS after the clipping stops.
bool knee_guard_tripped();

// Housekeeping + auto-clear. Call once per loop() pass (cheap).
void knee_guard_update();

// Throttled human-readable status: prints only if something clipped since last call.
void knee_guard_report();

// Totals (for SCAN/telemetry lines if wanted).
uint32_t knee_guard_total_clips();
