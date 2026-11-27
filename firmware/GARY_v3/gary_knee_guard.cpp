// gary_knee_guard.cpp — see header for rationale. 2026-08-18.
#include "gary_knee_guard.h"

#define KNEE_GUARD_COOLDOWN_MS 2000   // clear the trip latch this long after clipping stops

static uint32_t clip_count[KNEE_GUARD_LEGS]   = {0};   // total clips per leg
static uint32_t clip_since_ms[KNEE_GUARD_LEGS] = {0};  // start of the current continuous-clip run (0 = not clipping)
static uint32_t last_clip_ms   = 0;                    // last time any leg clipped
static uint32_t report_seen    = 0;                    // clips already reported
static bool     tripped        = false;
static int8_t   trip_leg       = -1;

uint16_t knee_guard_clamp(uint8_t leg_id, uint16_t duty) {
  uint8_t i = (leg_id >= 1 && leg_id <= KNEE_GUARD_LEGS) ? (leg_id - 1) : 0;
  uint32_t now = millis();
  if (duty < KNEE_GUARD_MIN_US || duty > KNEE_GUARD_MAX_US) {
    clip_count[i]++;
    last_clip_ms = now;
    if (clip_since_ms[i] == 0) clip_since_ms[i] = now;
    else if (!tripped && (now - clip_since_ms[i]) > KNEE_GUARD_STALL_MS) {
      tripped = true;
      trip_leg = (int8_t)leg_id;
      Serial.printf(">>> KNEE GUARD TRIP | leg %d clipped continuously for %lums — holding position\n",
                    leg_id, (unsigned long)(now - clip_since_ms[i]));
    }
    return duty < KNEE_GUARD_MIN_US ? KNEE_GUARD_MIN_US : KNEE_GUARD_MAX_US;
  }
  clip_since_ms[i] = 0;   // this leg is back inside the window
  return duty;
}

bool knee_guard_tripped() { return tripped; }

void knee_guard_update() {
  if (tripped && last_clip_ms && (millis() - last_clip_ms) > KNEE_GUARD_COOLDOWN_MS) {
    Serial.printf(">>> KNEE GUARD | leg %d clear, resuming\n", trip_leg);
    tripped = false;
    trip_leg = -1;
  }
}

uint32_t knee_guard_total_clips() {
  uint32_t t = 0;
  for (int i = 0; i < KNEE_GUARD_LEGS; i++) t += clip_count[i];
  return t;
}

void knee_guard_report() {
  uint32_t total = knee_guard_total_clips();
  if (total == report_seen) return;
  Serial.printf(">>> KNEE GUARD | clips L1..L6: %lu %lu %lu %lu %lu %lu%s\n",
                (unsigned long)clip_count[0], (unsigned long)clip_count[1],
                (unsigned long)clip_count[2], (unsigned long)clip_count[3],
                (unsigned long)clip_count[4], (unsigned long)clip_count[5],
                tripped ? "  [TRIPPED]" : "");
  report_seen = total;
}
