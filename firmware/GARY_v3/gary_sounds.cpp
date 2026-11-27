// =============================================================================
// gary_sounds.cpp — R2-D2 frequency-sweep synthesis on a single buzzer
//
// Architecture:
//   - Each emotion is a const array of SoundStep entries.
//   - Each step has start_hz, end_hz, duration_ms.
//   - During playback, gary_sounds_update() updates the buzzer frequency
//     based on linear interpolation between start_hz and end_hz over the
//     elapsed time within the step.
//   - Held tones are just steps where start_hz == end_hz.
//   - Silence is start_hz == end_hz == 0.
//
// Why this is better than fixed-pitch steps:
//   - R2-D2 sounds are continuous frequency curves, not staccato notes.
//   - Held tones for >150ms feel "stuck"; sweeps feel alive.
//   - Same buzzer, same single channel — pure software change.
//
// Update granularity:
//   - We re-write the LEDC frequency every UPDATE_PERIOD_MS during a sweep.
//   - 5ms granularity = 200 updates/sec = smooth-sounding sweeps.
// =============================================================================

#include "gary_sounds.h"
#include <Arduino.h>
#include "hiwonder_board.h"

struct SoundStep {
  uint16_t start_hz;
  uint16_t end_hz;
  uint16_t duration_ms;
};

static const uint16_t UPDATE_PERIOD_MS = 5;

static const SoundStep* g_track = nullptr;
static uint16_t g_track_len = 0;
static uint16_t g_step = 0;
static uint32_t g_step_started_ms = 0;
static uint32_t g_last_freq_update_ms = 0;
static uint16_t g_last_freq = 0;
static bool g_sounds_enabled = true;

static inline uint16_t lerp_freq(uint16_t a, uint16_t b, uint32_t elapsed_ms, uint16_t total_ms) {
  if (total_ms == 0) return b;
  if (elapsed_ms >= total_ms) return b;
  // Linear interpolation: a + (b - a) * (elapsed / total)
  int32_t delta = (int32_t)b - (int32_t)a;
  int32_t result = (int32_t)a + (delta * (int32_t)elapsed_ms) / (int32_t)total_ms;
  if (result < 0) result = 0;
  if (result > 5000) result = 5000;
  return (uint16_t)result;
}

static inline void write_freq(uint16_t hz) {
  if (hz == g_last_freq) return;  // avoid redundant LEDC writes
  g_last_freq = hz;
  ledcWriteTone(LEDC_CHANNEL_0, hz);
}

static void start_track(const SoundStep* track, uint16_t len) {
  if (!g_sounds_enabled) return;
  g_track = track;
  g_track_len = len;
  g_step = 0;
  g_step_started_ms = millis();
  g_last_freq_update_ms = g_step_started_ms;
  write_freq(track[0].start_hz);
}

void gary_sounds_begin() {
  write_freq(0);
  g_track = nullptr;
  g_track_len = 0;
  g_step = 0;
  g_sounds_enabled = true;
}

void gary_sounds_set_enabled(bool en) {
  g_sounds_enabled = en;
  if (!en) {
    write_freq(0);
    g_track = nullptr;
  }
}

bool gary_sounds_is_playing() {
  return g_track != nullptr;
}

void gary_sounds_stop() {
  write_freq(0);
  g_track = nullptr;
  g_track_len = 0;
  g_step = 0;
}

void gary_sounds_update() {
  if (g_track == nullptr) return;

  uint32_t now = millis();
  const SoundStep& step = g_track[g_step];

  uint32_t elapsed = now - g_step_started_ms;

  // Time to advance to next step?
  if (elapsed >= step.duration_ms) {
    g_step++;
    if (g_step >= g_track_len) {
      write_freq(0);
      g_track = nullptr;
      g_track_len = 0;
      g_step = 0;
      return;
    }
    g_step_started_ms = now;
    g_last_freq_update_ms = now;
    write_freq(g_track[g_step].start_hz);
    return;
  }

  // Within current step. If it's a sweep, update frequency periodically.
  if (step.start_hz != step.end_hz) {
    if (now - g_last_freq_update_ms >= UPDATE_PERIOD_MS) {
      uint16_t hz = lerp_freq(step.start_hz, step.end_hz, elapsed, step.duration_ms);
      write_freq(hz);
      g_last_freq_update_ms = now;
    }
  }
  // For held tones (start == end), no need to re-write — already set on entry.
}

// =============================================================================
// TRACKS — Sweep-based R2-D2 sounds
//
// Format: { start_hz, end_hz, duration_ms }
//   - start == end: held tone
//   - start < end: rising sweep
//   - start > end: falling sweep
//   - start == end == 0: silence (rest)
// =============================================================================

// CHIRP_HAPPY — quick "okay!" chirp
static const SoundStep TRK_CHIRP_HAPPY[] = {
  { 900, 1600, 90},     // rising chirp
  {   0,    0, 20},     // brief rest
  {1600, 1900, 60},     // upward tail
};

// CHIRP_SNAPPY — fast double-blip "got it"
static const SoundStep TRK_CHIRP_SNAPPY[] = {
  {2200, 2600, 50},
  {   0,    0, 30},
  {2200, 2700, 70},
};

// WHISTLE_CURIOUS — slow ascending warble (the classic "what is that?")
static const SoundStep TRK_WHISTLE_CURIOUS[] = {
  { 700, 1800, 280},    // long upward sweep
  {1800, 1700, 60},     // tiny dip
  {1700, 2100, 120},    // little flick up
};

// WHISTLE_EXCITED — fast rising trill
static const SoundStep TRK_WHISTLE_EXCITED[] = {
  {1200, 1800,  60},
  {1800, 1300,  60},
  {1300, 2000,  60},
  {2000, 1400,  60},
  {1400, 2400, 200},    // big rise to finish
};

// CHATTER_PLAYFUL — varied bleeps that sound like talking
static const SoundStep TRK_CHATTER_PLAYFUL[] = {
  {1200, 1500,  90},
  {   0,    0,  30},
  {1800, 1600, 120},
  {   0,    0,  30},
  {1400, 1900, 100},
  {   0,    0,  40},
  {2000, 1700,  90},
  {   0,    0,  20},
  {1500, 1700,  80},
};

// PROUD_FANFARE — triumphant
static const SoundStep TRK_PROUD_FANFARE[] = {
  { 900, 1100, 140},
  {   0,    0,  30},
  {1300, 1500, 140},
  {   0,    0,  30},
  {1700, 2200, 280},    // climactic rise
};

// UNSURE_DESCEND — slow falling "I don't know"
static const SoundStep TRK_UNSURE_DESCEND[] = {
  {1400, 1300, 150},
  {   0,    0,  20},
  {1100,  700, 320},    // long descending question
};

// CONCERNED_LOW — low warble (uneasy)
static const SoundStep TRK_CONCERNED_LOW[] = {
  { 450, 650, 80},
  { 650, 450, 80},
  { 450, 650, 80},
  { 650, 450, 80},
  { 450, 600, 100},
  { 600, 400, 120},
};

// SAD_DESCEND — long downward droop
static const SoundStep TRK_SAD_DESCEND[] = {
  {1300, 380, 700},     // big slow descent
  {   0,   0,  40},
  { 380, 280, 250},     // final sigh
};

// SCREAM — erratic + descending finish (alarmed/startled)
static const SoundStep TRK_SCREAM[] = {
  {2200, 2900,  60},
  {2900, 2100,  60},
  {2100, 2900,  60},
  {2900, 2000,  60},
  {2000, 2800,  60},
  {2800, 1700,  60},
  {1700, 2400,  60},
  {2400,  600, 250},    // big crash down
};

// SHOCKED_BLAST — sharp high pulse + dip
static const SoundStep TRK_SHOCKED_BLAST[] = {
  {2900, 2900, 120},    // held high
  {   0,    0,  40},
  {2400, 1800, 200},    // disappointed dip
};

// DETERMINED_GROWL — low rising "let's go"
static const SoundStep TRK_DETERMINED_GROWL[] = {
  {280, 360, 80},
  {360, 280, 80},
  {280, 360, 80},
  {360, 280, 80},
  {280, 800, 350},      // ramp up to fight pitch
};

// LAUGHTER — bouncy chuckle (NOT painful smoke-alarm beeping)
static const SoundStep TRK_LAUGHTER[] = {
  { 700, 1000,  90}, {0, 0, 30},
  { 700, 1000,  90}, {0, 0, 30},
  { 700, 1000,  90}, {0, 0, 80},
  { 800, 1100,  90}, {0, 0, 30},
  { 800, 1100,  90}, {0, 0, 30},
  { 800, 1100, 110},
};

// PROCESSING — VIBRATO hum (no longer a flat 950Hz drone!)
// Oscillates ±60Hz around 900Hz to sound "alive" rather than alarming
static const SoundStep TRK_PROCESSING[] = {
  {870, 950, 150},
  {950, 870, 150},
  {870, 950, 150},
  {950, 870, 150},
};

// WHISTLE — gentle mid sweep
static const SoundStep TRK_WHISTLE[] = {
  {1000, 1500, 200},
  {1500, 1300, 100},
};

// ACKNOWLEDGE — short affirmative beep
static const SoundStep TRK_ACKNOWLEDGE[] = {
  {1500, 1700, 80},
};

// QUERY — "huh?" two-note rising
static const SoundStep TRK_QUERY[] = {
  {1200, 1300, 100},
  {   0,    0,  30},
  {1500, 1900, 180},    // upward question inflection
};

// ALERT — insistent attention-getter
static const SoundStep TRK_ALERT[] = {
  {1800, 2400,  90}, {0, 0, 40},
  {1800, 2400,  90}, {0, 0, 40},
  {1800, 2400,  90}, {0, 0, 40},
  {2400, 1800, 150},
};

#define TRACK_LEN(arr) (sizeof(arr) / sizeof(arr[0]))

void gary_chirp_happy()          { start_track(TRK_CHIRP_HAPPY,        TRACK_LEN(TRK_CHIRP_HAPPY)); }
void gary_chirp_snappy()         { start_track(TRK_CHIRP_SNAPPY,       TRACK_LEN(TRK_CHIRP_SNAPPY)); }
void gary_whistle_curious()      { start_track(TRK_WHISTLE_CURIOUS,    TRACK_LEN(TRK_WHISTLE_CURIOUS)); }
void gary_whistle_excited()      { start_track(TRK_WHISTLE_EXCITED,    TRACK_LEN(TRK_WHISTLE_EXCITED)); }
void gary_chatter_playful()      { start_track(TRK_CHATTER_PLAYFUL,    TRACK_LEN(TRK_CHATTER_PLAYFUL)); }
void gary_proud_fanfare()        { start_track(TRK_PROUD_FANFARE,      TRACK_LEN(TRK_PROUD_FANFARE)); }
void gary_unsure_descend()       { start_track(TRK_UNSURE_DESCEND,     TRACK_LEN(TRK_UNSURE_DESCEND)); }
void gary_concerned_low()        { start_track(TRK_CONCERNED_LOW,      TRACK_LEN(TRK_CONCERNED_LOW)); }
void gary_sad_descend()          { start_track(TRK_SAD_DESCEND,        TRACK_LEN(TRK_SAD_DESCEND)); }
void gary_scream()               { start_track(TRK_SCREAM,             TRACK_LEN(TRK_SCREAM)); }
void gary_shocked_blast()        { start_track(TRK_SHOCKED_BLAST,      TRACK_LEN(TRK_SHOCKED_BLAST)); }
void gary_determined_growl()     { start_track(TRK_DETERMINED_GROWL,   TRACK_LEN(TRK_DETERMINED_GROWL)); }
void gary_laughter()             { start_track(TRK_LAUGHTER,           TRACK_LEN(TRK_LAUGHTER)); }
void gary_processing()           { start_track(TRK_PROCESSING,         TRACK_LEN(TRK_PROCESSING)); }
void gary_whistle()              { start_track(TRK_WHISTLE,            TRACK_LEN(TRK_WHISTLE)); }
void gary_acknowledge()          { start_track(TRK_ACKNOWLEDGE,        TRACK_LEN(TRK_ACKNOWLEDGE)); }
void gary_query()                { start_track(TRK_QUERY,              TRACK_LEN(TRK_QUERY)); }
void gary_alert()                { start_track(TRK_ALERT,              TRACK_LEN(TRK_ALERT)); }
