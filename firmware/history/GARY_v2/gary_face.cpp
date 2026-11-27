// =============================================================================
// gary_face.cpp — Star Wars-themed animations for Gary's matrix
//
// Each anim_*() sets g_anim_kind, and gary_face_update() drives the animation.
// Animations loop until replaced by another anim_*() call.
//
// Buffer is 16 columns × 8 rows. Each byte = column, bit 0 = top, bit 7 = bottom.
// =============================================================================

#include "gary_face.h"
#include "WMMatrixLed.h"
#include <Arduino.h>
#include <string.h>

static const uint8_t MATRIX_SCK_PIN = 33;
static const uint8_t MATRIX_DIN_PIN = 32;

static WMMatrixLed g_matrix(MATRIX_SCK_PIN, MATRIX_DIN_PIN);
static bool g_face_enabled = true;

enum AnimKind {
  ANIM_NONE,
  // Originals
  ANIM_SCANNER,
  ANIM_SENSOR_GRID,
  ANIM_PROGRESS_BARS,
  ANIM_RADAR,
  ANIM_CONVERGING,
  ANIM_STROBE,
  ANIM_RED_ALERT,
  ANIM_SOLID_OFF,
  ANIM_FULL_BRIGHT,
  ANIM_SCROLL,
  // Star Wars additions
  ANIM_LIGHTSABER,
  ANIM_DEATH_STAR,
  ANIM_X_WING,
  ANIM_TIE_FIGHTER,
  ANIM_IMPERIAL_COG,
  ANIM_REBEL_STARBIRD,
  ANIM_HYPERSPACE,
  ANIM_CONTROL_PANEL,
  ANIM_CRAWL,
  // More control-panel
  ANIM_PULSE_SQUARES,
  ANIM_TARGET_HUD,
  ANIM_RADAR_ROOM,
  ANIM_DIAGNOSTIC,
  ANIM_DISTANCE,    // parameterized: shows distance bar
  ANIM_BATTERY,     // parameterized: shows battery fuel gauge
};

static AnimKind g_anim_kind = ANIM_NONE;
static uint32_t g_anim_last_step_ms = 0;
static uint16_t g_anim_step = 0;

// Scroll state (horizontal)
static char g_scroll_buf[24];
static uint16_t g_distance_cm = 0;
static uint16_t g_battery_mv = 8400;
static int16_t g_scroll_x = 16;
static const uint16_t SCROLL_INTERVAL_MS = 90;

// Crawl state (vertical, multi-line)
static const char* CRAWL_LINES[] = {
  "GARY", "MK1", "ESP32", "REBEL"
};
static const uint8_t CRAWL_LINE_COUNT = sizeof(CRAWL_LINES) / sizeof(CRAWL_LINES[0]);

// =============================================================================
// PUBLIC API
// =============================================================================

void gary_face_begin() {
  g_matrix.setBrightness(5);
  g_matrix.clearScreen();
  g_matrix.setColorIndex(1);
  g_face_enabled = true;
  g_anim_kind = ANIM_NONE;
}

void gary_face_set_enabled(bool en) {
  g_face_enabled = en;
  if (!en) {
    g_matrix.clearScreen();
    g_anim_kind = ANIM_NONE;
  }
}

void gary_face_brightness(uint8_t b) {
  if (b > 8) b = 8;
  g_matrix.setBrightness(b);
}

void gary_face_clear() {
  if (!g_face_enabled) return;
  g_matrix.clearScreen();
}

bool gary_face_is_scrolling() {
  return g_anim_kind == ANIM_SCROLL || g_anim_kind == ANIM_CRAWL;
}

void face_scroll_text(const char* text) {
  if (!g_face_enabled) return;
  uint8_t i = 0;
  while (text[i] && i < (sizeof(g_scroll_buf) - 1)) {
    g_scroll_buf[i] = text[i];
    i++;
  }
  g_scroll_buf[i] = '\0';
  g_scroll_x = 16;
  g_anim_kind = ANIM_SCROLL;
  g_anim_last_step_ms = millis();
  g_matrix.clearScreen();
  g_matrix.drawStr(g_scroll_x, 8, g_scroll_buf);
}

static void start_anim(AnimKind k) {
  g_anim_kind = k;
  g_anim_step = 0;
  g_anim_last_step_ms = millis();
  g_matrix.clearScreen();
}

// Originals
void anim_scanner()        { start_anim(ANIM_SCANNER); }
void anim_sensor_grid()    { start_anim(ANIM_SENSOR_GRID); }
void anim_progress_bars()  { start_anim(ANIM_PROGRESS_BARS); }
void anim_radar()          { start_anim(ANIM_RADAR); }
void anim_converging()     { start_anim(ANIM_CONVERGING); }
void anim_strobe()         { start_anim(ANIM_STROBE); }
void anim_red_alert()      { start_anim(ANIM_RED_ALERT); }
void anim_solid_off()      { start_anim(ANIM_SOLID_OFF); }
void anim_full_bright()    { start_anim(ANIM_FULL_BRIGHT); }

// Star Wars
void anim_lightsaber()     { start_anim(ANIM_LIGHTSABER); }
void anim_death_star()     { start_anim(ANIM_DEATH_STAR); }
void anim_x_wing()         { start_anim(ANIM_X_WING); }
void anim_tie_fighter()    { start_anim(ANIM_TIE_FIGHTER); }
void anim_imperial_cog()   { start_anim(ANIM_IMPERIAL_COG); }
void anim_rebel_starbird() { start_anim(ANIM_REBEL_STARBIRD); }
void anim_hyperspace()     { start_anim(ANIM_HYPERSPACE); }
void anim_control_panel()  { start_anim(ANIM_CONTROL_PANEL); }
void anim_crawl()          { start_anim(ANIM_CRAWL); }
void anim_pulse_squares()  { start_anim(ANIM_PULSE_SQUARES); }
void anim_target_hud()     { start_anim(ANIM_TARGET_HUD); }
void anim_radar_room()     { start_anim(ANIM_RADAR_ROOM); }
void anim_diagnostic()     { start_anim(ANIM_DIAGNOSTIC); }

void anim_distance(uint16_t cm) {
  g_distance_cm = cm;
  start_anim(ANIM_DISTANCE);
}

void anim_battery(uint16_t mv) {
  g_battery_mv = mv;
  start_anim(ANIM_BATTERY);
}

// =============================================================================
// FRAME RENDERERS
// =============================================================================

static void blit(uint8_t* buf) {
  if (!g_face_enabled) return;
  g_matrix.drawBitmap(0, 0, 16, buf);
}

// ---- Originals (unchanged) ----

static void render_scanner() {
  uint8_t buf[16] = {0};
  uint8_t pos;
  uint16_t stp = g_anim_step % 32;
  if (stp < 16) pos = stp;
  else pos = 31 - stp;
  buf[pos] = 0xFF;
  if (pos > 0) buf[pos - 1] = 0x18;
  if (pos < 15) buf[pos + 1] = 0x18;
  blit(buf);
}

static void render_sensor_grid() {
  uint8_t buf[16] = {0};
  bool phase = (g_anim_step / 6) & 1;
  uint8_t pattern = (1 << 1) | (1 << 4) | (1 << 7);
  for (uint8_t c = 0; c < 16; c++) {
    if ((c & 1) == phase) buf[c] = pattern;
  }
  blit(buf);
}

static void render_progress_bars() {
  uint8_t buf[16] = {0};
  uint16_t total = 16 * 4;
  uint16_t stp = g_anim_step % total;
  uint8_t active_col = stp / 4;
  uint8_t fill_lvl = (stp % 4) + 1;
  for (uint8_t c = 0; c < active_col; c++) {
    buf[c] = 0xFF;
  }
  uint8_t pattern = 0;
  for (uint8_t i = 0; i < fill_lvl * 2; i++) {
    pattern |= (1 << (7 - i));
  }
  buf[active_col] = pattern;
  blit(buf);
}

static void render_radar() {
  uint8_t buf[16] = {0};
  uint16_t phase = g_anim_step % 24;
  for (uint8_t c = 0; c < 16; c++) {
    uint8_t row = (c + phase) % 8;
    buf[c] = (1 << row);
  }
  blit(buf);
}

static void render_converging() {
  uint8_t buf[16] = {0};
  uint16_t total = 32;
  uint16_t stp = g_anim_step % total;
  uint8_t offset;
  if (stp < 16) offset = stp / 2;
  else offset = (stp - 16) / 2;
  uint8_t left, right;
  if (stp < 16) {
    left = offset;
    right = 15 - offset;
  } else {
    left = 7 - offset;
    right = 8 + offset;
  }
  uint8_t pattern = 0x3C;
  buf[left] = pattern;
  buf[right] = pattern;
  blit(buf);
}

static void render_strobe() {
  uint8_t buf[16];
  bool on = (g_anim_step & 1);
  for (uint8_t c = 0; c < 16; c++) buf[c] = on ? 0xFF : 0x00;
  blit(buf);
}

static void render_red_alert() {
  uint8_t buf[16] = {0};
  bool left = (g_anim_step & 1);
  if (left) {
    for (uint8_t c = 0; c < 8; c++) buf[c] = 0xFF;
  } else {
    for (uint8_t c = 8; c < 16; c++) buf[c] = 0xFF;
  }
  blit(buf);
}

// ---- Star Wars renderers ----

// LIGHTSABER — blade grows from bottom up, then shimmers / extends + retracts
//   Phase 0-7: ignition (blade extending up the matrix center cols 7-8)
//   Phase 8-23: hold + flicker (occasional dimming/brightening)
//   Phase 24-31: retraction (blade shrinks back down)
//   Loops.
static void render_lightsaber() {
  uint8_t buf[16] = {0};
  uint16_t stp = g_anim_step % 32;

  // Compute blade height (rows visible from bottom up)
  uint8_t blade_height = 0;
  if (stp < 8) blade_height = stp + 1;             // 1..8 (igniting)
  else if (stp < 24) blade_height = 8;             // hold full
  else blade_height = 32 - stp;                    // 8..1 (retracting)

  // Build blade pattern: bits set from bottom up
  uint8_t blade_pat = 0;
  for (uint8_t i = 0; i < blade_height; i++) {
    blade_pat |= (1 << (7 - i));
  }

  // Center 2 cols = blade
  buf[7] = blade_pat;
  buf[8] = blade_pat;

  // Hilt at the very bottom (cols 5-10, row 7) — visible always
  // (No actual hilt rendering since blade fills bottom; show small hilt cross at row 7)
  // Skip — keeps blade pure. If desired, add: buf[5] |= 0x80; buf[10] |= 0x80;

  // Flicker effect during hold phase: every 4 frames dim one center col
  if (stp >= 8 && stp < 24) {
    if ((stp & 3) == 0) buf[7] = blade_pat & 0xFE;  // drop top pixel briefly
  }

  blit(buf);
}

// DEATH_STAR — full silhouette with pulsing superlaser dish position
static void render_death_star() {
  // Base bitmap — modified per frame to "aim" the superlaser
  static const uint8_t DS_BASE[16] = {
    0x38, 0x44, 0x82, 0x82, 0x81, 0x81, 0x81, 0xB1,
    0xB1, 0x81, 0x81, 0x81, 0x82, 0x82, 0x44, 0x38
  };
  uint8_t buf[16];
  memcpy(buf, DS_BASE, 16);

  // Add pulsing "charging" indicator at the dish (cols 7-8, rows 4-5)
  uint16_t pulse = (g_anim_step / 4) & 3;
  if (pulse == 0) {
    // Off
  } else if (pulse == 1) {
    buf[7] |= 0x10;  // row 4
  } else if (pulse == 2) {
    buf[7] |= 0x18;  // rows 3-4
    buf[8] |= 0x18;
  } else {
    buf[7] |= 0x3C;  // rows 2-5 — fully charged!
    buf[8] |= 0x3C;
  }
  blit(buf);
}

// X-WING — flies right-to-left with subtle vertical bob
static void render_x_wing() {
  static const uint8_t XW[16] = {
    0x00, 0x00, 0x00, 0x54, 0xFE, 0xFE, 0x7C, 0x38,
    0x38, 0x38, 0x10, 0x54, 0xD6, 0xC6, 0x44, 0x00
  };
  uint8_t buf[16] = {0};

  // Position cycles: ship moves left across screen
  // Total range = 32 frames, x ranges 16..-16
  int16_t x = 16 - (int16_t)(g_anim_step % 32);

  // Subtle vertical bob: ±1 row
  int8_t y_bob = ((g_anim_step / 4) & 1) ? 0 : 1;

  for (int16_t i = 0; i < 16; i++) {
    int16_t target = i + x;
    if (target < 0 || target >= 16) continue;
    uint8_t src = XW[i];
    // Apply vertical bob
    if (y_bob) src = (src << 1);
    buf[target] = src;
  }
  blit(buf);
}

// TIE FIGHTER — flies left-to-right
static void render_tie_fighter() {
  static const uint8_t TF[16] = {
    0x7E, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x18, 0x3C,
    0x3C, 0x18, 0x00, 0x00, 0x7E, 0x7E, 0x00, 0x00
  };
  uint8_t buf[16] = {0};

  int16_t x = -16 + (int16_t)(g_anim_step % 32);

  // Engine flicker: cols 6,9 (cockpit ball outer) flicker every other frame
  bool flicker = (g_anim_step & 1);

  for (int16_t i = 0; i < 16; i++) {
    int16_t target = i + x;
    if (target < 0 || target >= 16) continue;
    uint8_t src = TF[i];
    if (flicker && (i == 6 || i == 9)) src ^= 0x18;  // flicker cockpit outer
    buf[target] = src;
  }
  blit(buf);
}

// IMPERIAL COG — full bitmap with subtle rotation suggestion
static void render_imperial_cog() {
  static const uint8_t IC_BASE[16] = {
    0x00, 0x00, 0x18, 0x18, 0x66, 0xFF, 0x81, 0x99,
    0x99, 0x81, 0xFF, 0x66, 0x18, 0x18, 0x00, 0x00
  };
  uint8_t buf[16];
  memcpy(buf, IC_BASE, 16);

  // Pulse animation: every 8 frames, intensify the spokes by adding extras
  uint16_t phase = (g_anim_step / 8) & 1;
  if (phase) {
    // Brighten the inner cross
    buf[6] |= 0x18;
    buf[9] |= 0x18;
  } else {
    // Brighten outer corners
    buf[2] |= 0x66;
    buf[13] |= 0x66;
  }
  blit(buf);
}

// REBEL STARBIRD — wings pulse (flap)
static void render_rebel_starbird() {
  // Two slightly different wing positions
  static const uint8_t RS_UP[16] = {
    0x0C, 0x1C, 0x38, 0x30, 0x70, 0x78, 0x78, 0x78,
    0x58, 0x28, 0x38, 0x30, 0x30, 0x18, 0x1C, 0x1C
  };
  static const uint8_t RS_DOWN[16] = {
    // Same shape but wings tucked slightly
    0x04, 0x0C, 0x18, 0x30, 0x70, 0x78, 0x78, 0x78,
    0x58, 0x28, 0x38, 0x30, 0x18, 0x18, 0x0C, 0x04
  };
  uint8_t buf[16];
  bool up = ((g_anim_step / 6) & 1);
  memcpy(buf, up ? RS_UP : RS_DOWN, 16);
  blit(buf);
}

// HYPERSPACE — streaks emanating from center, growing outward
static void render_hyperspace() {
  uint8_t buf[16] = {0};
  uint16_t phase = g_anim_step % 8;

  // Center
  const int8_t cx = 7;
  const int8_t cy = 3;

  // 8 directional streaks
  static const int8_t DIRS[8][2] = {
    {-1,  0}, {-1, 1}, {0, 1}, {1, 1},
    {1,  0}, {1, -1}, {0, -1}, {-1, -1}
  };

  for (uint8_t d = 0; d < 8; d++) {
    int8_t dy = DIRS[d][0];
    int8_t dx = DIRS[d][1];
    for (int8_t step = 0; step <= phase; step++) {
      int8_t y = cy + dy * step;
      int8_t x = cx + dx * step;
      if (y >= 0 && y < 8 && x >= 0 && x < 16) {
        buf[x] |= (1 << y);
      }
    }
  }
  blit(buf);
}

// CONTROL_PANEL — busy dashboard with multiple regions blinking at different rates
//   Section 1 (cols 0-3): vertical bar gauges with varying levels
//   Section 2 (cols 4-7): scattered status dots blinking individually
//   Section 3 (cols 8-11): horizontal scanning bar
//   Section 4 (cols 12-15): radar pulse rings
static void render_control_panel() {
  uint8_t buf[16] = {0};
  uint16_t s = g_anim_step;

  // Section 1: gauges (cols 0-3) — each at different fill levels that shift
  //   col 0 oscillates 2-7, col 1 oscillates 1-6, col 2 holds at 4, col 3 oscillates 0-5
  uint8_t lvl0 = ((s + 0) % 12 < 6) ? ((s + 0) % 6) + 2 : 7 - ((s + 0) % 6);
  uint8_t lvl1 = ((s + 3) % 10 < 5) ? ((s + 3) % 5) + 1 : 5 - ((s + 3) % 5);
  uint8_t lvl2 = 4;
  uint8_t lvl3 = ((s + 1) % 8 < 4) ? ((s + 1) % 4) : 3 - ((s + 1) % 4);

  uint8_t levels[4] = {lvl0, lvl1, lvl2, lvl3};
  for (uint8_t c = 0; c < 4; c++) {
    uint8_t lv = levels[c];
    uint8_t pattern = 0;
    for (uint8_t i = 0; i < lv; i++) {
      pattern |= (1 << (7 - i));
    }
    buf[c] = pattern;
  }

  // Section 2: status dots (cols 4-7) — checkerboard of dots blinking individually
  // 8 dots at positions (col, row): each toggles based on different prime modulus
  // for chaotic but deterministic blinking
  if ((s % 5) == 0) buf[4] |= 0x02;  // (4,1)
  if ((s % 7) < 4)  buf[4] |= 0x40;  // (4,6)
  if ((s % 3) == 0) buf[5] |= 0x10;  // (5,4)
  if ((s % 4) < 2)  buf[5] |= 0x01;  // (5,0)
  if ((s % 11) < 5) buf[6] |= 0x08;  // (6,3)
  if ((s % 6) < 3)  buf[6] |= 0x80;  // (6,7)
  if ((s % 9) < 4)  buf[7] |= 0x04;  // (7,2)
  if ((s % 13) < 6) buf[7] |= 0x20;  // (7,5)

  // Section 3: scanning bar (cols 8-11) — vertical bar sweeps left-right
  uint8_t scan_pos = 8 + (s % 8 < 4 ? s % 4 : 3 - (s % 4));
  buf[scan_pos] |= 0xFF;  // full vertical bar at scan position
  // dimmer trail
  if (scan_pos > 8) buf[scan_pos - 1] |= 0x24;

  // Section 4: radar rings (cols 12-15) — concentric pulse expanding outward
  uint8_t radar_phase = (s / 3) & 3;
  if (radar_phase == 0) {
    buf[13] |= 0x18; buf[14] |= 0x18;  // tight center
  } else if (radar_phase == 1) {
    buf[13] |= 0x3C; buf[14] |= 0x3C;
  } else if (radar_phase == 2) {
    buf[12] |= 0x24; buf[13] |= 0x42; buf[14] |= 0x42; buf[15] |= 0x24;
  } else {
    buf[12] |= 0x81; buf[15] |= 0x81;
  }

  blit(buf);
}

// CRAWL — Star Wars opening crawl: multi-line text scrolls bottom-to-top
//   Each "line" is one of CRAWL_LINES, displayed centered.
//   At any time we show the current line in different vertical positions
//   creating an upward scroll feel.
static void render_crawl() {
  // We display one line at a time, scrolling it up the matrix.
  // Each line takes ~24 frames: enters from bottom, exits at top.
  uint16_t total_per_line = 16;
  uint16_t total_cycle = total_per_line * CRAWL_LINE_COUNT;
  uint16_t s = g_anim_step % total_cycle;

  uint8_t line_idx = s / total_per_line;
  uint8_t line_phase = s % total_per_line;  // 0..15

  // Vertical position: line starts off-screen below (y > 8), slides up to off-screen top (y < 0)
  // y_top of text = 16 - line_phase (where 8 means top of matrix)
  int16_t y = 16 - (int16_t)line_phase;  // 16, 15, ..., 1

  g_matrix.clearScreen();
  // drawStr y-coord is the BASELINE in pixels. Text is 8 tall.
  g_matrix.drawStr(0, y, CRAWL_LINES[line_idx]);
  // Note: blit() not used here because drawStr renders directly
}

// PULSE_SQUARES - concentric squares pulse outward at 3 staggered phases
static void render_pulse_squares() {
  uint8_t buf[16] = {0};
  const int8_t cx = 7;
  const int8_t cy = 3;
  uint16_t s = g_anim_step;
  static const int8_t OFFSETS[3] = {0, 2, 4};
  for (uint8_t i = 0; i < 3; i++) {
    int8_t p = (s + OFFSETS[i]) % 6;
    if (p == 0) continue;
    int8_t size = p;
    for (int8_t dy = -size; dy <= size; dy++) {
      for (int8_t dx = -size; dx <= size; dx++) {
        if (abs(dy) == size || abs(dx) == size) {
          int8_t y = cy + dy;
          int8_t x = cx + dx;
          if (y >= 0 && y < 8 && x >= 0 && x < 16) {
            buf[x] |= (1 << y);
          }
        }
      }
    }
  }
  blit(buf);
}

// TARGET_HUD - corner brackets, center crosshair, moving reticle, status pulses
static void render_target_hud() {
  uint8_t buf[16] = {0};
  uint16_t s = g_anim_step;

  // Corner brackets (L-shapes pointing inward)
  buf[0]  |= 0b00000111;
  buf[1]  |= 0b00000001;
  buf[2]  |= 0b00000001;
  buf[15] |= 0b00000111;
  buf[14] |= 0b00000001;
  buf[13] |= 0b00000001;
  buf[0]  |= 0b11100000;
  buf[1]  |= 0b10000000;
  buf[2]  |= 0b10000000;
  buf[15] |= 0b11100000;
  buf[14] |= 0b10000000;
  buf[13] |= 0b10000000;

  // Center crosshair
  buf[7] |= 0b00011000;
  buf[8] |= 0b00011000;

  // Crosshair extending lines (blink)
  if ((s % 4) < 2) {
    buf[5]  |= 0b00011000;
    buf[10] |= 0b00011000;
  }

  // Moving reticle around perimeter
  static const int8_t TX[8] = {4, 11, 12, 12, 11, 4, 3, 3};
  static const int8_t TY[8] = {1,  1,  2,  5,  6, 6, 5, 2};
  uint8_t idx = s % 8;
  buf[TX[idx]] |= (1 << TY[idx]);
  if ((s & 1) == 0) {
    uint8_t prev = (idx + 7) % 8;
    buf[TX[prev]] |= (1 << TY[prev]);
  }

  // Top/bottom status pulses
  if ((s % 6) < 3) {
    buf[7] |= 0b00000001;
    buf[8] |= 0b00000001;
  } else {
    buf[7] |= 0b10000000;
    buf[8] |= 0b10000000;
  }

  blit(buf);
}

// RADAR_ROOM - sweep line + range rings + persistent blips
static void render_radar_room() {
  uint8_t buf[16] = {0};
  const int8_t cx = 7;
  const int8_t cy = 3;
  uint16_t s = g_anim_step;

  // Inner range ring
  static const int8_t RING_X[8] = { 0,  1,  2,  1,  0, -1, -2, -1};
  static const int8_t RING_Y[8] = {-2, -1,  0,  1,  2,  1,  0, -1};
  for (uint8_t i = 0; i < 8; i++) {
    int8_t y = cy + RING_Y[i];
    int8_t x = cx + RING_X[i];
    if (y >= 0 && y < 8 && x >= 0 && x < 16) {
      buf[x] |= (1 << y);
    }
  }

  // Outer range markers
  if (cy - 3 >= 0) buf[cx - 4] |= (1 << (cy - 3));
  if (cy - 3 >= 0) buf[cx + 4] |= (1 << (cy - 3));
  if (cy + 3 < 8) buf[cx - 4] |= (1 << (cy + 3));
  if (cy + 3 < 8) buf[cx + 4] |= (1 << (cy + 3));

  // Sweep line
  static const int8_t SWEEP_DX[16] = { 0,  1,  2,  3,  4,  3,  2,  1,
                                       0, -1, -2, -3, -4, -3, -2, -1};
  static const int8_t SWEEP_DY[16] = {-3, -3, -2, -1,  0,  1,  2,  3,
                                       3,  3,  2,  1,  0, -1, -2, -3};
  uint8_t angle = s % 16;
  for (uint8_t d = 1; d <= 4; d++) {
    int8_t dy = (SWEEP_DY[angle] * d) / 4;
    int8_t dx = (SWEEP_DX[angle] * d) / 4;
    int8_t y = cy + dy;
    int8_t x = cx + dx;
    if (y >= 0 && y < 8 && x >= 0 && x < 16) {
      buf[x] |= (1 << y);
    }
  }

  // Persistent contact blips
  if ((s % 5) < 3) buf[2]  |= (1 << 0);
  if ((s % 7) < 4) buf[12] |= (1 << 1);
  if ((s % 9) < 5) buf[3]  |= (1 << 6);

  blit(buf);
}

// DIAGNOSTIC - 8 cells (4x2), each cycling pattern at different rate
static void render_diagnostic() {
  uint8_t buf[16] = {0};
  uint16_t s = g_anim_step;

  static const uint8_t MODS[8] = {3, 5, 7, 11, 4, 9, 6, 13};
  static const uint8_t PATTERNS[5][4] = {
    {0b0011, 0b0011, 0b0000, 0b0000},
    {0b0110, 0b1001, 0b1001, 0b0110},
    {0b1010, 0b0101, 0b1010, 0b0101},
    {0b1111, 0b1001, 0b1001, 0b1111},
    {0b0000, 0b0110, 0b0110, 0b0000},
  };

  for (uint8_t cell_y = 0; cell_y < 2; cell_y++) {
    for (uint8_t cell_x = 0; cell_x < 4; cell_x++) {
      uint8_t cell_idx = cell_y * 4 + cell_x;
      uint8_t mod = MODS[cell_idx];
      uint8_t sub_phase = (s + cell_idx * 2) % mod;
      uint8_t pat_idx = sub_phase % 5;
      uint8_t base_x = cell_x * 4;
      uint8_t base_y = cell_y * 4;

      for (uint8_t dy = 0; dy < 4; dy++) {
        uint8_t row_pat = PATTERNS[pat_idx][dy];
        for (uint8_t dx = 0; dx < 4; dx++) {
          if (row_pat & (1 << (3 - dx))) {
            uint8_t y = base_y + dy;
            uint8_t x = base_x + dx;
            if (y < 8 && x < 16) {
              buf[x] |= (1 << y);
            }
          }
        }
      }
    }
  }
  blit(buf);
}


// DISTANCE — sonar-style proximity bar graph
//   cm value mapped to bar fill (0-200cm range)
//   Closer = more bars + faster pulsing on the bar
//   Far/no-detect (>200cm or 0): minimal bar at left, slow pulse
static void render_distance() {
  uint8_t buf[16] = {0};
  uint16_t s = g_anim_step;
  uint16_t cm = g_distance_cm;

  // Map distance to fill: 200cm = 0 cols, 0cm = 16 cols
  int16_t fill = 16 - (int16_t)(cm / 12);  // 12cm per column
  if (cm == 0 || cm > 200) fill = 1;       // no-data fallback
  if (fill < 0) fill = 0;
  if (fill > 16) fill = 16;

  // Pulse rate: closer = faster pulse (more urgent)
  uint8_t pulse_period;
  if (fill >= 12)      pulse_period = 2;   // very close = very fast
  else if (fill >= 8)  pulse_period = 4;
  else if (fill >= 4)  pulse_period = 6;
  else                 pulse_period = 10;  // far = slow

  bool pulse_on = (s % pulse_period) < (pulse_period / 2);

  // Solid bar for the filled portion (rows 2-5)
  for (int8_t c = 0; c < fill; c++) {
    if (c == fill - 1 && pulse_on) {
      // Leading edge pulses brighter (full vertical)
      buf[c] = 0xFF;
    } else {
      buf[c] = 0b00111100;  // rows 2-5
    }
  }

  // Tick marks at top/bottom for the rest (visual scale)
  for (int8_t c = fill; c < 16; c++) {
    if (c % 4 == 3) {  // every 4th column has tick marks
      buf[c] = 0b10000001;  // top + bottom dots
    }
  }

  blit(buf);
}

// BATTERY — horizontal fuel gauge (left to right) with charge level icon
//   mv: 8400 = full (8 cells), 6600 = empty (1 cell)
//   Animation: gentle pulse on the rightmost lit cell
static void render_battery() {
  uint8_t buf[16] = {0};
  uint16_t s = g_anim_step;
  uint16_t mv = g_battery_mv;

  // Map voltage to cells (0-12, since gauge is 12 cols wide)
  // 6600mV = 0 cells (empty), 8400mV = 12 cells (full)
  int16_t cells;
  if (mv >= 8400) cells = 12;
  else if (mv <= 6600) cells = 0;
  else cells = (int16_t)((mv - 6600) * 12 / 1800);
  if (cells < 0) cells = 0;
  if (cells > 12) cells = 12;

  // Outer battery casing (cols 1-14, top + bottom rows)
  for (uint8_t c = 1; c < 14; c++) {
    buf[c] |= 0b00000001;  // top row
    buf[c] |= 0b10000000;  // bottom row
  }
  // Left wall
  buf[1] |= 0b11111111;
  // Right wall (wraps to col 13)
  buf[13] |= 0b11111111;
  // Battery tip on right (positive terminal)
  buf[14] |= 0b00111100;
  buf[15] |= 0b00011000;

  // Fill cells from left (col 2) up to current cells
  bool pulse_on = (s & 1);
  for (int8_t c = 2; c < 2 + cells; c++) {
    bool is_leading = (c == 2 + cells - 1);
    if (is_leading && !pulse_on) {
      // Leading cell pulses
      buf[c] |= 0b00011000;  // dim (just middle)
    } else {
      buf[c] |= 0b01111110;  // full fill (rows 1-6)
    }
  }

  // Low-battery flash: if cells <= 2, blink the entire gauge
  if (cells <= 2 && pulse_on) {
    for (uint8_t c = 0; c < 16; c++) buf[c] = 0;
  }

  blit(buf);
}

// =============================================================================
// UPDATE — called every loop iteration
// =============================================================================

void gary_face_update() {
  if (!g_face_enabled) return;
  if (g_anim_kind == ANIM_NONE) return;

  uint32_t now = millis();

  uint16_t interval_ms;
  switch (g_anim_kind) {
    case ANIM_SCROLL:           interval_ms = SCROLL_INTERVAL_MS; break;
    case ANIM_SCANNER:          interval_ms = 80; break;
    case ANIM_SENSOR_GRID:      interval_ms = 60; break;
    case ANIM_PROGRESS_BARS:    interval_ms = 50; break;
    case ANIM_RADAR:            interval_ms = 90; break;
    case ANIM_CONVERGING:       interval_ms = 70; break;
    case ANIM_STROBE:           interval_ms = 90; break;
    case ANIM_RED_ALERT:        interval_ms = 350; break;
    case ANIM_SOLID_OFF:        return;
    case ANIM_FULL_BRIGHT:      return;
    // Star Wars
    case ANIM_LIGHTSABER:       interval_ms = 70; break;
    case ANIM_DEATH_STAR:       interval_ms = 120; break;
    case ANIM_X_WING:           interval_ms = 90; break;
    case ANIM_TIE_FIGHTER:      interval_ms = 90; break;
    case ANIM_IMPERIAL_COG:     interval_ms = 120; break;
    case ANIM_REBEL_STARBIRD:   interval_ms = 100; break;
    case ANIM_HYPERSPACE:       interval_ms = 80; break;
    case ANIM_CONTROL_PANEL:    interval_ms = 90; break;
    case ANIM_CRAWL:            interval_ms = 110; break;
    case ANIM_PULSE_SQUARES:    interval_ms = 110; break;
    case ANIM_TARGET_HUD:       interval_ms = 90; break;
    case ANIM_RADAR_ROOM:       interval_ms = 80; break;
    case ANIM_DIAGNOSTIC:       interval_ms = 90; break;
    case ANIM_DISTANCE:         interval_ms = 100; break;
    case ANIM_BATTERY:          interval_ms = 250; break;
    default:                    interval_ms = 100; break;
  }

  if (now - g_anim_last_step_ms < interval_ms) return;
  g_anim_last_step_ms = now;
  g_anim_step++;

  switch (g_anim_kind) {
    case ANIM_SCROLL: {
      g_scroll_x--;
      int16_t text_width = 6 * (int16_t)strlen(g_scroll_buf);
      if (g_scroll_x < -text_width) {
        g_matrix.clearScreen();
        g_anim_kind = ANIM_NONE;
        return;
      }
      g_matrix.clearScreen();
      g_matrix.drawStr(g_scroll_x, 8, g_scroll_buf);
      break;
    }
    case ANIM_SCANNER:        render_scanner(); break;
    case ANIM_SENSOR_GRID:    render_sensor_grid(); break;
    case ANIM_PROGRESS_BARS:  render_progress_bars(); break;
    case ANIM_RADAR:          render_radar(); break;
    case ANIM_CONVERGING:     render_converging(); break;
    case ANIM_STROBE:         render_strobe(); break;
    case ANIM_RED_ALERT:      render_red_alert(); break;
    case ANIM_SOLID_OFF: {
      uint8_t buf[16] = {0};
      blit(buf);
      g_anim_kind = ANIM_NONE;
      break;
    }
    case ANIM_FULL_BRIGHT: {
      uint8_t buf[16];
      for (uint8_t c = 0; c < 16; c++) buf[c] = 0xFF;
      blit(buf);
      g_anim_kind = ANIM_NONE;
      break;
    }
    // Star Wars
    case ANIM_LIGHTSABER:       render_lightsaber(); break;
    case ANIM_DEATH_STAR:       render_death_star(); break;
    case ANIM_X_WING:           render_x_wing(); break;
    case ANIM_TIE_FIGHTER:      render_tie_fighter(); break;
    case ANIM_IMPERIAL_COG:     render_imperial_cog(); break;
    case ANIM_REBEL_STARBIRD:   render_rebel_starbird(); break;
    case ANIM_HYPERSPACE:       render_hyperspace(); break;
    case ANIM_CONTROL_PANEL:    render_control_panel(); break;
    case ANIM_CRAWL:            render_crawl(); break;
    case ANIM_PULSE_SQUARES:    render_pulse_squares(); break;
    case ANIM_TARGET_HUD:       render_target_hud(); break;
    case ANIM_RADAR_ROOM:       render_radar_room(); break;
    case ANIM_DIAGNOSTIC:       render_diagnostic(); break;
    case ANIM_DISTANCE:         render_distance(); break;
    case ANIM_BATTERY:          render_battery(); break;
    default: break;
  }
}
