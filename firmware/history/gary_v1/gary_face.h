// =============================================================================
// gary_face.h
//
// Star Wars-themed matrix animations for Gary's K12 LED matrix (16x8 mono).
//
// All animations are NON-BLOCKING — triggering one returns immediately.
// gary_face_update() must be called every loop iteration to advance frames.
//
// Pin assignment: CLK=33, DIN=32 (per Hiwonder K12 dot matrix tutorial PDF).
// Uses the timing-patched WMMatrixLed.cpp (delayMicroseconds for ESP32).
// =============================================================================

#ifndef GARY_FACE_H
#define GARY_FACE_H

#include <stdint.h>

void gary_face_begin();
void gary_face_set_enabled(bool en);
void gary_face_brightness(uint8_t b);
void gary_face_clear();
void gary_face_update();
bool gary_face_is_scrolling();
void face_scroll_text(const char* text);

// -----------------------------------------------------------------------------
// State animations — call to start the animation, it loops until replaced.
// -----------------------------------------------------------------------------

// Original (kept):
void anim_scanner();        // slow horizontal sweep — IDLE
void anim_sensor_grid();    // pulsing grid of dots — LISTENING
void anim_progress_bars();  // bars filling column-by-column — EXECUTING
void anim_radar();          // diagonal radar sweep — ROAMING
void anim_converging();     // dots moving inward — COMING
void anim_strobe();         // rapid full-on/full-off — STARTLED
void anim_red_alert();      // alternating halves blink — ALARMED
void anim_solid_off();      // all pixels off
void anim_full_bright();    // all pixels on, no animation

// -----------------------------------------------------------------------------
// NEW Star Wars animations
// -----------------------------------------------------------------------------

void anim_lightsaber();     // blade ignites bottom-up, then shimmers — wake/boot
void anim_death_star();     // pulsing Death Star with rotating superlaser dot
void anim_x_wing();         // X-Wing flies across screen left-to-right
void anim_tie_fighter();    // TIE Fighter flies right-to-left (or patrols)
void anim_imperial_cog();   // Imperial cog with rotating spokes
void anim_rebel_starbird(); // Rebel starbird with wing-flap pulse
void anim_hyperspace();     // streaks radiate outward from center
void anim_control_panel();  // dashboard lights flickering at different rates
void anim_crawl();          // text scrolls bottom-to-top with squish (Star Wars crawl)

// MORE control-panel-style animations (multi-region, "lots happening" feel)
void anim_pulse_squares();  // concentric squares pulse outward at 3 phases
void anim_target_hud();     // crosshair + brackets + moving reticle target acquisition
void anim_radar_room();     // sweep line + persistent blips + range rings
void anim_diagnostic();     // 4-cell pixel pattern grid, each blinking at different rate

#endif // GARY_FACE_H
