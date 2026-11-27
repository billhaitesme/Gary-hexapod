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
//   1. Wake: "HELLO-HI-WONDER" or "Mister. Gary" → "I'm here"
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
#include <Bluepad32.h>

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
  STATE_CONTROLLER  // PS5 DualSense controller is in command
};

GaryState current_state = STATE_BOOT;
GaryState previous_state = STATE_BOOT;
uint32_t state_entered_ms = 0;
uint32_t last_voice_check_ms = 0;
uint32_t last_boredom_kick_ms = 0;
uint16_t last_sound_value = 0;

// Coming-mode sub-state
enum ComingPhase { COMING_SEARCH, COMING_APPROACH, COMING_ARRIVED };
ComingPhase coming_phase = COMING_SEARCH;

// LISTENING two-tier timeout tracker.
bool listening_got_command = false;

// Movement targets.
Velocity_t vel = {0.0f, 0.0f, 0.0f};
Vector_t pos = {0.0f, 0.0f, 1.5f};
Euler_t att = {0.0f, 0.0f, 0.0f};

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

// -----------------------------------------------------------------------------
// gary_delay(ms) — replacement for delay() that keeps the buzzer track alive.
// Use this in action sequences where you want sound to keep playing during
// the wait. For super-short waits (<10ms) plain delay() is fine.
// -----------------------------------------------------------------------------
void gary_delay(uint32_t ms) {
  uint32_t end_t = millis() + ms;
  while ((int32_t)(end_t - millis()) > 0) {
    gary_sounds_update();
    delay(2);
  }
}

void transition_to(GaryState new_state) {
  if (new_state == current_state) return;

  previous_state = current_state;
  current_state = new_state;
  state_entered_ms = millis();

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
// CONVENTIONS:
//   Euler_t att = {pitch_deg, roll_deg, yaw_deg}   <- DEGREES, PITCH FIRST
//   Vector_t pos = {x_cm, y_cm, z}                  <- z is body height
//   Velocity_t vel = {vx, vy, vz}                   <- vz is yaw rate
//
// MAPPING:
//   L stick:         walk + turn (vector-magnitude exponential blend)
//   R stick (alone): roll/pitch when stopped (degrees)
//   L1 + R stick Y:  body height
//   L1 + R stick X:  ignored
//   R1 + R stick X:  body lean (roll degrees)
//   R1 + R stick Y:  body shift forward / back
//   L1 + R1 chord:   trigger Action 6
//   L1 + D-pad L/R held:    strafe while held
//   L1 + D-pad L/R 2-tap:   strafe toggle
//   D-pad L/R alone: cycle action group
//   D-pad U/D:       step-over / squeeze gait modes
//   L2 / R2:         left/right hook
//   L3:              cycle speed cap
//   Cross:           execute selected action
//   Square: tap=dive_fwd, 2tap=Action 8
//   Circle: tap=dive_back, 2tap=spread
//   Triangle: tap=introduce, 2tap=dance
//   Select: flashlight toggle
//   PS: gyro toggle (tap), DISCONNECT (hold ~2s)
// ============================================================================

GamepadPtr myGamepad = nullptr;
bool controller_was_connected = false;
bool controller_muted = false;

// ----------------------------------------------------------------------------
// MOVEMENT TUNING — change these to taste
// ----------------------------------------------------------------------------
const float CTRL_WALK_SPEED_BASE = 2.0f;    // forward/back base velocity
const float CTRL_TURN_SPEED_BASE = 5.0f;   // yaw rate (degrees per gait step)
const float CTRL_R_ROLL_MAX_DEG  = 30.0f;   // R-stick X scaled to this
const float CTRL_R_PITCH_MAX_DEG = 30.0f;   // R-stick Y scaled to this
const float CTRL_R1_LEAN_MAX_DEG = 30.0f;   // R1+R stick X
const float CTRL_R1_SHIFT_MAX_CM = 1.5f;    // R1+R stick Y body translation
const float CTRL_HEIGHT_BASE     = 0.5f;
const float CTRL_HEIGHT_RANGE    = 5.0f;    // L1+R stick Y → 1.5 ± 2.0 = -0.5..3.5
const float CTRL_GYRO_PITCH_MAX  = 5.0f;
const float CTRL_GYRO_ROLL_MAX   = 5.0f;
const float CTRL_STICK_EXPONENT  = 1.6f;    // exponential curve power
const int   CTRL_STICK_DEADBAND  = 30;

// Continuous movement targets
float ctrl_target_vy = 0.0f;
float ctrl_target_vz = 0.0f;
float ctrl_target_vx = 0.0f;
float ctrl_target_pitch = 0.0f;   // DEGREES
float ctrl_target_roll  = 0.0f;   // DEGREES
float ctrl_target_height = CTRL_HEIGHT_BASE;
float ctrl_target_body_y = 0.0f;

// Strafe state
int8_t  ctrl_strafe_toggle_dir = 0;
uint32_t dpad_left_last_tap_ms = 0;
uint32_t dpad_right_last_tap_ms = 0;

// Gait mode
int8_t ctrl_gait_mode = 0;

// Edge-detection state
struct ButtonEdge {
  bool cross, circle, triangle, square;
  bool l1, r1, l2, r2, l3, r3;
  bool select_b, start_b, ps_b, mute_b;
  bool tp_click;
  uint8_t dpad;
} prev_btn = {0};

uint8_t ctrl_gait_speed_idx = 1;
const float CTRL_GAIT_SPEEDS[3] = {0.6f, 1.0f, 1.6f};
const char* CTRL_SPEED_NAMES[3] = {"SLOW", "NORMAL", "FAST"};

bool ctrl_combat_mode = false;
bool ctrl_flashlight_on = false;
bool ctrl_gyro_mode = false;

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

uint32_t last_ctrl_move_ms = 0;
const uint16_t CTRL_MOVE_INTERVAL_MS = 50;

const float CTRL_SMOOTHING = 0.35f;
float ctrl_smooth_vy = 0.0f;
float ctrl_smooth_vz = 0.0f;
float ctrl_smooth_vx = 0.0f;
float ctrl_smooth_pitch = 0.0f;
float ctrl_smooth_roll = 0.0f;
float ctrl_smooth_height = CTRL_HEIGHT_BASE;
float ctrl_smooth_body_y = 0.0f;

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
// Stick math
// ----------------------------------------------------------------------------

inline int apply_deadband(int v, int dz) {
  if (v >  dz) return v - dz;
  if (v < -dz) return v + dz;
  return 0;
}

// Per-axis float in [-1, 1] WITHOUT exponential (raw)
inline float stick_raw(int v) {
  v = apply_deadband(v, CTRL_STICK_DEADBAND);
  if (v == 0) return 0.0f;
  float f = (float)v / (512.0f - CTRL_STICK_DEADBAND);
  if (f >  1.0f) f =  1.0f;
  if (f < -1.0f) f = -1.0f;
  return f;
}

// Apply exponential to a single value (preserve sign)
inline float curve(float f) {
  float sign = (f < 0) ? -1.0f : 1.0f;
  return sign * powf(fabsf(f), CTRL_STICK_EXPONENT);
}

// VECTOR-MAGNITUDE exponential blend.
// Takes raw (lx, ly), returns curved (lx_out, ly_out) preserving angle.
// Magnitude is exponentialized; angle is preserved.
// This way 45-degree pegged stick gives full diagonal output, not partial.
void stick_blend_curve(float lx_in, float ly_in, float* lx_out, float* ly_out) {
  float mag = sqrtf(lx_in * lx_in + ly_in * ly_in);
  if (mag < 0.001f) {
    *lx_out = 0.0f;
    *ly_out = 0.0f;
    return;
  }
  if (mag > 1.0f) mag = 1.0f;
  float curved_mag = powf(mag, CTRL_STICK_EXPONENT);
  // Scale original vector by (curved_mag / raw_mag)
  float scale = curved_mag / mag;
  *lx_out = lx_in * scale;
  *ly_out = ly_in * scale;
}

void cycle_gait_speed() {
  ctrl_gait_speed_idx = (ctrl_gait_speed_idx + 1) % 3;
  Serial.printf("[CTRL] Speed cap: %s (%.2fx)\n",
                CTRL_SPEED_NAMES[ctrl_gait_speed_idx],
                CTRL_GAIT_SPEEDS[ctrl_gait_speed_idx]);
  say(CLIP_WHISTLE);
}

void cycle_gait_mode_up() {
  ctrl_gait_mode = (ctrl_gait_mode == 1) ? 0 : 1;
  Serial.printf("[CTRL] Gait: %s\n", ctrl_gait_mode == 1 ? "STEP_OVER" : "NORMAL");
  say(CLIP_SNAPPY);
}

void cycle_gait_mode_down() {
  ctrl_gait_mode = (ctrl_gait_mode == 2) ? 0 : 2;
  Serial.printf("[CTRL] Gait: %s\n", ctrl_gait_mode == 2 ? "SQUEEZE" : "NORMAL");
  say(CLIP_SNAPPY);
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

void toggle_gyro_mode() {
  ctrl_gyro_mode = !ctrl_gyro_mode;
  Serial.printf("[CTRL] Gyro: %s\n", ctrl_gyro_mode ? "ON" : "OFF");
  say(CLIP_PLAYFUL);
}

void cycle_action_group(int8_t direction) {
  if (direction > 0) ctrl_action_idx = (ctrl_action_idx + 1) % CTRL_ACTION_COUNT;
  else ctrl_action_idx = (ctrl_action_idx + CTRL_ACTION_COUNT - 1) % CTRL_ACTION_COUNT;
  Serial.printf("[CTRL] Action group: %d\n", ctrl_action_idx + 1);
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
      Serial.println("[CTRL] Strafe toggle CANCELED (direction change)");
    }
    return;
  }
  if (dir == 0) {
    if (ctrl_strafe_toggle_dir != 0) Serial.println("[CTRL] Strafe toggle OFF");
    ctrl_strafe_toggle_dir = 0;
  } else if (ctrl_strafe_toggle_dir == dir) {
    ctrl_strafe_toggle_dir = 0;
    Serial.println("[CTRL] Strafe toggle OFF (re-toggle)");
  } else {
    ctrl_strafe_toggle_dir = dir;
    Serial.printf("[CTRL] Strafe toggle %s\n", dir < 0 ? "LEFT" : "RIGHT");
    say(CLIP_SNAPPY);
  }
}

// ----------------------------------------------------------------------------
// Process one frame
// ----------------------------------------------------------------------------

void process_controller() {
  if (!myGamepad || !myGamepad->isConnected()) {
    if (controller_was_connected) {
      Serial.println("[CTRL] Connection lost, returning to voice mode");
      controller_was_connected = false;
      controller_muted = false;
      ctrl_combat_mode = false;
      ctrl_flashlight_on = false;
      ctrl_gyro_mode = false;
      ctrl_strafe_toggle_dir = 0;
      ctrl_gait_mode = 0;
      transition_to(STATE_IDLE);
    }
    return;
  }
  controller_was_connected = true;

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

  // PS button: tap = gyro toggle, hold ~2s = disconnect
  if (ps_b && !prev_btn.ps_b) ps_hold_start_ms = millis();
  if (!ps_b && prev_btn.ps_b) {
    uint32_t held = millis() - ps_hold_start_ms;
    if (held >= PS_HOLD_DISCONNECT_MS) {
      Serial.println("[CTRL] PS held - disconnecting");
      myGamepad->disconnect();
      ps_hold_start_ms = 0;
    } else {
      toggle_gyro_mode();
    }
  }

  // Mute button
  if (mute_b && !prev_btn.mute_b) {
    controller_muted = !controller_muted;
    Serial.printf("[CTRL] %s\n", controller_muted ? "MUTED" : "UNMUTED");
    if (controller_muted) {
      ctrl_target_vy = 0; ctrl_target_vz = 0; ctrl_target_vx = 0;
      ctrl_target_roll = 0; ctrl_target_pitch = 0;
      ctrl_target_height = CTRL_HEIGHT_BASE; ctrl_target_body_y = 0;
      ctrl_strafe_toggle_dir = 0;
      say(CLIP_SAD);
    } else {
      say(CLIP_PLAYFUL);
    }
  }

  if (controller_muted) {
    prev_btn.ps_b = ps_b;
    prev_btn.mute_b = mute_b;
    return;
  }

  if (l3 && !prev_btn.l3) cycle_gait_speed();

  if (l2 && !prev_btn.l2) {
    controller_execute_action(CMD_LEFT_HOOK);
    return;
  }
  if (r2 && !prev_btn.r2) {
    controller_execute_action(CMD_RIGHT_HOOK);
    return;
  }

  if (l1 && r1 && !(prev_btn.l1 && prev_btn.r1)) {
    controller_execute_action(CMD_ACTION_6);
    return;
  }

  if (select_b && !prev_btn.select_b) toggle_flashlight();

  if (cross && !prev_btn.cross) {
    controller_execute_selected_action();
    return;
  }

  // Square: tap=dive_fwd, 2tap=Action 8
  if (square && !prev_btn.square) {
    uint32_t now = millis();
    if (now - square_last_press_ms < DOUBLE_TAP_WINDOW_MS) {
      controller_execute_action(CMD_ACTION_8);
      square_last_press_ms = 0;
      return;
    } else {
      square_last_press_ms = now;
    }
  }
  if (square_last_press_ms != 0 && (millis() - square_last_press_ms) >= DOUBLE_TAP_WINDOW_MS) {
    square_last_press_ms = 0;
    controller_execute_action(CMD_DIVE_FWD);
    return;
  }

  // Circle: tap=dive_back, 2tap=spread
  if (circle && !prev_btn.circle) {
    uint32_t now = millis();
    if (now - circle_last_press_ms < DOUBLE_TAP_WINDOW_MS) {
      controller_execute_action(CMD_SPREAD);
      circle_last_press_ms = 0;
      return;
    } else {
      circle_last_press_ms = now;
    }
  }
  if (circle_last_press_ms != 0 && (millis() - circle_last_press_ms) >= DOUBLE_TAP_WINDOW_MS) {
    circle_last_press_ms = 0;
    controller_execute_action(CMD_DIVE_BACK);
    return;
  }

  // Triangle: tap=introduce, 2tap=dance
  if (triangle && !prev_btn.triangle) {
    uint32_t now = millis();
    if (now - triangle_last_press_ms < DOUBLE_TAP_WINDOW_MS) {
      controller_execute_action(CMD_DANCE);
      triangle_last_press_ms = 0;
      return;
    } else {
      triangle_last_press_ms = now;
    }
  }
  if (triangle_last_press_ms != 0 && (millis() - triangle_last_press_ms) >= DOUBLE_TAP_WINDOW_MS) {
    triangle_last_press_ms = 0;
    controller_execute_action(CMD_INTRODUCE);
    return;
  }

  // D-pad
  bool dpad_up_now    = (dpad & DPAD_UP)    && !(prev_btn.dpad & DPAD_UP);
  bool dpad_down_now  = (dpad & DPAD_DOWN)  && !(prev_btn.dpad & DPAD_DOWN);
  bool dpad_left_now  = (dpad & DPAD_LEFT)  && !(prev_btn.dpad & DPAD_LEFT);
  bool dpad_right_now = (dpad & DPAD_RIGHT) && !(prev_btn.dpad & DPAD_RIGHT);

  if (dpad_up_now)   cycle_gait_mode_up();
  if (dpad_down_now) cycle_gait_mode_down();

  if (dpad_left_now) {
    if (l1) {
      uint32_t now = millis();
      if (now - dpad_left_last_tap_ms < DOUBLE_TAP_WINDOW_MS) {
        set_strafe(-1, false);
        dpad_left_last_tap_ms = 0;
      } else {
        dpad_left_last_tap_ms = now;
      }
    } else {
      cycle_action_group(-1);
    }
  }
  if (dpad_right_now) {
    if (l1) {
      uint32_t now = millis();
      if (now - dpad_right_last_tap_ms < DOUBLE_TAP_WINDOW_MS) {
        set_strafe(+1, false);
        dpad_right_last_tap_ms = 0;
      } else {
        dpad_right_last_tap_ms = now;
      }
    } else {
      cycle_action_group(+1);
    }
  }

  // ---- Stick reads (RAW per-axis floats first) ----
  float lx_raw = stick_raw(myGamepad->axisX());
  float ly_raw = stick_raw(myGamepad->axisY());
  float rx_raw = stick_raw(myGamepad->axisRX());
  float ry_raw = stick_raw(myGamepad->axisRY());

  // ---- L stick: vector-magnitude exponential blend ----
  // ly_raw: positive = stick down → walk backward
  // lx_raw: positive = stick right → turn right (negative vz in Hiwonder convention?)
  // We negate ly so positive output = forward. lx stays as-is for "stick right = turn right".
  float lx_curved, ly_curved;
  stick_blend_curve(lx_raw, ly_raw, &lx_curved, &ly_curved);

  float speed_cap = CTRL_GAIT_SPEEDS[ctrl_gait_speed_idx];

  ctrl_target_vy = -ly_curved * CTRL_WALK_SPEED_BASE * speed_cap;
  ctrl_target_vz = -lx_curved * CTRL_TURN_SPEED_BASE * speed_cap;
  // ^^ NOTE: negated lx_curved for stick-right = turn-right (Hiwonder convention).
  // If still flipped, try removing the leading minus.

  // ---- Gyro mode overrides body posture from controller tilt ----
  if (ctrl_gyro_mode) {
    int rawPitch = myGamepad->gyroX();
    int rawLean  = myGamepad->gyroZ();
    rawPitch = apply_deadband(rawPitch, 100);
    rawLean  = apply_deadband(rawLean, 100);
    ctrl_target_pitch = ((float)rawPitch / 4000.0f) * CTRL_GYRO_PITCH_MAX;
    ctrl_target_roll  = ((float)rawLean  / 4000.0f) * CTRL_GYRO_ROLL_MAX;
  }

  // ---- R stick interpretation ----
  if (l1) {
    // L1+R stick Y -> height
    ctrl_target_height = CTRL_HEIGHT_BASE + (-curve(ry_raw)) * CTRL_HEIGHT_RANGE;
  } else if (r1) {
    // R1+R stick: X = lean roll, Y = body shift forward/back
    ctrl_target_roll  = curve(rx_raw) * CTRL_R1_LEAN_MAX_DEG;
    ctrl_target_body_y = -curve(ry_raw) * CTRL_R1_SHIFT_MAX_CM;
  } else {
    // No modifier — R stick = roll/pitch when L stick is centered
    bool l_stopped = (fabsf(ctrl_target_vy) < 0.05f && fabsf(ctrl_target_vz) < 0.05f);
    if (l_stopped) {
      if (!ctrl_gyro_mode) {
        ctrl_target_roll  =  curve(rx_raw) * CTRL_R_ROLL_MAX_DEG;
        ctrl_target_pitch = -curve(ry_raw) * CTRL_R_PITCH_MAX_DEG;
      }
    } else {
      // Walking — drift posture back toward neutral unless gyro mode owns it
      if (!ctrl_gyro_mode) {
        ctrl_target_roll  *= 0.7f;
        ctrl_target_pitch *= 0.7f;
      }
    }
    ctrl_target_body_y = 0.0f;
  }

  // ---- Strafe (hold or toggle) ----
  ctrl_target_vx = 0.0f;
  if (l1 && (dpad & DPAD_LEFT)) {
    ctrl_target_vx = -CTRL_WALK_SPEED_BASE * speed_cap;
    set_strafe(-1, true);
  } else if (l1 && (dpad & DPAD_RIGHT)) {
    ctrl_target_vx =  CTRL_WALK_SPEED_BASE * speed_cap;
    set_strafe(+1, true);
  } else if (ctrl_strafe_toggle_dir != 0) {
    ctrl_target_vx = (float)ctrl_strafe_toggle_dir * CTRL_WALK_SPEED_BASE * speed_cap;
  }

  // ---- Gait mode multipliers ----
  float stride_mult = 1.0f;
  float height_offset = 0.0f;
  if (ctrl_gait_mode == 1) { stride_mult = 1.4f; height_offset = 0.2f; }
  else if (ctrl_gait_mode == 2) { stride_mult = 0.6f; height_offset = -0.4f; }

  // ---- Smoothing ----
  ctrl_smooth_vy     += CTRL_SMOOTHING * (ctrl_target_vy     - ctrl_smooth_vy);
  ctrl_smooth_vz     += CTRL_SMOOTHING * (ctrl_target_vz     - ctrl_smooth_vz);
  ctrl_smooth_vx     += CTRL_SMOOTHING * (ctrl_target_vx     - ctrl_smooth_vx);
  ctrl_smooth_pitch  += CTRL_SMOOTHING * (ctrl_target_pitch  - ctrl_smooth_pitch);
  ctrl_smooth_roll   += CTRL_SMOOTHING * (ctrl_target_roll   - ctrl_smooth_roll);
  float effective_height = ctrl_target_height + height_offset;
  ctrl_smooth_height += CTRL_SMOOTHING * (effective_height - ctrl_smooth_height);
  ctrl_smooth_body_y += CTRL_SMOOTHING * (ctrl_target_body_y - ctrl_smooth_body_y);

  // ---- Push to gary.move() ----
  if (millis() - last_ctrl_move_ms >= CTRL_MOVE_INTERVAL_MS) {
    last_ctrl_move_ms = millis();
    Velocity_t cv = {ctrl_smooth_vx, ctrl_smooth_vy, ctrl_smooth_vz};
    Vector_t   cp = {0.0f, ctrl_smooth_body_y, ctrl_smooth_height};
    // CRITICAL: Euler_t is {pitch, roll, yaw} in DEGREES
    Euler_t    ca = {ctrl_smooth_pitch, ctrl_smooth_roll, 0.0f};

    uint16_t step_dur = (uint16_t)(STEP_TIME_MS * stride_mult / CTRL_GAIT_SPEEDS[ctrl_gait_speed_idx]);
    if (step_dur < 200)  step_dur = 200;
    if (step_dur > 3000) step_dur = 3000;

    bool moving = fabsf(ctrl_smooth_vy) > 0.05f
               || fabsf(ctrl_smooth_vz) > 0.05f
               || fabsf(ctrl_smooth_vx) > 0.05f;

    if (moving) {
      gary.move(&cv, &cp, &ca, step_dur, 1);
    } else {
      Velocity_t zero = {0.0f, 0.0f, 0.0f};
      gary.move(&zero, &cp, &ca, 80);
    }
  }

  // Save edge-detection state
  prev_btn.cross = cross;
  prev_btn.circle = circle;
  prev_btn.triangle = triangle;
  prev_btn.square = square;
  prev_btn.l1 = l1;
  prev_btn.r1 = r1;
  prev_btn.l2 = l2;
  prev_btn.r2 = r2;
  prev_btn.l3 = l3;
  prev_btn.r3 = r3;
  prev_btn.select_b = select_b;
  prev_btn.start_b = start_b;
  prev_btn.ps_b = ps_b;
  prev_btn.mute_b = mute_b;
  prev_btn.tp_click = tp_click;
  prev_btn.dpad = dpad;
}

void loop_controller() {
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
      set_eyes_breathing(EYE_SABER_BLUE);
      anim_target_hud();
      stop_motion();
      ctrl_target_vy = 0.0f;
      ctrl_target_vz = 0.0f;
      ctrl_target_roll = 0.0f;
      ctrl_target_pitch = 0.0f;
      ctrl_target_height = 1.5f;
      ctrl_smooth_vy = 0.0f;
      ctrl_smooth_vz = 0.0f;
      ctrl_smooth_roll = 0.0f;
      ctrl_smooth_pitch = 0.0f;
      ctrl_smooth_height = 1.5f;
      Serial.println("[CTRL] Entering controller mode");
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
      exec_turn(5.0, DISMISSAL_TURN_MS , 5);
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
      transition_to(STATE_EXECUTING);
      say(CLIP_WHISTLE_INQUISITIVE);
      att = {0.0f, 0.0f, 20.0f};
      gary.move(&vel, &pos, &att, 200, 1);
      gary_delay(250);
      att = {0.0f, 0.0f, -20.0f};
      gary.move(&vel, &pos, &att, 200, 1);
      gary_delay(250);
      att = {0.0f, 0.0f, 20.0f};
      gary.move(&vel, &pos, &att, 200, 1);
      gary_delay(250);
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
    consecutive_tip_readings++;
    if (consecutive_tip_readings >= 3 && current_state != STATE_ALARMED) {
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
  if (startle_enabled) {
    uint16_t sound_value = gary.board.get_sound_val();
    int delta = (int)sound_value - (int)last_sound_value;

    if (sound_value >= SOUND_STARTLE_THRESHOLD
        && delta >= SOUND_STARTLE_DELTA
        && current_state != STATE_STARTLED
        && current_state != STATE_ALARMED
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
  uint8_t cmd = gary.sensor.asr.rec_recognition();
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
  uint8_t cmd = gary.sensor.asr.rec_recognition();
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
  uint8_t cmd = gary.sensor.asr.rec_recognition();
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

  uint8_t cmd = gary.sensor.asr.rec_recognition();
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
  Serial.begin(115200);
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
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
  face_scroll_text("GARY");  // boot greeting

#if ENABLE_DEBUG_SERIAL
  Serial.println(">>> GARY: standing up");
#endif
  Velocity_t stand_vel = {0.0f, 0.0f, 0.0f};
  Vector_t stand_pos = {0.0f, 0.0f, 1.5f};
  Euler_t stand_att = {0.0f, 0.0f, 0.0f};
  gary.move(&stand_vel, &stand_pos, &stand_att, 1000);
  delay(1500);

  set_eyes(EYE_BOOT);
  delay(500);

  // Boot into IDLE — Gary stays put until commanded.
  // To revert to autonomous roaming, change to STATE_ROAMING and re-enable
  // the IDLE_TO_ROAM_MS timeout in loop_idle().
  transition_to(STATE_IDLE);

#if ENABLE_DEBUG_SERIAL
  Serial.println("=== GARY v1 READY ===");
#endif
}

void loop() {
  // Pump Bluepad32 first - keeps connection alive and reads new packets
  BP32.update();

  // Auto-transition to controller mode when DualSense connects
  if (myGamepad && myGamepad->isConnected() && current_state != STATE_CONTROLLER) {
    transition_to(STATE_CONTROLLER);
  }

  // Pump the sound state machine every loop iteration
  gary_sounds_update();
  gary_face_update();

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
#endif

  if (run_background_checks()) {
    delay(LOOP_DELAY_MS);
    return;
  }

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
  }

  delay(LOOP_DELAY_MS);
}
