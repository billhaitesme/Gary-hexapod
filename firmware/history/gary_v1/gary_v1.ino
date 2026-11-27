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
  STATE_ALARMED
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
  anim_imperial_cog();
  for (int i = 0; i < 2; i++) {
    att = {15.0f, 0.0f, 0.0f};
    gary.move(&zero, &pos, &att, 250);
    gary_delay(280);
    att = {-15.0f, 0.0f, 0.0f};
    gary.move(&zero, &pos, &att, 250);
    gary_delay(280);
  }

  // Beat 2: spin in place — Rebel starbird flapping
  say(CLIP_VERY_EXCITED);
  anim_rebel_starbird();
  vel = {0.0f, 0.0f, 10.0f};
  gary.move(&vel, &pos, &att, 150);
  gary_delay(200);
    vel = {0.0f, 0.0f, -10.0f};
  gary.move(&vel, &pos, &att, 150);
  gary_delay(200);

  // Beat 3: pitch + roll combo + yaw — control panel chaos
  say(CLIP_PLAYFUL);
  anim_control_panel();
  att = {20.0f, 10.0f, -10.0f};
  gary.move(&zero, &pos, &att, 250,3);
  gary_delay(280);
  att = {-20.0f, -10.0f, 10.0f};
  gary.move(&zero, &pos, &att, 325,3);
  gary_delay(350);

  // Beat 4: BIG finish — hyperspace jump + scream
  say(CLIP_SCREAM);
  set_eyes(EYE_SABER_PURPLE);    // mace windu purple for finale flair
  anim_hyperspace();
  pos = {0.0f, 0.0f, 10.0f};         // stand tall
  att = {0.0f, 0.0f, 20.0f};
  gary.move(&zero, &pos, &att, 150,2);
  gary_delay(200);
  pos = {0.0f, 0.0f, 0.2f};         // crouch low
  gary.move(&zero, &pos, &att, 300);
  gary_delay(320);
  say(CLIP_VERY_EXCITED);
    pos = {0.0f, 0.0f, 10.0f};         // stand tall
  att = {0.0f, 0.0f, -20.0f};
  gary.move(&zero, &pos, &att, 250);
  gary_delay(300);
  say(CLIP_SCREAM);
    pos = {0.0f, 0.0f, 0.2f};         // crouch low
  gary.move(&zero, &pos, &att, 300);
  gary_delay(320);
  pos = {0.0f, 0.0f, 1.5f};         // back to neutral
  gary.move(&zero, &pos, &att, 300);
  gary_delay(400);
}

void gary_introduce_routine() {
  Velocity_t zero = {0.0f, 0.0f, 0.0f};
  Vector_t pos = {0.0f, 0.0f, 1.5f};
  Euler_t att = {0.0f, 0.0f, 0.0f};

  say(CLIP_PROUD);
  anim_lightsaber();          // saber ignition during introduction
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
  anim_rebel_starbird();
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
      set_eyes(EYE_LISTENING);
      anim_sensor_grid();
      say(CLIP_LOOK);
      listening_got_command = false;
      break;

    case STATE_EXECUTING:
      set_eyes(EYE_EXECUTING);
      anim_progress_bars();
      say(CLIP_SNAPPY);
      break;

    case STATE_ROAMING:
      set_eyes_breathing(EYE_TIE_GREEN);  // TIE engine glow
      anim_tie_fighter();            // TIE patrol while roaming
      say(CLIP_CHATTER);
      last_boredom_kick_ms = millis();
      break;

    case STATE_COMING:
      set_eyes(EYE_REBEL);           // rebel orange — heading to base
      anim_x_wing();                 // X-Wing inbound
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
      exec_turn(5.0, DISMISSAL_TURN_MS , 6);
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
      exec_pose({0.0f, 0.0f, 60.0f}, 3000);
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
      transition_to(STATE_EXECUTING);
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
      transition_to(STATE_EXECUTING);
      say(CLIP_SAD);
      pos = {0.0f, 0.0f, 0.3f};
      gary.move(&vel, &pos, &att, 1500);
      gary_delay(1700);
      stop_motion();
      transition_to(STATE_IDLE);
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
      transition_to(STATE_EXECUTING);
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
      transition_to(STATE_EXECUTING);
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

    case CMD_ACTION_11:
#if ENABLE_CAMERA
      transition_to(STATE_COMING);
#else
      say(CLIP_WHISTLING_EXCITED);
      exec_walk(WALK_SPEED * 1.2f, STEP_TIME_MS, 3);
#endif
      return true;

    case CMD_ACTION_12:
      say(CLIP_WHISTLING_EXCITED);
      transition_to(STATE_ROAMING);
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
      att = {0.4f, 0.0f, 0.0f};
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
      att = {-0.4f, 0.0f, 0.0f};
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
      exec_pose({0.0f, 0.0f, 0.8f}, 1000);
      return true;

    case CMD_MAKE_SOUND:
      // Just chirp playfully and stay listening
      say(CLIP_PLAYFUL);
      return_to_listening();
      return true;

    case CMD_DIVE_FWD:
      // Lower + lurch forward like a dive
      transition_to(STATE_EXECUTING);
      say(CLIP_DETERMINED);
      pos = {0.0f, 0.0f, 0.8f};
      att = {20.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 300);
      gary_delay(400);
      vel = {0.0f, WALK_SPEED * 2.0f, 0.0f};
      gary.move(&vel, &pos, &att, 600, 1);
      gary_delay(700);
      pos = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 400);
      gary_delay(500);
      return_to_listening();
      return true;

    case CMD_DIVE_BACK:
      transition_to(STATE_EXECUTING);
      say(CLIP_CONCERNED);
      pos = {0.0f, 0.0f, 0.8f};
      gary.move(&vel, &pos, &att, 300);
      gary_delay(400);
      vel = {0.0f, -WALK_SPEED * 1.4f, 0.0f};
      gary.move(&vel, &pos, &att, 600, 1);
      gary_delay(700);
      pos = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 400);
      gary_delay(500);
      return_to_listening();
      return true;

    case CMD_SQUAT:
      // Lower than sit, hold longer
      exec_pose({0.0f, 0.0f, 0.4f}, 1800);
      return true;

    // ====================================================================
    //  MORE FACTORY ANIMATIONS
    // ====================================================================

    case CMD_PUSH_UP:
      // Body pumps up and down repeatedly
      transition_to(STATE_EXECUTING);
      say(CLIP_DETERMINED);
      for (int i = 0; i < 3; i++) {
        pos = {0.0f, 0.0f, 0.8f};
        gary.move(&vel, &pos, &att, 350);
        gary_delay(400);
        pos = {0.0f, 0.0f, 1.7f};
        gary.move(&vel, &pos, &att, 350);
        gary_delay(400);
      }
      pos = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 400);
      gary_delay(500);
      return_to_listening();
      return true;

    case CMD_TWIST:
      // Yaw body left-right rapidly without translating
      transition_to(STATE_EXECUTING);
      say(CLIP_PLAYFUL);
      for (int i = 0; i < 3; i++) {
        att = {0.0f, 0.0f, 0.3f};
        gary.move(&vel, &pos, &att, 250);
        gary_delay(280);
        att = {0.0f, 0.0f, -0.3f};
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
      exec_pose({0.0f, 0.0f, 1.8f}, 800);
      return true;

    case CMD_SPREAD:
      // Hexapod can't really spread "hands" — interpret as wide stance pose:
      // pitch back + elevated body to look proud / stretched
      transition_to(STATE_EXECUTING);
      say(CLIP_PROUD);
      pos = {0.0f, 0.0f, 2.0f};
      att = {0.0f, -0.25f, 0.0f};
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
      gary.move(&vel, &pos, &att, 800);
      gary_delay(900);
      gary_delay(800);  // lie there
      say(CLIP_DETERMINED);
      pos = {0.0f, 0.0f, 1.5f};
      gary.move(&vel, &pos, &att, 1000);
      gary_delay(1100);
      return_to_listening();
      return true;

    case CMD_LEFT_HOOK:
      // Quick body lunge to the left + lean right (like throwing a hook)
      transition_to(STATE_EXECUTING);
      say(CLIP_DETERMINED);
      att = {0.4f, 0.0f, 0.2f};   // roll left + slight yaw
      gary.move(&vel, &pos, &att, 250);
      gary_delay(300);
      att = {-0.4f, 0.0f, -0.2f}; // recover with opposite roll
      gary.move(&vel, &pos, &att, 250);
      gary_delay(300);
      att = {0.0f, 0.0f, 0.0f};
      gary.move(&vel, &pos, &att, 300);
      gary_delay(350);
      return_to_listening();
      return true;

    case CMD_RIGHT_HOOK:
      transition_to(STATE_EXECUTING);
      say(CLIP_DETERMINED);
      att = {-0.4f, 0.0f, -0.2f};
      gary.move(&vel, &pos, &att, 250);
      gary_delay(300);
      att = {0.4f, 0.0f, 0.2f};
      gary.move(&vel, &pos, &att, 250);
      gary_delay(300);
      att = {0.0f, 0.0f, 0.0f};
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
  // To re-enable autonomous roaming, uncomment the block below.
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

      vel = {0.0f, 0.0f, 0.5f};
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
  }

  delay(LOOP_DELAY_MS);
}
