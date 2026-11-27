// =============================================================================
// gary_config.h
//
// All Gary's tunable values live here. Adjust these to change behavior
// without touching the main state-machine code.
// =============================================================================

#ifndef GARY_CONFIG_H
#define GARY_CONFIG_H

// -----------------------------------------------------------------------------
// VOICE CLIP IDs
//
// These are placeholder IDs. After WonderEcho is reflashed with R2-D2 clips
// these would map to the new IDs. For now (factory firmware) ENABLE_VOICE_PLAYBACK
// is 0 and these are unused -- the speak() helper is a no-op.
// -----------------------------------------------------------------------------

#define VOICE_BROADCAST_TYPE   0xFF   // Per WonderEcho protocol
#define VOICE_COMMAND_TYPE     0x00

#define CLIP_LOOK              1
#define CLIP_SNAPPY            2
#define CLIP_UNSURE            3
#define CLIP_PROCESSING        4
#define CLIP_CHATTER            5
#define CLIP_WHISTLING_EXCITED 6
#define CLIP_PROUD             7
#define CLIP_SCREAM             8
#define CLIP_SAD                9
#define CLIP_WHISTLE            10
#define CLIP_PLAYFUL             11
#define CLIP_VERY_EXCITED       12
#define CLIP_LAUGHTER            13
#define CLIP_CONCERNED           14
#define CLIP_SHOCKED              15
#define CLIP_DETERMINED           16
#define CLIP_WHISTLE_INQUISITIVE 17

// -----------------------------------------------------------------------------
// VOICE COMMAND IDs -- REAL VALUES from factory firmware
//
// These come from WonderEcho's Command Word and Playback Phrase Protocol List.
// Factory firmware (CI1302_English_SingleMic_V00729) returns the command byte
// (4th byte of the AA 55 protocol frame) via I2C register 100.
//
// Verified working via gary_voice_poc.ino:
//   "go straight" -> 0x01
//   "turn left"   -> 0x03
//   "stop"        -> 0x09
//   "dance"       -> 0x6C
//   "execute action one" -> 0x80
//
// All values below come from 4_3_Command_Word_Broadcasting_Protocol_List.xlsx.
// -----------------------------------------------------------------------------

#define CMD_NONE        0

// --- Movement / orientation ---
#define CMD_FORWARD     0x01   // "go straight"
#define CMD_BACKWARD    0x02   // "go backward"
#define CMD_TURN_LEFT   0x03   // "turn left"
#define CMD_TURN_RIGHT  0x04   // "turn right"
#define CMD_LEAN_FWD    0x05   // "lean forward"   (factory cmd, optional use)
#define CMD_LEAN_BACK   0x06   // "lean back"
#define CMD_LEAN_LEFT   0x07   // "lean left"
#define CMD_LEAN_RIGHT  0x08   // "lean right"
#define CMD_STOP        0x09   // "stop"
#define CMD_ATTENTION   0x0A   // "attention"      -> Gary STAY
#define CMD_GET_DOWN    0x0B   // "get down"
#define CMD_SIT_DOWN    0x0C   // "sit down"       -> Gary SIT
#define CMD_SPEED_UP    0x0D   // "speed up"       -> Gary GO_FASTER
#define CMD_SLOW_DOWN   0x0E   // "slow down"
#define CMD_RAISE_BODY  0x0F   // "raise the body" -> Gary WAKE_UP
#define CMD_LOWER_BODY  0x10   // "lower the body"
#define CMD_MAKE_SOUND  0x11   // "make a sound"

// --- Speech / interaction ---
#define CMD_HELLO       0x1A   // "hello"          -> Gary SAY_HELLO
#define CMD_INTRODUCE   0x1B   // "introduce yourself"
#define CMD_SHOW_SKILL  0x1C   // "show a skill"   -> Gary SHOW_OFF
#define CMD_MARCH       0x1D   // "march"          -> Gary TURN_AROUND placeholder
#define CMD_SHAKE_HEAD  0x1E   // "shake head"     -> Gary LOOK_AROUND
#define CMD_DIVE_FWD    0x1F   // "dive forward"
#define CMD_DIVE_BACK   0x20   // "dive backward"
#define CMD_COMBAT      0x21   // "combat mode"    -> Gary FIGHT
#define CMD_SQUAT       0x22   // "squat down"
#define CMD_SHAKE_OFF   0x23   // "shake it off"   -> Gary SHAKE

// --- Animations / dance ---
#define CMD_DANCE       0x6C   // "dance"          -> Gary DANCE
#define CMD_PUSH_UP     0x6D   // "push up"
#define CMD_TWIST       0x71   // "twist"
#define CMD_SIT_UP      0x72   // "sit up"
#define CMD_BOW         0x73   // "bow"            -> Gary BOW
#define CMD_SPREAD      0x74   // "spread the hands"
#define CMD_WAVE        0x75   // "wave hand"      -> Gary WAVE
#define CMD_MARCH_PLACE 0x76   // "march in place"
#define CMD_FALL_STAND  0x77   // "fall and stand up"
#define CMD_STRUT       0x78   // "strut"
#define CMD_LEFT_HOOK   0x79   // "left hook"
#define CMD_RIGHT_HOOK  0x7A   // "right hook"

// --- Action slots -- say "execute action ONE" through "execute action TEN" ---
#define CMD_ACTION_1    0x80   // Gary RUN
#define CMD_ACTION_2    0x81   // Gary GO_HOME
#define CMD_ACTION_3    0x82   // Gary SAY_GOODBYE
#define CMD_ACTION_4    0x83   // Gary SAY_GOODNIGHT
#define CMD_ACTION_5    0x84   // Gary GOOD_JOB
#define CMD_ACTION_6    0x85   // Gary HIGH_FIVE
#define CMD_ACTION_7    0x86   // Gary HUG
#define CMD_ACTION_8    0x87   // Gary SIC_EM
#define CMD_ACTION_9    0x88   // Gary CHARGE
#define CMD_ACTION_10   0x89   // Gary GET_OUT (replaces old "get out of here" meaning)

// --- Action slots 11-30 -- say "execute action ELEVEN" through "execute action THIRTY" ---
#define CMD_ACTION_11   0xA1   // Gary COME_HERE
#define CMD_ACTION_12   0xA2   // "execute action twelve" -> SHOW BATTERY display (NOT explore -- explore is 16)
#define CMD_ACTION_13   0xA3   // "execute action thirteen" -> Gary STATE_SENSING (WiFi CSI capture, holds still ~8s)
#define CMD_ACTION_14   0xA4   // "execute action fourteen" -> Gary STATE_NODSCAN (3D LiDAR tilt-sweep point cloud)
#define CMD_ACTION_15   0xA5   // "execute action fifteen" -> toggle STATE_AUTOSCAN (CSI watch; auto-fires a nod scan)
#define CMD_ACTION_16   0xA6   // "execute action sixteen" -> toggle STATE_EXPLORE (autonomous roam; say "stop" to end)
#define CMD_ACTION_17   0xA7
#define CMD_ACTION_18   0xA8
#define CMD_ACTION_19   0xA9
#define CMD_ACTION_20   0xAA

// Legacy aliases -- keep these so existing code that references CMD_GET_OUT
// or CMD_TWO_STEPS still compiles. Both now route via dispatcher to clearer
// modern equivalents.
#define CMD_GET_OUT     CMD_ACTION_10
#define CMD_TWO_STEPS   CMD_MARCH       // "march" is the closest factory phrase

// CMD_WAKE_WORD: factory firmware DOES NOT push a wake-word event to register
// 100 (POC test confirmed). The wake word "HELLO-HI-WONDER" only puts the
// chip into recognition mode internally, then commands flow.
// We don't need a separate state for wake -- Gary stays in ROAMING and any
// command byte arriving promotes us to LISTENING for a follow-up window.

// -----------------------------------------------------------------------------
// EYE COLOR PALETTE -- unchanged
// -----------------------------------------------------------------------------

const uint8_t EYE_OFF[3]            = {  0,   0,   0 };
const uint8_t EYE_BOOT[3]           = {100, 100, 100 };
const uint8_t EYE_IDLE[3]           = {  0,   0, 100 };
const uint8_t EYE_LISTENING[3]      = {  0, 100, 255 };
const uint8_t EYE_EXECUTING[3]      = {  0, 200,   0 };
const uint8_t EYE_ACK_FLASH[3]      = {  0, 255,   0 };
const uint8_t EYE_ROAMING_A[3]      = {100,   0, 150 };
const uint8_t EYE_ROAMING_B[3]      = {  0, 150, 150 };
const uint8_t EYE_COMING[3]         = {255, 200,   0 };
const uint8_t EYE_PROUD[3]          = { 50, 255,  50 };
const uint8_t EYE_STARTLED[3]       = {255,   0,   0 };
const uint8_t EYE_ALERT[3]          = {255, 100,   0 };
const uint8_t EYE_ALARMED[3]        = {200,   0,   0 };
const uint8_t EYE_SAD[3]            = {255, 100,  50 };

// -----------------------------------------------------------------------------
// STAR WARS PALETTE -- bonus colors for thematic moments
// -----------------------------------------------------------------------------

const uint8_t EYE_SABER_BLUE[3]     = { 30, 100, 255 };  // jedi blue
const uint8_t EYE_SABER_GREEN[3]    = { 30, 255,  60 };  // luke ROTJ green
const uint8_t EYE_SABER_RED[3]      = {255,  20,   60 };  // sith red
const uint8_t EYE_SABER_PURPLE[3]   = {180,   0, 255 };  // mace windu
const uint8_t EYE_REBEL[3]          = {255, 100,   0 };  // rebel orange
const uint8_t EYE_IMPERIAL[3]       = {200, 200, 220 };  // stormtrooper white-blue
const uint8_t EYE_TIE_GREEN[3]      = {  0, 255, 100 };  // TIE engine glow
const uint8_t EYE_HYPERSPACE[3]     = {200, 220, 255 };  // hyperspace blue-white

// -----------------------------------------------------------------------------
// TIMING / THRESHOLDS
// -----------------------------------------------------------------------------

// Was 50. A hardcoded 50ms sleep at the bottom of loop() -- Gary spent 77% of his
// life asleep. Everything that runs once per loop was throttled by it: the ASR poll
// (which is WHY voice kept dropping), the gait controller (a walker steered 15x/sec
// -- 4x/sec while mapping -- is going to wander, and probably explains a good part
// of the 9.7 deg/m veer), and the odometry timestep.
//
// Not zero: delay(1) yields to FreeRTOS so the BLE/WiFi stacks on the other core
// still get scheduled. A busy-wait loop() is how you get a robot that walks
// beautifully and drops its controller connection.
#define LOOP_DELAY_MS              1
#define LISTENING_FIRST_TIMEOUT_MS 5000
#define LISTENING_TIMEOUT_MS       90000
#define IDLE_TO_ROAM_MS            30000
#define ROAM_BOREDOM_MS            8000
#define ROAM_TIMEOUT_MS            300000
#define DISMISSAL_TURN_MS          1500
#define COMING_SEARCH_TIMEOUT_MS   15000
#define STARTLED_DURATION_MS       3000
#define EYES_BREATHING_PERIOD_MS   800

#define SOUND_STARTLE_THRESHOLD    3000
#define SOUND_STARTLE_DELTA        2500
// After a recognized voice command, the WonderEcho plays an acknowledgment clip
// through its own speaker -- which the sound sensor hears and (before this) took
// for a scare, startling him right out of EXPLORE. Ignore the mic this long after
// a command so his own voice can't self-startle. Covers the ~1-2s ack clips.
#define VOICE_ACK_STARTLE_SUPPRESS_MS 2500
#define DISTANCE_OBSTACLE_MM       250
#define DISTANCE_ARRIVED_MM        180
#define BATTERY_LOW_MV             6000

// IMU_TIP_THRESHOLD_DEG -- was 30. That's the SAME value as CLAMP_PITCH_DEG
// in the main sketch used to be, which meant commanding Gary to its own max
// intentional pitch sat right at the tip-alarm trip point -- no margin at
// all. Raised to 45 so intentional tilting (including the nod-scan sweep,
// which uses CLAMP_PITCH_DEG=35 and stays well under this) can't
// accidentally fire STATE: * -> ALARMED. A REAL tip-over is a much bigger
// event than any commanded tilt -- the log that first surfaced this issue
// showed roll swinging to -94/-109 degrees during an actual fall, so 45
// still leaves a huge margin below genuine tip-over magnitudes and doesn't
// meaningfully compromise the alarm's real job.
#define IMU_TIP_THRESHOLD_DEG      45
// TIP = EXTREME ATTITUDE AT REST (2026-08-21, from the sim's balance work;
// Bill: "he panics too often... it's an IMU trigger"): a genuinely tipped
// robot lies STILL at a bad angle; a robot being PICKED UP swings past 45
// with the gyro busy — that's handling, not a fall, and it fired the alarm
// on every pickup. The tip counter now only advances while the gyro is
// quiet, and needs more consecutive hits.
#define IMU_TIP_GYRO_QUIET_DPS     40.0f
#define IMU_TIP_CONSEC             5
#define FACE_SIZE_CLOSE            55
#define FACE_SIZE_FAR              25

#define WALK_SPEED                 3.5f
#define TURN_SPEED                 2.5f
#define STEP_TIME_MS               1200   // was 15000: every walk/turn ran its gait over 15 s (bench 2026-08-23, 'WAAAAY too slow')
#define DEFAULT_STEPS              2

// -----------------------------------------------------------------------------
// FEATURE FLAGS
// -----------------------------------------------------------------------------

#define ENABLE_VOICE               1
#define ENABLE_CAMERA              0
#define ENABLE_BALANCE             1
#define ENABLE_SOUND_STARTLE       1
#define ENABLE_BATTERY_MONITOR     1
#define ENABLE_BOREDOM_TIMER       0
#define ENABLE_VOICE_PLAYBACK      0   // Factory firmware -- leave 0
#define ENABLE_DEBUG_SERIAL        1

// ---------------------------------------------------------------------------
// GAIT_TUNED — sim-trained wave/ripple parameters (2026-08-18, NOT bench-tested)
//
// Source: sim/train_gait.py (MuJoCo twin from CAD geometry, CMA-ES over
// {cadence, lift, speed_cap} scored as speed minus knee-torque-saturation, on
// the velocity-matched Hermite swing). Retrained 2026-08-18 against the REAL
// servo spec — HPD-0127, clutch limit 1.8 kgf*cm = 0.176 N*m, 0.05 s/60deg —
// and the confirmed 46 g D500 at its actual mount. sim/out/train_*.json:
//   WAVE   911 ms / lift 1.09 cm / cap 1.0 -> 3.1 cm/s @ 5.4% knee sat (July fw: 0.5 cm/s)
//   RIPPLE 416 ms / lift 0.76 cm / cap 1.0 -> 4.0 cm/s @ 0.9%          (July fw: 1.2 cm/s)
// Ripple's low lift is a FLAT-FLOOR optimum — raise it on carpet/rough ground.
// Set to 0 to get the exact July-13 behavior back (cosine swing, halved caps).
//
// Sim-to-real derating: robot MASS is still an estimate (~1.25 kg component
// sum), so the trained cap of 1.0 ships scaled by GAIT_TUNED_CAP_SCALE for the
// first bench runs. Raise toward 1.0 only after the knee-guard clip counters
// stay quiet. Yaw (omega) caps were NOT trained — they stay at the July values.
// TRIPOD runs the vendor library path, untouched by any of this.
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// ENABLE_OTA — wireless flashing via ArduinoOTA (2026-08-18, NOT bench-tested).
// Once Gary boots on WiFi, Arduino IDE shows "gary at <ip> (network)" under
// Tools -> Port; upload over the air, password in wifi_secrets.h (OTA_PASSWORD).
// Motion halts automatically when an upload starts. USB flashing still works.
// ---------------------------------------------------------------------------
#define ENABLE_OTA                 1
#define OTA_HOSTNAME               "gary"

// BAL_TUNED — sim-proven balance upgrade (2026-08-23, sim/balance_gym.py):
// the stock balance() is P-only (command = -measured) and mathematically
// converges to HALF the slope, never level. ONE INTEGRATOR closes it: sim
// battery went from "level at 3 deg, sags 7.8 at 6" to "level chassis
// through 12 deg static, 8-deg step re-level in 1.5 s". Cross-validations:
// sim evolution independently found kp 1.05 (this loop runs 1.0); the
// vendor's 18-deg authority clamp matches the sim's evolved 17.2. Level
// chassis = level lidar ring = maps that survive slopes. NOT bench-tested;
// leak keeps the integrator honest if the IMU drifts.
#define BAL_TUNED                  1
#define BAL_KI                     0.8f    // integral gain (sim stable band 0.8-1.7)
#define BAL_LEAK                   0.6f    // integrator leak, 1/s
#define BAL_CLAMP_DEG              18.0f   // command authority (vendor value, sim-confirmed)

#define GAIT_TUNED                 1
#define GAIT_TUNED_CAP_SCALE       0.5f     // first-bench derating on the trained caps
#define GAIT_TUNED_SPEED_CAP_WAVE  1.0f     // trained (July value was 0.15)
#define GAIT_TUNED_SPEED_CAP_RIP   1.0f     // trained (July value was 0.30)
#define GAIT_TUNED_CADENCE_WAVE    911      // ms (was 1200)
#define GAIT_TUNED_CADENCE_RIP     416      // ms (was 800)
#define GAIT_TUNED_LIFT_WAVE       1.09f    // cm, used directly (replaces height-scaled lift)
#define GAIT_TUNED_LIFT_RIP        0.76f    // cm — flat-floor optimum, raise on carpet

// ---------------------------------------------------------------------------
// EXPLORE -- the sim-graduated reflex policy (2026-08-19/20 honors run), speeds
// as stick-fractions of WALK_SPEED, heights/lifts in cm relative to stand.
// ALL values ship derated by EXP_DERATE for the first live runs; raise it only
// while the knee-guard clip counters stay quiet. Sim source of truth:
// sim/out/course_policy_sensor.json.
// ---------------------------------------------------------------------------
#define EXP_TICK_MS          50
#define EXP_DERATE           1.0f    // governance RELEASED by Bill 2026-08-24 (was 0.6 for the first live runs)
#define EXP_CRUISE_SPEED     1.08f   // honors genes
#define EXP_CRUISE_LIFT      2.8f
#define EXP_CRUISE_HEIGHT    2.7f
#define EXP_CLIMB_TRIG_MM    300     // forward sonar (see tilt-shim note in the .ino)
#define EXP_CLIMB_HOLD_MS    1500
#define EXP_CLIMB_SPEED      0.88f
#define EXP_CLIMB_LIFT       2.6f
#define EXP_CLIMB_HEIGHT     2.3f
#define EXP_DUCK_RING_MM     250     // D500 front arc: something at mast height
#define EXP_SONAR_CLEAR_MM   400     // ...while the low path is open = duck, not wall
#define EXP_CONTACT_RING_MM  120     // lidar dead-ahead under this = CONTACT: wall turn,
                                     //   no sonar opinion consulted (oven, 2026-08-24)
#define EXP_DUCK_SPEED       0.60f   // derate applies on top (honors gene was 0.88)
#define EXP_DUCK_HEIGHT     -1.2f
#define EXP_DUCK_TRAVEL_MM   240     // doorway fix: stay low this far past last sighting
#define EXP_RETREAT_MS       1200
#define EXP_PIVOT_MS         6500    // ~180 deg at the real pivot rate -- TUNE ON BENCH
#define EXP_PIVOT_OMEGA      0.8f
#define EXP_STUCK_MS         4000
#define EXP_PUSH_WALL_MS     3500   // pinned range this long during a push = wall, not movable
// #define EXPLORE_SONAR_TILTED 1    // enable after fitting the ~10 deg sonar shim:
//                                      unlocks the sim's shortfall/overshoot (cliff) math

// ---------------------------------------------------------------------------
// BREACH FSM (Layer 1, ported from sim BreachFSM 2026-09-17) -- purposeful
// door-working at a frontal block: square up, ONE committed shove, and if it
// is a wall: look it up and down, probe-knock, head-shake, turn away with a
// bounded mule-kick, then move on. Reactive: if the forward range opens at
// any point, abort and drive through. Replaces the bare push+defeat loop that
// left him bumping the same spot forever. NOT bench-tuned yet -- these are
// first-cut numbers to refine with Bill watching. BREACH_ENABLE 0 restores
// the old door-trial behavior without a reflash-elsewhere.
// ---------------------------------------------------------------------------
#define BREACH_ENABLE        1
#define BREACH_OPEN_MM       150     // forward range rising this far = "it gave" -> go through
#define BREACH_MIN_OPEN_MM   500     // ...and only once the way is at least this clear
#define BREACH_SHOVE_SPD     1.40f   // committed shove, as a fraction of WALK_SPEED (> cruise 1.08)
#define BREACH_SHOVE_MS      1400    // how long he leans on it before judging it a wall
#define BREACH_LOOK_MS       1600    // look-it-up-and-down gesture duration
#define BREACH_LOOK_PITCH    18.0f   // peak body pitch of the look, deg (well under CLAMP_PITCH_DEG)
#define BREACH_KNOCK_MS      260     // the probe-knock jab: a short forward tap
#define BREACH_KNOCK_END_MS  700     // total knock phase (jab + settle) before the shake
#define BREACH_SHAKE_MS      1000    // head-shake wag duration
#define BREACH_TURN_MS       3000    // turn-away pivot before the mule-kick
#define BREACH_KICK_SPD      1.20f   // mule-kick backward buck, fraction of WALK_SPEED
#define BREACH_KICK_MS       350     // ...kept SHORT: a buck, not a retreat
#define BREACH_COOL_MS       6000    // after a full episode, don't re-breach this spot for a bit

#endif // GARY_CONFIG_H
