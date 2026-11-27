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
// is 0 and these are unused — the speak() helper is a no-op.
// -----------------------------------------------------------------------------

#define VOICE_BROADCAST_TYPE   0xFF   // Per WonderEcho protocol
#define VOICE_COMMAND_TYPE     0x00

#define CLIP_LOOK              1
#define CLIP_SNAPPY            2
#define CLIP_UNSURE            3
#define CLIP_PROCESSING        4
#define CLIP_CHATTER           5
#define CLIP_WHISTLING_EXCITED 6
#define CLIP_PROUD             7
#define CLIP_SCREAM            8
#define CLIP_SAD               9
#define CLIP_WHISTLE           10
#define CLIP_PLAYFUL           11
#define CLIP_VERY_EXCITED      12
#define CLIP_LAUGHTER          13
#define CLIP_CONCERNED         14
#define CLIP_SHOCKED           15
#define CLIP_DETERMINED        16
#define CLIP_WHISTLE_INQUISITIVE 17

// -----------------------------------------------------------------------------
// VOICE COMMAND IDs — REAL VALUES from factory firmware
//
// These come from WonderEcho's Command Word and Playback Phrase Protocol List.
// Factory firmware (CI1302_English_SingleMic_V00729) returns the command byte
// (4th byte of the AA 55 protocol frame) via I2C register 100.
//
// Verified working via gary_voice_poc.ino:
//   "go straight" → 0x01
//   "turn left"   → 0x03
//   "stop"        → 0x09
//   "dance"       → 0x6C
//   "execute action one" → 0x80
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
#define CMD_ATTENTION   0x0A   // "attention"      → Gary STAY
#define CMD_GET_DOWN    0x0B   // "get down"
#define CMD_SIT_DOWN    0x0C   // "sit down"       → Gary SIT
#define CMD_SPEED_UP    0x0D   // "speed up"       → Gary GO_FASTER
#define CMD_SLOW_DOWN   0x0E   // "slow down"
#define CMD_RAISE_BODY  0x0F   // "raise the body" → Gary WAKE_UP
#define CMD_LOWER_BODY  0x10   // "lower the body"
#define CMD_MAKE_SOUND  0x11   // "make a sound"

// --- Speech / interaction ---
#define CMD_HELLO       0x1A   // "hello"          → Gary SAY_HELLO
#define CMD_INTRODUCE   0x1B   // "introduce yourself"
#define CMD_SHOW_SKILL  0x1C   // "show a skill"   → Gary SHOW_OFF
#define CMD_MARCH       0x1D   // "march"          → Gary TURN_AROUND placeholder
#define CMD_SHAKE_HEAD  0x1E   // "shake head"     → Gary LOOK_AROUND
#define CMD_DIVE_FWD    0x1F   // "dive forward"
#define CMD_DIVE_BACK   0x20   // "dive backward"
#define CMD_COMBAT      0x21   // "combat mode"    → Gary FIGHT
#define CMD_SQUAT       0x22   // "squat down"
#define CMD_SHAKE_OFF   0x23   // "shake it off"   → Gary SHAKE

// --- Animations / dance ---
#define CMD_DANCE       0x6C   // "dance"          → Gary DANCE
#define CMD_PUSH_UP     0x6D   // "push up"
#define CMD_TWIST       0x71   // "twist"
#define CMD_SIT_UP      0x72   // "sit up"
#define CMD_BOW         0x73   // "bow"            → Gary BOW
#define CMD_SPREAD      0x74   // "spread the hands"
#define CMD_WAVE        0x75   // "wave hand"      → Gary WAVE
#define CMD_MARCH_PLACE 0x76   // "march in place"
#define CMD_FALL_STAND  0x77   // "fall and stand up"
#define CMD_STRUT       0x78   // "strut"
#define CMD_LEFT_HOOK   0x79   // "left hook"
#define CMD_RIGHT_HOOK  0x7A   // "right hook"

// --- Action slots — say "execute action ONE" through "execute action TEN" ---
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

// --- Action slots 11-30 — say "execute action ELEVEN" through "execute action THIRTY" ---
#define CMD_ACTION_11   0xA1   // Gary COME_HERE
#define CMD_ACTION_12   0xA2   // Gary EXPLORE
#define CMD_ACTION_13   0xA3   // (reserved for future use)
#define CMD_ACTION_14   0xA4
#define CMD_ACTION_15   0xA5
#define CMD_ACTION_16   0xA6
#define CMD_ACTION_17   0xA7
#define CMD_ACTION_18   0xA8
#define CMD_ACTION_19   0xA9
#define CMD_ACTION_20   0xAA

// Legacy aliases — keep these so existing code that references CMD_GET_OUT
// or CMD_TWO_STEPS still compiles. Both now route via dispatcher to clearer
// modern equivalents.
#define CMD_GET_OUT     CMD_ACTION_10
#define CMD_TWO_STEPS   CMD_MARCH       // "march" is the closest factory phrase

// CMD_WAKE_WORD: factory firmware DOES NOT push a wake-word event to register
// 100 (POC test confirmed). The wake word "HELLO-HI-WONDER" only puts the
// chip into recognition mode internally, then commands flow.
// We don't need a separate state for wake — Gary stays in ROAMING and any
// command byte arriving promotes us to LISTENING for a follow-up window.

// -----------------------------------------------------------------------------
// EYE COLOR PALETTE — unchanged
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
// STAR WARS PALETTE — bonus colors for thematic moments
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
// TIMING / THRESHOLDS — unchanged
// -----------------------------------------------------------------------------

#define LOOP_DELAY_MS              50
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
#define DISTANCE_OBSTACLE_MM       250
#define DISTANCE_ARRIVED_MM        180
#define BATTERY_LOW_MV             6000
#define IMU_TIP_THRESHOLD_DEG      30
#define FACE_SIZE_CLOSE            55
#define FACE_SIZE_FAR              25

#define WALK_SPEED                 3.5f
#define TURN_SPEED                 2.5f
#define STEP_TIME_MS               15000
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
#define ENABLE_VOICE_PLAYBACK      0   // Factory firmware — leave 0
#define ENABLE_DEBUG_SERIAL        1

#endif // GARY_CONFIG_H
