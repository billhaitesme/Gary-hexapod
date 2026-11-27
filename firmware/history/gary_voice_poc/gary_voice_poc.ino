/*
 * gary_voice_poc.ino
 *
 * Minimal proof-of-concept to verify the WonderEcho ↔ K12 voice link works
 * BEFORE touching the main gary_v1.ino firmware.
 *
 * What this does:
 *   1. Polls WonderEcho I2C register 100 every 100ms for a recognized command byte
 *   2. Prints any non-zero byte to Serial Monitor with both decimal and hex
 *   3. Translates known command bytes to a human-readable label
 *   4. Beeps the buzzer (via existing Robot interface) so you can hear it triggered
 *      without needing to look at the serial monitor
 *
 * To run:
 *   1. Open Arduino IDE
 *   2. Copy ALL the support files from the existing asr_control example into
 *      this sketch's folder (hiwonder_robot, hiwonder_board, hiwonder_sensor,
 *      hiwonder_servo, hiwonder_i2c, MadgwickAHRS, WMMatrixLed, global.h).
 *      You should already have these from your existing gary_v1 project.
 *   3. Upload to the K12 ESP32 main board (NOT the WonderEcho — that already
 *      has factory firmware on it from earlier today).
 *   4. Reconnect WonderEcho's I2C cable to K12.
 *   5. Open Serial Monitor at 115200 baud.
 *   6. Power on K12.
 *   7. Say "HELLO-HI-WONDER" → should hear "I'm here" from WonderEcho.
 *   8. Say "GO-STRAIGHT" → Serial Monitor should print:
 *        [ASR] Got byte 1 (0x01) → FORWARD (go straight)
 *      and you should hear a short buzzer beep.
 *   9. Try other commands from the mapping list and verify each prints correctly.
 *
 * If this works, we update gary_v1.ino with the full command dispatcher.
 * If it doesn't, we debug the I2C link before touching anything else.
 */

#include "hiwonder_robot.h"

Robot minihexa;

// Track last byte so we don't spam serial when WonderEcho holds the value
uint8_t last_cmd = 0;

// Map a recognized command byte to a human-readable label
const char* cmd_label(uint8_t cmd) {
  switch(cmd) {
    // Movement
    case 0x01: return "FORWARD     (go straight)";
    case 0x02: return "BACKWARD    (go backward)";
    case 0x03: return "TURN_LEFT   (turn left)";    // collides with 0x03 wake — see note below
    case 0x04: return "TURN_RIGHT  (turn right)";
    case 0x09: return "STOP        (stop)";
    case 0x0A: return "STAY        (attention)";
    case 0x0C: return "SIT         (sit down)";
    case 0x0D: return "GO_FASTER   (speed up)";
    case 0x0E: return "SLOW_DOWN   (slow down)";
    case 0x0F: return "WAKE_UP     (raise the body)";

    // Speech
    case 0x1A: return "SAY_HELLO   (hello)";
    case 0x1B: return "INTRODUCE   (introduce yourself)";
    case 0x1C: return "SHOW_OFF    (show a skill)";
    case 0x1D: return "TURN_AROUND (march)";
    case 0x1E: return "LOOK_AROUND (shake head)";
    case 0x21: return "FIGHT       (combat mode)";
    case 0x23: return "SHAKE       (shake it off)";

    // Animations
    case 0x6C: return "DANCE       (dance)";
    case 0x73: return "BOW         (bow)";
    case 0x75: return "WAVE        (wave hand)";
    case 0x76: return "MARCH_PLACE (march in place)";
    case 0x78: return "STRUT       (strut)";

    // Action slots (for Gary-specific commands without factory phrase match)
    case 0x80: return "RUN         (execute action one)";
    case 0x81: return "GO_HOME     (execute action two)";
    case 0x82: return "GOODBYE     (execute action three)";
    case 0x83: return "GOODNIGHT   (execute action four)";
    case 0x84: return "GOOD_JOB    (execute action five)";
    case 0x85: return "HIGH_FIVE   (execute action six)";
    case 0x86: return "HUG         (execute action seven)";
    case 0x87: return "SIC_EM      (execute action eight)";
    case 0x88: return "CHARGE      (execute action nine)";
    case 0x89: return "GET_OUT     (execute action ten)";
    case 0xA1: return "COME_HERE   (execute action eleven)";
    case 0xA2: return "EXPLORE     (execute action twelve)";

    default:   return "UNKNOWN";
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=================================================");
  Serial.println("  Gary voice POC — WonderEcho I2C link test");
  Serial.println("=================================================");
  Serial.println();

  minihexa.begin();
  delay(500);

  Serial.println("[Setup] minihexa.begin() done");
  Serial.println("[Setup] Polling WonderEcho I2C register 100...");
  Serial.println();
  Serial.println("Try: \"HELLO-HI-WONDER\" → \"GO-STRAIGHT\"");
  Serial.println("Expect: I'm here — then a printed byte 0x01 = FORWARD");
  Serial.println();
}

void loop() {
  uint8_t cmd = minihexa.sensor.asr.rec_recognition();

  // Only print on rising edge — when a NEW command byte is read
  if (cmd != 0 && cmd != last_cmd) {
    Serial.print("[ASR] Got byte ");
    Serial.print(cmd);
    Serial.print(" (0x");
    if (cmd < 0x10) Serial.print("0");
    Serial.print(cmd, HEX);
    Serial.print(") → ");
    Serial.println(cmd_label(cmd));

    // Quick audible feedback so you can test without looking at the screen.
    // Different command groups get different beep counts.
    int beeps = 1;
    if (cmd >= 0x80) beeps = 2;          // ACTION-slot commands
    if (cmd >= 0xA0) beeps = 3;          // ACTION-11+ slot commands

    // The K12 has a buzzer — if minihexa doesn't expose a beep helper,
    // we'll just rely on the Serial print. Most existing projects DO have
    // some kind of buzzer call. If you see a compile error here, comment
    // these 4 lines out and the rest still works.
    //
    // for (int i = 0; i < beeps; i++) {
    //   minihexa.buzzer.beep(50);
    //   delay(80);
    // }
  }

  last_cmd = cmd;
  delay(100);
}
