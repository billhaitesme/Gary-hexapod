// =============================================================================
// matrix_test.ino — PIN SCANNER
//
// Tries every plausible (SCK, DIN) pin pair on the K12 ESP32. For each pair,
// it lights ALL pixels at max brightness for 4 seconds, then prints which pair
// it just tried via Serial.
//
// Watch the matrix. When it lights up — write down the most recent Serial line.
// That's your wiring.
//
// To use:
//   - This file in C:\Users\BillH\Documents\Arduino\matrix_test\
//   - Plus WMMatrixLed.h and WMMatrixLed.cpp (already there)
//   - Compile, upload, open Serial Monitor at 115200 baud
//   - Watch matrix. The instant it lights, glance at Serial Monitor.
// =============================================================================

#include "WMMatrixLed.h"

// Candidate pins. Includes Hiwonder's IO ports plus a few ESP32 pins
// that could plausibly be exposed on a 4-pin sensor port behind ultrasonic.
const uint8_t CANDIDATES[] = {
  14, 17, 18, 19, 25, 26, 27, 32, 33
};
const uint8_t N = sizeof(CANDIDATES);

void try_pair(uint8_t sck, uint8_t din) {
  Serial.print("Trying SCK=");
  Serial.print(sck);
  Serial.print(" DIN=");
  Serial.println(din);

  WMMatrixLed m(sck, din);
  m.setBrightness(8);
  m.clearScreen();
  delay(50);

  uint8_t all_on[16];
  for (int i = 0; i < 16; i++) all_on[i] = 0xFF;
  m.drawBitmap(0, 0, 16, all_on);

  delay(4000);  // 4 seconds to spot it lighting up

  m.clearScreen();
  delay(200);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("=================================");
  Serial.println("MATRIX PIN SCANNER");
  Serial.println("Watch matrix. When it lights up,");
  Serial.println("note the most recent Serial line.");
  Serial.println("=================================");
  delay(2000);

  // Try every ordered pair (sck, din) where sck != din
  for (uint8_t i = 0; i < N; i++) {
    for (uint8_t j = 0; j < N; j++) {
      if (i == j) continue;
      try_pair(CANDIDATES[i], CANDIDATES[j]);
    }
  }

  Serial.println();
  Serial.println("=== ALL PAIRS TESTED ===");
  Serial.println("If matrix never lit, wiring/power is wrong.");
}

void loop() {
  // Done. Just sit.
  delay(1000);
}
