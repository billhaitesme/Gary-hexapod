// =============================================================================
// matrix_test.ino — TM640B with PDF-correct pins
//
// Per Hiwonder's Dot Matrix Display tutorial PDF (page 5):
//   WMMatrixLed matrix(33, 32);  // SCK / DIN
//
// Tries this and the inverted-polarity variant.
// =============================================================================

#include "WMMatrixLed.h"

void run_test(uint8_t test_num, uint8_t sck, uint8_t din, uint8_t color_index) {
  Serial.print(">>> TEST ");
  Serial.print(test_num);
  Serial.print(": SCK=");
  Serial.print(sck);
  Serial.print(" DIN=");
  Serial.print(din);
  Serial.print(" colorIndex=");
  Serial.println(color_index);

  WMMatrixLed m(sck, din);
  m.setBrightness(8);
  m.setColorIndex(color_index);
  m.clearScreen();
  delay(100);

  uint8_t all_on[16];
  for (int i = 0; i < 16; i++) all_on[i] = 0xFF;
  m.drawBitmap(0, 0, 16, all_on);
  delay(5000);

  m.clearScreen();
  delay(300);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("====================================");
  Serial.println("TM640B TEST - PDF pins (33, 32)");
  Serial.println("====================================");
  delay(1500);

  // Test 1: PDF default
  run_test(1, 33, 32, 1);

  // Test 2: PDF default, inverted
  run_test(2, 33, 32, 0);

  // Test 3: pins swapped
  run_test(3, 32, 33, 1);

  // Test 4: pins swapped, inverted
  run_test(4, 32, 33, 0);

  // Test 5: scrolling text using PDF method
  Serial.println(">>> TEST 5: scrolling 'GARY' on (33, 32)");
  WMMatrixLed scroll_m(33, 32);
  scroll_m.setBrightness(5);
  scroll_m.clearScreen();
  const char* text = "GARY";
  int textWidth = 6 * strlen(text);
  for (int x = 16; x > -textWidth; x--) {
    scroll_m.drawStr(x, 8, text);
    delay(120);
  }

  Serial.println();
  Serial.println("=== ALL TESTS DONE ===");
  Serial.println("Tell Claude which test number lit");
  Serial.println("the matrix (or 'none').");
}

void loop() {
  delay(1000);
}
