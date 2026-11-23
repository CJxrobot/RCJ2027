#include <Arduino.h>
#include <core_sense.h>

// T03 (Sense): how many line scans per second, with and without the old 1 ms IR wait.
// Press LINE_SCAN_BUTTON (pin 36) to switch mode.
// Pass: NEW mode is at least 2x OLD mode.

static bool oldMode = true;

void setup() {
  core_sense_init();
  analogWriteFrequency(LS_BRIGHTNESS, 20000);
  analogWrite(LS_BRIGHTNESS, 105);   // same as sense_ir.cpp / line_sensing.cpp calibration
  Serial.println("T03 test_line_scan_rate: press pin 36 to switch OLD / NEW");
}

void loop() {
  static uint32_t scans = 0, lastPrint = 0, lastPress = 0, worstUs = 0;
  static bool lastBtn = HIGH;

  bool btn = digitalRead(LINE_SCAN_BUTTON);
  if (lastBtn == HIGH && btn == LOW && millis() - lastPress > 200) {
    lastPress = millis();
    oldMode = !oldMode;
    scans = 0; worstUs = 0;
  }
  lastBtn = btn;

  uint32_t t0 = micros();
  fast_update_line_sensor();          // NEW: IR is sampled inside, via ir_sample()
  if (oldMode) update_ball_sensor();  // OLD: extra 1 ms busy wait per loop
  send_sense_packet(lineState, IrBall());   // no IR here: only the scan is timed
  uint32_t dt = micros() - t0;
  if (dt > worstUs) worstUs = dt;
  scans++;

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.printf("%s mode: %5lu scans/s, worst loop %4lu us\n",
                  oldMode ? "OLD" : "NEW", scans, worstUs);
    scans = 0; worstUs = 0;
  }
}
