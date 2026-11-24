#include <Arduino.h>
#include <core_sense.h>

// R03 (Sense): ultrasonic raw data, 4 x RCWL-1601 on headers H1-H4.
// Prints every 200 ms (USB serial 115200):
//   H1-H4: filtered cm / last raw cm, readings and misses per second.
// Nothing is sent to Act. Hold a flat board in front of each sensor (10, 30, 100 cm) and write
// down which header faces front / right / back / left.

void setup() {
  Serial.begin(115200);
  us_begin();
  Serial.println("R03 test_us: ultrasonic H1-H4 (cm: filtered / raw, 0 = no reading)");
}

void loop() {
  static uint32_t lastPrint = 0, loops = 0;
  static uint32_t prevRead[US_COUNT], prevMiss[US_COUNT];
  us_update();
  loops++;

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    for (int i = 0; i < US_COUNT; i++) {
      Serial.printf("H%d %6.1f / %6.1f cm (%2lu ok %2lu miss/s) | ", i + 1, usDistCm[i], usRawCm[i],
                    (usReadings[i] - prevRead[i]) * 5, (usMisses[i] - prevMiss[i]) * 5);
      prevRead[i] = usReadings[i];
      prevMiss[i] = usMisses[i];
    }
    Serial.printf("loops/s %lu\n", loops * 5);
    loops = 0;
  }
}
