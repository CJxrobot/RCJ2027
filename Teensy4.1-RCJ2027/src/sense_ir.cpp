#include <Arduino.h>
#include <core_sense.h>

// Sense core: IR ball (vector sum, like ir_sensing.cpp) + line, both sent to the act core
//   IR:   BB found angleL angleH count checksum, every loop
//   Line: same as sense_defense.cpp (PIN_0/PIN_1 + AA packet when a line is seen)
// Angle: 0 = right, 90 = front, CCW. Near sensors seeing the ball -> near only, else far

#define IR_N         20
#define IR_WINDOW_US 1000 // sampling window, one ball pulse cycle is ~833 us
#define IR_MIN       0    // count above this = ball seen

#define M2_UNUSED (0x1740UL << 16) // mux 2 channel 6, 8, 9, 10, 12 have no sensor
#define M2_IGNORE (0x8024UL << 16) // R_out 15, B_out 5, L_out 2: read but never reported

// out 1-20 -> pin
const uint8_t IR_PIN[IR_N] = {
  12, 11, 10,  9,  8,  7,  6,  5,  4,  3, // out 1-10
   2, 23, 22, 21, 30, 29, 39, 38, 32, 33  // out 11-20
};

struct IrSensor { uint8_t out; int16_t deg; };
const IrSensor IR_NEAR[] = {
  {2, 0}, {4, 45}, {5, 60}, {6, 85}, {8, 95}, {9, 120}, {10, 135},
  {12, 180}, {14, 230}, {16, 260}, {18, 280}, {20, 310}
};
const IrSensor IR_FAR[] = {
  {1, 0}, {3, 45}, {7, 90}, {11, 135}, {13, 180}, {15, 225}, {17, 270}, {19, 315}
};
const int NEAR_N = sizeof(IR_NEAR) / sizeof(IR_NEAR[0]);
const int FAR_N  = sizeof(IR_FAR)  / sizeof(IR_FAR[0]);

uint16_t irCount[IR_N]; // index = out - 1

// Count LOW samples on every sensor over IR_WINDOW_US
void readIR() {
  for (int i = 0; i < IR_N; i++) irCount[i] = 0;
  uint32_t start = micros();
  while (micros() - start < IR_WINDOW_US) {
    for (int i = 0; i < IR_N; i++) {
      if (digitalRead(IR_PIN[i]) == LOW) irCount[i]++;
    }
  }
}

// Ball angle from one group, -1 if no sensor in the group sees the ball
// Vector sum of the strongest sensor + its two neighbours (group sorted by angle), count-weighted
float groupAngle(const IrSensor* g, int n, uint16_t& best) {
  int k = 0;
  best = 0;
  for (int i = 0; i < n; i++) {
    uint16_t c = irCount[g[i].out - 1];
    if (c > best) { best = c; k = i; }
  }
  if (best <= IR_MIN) return -1;

  float x = 0, y = 0;
  for (int d = -1; d <= 1; d++) {
    const IrSensor& s = g[(k + d + n) % n];
    float c = irCount[s.out - 1];
    x += c * cosf(s.deg * DEG_TO_RAD);
    y += c * sinf(s.deg * DEG_TO_RAD);
  }
  float a = atan2f(y, x) * RAD_TO_DEG;
  return a < 0 ? a + 360.0f : a;
}

void setup() {
  core_sense_init();
  for (int i = 0; i < IR_N; i++) pinMode(IR_PIN[i], INPUT);
  analogWriteFrequency(LS_BRIGHTNESS, 20000);
  analogWrite(LS_BRIGHTNESS, 105); // same as line_sensing.cpp calibration
}

void loop() {
  // IR ball: near first, else far
  readIR();
  uint16_t best;
  float angle = groupAngle(IR_NEAR, NEAR_N, best);
  if (angle < 0) angle = groupAngle(IR_FAR, FAR_N, best);

  // Distance: how many sensors (all 20) see the ball, 1 = farthest, more = closer
  uint16_t dist = 0;
  for (int i = 0; i < IR_N; i++) {
    if (irCount[i] > IR_MIN) dist++;
  }

  if (angle >= 0) send_ball_ir(true, (uint16_t)lroundf(angle) % 360, dist);
  else            send_ball_ir(false, 0xFFFF, 0);

  // Line
  fast_update_line_sensor();
  lineState |= M2_UNUSED | M2_IGNORE;
  send_line_state();
  digitalWriteFast(LED_BUILTIN, lineState != 0xFFFFFFFF); // LED 13 on = line seen
}
