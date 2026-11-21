#include <Arduino.h>

// IR ball sensing test, MAX version (sense core), library not used
// Angle: 0 = right, 90 = front, CCW
// Each sensor: count LOW samples over 1 ms, count > IR_MIN = ball seen
// Near sensors seeing the ball -> use near only, ignore far
// Otherwise far sensors -> use far
// Ball angle = angle of the sensor with the highest count in that group

#define IR_COUNT     20
#define IR_WINDOW_US 1000 // sampling window, one ball pulse cycle is ~833 us
#define IR_MIN       0    // count above this = ball seen
#define PRINT_MS     100

// out 1-20 -> pin
const uint8_t IR_PIN[IR_COUNT] = {
  12, 11, 10,  9,  8,  7,  6,  5,  4,  3, // out 1-10
   2, 23, 22, 21, 30, 29, 39, 38, 32, 33  // out 11-20
};

// Groups: out number, angle
struct IrSensor { uint8_t out; int16_t deg; };
const IrSensor IR_NEAR[] = {
  {1, 0}, {4, 45}, {5, 60}, {6, 85}, {8, 95}, {9, 120}, {10, 135},
  {13, 180}, {15, 230}, {16, 260}, {18, 280}, {19, 310}
};
const IrSensor IR_FAR[] = {
  {2, 0}, {3, 45}, {7, 90}, {11, 135}, {12, 180}, {14, 225}, {17, 270}, {20, 315}
};
const int NEAR_N = sizeof(IR_NEAR) / sizeof(IR_NEAR[0]);
const int FAR_N  = sizeof(IR_FAR)  / sizeof(IR_FAR[0]);

uint16_t irCount[IR_COUNT]; // index = out - 1
int16_t  irDeg[IR_COUNT];   // for printing
char     irType[IR_COUNT];  // 'N' near, 'F' far, for printing

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < IR_COUNT; i++) pinMode(IR_PIN[i], INPUT);

  for (int i = 0; i < NEAR_N; i++) { irType[IR_NEAR[i].out - 1] = 'N'; irDeg[IR_NEAR[i].out - 1] = IR_NEAR[i].deg; }
  for (int i = 0; i < FAR_N;  i++) { irType[IR_FAR[i].out - 1]  = 'F'; irDeg[IR_FAR[i].out - 1]  = IR_FAR[i].deg; }
}

// Count LOW samples on every sensor over IR_WINDOW_US
void readIR() {
  for (int i = 0; i < IR_COUNT; i++) irCount[i] = 0;
  uint32_t start = micros();
  while (micros() - start < IR_WINDOW_US) {
    for (int i = 0; i < IR_COUNT; i++) {
      if (digitalRead(IR_PIN[i]) == LOW) irCount[i]++;
    }
  }
}

// Sensor with the highest count in a group, nullptr if none sees the ball
const IrSensor* groupMax(const IrSensor* g, int n) {
  const IrSensor* best = &g[0];
  for (int i = 1; i < n; i++) {
    if (irCount[g[i].out - 1] > irCount[best->out - 1]) best = &g[i];
  }
  return irCount[best->out - 1] > IR_MIN ? best : nullptr;
}

void loop() {
  static uint32_t lastPrint = 0;

  readIR();

  const char* mode = "NEAR";
  const IrSensor* ball = groupMax(IR_NEAR, NEAR_N);
  if (!ball) { // no near: try far
    mode = "FAR";
    ball = groupMax(IR_FAR, FAR_N);
  }

  if (millis() - lastPrint < PRINT_MS) return;
  lastPrint = millis();

  Serial.print("OUT :");
  for (int i = 0; i < IR_COUNT; i++) Serial.printf("%5d", i + 1);
  Serial.print("\nTYPE:");
  for (int i = 0; i < IR_COUNT; i++) Serial.printf("%5c", irType[i]);
  Serial.print("\nDEG :");
  for (int i = 0; i < IR_COUNT; i++) Serial.printf("%5d", irDeg[i]);
  Serial.print("\nCNT :");
  for (int i = 0; i < IR_COUNT; i++) Serial.printf("%5d", irCount[i]);
  if (!ball) Serial.printf("\nBall NONE\n\n");
  else Serial.printf("\nBall %s  angle %d  max out%d = %d\n\n", mode, ball->deg, ball->out, irCount[ball->out - 1]);
}
