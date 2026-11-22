#include <Arduino.h>

// Ball possession sensor test (act core), pin 38
// Prints digitalRead every 100 ms: 0 = LOW = ball detected, 1 = HIGH = no ball

#define BALL_PIN 38

void setup() {
  Serial.begin(115200);
  pinMode(BALL_PIN, INPUT_PULLUP); // no ball = HIGH, pin never floats
}

void loop() {
  int v = digitalRead(BALL_PIN);
  Serial.printf("pin 38 = %d  %s\n", v, v == LOW ? "BALL" : "-");
  delay(100);
}
