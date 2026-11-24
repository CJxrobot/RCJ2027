#include <Arduino.h>
#include <core_act.h>

// T05 (Act): kicker only, no motors.
// Pin 38: LOW = ball, HIGH = no ball (INPUT, as tested on the robot).
//
// 1. Press START: kicking is still OFF. Watch "pin38": no ball -> 1, ball -> 0.
// 2. Only when pin38 is right, press START again: kicking ON (START toggles ON / OFF).
// Pass: no kick without a ball; ball -> kick; 2 kicks (500 ms apart) then a 2 s rest (REST);
//       worst loop < 1000 us

void setup() {
  core_act_init();
  MotorStop();
  kicker_begin();
  wait_start("T05 kick");
}

void loop() {
  static LoopMeter lm;
  static bool kickOn = false;
  static uint32_t lowMs = 0, lastMs = millis();
  lm.begin();

  if (start_pressed()) kickOn = !kickOn;

  kicker_update();
  bool ball = has_ball();
  if (ball) lowMs += millis() - lastMs;      // how long pin 38 was LOW in this second
  lastMs = millis();
  if (kickOn && ball) kicker_fire();

  lm.end();
  if (lm.report()) {
    Serial.printf("KICK %s | pin38 %d (LOW %4lu ms of the last second) | %-8s ready %d | shots left %u%s | kicks %lu | worst %lu us\n",
                  kickOn ? "ON " : "OFF", digitalReadFast(KICK_BALL_PIN), lowMs,
                  kicker_state_name(), kicker_ready(), kicker_shots_left(),
                  kicker_resting() ? " REST" : "", kicker_count(), lm.worst);
    lowMs = 0;
    lm.reset();
  }
}
