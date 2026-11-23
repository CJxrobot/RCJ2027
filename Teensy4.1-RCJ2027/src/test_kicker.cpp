#include <Arduino.h>
#include <core_act.h>

// T05 (Act): kicker only, no motors.
// Press START, then push a ball into the catcher: it kicks when ready.
// Pass: worst loop < 1000 us during kicks, kicks only when ready and holding the ball,
//       2 kicks (500 ms apart) then a 2 s rest (REST shown) before the next kick

void setup() {
  core_act_init();
  MotorStop();
  kicker_begin();
  wait_start("T05 kick");
}

void loop() {
  static LoopMeter lm;
  lm.begin();

  kicker_update();
  if (has_ball()) kicker_fire();

  lm.end();
  if (lm.report()) {
    Serial.printf("loops/s %6lu | worst %4lu us | %-8s | ball %d | ready %d | shots left %u%s | kicks %lu\n",
                  lm.loops, lm.worst, kicker_state_name(), has_ball(), kicker_ready(),
                  kicker_shots_left(), kicker_resting() ? " REST" : "", kicker_count());
    lm.reset();
  }
}
