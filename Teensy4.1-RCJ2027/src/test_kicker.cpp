#include <Arduino.h>
#include <core_act.h>

// T05 (Act): kicker without delay(), debounced possession, motor 3 unaffected by charging.
// After START the motors stay OFF. Push a ball into the catcher: it kicks when ready.
// Optional: each START press turns motor 3 on / off (30 %) to check charging does not affect it.
// Pass: worst loop < 1000 us during kicks, kicks only when ready, motor 3 steady while charging,
//       2 kicks (500 ms apart) then a 2 s rest (REST shown) before the next kick

void setup() {
  core_act_init();
  kicker_begin();
  wait_start("T05 kick");
}

void loop() {
  static LoopMeter lm;
  static bool motorOn = false;   // motors stay off; START turns motor 3 on only if you want to check it
  static uint32_t refused = 0;
  lm.begin();

  if (start_pressed()) motorOn = !motorOn;
  SetMotorSpeed(3, motorOn ? 30 : 0);

  kicker_update();
  if (has_ball() && !kicker_fire()) refused++;

  lm.end();
  if (lm.report()) {
    Serial.printf("loops/s %6lu | worst %4lu us | %-8s | ball %d | ready %d | shots left %u%s | kicks %lu | motor3 %s\n",
                  lm.loops, lm.worst, kicker_state_name(), has_ball(), kicker_ready(),
                  kicker_shots_left(), kicker_resting() ? " REST" : "",
                  kicker_count(), motorOn ? "30%" : "off");
    lm.reset();
  }
}
