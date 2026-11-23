#include <Arduino.h>
#include <core_act.h>

// T05 (Act): kicker without delay(), debounced possession, motor 3 unaffected by charging.
// After START: motor 3 runs at 30 %. Push a ball into the catcher: it kicks when ready.
// Press START again to stop / start motor 3.
// Pass: worst loop < 1000 us during kicks, kicks only when ready, motor 3 steady while charging

void setup() {
  core_act_init();
  kicker_begin();
  wait_start("T05 kick");
}

void loop() {
  static LoopMeter lm;
  static bool motorOn = true;
  static uint32_t refused = 0;
  lm.begin();

  if (start_pressed()) motorOn = !motorOn;
  SetMotorSpeed(3, motorOn ? 30 : 0);

  kicker_update();
  if (has_ball() && !kicker_fire()) refused++;

  lm.end();
  if (lm.report()) {
    Serial.printf("loops/s %6lu | worst %4lu us | %-8s | ball %d | ready %d | kicks %lu | motor3 %s\n",
                  lm.loops, lm.worst, kicker_state_name(), has_ball(), kicker_ready(),
                  kicker_count(), motorOn ? "30%" : "off");
    lm.reset();
  }
}
