#include <Arduino.h>
#include <core_act.h>

// T06 (Act): main loop speed with every input read and the strategy math running.
// Sense runs test_sense_tx, Cam runs main.py (or camera_test/cam_fake_v2.py). Motors never move. OLED not drawn.
// Pass: >= 1000 loops/s, worst loop < 2000 us

void setup() {
  core_act_init();
  camera_v2_begin();
  kicker_begin();
  strategy_begin();
  MotorStop();
  drawMessage("T06 rate");
}

void loop() {
  static LoopMeter lm;
  lm.begin();

  update_gyro_sensor();
  read_camera_v2();
  uint8_t got = read_sense();
  kicker_update();

  // Same math as the game, results not used
  LineCtl line = back_to_field(got & SENSE_GOT_LINE);
  MergedBall ball = merge_ball_data(fieldData, irBall);
  Vec2 v = chase_ball(ball, fieldData);
  Vec2 f = Vector_Field(v, fieldData, US_Sensor());
  volatile float sink = v.x + f.x + shoot_control(fieldData).heading + line.v.x;
  (void)sink;

  lm.end();
  if (lm.report()) {
    Serial.printf("loops/s %6lu | worst %4lu us | gyro %.1f | cam age %lu ms | ball %s\n",
                  lm.loops, lm.worst, gyroData.heading,
                  fieldData.t_ms ? millis() - fieldData.t_ms : 0, ball_src_name(ball.src));
    lm.reset();
  }
}
