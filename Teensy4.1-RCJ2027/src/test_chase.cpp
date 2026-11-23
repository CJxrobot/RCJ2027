#include <Arduino.h>
#include <core_act.h>

// T09 (Act): merge_ball_data() + chase_ball(), with line priority for safety.
// First run with the wheels off the ground and DRIVE 0 (START press toggles DRIVE).
// Pass: source switches CAM / IR / NONE as expected; robot circles behind the ball

void setup() {
  core_act_init();
  camera_v2_begin();
  strategy_begin();
  wait_start("T09 chase");
}

void loop() {
  static bool drive = false;
  if (start_pressed()) drive = !drive;

  update_gyro_sensor();
  read_camera_v2();
  uint8_t got = read_sense();

  LineCtl line = back_to_field(got & SENSE_GOT_LINE);
  MergedBall ball = merge_ball_data(fieldData, irBall);
  Vec2 v = line.active ? line.v : chase_ball(ball, fieldData);

  set_target_heading(90);
  if (drive) Vector_Motion(v.x, v.y, 0);
  else MotorStop();

  EVERY_MS(tp, 100) {
    Serial.printf("DRIVE %d | src %-4s | ball %5.1f deg close %.2f | cam %s %5.1f | ir %s %5.1f | v (%5.1f, %5.1f)%s\n",
                  drive, ball_src_name(ball.src), ball.angle, ball.closeness,
                  fieldData.ball.seen ? "on " : "off", fieldData.ball.angle,
                  irBall.found ? "on " : "off", irBall.angle, v.x, v.y, line.active ? " LINE" : "");
  }
}
