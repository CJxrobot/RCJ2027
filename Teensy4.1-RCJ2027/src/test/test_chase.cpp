#include <Arduino.h>
#include <core_act.h>

// T09 (Act): merge_ball_data() + chase_ball(), with line priority for safety.
// First run with the wheels off the ground and DRIVE 0 (START press toggles DRIVE).
// Ball fusion: both seen and within MERGE_MAX_ERR_DEG -> FUSED (camera angle + IR distance),
//              disagree -> IR. The print shows both sources, their angle difference and the result.
// Pass: source switches FUSED / CAM / IR / NONE as expected; mode ORBIT -> CAPTURE -> PUSH as the
//       robot gets behind the ball; robot circles behind the ball and pushes it forward

void setup() {
  core_act_init();
  camera_begin();
  strategy_begin();
  wait_start("T09 chase");
}

void loop() {
  static bool drive = false;
  if (start_pressed()) drive = !drive;

  update_gyro_sensor();
  read_camera();
  uint8_t got = read_sense();

  LineCtl line = back_to_field(got & SENSE_GOT_LINE);
  set_ramp(!line.active);            // line escape: full speed at once, no ramp
  MergedBall ball = merge_ball_data(fieldData, irBall);
  Vec2 v = line.active ? line.v : chase_ball(ball, fieldData);

  set_target_heading(90);
  if (picked_up()) MotorStop();
  else if (drive) Vector_Motion(v.x, v.y, 0);
  else MotorStop();

  EVERY_MS(tp, 100) {
    float diff = (fieldData.ball.seen && irBall.found) ? fabsf(wrap180(fieldData.ball.angle - irBall.angle)) : -1;
    Serial.printf("DRIVE %d | cam %s %5.1f deg %4u px | ir %s %5.1f deg str %2u | diff %5.1f | -> %-5s %5.1f deg str %2.0f | %-7s v (%5.1f, %5.1f)%s\n",
                  drive, fieldData.ball.seen ? "on " : "off", fieldData.ball.angle, fieldData.ball.dist,
                  irBall.found ? "on " : "off", irBall.angle, irBall.strength, diff,
                  ball_src_name(ball.src), ball.angle, ball.strength,
                  chase_mode_name(), v.x, v.y, line.active ? " LINE" : "");
  }
}
