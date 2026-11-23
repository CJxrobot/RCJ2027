#include <Arduino.h>
#include <core_act.h>

// T08 (Act): heading hold + rotate() toward the front goal. Robot does not translate.
// START press toggles mode: HOLD (always 90) / GOAL (rotate to the front goal if seen).
// Pass: twisted by hand, returns within 2 deg of 90 in < 0.5 s;
//       GOAL mode settles with the front goal at 90 +- 5 deg

void setup() {
  core_act_init();
  camera_v2_begin();
  strategy_begin();
  wait_start("T08 head");
}

void loop() {
  static bool goalMode = false;
  if (start_pressed()) goalMode = !goalMode;

  update_gyro_sensor();
  read_camera_v2();
  read_sense();

  float target = goalMode ? rotate(fieldData) : 90.0f;
  set_target_heading(target);

  if (line_flag()) MotorStop();      // safety: line seen, do nothing
  else Vector_Motion(0, 0, 0);

  EVERY_MS(tp, 100) {
    Serial.printf("%s | heading %6.1f | target %6.1f | error %5.1f | front goal %s %.0f deg\n",
                  goalMode ? "GOAL" : "HOLD", current_heading(), robot.robot_heading,
                  wrap180(robot.robot_heading - current_heading()),
                  fieldData.front_goal.seen ? "seen" : "---", fieldData.front_goal.angle);
  }
}
