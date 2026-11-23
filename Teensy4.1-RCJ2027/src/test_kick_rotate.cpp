#include <Arduino.h>
#include <core_act.h>

// T08 (Act): heading hold + short turn on a kick. The robot does not drive.
//   No ball:   holds 90 deg (twist it by hand, it comes back)
//   Ball held: kicks, then turns toward the front goal for KICK_ROTATE_MS
//              (at most KICK_ROTATE_MAX_DEG), then back to 90
// Pass: twisted by hand -> back within 2 deg of 90 in < 0.5 s;
//       each kick -> one turn toward the goal lasting ~KICK_ROTATE_MS, then back to 90

void setup() {
  core_act_init();
  camera_v2_begin();
  kicker_begin();
  strategy_begin();
  wait_start("T08 kick");
}

void loop() {
  update_gyro_sensor();
  read_camera_v2();
  read_sense();
  kicker_update();

  bool kicked = has_ball() && kicker_fire();
  set_target_heading(kick_rotate(kicked, fieldData));

  if (line_flag()) MotorStop();      // safety: line seen, do nothing
  else Vector_Motion(0, 0, 0);

  if (kicked) Serial.printf("%lu ms: KICK, turn to %.1f (goal %s %.0f deg)\n", millis(),
                            robot.robot_heading, fieldData.front_goal.seen ? "seen" : "---",
                            fieldData.front_goal.angle);
  EVERY_MS(tp, 100) {
    Serial.printf("%s | heading %6.1f | target %6.1f | error %5.1f | kicker %s ready %d\n",
                  kick_rotating() ? "TURN" : "HOLD", current_heading(), robot.robot_heading,
                  wrap180(robot.robot_heading - current_heading()), kicker_state_name(), kicker_ready());
  }
}
