#include <Arduino.h>
#include <core_act.h>

// T08 (Act): shooting with the ball held (reference code, tested) + corner shot.
//   No ball:   stands still, heading 90 (twist it by hand, it comes back: PD heading hold)
//   Ball held: front goal seen -> push forward at AIM_PUSH_SPEED and turn to the goal
//              (AIM_HEADING_P); no goal -> push at NOGOAL_PUSH_SPEED, heading 90.
//              Kick once after KICK_HOLD_MS.
//   Corner:    in_corner() -> back up, aim, push + kick, wait for the ball to leave.
// START press toggles FORCE CORNER: the next ball held runs the corner sequence anywhere.
// Lift the robot: motors stop (pitch > PICKUP_PITCH_DEG).
// Pass: twisted by hand -> back within 2 deg of 90 in < 0.5 s; ball held -> turns to the
//       goal while pushing, one kick; corner -> the 4 phases in order

void setup() {
  core_act_init();
  camera_v2_begin();
  kicker_begin();
  strategy_begin();
  wait_start("T08 shoot");
}

void loop() {
  static bool forceCorner = false;
  if (start_pressed()) forceCorner = !forceCorner;

  update_gyro_sensor();
  read_camera_v2();
  uint8_t got = read_sense();
  kicker_update();

  // FORCE CORNER: pretend the front goal is close and far to the side while the ball is held
  FieldData f = fieldData;
  if (forceCorner && has_ball()) {
    f.front_goal.seen = true;
    f.front_goal.dist = (uint16_t)(CORNER_GOAL_DIST_CM - 10);
    if (fabsf(wrap180(f.front_goal.angle - 90)) <= CORNER_GOAL_DEG) f.front_goal.angle = 90 + CORNER_GOAL_DEG + 10;
    f.t_ms = millis();
  }

  ShootCtl shot = shoot_control(f);
  LineCtl line = back_to_field(got & SENSE_GOT_LINE);
  set_ramp(!line.active);            // line escape: full speed at once, no ramp

  Vec2 v = shot.v;
  set_aim(shot.aiming);
  set_target_heading(shot.active ? shot.heading : 90);
  if (line.active) { v = line.v; set_aim(false); set_target_heading(90); }

  if (picked_up()) MotorStop();
  else Vector_Motion(v.x, v.y, 0);

  static uint32_t lastKicks = 0;
  if (kicker_count() != lastKicks) {
    lastKicks = kicker_count();
    Serial.printf("%lu ms: KICK #%lu (%s)\n", millis(), lastKicks, shot.phase);
  }
  EVERY_MS(tp, 100) {
    Serial.printf("%s | ball %d | phase %-7s | heading %6.1f target %6.1f | v (%4.0f, %4.0f) | goal %s %.0f deg %u cm | kicker %s shots %u%s\n",
                  forceCorner ? "FORCE-CORNER" : "normal", has_ball(), shot.active ? shot.phase : "-",
                  current_heading(), robot.robot_heading, v.x, v.y,
                  fieldData.front_goal.seen ? "seen" : "---", fieldData.front_goal.angle, fieldData.front_goal.dist,
                  kicker_state_name(), kicker_shots_left(), picked_up() ? " LIFTED" : "");
  }
}
