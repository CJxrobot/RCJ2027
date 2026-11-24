#include <Arduino.h>
#include <core_act.h>

// T10 (Act): whole-field Vector_Field().
// START press switches mode:
//   DRIVE 90 / 180 / 270 / 0 : drive straight that way at DRIVE_SPEED -> slows down near the
//                              front goal, back goal (and walls once R03 is in), never runs out
//   CHASE                    : chase the ball; put it next to / behind our penalty area
//                              -> robot slides along the area edge, never crosses it
// Pass: no run-out in DRIVE modes; in CHASE the printed min back goal distance stays above
//       PENALTY_R_CM
#define DRIVE_SPEED 40.0f

void setup() {
  core_act_init();
  camera_begin();
  strategy_begin();
  wait_start("T10 field");
}

void loop() {
  static int mode = 0;   // 0-3: DRIVE 90, 180, 270, 0   4: CHASE
  static float minBack = 9999;
  if (start_pressed()) { mode = (mode + 1) % 5; minBack = 9999; }

  update_gyro_sensor();
  read_camera();
  uint8_t got = read_sense();

  LineCtl line = back_to_field(got & SENSE_GOT_LINE);
  set_ramp(!line.active);            // line escape: full speed at once, no ramp
  Vec2 in;
  if (mode == 4) in = chase_ball(merge_ball_data(fieldData, irBall), fieldData);
  else {
    float dir = 90.0f * (mode + 1);
    in.x = DRIVE_SPEED * cosf(dir * DEG_TO_RAD);
    in.y = DRIVE_SPEED * sinf(dir * DEG_TO_RAD);
  }
  Vec2 v = Vector_Field(in, fieldData, US_Sensor());
  if (line.active) v = line.v;
  set_target_heading(90);
  if (picked_up()) MotorStop();
  else Vector_Motion(v.x, v.y, 0);

  if (fieldData.back_goal.seen && fieldData.back_goal.dist < minBack) minBack = fieldData.back_goal.dist;
  EVERY_MS(tp, 100) {
    if (mode == 4) Serial.print("CHASE    ");
    else Serial.printf("DRIVE %3d", (int)fmodf(90.0f * (mode + 1), 360));
    Serial.printf(" | front %4u cm | back %4u cm (min %4.0f) | in (%5.1f, %5.1f) -> v (%5.1f, %5.1f)%s\n",
                  fieldData.front_goal.dist, fieldData.back_goal.dist, minBack,
                  in.x, in.y, v.x, v.y, line.active ? " LINE" : "");
  }
}
