#include <Arduino.h>
#include <core_act.h>

// T10 (Act): Vector_Field() keeps the robot out of OUR OWN penalty area while chasing.
// Put the ball next to / behind our penalty area and let the robot chase it.
// START press toggles the field ON / OFF so you can compare (OFF: robot cuts through the area).
// Pass (field ON): robot slides along the penalty area edge and never crosses it,
//                  back goal distance never drops below PENALTY_R_CM

void setup() {
  core_act_init();
  camera_v2_begin();
  strategy_begin();
  wait_start("T10 field");
}

void loop() {
  static bool fieldOn = true;
  static float minDist = 9999;
  if (start_pressed()) { fieldOn = !fieldOn; minDist = 9999; }

  update_gyro_sensor();
  read_camera_v2();
  uint8_t got = read_sense();

  LineCtl line = back_to_field(got & SENSE_GOT_LINE);
  Vec2 chase = chase_ball(merge_ball_data(fieldData, irBall), fieldData);
  Vec2 v = fieldOn ? Vector_Field(chase, fieldData, US_Sensor()) : chase;
  if (line.active) v = line.v;
  set_target_heading(90);
  Vector_Motion(v.x, v.y, 0);

  if (fieldData.back_goal.seen && fieldData.back_goal.dist < minDist) minDist = fieldData.back_goal.dist;
  EVERY_MS(tp, 100) {
    Serial.printf("field %s | back goal %3.0f deg %4u cm (min %4.0f) | chase (%5.1f, %5.1f) -> v (%5.1f, %5.1f)%s\n",
                  fieldOn ? "ON " : "OFF", fieldData.back_goal.angle, fieldData.back_goal.dist, minDist,
                  chase.x, chase.y, v.x, v.y, line.active ? " LINE" : "");
  }
}
