#include <Arduino.h>
#include <core_act.h>

// T10 (Act): Vector_Field() near goals (ultrasonic part returns 0 until R03).
// After START the robot drives at DRIVE_SPEED; each START press turns the direction by 90 deg.
// Pass: slows near each goal and does not enter the penalty area
#define DRIVE_SPEED 40.0f

void setup() {
  core_act_init();
  camera_v2_begin();
  strategy_begin();
  wait_start("T10 field");
}

void loop() {
  static float driveDir = 90;
  if (start_pressed()) driveDir = fmodf(driveDir + 90, 360);

  update_gyro_sensor();
  read_camera_v2();
  uint8_t got = read_sense();

  LineCtl line = back_to_field(got & SENSE_GOT_LINE);
  Vec2 f = Vector_Field(fieldData, US_Sensor());
  float vx = DRIVE_SPEED * cosf(driveDir * DEG_TO_RAD) + f.x;
  float vy = DRIVE_SPEED * sinf(driveDir * DEG_TO_RAD) + f.y;
  if (line.active) { vx = line.v.x; vy = line.v.y; }
  set_target_heading(90);
  Vector_Motion(vx, vy, 0);

  EVERY_MS(tp, 100) {
    Serial.printf("dir %3.0f | front %4u cm | back %4u cm | F (%5.1f, %5.1f) | v (%5.1f, %5.1f)%s\n",
                  driveDir, fieldData.front_goal.dist, fieldData.back_goal.dist,
                  f.x, f.y, vx, vy, line.active ? " LINE" : "");
  }
}
