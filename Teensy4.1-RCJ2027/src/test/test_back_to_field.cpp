#include <Arduino.h>
#include <core_act.h>

// T07 (Act): back_to_field().
// After START the robot drives at DRIVE_SPEED. Each START press turns the drive
// direction by 90 deg (front -> left -> back -> right). Drive into straight and curved lines.
// Pass: never fully crosses a line in 10 of 10 tries, no bouncing back onto it
#define DRIVE_SPEED 40.0f

void setup() {
  core_act_init();
  strategy_begin();
  wait_start("T07 line");
}

void loop() {
  static float driveDir = 90;
  if (start_pressed()) driveDir = fmodf(driveDir + 90, 360);

  update_gyro_sensor();
  uint8_t got = read_sense();
  LineCtl line = back_to_field(got & SENSE_GOT_LINE);

  float vx, vy;
  if (line.active) { vx = line.v.x; vy = line.v.y; }
  else { vx = DRIVE_SPEED * cosf(driveDir * DEG_TO_RAD); vy = DRIVE_SPEED * sinf(driveDir * DEG_TO_RAD); }
  if (picked_up()) MotorStop();
  else Vector_Motion(vx, vy, 0);

  static bool lastActive = false;
  if (line.active != lastActive) {
    Serial.printf("%lu ms: line %s, dir %.0f deg, away (%.0f, %.0f)\n", millis(),
                  line.active ? "ON" : "cleared", line.line_dir, line.v.x, line.v.y);
    lastActive = line.active;
  }
}
