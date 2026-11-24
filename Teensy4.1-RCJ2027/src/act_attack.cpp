#include <Arduino.h>
#include <core_act.h>

// T11 (Act): attack game program, same order as the system design
//
//   update_gyro / read camera / read sense / kicker_update
//   if picked up (|pitch| > PICKUP_PITCH_DEG): motors off
//   if line:  v = back_to_field()                        (highest priority)
//   else:
//     ball held / corner shot: shoot_control() -> push forward, aim at the front goal, kick
//     else: ball = merge_ball_data(camera ball, IR ball); v = chase_ball(ball), heading 90
//     every US_PERIOD_MS: us = US_Sensor()               (reserved R03)
//     v = Vector_Field(v, field, us)                     (whole field + our penalty area)
//   Vector_Motion(v)
//
// OLED is drawn only before START (a redraw takes ~25 ms).

static void drawWaiting() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ATTACK  press START");
  display.printf("Head %.1f\n", current_heading());
  display.printf("Cam  %s age %lu\n", camera_fresh(CAM_TIMEOUT_MS) ? "ok" : "--",
                 fieldData.t_ms ? millis() - fieldData.t_ms : 0);
  display.printf("Ball cam %s ir %s\n", fieldData.ball.seen ? "Y" : "-", irBall.found ? "Y" : "-");
  display.printf("Goal F %s B %s\n", fieldData.front_goal.seen ? "Y" : "-", fieldData.back_goal.seen ? "Y" : "-");
  display.printf("Line %d  Hold %d\n", line_flag(), has_ball());
  display.printf("Kick %s\n", kicker_state_name());
  display.display();
}

void setup() {
  core_act_init();
  camera_begin();
  kicker_begin();
  strategy_begin();

  if (CrashReport) { Serial.print(CrashReport); drawMessage("CRASHED"); delay(2000); }

  MotorStop();
  uint32_t lastDraw = 0;
  while (digitalRead(START_BUTTON) == HIGH) {
    update_gyro_sensor();
    read_camera();
    read_sense();
    kicker_update();
    if (millis() - lastDraw >= 100) { lastDraw = millis(); drawWaiting(); }
  }
  drawMessage("GO");
  delay(200);
}

void loop() {
  static UsData us;
  static uint32_t lastUs = 0;

  update_gyro_sensor();
  read_camera();
  uint8_t got = read_sense();
  kicker_update();

  if (picked_up()) {                 // lifted: stop, start again from heading 90
    set_aim(false);
    set_target_heading(90);
    MotorStop();
    return;
  }

  Vec2 v;
  LineCtl line = back_to_field(got & SENSE_GOT_LINE);
  set_ramp(!line.active);            // line escape: full speed at once, no ramp
  if (line.active) {
    v = line.v;
    set_aim(false);
    set_target_heading(90);
  } else {
    if (millis() - lastUs >= US_PERIOD_MS) { lastUs = millis(); us = US_Sensor(); }

    ShootCtl shot = shoot_control(fieldData);
    if (shot.active) {
      v = shot.v;
      set_aim(shot.aiming);
      set_target_heading(shot.heading);
    } else {
      set_aim(false);
      set_target_heading(90);
      v = chase_ball(merge_ball_data(fieldData, irBall), fieldData);
    }
    v = Vector_Field(v, fieldData, us);
  }
  Vector_Motion(v.x, v.y, 0);
}
