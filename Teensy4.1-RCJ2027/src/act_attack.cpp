#include <Arduino.h>
#include <core_act.h>

// T11 (Act): attack game program, same order as the system design
//
//   update_gyro / read camera / read sense / kicker_update
//   if line:  v = back_to_field()                        (highest priority)
//   else:
//     if ball held: kick; on a kick turn toward the front goal for KICK_ROTATE_MS, else hold 90
//     every US_PERIOD_MS: us = US_Sensor()               (reserved R03)
//     ball = merge_ball_data(camera ball, IR ball)
//     v  = chase_ball(ball)
//     v  = Vector_Field(v, field, us)                    (keep out of our own penalty area)
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
  camera_v2_begin();
  kicker_begin();
  strategy_begin();

  if (CrashReport) { Serial.print(CrashReport); drawMessage("CRASHED"); delay(2000); }

  MotorStop();
  uint32_t lastDraw = 0;
  while (digitalRead(START_BUTTON) == HIGH) {
    update_gyro_sensor();
    read_camera_v2();
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
  read_camera_v2();
  uint8_t got = read_sense();
  kicker_update();

  Vec2 v;
  LineCtl line = back_to_field(got & SENSE_GOT_LINE);
  if (line.active) {
    v = line.v;
    set_target_heading(90);
  } else {
    bool kicked = has_ball() && kicker_fire();
    set_target_heading(kick_rotate(kicked, fieldData));

    if (millis() - lastUs >= US_PERIOD_MS) { lastUs = millis(); us = US_Sensor(); }

    MergedBall ball = merge_ball_data(fieldData, irBall);
    v = chase_ball(ball, fieldData);
    v = Vector_Field(v, fieldData, us);
  }
  Vector_Motion(v.x, v.y, 0);
}
