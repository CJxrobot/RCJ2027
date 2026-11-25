#include <Arduino.h>
#include <core_act.h>

// Motor direction of this robot body: 1 = attack body (wired the other way), 0 = defense body
#define MOTOR_REVERSE 1

// T11 (Act): attack game program, same order as the system design
//
//   update_gyro / read camera / read sense / kicker_update
//   if picked up (|pitch| > PICKUP_PITCH_DEG): motors off
//   if line:  v = back_to_field()                        (highest priority)
//   else:
//     ball held / corner shot: shoot_control() -> push forward, aim at the front goal, kick
//     else: final_ball_data (IR, or merge_ball_data for camera + IR); v = chase_ball(), heading 90
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

// ===== Ball data: final_ball_data is built here (this program decides, not the library) =====
// IR only: IR angle + IR distance (strength)
__attribute__((unused)) static MergedBall ir_ball_data(const IrBallData& ir) {
  MergedBall m;
  if (!ir_ball_fresh(IR_TIMEOUT_MS)) return m;          // no fresh IR ball -> SRC_NONE
  m.src = SRC_IR;
  m.angle = ir.angle;
  m.strength = ir.strength;
  m.closeness = constrain((ir.strength - IR_FAR_STR) / (IR_NEAR_STR - IR_FAR_STR), 0.0f, 1.0f);
  return m;
}

// Camera + IR:
//   both seen, angles within MERGE_MAX_ERR_DEG -> FUSED: camera angle + IR distance
//   both seen, farther apart                   -> IR
//   only IR -> IR,  only camera -> CAM (camera pixel distance),  none -> NONE
__attribute__((unused)) static MergedBall merge_ball_data(const FieldData& field, const IrBallData& ir) {
  MergedBall m = ir_ball_data(ir);                      // IR part (strength, closeness)
  bool cam = field.t_ms != 0 && millis() - field.t_ms <= CAM_TIMEOUT_MS && field.ball.seen;
  if (!cam) return m;
  m.dist = field.ball.dist;
  if (m.src == SRC_IR) {
    if (fabsf(wrap180(field.ball.angle - ir.angle)) < MERGE_MAX_ERR_DEG) {
      m.src = SRC_FUSED;                                // agree: camera angle, IR distance
      m.angle = field.ball.angle;
    }                                                   // disagree: keep IR
  } else {
    m.src = SRC_CAM;
    m.angle = field.ball.angle;
  }
  return m;
}

void setup() {
  set_motor_reverse(MOTOR_REVERSE);   // before anything moves a motor
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
      MergedBall final_ball_data = ir_ball_data(irBall);
      //MergedBall final_ball_data = merge_ball_data(fieldData, irBall);
      v = chase_ball(final_ball_data, fieldData);
    }
    v = Vector_Field(v, fieldData, us);
  }
  Vector_Motion(v.x, v.y, 0);
}
