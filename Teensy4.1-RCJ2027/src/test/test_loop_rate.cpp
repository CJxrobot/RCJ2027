#include <Arduino.h>
#include <core_act.h>

// T06 (Act): main loop speed with every input read and the strategy math running.
// Sense runs test_sense_tx, Cam runs main.py (or camera_test/cam_fake_v2.py). Motors never move. OLED not drawn.
// Pass: >= 1000 loops/s, worst loop < 2000 us

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
  core_act_init();
  camera_begin();
  kicker_begin();
  strategy_begin();
  MotorStop();
  drawMessage("T06 rate");
}

void loop() {
  static LoopMeter lm;
  lm.begin();

  update_gyro_sensor();
  read_camera();
  uint8_t got = read_sense();
  kicker_update();

  // Same math as the game, results not used
  LineCtl line = back_to_field(got & SENSE_GOT_LINE);
  MergedBall final_ball_data = ir_ball_data(irBall);
  //MergedBall final_ball_data = merge_ball_data(fieldData, irBall);
  MergedBall& ball = final_ball_data;
  Vec2 v = chase_ball(ball, fieldData);
  Vec2 f = Vector_Field(v, fieldData, US_Sensor());
  volatile float sink = v.x + f.x + shoot_control(fieldData).heading + line.v.x;
  (void)sink;

  lm.end();
  if (lm.report()) {
    Serial.printf("loops/s %6lu | worst %4lu us | gyro %.1f | cam age %lu ms | ball %s\n",
                  lm.loops, lm.worst, gyroData.heading,
                  fieldData.t_ms ? millis() - fieldData.t_ms : 0, ball_src_name(ball.src));
    lm.reset();
  }
}
