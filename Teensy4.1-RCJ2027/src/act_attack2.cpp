#include <Arduino.h>
#include <core_act.h>

// T11b (Act): attack robot -- act_attack + IR orbit + ultrasonic (act_attack.cpp is not changed)
// Hardware: mux 1 ring only (Sense USE_MUX2 0), IR, camera, 4 ultrasonic, kicker.
//
//   update_gyro / read camera / read sense / kicker_update
//   picked up -> motors off
//   line on   -> back_to_field()                          (highest priority, as act_attack)
//   else:
//     ball held / corner shot: shoot_control()            (as act_attack)
//     else chase the IR ball (attackChase, basic):
//       no ball             -> library chase_ball() (back home toward our goal)
//       ball 90 +- PUSH_DEG -> straight forward at PUSH_SPEED_ATK
//       ball far (IR strength <= IR_ORBIT_STR) -> straight at the ball, CHASE_SPEED_ATK
//       ball near, not in front -> around it: ball angle + offset (ORBIT_K x how far off the
//                                  front, at most ORBIT_MAX_DEG), ORBIT_SPEED_ATK
//       (IR distance is not settled yet: only one strength threshold is used)
//     Vector_Field(v, camera, ultrasonic)                  (walls, goals, our penalty area)
//     ultrasonic failed -> speed toward that side x US_FAIL_K
//   Vector_Motion(v)
//
// Ultrasonic (camera first, the library Vector_Field does the walls):
//   each sensor has a switch (US_USE_*); failed = switched off / reading 0 / no packet US_STALE_MS
//   the sensor the ball is in front of (ball angle within US_BALL_SKIP_DEG, or ball held for the
//   front one) is not used: it may be reading the ball, the robot must still reach / push it
//
// Serial every 200 ms: mode | ball | goals | line hex | us | v
// OLED is drawn only before START (a redraw takes ~25 ms).

// ----- Chase (basic) -----
#define CHASE_SPEED_ATK  50.0f // far ball: straight at it
#define ORBIT_SPEED_ATK  40.0f // near ball, not in front: around it
#define PUSH_SPEED_ATK   60.0f // ball in front: forward
#ifndef ORBIT_K
#define ORBIT_K          1.0f  // offset from the ball angle = ORBIT_K x how far the ball is off the front ...
#endif
#ifndef ORBIT_MAX_DEG
#define ORBIT_MAX_DEG    60.0f // ... at most this
#endif

// ----- Ultrasonic (R03) -----
#define US_USE_FRONT     1     // H1, 90   (0 = broken / not fitted: never used, counts as failed)
#define US_USE_RIGHT     1     // H2, 0
#define US_USE_BACK      1     // H3, 270
#define US_USE_LEFT      1     // H4, 180
#define US_STALE_MS      200   // no AB packet for this long = every sensor failed
#define US_FAIL_K        0.7f  // failed sensor: speed toward its side x this
#define US_BALL_SKIP_DEG 30.0f // ball within this of a sensor: skip that sensor

static const bool usUse[4] = {US_USE_FRONT, US_USE_RIGHT, US_USE_BACK, US_USE_LEFT};
static const float usDir[4] = {90.0f, 0.0f, 270.0f, 180.0f};    // front, right, back, left

enum AtkMode : uint8_t { M_LINE, M_SHOT, M_NONE, M_PUSH, M_CHASE, M_ORBIT };
static const char* modeName[] = {"LINE", "SHOT", "NOBALL", "PUSH", "CHASE", "ORBIT"};
static AtkMode mode = M_NONE;

static Vec2 polarV(float speed, float deg) {
  Vec2 v;
  v.x = speed * cosf(deg * DEG_TO_RAD);
  v.y = speed * sinf(deg * DEG_TO_RAD);
  return v;
}

// cm, or 0 = failed (switched off, no reading, or no packet for US_STALE_MS)
static float usCm(int i) {
  if (!usUse[i] || senseUs.t_ms == 0 || millis() - senseUs.t_ms > US_STALE_MS) return 0;
  return senseUs.cm[i];
}

static bool ballNear(const MergedBall& ball, int i) {
  if (i == US_N_FRONT && has_ball()) return true;
  return ball.src != SRC_NONE && fabsf(wrap180(ball.angle - usDir[i])) < US_BALL_SKIP_DEG;
}

// Ultrasonic for the library Vector_Field: 0 = do not use (failed, or the ball is there)
static UsData usForField(const MergedBall& ball) {
  float d[4];
  for (int i = 0; i < 4; i++) d[i] = ballNear(ball, i) ? 0 : usCm(i);
  UsData us;
  us.front = d[US_N_FRONT];
  us.right = d[US_N_RIGHT];
  us.back  = d[US_N_BACK];
  us.left  = d[US_N_LEFT];
  return us;
}

// Failed sensor (not the ball one): the part of v toward its side x US_FAIL_K
static void usFailSlow(Vec2& v, const MergedBall& ball) {
  for (int i = 0; i < 4; i++) {
    if (usCm(i) > 0 || ballNear(ball, i)) continue;
    float ux = cosf(usDir[i] * DEG_TO_RAD), uy = sinf(usDir[i] * DEG_TO_RAD);
    float into = v.x * ux + v.y * uy;
    if (into <= 0) continue;
    float cut = into * (1.0f - US_FAIL_K);
    v.x -= cut * ux;
    v.y -= cut * uy;
  }
}

// IR only: IR angle + IR strength
static MergedBall ir_ball_data(const IrBallData& ir) {
  MergedBall m;
  if (!ir_ball_fresh(IR_TIMEOUT_MS)) return m;          // no fresh IR ball -> SRC_NONE
  m.src = SRC_IR;
  m.angle = ir.angle;
  m.strength = ir.strength;
  m.closeness = constrain((ir.strength - IR_FAR_STR) / (IR_NEAR_STR - IR_FAR_STR), 0.0f, 1.0f);
  return m;
}

// Basic chase: push / straight / around (see the top)
static Vec2 attackChase(const MergedBall& ball) {
  if (ball.src == SRC_NONE) {
    mode = M_NONE;
    return chase_ball(ball, fieldData);                  // library: home toward our goal
  }
  float a = fmodf(ball.angle + 360.0f, 360.0f);
  float off = wrap180(a - 90.0f);                        // > 0: ball left of the front
  if (fabsf(off) <= PUSH_DEG) {
    mode = M_PUSH;
    return polarV(PUSH_SPEED_ATK, 90.0f);
  }
  if (ball.strength <= IR_ORBIT_STR) {
    mode = M_CHASE;
    return polarV(CHASE_SPEED_ATK, a);
  }
  mode = M_ORBIT;                                        // go around, away from the front side
  float add = min(fabsf(off) * (float)ORBIT_K, (float)ORBIT_MAX_DEG);
  return polarV(ORBIT_SPEED_ATK, a + (off > 0 ? add : -add));
}

static void drawWaiting() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("ATTACK2 press START");
  display.printf("Head %.1f\n", current_heading());
  display.printf("Cam  %s age %lu\n", camera_fresh(CAM_TIMEOUT_MS) ? "ok" : "--",
                 fieldData.t_ms ? millis() - fieldData.t_ms : 0);
  display.printf("Ball ir %s %.0f s%u\n", irBall.found ? "Y" : "-", irBall.angle, irBall.strength);
  display.printf("Goal F %s B %s\n", fieldData.front_goal.seen ? "Y" : "-", fieldData.back_goal.seen ? "Y" : "-");
  display.printf("Line %d  Hold %d\n", line_flag(), has_ball());
  display.print("US");                                  // F R B L cm, -- = failed
  for (int i = 0; i < 4; i++) {
    float d = usCm(i);
    if (d > 0) display.printf(" %.0f", d);
    else       display.print(" --");
  }
  display.println();
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
  MergedBall ball = ir_ball_data(irBall);
  LineCtl line = back_to_field(got & SENSE_GOT_LINE);
  set_ramp(!line.active);            // line escape: full speed at once, no ramp
  if (line.active) {
    mode = M_LINE;
    v = line.v;
    set_aim(false);
    set_target_heading(90);
  } else {
    ShootCtl shot = shoot_control(fieldData);
    if (shot.active) {
      mode = M_SHOT;
      v = shot.v;
      set_aim(shot.aiming);
      set_target_heading(shot.heading);
    } else {
      set_aim(false);
      set_target_heading(90);
      v = attackChase(ball);
    }
    v = Vector_Field(v, fieldData, usForField(ball));
    usFailSlow(v, ball);
  }
  Vector_Motion(v.x, v.y, 0);

  EVERY_MS(tp, 200) {
    Serial.printf("%-6s | ball %-4s %5.1f s%2.0f | goal F %s %5.1f %4u B %s %5.1f %4u | line %08lX | "
                  "us F %3.0f R %3.0f B %3.0f L %3.0f | hold %d | v (%4.0f, %4.0f)\n",
                  modeName[mode], ball_src_name(ball.src), ball.angle, ball.strength,
                  fieldData.front_goal.seen ? "Y" : "-", fieldData.front_goal.angle, fieldData.front_goal.dist,
                  fieldData.back_goal.seen ? "Y" : "-", fieldData.back_goal.angle, fieldData.back_goal.dist,
                  lineData.state, usCm(US_N_FRONT), usCm(US_N_RIGHT), usCm(US_N_BACK), usCm(US_N_LEFT),
                  has_ball(), v.x, v.y);
  }
}
