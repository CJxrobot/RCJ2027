#include <Arduino.h>
#include <core_act.h>

// T11b (Act): attack robot -- act_attack + IR orbit + line slow-down
// (act_attack.cpp is not changed)
// Hardware: mux 1 ring only (Sense USE_MUX2 0), IR, camera, 4 ultrasonic, kicker.
//
//   update_gyro / read camera / read sense / kicker_update
//   picked up -> motors off
//   line on   -> back_to_field() escape (away from the line) + the chase move WITHOUT its part
//                toward the line: only the speed perpendicular to the line and toward it is
//                blocked, moving along the line goes on. Line direction unknown yet: stop.
//   else:
//     ball held / corner shot: library shoot_control(), same as the defense LEAVE:
//       push forward while turning to the front goal, kick after KICK_HOLD_MS (the kick
//       happens during the turn); corner: back up, aim, kick
//     else chase the IR ball (attackChase, basic):
//       no ball             -> library chase_ball() (back home toward our goal)
//       ball 90 +- PUSH_DEG -> straight forward at PUSH_SPEED_ATK
//       ball far (IR strength <= IR_ORBIT_STR) -> straight at the ball, CHASE_SPEED_ATK
//       ball near, not in front -> around it: ball angle + offset (ORBIT_K x how far off the
//                                  front, at most ORBIT_MAX_DEG), ORBIT_SPEED_ATK
//       (IR distance is not settled yet: only one strength threshold is used)
//     line slow-down, every speed toward a white line falls exponentially to 0 before it:
//       A. end lines beside the goals (camera goal ANGLE): the closer a goal is to 0 / 180
//          (field frame), the closer the robot is to that end. The angle AT the line depends on
//          x (13 deg at x 70, 23 deg at x 40), so it is turned into cm with x (from B):
//          centre to the end line = |x| tan(angle) - GOAL_POINT_IN_CM -> exponential, 0 at
//          END_STOP_CM. In front of a goal (|x| < GOAL_HALF_CM) not used (shots go there).
//       B. side lines (camera goal ANGLES): x from both goal angles (one goal: angle + distance)
//          -> speed toward that side line, exponential, 0 at SIDE_STOP_CM (robot centre to line)
//       C. ultrasonic, last line: distance to the LINE (wall - US_WALL_TO_LINE_CM), minus
//          last loop's speed x US_LAG_S (the reading is ~0.3 s old), exponential, 0 at US_LINE_STOP_CM
//          failed sensor -> speed toward that side x US_FAIL_K
//     around our penalty area (camera, back goal): the part of the move toward the area is
//       turned sideways (tangent, toward the ball's side) instead of cut, more the closer:
//       exponential, all of it at AREA_STOP_CM (robot centre outside the penalty line)
//     Vector_Field(v, camera)                              (goals, our penalty area; walls = C)
//   Vector_Motion(v)
//
// Ultrasonic: each sensor has a switch (US_USE_*); failed = switched off / reading 0 / no packet
//   US_STALE_MS. The FRONT one is skipped while the ball is in front of it (or held): it may be
//   reading the ball, the robot must still reach / push it.
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
#define US_BALL_SKIP_DEG 30.0f // ball within this of the front sensor (or held): skip it
#define US_WALL_TO_LINE_CM 13.0f // wall to the white line centre (91 - 78)
#define US_LINE_STOP_CM  3.0f  // sensor (robot edge) this far before the line: no speed toward it
#define US_LINE_LAMBDA_CM 8.0f
#define US_LAG_S         0.30f // ultrasonic reading age (Sense: 1 sensor / 30 ms + filter)
#define CM_S_PER_PCT     3.0f  // 100 % = 300 cm/s

// ----- Line slow-down (camera first) -----
#define END_STOP_CM      14.0f // A. robot centre to the end line: no speed toward it (edge 3 cm inside)
#define END_LAMBDA_CM    20.0f
#define GOAL_POINT_IN_CM 6.0f  //    camera goal point is this far behind the goal line
#define GOAL_HALF_CM     30.0f //    goal 60 cm wide
#define SIDE_STOP_CM     14.0f // B. robot centre to the side line: no speed toward it (edge 3 cm inside)
#define SIDE_LAMBDA_CM   20.0f
#define FIELD_HALF_W_CM  78.0f // centre to the side line
#define FIELD_HALF_LEN_CM 114.5f // centre to the camera goal point (5 cm inside the goal)

// ----- Around our penalty area (back goal camera, same geometry as the defense) -----
#define AREA_STOP_CM     8.0f  // robot centre this far outside the penalty line: nothing toward it
#define AREA_LAMBDA_CM   15.0f // turned sideways: exp(-(dist - stop) / lambda)
#define PA_FRONT_CM      30.0f // camera goal point to the front line
#define PA_HALF_CM       39.0f // side lines
#define PA_R_CM          15.0f // rounded front corners
#define GOAL_K_DEG1      44.0f // camera distance slant correction (defense D1)
#define GOAL_K1          0.81f
#define GOAL_K_DEG2      66.0f
#define GOAL_K2          0.61f

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

static bool ballNear(const MergedBall& ball, int i) {   // ball in front of sensor i (or held: front)
  if (i == US_N_FRONT && has_ball()) return true;
  return ball.src != SRC_NONE && fabsf(wrap180(ball.angle - usDir[i])) < US_BALL_SKIP_DEG;
}

static bool goalOk(const FieldObj& g) { return camera_fresh(CAM_TIMEOUT_MS) && g.seen; }

// Cut the part of v going toward direction dir (robot frame, deg): x k (k = 1: all of it)
static void cutToward(Vec2& v, float dir, float k) {
  float ux = cosf(dir * DEG_TO_RAD), uy = sinf(dir * DEG_TO_RAD);
  float into = v.x * ux + v.y * uy;
  if (into <= 0) return;                                        // away / along: keep
  k = constrain(k, 0.0f, 1.0f);
  v.x -= k * into * ux;
  v.y -= k * into * uy;
}

// B. Robot x (cm, + = right) from the goal angles: both goals -> triangulation (angles only),
//    one goal -> its angle + distance. false = no goal.
static bool fieldX(float& x) {
  float turn = current_heading() - 90.0f;
  const FieldObj& f = fieldData.front_goal;
  const FieldObj& b = fieldData.back_goal;
  bool fo = goalOk(f) && f.dist != PKT_NONE, bo = goalOk(b) && b.dist != PKT_NONE;
  if (goalOk(f) && goalOk(b)) {
    float fa = (f.angle + turn) * DEG_TO_RAD, ba = (b.angle + turn) * DEG_TO_RAD;
    float det = sinf(fa - ba);                                  // 0 when the goals are 180 apart
    if (fabsf(det) < 0.05f) { x = 0; return true; }             // on the centre line
    x = -2.0f * FIELD_HALF_LEN_CM * cosf(ba) * cosf(fa) / det;
    return true;
  }
  if (fo) { x = -f.dist * cosf((f.angle + turn) * DEG_TO_RAD); return true; }
  if (bo) { x = -b.dist * cosf((b.angle + turn) * DEG_TO_RAD); return true; }
  return false;
}

// A. End lines from the goal angle (+ x from B), only beside the goals
static void slowToEnds(Vec2& v) {
  float x;
  if (!fieldX(x) || fabsf(x) < GOAL_HALF_CM) return;
  float turn = current_heading() - 90.0f;                       // robot frame -> field frame
  const FieldObj* g[2] = {&fieldData.front_goal, &fieldData.back_goal};
  for (int i = 0; i < 2; i++) {
    if (!goalOk(*g[i])) continue;
    float a = g[i]->angle + turn;
    float off = min(fabsf(wrap180(a)), fabsf(wrap180(a - 180.0f)));   // 0 = level with it
    float d = fabsf(x) * tanf(min(off, 80.0f) * DEG_TO_RAD) - GOAL_POINT_IN_CM;   // centre to end line
    cutToward(v, (i == 0 ? 90.0f : 270.0f) - turn, expf(-(d - END_STOP_CM) / END_LAMBDA_CM));
  }
}

// Camera goal distance slant correction (same as the defense, measured on the field)
static float goalDistK(float fieldAngle) {
  float off = fabsf(wrap180(fieldAngle - 270.0f));
  if (off <= GOAL_K_DEG1) return 1.0f + (GOAL_K1 - 1.0f) * off / GOAL_K_DEG1;
  if (off <= GOAL_K_DEG2) return GOAL_K1 + (GOAL_K2 - GOAL_K1) * (off - GOAL_K_DEG1) / (GOAL_K_DEG2 - GOAL_K_DEG1);
  return GOAL_K2;
}

// Around our penalty area: the part of v toward the area turns into a sideways (tangent) move,
// toward the ball's side (no ball: the side v already goes, else the robot's own side)
static void aroundOurArea(Vec2& v, const MergedBall& ball) {
  const FieldObj& b = fieldData.back_goal;
  if (!goalOk(b) || b.dist == PKT_NONE) return;
  float turn = current_heading() - 90.0f;
  float fa = b.angle + turn;                                    // field frame
  float dist = b.dist * goalDistK(fa);
  float x = -dist * cosf(fa * DEG_TO_RAD), y = -dist * sinf(fa * DEG_TO_RAD);   // robot from goal point
  float qx = fabsf(x), cx = PA_HALF_CM - PA_R_CM, cy = PA_FRONT_CM - PA_R_CM;
  float ad = (qx > cx && y > cy) ? hypotf(qx - cx, y - cy) - PA_R_CM : max(qx - PA_HALF_CM, y - PA_FRONT_CM);
  float ux = cosf(b.angle * DEG_TO_RAD), uy = sinf(b.angle * DEG_TO_RAD);   // toward the area, robot frame
  float into = v.x * ux + v.y * uy;
  if (into <= 0) return;
  float k = min(1.0f, expf(-(ad - AREA_STOP_CM) / AREA_LAMBDA_CM));
  float tx = -uy, ty = ux;                                      // one tangent; the other is -t
  float pick;
  if (ball.src != SRC_NONE) pick = cosf(ball.angle * DEG_TO_RAD) * tx + sinf(ball.angle * DEG_TO_RAD) * ty;
  else pick = v.x * tx + v.y * ty;
  if (fabsf(pick) < 0.05f) pick = x * (tx * cosf(-turn * DEG_TO_RAD) + ty * sinf(-turn * DEG_TO_RAD));
  if (pick < 0) { tx = -tx; ty = -ty; }
  v.x += k * into * (tx - ux);
  v.y += k * into * (ty - uy);
}

static void slowToSides(Vec2& v) {
  float x;
  if (!fieldX(x)) return;
  float turn = current_heading() - 90.0f;
  for (int s = -1; s <= 1; s += 2) {                            // -1 left line, +1 right line
    float d = FIELD_HALF_W_CM - s * x;                          // centre to that side line
    cutToward(v, (s > 0 ? 0.0f : 180.0f) - turn, expf(-(d - SIDE_STOP_CM) / SIDE_LAMBDA_CM));
  }
}

// C. Ultrasonic, last line: distance to the white line, lag-compensated, exponential.
//    Failed sensor: x US_FAIL_K. Front sensor skipped while the ball is in front of it / held.
//    The lag uses the speed actually sent last loop (standing still: no correction).
static void usLastLine(Vec2& v, const MergedBall& ball, const Vec2& last) {
  for (int i = 0; i < 4; i++) {
    float ux = cosf(usDir[i] * DEG_TO_RAD), uy = sinf(usDir[i] * DEG_TO_RAD);
    float into = v.x * ux + v.y * uy;
    if (into <= 0) continue;
    if (i == US_N_FRONT && ballNear(ball, i)) continue;
    float d = usCm(i);
    if (d <= 0) { cutToward(v, usDir[i], 1.0f - US_FAIL_K); continue; }
    float moving = max(0.0f, last.x * ux + last.y * uy);      // % toward it last loop
    float line = d - US_WALL_TO_LINE_CM - moving * CM_S_PER_PCT * US_LAG_S;   // where it is now
    cutToward(v, usDir[i], expf(-(line - US_LINE_STOP_CM) / US_LINE_LAMBDA_CM));
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

  static Vec2 lastV;                 // speed sent last loop (ultrasonic lag)
  Vec2 v;
  MergedBall ball = ir_ball_data(irBall);
  LineCtl line = back_to_field(got & SENSE_GOT_LINE);
  set_ramp(!line.active);            // line escape: full speed at once, no ramp
  if (line.active) {
    mode = M_LINE;
    set_aim(false);
    set_target_heading(90);
    if (line.line_dir >= 0) {        // keep the move along the line, block only toward it
      v = attackChase(ball);
      mode = M_LINE;
      cutToward(v, line.line_dir, 1.0f);
      v.x += line.v.x;
      v.y += line.v.y;
    }                                // direction not known yet: stop
  } else {
    ShootCtl shot = shoot_control(fieldData);              // as the defense LEAVE
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
  }
  aroundOurArea(v, ball);                                // our penalty area: go around it
  v = Vector_Field(v, fieldData, UsData());              // goals + our penalty area (camera)
  slowToEnds(v);                                         // A
  slowToSides(v);                                        // B
  usLastLine(v, ball, lastV);                            // C
  Vector_Motion(v.x, v.y, 0);
  lastV = v;

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
