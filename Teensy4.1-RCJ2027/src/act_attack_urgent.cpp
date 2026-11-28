#include <Arduino.h>
#include <core_act.h>

// Motor direction of this robot body: 1 = attack body (wired the other way), 0 = defense body
#define MOTOR_REVERSE 1

// T11u (Act): URGENT attack (2026-11-28) = act_attack5 with (act_attack5 is not changed):
//   ring (mux 1) and kicker damaged -> not used at all.
//
// White line: only these 8 mux 2 sensors (everything else in lineData.state is masked off)
//     side     inner          outer
//     front    F_in  (ch 7)   F_out (ch 11)
//     left     L_in  (ch 0)   L_mid (ch 1)
//     right    R_in  (ch 13)  R_mid (ch 14)
//     back     B_in  (ch 3)   B_mid (ch 4)
//   outer on a line            -> brake: no move toward that side
//   inner on a line            -> escape: back the other way (LINE_ESCAPE3, front LINE_ESCAPE_FRONT5)
//                                 until the outer sensor is on the line and the inner is off
//                                 (robot back at the edge), or it clearly moved back (that side's
//                                 ultrasonic ESC_BACK_CM farther than at the touch, or for front /
//                                 back the camera goal > ESC_GOAL_CM). No timer.
//   centre inside their area (going straight out, camera): no line handling (area lines).
// Ultrasonic slow-down stronger than attack5: stop US_LINE_STOP3 8 (3), range US_LAMBDA3 15 (10),
//   failed sensor x US_FAIL3 0.5 (0.7). Front sensor always used, calm in front of their goal (attack5).
// Kicker: kicker_begin / kicker_update never called, charge + discharge pins held LOW.
//   Ball held (USE_HOLD 1): push forward, aiming at the front goal (AIM_PUSH_SPEED, rotate()),
//   goal not seen: straight NOGOAL_PUSH_SPEED, heading 90. No corner shot (it waits for a kick).
//   USE_HOLD 0: the ball sensor (pin 38) is not read, the robot just chases / pushes.
// Sense: sense_attack (mux 2 on). The Sense line pins (PIN_0 / PIN_1) are detached.
// Chase / orbit, goal slow-down, their area stop / out, own-goal risk, link print, SW2: attack5.

#define DATA_BUTTON    26     // SW2 (netlist U1.18 = pin 26), pressed = LOW: stop + show the data
#define PUSH_DEG3      5.0f   // ball this close to 90: push (library PUSH_DEG 10 is kept for defense)
#define PUSH_SPEED3    60.0f  // ball in front: forward ...
#define PUSH_SLOW3     40.0f  // ... but this for the first PUSH_SLOW_MS
#define PUSH_SLOW_MS   300
#define ORBIT_SLOW3    25.0f  // orbit speed with the ball near the front ...
#define ORBIT_SLOW_DEG 60.0f  // ... up to ORBIT_SPEED3 from this far off the front
#define LINE_ESCAPE3   40.0f  // inner sensor on a line: back the other way at this ...
#define LINE_ESCAPE_FRONT5 60.0f // ... front (F_in): harder (attack5)
#define FACE_GOAL_DEG5     30.0f // facing their goal: camera front goal within 90 +- this (attack5)
#define GOAL_CALM_CM5      30.0f // ... front ultrasonic under this: forward at most GOAL_CALM_SPEED5
#define GOAL_CALM_SPEED5   30.0f
#define ESC_BACK_CM    15.0f  // escape backup: that side's ultrasonic this much farther than at the touch = back
#define ESC_GOAL_CM    70.0f  // escape backup (front / back): that goal this far (camera) = inside
#define USE_HOLD       1      // 1 = ball sensor (pin 38) used (push the ball in), 0 = not read

// The 8 line sensors (mux 2 ch c = bit 16 + c). Everything else is masked: never "line".
#define M2B(ch) (1UL << (16 + (ch)))
struct LineSide { float deg; uint32_t inner, outer; int us; };
static const LineSide LSIDE[4] = {
  {0.0f,   M2B(13), M2B(14), US_N_RIGHT},   // right: R_in / R_mid
  {90.0f,  M2B(7),  M2B(11), US_N_FRONT},   // front: F_in / F_out
  {180.0f, M2B(0),  M2B(1),  US_N_LEFT},    // left:  L_in / L_mid
  {270.0f, M2B(3),  M2B(4),  US_N_BACK},    // back:  B_in / B_mid
};
#define LINE8 (M2B(13) | M2B(14) | M2B(7) | M2B(11) | M2B(0) | M2B(1) | M2B(3) | M2B(4))
#define LINE_MASK_OFF (~(uint32_t)LINE8)    // OR into lineData.state: only the 8 sensors left
static uint8_t escMask = 0;            // bit i = side i escaping (inner touched)
static uint8_t brakeMask = 0;          // bit i = side i braking (outer on the line)
#define CHASE_SPEED3   50.0f  // far ball: straight at it
#define ORBIT_SPEED3   40.0f  // near ball: around it
#define STR_ORBIT      5      // IR strength from here: orbit, below: straight at the ball
#define STR_FULL       7.0f   // IR strength from here: full offset
#define STR_LAMBDA     2.9f   // g = 0.5 at strength 5
#define ORBIT_K        1.5f   // offset = how far off the front x this ...
#define ORBIT_MAX_DEG  90.0f  // ... at most this
#define ORBIT_MIN_DEG  35.0f  // near ball, front half: at least this (never straight at it)
#define ORBIT_MIN_BACK_DEG 60.0f // near ball, back half (> 90 off the front): at least this
#define WHEEL_ZONE_DEG 20.0f  // wheel shadow: compensation from the wheel angle out to this
static const float WHEEL_BONUS[4] = {3.0f, 3.0f, 2.0f, 2.0f};   // 45, 135, 225, 315 (estimate; 45 / 135 +1 2026-11-26)
#define BACK_ZONE_DEG  30.0f  // behind: compensation from 270 out to this
#define BACK_BONUS     2.0f   // strength lost at 270 (estimate, measure)

// ----- Goals (camera) -----
#define GOAL_NEAR_CM   25.0f  // camera goal distance where the factor reaches its minimum
#define FRONT_MIN_K    0.5f   // attacking straight: slower near the opponent goal
#define FRONT_LAMBDA   30.0f  //   (85 cm -> 0.93, 55 -> 0.82, 40 -> 0.70, 25 -> 0.5)
#define THEIR_STOP_CM  2.0f   // robot centre this far outside their area line: no move toward the goal
#define THEIR_LAMBDA   12.0f  //   exponential: 14 cm before the stop -> 31 % left
#define THEIR_OUT_CM   -3.0f  // centre this far inside their area line (camera) -> straight out ...
#define THEIR_OUT_SPEED 40.0f //   ... at this speed (fully inside at about -10)
#define PA_FRONT_CM3   30.0f  // area shape from the camera goal point (as the defense)
#define PA_HALF_CM3    39.0f
#define PA_R_CM3       15.0f
#define OWN_MIN_K      0.3f   // own-goal risk: much slower near our goal
#define OWN_LAMBDA     30.0f  //   (85 cm -> 0.91, 55 -> 0.68, 40 -> 0.55, 25 -> 0.3)
#define OWN_RISK_DEG   45.0f  // ball within this of our goal's direction = between robot and goal

// ----- Ultrasonic field -----
#define US_FIELD3      1      // 0 = no ultrasonic slow-down at all (readings still shown / printed)
#define US_USE_FRONT3  1      // H1, 90   (0 = broken: never used, counts as failed)
#define US_USE_RIGHT3  1      // H2, 0
#define US_USE_BACK3   1      // H3, 270
#define US_USE_LEFT3   1      // H4, 180
#define US_STALE3_MS   200    // no AB packet for this long = every sensor failed
#define US_FAIL3       0.5f   // failed sensor: speed toward its side x this (attack5 0.7)
#define US_WALL_TO_LINE3 13.0f // wall to the white line centre
#define US_LINE_STOP3  8.0f   // sensor (robot edge) this far before the line: no speed toward it (attack5 3)
#define US_LAMBDA3     15.0f  // exponential: 15 cm before the stop -> 37 % left ... 0 at it (attack5 10)
#define US_LAG3        0.30f  // reading age (s)
#define US_LAG_MAX3    20.0f  // the lag correction at most this (cm)
#define US_CMS_PCT3    3.0f   // 100 % = 300 cm/s

enum Mode3 : uint8_t { M3_NONE, M3_SHOT, M3_PUSH, M3_CHASE, M3_ORBIT };
static bool lineLock = false;          // a side is escaping (no ramp)
static bool lineOn = false;            // a sensor sees the line now (for the print)
static const char* modeName3[] = {"NOBALL", "SHOT", "PUSH", "CHASE", "ORBIT"};
static Mode3 mode = M3_NONE;
static float orbitG = 0;               // last orbit factor (for the print)

static Vec2 polarV(float speed, float deg) {
  Vec2 v;
  v.x = speed * cosf(deg * DEG_TO_RAD);
  v.y = speed * sinf(deg * DEG_TO_RAD);
  return v;
}

// IR strength with the wheel shadow compensated
static float strengthComp(float angle, float str) {
  float bonus = 0;
  for (int i = 0; i < 4; i++) {
    float d = fabsf(wrap180(angle - (45.0f + 90.0f * i)));
    if (d < WHEEL_ZONE_DEG) bonus = max(bonus, WHEEL_BONUS[i] * (1.0f - d / WHEEL_ZONE_DEG));
  }
  float db = fabsf(wrap180(angle - 270.0f));
  if (db < BACK_ZONE_DEG) bonus = max(bonus, BACK_BONUS * (1.0f - db / BACK_ZONE_DEG));
  return str + bonus;
}
static float strUsed = 0;              // last compensated strength (for the print)

// ----- Ultrasonic field -----
static const bool usUse3[4] = {US_USE_FRONT3, US_USE_RIGHT3, US_USE_BACK3, US_USE_LEFT3};
static const float usDir3[4] = {90.0f, 0.0f, 270.0f, 180.0f};   // front, right, back, left

// cm, or 0 = failed (switched off, no reading, or no packet for US_STALE3_MS)
static float usCm3(int i) {
  if (!usUse3[i] || senseUs.t_ms == 0 || millis() - senseUs.t_ms > US_STALE3_MS) return 0;
  return senseUs.cm[i];
}

static float toward(const Vec2& v, float dir);

// Speed factor near the goals (see the top), 1 = no change
static float goalK3 = 1.0f;            // last factor (for the print / screen)
static float theirArea3 = 999;         // robot centre outside their area line (cm, camera)
static bool theirInside3 = false;      // centre inside their area: going straight out
static float goalSlow(Vec2& v) {
  goalK3 = 1.0f;
  if (!camera_fresh(CAM_TIMEOUT_MS)) return 1.0f;
  const FieldObj& f = fieldData.front_goal;
  const FieldObj& b = fieldData.back_goal;
  if (f.seen && f.dist != PKT_NONE && toward(v, f.angle) > 0)
    goalK3 = min(goalK3, FRONT_MIN_K + (1 - FRONT_MIN_K) *
                         (1 - min(1.0f, expf(-(f.dist - GOAL_NEAR_CM) / FRONT_LAMBDA))));
  bool risk = b.seen && b.dist != PKT_NONE && ir_ball_fresh(IR_TIMEOUT_MS) &&
              fabsf(wrap180(irBall.angle - b.angle)) < OWN_RISK_DEG && toward(v, b.angle) > 0;
  if (risk)
    goalK3 = min(goalK3, OWN_MIN_K + (1 - OWN_MIN_K) *
                         (1 - min(1.0f, expf(-(b.dist - GOAL_NEAR_CM) / OWN_LAMBDA))));
  goalK3 = max(goalK3, min(FRONT_MIN_K, OWN_MIN_K));
  v.x *= goalK3;
  v.y *= goalK3;

  // Opponent penalty area: no move toward the goal once the centre reaches its line
  theirArea3 = 999;
  theirInside3 = false;
  if (f.seen && f.dist != PKT_NONE) {
    float turn = current_heading() - 90.0f;
    float fa = (f.angle + turn) * DEG_TO_RAD;                   // field frame
    float x = -f.dist * cosf(fa), y = f.dist * sinf(fa);        // robot from the goal point, y > 0 = before it
    float qx = fabsf(x), cx = PA_HALF_CM3 - PA_R_CM3, cy = PA_FRONT_CM3 - PA_R_CM3;
    theirArea3 = (qx > cx && y > cy) ? hypotf(qx - cx, y - cy) - PA_R_CM3
                                     : max(qx - PA_HALF_CM3, y - PA_FRONT_CM3);
    float in = toward(v, f.angle);
    if (in > 0) {
      float k = min(1.0f, expf(-(theirArea3 - THEIR_STOP_CM) / THEIR_LAMBDA));
      v.x -= k * in * cosf(f.angle * DEG_TO_RAD);
      v.y -= k * in * sinf(f.angle * DEG_TO_RAD);
    }
    // centre inside their area (carried over the line at speed): straight out, away from the goal
    theirInside3 = theirArea3 < THEIR_OUT_CM;
    if (theirInside3) {
      v.x = -THEIR_OUT_SPEED * cosf(f.angle * DEG_TO_RAD);
      v.y = -THEIR_OUT_SPEED * sinf(f.angle * DEG_TO_RAD);
    }
  }
  return goalK3;
}

static float usCut3[4] = {0, 0, 0, 0};   // last cut per sensor (0 = none, 1 = all), for the screen

static void usField(Vec2& v, const Vec2& last) {
  for (int i = 0; i < 4; i++) usCut3[i] = 0;
  if (!US_FIELD3) return;
  // attack5: the front sensor does not see the ball -> always used. Facing their goal: cap only.
  const FieldObj& fg = fieldData.front_goal;
  bool facingGoal = camera_fresh(CAM_TIMEOUT_MS) && fg.seen &&
                    fabsf(wrap180(fg.angle - 90.0f)) <= FACE_GOAL_DEG5;
  for (int i = 0; i < 4; i++) {
    float ux = cosf(usDir3[i] * DEG_TO_RAD), uy = sinf(usDir3[i] * DEG_TO_RAD);
    float into = v.x * ux + v.y * uy;
    if (into <= 0) continue;                                     // away / along: keep
    float d = usCm3(i);
    if (i == US_N_FRONT && facingGoal && d > 0) {                // their goal ahead: no rush
      if (d < GOAL_CALM_CM5 && into > GOAL_CALM_SPEED5) {
        usCut3[i] = 1.0f - GOAL_CALM_SPEED5 / into;
        v.x -= (into - GOAL_CALM_SPEED5) * ux;
        v.y -= (into - GOAL_CALM_SPEED5) * uy;
      }
      continue;
    }
    float k;
    if (d <= 0) k = 1.0f - US_FAIL3;
    else {
      float moving = max(0.0f, last.x * ux + last.y * uy);       // % toward it last loop
      float lag = min((float)US_LAG_MAX3, moving * US_CMS_PCT3 * US_LAG3);
      float line = d - US_WALL_TO_LINE3 - lag;
      k = min(1.0f, expf(-(line - US_LINE_STOP3) / US_LAMBDA3));
    }
    usCut3[i] = k;
    v.x -= k * into * ux;
    v.y -= k * into * uy;
  }
}

static Vec2 chase3() {
  Vec2 v;
  if (!ir_ball_fresh(IR_TIMEOUT_MS)) { mode = M3_NONE; return v; }   // no ball: stop
  static uint32_t pushStart = 0;
  float a = fmodf(irBall.angle + 360.0f, 360.0f);
  float off = wrap180(a - 90.0f);                                     // > 0: ball left of the front
  if (fabsf(off) <= PUSH_DEG3) {
    if (mode != M3_PUSH) pushStart = millis();
    mode = M3_PUSH;
    return polarV(millis() - pushStart < PUSH_SLOW_MS ? PUSH_SLOW3 : PUSH_SPEED3, 90.0f);
  }
  strUsed = strengthComp(a, irBall.strength);
  if (strUsed < STR_ORBIT) { mode = M3_CHASE; return polarV(CHASE_SPEED3, a); }
  mode = M3_ORBIT;
  orbitG = min(1.0f, expf((strUsed - STR_FULL) / STR_LAMBDA));
  float add = min((float)ORBIT_MAX_DEG, fabsf(off) * ORBIT_K) * orbitG;
  add = max(add, fabsf(off) > 90.0f ? (float)ORBIT_MIN_BACK_DEG : (float)ORBIT_MIN_DEG);
  float speed = ORBIT_SLOW3 + (ORBIT_SPEED3 - ORBIT_SLOW3) * min(1.0f, fabsf(off) / ORBIT_SLOW_DEG);
  return polarV(speed, a + (off > 0 ? add : -add));
}

// Part of v going toward dir (robot frame, deg)
static float toward(const Vec2& v, float dir) {
  return v.x * cosf(dir * DEG_TO_RAD) + v.y * sinf(dir * DEG_TO_RAD);
}

static bool seenL(uint32_t st, uint32_t m) { return (~st & m) != 0; }

// Escape backup: moved clearly back from where the inner sensor touched (that side's ultrasonic
// ESC_BACK_CM farther than at the touch; relative, so it also works on the area lines), or for
// front / back the camera goal far away
static float escTouchCm[4] = {0, 0, 0, 0};
static bool sideClear(int i) {
  float d = usCm3(LSIDE[i].us);
  if (d > 0 && escTouchCm[i] > 0 && d - escTouchCm[i] > ESC_BACK_CM) return true;
  if (i == 1 || i == 3) {
    const FieldObj& g = i == 1 ? fieldData.front_goal : fieldData.back_goal;
    if (camera_fresh(CAM_TIMEOUT_MS) && g.seen && g.dist != PKT_NONE && g.dist > ESC_GOAL_CM) return true;
  }
  return false;
}

// White line, 8 sensors (no timer, see the top)
static void lineGuard(Vec2& v) {
  uint32_t st = lineData.state;                         // masked (see loop)
  lineOn = st != 0xFFFFFFFFUL;
  brakeMask = 0;
  if (theirInside3) { escMask = 0; lineLock = false; return; }   // going out of their area
  for (int i = 0; i < 4; i++) {
    bool in = seenL(st, LSIDE[i].inner), out = seenL(st, LSIDE[i].outer);
    uint8_t bit = 1 << i;
    if (in) { escMask |= bit; escTouchCm[i] = usCm3(LSIDE[i].us); }   // inner on the line: escape
    else if ((escMask & bit) && (out || sideClear(i))) escMask &= ~bit;   // back at the edge / inside
    if (!(escMask & bit) && !out) continue;
    float ux = cosf(LSIDE[i].deg * DEG_TO_RAD), uy = sinf(LSIDE[i].deg * DEG_TO_RAD);
    float in2 = v.x * ux + v.y * uy;
    if (in2 > 0) { v.x -= in2 * ux; v.y -= in2 * uy; }  // no speed toward that side
    if (escMask & bit) {
      float esc = i == 1 ? LINE_ESCAPE_FRONT5 : LINE_ESCAPE3;
      v.x -= esc * ux; v.y -= esc * uy;                 // back the other way
    } else brakeMask |= bit;
  }
  lineLock = escMask != 0;
}

// Ball held, no kicker: push it in, aiming at the front goal (rotate(): 90 if not seen)
static bool pushShot(Vec2& v) {
  if (!USE_HOLD || !has_ball()) return false;
  bool goal = camera_fresh(CAM_TIMEOUT_MS) && fieldData.front_goal.seen;
  v = Vec2();
  v.y = goal ? AIM_PUSH_SPEED : NOGOAL_PUSH_SPEED;
  set_aim(goal);
  set_target_heading(goal ? rotate(fieldData) : 90.0f);
  return true;
}

// Kicker damaged: pins off, ball sensor input (what kicker_begin() did, without charging)
static void kickerOff() {
  pinMode(KICK_CHARGE_PIN, OUTPUT);
  digitalWriteFast(KICK_CHARGE_PIN, KICK_CHARGE_ON_HIGH ? LOW : HIGH);
  pinMode(KICK_DISCHARGE_PIN, OUTPUT);
  digitalWriteFast(KICK_DISCHARGE_PIN, LOW);
  pinMode(KICK_BALL_PIN, INPUT);
}

// SW2 while running: motors stop, the screen shows the moment of the press (mode, v, cuts) and
// the live sensors. SW2 again (or START): go on.
static bool sw2Pressed() {
  static bool prev = HIGH;
  static uint32_t t = 0;
  bool now = digitalRead(DATA_BUTTON);
  bool press = prev == HIGH && now == LOW && millis() - t > 200;
  if (press) t = millis();
  prev = now;
  return press;
}

static void dataScreen(const char* modeAt, Vec2 vAt, const float cutAt[4]) {
  MotorStop();
  uint32_t lastDraw = 0;
  while (true) {
    update_gyro_sensor();
    read_camera();
    read_sense();
    lineData.state |= LINE_MASK_OFF;
    if (sw2Pressed() || digitalRead(START_BUTTON) == LOW) break;
    if (millis() - lastDraw < 100) continue;
    lastDraw = millis();
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.printf("PAUSE %s v%.0f,%.0f\n", modeAt, vAt.x, vAt.y);
    display.printf("cut F%.1f R%.1f B%.1f L%.1f\n", cutAt[0], cutAt[1], cutAt[2], cutAt[3]);
    display.print("US");
    for (int i = 0; i < 4; i++) {
      float d = usCm3(i);
      if (d > 0) display.printf(" %.0f", d);
      else       display.print(" --");
    }
    display.println();
    display.printf("Ball %s %.0f s%u\n", irBall.found ? "Y" : "-", irBall.angle, irBall.strength);
    display.printf("Line %08lX esc %X brk %X\n", lineData.state, escMask, brakeMask);
    display.printf("Head %.0f Hold %d k%.2f A%.0f\n", current_heading(), has_ball(), goalK3, theirArea3);
    display.printf("Goal F%s%.0f/%u B%s%.0f/%u\n", fieldData.front_goal.seen ? "" : "-",
                   fieldData.front_goal.angle, fieldData.front_goal.dist,
                   fieldData.back_goal.seen ? "" : "-", fieldData.back_goal.angle, fieldData.back_goal.dist);
    display.println("SW2/START: go on");
    display.display();
  }
  while (digitalRead(START_BUTTON) == LOW || digitalRead(DATA_BUTTON) == LOW) {}   // release
  sw2Pressed();
}

static uint8_t senseRxExtra[512];    // Serial8 receive buffer + this (see the top)
static uint32_t linkGapMax = 0;      // ms, longest gap since the last print
static uint32_t linkPk0 = 0, linkBad0 = 0, linkJunk0 = 0;   // counters at the last print

void setup() {
  set_motor_reverse(MOTOR_REVERSE);   // before anything moves a motor
  core_act_init();
#ifndef SIM_PROG
  Serial8.addMemoryForRead(senseRxExtra, sizeof(senseRxExtra));
#endif
  pinMode(DATA_BUTTON, INPUT_PULLUP);
  detachInterrupt(digitalPinToInterrupt(PIN_0));   // Sense line pins: not maskable here (see top)
  detachInterrupt(digitalPinToInterrupt(PIN_1));
  camera_begin();
  kickerOff();
  strategy_begin();

  if (CrashReport) { Serial.print(CrashReport); drawMessage("CRASHED"); delay(2000); }

  MotorStop();
  uint32_t lastDraw = 0;
  while (digitalRead(START_BUTTON) == HIGH) {
    update_gyro_sensor();
    read_camera();
    read_sense();
    lineData.state |= LINE_MASK_OFF;
    if (millis() - lastDraw >= 100) {
      lastDraw = millis();
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.printf("URGENT START mot %s\n", MOTOR_REVERSE ? "REV" : "nor");
      display.printf("Head %.1f\n", current_heading());
      display.printf("Ball %s %.0f s%u\n", irBall.found ? "Y" : "-", irBall.angle, irBall.strength);
      display.printf("Hold %d%s  no kicker\n", has_ball(), USE_HOLD ? "" : " (off)");
      display.printf("Goal F%s%.0f/%u B%s%.0f/%u\n", fieldData.front_goal.seen ? "" : "-",
                     fieldData.front_goal.angle, fieldData.front_goal.dist,
                     fieldData.back_goal.seen ? "" : "-", fieldData.back_goal.angle, fieldData.back_goal.dist);
      display.printf("Line %08lX\n", lineData.state);
      display.printf("Sense pin %d (raw)\n", line_flag());
      display.print("US");                                  // F R B L cm, -- = failed
      for (int i = 0; i < 4; i++) {
        float d = usCm3(i);
        if (d > 0) display.printf(" %.0f", d);
        else       display.print(" --");
      }
      display.printf(US_FIELD3 ? "\n" : " OFF\n");
      display.display();
    }
  }
  drawMessage("GO");
  delay(200);
}

void loop() {
  update_gyro_sensor();
  read_camera();
  uint8_t sGot = read_sense();
  {                                  // link check: longest gap between two Sense packets
    static uint32_t lastPkt = 0;
    uint32_t nowMs = millis();
    if (lastPkt) linkGapMax = max(linkGapMax, nowMs - lastPkt);
    if (sGot & SENSE_GOT_PKT) lastPkt = nowMs;
  }
  lineData.state |= LINE_MASK_OFF;   // only the 8 sensors (ring + kicker damaged)

  if (picked_up()) {                 // lifted: stop, start again from heading 90
    set_aim(false);
    set_target_heading(90);
    MotorStop();
    return;
  }

  Vec2 v;
  if (pushShot(v)) {
    mode = M3_SHOT;
  } else {
    set_aim(false);
    set_target_heading(90);
    v = chase3();
  }

  static Vec2 lastV;                 // speed sent last loop (ultrasonic lag)
  goalSlow(v);                       // near the goals (camera)
  usField(v, lastV);                 // walls (ultrasonic), exponential
  lineGuard(v);                      // white line (8 sensors): last word
  set_ramp(!lineLock && !brakeMask); // braking: at once

  if (mode == M3_NONE && !lineLock && !theirInside3) MotorStop();
  else Vector_Motion(v.x, v.y, 0);
  lastV = (mode == M3_NONE && !lineLock && !theirInside3) ? Vec2() : v;

  if (sw2Pressed()) {                // SW2: stop and show the data (see dataScreen)
    float cutAt[4] = {usCut3[US_N_FRONT], usCut3[US_N_RIGHT], usCut3[US_N_BACK], usCut3[US_N_LEFT]};
    dataScreen(modeName3[mode], v, cutAt);
    lastV = Vec2();
  }

  EVERY_MS(tp, 200) {
    Serial.printf("%-6s | ball %s %5.1f s%2u > %4.1f | orbit g %.2f | hold %d | line %08lX esc %X brk %X | "
                  "us F %3.0f R %3.0f B %3.0f L %3.0f cut F %.1f | goal k %.2f area %4.0f | v (%4.0f, %4.0f) | "
                  "link %lu pk bad %lu junk %lu gap %lu ms\n",
                  modeName3[mode], irBall.found ? "Y" : "-", irBall.angle, irBall.strength,
                  strUsed, orbitG, has_ball(), lineData.state, escMask, brakeMask,
                  usCm3(US_N_FRONT), usCm3(US_N_RIGHT), usCm3(US_N_BACK), usCm3(US_N_LEFT),
                  usCut3[US_N_FRONT], goalK3, theirArea3, v.x, v.y,
                  (unsigned long)(senseLinkStats.pkts - linkPk0), (unsigned long)(senseLinkStats.bad - linkBad0),
                  (unsigned long)(senseLinkStats.junk - linkJunk0), (unsigned long)linkGapMax);   // strength raw > compensated
    linkPk0 = senseLinkStats.pkts; linkBad0 = senseLinkStats.bad; linkJunk0 = senseLinkStats.junk;
    linkGapMax = 0;
  }
}
