#include <Arduino.h>
#include <core_act.h>

// T12f (Act): defense robot, act_defense_smach5 rewritten SHORT (2026-11-27).
// Same main behaviour, far fewer parameters (all in the TUNING block below).
//
//   DEFEND  stay on the penalty line with F_in, move sideways onto the ball - goal line
//           pushed back: ring 3-5 / F_out on the line -> forward, all lost -> forward ramp
//           side arm: mid on a line = no more sideways, in = back off
//           corner: front + side arm on lines (facing 90) -> that side locked
//           ring guard: any other ring channel on a line pushes the robot away (never back)
//           ball 85-95 for LEAVE_HOLD_MS, or at a corner with the ball in that corner's window
//           for CORNER_HOLD_MS                                                    -> LEAVE
//           back sensor alone on a line and the camera says outside, or pushed out -> RETURN
//   LEAVE   ball held: shoot_control(), ball 85-95: rush, else chase_ball(); never backward
//           inside the area; near our goal after leaving: never back into the area;
//           near the field side: back_to_field() line avoid
//           ends: ball lost / behind, LEAVE_MAX_MS, too far -> RETURN (out) / DEFEND (in)
//   RETURN  turn to 90 first; beside our goal: forward first; else toward the goal, slower
//           near the stop point; a front sensor on the line -> DEFEND
// Every loop: our goal not seen -> stop. Lifted -> motors off.
// Sense line pin interrupt off (flickering sensors kept the LEAVE ramp at 0); the line comes
// from the Sense packet (lineData.state, cleared by read_sense() after 50 ms without packets).

#define MOTOR_REVERSE 0      // defense body

// ============================== TUNING ==============================
// DEFEND
#define SPEED            30   // forward when deep (F_out) or the line is unknown
#define FINE_SPEED       15   // small corrections, backing off a side line, ring guard push
#define MIN_SPEED        15   // any move slower than this (not 0) is raised to it
#define RECOVER_MAX      60   // pushed far back: forward from SPEED rising to this in 3 s
#define SIDE_KP          1.0f // sideways % per deg the ball is off the ball - goal line
#define SIDE_MAX         40   // largest sideways speed
#define SIDE_SLOW_DEG    45   // camera: goal this far off 270 -> sideways away from it slows ...
#define SIDE_STOP_DEG    75   // ... to MIN_SPEED here (never 0: the side arms stop it)
// DEFEND -> LEAVE
#define LEAVE_HOLD_MS    3000 // ball straight ahead (90 +- 5) this long
#define CORNER_HOLD_MS   4000 // at a corner, ball in 0..CORNER_R_MAX / CORNER_L_MIN..180 this long
#define CORNER_R_MAX     80
#define CORNER_L_MIN     100
#define CORNER_CAM_DEG  20   // camera corner: goal this far off 270 (or a side arm / lock)
// LEAVE
#define LEAVE_MAX_MS     1500 // longest LEAVE
#define LEAVE_SPEED      50   // speed limit (half of it with the ball touching)
#define LEAVE_RUSH_SPEED 70   // ball 85-95: straight at it
// RETURN
#define RETURN_SPEED     55   // top speed
#define RETURN_KP        0.7f // % per cm to the stop point (slow-down near the line)
#define GOAL_FAR_CM      50   // our goal farther than this (camera) = not at the penalty line
// Ultrasonic (each switch: 0 = broken, never used)
#define US_USE_FRONT     1
#define US_USE_RIGHT     1
#define US_USE_BACK      1
#define US_USE_LEFT      1
#define US_BACK_MIN_CM   15.5f // back reading at the stop point (MEASURE): never backward below it
#define US_WALL_CM       15    // side / back sensor: LEAVE / RETURN speed toward it 0 at this
// Heading
#define HEAD_I_K         2.0f  // heading I term (steady turn while moving sideways), 0 = off
// =====================================================================

// ----- Fixed: robot / field geometry and timing (not for tuning) -----
#define STOP_CM        4.4f   // F_in ahead of the robot centre (the stop point)
#define PA_FRONT_CM    30.0f  // penalty area from the camera's goal point: front line
#define PA_HALF_CM     39.0f  // ... side lines
#define PA_R_CM        15.0f  // ... rounded corners
#define NEAR_GOAL_CM   60.0f  // LEAVE: closer than this every line is the penalty line
#define BALL_LOST_MS   200
#define BREAK_MS       200    // ball out of the LEAVE window shorter than this: timer goes on

// Line bits (bit 0 = line): ring = mux 1 ch 0-15, mux 2 ch c = bit 16 + c
#define M2(c)      (1UL << (16 + (c)))
#define F_IN       M2(7)
#define F_OUT      M2(11)
#define BACK       (M2(4) | M2(3))
#define R_ARM      (M2(14) | M2(13))
#define L_ARM      (M2(1) | M2(0))
#define R_IN       M2(13)
#define L_IN       M2(0)
#define RING_DEEP  0x0038UL                     // ring ch 3-5 (ahead of F_in)
#define RING_FREE  0x007CUL                     // ring ch 2-6 ride the front line
#define RING_EDGE  0x0082UL                     // ring ch 1 / 7: front line too while F_in / 2-6 see it
#define RING_R     0x8003UL                     // ring ch 15, 0, 1
#define RING_L     0x0380UL                     // ring ch 7, 8, 9
#define FRONT      (F_IN | F_OUT | RING_DEEP)

static bool seen(uint32_t s, uint32_t m) { return (~s & m) != 0; }
static float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
static Vec2 polarV(float sp, float deg) { Vec2 v; v.x = sp * cosf(deg * DEG_TO_RAD); v.y = sp * sinf(deg * DEG_TO_RAD); return v; }

// ----- Our goal (camera, distance corrected for the slant: 0 deg x1, 44 x0.81, 66+ x0.61) -----
struct Goal { bool known; float angle, dist, off; };   // off = angle - 270 (< 0: robot right of the goal)
static Goal ourGoal() {
  Goal g = {false, 270, 0, 0};
  if (!camera_fresh(CAM_TIMEOUT_MS) || !fieldData.back_goal.seen) return g;
  g.known = true;
  g.angle = fieldData.back_goal.angle;
  g.off = wrap180(g.angle - 270.0f);
  float a = fabsf(g.off);
  float k = a <= 44 ? 1.0f - 0.19f * a / 44 : a <= 66 ? 0.81f - 0.20f * (a - 44) / 22 : 0.61f;
  g.dist = fieldData.back_goal.dist * k;
  return g;
}
// Robot centre outside the penalty line (cm, < 0 = inside), from the camera
static float areaDist(const Goal& g) {
  float x = fabsf(g.dist * cosf(g.angle * DEG_TO_RAD)), y = -g.dist * sinf(g.angle * DEG_TO_RAD);
  float cx = PA_HALF_CM - PA_R_CM, cy = PA_FRONT_CM - PA_R_CM;
  if (x > cx && y > cy) return hypotf(x - cx, y - cy) - PA_R_CM;
  return max(x - PA_HALF_CM, y - PA_FRONT_CM);
}

// ----- Ultrasonic: cm, 0 = failed (switched off / no reading / no packet for 200 ms) -----
static const bool usUse[4] = {US_USE_FRONT, US_USE_RIGHT, US_USE_BACK, US_USE_LEFT};
static const float usDir[4] = {90, 0, 270, 180};
static float usCm(int i) {
  if (!usUse[i] || senseUs.t_ms == 0 || millis() - senseUs.t_ms > 200) return 0;
  return senseUs.cm[i];
}
// Speed toward each sensor in mask: 0 at its limit, +1.5 % per cm past it; failed: x 0.7.
// minBack: backward is never cut below this.
static void usSlow(Vec2& v, uint8_t mask, float minBack = 0) {
  for (int i = 0; i < 4; i++) {
    if (!(mask & (1 << i))) continue;
    float ux = cosf(usDir[i] * DEG_TO_RAD), uy = sinf(usDir[i] * DEG_TO_RAD);
    float into = v.x * ux + v.y * uy;
    if (into <= 0) continue;
    float d = usCm(i), stop = i == US_N_BACK ? US_BACK_MIN_CM : US_WALL_CM;
    float lim = d <= 0 ? into * 0.7f : max(0.0f, 1.5f * (d - stop));
    if (i == US_N_BACK) lim = max(lim, min(into, minBack));
    if (into > lim) { v.x -= (into - lim) * ux; v.y -= (into - lim) * uy; }
  }
}

// ----- Ball: IR only -----
static MergedBall irBallData() {
  MergedBall m;
  if (!ir_ball_fresh(IR_TIMEOUT_MS)) return m;
  m.src = SRC_IR;
  m.angle = irBall.angle;
  m.strength = irBall.strength;
  m.closeness = clampf((irBall.strength - IR_FAR_STR) / (IR_NEAR_STR - IR_FAR_STR), 0, 1);
  return m;
}

// ----- State -----
enum St : uint8_t { DEFEND, LEAVE, RETURN };
static const char* stName[] = {"DEFEND", "LEAVE", "RETURN"};
static St st = DEFEND;
static const char* why = "";
static uint32_t stStart = 0, lastBall = 0;
static bool lockR = false, lockL = false;      // DEFEND corner lock
static bool recovering = false;                // DEFEND pushed far back
static uint32_t recoverStart = 0;
static bool prevOut = false;                   // DEFEND F_out saw the line last loop
static uint8_t lastFront = 0;                  // last front sensor that saw the line: 0 ?, 1 F_in, 2 deeper
static bool timing = false, timingCorner = false;   // LEAVE timer
static uint32_t timerStart = 0, timerLastIn = 0;
static bool backSeen = false, leaveOut = false;     // LEAVE: back sensors saw the line, then left it
static bool beside = false;                    // RETURN: beside our goal, forward first

static void go(St s, const char* reason) {
  st = s; why = reason; stStart = millis();
  lockR = lockL = recovering = prevOut = timing = backSeen = leaveOut = beside = false;
  if (s == RETURN) lastFront = 0;
  reset_heading_i();
  Serial.printf("%lu ms: -> %s (%s)\n", millis(), stName[s], reason);
}
static void leaveEnd(const char* reason) { if (leaveOut) go(RETURN, reason); else go(DEFEND, "still in the area"); }
static bool facing(float tol) { return fabsf(wrap180(current_heading() - 90.0f)) <= tol; }

// ----- DEFEND motion -----
static Vec2 defendMotion(uint32_t s, const MergedBall& ball, const Goal& goal) {
  Vec2 v;
  bool armR = seen(s, R_ARM), armL = seen(s, L_ARM);
  bool fin = seen(s, F_IN), out = seen(s, F_OUT), deep = seen(s, RING_DEEP);
  bool anyFront = fin || out || deep;
  bool frontRide = seen(s, F_IN | RING_FREE);                 // F_in / ring 2-6 on the front line
  uint32_t ringSide = frontRide ? ~RING_EDGE : 0xFFFFFFFFUL;  // ch 1 / 7 then = front line, not side
  bool ringR = seen(s, RING_R & ringSide), ringL = seen(s, RING_L & ringSide);

  if ((armR && armL) || (ringR && ringL)) { v.y = -SPEED; return v; }   // front line under the robot

  // Forward / back: F_in on the line. Pushed back: ring 3-5 / F_out first, then all lost.
  if (fin) lastFront = 1; else if (out || deep) lastFront = 2;
  if (!recovering && prevOut && !anyFront) { recovering = true; recoverStart = millis(); }
  prevOut = out;
  if (recovering && (anyFront || areaDist(goal) > 0)) recovering = false;
  if (recovering) { v.y = min((float)RECOVER_MAX, SPEED + (millis() - recoverStart) * 0.01f); return v; }
  if (fin)                 v.y = 0;
  else if (out)            v.y = SPEED;
  else if (deep)           v.y = FINE_SPEED;
  else if (lastFront == 1) v.y = -FINE_SPEED;          // F_in lost it toward the centre: went out
  else if (lastFront == 2) v.y = FINE_SPEED;
  else                     v.y = SPEED;                // unknown: forward, never deeper
  float usB = usCm(US_N_BACK);
  if (!anyFront && usB > 0 && usB < US_BACK_MIN_CM) v.y = max(v.y, (float)SPEED);   // deep by the ultrasonic
  if (armR || armL || ringR || ringL) v.y = max(v.y, 0.0f);                       // at a side line: never back

  // Sideways onto the ball - goal line (ball and goal 180 apart); no ball: goal at 270
  float err = ball.src != SRC_NONE ? wrap180(ball.angle - (goal.angle + 180.0f)) : -goal.off;
  v.x = fabsf(err) <= 5 ? 0 : clampf(-err * SIDE_KP, -SIDE_MAX, SIDE_MAX);
  float want = v.x;

  // Corner lock: facing 90, a front sensor and a side arm on lines; unlocked when it wants back
  if (facing(10) && anyFront) { if (armR) lockR = true; if (armL) lockL = true; }
  if (lockR && want < 0) lockR = false;
  if (lockL && want > 0) lockL = false;

  // Camera: slower toward the end of the penalty line, never below MIN_SPEED
  if ((goal.off < 0 && v.x > 0) || (goal.off > 0 && v.x < 0)) {
    float k = clampf((SIDE_STOP_DEG - fabsf(goal.off)) / (float)(SIDE_STOP_DEG - SIDE_SLOW_DEG), 0, 1);
    float a = fabsf(v.x);
    v.x = copysignf(max(a * k, min(a, (float)MIN_SPEED)), v.x);
  }
  // Side arms always win: mid = stop, in = back off
  if (seen(s, R_IN)) v.x = min(v.x, -(float)FINE_SPEED); else if (armR) v.x = min(v.x, 0.0f);
  if (seen(s, L_IN)) v.x = max(v.x, (float)FINE_SPEED);  else if (armL) v.x = max(v.x, 0.0f);
  // Locked: no sideways, no backward (an in sensor still moves it back in)
  if (lockR || lockL) {
    if (!seen(s, R_IN | L_IN)) v.x = 0;
    v.y = max(v.y, 0.0f);
  }

  // Ring guard: other ring channels on a line push the robot away (at least FINE_SPEED, never back)
  float ax = 0, ay = 0;
  for (int ch = 0; ch < 16; ch++) {
    uint32_t b = 1UL << ch;
    if ((RING_FREE & b) || (frontRide && (RING_EDGE & b)) || !seen(s, b)) continue;
    ax -= cosf(ch * 22.5f * DEG_TO_RAD);
    ay -= sinf(ch * 22.5f * DEG_TO_RAD);
  }
  ay = max(ay, 0.0f);
  float an = hypotf(ax, ay);
  if (an > 0.01f) {
    ax /= an; ay /= an;
    float along = v.x * ax + v.y * ay;
    if (along < FINE_SPEED) { v.x += (FINE_SPEED - along) * ax; v.y += (FINE_SPEED - along) * ay; }
  }
  return v;
}

void setup() {
  set_motor_reverse(MOTOR_REVERSE);
  core_act_init();
  detachInterrupt(digitalPinToInterrupt(PIN_0));   // see the top
  detachInterrupt(digitalPinToInterrupt(PIN_1));
  camera_begin();
  kicker_begin();
  strategy_begin();
  set_heading_i(HEAD_I_K, 3, 15, 15);              // integrate beyond 3 deg (below 15), at most 15 %

  MotorStop();
  while (digitalRead(START_BUTTON) == HIGH) {
    update_gyro_sensor(); read_camera(); read_sense(); kicker_update();
    EVERY_MS(td, 100) {
      Goal g = ourGoal();
      display.clearDisplay(); display.setTextSize(1); display.setCursor(0, 0);
      display.println("DEFENSE SM6 START");
      display.printf("Head %.1f\n", current_heading());
      display.printf("Ball %s %.0f\n", irBall.found ? "Y" : "-", irBall.angle);
      display.printf("Goal %s %.0f %.0fcm\n", g.known ? "Y" : "-", g.angle, g.dist);
      display.printf("Line %08lX\n", lineData.state);
      display.print("US");
      for (int i = 0; i < 4; i++) { float d = usCm(i); if (d > 0) display.printf(" %.0f", d); else display.print(" --"); }
      display.display();
    }
  }
  drawMessage("GO");
  delay(200);
  go(DEFEND, "start");
}

void loop() {
  update_gyro_sensor();
  read_camera();
  uint8_t got = read_sense();
  kicker_update();
  if (picked_up()) { set_aim(false); set_target_heading(90); reset_heading_i(); MotorStop(); return; }

  uint32_t now = millis();
  uint32_t s = lineData.state;
  MergedBall ball = irBallData();
  if (ball.src != SRC_NONE || has_ball()) lastBall = now;
  set_aim(false);
  set_target_heading(90);
  set_ramp(st != DEFEND);

  Goal goal = ourGoal();
  if (!goal.known) {                                   // our goal not seen: stop
    if (st == LEAVE) leaveEnd("goal unknown");
    timing = false;
    set_ramp(false);
    Vector_Motion(0, 0, 0);
    EVERY_MS(tu, 200) Serial.printf("%-6s (%s) | goal unknown: stopped\n", stName[st], why);
    return;
  }

  Vec2 v;
  bool shooting = false;
  float usB = usCm(US_N_BACK);
  switch (st) {
    case DEFEND: {
      // Pushed out: goal far, no front / side sensor on a line, the back ultrasonic agrees
      if (goal.dist > GOAL_FAR_CM && !seen(s, FRONT | R_ARM | L_ARM) &&
          (usB <= 0 || usB > US_BACK_MIN_CM + 15)) { go(RETURN, "pushed out"); break; }
      // LEAVE timer
      float ba = fmodf(ball.angle + 360.0f, 360.0f);
      bool atR = goal.off <= -CORNER_CAM_DEG || lockR || seen(s, R_ARM);
      bool atL = goal.off >=  CORNER_CAM_DEG || lockL || seen(s, L_ARM);
      bool front = ball.src != SRC_NONE && fabsf(wrap180(ba - 90.0f)) <= 5;
      bool corner = ball.src != SRC_NONE && ((atR && ba <= CORNER_R_MAX) || (atL && ba >= CORNER_L_MIN && ba <= 180));
      if (front || corner) {
        if (!timing || timingCorner != !front) { timing = true; timerStart = now; timingCorner = !front; }
        timerLastIn = now;
      } else if (timing && now - timerLastIn > BREAK_MS) timing = false;
      if (timing && now - timerStart >= (timingCorner ? CORNER_HOLD_MS : LEAVE_HOLD_MS)) {
        go(LEAVE, timingCorner ? "ball at the corner" : "ball in front"); break;
      }
      if (!facing(20)) break;                          // turned: turn back first
      // Back sensor alone on a line: camera outside -> front line (RETURN), else forward
      if (seen(s, BACK) && !seen(s, FRONT)) {
        if (areaDist(goal) > 3) { go(RETURN, "back sensor on the front line"); break; }
        lastFront = 0;
      }
      v = defendMotion(s, ball, goal);
      if (v.y < 0 && usB > 0 && usB < US_BACK_MIN_CM) v.y = 0;   // never backward below the limit
      break;
    }

    case LEAVE: {
      if (!has_ball()) {
        if (now - lastBall > BALL_LOST_MS) { leaveEnd("ball lost"); break; }
        if (ball.src != SRC_NONE && fabsf(wrap180(ball.angle - 270.0f)) < 60) { leaveEnd("ball behind"); break; }
      }
      if (now - stStart > LEAVE_MAX_MS) { leaveEnd("time up"); break; }
      if (goal.dist > 90)               { leaveEnd("too far"); break; }

      ShootCtl shot = shoot_control(fieldData);
      shooting = shot.active;
      if (shot.active) {
        v = shot.v; set_aim(shot.aiming); set_target_heading(shot.heading);
      } else {
        if (ball.src != SRC_NONE && fabsf(wrap180(ball.angle - 90.0f)) <= 5) v = polarV(LEAVE_RUSH_SPEED, ball.angle);
        else v = chase_ball(ball, fieldData);
        float lim = LEAVE_SPEED * (1.0f - 0.5f * ball.closeness), sp = hypotf(v.x, v.y);
        if (sp > lim) { v.x *= lim / sp; v.y *= lim / sp; }
      }
      if (seen(s, BACK)) backSeen = true; else if (backSeen) leaveOut = true;
      if (!leaveOut && !shooting) v.y = max(v.y, 0.0f);          // inside the area: never backward
      if (leaveOut && goal.dist < NEAR_GOAL_CM) {                 // near our goal: never back in
        v.y = max(v.y, 0.0f);
        if (seen(s, F_IN | F_OUT)) v.y = min(v.y, 0.0f);
        if (seen(s, R_ARM)) v.x = min(v.x, 0.0f);
        if (seen(s, L_ARM)) v.x = max(v.x, 0.0f);
      }
      usSlow(v, (1 << US_N_RIGHT) | (1 << US_N_LEFT) | (1 << US_N_BACK));
      // Near the field side (goal > 45 off 270, not near our goal): leave the outer line
      if (goal.dist >= NEAR_GOAL_CM && fabsf(goal.off) > 45) {
        LineCtl line = back_to_field(got & SENSE_GOT_LINE);
        if (line.active) {
          v = line.v;
          if (line.line_dir < 0) {                     // only a mux 2 arm sees it: away from that arm
            float ax = (seen(s, L_ARM) ? 1 : 0) - (seen(s, R_ARM) ? 1 : 0);
            float ay = (seen(s, BACK) ? 1 : 0) - (seen(s, F_IN | F_OUT) ? 1 : 0);
            if (ax != 0 || ay != 0) v = polarV(LINE_AVOID_SPEED, atan2f(ay, ax) * RAD_TO_DEG);
          }
          set_aim(false); set_target_heading(90); set_ramp(false);
        }
      }
      if (!leaveOut && !shooting) v.y = max(v.y, 0.0f);   // also the line avoid: inside the area never back
      break;
    }

    case RETURN: {
      // a front sensor on the line = at the penalty line; camera only rules out a line far away
      // (x 1.5: a camera reading 50 % long at the line still counts)
      if (seen(s, FRONT) && !seen(s, BACK) && goal.dist <= 1.5f * GOAL_FAR_CM) { go(DEFEND, "at the line"); break; }
      if (!facing(20)) break;                          // turn first (rule 2.8)
      if (areaDist(goal) < -3 && goal.dist <= GOAL_FAR_CM) { go(DEFEND, "camera: inside the area"); break; }
      // Beside our goal (goal near 0 / 180): forward first (from 30 deg, until 40 deg)
      float sideOff = min(fabsf(wrap180(goal.angle)), fabsf(wrap180(goal.angle - 180.0f)));
      if (sideOff < 30) beside = true; else if (sideOff > 40) beside = false;
      float sp = beside ? RETURN_SPEED : clampf(RETURN_KP * (areaDist(goal) + STOP_CM), FINE_SPEED, RETURN_SPEED);
      if (seen(s, BACK | 0xFFFFUL)) sp = FINE_SPEED;    // coming in over a line: slowest
      v = polarV(sp, beside ? 90.0f : goal.angle);
      // walls, and never backward below the back limit (unless the camera says far: wrong reading)
      usSlow(v, (1 << US_N_RIGHT) | (1 << US_N_LEFT) | (1 << US_N_BACK), goal.dist > 60 ? FINE_SPEED : 0);
      break;
    }
  }

  float sp = hypotf(v.x, v.y);
  if (!shooting && sp > 0.5f && sp < MIN_SPEED) { v.x *= MIN_SPEED / sp; v.y *= MIN_SPEED / sp; }
  if (st == DEFEND && (lockR || lockL) && v.x == 0 && v.y == 0 && facing(10)) MotorStop();
  else Vector_Motion(v.x, v.y, 0);

  EVERY_MS(tp, 200) {
    Serial.printf("%-6s (%s) | ball %s %5.1f | goal %5.1f %3.0f cm area %4.0f | line %08lX | "
                  "us R %3.0f B %3.0f L %3.0f | lock %c%c | v (%4.0f, %4.0f)\n",
                  stName[st], why, ball.src != SRC_NONE ? "IR" : "- ", ball.angle, goal.angle, goal.dist,
                  areaDist(goal), s, usCm(US_N_RIGHT), usB, usCm(US_N_LEFT),
                  lockL ? 'L' : '-', lockR ? 'R' : '-', v.x, v.y);
  }
}
