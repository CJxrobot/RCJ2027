#include <Arduino.h>
#include <core_act.h>

// Motor direction of this robot body: 1 = attack body (wired the other way), 0 = defense body
#define MOTOR_REVERSE 1

// T11g (Act): act_attack_1129 = act_attack5 + (2026-11-29, act_attack5 is not changed):
//   1. No ball (IR): robot in the front half -> go to the field centre (0, MID_Y_OFFSET), heading
//      90, P speed (MID_P, max MID_SPEED, stop within MID_TOL); back half -> stop. Ball seen on
//      the way: chase at once (attack5).
//      Position: x = (L - R) / 2, y = (B - F) / 2 from the ultrasonics when the pair is sane
//      (both read, sum + robot within US_SUM_TOL of the field); else the camera (goal angle +
//      distance, goals at (0, +-CAM_GOAL_Y), both goals averaged). x unknown: no side move;
//      y unknown: stop. Front half: y > HALF_HYST, back half: y < -HALF_HYST (keeps in between).
//   2. Gone over the line (lineCrossed, full-speed back): also ends when that side's ultrasonic is
//      ESC_BACK_CM farther than at the last touch, or (line in front / behind) that goal's camera
//      distance ESC_BACK_CM farther than at the last touch (relative: an absolute distance is
//      already long beside the goal, in a corner). No timer. Ring sees the line again: as before.
//   3. No kicker: kicker_begin / kicker_update never called, charge + discharge pins LOW.
//      Ball held: push, turning to the goal as attack5 (aim, rotate(), AIM_PUSH_SPEED); goal not
//      seen: heading 90, NOGOAL_PUSH_SPEED. No kick, no corner shot (it waits for a kick).
//      AIM_LOCK_ON 1: goal angle < AIM_LOCK_LOW or > AIM_LOCK_HIGH -> heading locked where it
//      was, no turn (as act_attack_urgent); goal lost while locked: heading 90.
//   Everything else: act_attack5.
//
// act_attack5 = act_attack4 + (2026-11-27, act_attack4 is not changed):
//   front ultrasonic ALWAYS used (it does not see the ball: no skip while the ball is in front
//     or held). Facing the opponent goal (camera front goal within 90 +- FACE_GOAL_DEG5): no wall
//     slow-down from the front sensor, instead the forward part is at most GOAL_CALM_SPEED5 while
//     it reads under GOAL_CALM_CM5 (no full-speed rush into the goal; the stop before their area
//     is still the camera's). Not facing the goal: the normal exponential wall slow-down.
//   white line in front (line direction within 90 +- LINE_FRONT_DEG5): escape LINE_ESCAPE_FRONT5
//     instead of LINE_ESCAPE3 (comes back into the field harder).
// Shooting (shoot_control) not changed.
// Sense link (2026-11-27: line / ball data missing now and then): Serial8 receive buffer + 512
//   bytes (default 64 = < 1 ms at 921600 baud if one loop runs long), and the 200 ms print ends
//   with "link": packets in those 200 ms, checksum errors, junk bytes, longest gap between two
//   packets (ms). Act treats > 50 ms without a packet as "no line / no ball" (IR_TIMEOUT_MS).
//
// From act_attack4 (= act_attack3 + mux 2 early brake):
// Sense: sense_attack (mux 2 on). Mux 2 sensors sit outside the ring (6.1 cm) and touch a
// line first. They do NOT take part in the line direction (ring only, as act_attack3); each
// one only cuts the move toward its own side while it sees a line (no memory, no escape):
//   F_out (8.0 cm)            -> no move forward (90)      F_in: ignored for now
//   R_mid / R_in (9.5 / 8.5)  -> no move right (0)
//   L_mid / L_in              -> no move left (180)
//   B_mid / B_in              -> no move back (270)
// Not while coming back over a line on purpose (crossed it, or centre inside their area).
// Everything below is act_attack3.
//
//   picked up               -> motors off
//   ball held               -> shoot_control() (push while turning to the goal, kick; as defense)
//   ball 90 +- PUSH_DEG3    -> forward: PUSH_SLOW3 for PUSH_SLOW_MS, then PUSH_SPEED3
//   IR strength >= STR_ORBIT -> around the ball (exponential, see below), slower near the front:
//                              ORBIT_SLOW3 + (ORBIT_SPEED3 - ORBIT_SLOW3) x min(1, |off| / ORBIT_SLOW_DEG)
//   IR strength <  STR_ORBIT -> straight at the ball, CHASE_SPEED3
//   no ball                 -> stop
//   white line (no timer):
//     first touch -> init = line direction (line_angle of the masked ring bits, field frame), LOCK
//     line seen   -> current direction updates dir only while within LINE_SIDE_DEG (90) of init:
//                    past the middle of the line the sensors see it on the other side, that
//                    one is ignored (dir stays). Move: the part of the wanted move toward dir is
//                    0 (brake, no ramp), moves along the line go on, + LINE_ESCAPE3 away from dir
//     line gone   -> normally unlocked at once (no memory). The old "stay locked until the wanted
//                    move is not toward the line" kept LOCK with line FFFF anywhere on the field
//                    and blocked every move in that direction (2026-11-26): removed.
//                    CROSSED: the last sensor that saw the line was on the far side (raw direction
//                    more than LINE_SIDE_DEG from init) -> the robot went all the way over it at
//                    speed (2026-11-26: pushed on into the opponent area) -> stay locked and keep
//                    going back (away from init) until the line is seen again and left on the
//                    right side.
//     direction not known yet -> stop
//
// Orbit (exponential in the IR strength, the closer the ball the wider around it):
//   off = how far the ball is off the front (-180..180)
//   g   = min(1, exp((strength - STR_FULL) / STR_LAMBDA))     strength 5 -> 0.50, 6 -> 0.71, 7+ -> 1
//   add = min(ORBIT_MAX_DEG, |off| x ORBIT_K) x g, at least ORBIT_MIN_DEG (ball in the back half:
//         ORBIT_MIN_BACK_DEG) so a near ball is never driven straight at
//   dir = ball angle + sign(off) x add
//   strength 5: ball 60-180 deg off the front -> 45 deg (field 2026-11-25: "about 45 at 5")
//
// Wheel shadow (field 2026-11-25): near 45 / 135 / 225 / 315 the wheels hide part of the IR ring,
// the strength reads low ("far") and the robot drives straight into the ball. The strength used
// above is compensated: + WHEEL_BONUS[i] at the wheel angle, falling linearly to 0 at
// WHEEL_ZONE_DEG from it. MEASURE: ball 20 cm away at 90 and at 45 / 135 / 225 / 315, read s on
// the OLED / Serial: bonus = s(90) - s(wheel angle).
// Around 270 (behind) the same: + BACK_BONUS at 270, 0 at BACK_ZONE_DEG from it (2026-11-26).
// MEASURE: ball 20 cm away at 90 and at 270: BACK_BONUS = s(90) - s(270).
//
// Ultrasonic field (2026-11-26: at high speed the front sensors run over the line):
//   per sensor (front / right / back / left), the part of the move toward it x
//     (1 - exp(-(line - US_LINE_STOP3) / US_LAMBDA3)) ... 0 at the stop
//   line = reading - US_WALL_TO_LINE3 (wall to white line) - last loop's speed x US_LAG3,
//          that correction at most US_LAG_MAX3 (the reading is ~0.3 s old)
//   front sensor: act_attack5 always uses it (see the top); facing their goal: speed cap only
//   failed (switched off / 0 / no packet US_STALE3_MS) -> x US_FAIL3 toward that side
//   Applied before the white line (the line sensors have the last word).
//
// Goals (camera, 2026-11-26; no camera / goal not seen: nothing changes):
//   toward the opponent goal (front goal, move with a part toward it): whole speed x
//     FRONT_MIN_K + (1 - FRONT_MIN_K) x (1 - exp(-(dist - GOAL_NEAR_CM) / FRONT_LAMBDA))  (a bit)
//   own-goal risk: the ball between the robot and our goal (ball within OWN_RISK_DEG of the back
//     goal direction) and the move goes toward our goal: whole speed x the same with
//     OWN_MIN_K / OWN_LAMBDA (strong)
//   opponent penalty area (camera front goal, same area shape as the defense): the part of the
//     move toward the goal x (1 - exp(-(area dist - THEIR_STOP_CM) / THEIR_LAMBDA)), 0 when the
//     robot centre is THEIR_STOP_CM outside the area line (robot partly inside, never fully:
//     2026-11-26 the push went on into the area). Sideways along the area goes on.
//     Centre already THEIR_OUT_CM inside (carried over at speed): straight out, THEIR_OUT_SPEED.
//   the white line escape is never slowed
//
// Serial every 200 ms: mode | ball angle / strength | hold | v

#define DATA_BUTTON    26     // SW2 (netlist U1.18 = pin 26), pressed = LOW: stop + show the data
#define PUSH_DEG3      5.0f   // ball this close to 90: push (library PUSH_DEG 10 is kept for defense)
#define PUSH_SPEED3    60.0f  // ball in front: forward ...
#define PUSH_SLOW3     40.0f  // ... but this for the first PUSH_SLOW_MS
#define PUSH_SLOW_MS   300
#define ORBIT_SLOW3    25.0f  // orbit speed with the ball near the front ...
#define ORBIT_SLOW_DEG 60.0f  // ... up to ORBIT_SPEED3 from this far off the front
#define LINE_ESCAPE3   40.0f  // on the line: away from it
#define LINE_ESCAPE_FRONT5 60.0f // ... line in front (within 90 +- LINE_FRONT_DEG5): harder (attack5)
#define LINE_FRONT_DEG5    45.0f
#define FACE_GOAL_DEG5     30.0f // facing their goal: camera front goal within 90 +- this (attack5)
#define GOAL_CALM_CM5      30.0f // ... front ultrasonic under this: forward at most GOAL_CALM_SPEED5
#define GOAL_CALM_SPEED5   30.0f
#define LINE_SIDE_DEG  90.0f  // line direction updates only within this of the first one
// Broken ring channels, Act side (emergency, no Sense flash needed; sense.cpp has its own mask).
// Bit ch = ring channel ch (angle ch x 22.5). Masked channels are never "line". Everything about
// the line here comes from the masked ring bits ONLY: the Sense flag pins (PIN_0 / PIN_1) cannot
// be masked on the Act side (a broken channel, or mux 2 noise with USE_MUX2 1, keeps them on and
// their interrupt stops the motors), so this program detaches that interrupt and does not use
// line_flag() / back_to_field().
#define MID_Y_OFFSET   0.0f   // no ball, front half: stop at (0, this): + toward their goal, - ours (cm)
#define MID_SPEED     50.0f  // ... at most this fast
#define MID_P         0.6f   // ... speed = this x distance (cm), up to MID_SPEED (sim: 2 overshot ~30 cm, US lag)
#define MID_TOL       5.0f   // ... stop within this (cm)
#define HALF_HYST      5.0f   // front half: y > this, back half: y < -this (between: as before)
#define FIELD_LEN_CM   243.0f // wall to wall, front - back
#define FIELD_WID_CM   182.0f // wall to wall, left - right
#define US_ROBOT_CM    22.0f  // between two opposite ultrasonics (robot size)
#define US_SUM_TOL     30.0f  // pair sane: F + B (L + R) + US_ROBOT_CM within this of the field
#define CAM_GOAL_Y     108.5f // goal (camera distance point) from the centre line
#define ESC_BACK_CM    15.0f  // gone over: that side's ultrasonic this much farther than at the touch = back
#define AIM_LOCK_ON    1      // 1 = heading lock while pushing when the goal is too far off
#define AIM_LOCK_LOW   30.0f  //   goal angle under this ...
#define AIM_LOCK_HIGH  150.0f //   ... or over this (incl. the back half): locked
#define AIM_LOCK_HYST  5.0f   //   unlocked again only inside LOW + this .. HIGH - this
#define ORBIT_SLIDE    30.0f  // orbit blocked by a mux 2 brake side: slide along that line toward the ball ...
#define ORBIT_SLIDE_MIN 20.0f //   ... when the brake leaves less than this or it goes away from the ball
                              //   (2026-11-29: stuck at our area line, ball right behind)
#define LINE_IGNORE4   0x00000080UL  // 32 bit: ring ch 7 (157.5 deg) broken (2026-11-25); mux 2 ch c = bit 16 + c

// Mux 2 early brake (same channels as the defense robot)
#define M2B(ch) (1UL << (16 + (ch)))
static const uint32_t ARM_MASK[4] = {
  M2B(14) | M2B(13),   // right: R_mid, R_in      -> 0
  M2B(11),             // front: F_out (F_in 7 ignored for now) -> 90
  M2B(1) | M2B(0),     // left:  L_mid, L_in      -> 180
  M2B(4) | M2B(3),     // back:  B_mid, B_in      -> 270
};
static uint8_t armSeen4 = 0;           // bit i = ARM_MASK[i] sees a line (print / screen)
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
#define US_FAIL3       0.7f   // failed sensor: speed toward its side x this
#define US_WALL_TO_LINE3 13.0f // wall to the white line centre
#define US_LINE_STOP3  3.0f   // sensor (robot edge) this far before the line: no speed toward it
#define US_LAMBDA3     10.0f  // exponential: 13 cm before the stop -> 27 % left ... 0 at it
#define US_LAG3        0.30f  // reading age (s)
#define US_LAG_MAX3    20.0f  // the lag correction at most this (cm)
#define US_CMS_PCT3    3.0f   // 100 % = 300 cm/s
#define US_BALL_DEG3   30.0f  // (attack5: not used, the front sensor is always used)

enum Mode3 : uint8_t { M3_NONE, M3_SHOT, M3_PUSH, M3_CHASE, M3_ORBIT, M3_HOME };
static bool lineLock = false;          // white line: direction locked
static float lineDirField = 0;         // line direction, field frame (robot frame + heading - 90)
static float lineInitField = 0;        // direction at the first touch, field frame
static float lineRawField = 0;         // direction the sensors saw last, field frame
static bool lineCrossed = false;       // went all the way over the line (see the top)
static bool lineOn = false;            // a sensor sees the line now (for the print)
static const char* modeName3[] = {"NOBALL", "SHOT", "PUSH", "CHASE", "ORBIT", "HOME"};
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

static float wrap360f(float a) { a = fmodf(a, 360.0f); return a < 0 ? a + 360.0f : a; }

// Part of v going toward dir (robot frame, deg)
static float toward(const Vec2& v, float dir) {
  return v.x * cosf(dir * DEG_TO_RAD) + v.y * sinf(dir * DEG_TO_RAD);
}

// Mux 2 early brake: each side that sees a line cuts the move toward it (see the top)
static void armBrake(Vec2& v) {
  armSeen4 = 0;
  for (int i = 0; i < 4; i++) {
    if ((lineData.state & ARM_MASK[i]) == ARM_MASK[i]) continue;   // none of them on a line
    armSeen4 |= 1 << i;
    // going back over a line on purpose (crossed it / centre inside their area): not braked
    if (lineCrossed || theirInside3) continue;
    float dir = 90.0f * i;
    float in = toward(v, dir);
    if (in > 0) { v.x -= in * cosf(dir * DEG_TO_RAD); v.y -= in * sinf(dir * DEG_TO_RAD); }
  }
}

// Orbit blocked by a mux 2 brake side (the way round the ball is over that line, e.g. ball beside
// our area line, behind): what the brake would leave is under ORBIT_SLIDE_MIN, or goes away from
// the ball -> slide along that line toward the ball at ORBIT_SLIDE instead. Runs before goalSlow /
// usField / armBrake (they still apply to the slide).
static bool slide4 = false;            // for the print
static void orbitSlide(Vec2& v) {
  slide4 = false;
  if (mode != M3_ORBIT || lineCrossed || theirInside3) return;
  float a = irBall.angle * DEG_TO_RAD;
  for (int i = 0; i < 4; i++) {
    if ((lineData.state & ARM_MASK[i]) == ARM_MASK[i]) continue;   // that side sees no line
    float ux = cosf(90.0f * i * DEG_TO_RAD), uy = sinf(90.0f * i * DEG_TO_RAD);
    float in = v.x * ux + v.y * uy;
    if (in <= 0) continue;                                         // not going toward it
    float lx = v.x - in * ux, ly = v.y - in * uy;                  // what the brake would leave
    bool awayFromBall = lx * cosf(a) + ly * sinf(a) < 0;
    if (hypotf(lx, ly) >= ORBIT_SLIDE_MIN && !awayFromBall) continue;   // enough left, toward the ball
    float tx = -uy, ty = ux;                                       // along the line ...
    if (cosf(a) * tx + sinf(a) * ty < 0) { tx = -tx; ty = -ty; }   // ... toward the ball
    v.x = tx * ORBIT_SLIDE;
    v.y = ty * ORBIT_SLIDE;
    slide4 = true;
    return;
  }
}

// White line (no timer, see the top)
// Gone over the line: side (0 right, 1 front, 2 left, 3 back) and its ultrasonic at the last touch
static const int SIDE_US[4] = {US_N_RIGHT, US_N_FRONT, US_N_LEFT, US_N_BACK};
static int crossSide = 1;
static float crossUsCm = 0, crossGoalCm = 0;
static float goalCm(int side) {                         // front / back goal (camera), 0 = not seen
  if (side != 1 && side != 3) return 0;
  const FieldObj& g = side == 1 ? fieldData.front_goal : fieldData.back_goal;
  return camera_fresh(CAM_TIMEOUT_MS) && g.seen && g.dist != PKT_NONE ? (float)g.dist : 0;
}
static int sideOf(float dirRobot) { return ((int)lroundf(wrap360f(dirRobot) / 90.0f)) % 4; }
static bool crossedBack() {
  float d = usCm3(SIDE_US[crossSide]);
  if (d > 0 && crossUsCm > 0 && d - crossUsCm > ESC_BACK_CM) return true;
  float g = goalCm(crossSide);
  return g > 0 && crossGoalCm > 0 && g - crossGoalCm > ESC_BACK_CM;
}

static void lineGuard(Vec2& v) {
  float turn = current_heading() - 90.0f;
  uint32_t ring = lineData.state | 0xFFFF0000UL;        // ring only, masked (see loop)
  lineOn = ring != 0xFFFFFFFFUL;
  // line direction from the ring, robot frame; opposite sensors cancel -> the locked side
  float lineDir = lineOn ? line_angle(ring, lineLock ? wrap360f(lineDirField - turn) : -1.0f) : -1.0f;
  if (lineOn && lineDir >= 0) {
    float d = lineDir + turn;
    if (!lineLock) { lineLock = true; lineInitField = d; lineDirField = d; }
    else if (fabsf(wrap180(d - lineInitField)) <= LINE_SIDE_DEG) lineDirField = d;
    lineCrossed = false;
  }
  if (lineOn) {                                         // raw side, no fallback (cancel -> keep)
    float r = line_angle(ring, -1.0f);
    if (r >= 0) lineRawField = r + turn;
    if (lineLock) {                                     // side of the line + its ultrasonic now
      crossSide = sideOf(lineDirField - turn);
      crossUsCm = usCm3(SIDE_US[crossSide]);
      crossGoalCm = goalCm(crossSide);
    }
  }
  if (!lineOn) {
    // line gone: unlocked, unless the far side saw it last (went over it) -> keep going back
    if (lineLock && fabsf(wrap180(lineRawField - lineInitField)) > LINE_SIDE_DEG) lineCrossed = true;
    if (lineCrossed && crossedBack()) lineCrossed = false;   // clearly back (see the top)
    if (!lineCrossed) { lineLock = false; return; }
  }
  if (!lineLock) { v = Vec2(); return; }                // direction not known yet: stop
  float dir = lineDirField - turn;                      // robot frame now
  float ux = cosf(dir * DEG_TO_RAD), uy = sinf(dir * DEG_TO_RAD);
  float in = toward(v, dir);
  if (in > 0) { v.x -= in * ux; v.y -= in * uy; }       // brake toward the line, along it goes on
  float esc = fabsf(wrap180(dir - 90.0f)) <= LINE_FRONT_DEG5 ? LINE_ESCAPE_FRONT5 : LINE_ESCAPE3;
  v.x -= esc * ux;                                      // on it: away (line in front: harder)
  v.y -= esc * uy;
}

// ---------- No ball: position, front / back half, go to the centre (see the top) ----------
static bool posXOk = false, posYOk = false;
static float posX = 0, posY = 0;       // field frame, cm, centre = 0, +y toward their goal
static bool inFront = false;           // front half (hysteresis)
static void estimatePos() {
  float L = usCm3(US_N_LEFT), R = usCm3(US_N_RIGHT), F = usCm3(US_N_FRONT), B = usCm3(US_N_BACK);
  bool usX = L > 0 && R > 0 && fabsf(L + R + US_ROBOT_CM - FIELD_WID_CM) < US_SUM_TOL;
  bool usY = F > 0 && B > 0 && fabsf(F + B + US_ROBOT_CM - FIELD_LEN_CM) < US_SUM_TOL;
  float cx = 0, cy = 0; int n = 0;                      // camera: from each goal seen
  if (camera_fresh(CAM_TIMEOUT_MS)) {
    float turn = current_heading() - 90.0f;
    const FieldObj* gs[2] = {&fieldData.front_goal, &fieldData.back_goal};
    for (int k = 0; k < 2; k++) {
      const FieldObj& g = *gs[k];
      if (!g.seen || g.dist == PKT_NONE) continue;
      float a = (g.angle + turn) * DEG_TO_RAD;          // field frame
      cx += -g.dist * cosf(a);
      cy += (k == 0 ? CAM_GOAL_Y : -CAM_GOAL_Y) - g.dist * sinf(a);
      n++;
    }
  }
  posXOk = usX || n > 0;
  posYOk = usY || n > 0;
  posX = usX ? (L - R) / 2.0f : (n ? cx / n : 0);
  posY = usY ? (B - F) / 2.0f : (n ? cy / n : 0);
  if (posYOk) {
    if (posY > HALF_HYST) inFront = true;
    else if (posY < -HALF_HYST) inFront = false;
  }
}
static Vec2 goHome() {                 // estimatePos() ran this loop
  if (!posYOk || !inFront) { mode = M3_NONE; return Vec2(); }      // back half / unknown: stop
  float ex = posXOk ? -posX : 0, ey = MID_Y_OFFSET - posY;           // field frame
  float d = sqrtf(ex * ex + ey * ey);
  if (d < MID_TOL) { mode = M3_NONE; return Vec2(); }              // there: stop
  float sp = min(MID_SPEED, MID_P * d);
  float turn = (current_heading() - 90.0f) * DEG_TO_RAD;            // field -> robot frame
  float rx = ex * cosf(turn) + ey * sinf(turn), ry = -ex * sinf(turn) + ey * cosf(turn);
  mode = M3_HOME;
  Vec2 out;
  out.x = rx / d * sp;
  out.y = ry / d * sp;
  return out;
}

// ---------- Ball held, no kicker: push it in, turning to the goal as attack5 ----------
static bool aimLocked = false;         // for the print / screen
static float aimLockHeading = 90;
static bool pushShot(Vec2& v) {
  if (!has_ball()) { aimLocked = false; return false; }
  bool goal = camera_fresh(CAM_TIMEOUT_MS) && fieldData.front_goal.seen;
  v = Vec2();
  v.y = goal ? AIM_PUSH_SPEED : NOGOAL_PUSH_SPEED;
  if (!goal) {                                          // goal not seen / lost: heading 90
    aimLocked = false;
    set_aim(false);
    set_target_heading(90.0f);
    return true;
  }
  float a = fmodf(fieldData.front_goal.angle + 360.0f, 360.0f);
  if (!AIM_LOCK_ON) aimLocked = false;
  else if (!aimLocked && (a < AIM_LOCK_LOW || a > AIM_LOCK_HIGH)) {
    aimLocked = true;
    aimLockHeading = current_heading();                 // stay where it is
  } else if (aimLocked && a >= AIM_LOCK_LOW + AIM_LOCK_HYST && a <= AIM_LOCK_HIGH - AIM_LOCK_HYST) {
    aimLocked = false;
  }
  set_aim(!aimLocked);
  set_target_heading(aimLocked ? aimLockHeading : rotate(fieldData));
  return true;
}

// Kicker not used: pins off, ball sensor input (what kicker_begin() did, without charging)
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
    lineData.state |= LINE_IGNORE4;
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
    display.printf("Line %08lX %s\n", lineData.state, lineLock ? "LOCK" : "");
    display.printf("Head %.0f%s Hold %d k%.2f A%.0f\n", current_heading(), aimLocked ? "L" : "", has_ball(), goalK3, theirArea3);
    display.printf("Pos %s%.0f,%s%.0f %s\n", posXOk ? "" : "?", posX, posYOk ? "" : "?", posY, inFront ? "FRONT" : "BACK");
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
    lineData.state |= LINE_IGNORE4;
    if (millis() - lastDraw >= 100) {
      lastDraw = millis();
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.printf("A1129 START mot %s\n", MOTOR_REVERSE ? "REV" : "nor");
      display.printf("Head %.1f\n", current_heading());
      display.printf("Ball %s %.0f s%u\n", irBall.found ? "Y" : "-", irBall.angle, irBall.strength);
      display.printf("Hold %d  no kicker\n", has_ball());
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
  lineData.state |= LINE_IGNORE4;    // broken channels never "line"

  if (picked_up()) {                 // lifted: stop, start again from heading 90
    set_aim(false);
    set_target_heading(90);
    MotorStop();
    return;
  }

  estimatePos();                     // position (no ball: go to the centre; print)
  Vec2 v;
  if (pushShot(v)) {
    mode = M3_SHOT;
  } else {
    set_aim(false);
    set_target_heading(90);
    v = chase3();
    if (mode == M3_NONE) v = goHome();   // no ball: front half -> centre, back half -> stop
  }

  static Vec2 lastV;                 // speed sent last loop (ultrasonic lag)
  orbitSlide(v);                     // orbit blocked by a brake side: slide along the line
  goalSlow(v);                       // near the goals (camera)
  usField(v, lastV);                 // walls (ultrasonic), exponential
  armBrake(v);                       // mux 2: no move toward a side that sees a line
  lineGuard(v);                      // white line (ring): last word
  set_ramp(!lineLock && !armSeen4);  // braking: at once

  if (mode == M3_NONE && !lineLock && !theirInside3) MotorStop();
  else Vector_Motion(v.x, v.y, 0);
  lastV = (mode == M3_NONE && !lineLock && !theirInside3) ? Vec2() : v;

  if (sw2Pressed()) {                // SW2: stop and show the data (see dataScreen)
    float cutAt[4] = {usCut3[US_N_FRONT], usCut3[US_N_RIGHT], usCut3[US_N_BACK], usCut3[US_N_LEFT]};
    dataScreen(modeName3[mode], v, cutAt);
    lastV = Vec2();
  }

  EVERY_MS(tp, 200) {
    Serial.printf("%-6s | ball %s %5.1f s%2u > %4.1f | orbit g %.2f | hold %d%s | pos %s%4.0f,%s%4.0f %s | line %08lX arm %X %s %5.1f | "
                  "us F %3.0f R %3.0f B %3.0f L %3.0f cut F %.1f | goal k %.2f area %4.0f | v (%4.0f, %4.0f) | "
                  "link %lu pk bad %lu junk %lu gap %lu ms\n",
                  slide4 ? "SLIDE" : modeName3[mode], irBall.found ? "Y" : "-", irBall.angle, irBall.strength,
                  strUsed, orbitG, has_ball(), aimLocked ? " LOCK" : "", posXOk ? "" : "?", posX, posYOk ? "" : "?", posY,
                  inFront ? "FRONT" : "BACK", lineData.state, armSeen4,
                  lineOn ? "ON  " : lineCrossed ? "OVER" : lineLock ? "LOCK" : "-   ", lineLock ? lineDirField : -1.0f,
                  usCm3(US_N_FRONT), usCm3(US_N_RIGHT), usCm3(US_N_BACK), usCm3(US_N_LEFT),
                  usCut3[US_N_FRONT], goalK3, theirArea3, v.x, v.y,
                  (unsigned long)(senseLinkStats.pkts - linkPk0), (unsigned long)(senseLinkStats.bad - linkBad0),
                  (unsigned long)(senseLinkStats.junk - linkJunk0), (unsigned long)linkGapMax);   // strength raw > compensated
    linkPk0 = senseLinkStats.pkts; linkBad0 = senseLinkStats.bad; linkJunk0 = senseLinkStats.junk;
    linkGapMax = 0;
  }
}
