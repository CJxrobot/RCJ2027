#include <Arduino.h>
#include <core_act.h>

// Motor direction of this robot body: 1 = attack body (wired the other way), 0 = defense body
#define MOTOR_REVERSE 0

// T12b (Act): defense robot as a state machine -- act_defense_smach + mux 1 whole-ring guard
// Same as act_defense_smach.cpp, plus (DEFEND only, no memory): the WHOLE mux 1 ring (16 channels,
// 6.1 cm, inside every arm sensor) must never go over a line. Any ring channel on a line, except
// the front ones that ride the penalty line (ch RING_FREE_FROM..RING_FREE_TO), pushes the robot
// away from it: at least BACK_SPEED along the sum of "away" directions (never backward: deeper).
//   ring left AND right side on a line -> it is the front line under the robot -> straight back
// LEAVE and RETURN are not changed by the ring guard.
// Camera depth floor only while no front sensor is on a line (2026-11-25).
// Collisions (2026-11-24): corner lock released when the camera says the robot was pushed back
// toward our goal; ring lost while standing still -> the camera says which way it was moved.
// Also (2026-11-24): LEAVE up to 4 s; every move at least MIN_SPEED (15 %); back sensor on a line
// -> camera decides front line (RETURN) / deep (forward) with BACK_LINE_SPLIT_CM; RETURN backward
// is never cut below FINE_SPEED by the back ultrasonic while the camera says outside the line.
// Ball: final_ball_data = IR only for now; switch the commented line to use camera + IR.
//
// Rules 2.6 / 2.8: never FULLY inside our penalty area (the line is part of the area).
//
//   DEFEND  back sensor on a line (robot outside the area)                       -> RETURN
//           front ring sensor (ch 4) kept on the penalty line (front of the robot past the line)
//           ball seen:    move sideways onto the ball - goal centre line (ball and goal 180 apart)
//           no ball:      move sideways until the back goal is at 270 (in front of our goal)
//           corner: front sensor + one side arm on lines, facing 90 -> that side LOCKED: no
//             sideways / backward move, motors cut (MotorStop) if nothing is left; an in sensor on
//             the line moves it back in; unlocked when it wants to go the other way
//           pushed back: line went ring -> F_out while not backing, or back ultrasonic under its
//             lower limit -> forward 30 % rising to 60 % until the ring sees the line again
//           (breaks < LEAVE_BREAK_MS ignored)
//             ball within 90 +- LEAVE_FRONT_DEG for LEAVE_HOLD_MS (3 s), or
//             robot at a corner of the area, ball 40-80 (right) / 100-140 (left) for
//             DEF_CORNER_HOLD_MS (4 s) (rule 2.5: ball on the neutral spot)          -> LEAVE
//   LEAVE   leave the penalty area and take the ball
//           ball held:    shoot_control() (same as the attack robot)
//           ball 85-95:   rush straight at it; in the area chase_ball(), out of it a slow orbit
//           once out and near our goal: never drive back into the area
//           line handling only when the back goal is more than LINE_SIDE_GOAL_DEG off 270
//           ball lost / ball behind / too far from our goal / LEAVE_MAX_MS over
//             -> RETURN if it crossed out of the area, else DEFEND
//   RETURN  heading more than RETURN_TURN_DEG off 90: turn first (put back facing our goal)
//           drive toward our goal (camera), go around the ball if it is in the way;
//           goal beside the robot (0 / 180 +- RETURN_SIDE_DEG): forward first, then to the goal
//           a FRONT sensor sees the line                                         -> DEFEND
//
// Every loop, before the states: our goal must be known (camera).
//   unknown -> stop (LEAVE ends);  DEFEND farther than DEFEND_MAX_CM -> RETURN
//
// Ultrasonic (R03, from Sense in the AB packet): second safety layer, the camera decides first.
//   LEAVE / RETURN: speed toward a side wall / backward falls to 0 near it
//   RETURN: back ultrasonic lower limit (never backward below it) + slow-down before it
//   DEFEND: never backward below the back lower limit; below it = pushed in -> forward
//   Each sensor has its own switch (US_USE_*). Switched off / no reading (0) / no packet for
//   US_STALE_MS = failed: speed toward that side x US_FAIL_K (slower, the camera alone guards it)
//
// Before START the OLED shows live data (also the 4 ultrasonic: put the robot on its defending
// spot and write down B for US_BACK_GOAL_CM / US_BACK_WALL_CM). Lift the robot: motors stop.

// ----- DEFEND: penalty line -----
#define SPEED       30  // forward / backward on the penalty line
#define BACK_SPEED  15  // backing off from a side line
#define FINE_SPEED  15  // small forward / back correction around the ring stop point
#define DEF_SLOW_GOAL_DEG 35.0f // camera: back goal more than this off 270 -> sideways move away from it slows ...
#define DEF_SIDE_GOAL_DEG 60.0f // ... down to 0 here (end of the penalty line)
#define LOCK_HEAD_DEG     10.0f // corner lock only facing 90 +- this (like the reference program)
#define LOCK_PUSH_DEG     5.0f  // locked robot moved this much back toward our goal (camera angle) =
                                // pushed in by an opponent -> unlock (field 2026-11-24: stuck inside)
// Pushed back into the area (opponent): forward, speed rising (reference program)
// Smallest move: a speed above 0 but under this does not move the real robot (stuck on the
// field 2026-11-24). Any move slower than this (not 0) is raised to it, keeping its direction.
#define MIN_SPEED         15.0f
// Back sensor on a line, no front sensor: the camera says which line it is.
// Robot centre more than this outside the penalty line -> it is the front line (robot outside)
// -> RETURN; else it is the goal / side line behind a deep robot -> forward.
#define BACK_LINE_SPLIT_CM 3.0f
#define RECOVER_SPEED     30.0f // start
#define RECOVER_MAX       60.0f // most
#define RECOVER_RAMP      0.01f // % per ms (30 -> 60 in 3 s)

// ----- DEFEND: sideways -----
#define DEF_SIDE_KP      1.0f  // sideways speed per deg the ball is off the ball - goal line
#define DEF_GOAL_KP      1.0f  // no ball: sideways speed per deg the back goal is off 270
#define DEF_CENTER_DEAD  5.0f  // deg, no sideways move inside this
#define DEF_SIDE_MAX     40.0f // largest sideways speed

// ----- DEFEND -> LEAVE -----
#define LEAVE_FRONT_DEG  5.0f  // ball within 90 +- this counts as "straight ahead" (85-95)
#define LEAVE_HOLD_MS    3000  // ... for this long
#define LEAVE_BREAK_MS   200   // ball out of the window shorter than this (noise): timer keeps running
// Robot at a corner of the area: our goal more than DEF_CORNER_GOAL_DEG off 270. Ball on the
// neutral spot seen at ~60 (right corner) / ~126 (left corner), field test 2026-11-24.
#define DEF_CORNER_GOAL_DEG  25.0f
#define DEF_CORNER_R_MIN     40.0f // right corner: ball between these angles ...
#define DEF_CORNER_R_MAX     80.0f
#define DEF_CORNER_L_MIN     100.0f // left corner
#define DEF_CORNER_L_MAX     140.0f
#define DEF_CORNER_HOLD_MS   4000  // ... for this long -> LEAVE

// ----- LEAVE -----
#define LEAVE_RUSH_SPEED 70.0f // ball 85-95: straight at it
#define ORBIT_SPEED      25.0f // LEAVE, out of the area: slow circle around the ball to get it to 90
#define LEAVE_FAR_SPEED  60.0f // LEAVE speed limit with the ball far (IR closeness 0) ...
#define LEAVE_NEAR_SPEED 30.0f // ... down to this with the ball close (closeness 1): no running over it
#define LEAVE_MAX_MS     4000  // longest time out of the penalty area
#define LEAVE_MAX_DIST_CM 90.0f // back goal farther than this: too far, return
#define BALL_LOST_MS     200   // no ball for this long: return
#define BEHIND_DEG       60.0f // ball within 270 +- this: behind the robot, return
#define LINE_SIDE_GOAL_DEG 45.0f // back goal more than this off 270: line handling on
#define PENALTY_ZONE_CM  60.0f // our goal closer than this: any line is the penalty line, ignore it
                               // (penalty area reaches ~46 cm from the goal, field side lines >= 78 cm)

// ----- RETURN -----
#define RETURN_SPEED     40.0f // top speed (50 was too fast on the field)
// PD slow-down toward the stop point (ring on the line, centre RING_STOP_CM inside the line),
// distance from the camera (corrected). speed = KP * cm to go + KD * (cm/s, < 0 while closing)
// clamped to FINE_SPEED .. RETURN_SPEED. No I term: it would push past the stop point.
#define RETURN_KP        0.7f   // % per cm to go   (57 cm -> 40 %, 21 cm -> 15 %)
#define RETURN_KD        0.2f   // % per cm/s       (closing at 100 cm/s -> -20 %)
#define RING_STOP_CM     6.0f   // ring sensor 6.1 cm ahead of the centre
#define RETURN_SIDE_DEG  30.0f // our goal within 0 / 180 +- this (robot beside the goal): go forward first
#define RETURN_SIDE_HYST 10.0f // ... until the goal is more than RETURN_SIDE_DEG + this away from 0 / 180
#define DODGE_DEG        30.0f // ball within this of the goal direction: go around it
#define DODGE_OFFSET_DEG 60.0f // ... by turning the path this much, on the field side (toward 90)
#define DEFEND_MAX_CM    45.0f // DEFEND: our goal farther than this = pushed out of the area -> RETURN
                               // (defending spots 16-33 cm, far corner of the area ~46 cm)
// ----- Camera goal distance correction (measured on the field 2026-11-24) -----
// The goal distance reads too long when the goal is seen at a slant. Multiply by K, linear
// between these points (deg off 270 -> K):  0 -> 1.00 (45 read 45), 44 -> 0.81 (corner: read 37,
// real ~30), 66+ -> 0.61 (beside the goal: read 90, real ~55). Far straight ahead reads a bit
// short (centre of the field: read 90, real ~110): not corrected.
#define GOAL_K_DEG1      44.0f
#define GOAL_K1          0.81f
#define GOAL_K_DEG2      66.0f
#define GOAL_K2          0.61f
// The camera's goal point is ~5 cm inside the goal (defending spot reads 24, not 19), so the
// penalty area below is measured from that point.

// Penalty area from the field spec (cm, from the camera's goal point, line centres)
#define PA_FRONT_CM      30.0f // front line (25 from the goal line + 5 to the camera's goal point)
#define PA_HALF_CM       39.0f // side lines
#define PA_R_CM          15.0f // rounded front corners
#define DEPTH_FLOOR_CM   9.5f  // camera: robot centre deeper than this behind the front line -> forward first
                               // (normal spot ~6 cm, fully inside from 10 cm: front 11 cm vs line 1 cm)
// Field side lines (LEAVE / RETURN): robot x from the camera goal ANGLES (both goals + gyro),
// speed toward a side line cut exponentially (same law as the library's walls / goals)
#define FIELD_HALF_LEN_CM 114.5f // camera's goal point from the field centre (109.5 + 5)
#define FIELD_HALF_W_CM   78.0f  // side line (centre) from the field centre
#define SIDE_STOP_CM      14.0f  // robot centre this far from the side line: no more speed toward it
                                 // (robot 11 + half line 1 + 2 margin)
#define SIDE_LAMBDA_CM    20.0f  // slow-down length (stops ~64 cm from the centre at chase speed in the sim)
#define RETURN_TURN_DEG  20.0f // DEFEND / RETURN: heading more than this off 90: stop and turn first

// ----- Ultrasonic (R03): second layer after the camera -----
// Switch a sensor off (0) if its hardware is broken: it is then never used and counts as failed
#define US_USE_FRONT     1     // H1, 90  (not used for motion now: the front wall is never near)
#define US_USE_RIGHT     1     // H2, 0
#define US_USE_BACK      1     // H3, 270
#define US_USE_LEFT      1     // H4, 180
#define US_STALE_MS      200   // no AB packet for this long = every sensor failed
#define US_FAIL_K        0.7f  // failed sensor: speed toward its side x this
#define US_WALL_STOP_CM  15.0f // side sensor to a wall: no speed toward it (robot edge ~3 cm inside the line)
#define US_WALL_KP       1.5f  // speed allowed toward a wall: % per cm past the stop (35 cm -> 30 %)
// Back sensor at the defending spot (ring on the penalty line), MEASURE with test_us / the OLED:
#define US_BACK_GOAL_CM  14.0f // in front of our goal: sees the goal's back wall (estimate)
#define US_BACK_WALL_CM  20.0f // beside the goal (end of the penalty line): sees the field wall (estimate)
#define US_BACK_MARGIN_CM 2.0f // lower limit = spot - this (fully inside the area ~4 cm deeper than the spot)
#define US_BACK_KP       1.5f  // RETURN / LEAVE backward speed: % per cm above the lower limit
#define GOAL_HALF_CM     30.0f // robot x within this (camera): back sensor sees the goal, else the field wall
#define US_BALL_SKIP_DEG 30.0f // ball close and within this of a sensor: it may be reading the ball, skip it

// Sensors as lineState bits (ring = mux 1 channel 0-15, mux 2 = channel + 16)
#define M2(ch) (1UL << (16 + (ch)))
#define FRONT_IN  M2(7)
#define FRONT_OUT M2(11)
#define BACK  (M2(4) | M2(3))
#define RING_FRONT (1UL << 4)     // mux 1 ring channel 4 (90 deg, r 6.1 cm)
#define FRONT_ANY (FRONT_IN | FRONT_OUT | RING_FRONT)
#define RING_ALL   0xFFFFUL       // all 16 ring channels

#define RING_RIGHT ((1UL << 15) | (1UL << 0) | (1UL << 1))   // mux 1 ring, -22.5 .. 22.5 deg
#define RING_LEFT  ((1UL << 7) | (1UL << 8) | (1UL << 9))    // mux 1 ring, 157.5 .. 202.5 deg
#define RING_FREE_FROM 2   // ring channels 2..6 (45 .. 135 deg) may sit on the penalty line ...
#define RING_FREE_TO   6   // ... (ch 4 is the stop point), every other one is a guard

struct Side { uint32_t mid, in; };
static const Side rightSide = {M2(14), M2(13)};
static const Side leftSide  = {M2(1),  M2(0)};

static bool seen(uint32_t state, uint32_t mask) { return (~state & mask) != 0; }

// Rules 2.6 / 2.8: the robot must never be FULLY inside the penalty area (the line is part of
// the area). Front sensors from the centre: F_in 4.4 cm, ring ch 4 6.1 cm, F_out 8.0 cm.
// Target: ring ch 4 on the line -> centre ~6 cm behind it, front of the robot (11 cm) about
// 3-5 cm past the outer edge of the line. F_in on the line = too far forward, F_out only = too deep.
//   LP_AT     ring saw the line last
//   LP_AHEAD  line ahead of the ring (F_out saw it, or the ring lost it while backing) -> forward
//   LP_OVER   line behind the ring (F_in saw it, or the ring lost it moving forward)  -> back
//   unknown -> forward (never deeper into the area)
enum LinePos : uint8_t { LP_UNKNOWN, LP_AT, LP_AHEAD, LP_OVER };
static LinePos linePos = LP_UNKNOWN;

static void updateLinePos(uint32_t state) {
  if (seen(state, RING_FRONT))     linePos = LP_AT;
  else if (seen(state, FRONT_IN))  linePos = LP_OVER;
  else if (seen(state, FRONT_OUT)) linePos = LP_AHEAD;
}

// DEFEND only: last forward / back move (not 0) and whether the ring saw the line last loop
static float defLastMove = 0;
static bool defPrevRing = false;
static float defPrevVy = 0;           // forward / back speed asked last loop

// DEFEND only: corner lock per side, pushed-back recovery
static bool lockR = false, lockL = false;
static float lockGoalOff = 0;        // largest |goal angle - 270| while locked (camera)
static uint8_t pushSeq = 0;          // 1 = ring had the line, 2 = then F_out had it (line moving out)
static bool recovering = false;
static uint32_t recoverStart = 0;

// LEAVE only: back sensors saw the line and then left it = robot fully out of the area
static bool leaveBackSeen = false, leaveOut = false;

// RETURN only: robot beside our goal, going forward first (kept until clearly past, no flicker)
static bool returnBeside = false;
// RETURN only: PD slow-down state (cm to go, its rate from camera updates)
static float retLastE = 0, retRate = 0;
static uint32_t retLastT = 0;

// Side line: in sees it -> back off, only mid -> stop there, none -> free (no memory, no timer)
static float sideLimit(const Side& s, uint32_t state) {
  bool mid = seen(state, s.mid), in = seen(state, s.in);
  if (in)  return -BACK_SPEED;
  if (mid) return 0;
  return 1000;
}

// ----- Our goal: checked first every loop, highest priority -----
// camera sees it -> its angle / distance
// else unknown -> robot stops (no blind driving), LEAVE ends. The ultrasonic is not used here:
// the back one sees the goal OR the field wall, it cannot say where the goal is.
struct OurGoal { bool known; float angle, dist; };

static bool backGoalOk();

// Slant correction factor for the camera goal distance
static float goalDistK(float angle) {
  float off = fabsf(wrap180(angle - 270.0f));
  if (off <= GOAL_K_DEG1) return 1.0f + (GOAL_K1 - 1.0f) * off / GOAL_K_DEG1;
  if (off <= GOAL_K_DEG2) return GOAL_K1 + (GOAL_K2 - GOAL_K1) * (off - GOAL_K_DEG1) / (GOAL_K_DEG2 - GOAL_K_DEG1);
  return GOAL_K2;
}

static OurGoal ourGoal() {
  OurGoal g = {false, 270.0f, 0.0f};
  if (backGoalOk()) {
    g.known = true;
    g.angle = fieldData.back_goal.angle;
    g.dist = fieldData.back_goal.dist * goalDistK(g.angle);   // corrected, used everywhere
  }
  return g;
}

// ----- Ultrasonic -----
static const bool usUse[4] = {US_USE_FRONT, US_USE_RIGHT, US_USE_BACK, US_USE_LEFT};
static const float usDir[4] = {90.0f, 0.0f, 270.0f, 180.0f};    // front, right, back, left

// cm, or 0 = failed (switched off, no reading, or no packet for US_STALE_MS)
static float usCm(int i) {
  if (!usUse[i] || senseUs.t_ms == 0 || millis() - senseUs.t_ms > US_STALE_MS) return 0;
  return senseUs.cm[i];
}

// Back sensor lower limit: what it reads at the defending spot minus a margin.
// Robot in front of the goal (camera x): it sees the goal's back wall, else the field wall.
static float usBackMin(const OurGoal& goal) {
  float gx = -goal.dist * cosf(goal.angle * DEG_TO_RAD);
  return (fabsf(gx) < GOAL_HALF_CM ? US_BACK_GOAL_CM : US_BACK_WALL_CM) - US_BACK_MARGIN_CM;
}

enum DefState : uint8_t { DEFEND, LEAVE, RETURN };
static const char* stateName[] = {"DEFEND", "LEAVE", "RETURN"};
static DefState st = DEFEND;
static uint32_t stStart = 0, frontStart = 0, frontLastIn = 0, lastBallSeen = 0;
static bool frontTiming = false;
static bool timingCorner = false; // the running LEAVE timer is the corner one (4 s)
static const char* why = "";

static void go(DefState s, const char* reason) {
  st = s;
  stStart = millis();
  frontTiming = false;
  defLastMove = defPrevVy = 0;                 // DEFEND starts fresh (no old motion)
  defPrevRing = false;
  leaveOut = leaveBackSeen = false;
  returnBeside = false;
  retLastT = 0;
  retRate = 0;
  lockR = lockL = false;
  pushSeq = 0;
  recovering = false;
  if (s == RETURN) linePos = LP_UNKNOWN;       // old line memory may be wrong (pushed): find it again
  why = reason;
  Serial.printf("%lu ms: -> %s (%s)\n", millis(), stateName[s], reason);
}

// End of LEAVE: crossed out of the area -> RETURN; still in it -> DEFEND (RETURN would drive
// toward our goal = deeper into the area, rules 2.6 / 2.8)
static void leaveEnd(const char* reason) {
  if (leaveOut) go(RETURN, reason);
  else          go(DEFEND, "still in the area");
}

// Facing 90 (within RETURN_TURN_DEG)? DEFEND / RETURN line logic assumes it
static bool headingOk() { return fabsf(wrap180(current_heading() - 90.0f)) <= RETURN_TURN_DEG; }

// Where is the robot centre from the penalty line? Camera goal angle + distance -> robot position
// in front of our goal (heading 90). Returns cm outside the line (< 0 = inside the area).
// Tells a back sensor on the FRONT line (robot outside) from one on the goal / side line (deep).
static float areaDist(const OurGoal& g) {
  float x = -g.dist * cosf(g.angle * DEG_TO_RAD);     // + = right of the goal centre
  float y = -g.dist * sinf(g.angle * DEG_TO_RAD);     // + = in front of the goal line
  float qx = fabsf(x), cx = PA_HALF_CM - PA_R_CM, cy = PA_FRONT_CM - PA_R_CM;
  if (qx > cx && y > cy) return hypotf(qx - cx, y - cy) - PA_R_CM;   // rounded corner
  return max(qx - PA_HALF_CM, y - PA_FRONT_CM);
}

static Vec2 polarV(float speed, float deg);

// LEAVE, after leaving the area: circle around the IR ball until it is straight ahead (90),
// then push. Same thresholds as the library's chase_ball() orbit (which only runs for a camera
// ball; this robot uses IR only), but slow (ORBIT_SPEED).
static Vec2 orbitBall(const MergedBall& ball) {
  if (ball.src == SRC_NONE) return Vec2();
  float a = fmodf(ball.angle + 360.0f, 360.0f);
  float off = fabsf(wrap180(a - 90.0f));
  if (ball.strength >= IR_PUSH_STR && off <= PUSH_DEG) return polarV(ORBIT_SPEED, 90.0f);   // push
  float dir = a;
  if (ball.strength > IR_ORBIT_STR && off > CHASE_FRONT_DEG) {
    float ratio = ball.strength >= IR_ORBIT_FULL_STR ? 1.0f
                : expf(-IR_ORBIT_DECAY * (IR_ORBIT_FULL_STR - ball.strength));
    float side = (a > 90.0f + CHASE_FRONT_DEG && a < 270.0f) ? 1.0f : -1.0f;  // around the ball
    dir = a + side * ORBIT_OFFSET_DEG * ratio;
  }
  return polarV(ORBIT_SPEED, dir);
}

static Vec2 polarV(float speed, float deg) {
  Vec2 v;
  v.x = speed * cosf(deg * DEG_TO_RAD);
  v.y = speed * sinf(deg * DEG_TO_RAD);
  return v;
}

static bool backGoalOk() {
  return camera_fresh(CAM_TIMEOUT_MS) && fieldData.back_goal.seen;
}

// Ultrasonic guard on v (robot frame), only the sensors in mask (bit = 1 << US index).
// For each: the part of v toward that sensor's side is limited.
//   failed sensor:  x US_FAIL_K
//   slow = true:    to US_*_KP x (cm - stop), 0 at the stop (back: the lower limit)
//   slow = false:   only the stop (back: no backward below the lower limit)
// minBack: backward never cut below this (RETURN with the camera saying the robot is still outside
// the line: camera first, a wrong back reading must not stop the robot)
static void usGuard(Vec2& v, uint8_t mask, bool slow, const OurGoal& goal, const MergedBall& ball,
                    float minBack = 0) {
  for (int i = 0; i < 4; i++) {
    if (!(mask & (1 << i))) continue;
    float ux = cosf(usDir[i] * DEG_TO_RAD), uy = sinf(usDir[i] * DEG_TO_RAD);
    float into = v.x * ux + v.y * uy;
    if (into <= 0) continue;                                    // moving away / along: keep
    bool ballThere = ball.src != SRC_NONE && ball.strength > IR_ORBIT_STR &&
                     fabsf(wrap180(ball.angle - usDir[i])) < US_BALL_SKIP_DEG;
    if (ballThere) continue;                                    // may be reading the ball
    float d = usCm(i);
    bool back = i == US_N_BACK;
    float stop = back ? usBackMin(goal) : US_WALL_STOP_CM;
    float lim;
    if (d <= 0)    lim = into * US_FAIL_K;
    else if (slow) lim = max(0.0f, (back ? US_BACK_KP : US_WALL_KP) * (d - stop));
    else           lim = d < stop ? 0 : into;
    if (back) lim = max(lim, min(into, minBack));
    if (into > lim) { v.x -= (into - lim) * ux; v.y -= (into - lim) * uy; }
  }
}

// DEFEND: penalty-line following + sideways toward the ball (or the goal centre)
static Vec2 defendMotion(uint32_t state, const MergedBall& ball, const OurGoal& goal) {
  Vec2 v;

  // Left AND right side sensors see a line: it is the front line under the robot, not the
  // side lines (a side line only reaches one side). Robot is over the line: straight back.
  bool leftSeen  = seen(state, leftSide.mid | leftSide.in);
  bool rightSeen = seen(state, rightSide.mid | rightSide.in);
  // Mux 1 ring at the sides (6.1 cm, inside the arms): sees a line only after the arms passed it
  bool ringL = seen(state, RING_LEFT), ringR = seen(state, RING_RIGHT);
  if ((leftSeen && rightSeen) || (ringL && ringR)) {
    linePos = LP_OVER;
    defLastMove = defPrevVy = v.y = -SPEED;
    return v;
  }

  // Forward / back: front ring sensor on the line (robot partly outside the area, rules 2.6 / 2.8)
  bool ring = seen(state, RING_FRONT), inFront = seen(state, FRONT_IN), out = seen(state, FRONT_OUT);
  if (!ring && defPrevRing && !inFront && !out)                     // ring just lost it
    // We were not moving forward / back: an opponent moved us -> the camera says which way
    linePos = defPrevVy != 0 ? (defLastMove < 0 ? LP_AHEAD : LP_OVER)
                             : (areaDist(goal) < -RING_STOP_CM ? LP_AHEAD : LP_OVER);
  defPrevRing = ring;

  // Pushed back (reference program): the line moved ring -> F_out -> past F_out, i.e. the robot
  // went back ~4 cm from its spot; or the back ultrasonic reads under its lower limit while the
  // camera puts the robot centre inside the front line.
  // Forward, speed rising, until the ring sees the line again (or the camera says outside).
  if (ring) pushSeq = 1;
  else if (out && pushSeq == 1) pushSeq = 2;
  if (inFront) pushSeq = 0;
  float usBack = usCm(US_N_BACK);
  bool usDeep = usBack > 0 && usBack < usBackMin(goal) && areaDist(goal) < 0;   // camera: inside the line
  // ring -> F_out while not backing ourselves = pushed (the robot only gets ~0.5 cm more before
  // F_out loses the line and the whole robot is inside: no waiting for that)
  bool pushedLine = pushSeq == 2 && (!out || defPrevVy >= 0);
  if (!recovering && !ring && !inFront && (pushedLine || usDeep)) {
    recovering = true;
    recoverStart = millis();
    pushSeq = 0;
    Serial.printf("%lu ms: pushed back (%s): forward\n", millis(), usDeep ? "back ultrasonic" : "line");
  }
  if (recovering && (ring || inFront || areaDist(goal) > 0)) recovering = false;
  if (recovering) {
    linePos = LP_AHEAD;
    v.y = min(RECOVER_MAX, RECOVER_SPEED + (millis() - recoverStart) * RECOVER_RAMP);
    defLastMove = defPrevVy = v.y;
    return v;
  }
  if (ring)                      v.y = 0;            // right place
  else if (inFront)              v.y = -FINE_SPEED;  // too far forward: back a little
  else if (out)                  v.y = FINE_SPEED;   // too deep: forward a little
  else if (linePos == LP_OVER)   v.y = -FINE_SPEED;  // passed over it: back a little
  else if (linePos == LP_AHEAD)  v.y = FINE_SPEED;   // a bit deep: forward a little
  else if (linePos == LP_AT)     v.y = -FINE_SPEED;  // (memory from another state: back)
  else                           v.y = SPEED;        // unknown: forward, never deeper
  if (v.y != 0) defLastMove = v.y;
  bool atSide = leftSeen || rightSeen || ringL || ringR;   // at a side line: never back (deeper)
  if (atSide) v.y = max(v.y, 0.0f);

  // Sideways: stand on the line from the ball to our goal centre (covers both goal corners).
  // On that line the ball and our goal are 180 deg apart, seen from the robot.
  // No ball: centre on the goal (goal at 270).
  float err = ball.src != SRC_NONE ? wrap180(ball.angle - (goal.angle + 180.0f))
                                   : -wrap180(goal.angle - 270.0f);
  float off = -err * (ball.src != SRC_NONE ? DEF_SIDE_KP : DEF_GOAL_KP);   // err < 0 -> move right
  v.x = fabsf(err) <= DEF_CENTER_DEAD ? 0 : constrain(off, -DEF_SIDE_MAX, DEF_SIDE_MAX);
  float want = v.x;                                   // where the robot wants to go sideways

  // Corner lock (reference program): facing 90, a front sensor and one side arm on lines
  // = corner of the area -> lock that side. Unlocked only when the robot wants to go the
  // other way (moving direction opposite to the locked side).
  // Collision: pushed back toward our goal while locked (camera angle LOCK_PUSH_DEG smaller than
  // the farthest point) -> unlock, the robot goes out to the ball again.
  float goalOff = fabsf(wrap180(goal.angle - 270.0f));
  if (fabsf(wrap180(current_heading() - 90.0f)) <= LOCK_HEAD_DEG && seen(state, FRONT_ANY)) {
    if ((rightSeen && !lockR) || (leftSeen && !lockL)) lockGoalOff = goalOff;
    if (rightSeen) lockR = true;
    if (leftSeen)  lockL = true;
  }
  if (lockR || lockL) {
    lockGoalOff = max(lockGoalOff, goalOff);
    if (goalOff < lockGoalOff - LOCK_PUSH_DEG) {
      lockR = lockL = false;
      Serial.printf("%lu ms: pushed in at the corner: unlock\n", millis());
    }
  }
  if (lockR && want < 0) lockR = false;
  if (lockL && want > 0) lockL = false;

  // Camera angle slow-down: goal more than DEF_SLOW_GOAL_DEG off 270 = robot near the end of
  // the penalty line. Moving further away from the goal: speed falls linearly to 0 at
  // DEF_SIDE_GOAL_DEG. Applied BEFORE the side lines: backing off a side line (in sensor) is a
  // safety move and is never slowed (field 2026-11-24: back-off 15 -> 9 %, robot stuck).
  float g = wrap180(goal.angle - 270.0f);            // robot right of the goal -> g < 0
  float k = constrain((DEF_SIDE_GOAL_DEG - fabsf(g)) / (DEF_SIDE_GOAL_DEG - DEF_SLOW_GOAL_DEG), 0.0f, 1.0f);
  if ((g < 0 && v.x > 0) || (g > 0 && v.x < 0)) v.x *= k;

  float rightLim = sideLimit(rightSide, state), leftLim = sideLimit(leftSide, state);
  v.x = max(min(v.x, rightLim), -leftLim);           // side lines always win

  // Locked: no sideways and no backward move (loop() cuts the motors with MotorStop() when
  // nothing is left), except an in sensor on the side line: the robot slid past, move back in
  // slowly. Forward stays (out of the area: never a rule problem, and it holds against a push).
  if (lockR || lockL) {
    v.x = 0;
    v.y = max(v.y, 0.0f);
    if (lockR && !lockL && seen(state, rightSide.in)) v.x = -BACK_SPEED;
    if (lockL && !lockR && seen(state, leftSide.in))  v.x = BACK_SPEED;
  }

  // Whole-ring guard (last, innermost, no memory): sum the "away" directions of every guard
  // channel on a line; the robot moves at least BACK_SPEED that way, whatever the ball, the
  // camera or the corner lock want. Never backward (deeper into the area): y part >= 0.
  float ax = 0, ay = 0;
  for (int ch = 0; ch < 16; ch++) {
    if (ch >= RING_FREE_FROM && ch <= RING_FREE_TO) continue;
    if (!seen(state, 1UL << ch)) continue;
    ax -= cosf(ch * 22.5f * DEG_TO_RAD);
    ay -= sinf(ch * 22.5f * DEG_TO_RAD);
  }
  ay = max(ay, 0.0f);
  float an = hypotf(ax, ay);
  if (an > 0.01f) {
    ax /= an; ay /= an;
    float along = v.x * ax + v.y * ay;
    if (along < BACK_SPEED) { v.x += (BACK_SPEED - along) * ax; v.y += (BACK_SPEED - along) * ay; }
  }

  // Camera depth floor (rules 2.6 / 2.8): the sensors can be on a side line / curve while the
  // robot is deeper than the front line allows (e.g. at the corner of the area). Deeper than
  // DEPTH_FLOOR_CM behind the FRONT line: come forward first, no sliding toward the middle
  // (there the whole robot would be inside the area).
  // Not while a front sensor (F_in / ring / F_out) is on a line: then the line sensors know the
  // depth better than the camera (field 2026-11-25, smach3: camera 1.5 cm too deep at goal 235
  // -> forward 30 / sideways 0, back and forth). Pushed back: ring -> F_out catches it.
  float gx = -goal.dist * cosf(goal.angle * DEG_TO_RAD);     // robot x from the goal centre
  float gy = -goal.dist * sinf(goal.angle * DEG_TO_RAD);     // robot y from the goal line
  if (!seen(state, FRONT_ANY) && gy < PA_FRONT_CM - DEPTH_FLOOR_CM) {
    v.y = max(v.y, (float)SPEED);
    if ((gx > 0 && v.x < 0) || (gx < 0 && v.x > 0)) v.x = 0;
  }
  defPrevVy = v.y;
  return v;
}

// ----- Field side lines (LEAVE / RETURN) -----
// Robot x on the field (cm, + = right, 0 = centre line between the goals).
// Both goals seen: from their two ANGLES only (no distances), turned into field directions with
// the gyro. Robot on the goal-goal line (angles 180 apart): x = 0.
// Only our goal seen: its angle + distance.
static float fieldX(const OurGoal& goal) {
  float turn = current_heading() - 90.0f;                       // robot frame -> field frame
  float b = (goal.angle + turn) * DEG_TO_RAD;
  const FieldObj& f = fieldData.front_goal;
  if (backGoalOk() && f.seen) {
    float a = (f.angle + turn) * DEG_TO_RAD;
    float det = sinf(a - b);                                    // 0 when the goals are 180 apart
    if (fabsf(det) < 0.05f) return 0;
    return -2.0f * FIELD_HALF_LEN_CM * cosf(b) * cosf(a) / det;
  }
  return -goal.dist * cosf(b);
}

// Cut the part of v (robot frame) that goes toward a side line, more the closer it is
static void slowToSideLines(Vec2& v, const OurGoal& goal) {
  float x = fieldX(goal);
  float turn = current_heading() - 90.0f;
  for (int s = -1; s <= 1; s += 2) {                            // -1 left line, +1 right line
    float d = FIELD_HALF_W_CM - s * x;                          // centre to that side line
    float dir = (s > 0 ? 0.0f : 180.0f) - turn;                 // its direction, robot frame
    float ux = cosf(dir * DEG_TO_RAD), uy = sinf(dir * DEG_TO_RAD);
    float into = v.x * ux + v.y * uy;
    if (into <= 0) continue;                                    // moving away / along: keep
    float k = min(1.0f, expf(-(d - SIDE_STOP_CM) / SIDE_LAMBDA_CM));
    v.x -= k * into * ux;
    v.y -= k * into * uy;
  }
}

// LEAVE: line handling only near the side of the field
static bool leaveLineOn(const OurGoal& goal) {
  if (!backGoalOk()) return true;                    // no goal: stay safe
  if (goal.dist < PENALTY_ZONE_CM) return false;     // only the penalty line here: go out
  return fabsf(wrap180(fieldData.back_goal.angle - 270.0f)) > LINE_SIDE_GOAL_DEG;   // camera first
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

  MotorStop();
  uint32_t lastDraw = 0;
  while (digitalRead(START_BUTTON) == HIGH) {
    update_gyro_sensor();
    read_camera();
    read_sense();
    kicker_update();
    if (millis() - lastDraw >= 100) {
      lastDraw = millis();
      MergedBall b = ir_ball_data(irBall);
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("DEFENSE SM  START");
      display.printf("Head %.1f\n", current_heading());
      display.printf("Ball %s %.0f\n", ball_src_name(b.src), b.angle);
      display.printf("Goal %s %.0f %u>%.0f\n", fieldData.back_goal.seen ? "Y" : "-",
                     fieldData.back_goal.angle, fieldData.back_goal.dist,      // raw > corrected cm
                     fieldData.back_goal.dist * goalDistK(fieldData.back_goal.angle));
      display.printf("Line %d Hold %d\n", line_flag(), has_ball());
      display.print("US");                                  // F R B L cm, -- = failed
      for (int i = 0; i < 4; i++) {
        float d = usCm(i);
        if (d > 0) display.printf(" %.0f", d);
        else       display.print(" --");
      }
      display.println();
      display.printf("US min B %.0f\n", ourGoal().known ? usBackMin(ourGoal()) : 0.0f);
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

  if (picked_up()) { set_aim(false); set_target_heading(90); MotorStop(); return; }

  uint32_t now = millis();
  uint32_t state = line_flag() ? lineData.state : 0xFFFFFFFF;
  updateLinePos(state);                // every state: remember where the penalty line is
  MergedBall final_ball_data = ir_ball_data(irBall);
  //MergedBall final_ball_data = merge_ball_data(fieldData, irBall);
  MergedBall& ball = final_ball_data;
  if (ball.src != SRC_NONE || has_ball()) lastBallSeen = now;

  Vec2 v;
  bool shooting = false;
  set_aim(false);
  set_target_heading(90);
  set_ramp(st != DEFEND);            // DEFEND line following reacts at once; LEAVE / RETURN ramp

  // Highest priority, every state: where is our goal?
  OurGoal goal = ourGoal();
  if (!goal.known) {
    if (st == LEAVE) leaveEnd("goal unknown");
    frontTiming = false;                            // no LEAVE while the goal is unknown
    set_ramp(false);
    Vector_Motion(0, 0, 0);                         // stop, hold heading 90
    EVERY_MS(tu, 200) Serial.printf("%-6s (%s) | goal unknown: stopped\n", stateName[st], why);
    return;
  }
  if (st == DEFEND && goal.dist > DEFEND_MAX_CM) go(RETURN, "pushed out");

  switch (st) {
    case DEFEND: {
      // LEAVE timer: ball straight ahead (3 s), or robot at a corner of the area with the ball
      // in that corner's window (4 s, rule 2.5: ball on the neutral spot next to the area)
      bool seenBall = ball.src != SRC_NONE;
      bool front = seenBall && fabsf(wrap180(ball.angle - 90.0f)) <= LEAVE_FRONT_DEG;
      float g = wrap180(goal.angle - 270.0f);           // robot right of the goal -> g < 0
      float ba = fmodf(ball.angle + 360.0f, 360.0f);
      bool corner = seenBall &&
                    ((g <= -DEF_CORNER_GOAL_DEG && ba >= DEF_CORNER_R_MIN && ba <= DEF_CORNER_R_MAX) ||
                     (g >=  DEF_CORNER_GOAL_DEG && ba >= DEF_CORNER_L_MIN && ba <= DEF_CORNER_L_MAX));
      if (front || corner) {
        bool kind = !front;                            // front has priority
        if (!frontTiming || kind != timingCorner) { frontTiming = true; frontStart = now; timingCorner = kind; }
        frontLastIn = now;
      } else if (frontTiming && now - frontLastIn > LEAVE_BREAK_MS) {
        frontTiming = false;                           // out of the window too long: start again
      }
      if (frontTiming && now - frontStart >= (timingCorner ? DEF_CORNER_HOLD_MS : LEAVE_HOLD_MS)) {
        go(LEAVE, timingCorner ? "ball at the corner 4 s" : "ball in front 3 s"); break;
      }
      // Line sensors only mean something when the robot faces 90: turned -> turn back first
      if (!headingOk()) break;
      // Line ONLY behind (back sensor, no front sensor): robot is outside -> camera brings it back.
      // Back AND front on lines (corner, curve): stay in DEFEND, the front sensors decide.
      if (seen(state, BACK) && !seen(state, FRONT_ANY)) {
        if (areaDist(goal) > BACK_LINE_SPLIT_CM) { go(RETURN, "back sensor on the front line"); break; }
        linePos = LP_UNKNOWN;          // camera: robot inside = goal / side line behind it -> forward
      }
      v = defendMotion(state, ball, goal);
      usGuard(v, 1 << US_N_BACK, false, goal, ball);    // never backward below the lower limit
      break;
    }

    case LEAVE: {
      if (!has_ball()) {
        if (now - lastBallSeen > BALL_LOST_MS)                   { leaveEnd("ball lost"); break; }
        if (ball.src != SRC_NONE && fabsf(wrap180(ball.angle - 270.0f)) < BEHIND_DEG)
                                                                 { leaveEnd("ball behind"); break; }
      }
      if (goal.dist > LEAVE_MAX_DIST_CM)                          { leaveEnd("too far"); break; }
      if (now - stStart > LEAVE_MAX_MS)                          { leaveEnd("time up"); break; }

      ShootCtl shot = shoot_control(fieldData);
      shooting = shot.active;
      if (shot.active) {
        v = shot.v;
        set_aim(shot.aiming);
        set_target_heading(shot.heading);
      } else if (ball.src != SRC_NONE && fabsf(wrap180(ball.angle - 90.0f)) <= LEAVE_FRONT_DEG) {
        v = polarV(LEAVE_RUSH_SPEED, ball.angle);
      } else if (leaveOut) {
        v = orbitBall(ball);                          // out of the area: slow orbit to get the ball to 90
      } else {
        v = chase_ball(ball, fieldData);              // still in the area: straight at the ball (IR)
      }
      if (!shot.active) {                               // slower the closer the ball (IR closeness)
        float lim = LEAVE_FAR_SPEED + (LEAVE_NEAR_SPEED - LEAVE_FAR_SPEED) * ball.closeness;
        float sp = hypotf(v.x, v.y);
        if (sp > lim) { v.x *= lim / sp; v.y *= lim / sp; }
      }

      if (seen(state, BACK)) leaveBackSeen = true;               // back sensors on the line ...
      else if (leaveBackSeen) leaveOut = true;                   // ... and past it: fully out
      // Close to our goal after leaving: never drive back into the area (rules 2.6 / 2.8).
      // Block the move toward any arm that sees a line.
      if (leaveOut && goal.dist < PENALTY_ZONE_CM) {
        v.y = max(v.y, 0.0f);                             // never backward this close to our goal
        if (seen(state, FRONT_IN | FRONT_OUT))             v.y = min(v.y, 0.0f);
        if (seen(state, rightSide.mid | rightSide.in))     v.x = min(v.x, 0.0f);
        if (seen(state, leftSide.mid | leftSide.in))       v.x = max(v.x, 0.0f);
      }

      slowToSideLines(v, goal);                         // camera: slow down before a side line
      usGuard(v, (1 << US_N_RIGHT) | (1 << US_N_LEFT) | (1 << US_N_BACK), true, goal, ball);

      if (leaveLineOn(goal)) {
        LineCtl line = back_to_field(got & SENSE_GOT_LINE);
        if (line.active) { v = line.v; set_aim(false); set_target_heading(90); set_ramp(false); }
      }
      break;
    }

    case RETURN: {
      // Done when a FRONT sensor sees the line (robot coming in from outside: F_in first)
      // Exact opposite of the DEFEND -> RETURN check, so the two can never flip back and forth
      if (seen(state, FRONT_ANY) && !seen(state, BACK) && goal.dist <= DEFEND_MAX_CM) {
        go(DEFEND, "at the line"); break;
      }
      // Put back facing our goal (rule 2.8): turn first, do not drive toward the wall
      if (!headingOk()) break;
      // Camera: robot centre already inside the area (front sensors missed the line) -> DEFEND,
      // which moves forward. Opposite of the DEFEND check (areaDist > 0): no flipping.
      if (areaDist(goal) < -3.0f && goal.dist <= DEFEND_MAX_CM) { go(DEFEND, "camera: inside the area"); break; }

      // Beside our goal (goal near 0 / 180): forward first, then to the goal.
      // Hysteresis: starts inside RETURN_SIDE_DEG, ends past RETURN_SIDE_DEG + RETURN_SIDE_HYST
      float sideOff = min(fabsf(wrap180(goal.angle)), fabsf(wrap180(goal.angle - 180.0f)));
      if (sideOff < RETURN_SIDE_DEG) returnBeside = true;
      else if (sideOff > RETURN_SIDE_DEG + RETURN_SIDE_HYST) returnBeside = false;

      float dir = goal.angle;
      if (returnBeside) {
        dir = 90.0f;
      } else if (ball.src != SRC_NONE) {                       // ball in the way: go around it,
        float d = wrap180(ball.angle - dir);                   // always on the field side (toward
        if (fabsf(d) < DODGE_DEG) {                            // 90), never toward our wall
          float a = dir + DODGE_OFFSET_DEG, b = dir - DODGE_OFFSET_DEG;
          dir = fabsf(wrap180(a - 90.0f)) < fabsf(wrap180(b - 90.0f)) ? a : b;
        }
      }
      // PD slow-down: the camera goal only changes every few frames, so the rate is measured
      // between changes of the distance (smoothed)
      float toGo = areaDist(goal) + RING_STOP_CM;               // cm from the stop point
      if (retLastT == 0) { retLastE = toGo; retLastT = now; }
      else if (toGo != retLastE && now > retLastT) {
        float rate = (toGo - retLastE) * 1000.0f / (now - retLastT);
        retRate = 0.5f * retRate + 0.5f * rate;
        retLastE = toGo;
        retLastT = now;
      }
      float speed = constrain(RETURN_KP * toGo + RETURN_KD * retRate, (float)FINE_SPEED, RETURN_SPEED);
      if (returnBeside) speed = RETURN_SPEED;                   // going forward, away from the goal
      // Coming in: a back sensor or the ring is on a line -> slowest
      if (seen(state, BACK | RING_ALL)) speed = FINE_SPEED;
      v = polarV(speed, dir);
      slowToSideLines(v, goal);                         // camera: slow down before a side line
      // ultrasonic: side walls, and backward slows down to 0 at the back lower limit
      // (never fully inside the area, rules 2.6 / 2.8)
      usGuard(v, (1 << US_N_RIGHT) | (1 << US_N_LEFT) | (1 << US_N_BACK), true, goal, ball,
              areaDist(goal) > 0 ? (float)FINE_SPEED : 0.0f);
      break;
    }
  }

  // Smallest move (not while shooting: the aim needs its own small moves)
  float sp = hypotf(v.x, v.y);
  if (!shooting && sp > 0.5f && sp < MIN_SPEED) { v.x *= MIN_SPEED / sp; v.y *= MIN_SPEED / sp; }

  // Corner locked and nothing to do: motors cut (reference program). Only while still facing
  // 90 +- LOCK_HEAD_DEG: turned (pushed) -> Vector_Motion turns it back first.
  bool facing = fabsf(wrap180(current_heading() - 90.0f)) <= LOCK_HEAD_DEG;
  if (st == DEFEND && (lockR || lockL) && v.x == 0 && v.y == 0 && facing) MotorStop();
  else Vector_Motion(v.x, v.y, 0);

  EVERY_MS(tp, 200) {
    Serial.printf("%-6s (%s) | ball %-4s %5.1f | goal %s %5.1f %4u > %3.0f cm | line %d %08lX | hold %d | "
                  "us R %3.0f B %3.0f L %3.0f | lock %c%c | area %4.0f | v (%4.0f, %4.0f)\n",
                  stateName[st], why, ball_src_name(ball.src), ball.angle,
                  fieldData.back_goal.seen ? "Y" : "-", fieldData.back_goal.angle, fieldData.back_goal.dist,
                  goal.dist, line_flag(), state, has_ball(), usCm(US_N_RIGHT), usCm(US_N_BACK), usCm(US_N_LEFT),
                  lockL ? 'L' : '-', lockR ? 'R' : '-', areaDist(goal), v.x, v.y);       // raw -> corrected
  }
}
