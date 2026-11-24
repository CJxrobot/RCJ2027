#include <Arduino.h>
#include <core_act.h>

// T12 (Act): defense robot as a state machine
// Ball: final_ball_data = IR only for now; switch the commented line to use camera + IR.
//
// Rules 2.6 / 2.8: never FULLY inside our penalty area (the line is part of the area).
//
//   DEFEND  back sensor on a line (robot outside the area)                       -> RETURN
//           front ring sensor (ch 4) kept on the penalty line (front of the robot past the line)
//           ball seen:    move sideways onto the ball - goal centre line (ball and goal 180 apart)
//           no ball:      move sideways until the back goal is at 270 (in front of our goal)
//           LEAVE_HOLD_MS (breaks < LEAVE_BREAK_MS ignored) of:
//             ball within 90 +- LEAVE_FRONT_DEG, or
//             robot in place, ball near, within 90 +- LEAVE_SIDE_DEG and still (rule 2.5) -> LEAVE
//   LEAVE   leave the penalty area and take the ball
//           ball held:    shoot_control() (same as the attack robot)
//           ball 85-95:   rush straight at it, else chase_ball()
//           once out and near our goal: never drive back into the area
//           line handling only when the back goal is more than LINE_SIDE_GOAL_DEG off 270
//           (and, once US_Sensor() is filled, a side ultrasonic < LINE_SIDE_US_CM)
//           ball lost / ball behind / too far from our goal / LEAVE_MAX_MS over
//             -> RETURN if it crossed out of the area, else DEFEND
//   RETURN  heading more than RETURN_TURN_DEG off 90: turn first (put back facing our goal)
//           drive toward our goal (camera), go around the ball if it is in the way;
//           goal beside the robot (0 / 180 +- RETURN_SIDE_DEG): forward first, then to the goal
//           a FRONT sensor sees the line                                         -> DEFEND
//
// Every loop, before the states: our goal must be known (camera, or ultrasonic later).
//   unknown -> stop (LEAVE ends);  DEFEND farther than DEFEND_MAX_CM -> RETURN
//
// Before START the OLED shows live data. Lift the robot: motors stop.

// ----- DEFEND: penalty line -----
#define SPEED       30  // forward / backward on the penalty line
#define BACK_SPEED  15  // backing off from a side line
#define FINE_SPEED  15  // small forward / back correction around the ring stop point
#define DEF_SLOW_GOAL_DEG 45.0f // camera: back goal more than this off 270 -> sideways move away from it slows ...
#define DEF_SIDE_GOAL_DEG 65.0f // ... down to 0 here (end of the penalty line)

// ----- DEFEND: sideways -----
#define DEF_SIDE_KP      1.0f  // sideways speed per deg the ball is off the ball - goal line
#define DEF_GOAL_KP      1.0f  // no ball: sideways speed per deg the back goal is off 270
#define DEF_CENTER_DEAD  5.0f  // deg, no sideways move inside this
#define DEF_SIDE_MAX     40.0f // largest sideways speed

// ----- DEFEND -> LEAVE -----
#define LEAVE_FRONT_DEG  5.0f  // ball within 90 +- this counts as "straight ahead" (85-95)
#define LEAVE_HOLD_MS    3000  // ... for this long
#define LEAVE_BREAK_MS   200   // ball out of the window shorter than this (noise): timer keeps running
#define LEAVE_SIDE_DEG   50.0f // robot in place (or stopped at the side), ball near, within 90 +- this ...
#define BALL_STILL_DEG   8.0f  // ... and not moving (angle within this) for LEAVE_HOLD_MS -> LEAVE
                               // (rule 2.5: must touch the ball on the neutral spot next to the area)

// ----- LEAVE -----
#define LEAVE_RUSH_SPEED 70.0f // ball 85-95: straight at it
#define LEAVE_MAX_MS     3000  // longest time out of the penalty area
#define LEAVE_MAX_DIST_CM 90.0f // back goal farther than this: too far, return
#define BALL_LOST_MS     200   // no ball for this long: return
#define BEHIND_DEG       60.0f // ball within 270 +- this: behind the robot, return
#define LINE_SIDE_GOAL_DEG 45.0f // back goal more than this off 270: line handling on
#define LINE_SIDE_US_CM  20.0f // ... and a side ultrasonic closer than this (R03)
#define PENALTY_ZONE_CM  60.0f // our goal closer than this: any line is the penalty line, ignore it
                               // (penalty area reaches ~46 cm from the goal, field side lines >= 78 cm)

// ----- RETURN -----
#define RETURN_SPEED     50.0f
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

// Sensors as lineState bits (ring = mux 1 channel 0-15, mux 2 = channel + 16)
#define M2(ch) (1UL << (16 + (ch)))
#define FRONT_IN  M2(7)
#define FRONT_OUT M2(11)
#define BACK  (M2(4) | M2(3))
#define RING_FRONT (1UL << 4)     // mux 1 ring channel 4 (90 deg, r 6.1 cm)
#define FRONT_ANY (FRONT_IN | FRONT_OUT | RING_FRONT)
#define RING_ALL   0xFFFFUL       // all 16 ring channels

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
static bool defBlocked = false;    // sideways move toward the ball stopped by a side line / camera guard
static bool defSettled = false;    // standing on the ball - goal line (no sideways move needed)

// LEAVE only: back sensors saw the line = robot has crossed out of the area
static bool leaveOut = false;

// RETURN only: robot beside our goal, going forward first (kept until clearly past, no flicker)
static bool returnBeside = false;

// Side line: in sees it -> back off, only mid -> stop there, none -> free (no memory, no timer)
static float sideLimit(const Side& s, uint32_t state) {
  bool mid = seen(state, s.mid), in = seen(state, s.in);
  if (in)  return -BACK_SPEED;
  if (mid) return 0;
  return 1000;
}

// ----- Our goal: checked first every loop, highest priority -----
// camera sees it -> its angle / distance
// else ultrasonic fitted (R03, reserved; US_Sensor() returns 0 for now) -> back distance, angle 270
// else unknown -> robot stops (no blind driving), LEAVE ends
struct OurGoal { bool known; float angle, dist; };

static bool backGoalOk();

// Slant correction factor for the camera goal distance
static float goalDistK(float angle) {
  float off = fabsf(wrap180(angle - 270.0f));
  if (off <= GOAL_K_DEG1) return 1.0f + (GOAL_K1 - 1.0f) * off / GOAL_K_DEG1;
  if (off <= GOAL_K_DEG2) return GOAL_K1 + (GOAL_K2 - GOAL_K1) * (off - GOAL_K_DEG1) / (GOAL_K_DEG2 - GOAL_K_DEG1);
  return GOAL_K2;
}

static OurGoal ourGoal(const UsData& us) {
  OurGoal g = {false, 270.0f, 0.0f};
  if (backGoalOk()) {
    g.known = true;
    g.angle = fieldData.back_goal.angle;
    g.dist = fieldData.back_goal.dist * goalDistK(g.angle);   // corrected, used everywhere
  } else if (us.back > 0) {
    g.known = true;
    g.dist = us.back;
  }
  return g;
}

enum DefState : uint8_t { DEFEND, LEAVE, RETURN };
static const char* stateName[] = {"DEFEND", "LEAVE", "RETURN"};
static DefState st = DEFEND;
static uint32_t stStart = 0, frontStart = 0, frontLastIn = 0, lastBallSeen = 0;
static bool frontTiming = false;
static float frontAngle = 0;     // ball angle when the LEAVE timer started
static const char* why = "";

static void go(DefState s, const char* reason) {
  st = s;
  stStart = millis();
  frontTiming = false;
  defLastMove = 0;                             // DEFEND starts fresh (no old motion)
  defPrevRing = false;
  leaveOut = false;
  returnBeside = false;
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

static Vec2 polarV(float speed, float deg) {
  Vec2 v;
  v.x = speed * cosf(deg * DEG_TO_RAD);
  v.y = speed * sinf(deg * DEG_TO_RAD);
  return v;
}

static bool backGoalOk() {
  return camera_fresh(CAM_TIMEOUT_MS) && fieldData.back_goal.seen;
}

// DEFEND: penalty-line following + sideways toward the ball (or the goal centre)
static Vec2 defendMotion(uint32_t state, const MergedBall& ball, const OurGoal& goal) {
  Vec2 v;

  // Left AND right side sensors see a line: it is the front line under the robot, not the
  // side lines (a side line only reaches one side). Robot is over the line: straight back.
  bool leftSeen  = seen(state, leftSide.mid | leftSide.in);
  bool rightSeen = seen(state, rightSide.mid | rightSide.in);
  if (leftSeen && rightSeen) {
    linePos = LP_OVER;
    defLastMove = v.y = -SPEED;
    return v;
  }

  // Forward / back: front ring sensor on the line (robot partly outside the area, rules 2.6 / 2.8)
  bool ring = seen(state, RING_FRONT), inFront = seen(state, FRONT_IN), out = seen(state, FRONT_OUT);
  if (!ring && defPrevRing && !inFront && !out)                     // ring just lost it
    linePos = defLastMove < 0 ? LP_AHEAD : LP_OVER;
  defPrevRing = ring;
  if (ring)                      v.y = 0;            // right place
  else if (inFront)              v.y = -FINE_SPEED;  // too far forward: back a little
  else if (out)                  v.y = FINE_SPEED;   // too deep: forward a little
  else if (linePos == LP_OVER)   v.y = -FINE_SPEED;  // passed over it: back a little
  else if (linePos == LP_AHEAD)  v.y = FINE_SPEED;   // a bit deep: forward a little
  else if (linePos == LP_AT)     v.y = -FINE_SPEED;  // (memory from another state: back)
  else                           v.y = SPEED;        // unknown: forward, never deeper
  if (v.y != 0) defLastMove = v.y;
  bool atSide = leftSeen || rightSeen;                // at a side line: never back (deeper)
  if (atSide) v.y = max(v.y, 0.0f);

  // Sideways: stand on the line from the ball to our goal centre (covers both goal corners).
  // On that line the ball and our goal are 180 deg apart, seen from the robot.
  // No ball: centre on the goal (goal at 270).
  float err = ball.src != SRC_NONE ? wrap180(ball.angle - (goal.angle + 180.0f))
                                   : -wrap180(goal.angle - 270.0f);
  float off = -err * (ball.src != SRC_NONE ? DEF_SIDE_KP : DEF_GOAL_KP);   // err < 0 -> move right
  defSettled = fabsf(err) <= DEF_CENTER_DEAD;
  v.x = defSettled ? 0 : constrain(off, -DEF_SIDE_MAX, DEF_SIDE_MAX);
  float wantX = v.x;

  float rightLim = sideLimit(rightSide, state), leftLim = sideLimit(leftSide, state);
  v.x = max(min(v.x, rightLim), -leftLim);           // side lines always win

  // Camera guard: goal far off to one side = robot near the end of the penalty line,
  // do not move further away from the goal (replaces the old 300 ms memory)
  // Camera angle slow-down: goal more than DEF_SLOW_GOAL_DEG off 270 = robot near the end of
  // the penalty line. Moving further away from the goal: speed falls linearly to 0 at
  // DEF_SIDE_GOAL_DEG (the line sensors are only the last safety).
  float g = wrap180(goal.angle - 270.0f);            // robot right of the goal -> g < 0
  float k = constrain((DEF_SIDE_GOAL_DEG - fabsf(g)) / (DEF_SIDE_GOAL_DEG - DEF_SLOW_GOAL_DEG), 0.0f, 1.0f);
  if ((g < 0 && v.x > 0) || (g > 0 && v.x < 0)) v.x *= k;
  defBlocked = (wantX > 0 && v.x <= 0) || (wantX < 0 && v.x >= 0);

  // Camera depth floor (rules 2.6 / 2.8): the sensors can be on a side line / curve while the
  // robot is deeper than the front line allows (e.g. at the corner of the area). Deeper than
  // DEPTH_FLOOR_CM behind the FRONT line: come forward first, no sliding toward the middle
  // (there the whole robot would be inside the area).
  float gx = -goal.dist * cosf(goal.angle * DEG_TO_RAD);     // robot x from the goal centre
  float gy = -goal.dist * sinf(goal.angle * DEG_TO_RAD);     // robot y from the goal line
  if (gy < PA_FRONT_CM - DEPTH_FLOOR_CM) {
    v.y = max(v.y, (float)SPEED);
    if ((gx > 0 && v.x < 0) || (gx < 0 && v.x > 0)) v.x = 0;
  }
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
static bool leaveLineOn(const UsData& us, const OurGoal& goal) {
  if (!backGoalOk()) return true;                    // no goal: stay safe
  if (goal.dist < PENALTY_ZONE_CM) return false;     // only the penalty line here: go out
  bool sideGoal = fabsf(wrap180(fieldData.back_goal.angle - 270.0f)) > LINE_SIDE_GOAL_DEG;
  bool usFitted = us.left > 0 || us.right > 0;       // R03 not in yet: goal angle only
  if (!usFitted) return sideGoal;
  bool nearWall = (us.left > 0 && us.left < LINE_SIDE_US_CM) || (us.right > 0 && us.right < LINE_SIDE_US_CM);
  return sideGoal && nearWall;
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
      display.printf("Back goal %s %.0f %u\n", fieldData.back_goal.seen ? "Y" : "-",
                     fieldData.back_goal.angle, fieldData.back_goal.dist);
      display.printf("Line %d Hold %d\n", line_flag(), has_ball());
      display.display();
    }
  }
  drawMessage("GO");
  delay(200);
  go(DEFEND, "start");
}

void loop() {
  static UsData us;
  static uint32_t lastUs = 0;

  update_gyro_sensor();
  read_camera();
  uint8_t got = read_sense();
  kicker_update();
  if (millis() - lastUs >= US_PERIOD_MS) { lastUs = millis(); us = US_Sensor(); }

  if (picked_up()) { set_aim(false); set_target_heading(90); MotorStop(); return; }

  uint32_t now = millis();
  uint32_t state = line_flag() ? lineData.state : 0xFFFFFFFF;
  updateLinePos(state);                // every state: remember where the penalty line is
  MergedBall final_ball_data = ir_ball_data(irBall);
  //MergedBall final_ball_data = merge_ball_data(fieldData, irBall);
  MergedBall& ball = final_ball_data;
  if (ball.src != SRC_NONE || has_ball()) lastBallSeen = now;

  Vec2 v;
  set_aim(false);
  set_target_heading(90);
  set_ramp(st != DEFEND);            // DEFEND line following reacts at once; LEAVE / RETURN ramp

  // Highest priority, every state: where is our goal?
  OurGoal goal = ourGoal(us);
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
      // LEAVE timer: ball straight ahead, or (rule 2.5) robot stopped at the side of the area
      // with the ball not moving ahead of it (e.g. ball on the neutral spot next to the area)
      bool seenBall = ball.src != SRC_NONE;
      float off90 = fabsf(wrap180(ball.angle - 90.0f));
      bool front = seenBall && off90 <= LEAVE_FRONT_DEG;
      bool side  = seenBall && (defBlocked || defSettled) && off90 <= LEAVE_SIDE_DEG &&
                   ball.closeness >= 1.0f;                    // near (IR strength >= IR_NEAR_STR)
      if (front || side) {
        if (!frontTiming) { frontTiming = true; frontStart = now; frontAngle = ball.angle; }
        else if (!front && fabsf(wrap180(ball.angle - frontAngle)) > BALL_STILL_DEG) {
          frontStart = now; frontAngle = ball.angle;           // ball moving: start again
        }
        frontLastIn = now;
      } else if (frontTiming && now - frontLastIn > LEAVE_BREAK_MS) {
        frontTiming = false;                       // out of the window too long: start again
      }
      if (frontTiming && now - frontStart >= LEAVE_HOLD_MS) {
        go(LEAVE, front ? "ball in front 3 s" : "ball beside, still 3 s"); break;
      }
      // Line sensors only mean something when the robot faces 90: turned -> turn back first
      if (!headingOk()) break;
      // Line ONLY behind (back sensor, no front sensor): robot is outside -> camera brings it back.
      // Back AND front on lines (corner, curve): stay in DEFEND, the front sensors decide.
      if (seen(state, BACK) && !seen(state, FRONT_ANY)) {
        if (areaDist(goal) > 0) { go(RETURN, "back sensor on the line"); break; }
        linePos = LP_UNKNOWN;          // camera: robot inside = goal / side line behind it -> forward
      }
      v = defendMotion(state, ball, goal);
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
      if (shot.active) {
        v = shot.v;
        set_aim(shot.aiming);
        set_target_heading(shot.heading);
      } else if (ball.src != SRC_NONE && fabsf(wrap180(ball.angle - 90.0f)) <= LEAVE_FRONT_DEG) {
        v = polarV(LEAVE_RUSH_SPEED, ball.angle);
      } else {
        v = chase_ball(ball, fieldData);
      }

      if (seen(state, BACK)) leaveOut = true;           // robot has crossed out of the area
      // Close to our goal after leaving: never drive back into the area (rules 2.6 / 2.8).
      // Block the move toward any arm that sees a line.
      if (leaveOut && goal.dist < PENALTY_ZONE_CM) {
        if (seen(state, BACK))                             v.y = max(v.y, 0.0f);
        if (seen(state, FRONT_IN | FRONT_OUT))             v.y = min(v.y, 0.0f);
        if (seen(state, rightSide.mid | rightSide.in))     v.x = min(v.x, 0.0f);
        if (seen(state, leftSide.mid | leftSide.in))       v.x = max(v.x, 0.0f);
      }

      slowToSideLines(v, goal);                         // camera: slow down before a side line

      if (leaveLineOn(us, goal)) {
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
      // Coming in: a back sensor or the ring is on a line -> slow, so it stops on the line
      // and does not slide fully into the area (rules 2.6 / 2.8)
      bool onLine = seen(state, BACK | RING_ALL);
      v = polarV(onLine ? FINE_SPEED : RETURN_SPEED, dir);
      slowToSideLines(v, goal);                         // camera: slow down before a side line
      break;
    }
  }

  Vector_Motion(v.x, v.y, 0);

  EVERY_MS(tp, 200) {
    Serial.printf("%-6s (%s) | ball %-4s %5.1f | goal %s %5.1f %4u cm -> %3.0f cm | line %d | hold %d | v (%4.0f, %4.0f)\n",
                  stateName[st], why, ball_src_name(ball.src), ball.angle,
                  fieldData.back_goal.seen ? "Y" : "-", fieldData.back_goal.angle, fieldData.back_goal.dist,
                  goal.dist, line_flag(), has_ball(), v.x, v.y);       // raw -> corrected
  }
}
