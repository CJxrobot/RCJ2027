#include <Arduino.h>
#include <core_act.h>

// T12 (Act): defense robot as a state machine
// Ball: final_ball_data = IR only for now; switch the commented line to use camera + IR.
//
//   DEFEND  stay BEHIND the penalty line: line kept between F_in and F_out (front sensors)
//           ball seen:    move sideways to keep the ball straight ahead
//           no ball:      move sideways until the back goal is at 270 (in front of our goal)
//           ball within 90 +- LEAVE_FRONT_DEG for LEAVE_HOLD_MS (breaks < LEAVE_BREAK_MS ignored) -> LEAVE
//   LEAVE   leave the penalty area and take the ball
//           ball held:    shoot_control() (same as the attack robot)
//           ball 85-95:   rush straight at it, else chase_ball()
//           line handling only when the back goal is more than LINE_SIDE_GOAL_DEG off 270
//           (and, once US_Sensor() is filled, a side ultrasonic < LINE_SIDE_US_CM)
//           ball lost / ball behind / too far from our goal / LEAVE_MAX_MS over  -> RETURN
//   RETURN  drive toward our goal, go around the ball if it is in the way,
//           line handling on; a FRONT sensor sees the line (robot behind it)    -> DEFEND
//
// Before START the OLED shows live data. Lift the robot: motors stop.

// ----- DEFEND: line following (same as act_defense) -----
#define SPEED       30  // forward / backward on the penalty line
#define BACK_SPEED  15  // backing off from a side line
#define LOST_MS     300 // stop backing if that side sees no line this long

// ----- DEFEND: sideways -----
#define DEF_SIDE_KP      1.0f  // sideways speed per deg the ball is off 90
#define DEF_GOAL_KP      1.0f  // no ball: sideways speed per deg the back goal is off 270
#define DEF_CENTER_DEAD  5.0f  // deg, no sideways move inside this
#define DEF_SIDE_MAX     40.0f // largest sideways speed

// ----- DEFEND -> LEAVE -----
#define LEAVE_FRONT_DEG  5.0f  // ball within 90 +- this counts as "straight ahead" (85-95)
#define LEAVE_HOLD_MS    3000  // ... for this long
#define LEAVE_BREAK_MS   200   // ball out of the window shorter than this (noise): timer keeps running

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
#define DODGE_DEG        30.0f // ball within this of the goal direction: go around it
#define DODGE_OFFSET_DEG 60.0f // ... by turning the path this much away from the ball
#define RETURN_MAX_MS    4000  // safety: give up returning after this, back to DEFEND

// Sensors as lineState bits (mux 2 channel + 16), same as act_defense
#define M2(ch) (1UL << (16 + (ch)))
#define FRONT_IN  M2(7)
#define FRONT_OUT M2(11)
#define BACK  (M2(4) | M2(3))

struct Side { uint32_t mid, in; bool backing; uint32_t lastSeen; };
Side rightSide = {M2(14), M2(13), false, 0};
Side leftSide  = {M2(1),  M2(0),  false, 0};

static bool seen(uint32_t state, uint32_t mask) { return (~state & mask) != 0; }

// Where the penalty line is, remembered from the last mux 2 sensor that saw it.
// Goal: the line stays between F_in and F_out, so the robot is always behind it.
//   LP_NEAR    F_in saw it last:  line just ahead, or it has passed F_in (robot crossing) -> back
//   LP_FAR     F_out saw it last: line ahead out of reach (robot too deep)               -> forward
//   LP_BEHIND  B saw it last:     robot is outside, the line is behind it                -> back
//   LP_BETWEEN line left F_out going forward / F_in going back: it is between them     -> stop
//              (F_in and F_out are further apart than the line is wide: never both at once)
enum LinePos : uint8_t { LP_UNKNOWN, LP_NEAR, LP_FAR, LP_BEHIND, LP_BETWEEN };
static LinePos linePos = LP_UNKNOWN;

static void updateLinePos(uint32_t state) {
  bool in = seen(state, FRONT_IN), out = seen(state, FRONT_OUT), back = seen(state, BACK);
  if (in && !out)           linePos = LP_NEAR;
  else if (out && !in)      linePos = LP_FAR;
  else if (back && !in && !out) linePos = LP_BEHIND;
  // in && out: line between them = right place, keep the last value
}

// DEFEND only: last forward / back command and what the front sensors saw last loop
static float defLastVy = 0;
static bool defPrevIn = false, defPrevOut = false;

static bool behindLine() { return linePos == LP_NEAR || linePos == LP_FAR || linePos == LP_BETWEEN; }

static float sideLimit(Side& s, uint32_t state) {
  bool mid = seen(state, s.mid), in = seen(state, s.in);
  if (mid || in) s.lastSeen = millis();
  if (in) s.backing = true;
  if (mid && !in) s.backing = false;
  if (millis() - s.lastSeen > LOST_MS) s.backing = false;
  if (s.backing) return -BACK_SPEED;
  if (mid)       return 0;
  return 1000;
}

enum DefState : uint8_t { DEFEND, LEAVE, RETURN };
static const char* stateName[] = {"DEFEND", "LEAVE", "RETURN"};
static DefState st = DEFEND;
static uint32_t stStart = 0, frontStart = 0, frontLastIn = 0, lastBallSeen = 0;
static bool frontTiming = false;
static const char* why = "";

static void go(DefState s, const char* reason) {
  st = s;
  stStart = millis();
  frontTiming = false;
  defLastVy = 0;                               // DEFEND starts fresh (no old motion)
  defPrevIn = defPrevOut = false;
  why = reason;
  Serial.printf("%lu ms: -> %s (%s)\n", millis(), stateName[s], reason);
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
static Vec2 defendMotion(uint32_t state, const MergedBall& ball) {
  int back = __builtin_popcount(~state & BACK);
  Vec2 v;
  if (back == 2) return v;                           // vertical line behind: stop

  // Left AND right side sensors see a line: it is the front line under the robot, not the
  // side lines (a side line only reaches one side). Robot is over the line: straight back.
  bool leftSeen  = seen(state, leftSide.mid | leftSide.in);
  bool rightSeen = seen(state, rightSide.mid | rightSide.in);
  if (leftSeen && rightSeen) {
    leftSide.backing = rightSide.backing = false;
    linePos = LP_NEAR;
    v.y = -SPEED;
    return v;
  }

  // Forward / back: keep the penalty line between F_in and F_out (always behind the line)
  // The line just left F_out going forward, or F_in going back: it is in the gap between them
  bool in = seen(state, FRONT_IN), out = seen(state, FRONT_OUT);
  if (!in && !out && ((defPrevOut && defLastVy > 0) || (defPrevIn && defLastVy < 0)))
    linePos = LP_BETWEEN;
  defPrevIn = in;
  defPrevOut = out;
  if (in && out)       v.y = 0;                      // line between them: right place
  else if (in)         v.y = -SPEED;                 // line close: back off
  else if (out)        v.y =  SPEED;                 // line a bit far: come forward
  else if (linePos == LP_BETWEEN) v.y = 0;           // line between F_in and F_out: right place
  else if (linePos == LP_NEAR || linePos == LP_BEHIND) v.y = -SPEED;  // crossed it: go back
  else if (linePos == LP_FAR) v.y = SPEED;           // too deep: come forward
  defLastVy = v.y;
  bool atSide = seen(state, rightSide.mid | rightSide.in | leftSide.mid | leftSide.in) ||
                rightSide.backing || leftSide.backing;
  if (atSide) v.y = max(v.y, 0.0f);

  float off = 0;
  if (ball.src != SRC_NONE) off = -wrap180(ball.angle - 90.0f) * DEF_SIDE_KP;   // ball right -> +
  else if (backGoalOk())    off = wrap180(fieldData.back_goal.angle - 270.0f) * DEF_GOAL_KP;
  float ref = ball.src != SRC_NONE ? wrap180(ball.angle - 90.0f)
                                   : (backGoalOk() ? wrap180(fieldData.back_goal.angle - 270.0f) : 0);
  v.x = fabsf(ref) <= DEF_CENTER_DEAD ? 0 : constrain(off, -DEF_SIDE_MAX, DEF_SIDE_MAX);

  float rightLim = sideLimit(rightSide, state), leftLim = sideLimit(leftSide, state);
  v.x = max(min(v.x, rightLim), -leftLim);           // side lines always win
  return v;
}

// LEAVE: line handling only near the side of the field
static bool leaveLineOn(const UsData& us) {
  if (!backGoalOk()) return true;                    // no goal: stay safe
  if (fieldData.back_goal.dist < PENALTY_ZONE_CM) return false;   // only the penalty line here: go out
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

  switch (st) {
    case DEFEND: {
      bool front = ball.src != SRC_NONE && fabsf(wrap180(ball.angle - 90.0f)) <= LEAVE_FRONT_DEG;
      if (front) {
        if (!frontTiming) { frontTiming = true; frontStart = now; }
        frontLastIn = now;
      } else if (frontTiming && now - frontLastIn > LEAVE_BREAK_MS) {
        frontTiming = false;                       // out of the window too long: start again
      }
      if (frontTiming && now - frontStart >= LEAVE_HOLD_MS) { go(LEAVE, "ball in front 3 s"); break; }
      v = defendMotion(state, ball);
      break;
    }

    case LEAVE: {
      if (!has_ball()) {
        if (now - lastBallSeen > BALL_LOST_MS)                   { go(RETURN, "ball lost"); break; }
        if (ball.src != SRC_NONE && fabsf(wrap180(ball.angle - 270.0f)) < BEHIND_DEG)
                                                                 { go(RETURN, "ball behind"); break; }
      }
      if (backGoalOk() && fieldData.back_goal.dist > LEAVE_MAX_DIST_CM) { go(RETURN, "too far"); break; }
      if (now - stStart > LEAVE_MAX_MS)                          { go(RETURN, "time up"); break; }

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

      if (leaveLineOn(us)) {
        LineCtl line = back_to_field(got & SENSE_GOT_LINE);
        if (line.active) { v = line.v; set_aim(false); set_target_heading(90); set_ramp(false); }
      }
      break;
    }

    case RETURN: {
      // Done only when a FRONT sensor sees the line (line ahead = robot behind it).
      // Still behind it (never crossed): done at once.
      if (behindLine())                { go(DEFEND, "behind the line"); break; }
      if (now - stStart > RETURN_MAX_MS) { go(DEFEND, "return timeout"); break; }

      float dir = backGoalOk() ? fieldData.back_goal.angle : 270.0f;
      if (ball.src != SRC_NONE) {                              // ball in the way: go around it
        float d = wrap180(ball.angle - dir);
        if (fabsf(d) < DODGE_DEG) dir += d > 0 ? -DODGE_OFFSET_DEG : DODGE_OFFSET_DEG;
      }
      v = polarV(RETURN_SPEED, dir);
      break;
    }
  }

  Vector_Motion(v.x, v.y, 0);

  EVERY_MS(tp, 200) {
    Serial.printf("%-6s (%s) | ball %-4s %5.1f | goal %s %5.1f %4u cm | line %d | hold %d | v (%4.0f, %4.0f)\n",
                  stateName[st], why, ball_src_name(ball.src), ball.angle,
                  fieldData.back_goal.seen ? "Y" : "-", fieldData.back_goal.angle, fieldData.back_goal.dist,
                  line_flag(), has_ball(), v.x, v.y);
  }
}
