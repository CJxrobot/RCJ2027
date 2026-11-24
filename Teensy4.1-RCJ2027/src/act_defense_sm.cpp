#include <Arduino.h>
#include <core_act.h>

// T12 (Act): defense robot as a state machine
// Ball: IR only for now (ir_only_ball). For camera + IR use merge_ball_data(fieldData, irBall).
//
//   DEFEND  stay on the penalty line (same line logic as act_defense)
//           ball seen:    move sideways to keep the ball straight ahead
//           no ball:      move sideways until the back goal is at 270 (in front of our goal)
//           ball within 90 +- LEAVE_FRONT_DEG for LEAVE_HOLD_MS without a break  -> LEAVE
//   LEAVE   leave the penalty area and take the ball
//           ball held:    shoot_control() (same as the attack robot)
//           ball 85-95:   rush straight at it, else chase_ball()
//           line handling only when the back goal is more than LINE_SIDE_GOAL_DEG off 270
//           (and, once US_Sensor() is filled, a side ultrasonic < LINE_SIDE_US_CM)
//           ball lost / ball behind / too far from our goal / LEAVE_MAX_MS over  -> RETURN
//   RETURN  drive toward our goal, go around the ball if it is in the way,
//           line handling on; penalty line reached (line flag)                   -> DEFEND
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
#define LEAVE_HOLD_MS    3000  // ... for this long without a break

// ----- LEAVE -----
#define LEAVE_RUSH_SPEED 70.0f // ball 85-95: straight at it
#define LEAVE_MAX_MS     3000  // longest time out of the penalty area
#define LEAVE_MAX_DIST_CM 90.0f // back goal farther than this: too far, return
#define BALL_LOST_MS     200   // no ball for this long: return
#define BEHIND_DEG       60.0f // ball within 270 +- this: behind the robot, return
#define LINE_SIDE_GOAL_DEG 45.0f // back goal more than this off 270: line handling on
#define LINE_SIDE_US_CM  20.0f // ... and a side ultrasonic closer than this (R03)

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
static uint32_t stStart = 0, frontStart = 0, lastBallSeen = 0;
static bool frontTiming = false;
static const char* why = "";

static void go(DefState s, const char* reason) {
  st = s;
  stStart = millis();
  frontTiming = false;
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

  bool in = seen(state, FRONT_IN), out = seen(state, FRONT_OUT);
  if (in && !out)      v.y = -SPEED;
  else if (out && !in) v.y =  SPEED;
  else if (back > 0)   v.y = -SPEED;
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
  bool sideGoal = fabsf(wrap180(fieldData.back_goal.angle - 270.0f)) > LINE_SIDE_GOAL_DEG;
  bool usFitted = us.left > 0 || us.right > 0;       // R03 not in yet: goal angle only
  if (!usFitted) return sideGoal;
  bool nearWall = (us.left > 0 && us.left < LINE_SIDE_US_CM) || (us.right > 0 && us.right < LINE_SIDE_US_CM);
  return sideGoal && nearWall;
}

void setup() {
  core_act_init();
  camera_v2_begin();
  kicker_begin();
  strategy_begin();

  MotorStop();
  uint32_t lastDraw = 0;
  while (digitalRead(START_BUTTON) == HIGH) {
    update_gyro_sensor();
    read_camera_v2();
    read_sense();
    kicker_update();
    if (millis() - lastDraw >= 100) {
      lastDraw = millis();
      MergedBall b = ir_only_ball(irBall);
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
  read_camera_v2();
  uint8_t got = read_sense();
  kicker_update();
  if (millis() - lastUs >= US_PERIOD_MS) { lastUs = millis(); us = US_Sensor(); }

  if (picked_up()) { set_aim(false); set_target_heading(90); MotorStop(); return; }

  uint32_t now = millis();
  uint32_t state = line_flag() ? lineData.state : 0xFFFFFFFF;
  MergedBall ball = ir_only_ball(irBall);
  if (ball.src != SRC_NONE || has_ball()) lastBallSeen = now;

  Vec2 v;
  set_aim(false);
  set_target_heading(90);
  set_ramp(st != DEFEND);            // DEFEND line following reacts at once; LEAVE / RETURN ramp

  switch (st) {
    case DEFEND: {
      bool front = ball.src != SRC_NONE && fabsf(wrap180(ball.angle - 90.0f)) <= LEAVE_FRONT_DEG;
      if (!front) frontTiming = false;
      else if (!frontTiming) { frontTiming = true; frontStart = now; }
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
      if (line_flag())                 { go(DEFEND, "penalty line"); break; }
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
