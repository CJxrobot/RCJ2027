#include <Arduino.h>
#include <core_act.h>

// Motor direction of this robot body: 1 = attack body (wired the other way), 0 = defense body
#define MOTOR_REVERSE 1

// T11c (Act): attack robot, BASIC chase (act_attack / act_attack2 are not changed)
//
//   picked up               -> motors off
//   ball held               -> shoot_control() (push while turning to the goal, kick; as defense)
//   ball 90 +- PUSH_DEG3    -> forward: PUSH_SLOW3 for PUSH_SLOW_MS, then PUSH_SPEED3
//   IR strength >= STR_ORBIT -> around the ball (exponential, see below), slower near the front:
//                              ORBIT_SLOW3 + (ORBIT_SPEED3 - ORBIT_SLOW3) x min(1, |off| / ORBIT_SLOW_DEG)
//   IR strength <  STR_ORBIT -> straight at the ball, CHASE_SPEED3
//   no ball                 -> stop
//   white line (no timer):
//     line seen  -> LOCK its direction (from back_to_field's line_dir): the part of every move
//                   toward the line is 0 at once (brake, no ramp); moves along it go on
//     unlocked   -> only when no sensor sees the line AND the wanted move is not toward it
//                   (ball still out there: stays locked at the line, no back-and-forth)
//     line under the robot (two opposite ring sensors on it) -> also LINE_ESCAPE3 away
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
//
// Serial every 200 ms: mode | ball angle / strength | hold | v

#define PUSH_DEG3      5.0f   // ball this close to 90: push (library PUSH_DEG 10 is kept for defense)
#define PUSH_SPEED3    60.0f  // ball in front: forward ...
#define PUSH_SLOW3     40.0f  // ... but this for the first PUSH_SLOW_MS
#define PUSH_SLOW_MS   300
#define ORBIT_SLOW3    25.0f  // orbit speed with the ball near the front ...
#define ORBIT_SLOW_DEG 60.0f  // ... up to ORBIT_SPEED3 from this far off the front
#define LINE_ESCAPE3   40.0f  // line under the robot: away from it
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
static const float WHEEL_BONUS[4] = {2.0f, 2.0f, 2.0f, 2.0f};   // 45, 135, 225, 315 (estimate, measure)

enum Mode3 : uint8_t { M3_NONE, M3_SHOT, M3_PUSH, M3_CHASE, M3_ORBIT };
static bool lineLock = false;          // white line: direction locked
static float lineDirField = 0;         // line direction, field frame (robot frame + heading - 90)
static bool lineDeep = false;          // line under the robot (for the print)
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
  return str + bonus;
}
static float strUsed = 0;              // last compensated strength (for the print)

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

// Two opposite ring sensors on a line = the line is under the robot
static bool lineUnder(uint32_t state) {
  for (int ch = 0; ch < 8; ch++)
    if (!(state & (1UL << ch)) && !(state & (1UL << (ch + 8)))) return true;
  return false;
}

// White line: lock the direction toward it (no timer, see the top)
static void lineGuard(Vec2& v, uint8_t got) {
  LineCtl line = back_to_field(got & SENSE_GOT_LINE);   // only for line_dir
  float turn = current_heading() - 90.0f;
  bool on = line_flag();
  if (on && line.line_dir >= 0) {
    float d = line.line_dir + turn;
    // keep the side it was first locked on: past the middle of the line the sensors see it on
    // the other side, a flip there would push the robot OUT
    if (!lineLock || fabsf(wrap180(d - lineDirField)) <= 90.0f) lineDirField = d;
    lineLock = true;
  }
  if (on && !lineLock) { v = Vec2(); return; }          // direction not known yet: stop
  if (!lineLock) return;
  float dir = lineDirField - turn;                      // robot frame now
  if (!on && toward(v, dir) <= 0) { lineLock = false; return; }   // wants to go inside: unlock
  float in = toward(v, dir);
  if (in > 0) { v.x -= in * cosf(dir * DEG_TO_RAD); v.y -= in * sinf(dir * DEG_TO_RAD); }
  lineDeep = on && lineUnder(lineData.state);
  if (lineDeep) { v.x -= LINE_ESCAPE3 * cosf(dir * DEG_TO_RAD); v.y -= LINE_ESCAPE3 * sinf(dir * DEG_TO_RAD); }
}

void setup() {
  set_motor_reverse(MOTOR_REVERSE);   // before anything moves a motor
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
    if (millis() - lastDraw >= 100) {
      lastDraw = millis();
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println("ATTACK3 press START");
      display.printf("Motor %s\n", MOTOR_REVERSE ? "REVERSE (atk)" : "normal (def)");
      display.printf("Head %.1f\n", current_heading());
      display.printf("Ball %s %.0f s%u\n", irBall.found ? "Y" : "-", irBall.angle, irBall.strength);
      display.printf("Hold %d  Kick %s\n", has_ball(), kicker_state_name());
      display.printf("Line %04lX\n", lineData.state & 0xFFFF);
      display.display();
    }
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
  ShootCtl shot = shoot_control(fieldData);
  if (shot.active) {
    mode = M3_SHOT;
    v = shot.v;
    set_aim(shot.aiming);
    set_target_heading(shot.heading);
  } else {
    set_aim(false);
    set_target_heading(90);
    v = chase3();
  }

  lineDeep = false;
  lineGuard(v, got);
  set_ramp(!lineLock);               // locked: the brake works at once

  if (mode == M3_NONE && !lineDeep) MotorStop();
  else Vector_Motion(v.x, v.y, 0);

  EVERY_MS(tp, 200) {
    Serial.printf("%-6s | ball %s %5.1f s%2u > %4.1f | orbit g %.2f | hold %d | line %04lX %s %5.1f | v (%4.0f, %4.0f)\n",
                  modeName3[mode], irBall.found ? "Y" : "-", irBall.angle, irBall.strength,
                  strUsed, orbitG, has_ball(), lineData.state & 0xFFFF,
                  lineDeep ? "DEEP" : lineLock ? "LOCK" : "-   ", lineLock ? lineDirField : -1.0f,
                  v.x, v.y);                                    // strength raw > compensated
  }
}
