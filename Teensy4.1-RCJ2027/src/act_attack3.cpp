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
//     first touch -> init = line direction (line_angle of the masked ring bits, field frame), LOCK
//     line seen   -> current direction updates dir only while within LINE_SIDE_DEG (90) of init:
//                    past the middle of the line the sensors see it on the other side, that
//                    one is ignored (dir stays). Move: the part of the wanted move toward dir is
//                    0 (brake, no ramp), moves along the line go on, + LINE_ESCAPE3 away from dir
//     line gone   -> still locked: no move toward dir (no escape); unlocked when the wanted move
//                    is not toward dir any more (ball still out there: waits at the line)
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
//   front sensor: not used while the ball is in front of it (within US_BALL_DEG3) or held, the
//          robot must reach / push the ball (2026-11-26: front brake too early, robot did not go)
//   failed (switched off / 0 / no packet US_STALE3_MS) -> x US_FAIL3 toward that side
//   Applied before the white line (the line sensors have the last word).
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
#define LINE_SIDE_DEG  90.0f  // line direction updates only within this of the first one
// Broken ring channels, Act side (emergency, no Sense flash needed; sense.cpp has its own mask).
// Bit ch = ring channel ch (angle ch x 22.5). Masked channels are never "line". Everything about
// the line here comes from the masked ring bits ONLY: the Sense flag pins (PIN_0 / PIN_1) cannot
// be masked on the Act side (a broken channel, or mux 2 noise with USE_MUX2 1, keeps them on and
// their interrupt stops the motors), so this program detaches that interrupt and does not use
// line_flag() / back_to_field().
#define LINE_IGNORE3   0x0080UL  // ch 7 (157.5 deg) broken (2026-11-25); 0 = none
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
#define BACK_ZONE_DEG  30.0f  // behind: compensation from 270 out to this
#define BACK_BONUS     2.0f   // strength lost at 270 (estimate, measure)

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
#define US_BALL_DEG3   30.0f  // "ball in front of the front sensor": within this of 90

enum Mode3 : uint8_t { M3_NONE, M3_SHOT, M3_PUSH, M3_CHASE, M3_ORBIT };
static bool lineLock = false;          // white line: direction locked
static float lineDirField = 0;         // line direction, field frame (robot frame + heading - 90)
static float lineInitField = 0;        // direction at the first touch, field frame
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

static float usCut3[4] = {0, 0, 0, 0};   // last cut per sensor (0 = none, 1 = all), for the screen

static void usField(Vec2& v, const Vec2& last) {
  for (int i = 0; i < 4; i++) usCut3[i] = 0;
  if (!US_FIELD3) return;
  bool ballFront = has_ball() ||
                   (ir_ball_fresh(IR_TIMEOUT_MS) && fabsf(wrap180(irBall.angle - 90.0f)) < US_BALL_DEG3);
  for (int i = 0; i < 4; i++) {
    float ux = cosf(usDir3[i] * DEG_TO_RAD), uy = sinf(usDir3[i] * DEG_TO_RAD);
    float into = v.x * ux + v.y * uy;
    if (into <= 0) continue;                                     // away / along: keep
    if (i == US_N_FRONT && ballFront) continue;                  // pushing / reaching the ball
    float d = usCm3(i);
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

// White line (no timer, see the top)
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
  }
  if (lineOn && !lineLock) { v = Vec2(); return; }      // direction not known yet: stop
  if (!lineLock) return;
  float dir = lineDirField - turn;                      // robot frame now
  float ux = cosf(dir * DEG_TO_RAD), uy = sinf(dir * DEG_TO_RAD);
  float in = toward(v, dir);
  if (!lineOn && in <= 0) { lineLock = false; return; } // wants to go inside: unlock
  if (in > 0) { v.x -= in * ux; v.y -= in * uy; }       // brake toward the line, along it goes on
  if (lineOn) { v.x -= LINE_ESCAPE3 * ux; v.y -= LINE_ESCAPE3 * uy; }   // on it: away
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
    lineData.state |= LINE_IGNORE3;
    kicker_update();
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
    display.printf("Line %04lX %s\n", lineData.state & 0xFFFF, lineLock ? "LOCK" : "");
    display.printf("Head %.1f Hold %d\n", current_heading(), has_ball());
    display.println("SW2/START: go on");
    display.display();
  }
  while (digitalRead(START_BUTTON) == LOW || digitalRead(DATA_BUTTON) == LOW) {}   // release
  sw2Pressed();
}

void setup() {
  set_motor_reverse(MOTOR_REVERSE);   // before anything moves a motor
  core_act_init();
  pinMode(DATA_BUTTON, INPUT_PULLUP);
  detachInterrupt(digitalPinToInterrupt(PIN_0));   // Sense line pins: not maskable here (see top)
  detachInterrupt(digitalPinToInterrupt(PIN_1));
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
    lineData.state |= LINE_IGNORE3;
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
      display.printf("Line %04lX mask %04lX\n", lineData.state & 0xFFFF, LINE_IGNORE3);
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
  read_sense();
  lineData.state |= LINE_IGNORE3;    // broken channels never "line"
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

  static Vec2 lastV;                 // speed sent last loop (ultrasonic lag)
  usField(v, lastV);                 // walls (ultrasonic), exponential
  lineGuard(v);                      // white line: last word
  set_ramp(!lineLock);               // locked: the brake works at once

  if (mode == M3_NONE && !lineOn) MotorStop();
  else Vector_Motion(v.x, v.y, 0);
  lastV = (mode == M3_NONE && !lineOn) ? Vec2() : v;

  if (sw2Pressed()) {                // SW2: stop and show the data (see dataScreen)
    float cutAt[4] = {usCut3[US_N_FRONT], usCut3[US_N_RIGHT], usCut3[US_N_BACK], usCut3[US_N_LEFT]};
    dataScreen(modeName3[mode], v, cutAt);
    lastV = Vec2();
  }

  EVERY_MS(tp, 200) {
    Serial.printf("%-6s | ball %s %5.1f s%2u > %4.1f | orbit g %.2f | hold %d | line %04lX %s %5.1f | "
                  "us F %3.0f R %3.0f B %3.0f L %3.0f cut F %.1f | v (%4.0f, %4.0f)\n",
                  modeName3[mode], irBall.found ? "Y" : "-", irBall.angle, irBall.strength,
                  strUsed, orbitG, has_ball(), lineData.state & 0xFFFF,
                  lineOn ? "ON  " : lineLock ? "LOCK" : "-   ", lineLock ? lineDirField : -1.0f,
                  usCm3(US_N_FRONT), usCm3(US_N_RIGHT), usCm3(US_N_BACK), usCm3(US_N_LEFT),
                  usCut3[US_N_FRONT], v.x, v.y);                                    // strength raw > compensated
  }
}
