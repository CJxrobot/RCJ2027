#include <Arduino.h>
#include <core_act.h>

// Motor direction of this robot body: 1 = attack body (wired the other way), 0 = defense body
#define MOTOR_REVERSE 1

// T11c (Act): attack robot, BASIC chase (act_attack / act_attack2 are not changed)
// NO white line check yet: the robot CAN go out. Test on the field with someone watching,
// or with the wheels off the ground first to check the motor direction.
//
//   picked up               -> motors off
//   ball held               -> shoot_control() (push while turning to the goal, kick; as defense)
//   ball 90 +- PUSH_DEG     -> forward, PUSH_SPEED3
//   IR strength >= STR_ORBIT -> around the ball (exponential, see below), ORBIT_SPEED3
//   IR strength <  STR_ORBIT -> straight at the ball, CHASE_SPEED3
//   no ball                 -> stop
//
// Orbit (exponential in the IR strength, the closer the ball the wider around it):
//   off = how far the ball is off the front (-180..180)
//   g   = min(1, exp((strength - STR_FULL) / STR_LAMBDA))     strength 5 -> 0.26, 7 -> 0.51, 9 -> 1
//   dir = ball angle + sign(off) x min(|off|, ORBIT_MAX_DEG) x g
//
// Serial every 200 ms: mode | ball angle / strength | hold | v

#define PUSH_SPEED3    60.0f  // ball in front: forward
#define CHASE_SPEED3   50.0f  // far ball: straight at it
#define ORBIT_SPEED3   40.0f  // near ball: around it
#define STR_ORBIT      5      // IR strength from here: orbit, below: straight at the ball
#define STR_FULL       9.0f   // IR strength of a ball right at the robot: full offset
#define STR_LAMBDA     3.0f
#define ORBIT_MAX_DEG  90.0f  // largest offset from the ball angle

enum Mode3 : uint8_t { M3_NONE, M3_SHOT, M3_PUSH, M3_CHASE, M3_ORBIT };
static const char* modeName3[] = {"NOBALL", "SHOT", "PUSH", "CHASE", "ORBIT"};
static Mode3 mode = M3_NONE;
static float orbitG = 0;               // last orbit factor (for the print)

static Vec2 polarV(float speed, float deg) {
  Vec2 v;
  v.x = speed * cosf(deg * DEG_TO_RAD);
  v.y = speed * sinf(deg * DEG_TO_RAD);
  return v;
}

static Vec2 chase3() {
  Vec2 v;
  if (!ir_ball_fresh(IR_TIMEOUT_MS)) { mode = M3_NONE; return v; }   // no ball: stop
  float a = fmodf(irBall.angle + 360.0f, 360.0f);
  float off = wrap180(a - 90.0f);                                     // > 0: ball left of the front
  if (fabsf(off) <= PUSH_DEG) { mode = M3_PUSH; return polarV(PUSH_SPEED3, 90.0f); }
  if (irBall.strength < STR_ORBIT) { mode = M3_CHASE; return polarV(CHASE_SPEED3, a); }
  mode = M3_ORBIT;
  orbitG = min(1.0f, expf((irBall.strength - STR_FULL) / STR_LAMBDA));
  float add = min(fabsf(off), (float)ORBIT_MAX_DEG) * orbitG;
  return polarV(ORBIT_SPEED3, a + (off > 0 ? add : -add));
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
      display.println("NO line check!");
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

  if (mode == M3_NONE) MotorStop();
  else Vector_Motion(v.x, v.y, 0);

  EVERY_MS(tp, 200) {
    Serial.printf("%-6s | ball %s %5.1f s%2u | orbit g %.2f | hold %d | v (%4.0f, %4.0f)\n",
                  modeName3[mode], irBall.found ? "Y" : "-", irBall.angle, irBall.strength,
                  orbitG, has_ball(), v.x, v.y);
  }
}
