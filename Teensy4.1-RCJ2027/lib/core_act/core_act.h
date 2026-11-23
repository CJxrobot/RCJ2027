#ifndef CORE_ACT_H
#define CORE_ACT_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- OLED Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// --- Motor Pins (index = SetMotorSpeed port) ---
//               port:  1   2   3   4
#define MOTOR_DIR_PINS { 6, 10,  9,  8}   // 方向控制腳
#define MOTOR_PWM_PINS { 2,  5,  4,  3}   // PWM 控制腳
#define MOTOR_COUNT 4
#define MOTOR_SPEED_LIMIT 75.0f           // |speed| cap in percent (RCAP 2027)

#define SLP 11
#define BALL_POSESSION 38
#define START_BUTTON 27 // button 1, pressed = LOW (INPUT_PULLUP)

// Line flag from sense core: HIGH while a line is seen
#define PIN_0 0
#define PIN_1 1
#define LS_COUNT 16 // line sensors, sensor ch at ch * 22.5 deg, CCW, 0 = right, 90 = front


struct GyroData {
    float heading = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    bool exist = false;   // true only if a valid packet arrived this update
};

// Heading convention: 90 = facing forward (field frame), CCW positive
struct RobotState {
    float robot_heading = 90.0f;     // target heading used by Vector_Motion
    float heading_threshold = 2.0f;  // deg, no correction inside this band
    float P_factor = 0.5f;           // omega per degree of heading error
    float D_factor = 0.0f;           // omega per deg/s (0 = P only, as before)
    float heading_limit = 45.0f;     // target heading stays within 90 +- this
    bool  aiming = false;            // true: aim_P, no D, omega limited (set_aim())
    float aim_P = 1.5f;
    float aim_omega_max = 0.0f;      // 0 = no limit
};

// Top camera (MaixCam, Serial3); angle/dist = 0xFFFF when goal not found
struct TopMaixData {
    bool valid = false;
    bool goal_found = false;
    uint16_t goal_angle = 0xFFFF; // deg 0-359
    uint16_t goal_dist = 0xFFFF;
};

extern GyroData gyroData;
extern TopMaixData topmaixData;
extern RobotState robot;
extern Adafruit_SSD1306 display;

void core_act_init();
void update_gyro_sensor();
void drawMessage(const char* msg);
void readTopMaix_polar();  // old protocol: send 0xDD, wait for reply (blocking)

// --- Top camera, new protocol: camera sends every frame, Serial3 RX by DMA ---
// After camera_dma_begin(), readTopMaix_polar() no longer works
void camera_dma_begin();
bool read_camera();        // non-blocking, true when topmaixData got a new packet

// --- Line (from sense core, Serial8) ---
bool line_flag();                       // true if PIN_0 or PIN_1 is HIGH
bool read_line_state(uint32_t& state);  // true if a valid packet arrived, keeps the newest
                                        // (now a wrapper around read_sense(), keeps ball packets too)
float line_angle(uint32_t state, float fallback = -1); // deg 0-360 of the line, -1 if none

// --- Motors ---
void SetMotorSpeed(uint8_t port, float speed);   // port 1-4, speed -100..100
void MotorStop();

// --- Motion ---
void RobotIKControl(float vx, float vy, float omega);
void Vector_Motion(float Vx, float Vy, float rot_V);
void FC_Vector_Motion(float WVx, float WVy, float target_heading);
void set_target_heading(float h);       // absolute target, limited to 90 +- robot.heading_limit
void set_aim(bool aiming);              // aiming: robot.aim_P, no D (see RobotState)
float current_heading();                // gyro heading in the 90-forward convention
float wrap180(float a);                 // wrap to (-180, 180]

// =====================================================================
// Packet formats (keep the same as core_sense.h and the Packets tab)
// =====================================================================
// Packet formats shared by Teensy Act, Teensy Sense and the Maix Cam
// Same layout as the "Packets" tab of the plan sheet. Change both together.
//
// Angles: 0-359 deg, robot frame, 0 = right, 90 = front, CCW. 0xFFFF = not seen.
// Multi-byte fields are little endian. Checksum = low 8 bits of the byte sum.

#define PKT_NONE 0xFFFF

// ===== /field_data v2: Maix Cam -> Act, Serial3, 115200, ~100 Hz =====
// CC DE flags ballA ballD frontA frontD backA backD seq chk EE   (18 bytes)
// chk = sum of bytes [1..15]
#define CAM_BAUD        115200
#define CAM_H1          0xCC
#define CAM_H2          0xDE
#define CAM_END         0xEE
#define CAM_LEN         18
#define CAM_FLAG_BALL   0x01
#define CAM_FLAG_FRONT  0x02
#define CAM_FLAG_BACK   0x04

// ===== /sense_data: Sense -> Act, Serial8, 921600 =====
// Line + IR ball in ONE packet (11 bytes), so every packet has both:
//   AA s0 s1 s2 s3 found angL angH strL strH chk     chk = sum of bytes [1..9]
//   s0-s3: lineState, bit = 0 when that sensor sees the line (all 1 = no line)
//   found: 1 = ball seen; angle 0-359 (0xFFFF if not seen); strength: higher = closer
// Sent every loop while a line is seen, otherwise at SENSE_TX_HZ.
#define SENSE_BAUD      921600
#define SENSE_HDR       0xAA
#define SENSE_LEN       11
#define SENSE_TX_HZ     100

static inline uint8_t pkt_sum(const uint8_t* b, int from, int to) {
  uint8_t s = 0;
  for (int i = from; i <= to; i++) s += b[i];
  return s;
}
static inline uint16_t pkt_u16(const uint8_t* b, int i) {
  return (uint16_t)(b[i] | (b[i + 1] << 8));
}
static inline void pkt_put16(uint8_t* b, int i, uint16_t v) {
  b[i] = v & 0xFF;
  b[i + 1] = v >> 8;
}


// =====================================================================
// Sense link (Serial8)
// =====================================================================
// One parser for everything Teensy Sense sends on Serial8 (fixes bug B1:
// the old read_line_state() threw away every byte that was not 0xAA)

struct LineData {
  uint32_t state = 0xFFFFFFFF; // bit = 0 when that sensor sees the line
  uint32_t t_ms = 0;           // millis() of the newest packet, 0 = never
};

struct IrBallData {
  bool found = false;
  float angle = 0;             // deg, robot frame, 0 = right, 90 = front
  uint16_t strength = 0;       // higher = closer
  uint32_t t_ms = 0;           // millis() of the newest packet, 0 = never
};

struct SenseLinkStats {
  uint32_t pkts = 0;           // all packets
  uint32_t line_pkts = 0;      // packets with a line in them
  uint32_t bad = 0;            // checksum errors
  uint32_t junk = 0;           // bytes dropped while looking for a header
};

extern LineData lineData;
extern IrBallData irBall;
extern SenseLinkStats senseLinkStats;

// Parse every byte waiting on Serial8, non-blocking. Every packet updates lineData AND irBall.
// Returns a bit mask: SENSE_GOT_PKT (any packet) | SENSE_GOT_LINE (a packet with a line in it)
#define SENSE_GOT_PKT  0x01
#define SENSE_GOT_LINE 0x02
uint8_t read_sense();

// true if the IR ball packet is newer than max_age_ms and says the ball is seen
bool ir_ball_fresh(uint32_t max_age_ms);


// =====================================================================
// Camera /field_data v2 (Serial3 DMA)
// =====================================================================
// /field_data v2 from the Maix Cam (Serial3, RX by DMA)
// Angles: deg, robot frame, 0 = right, 90 = front, CCW.
// Distances: ball = pixel radius from the image centre, goals = cm (camera formula).
// No packet for CAM_TIMEOUT_MS -> read_camera_v2() clears ball and goals (seen = false).

struct FieldObj {
  bool seen = false;
  float angle = 0;
  uint16_t dist = PKT_NONE;   // ball: pixels, goals: cm
};

struct FieldData {
  FieldObj ball;
  FieldObj front_goal;   // goal we attack
  FieldObj back_goal;    // our own goal
  uint8_t seq = 0;
  uint32_t t_ms = 0;     // millis() of the newest packet, 0 = never
};

struct CameraStats {
  uint32_t pkts = 0;
  uint32_t bad = 0;      // checksum / end byte errors
  uint32_t lost = 0;     // packets missing, counted from seq gaps
};

extern FieldData fieldData;
extern CameraStats cameraStats;

// Start Serial3 RX by DMA. Do not use readTopMaix_polar() or camera_dma_begin() after this.
void camera_v2_begin();
// Parse every byte DMA received since the last call. true if fieldData got a new packet.
bool read_camera_v2();
// true if the newest camera packet is younger than max_age_ms
bool camera_fresh(uint32_t max_age_ms);


// =====================================================================
// Kicker
// =====================================================================
// Kicker + ball possession, no delay() anywhere (fixes B4, B5, B6)
// 12 V -> XL6019 boost -> IRF5305 P-MOS (pin 33) -> 20000 uF cap -> solenoid, discharge (pin 32)
//
// B5: pin 33 shares FlexPWM2.0 with pin 4 (motor 3 PWM), so hardware PWM on pin 33
//     would change motor 3. Soft charge is done in software instead: an IntervalTimer
//     (PIT) toggles pin 33 with digitalWriteFast, two interrupts per PWM period
//     (one at the on edge, one at the off edge). Motor 3 is never touched.
//     Never call analogWrite / analogWriteFrequency on pin 33.

#define KICK_CHARGE_PIN    33
#define KICK_DISCHARGE_PIN 32
#define KICK_BALL_PIN      38    // LOW = ball (same pin as BALL_POSESSION)
#define KICK_CHARGE_ON_HIGH 1    // 1: pin HIGH = P-MOS on

#define CHARGE_FREQ        1000  // Hz, software PWM frequency while ramping
#define CHARGE_DUTY        255   // duty after the ramp (255 = fully on)
#define CHARGE_MIN_EDGE_US 20    // on/off time shorter than this -> pin stays fully off/on
#define RAMP_MS            1000  // soft charge ramp 0 -> CHARGE_DUTY
#define CHARGE_TIMEOUT_MS  5000  // charge time, then charging stops
#define KICK_MIN_CHARGE_MS 1000  // charge at least this long before a kick is allowed
#define DEAD_MS            2     // charge off -> discharge on
#define KICK_MS            15    // discharge pulse (15 ms in the tested reference code)
#define MIN_KICK_GAP_MS    500   // shortest time between kicks
#define KICK_BURST         2     // kicks allowed in a row, then a rest
#define KICK_REST_MS       2000  // rest after the burst: soft charge ramp over this time

enum KickState : uint8_t { KICK_CHARGING, KICK_CHARGED, KICK_DEAD, KICK_PULSE };

void kicker_begin();     // call once in setup(), after core_act_init()
void kicker_update();    // call every loop, runs the state machine
bool kicker_fire();      // request a kick, true if accepted (charged enough, gap ok)
bool kicker_ready();     // true if kicker_fire() would be accepted now
bool has_ball();         // pin 38 LOW = ball, read directly (hardware RC filter)
KickState kicker_state();
const char* kicker_state_name();
uint32_t kicker_count(); // kicks since start
uint8_t kicker_shots_left();
bool kicker_resting();   // true during the KICK_REST_MS rest after a burst


// =====================================================================
// Strategy parameters (same names as the Parameters tab)
// =====================================================================
// Every tunable number for the attack strategy, same names as the "Parameters" tab.
// Write the tuned value into the sheet too, so the sheet and the code stay the same.

// ----- Data age -----
#define CAM_TIMEOUT_MS     100   // camera data older than this = missing
#define IR_TIMEOUT_MS      50    // IR ball data older than this = missing

// ----- Merge -----
#define MERGE_MAX_ERR_DEG  30.0f // camera and IR angles closer than this: camera angle + IR distance

// ----- Line -----
#define LINE_AVOID_SPEED   40.0f // % speed while leaving a line
#define LINE_HOLD_MS       150   // keep moving away after the flag clears

// ----- Heading (PD, values from the tested reference code) -----
#define HEADING_P          0.7f  // copied into robot.P_factor by strategy_begin()
#define HEADING_D          0.03f // copied into robot.D_factor
#define HEADING_DEADBAND   2.0f  // copied into robot.heading_threshold
#define HEADING_LIMIT_DEG  60.0f // target heading stays within 90 +- this
#define PICKUP_PITCH_DEG   15.0f // |pitch| above this = robot picked up -> motors stop

// ----- Shooting (ball held) -----
#define AIM_PUSH_SPEED     80.0f // front goal seen: push forward at this speed while aiming
#define NOGOAL_PUSH_SPEED  50.0f // front goal not seen: push forward, heading 90
#define AIM_HEADING_P      1.5f  // heading P while aiming (no D)
#define AIM_OMEGA_MAX      0.0f  // |omega| limit while aiming, 0 = no limit (as tested)
#define KICK_HOLD_MS       50    // hold the ball this long, then kick once

// ----- Corner shot -----
// "In the corner" = front goal seen, closer than CORNER_GOAL_DIST_CM and more than
// CORNER_GOAL_DEG off the front. Measure both on the field.
#define CORNER_GOAL_DIST_CM 60.0f
#define CORNER_GOAL_DEG    40.0f
#define CORNER_BACK_SPEED  30.0f // 1. back up
#define CORNER_BACK_MS     700
#define CORNER_AIM_OK_DEG  6.0f  // 2. turn to the goal until within this, or
#define CORNER_AIM_MAX_MS  2500  //    give up aiming after this
#define CORNER_SHOOT_SPEED 35.0f // 3. push forward and kick
#define CORNER_SHOOT_MS    300   // 4. then wait until the ball has left

// ----- Chase (values from the tested reference code) -----
// Distances are camera ball distances. The reference camera sent the image radius in
// pixels, not cm: re-measure these with our camera code (R01).
#define CHASE_SPEED_MIN    40.0f // speed at CHASE_DIST_NEAR
#define CHASE_SPEED_MAX    70.0f // speed at CHASE_DIST_FAR
#define CHASE_DIST_NEAR    25.0f
#define CHASE_DIST_FAR     55.0f
#define CHASE_ORBIT_DIST   65.0f // closer than this: slow down off-front and circle
#define CHASE_FRONT_DEG    15.0f // ball within 90 +- this: drive straight at it
#define CHASE_FRONT_SPEED  50.0f
#define ORBIT_OFFSET_DEG   90.0f // circling offset when close
#define ORBIT_FULL_DIST    55.0f // closer than this: full offset
#define ORBIT_DECAY        1.5f  // farther: offset x exp(-decay x (dist - full))
#define PUSH_DIST          37.0f // ball this close and within 90 +- PUSH_DEG:
#define PUSH_DEG           10.0f //   drive straight forward at PUSH_SPEED
#define PUSH_SPEED         80.0f
// Capture mode (straight-line approach with prediction), hysteresis on the predicted error
#define CAPTURE_ENTER_DEG  14.0f
#define CAPTURE_EXIT_DEG   25.0f
#define CAPTURE_LOOKAHEAD_S 0.07f // predict the ball angle this far ahead
#define CAPTURE_RATE_KEEP  0.75f // angle-rate filter: old share (new share = 1 - this)
#define CAPTURE_SPEED      80.0f
#define CAPTURE_KP         0.75f
#define CAPTURE_KD         0.15f
#define CAPTURE_MAX_LATERAL 50.0f
// Which distance the chase uses for a camera / FUSED ball:
//   1 = IR strength (sensors seeing the ball, 1-20, bigger = closer), thresholds below
//   0 = camera pixel radius (reference code, CHASE_DIST_* / ORBIT_FULL_DIST / PUSH_DIST above)
// With 1, a camera-only ball (no IR) still uses the camera pixels.
#define CHASE_USE_IR_DIST  1
// IR-strength thresholds (bigger = closer). First guesses: tune on the field with test_chase.
#define IR_CHASE_STR_NEAR  10.0f // strength at CHASE_SPEED_MIN (near, slow)
#define IR_CHASE_STR_FAR   4.0f  // strength at CHASE_SPEED_MAX (far, fast)
#define IR_ORBIT_STR       3.0f  // stronger than this: slow down off-front and circle
#define IR_ORBIT_FULL_STR  5.0f  // at least this: full circling offset
#define IR_ORBIT_DECAY     0.5f  // weaker: offset x exp(-decay x (full - strength))
#define IR_PUSH_STR        10.0f // at least this and within 90 +- PUSH_DEG: push forward
// IR only (no camera ball): drive straight at the ball
#define IR_SPEED_MIN       50.0f // speed at closeness 0
#define IR_SPEED_MAX       70.0f // speed at closeness 1
#define IR_NEAR_STR        8.0f  // IR strength (= sensors seeing the ball, 1-20) where closeness = 1
#define IR_FAR_STR         1.0f  // IR strength where closeness = 0
// No ball
#define HOME_SPEED         30.0f // drive toward our own goal at this speed
#define HOME_DIST_CM       60.0f // ... until the back goal is this close

// ----- Vector field (whole field) -----
// Exponential deceleration toward each boundary: the part of v moving toward it is cut by
//   k = exp(-(d - STOP) / LAMBDA), at most 1  (d <= STOP: fully cut, d = STOP + LAMBDA: 37 %)
// Motion along a boundary is kept, so the robot slides instead of stopping.
#define WALL_STOP_CM       15.0f // ultrasonic distance where motion toward a wall is fully cut (R03)
#define WALL_LAMBDA_CM     10.0f
#define GOAL_STOP_CM       25.0f // front / back goal distance where motion toward that end is fully cut
#define GOAL_LAMBDA_CM     15.0f
// Own penalty area (back goal only), on top of the above
#define PENALTY_R_CM       45.0f // back goal distance at our penalty area edge (measure on the field)
#define PENALTY_LAMBDA_CM  10.0f
#define VF_PENALTY_K       30.0f // already inside R: push straight out at this speed

// ----- Ultrasonic (R03) -----
#define US_PERIOD_MS       15

// ----- Game -----
#define ATTACK_YELLOW      1     // used by the camera code (R01), kept here for reference


// =====================================================================
// Strategy
// =====================================================================
// All angles in the robot frame: 0 = right, 90 = front, CCW. Speeds in % (-100..100).

struct Vec2 { float x = 0, y = 0; };

// SRC_FUSED = both seen and agree: angle from the camera, distance from IR
enum BallSrc : uint8_t { SRC_NONE, SRC_CAM, SRC_IR, SRC_FUSED };
struct MergedBall {
  BallSrc src = SRC_NONE;
  float angle = 0;
  float dist = -1;       // camera ball distance (pixels), -1 when the camera has no ball
  float strength = 0;    // IR distance: sensors seeing the ball (1-20), 0 when IR has no ball
  float closeness = 0;   // from the IR strength: 0 = far, 1 = touching (0 when IR has no ball)
};

enum ChaseMode : uint8_t { CHASE_NONE, CHASE_HOME, CHASE_IR, CHASE_ORBIT, CHASE_CAPTURE, CHASE_PUSH };

// Shooting / corner shot output
struct ShootCtl {
  bool active = false;   // true: use v / heading / aiming instead of chasing
  Vec2 v;
  float heading = 90;
  bool aiming = false;
  const char* phase = "";
};

// Ultrasonic distances in cm, 0 = no reading (filled by US_Sensor(), reserved R03)
struct UsData { float front = 0, back = 0, left = 0, right = 0; };

struct LineCtl {
  bool active = false;   // true: line has priority, use v
  Vec2 v;
  float line_dir = -1;   // deg, -1 = unknown (waiting for the packet)
};

void strategy_begin();   // copies HEADING_P / HEADING_DEADBAND into robot

// back_to_field(): call every loop. Uses line_flag() and lineData from read_sense().
//   flag HIGH, no packet yet  -> active, v = 0 (motors already stopped by the interrupt)
//   flag HIGH, packet         -> active, v away from the line (line_angle + 180)
//   flag LOW                  -> still active for LINE_HOLD_MS, then inactive
LineCtl back_to_field(bool got_line_packet);

// Both seen, angles within MERGE_MAX_ERR_DEG -> FUSED: camera angle + IR distance
// Both seen, angles farther apart            -> IR (angle and distance)
// Only IR -> IR,  only camera -> CAM,  none -> NONE
MergedBall merge_ball_data(const FieldData& field, const IrBallData& ir);

// Chase (reference code, tested):
//   camera ball: capture mode (predicted error < CAPTURE_ENTER_DEG, leaves at > CAPTURE_EXIT_DEG)
//                else orbit: speed by distance, straight when in front, circle when close;
//                very close and in front: push forward at PUSH_SPEED
//   IR ball:     straight at the ball, IR_SPEED_MIN..MAX by closeness
//   no ball:     drive home toward the back goal (if seen), else stop
Vec2 chase_ball(const MergedBall& ball, const FieldData& field);
ChaseMode chase_mode();          // what chase_ball() did last
const char* chase_mode_name();

// Target heading so the front goal ends up at 90 deg. 90 if the goal is not seen.
float rotate(const FieldData& field);

// Ball held (and the corner shot sequence). Call every loop:
//   normal: push forward, aim at the front goal (AIM_HEADING_P), kick once after KICK_HOLD_MS
//   corner: back up -> aim -> push + kick -> wait for the ball to leave
ShootCtl shoot_control(const FieldData& field);
bool in_corner(const FieldData& field);

// Robot lifted (|pitch| > PICKUP_PITCH_DEG): stop the motors and reset the heading target
bool picked_up();

// Whole-field vector field, returns the new velocity:
//   walls (ultrasonic, R03), front goal, back goal -> exponential deceleration toward each
//   own penalty area (back goal)                   -> same, with PENALTY_R_CM as the stop distance,
//                                                     and a push out if already inside
Vec2 Vector_Field(Vec2 v, const FieldData& field, const UsData& us);

// Reserved R03: define it in your ultrasonic file and this zero version goes away
UsData US_Sensor();

const char* ball_src_name(BallSrc s);


// =====================================================================
// Test helpers (src/test_*.cpp)
// =====================================================================
// Small helpers shared by the Act test programs (src/test_*.cpp)

// Wait for START (button 1) with motors stopped, keep reading inputs meanwhile
static inline void wait_start(const char* name) {
  MotorStop();
  drawMessage(name);
  Serial.printf("%s: press START\n", name);
  while (digitalRead(START_BUTTON) == HIGH) {
    update_gyro_sensor();
    read_sense();
    read_camera_v2();
  }
  delay(200);                                    // debounce
  while (digitalRead(START_BUTTON) == LOW) {}    // wait for release
  drawMessage("GO");
}

// true once per press of START (after wait_start), 200 ms debounce
static inline bool start_pressed() {
  static bool last = HIGH;
  static uint32_t t = 0;
  bool b = digitalRead(START_BUTTON);
  bool hit = last == HIGH && b == LOW && millis() - t > 200;
  if (hit) t = millis();
  last = b;
  return hit;
}

// Loop rate and worst loop time, reset every report
struct LoopMeter {
  uint32_t loops = 0, worst = 0, t0 = 0, lastReport = 0;
  void begin() { t0 = micros(); }
  void end() { uint32_t d = micros() - t0; if (d > worst) worst = d; loops++; }
  bool report(uint32_t every_ms = 1000) { return millis() - lastReport >= every_ms; }
  void reset() { lastReport = millis(); loops = 0; worst = 0; }
};

// every(ms) helper: true once per period
#define EVERY_MS(var, ms) static uint32_t var = 0; if (millis() - var >= (ms) && ((var = millis()), true))

#endif
