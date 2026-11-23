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
void set_target_heading(float h);       // absolute target, limited to 45..135 (90 = facing forward)
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

// ===== Sense -> Act, Serial8, 921600 =====
#define SENSE_BAUD      921600
// /line_data, high priority: AA s0 s1 s2 s3 chk   (6 bytes), chk = sum of bytes [1..4]
#define LINE_HDR        0xAA
#define LINE_LEN        6
// /ir_ball_data, low priority: BB found angL angH strL strH chk   (7 bytes), chk = sum of bytes [1..5]
#define BALL_HDR        0xBB
#define BALL_LEN        7

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
  uint32_t line_pkts = 0;
  uint32_t ball_pkts = 0;
  uint32_t bad = 0;            // checksum errors
  uint32_t junk = 0;           // bytes dropped while looking for a header
};

extern LineData lineData;
extern IrBallData irBall;
extern SenseLinkStats senseLinkStats;

// Parse every byte waiting on Serial8, non-blocking.
// Returns a bit mask of what arrived: SENSE_GOT_LINE | SENSE_GOT_BALL
#define SENSE_GOT_LINE 0x01
#define SENSE_GOT_BALL 0x02
uint8_t read_sense();

// true if the IR ball packet is newer than max_age_ms and says the ball is seen
bool ir_ball_fresh(uint32_t max_age_ms);


// =====================================================================
// Camera /field_data v2 (Serial3 DMA)
// =====================================================================
// /field_data v2 from the Maix Cam (Serial3, RX by DMA)
// Angles: deg, robot frame, 0 = right, 90 = front, CCW. Distances: cm.

struct FieldObj {
  bool seen = false;
  float angle = 0;
  uint16_t dist = PKT_NONE;
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
// B5: pin 33 shares FlexPWM2.0 with pin 4 (motor 3 PWM). The old program called
//     analogWriteFrequency(33, 1000), which also moved motor 3 to 1 kHz.
//     Here the charge PWM keeps the default frequency, the same one the motors use,
//     so nothing changes for motor 3. Never call analogWriteFrequency on pin 33 or 4.

#define KICK_CHARGE_PIN    33
#define KICK_DISCHARGE_PIN 32
#define KICK_BALL_PIN      38    // LOW = ball (same pin as BALL_POSESSION)
#define KICK_CHARGE_ON_HIGH 1    // 1: pin HIGH = P-MOS on

#define CHARGE_DUTY        255   // duty after the ramp (255 = fully on)
#define RAMP_MS            1000  // soft charge ramp 0 -> CHARGE_DUTY
#define CHARGE_TIMEOUT_MS  5000  // charge time, then charging stops
#define KICK_MIN_CHARGE_MS 1000  // charge at least this long before a kick is allowed
#define DEAD_MS            2     // charge off -> discharge on
#define KICK_MS            10    // discharge pulse
#define MIN_KICK_GAP_MS    500   // shortest time between kicks
#define BALL_DEBOUNCE_MS   20    // possession pin must stay LOW this long

enum KickState : uint8_t { KICK_CHARGING, KICK_CHARGED, KICK_DEAD, KICK_PULSE };

void kicker_begin();     // call once in setup(), after core_act_init()
void kicker_update();    // call every loop, runs the state machine
bool kicker_fire();      // request a kick, true if accepted (charged enough, gap ok)
bool kicker_ready();     // true if kicker_fire() would be accepted now
bool has_ball();         // debounced possession
KickState kicker_state();
const char* kicker_state_name();
uint32_t kicker_count(); // kicks since start


// =====================================================================
// Strategy parameters (same names as the Parameters tab)
// =====================================================================
// Every tunable number for the attack strategy, same names as the "Parameters" tab.
// Write the tuned value into the sheet too, so the sheet and the code stay the same.

// ----- Data age -----
#define CAM_TIMEOUT_MS     100   // camera data older than this = missing
#define IR_TIMEOUT_MS      50    // IR ball data older than this = missing

// ----- Merge -----
#define MERGE_MAX_ERR_DEG  90.0f // use camera ball only if within this angle of IR

// ----- Line -----
#define LINE_AVOID_SPEED   40.0f // % speed while leaving a line
#define LINE_HOLD_MS       150   // keep moving away after the flag clears

// ----- Heading -----
#define HEADING_P          0.5f  // copied into robot.P_factor by strategy_begin()
#define HEADING_DEADBAND   2.0f  // copied into robot.heading_threshold
#define KICK_ALIGN_DEG     180.0f // kick only when the front goal is within 90 +- this.
                                  // 180 = kick as soon as the ball is held (as in the design)

// ----- Chase -----
#define CHASE_SPEED        50.0f
#define CHASE_FRONT_DEG    15.0f // ball within 90 +- this: drive straight at it
#define CHASE_OFFSET_GAIN  1.0f  // offset = gain x angle from front x closeness
#define CHASE_OFFSET_MAX   90.0f
#define ORBIT_NEAR_CM      20.0f // camera distance where closeness = 1
#define ORBIT_FAR_CM       60.0f // camera distance where closeness = 0
#define IR_NEAR_STR        600.0f // IR strength where closeness = 1 (set after R02)
#define IR_FAR_STR         100.0f // IR strength where closeness = 0 (set after R02)
#define HOME_SPEED         30.0f // no ball: drive toward own goal at this speed
#define HOME_DIST_CM       60.0f // ... until the back goal is this close

// ----- Vector field -----
#define VF_K               40.0f // push at VF_D0_CM from a goal or wall
#define VF_D0_CM           30.0f
#define VF_LAMBDA_CM       15.0f // push falls by e every this many cm
#define PENALTY_R_CM       45.0f // goal distance treated as the penalty area edge
#define VF_PENALTY_K       60.0f // extra push inside PENALTY_R_CM
#define VF_MAX             40.0f // largest total push

// ----- Ultrasonic (R03) -----
#define US_PERIOD_MS       15

// ----- Game -----
#define ATTACK_YELLOW      1     // used by the camera code (R01), kept here for reference


// =====================================================================
// Strategy
// =====================================================================
// All angles in the robot frame: 0 = right, 90 = front, CCW. Speeds in % (-100..100).

struct Vec2 { float x = 0, y = 0; };

enum BallSrc : uint8_t { SRC_NONE, SRC_CAM, SRC_IR };
struct MergedBall {
  BallSrc src = SRC_NONE;
  float angle = 0;
  float closeness = 0;   // 0 = far, 1 = touching
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

// Camera ball if fresh and within MERGE_MAX_ERR_DEG of IR, else IR if fresh,
// else camera alone if IR is missing, else none.
MergedBall merge_ball_data(const FieldData& field, const IrBallData& ir);

// Get behind the ball. No ball: drive home toward the back goal (if seen), else stop.
Vec2 chase_ball(const MergedBall& ball, const FieldData& field);

// Target heading so the front goal ends up at 90 deg. 90 if the goal is not seen.
float rotate(const FieldData& field);

// Push away from goals (and walls, once US_Sensor() is filled), limited to VF_MAX
Vec2 Vector_Field(const FieldData& field, const UsData& us);

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
