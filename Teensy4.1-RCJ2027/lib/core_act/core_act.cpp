#include <core_act.h>
#include <DMAChannel.h>
GyroData gyroData;
RobotState robot;
// --- OLED OBJECT ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Speed actually sent (after the ramp). MotorStop() resets it to 0.
static volatile float rampVx = 0, rampVy = 0;
static const uint8_t motorDirPins[MOTOR_COUNT] = MOTOR_DIR_PINS;
static const uint8_t motorPwmPins[MOTOR_COUNT] = MOTOR_PWM_PINS;

void core_act_init() {
    Serial.begin(115200);
    Serial2.begin(115200);
    Serial8.begin(SENSE_BAUD);            // 921600, same constant on both boards
    pinMode(LED_BUILTIN, OUTPUT);

    // Motor Initialization
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        pinMode(motorDirPins[i], OUTPUT);
        pinMode(motorPwmPins[i], OUTPUT);
        analogWrite(motorPwmPins[i], 0);
    }
    pinMode(SLP, OUTPUT);
    digitalWrite(SLP, HIGH);   // wake motor drivers

    pinMode(BALL_POSESSION, INPUT);        // HIGH = no ball, LOW = ball (as tested on the robot)
    pinMode(START_BUTTON, INPUT_PULLUP);

    // Line flag: stop motors the moment either pin goes HIGH, before UART data arrives
    pinMode(PIN_0, INPUT_PULLDOWN);
    pinMode(PIN_1, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(PIN_0), MotorStop, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_1), MotorStop, RISING);

    Wire.begin();
    Wire.setClock(400000); // Fast I2C for OLED

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C) &&
        !display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
        Serial.println("OLED init failed at I2C addresses 0x3C and 0x3D");
        for (;;) {
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            delay(250);
        }
    }
    Serial.println("OLED initialized");

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    drawMessage("Starting");
}

void update_gyro_sensor(){
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];
  gyroData.exist = false; // Reset flag before read attempt

  while (Serial2.available() >= PACKET_SIZE){
    buffer[0] = Serial2.read();
    if(buffer[0] != 0xAA) continue;
    buffer[1] = Serial2.read();
    if(buffer[1] != 0xAA) continue;

    // Read remaining 17 bytes
    for (int i = 2; i < PACKET_SIZE; i++){
      buffer[i] = Serial2.read();
    }

    // --- Checksum: sum of bytes [2..16], mod 256 (uint8_t wraps) ---
    uint8_t esti_checksum = 0;
    for (int i = 2; i <= 16; i++){
      esti_checksum += buffer[i];
    }
    if(esti_checksum != buffer[18]) continue;

    // --- Extract angles (Little Endian, 0.01 deg) ---
    int16_t yaw_raw   = (int16_t)((buffer[4] << 8) | buffer[3]);
    int16_t pitch_raw = (int16_t)((buffer[6] << 8) | buffer[5]);
    int16_t roll_raw  = (int16_t)((buffer[8] << 8) | buffer[7]);

    if(abs(yaw_raw) <= 18000){
      gyroData.heading = yaw_raw * 0.01f;
      gyroData.exist = true;
    }
    if(abs(pitch_raw) <= 18000) gyroData.pitch = pitch_raw * 0.01f;
    if(abs(roll_raw)  <= 18000) gyroData.roll  = roll_raw  * 0.01f;
    // Acceleration (Little Endian, mg) -> m/s^2
    gyroData.ax = (int16_t)((buffer[10] << 8) | buffer[9])  * 0.00980665f;
    gyroData.ay = (int16_t)((buffer[12] << 8) | buffer[11]) * 0.00980665f;
    gyroData.az = (int16_t)((buffer[14] << 8) | buffer[13]) * 0.00980665f;
    // Keep reading: drain all queued packets so the newest one wins
  }
}

// ===== Line (from sense core) =====
bool line_flag() {
  return digitalReadFast(PIN_0) || digitalReadFast(PIN_1);
}

// Packet: AA s0 s1 s2 s3 checksum (sum of s0-s3)
// Uses the shared Serial8 parser, so ball packets (BB) are no longer thrown away
bool read_line_state(uint32_t& state) {
  if (!(read_sense() & SENSE_GOT_PKT)) return false;
  state = lineData.state;
  return true;
}

// Average direction of the sensors that see the line (bit = 0)
// Sensors on opposite sides cancel out -> return fallback (if >= 0), else the lowest sensor
float line_angle(uint32_t state, float fallback) {
  float x = 0, y = 0;
  int first = -1;
  for (int ch = 0; ch < LS_COUNT; ch++) {
    if (state & (1UL << ch)) continue;
    if (first < 0) first = ch;
    x += cosf(ch * 22.5f * DEG_TO_RAD);
    y += sinf(ch * 22.5f * DEG_TO_RAD);
  }
  if (first < 0) return -1;
  if (x * x + y * y < 0.25f) return fallback >= 0 ? fallback : first * 22.5f;

  float a = atan2f(y, x) * RAD_TO_DEG;
  return a < 0 ? a + 360.0f : a;
}

void drawMessage(const char* msg) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println(msg);
    display.display();
}

// ===== Motors =====
static bool motorReverse = false;            // set by each act program (MOTOR_REVERSE)
void set_motor_reverse(bool reverse) { motorReverse = reverse; }
bool motor_reverse() { return motorReverse; }

void SetMotorSpeed(uint8_t port, float speed){
  if (port < 1 || port > MOTOR_COUNT) return;
  uint8_t i = port - 1;

  speed = constrain(speed, -MOTOR_SPEED_LIMIT, MOTOR_SPEED_LIMIT);
  if (speed == 0) {
    analogWrite(motorPwmPins[i], 0);
    return;
  }
  // defense body: speed > 0 -> HIGH; attack body (reverse): speed > 0 -> LOW
  bool high = motorReverse ? speed < 0 : speed > 0;
  digitalWrite(motorDirPins[i], high ? HIGH : LOW);
  analogWrite(motorPwmPins[i], (int)(fabsf(speed) * 255.0f / 100.0f));
}

void MotorStop(){
  rampVx = 0;   // ramp starts again from standstill
  rampVy = 0;
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
    analogWrite(motorPwmPins[i], 0);
  }
}

// ===== Motion =====
// Wrap angle to (-180, 180]
float wrap180(float a) {
  while (a > 180.0f)   a -= 360.0f;
  while (a <= -180.0f) a += 360.0f;
  return a;
}

// Current heading in the 90-forward convention
static float currentHeading() {
  return 90.0f - gyroData.heading;
}
float current_heading() { return currentHeading(); }

void set_target_heading(float h) {
  robot.robot_heading = constrain(h, 90.0f - robot.heading_limit, 90.0f + robot.heading_limit);
}

void set_aim(bool aiming) { robot.aiming = aiming; }

// P controller with deadband: rotation command to reach target heading
// PD controller with deadband.
//   normal: P_factor + D_factor
//   aiming: aim_P, no D, |omega| limited to aim_omega_max (0 = no limit)
// D is reset when the mode changes or after a pause, so it never jumps.
static float headingCorrection(float target_heading) {
  static float prevErr = 0;
  static uint32_t prevUs = 0;
  static bool first = true, prevAim = false;

  float e = wrap180(target_heading - currentHeading());
  uint32_t now = micros();
  float d = 0;
  if (first || robot.aiming != prevAim) {
    first = false;
    prevAim = robot.aiming;
    prevErr = e;
    prevUs = now;
  } else {
    float dt = (now - prevUs) * 1e-6f;
    if (dt >= 0.001f && dt <= 0.1f) {
      d = wrap180(e - prevErr) / dt;
      prevErr = e;
      prevUs = now;
    } else if (dt > 0.1f) {
      prevErr = e;
      prevUs = now;
    }
  }

  if (fabsf(e) <= robot.heading_threshold) {
    prevErr = e;
    prevUs = now;
    return 0.0f;
  }
  if (robot.aiming) {
    float w = robot.aim_P * e;
    if (robot.aim_omega_max > 0) w = constrain(w, -robot.aim_omega_max, robot.aim_omega_max);
    return w;
  }
  return robot.P_factor * e + robot.D_factor * d;
}

void RobotIKControl(float vx, float vy, float omega) {
  // Inverse kinematics: wheel speed = projection of (vx, vy) on wheel axis + omega
  float w[4] = {-0.643f * vx + 0.766f * vy + omega,
                -0.643f * vx - 0.766f * vy + omega,
                 0.707f * vx - 0.707f * vy + omega,
                 0.707f * vx + 0.707f * vy + omega};
  // If a wheel is over the limit, scale ALL wheels down together: keeps the direction and
  // the turn right instead of clipping one wheel (clipping made the wheels fight and slip)
  float peak = 0;
  for (int i = 0; i < 4; i++) peak = max(peak, fabsf(w[i]));
  float k = peak > MOTOR_SPEED_LIMIT ? MOTOR_SPEED_LIMIT / peak : 1.0f;
  for (int i = 0; i < 4; i++) SetMotorSpeed(i + 1, w[i] * k);
}

// Robot-frame motion; rot_V nudges the target heading, clamped to 45..135
static uint32_t rampUs = 0;

void set_ramp(bool on) { robot.ramp_on = on; }
Vec2 motion_output() { Vec2 v; v.x = rampVx; v.y = rampVy; return v; }

// Move the output speed vector toward the target by at most accel (or decel) x dt.
// Works on the whole vector, so sudden turns / reversals are smoothed too.
static void rampToward(float& vx, float& vy) {
  uint32_t now = micros();
  float dt = constrain((now - rampUs) * 1e-6f, 0.0f, 0.02f);
  rampUs = now;
  if (!robot.ramp_on || robot.accel_max <= 0) { rampVx = vx; rampVy = vy; return; }

  float dx = vx - rampVx, dy = vy - rampVy;
  float dv = sqrtf(dx * dx + dy * dy);
  bool speedingUp = (vx * vx + vy * vy) >= (rampVx * rampVx + rampVy * rampVy);
  float rate = speedingUp ? robot.accel_max : (robot.decel_max > 0 ? robot.decel_max : robot.accel_max);
  float step = rate * dt;
  if (dv > step && dv > 0) {
    rampVx += dx / dv * step;
    rampVy += dy / dv * step;
  } else {
    rampVx = vx;
    rampVy = vy;
  }
  vx = rampVx;
  vy = rampVy;
}

void Vector_Motion(float Vx, float Vy, float rot_V) {
  robot.robot_heading = constrain(robot.robot_heading + rot_V,
                                  90.0f - robot.heading_limit, 90.0f + robot.heading_limit);
  rampToward(Vx, Vy);
  RobotIKControl(Vx, Vy, headingCorrection(robot.robot_heading));
}

// Field-centric motion: (WVx, WVy) in field frame, robot turns to target_heading
void FC_Vector_Motion(float WVx, float WVy, float target_heading) {
  // Rotate by the *current* heading so translation stays field-aligned while turning
  float rad = (currentHeading() - 90.0f) * DEG_TO_RAD;
  float cos_h = cosf(rad);
  float sin_h = sinf(rad);

  float robot_vx =  WVx * cos_h + WVy * sin_h;
  float robot_vy = -WVx * sin_h + WVy * cos_h;

  RobotIKControl(robot_vx, robot_vy, headingCorrection(target_heading));
}

// =====================================================================
// Sense link (Serial8 parser)
// =====================================================================
LineData lineData;
IrBallData irBall;
SenseLinkStats senseLinkStats;
SenseUsData senseUs;

uint8_t read_sense() {
  static uint8_t buf[SENSE_LEN];
  static uint8_t n = 0;
  uint8_t got = 0;

  while (Serial8.available()) {
    uint8_t b = Serial8.read();
    if (n == 0 && b != SENSE_HDR && b != SENSE_US_HDR) { senseLinkStats.junk++; continue; }
    buf[n++] = b;
    if (n < SENSE_LEN) continue;
    n = 0;

    if (pkt_sum(buf, 1, 9) != buf[10]) { senseLinkStats.bad++; continue; }
    uint32_t now = millis();
    if (buf[0] == SENSE_US_HDR) {                       // ultrasonic: only stored
      for (int i = 0; i < 4; i++) senseUs.cm[i] = pkt_u16(buf, 1 + 2 * i) * 0.1f;
      senseUs.t_ms = now;
      senseLinkStats.us_pkts++;
      got |= SENSE_GOT_US;
      continue;
    }
    lineData.state = buf[1] | (buf[2] << 8) | (buf[3] << 16) | ((uint32_t)buf[4] << 24);
    lineData.t_ms = now;
    uint16_t a = pkt_u16(buf, 6);
    irBall.found = buf[5] && a < 360;
    if (irBall.found) irBall.angle = a;
    irBall.strength = pkt_u16(buf, 8);
    irBall.t_ms = now;

    senseLinkStats.pkts++;
    got |= SENSE_GOT_PKT;
    if (lineData.state != 0xFFFFFFFF) { senseLinkStats.line_pkts++; got |= SENSE_GOT_LINE; }
  }
  // Sense core stopped or unplugged: forget the ball and the line
  uint32_t now = millis();
  bool linePkt = got & SENSE_GOT_PKT;
  if (!linePkt && irBall.t_ms != 0 && now - irBall.t_ms > IR_TIMEOUT_MS) irBall.found = false;
  if (!linePkt && lineData.t_ms != 0 && now - lineData.t_ms > IR_TIMEOUT_MS) lineData.state = 0xFFFFFFFF;
  return got;
}

bool ir_ball_fresh(uint32_t max_age_ms) {
  return irBall.found && irBall.t_ms != 0 && millis() - irBall.t_ms <= max_age_ms;
}

// =====================================================================
// Camera /field_data v2 (Serial3 DMA)
// =====================================================================
FieldData fieldData;
CameraStats cameraStats;

// Serial3 (LPUART2) RX by DMA into a ring buffer, parsed without blocking
#define CAM2_BUF 128 // power of 2, buffer aligned to its size
static volatile uint8_t cam2Rx[CAM2_BUF] __attribute__((aligned(CAM2_BUF)));
static DMAChannel cam2Dma;
static bool cam2Started = false;   // read_camera() does nothing before camera_begin()

void camera_begin() {
  Serial3.begin(CAM_BAUD);
  NVIC_DISABLE_IRQ(IRQ_LPUART2);
  LPUART2_CTRL &= ~(LPUART_CTRL_RIE | LPUART_CTRL_ILIE);
  LPUART2_WATER = LPUART_WATER_RXWATER(0);

  cam2Dma.begin(true);
  cam2Dma.source((volatile const unsigned char&)LPUART2_DATA);
  cam2Dma.destinationCircular(cam2Rx, CAM2_BUF);
  cam2Dma.triggerAtHardwareEvent(DMAMUX_SOURCE_LPUART2_RX);
  cam2Dma.enable();
  LPUART2_BAUD |= LPUART_BAUD_RDMAE;
  cam2Started = true;
}

static void decode(FieldObj& o, bool seen, const uint8_t* b, int i) {
  uint16_t a = pkt_u16(b, i);
  o.seen = seen && a < 360;
  if (o.seen) {
    o.angle = a;
    o.dist = pkt_u16(b, i + 2);
  } else {
    o.dist = PKT_NONE;
  }
}

bool read_camera() {
  static uint8_t buf[CAM_LEN];
  static int n = 0;
  static uint32_t tail = 0;
  static bool first = true;
  bool got = false;

  // Not started: the DMA address is not our buffer, so "head" would be garbage and the
  // loop below would never end (this froze every test that never called camera_begin)
  if (!cam2Started) return false;
  uint32_t head = ((uint32_t)cam2Dma.TCD->DADDR - (uint32_t)cam2Rx) % CAM2_BUF;
  while (tail != head) {
    uint8_t b = cam2Rx[tail];
    tail = (tail + 1) % CAM2_BUF;
    if (n == 0 && b != CAM_H1) continue;
    if (n == 1 && b != CAM_H2) { n = (b == CAM_H1); continue; }
    buf[n++] = b;
    if (n < CAM_LEN) continue;
    n = 0;

    if (buf[CAM_LEN - 1] != CAM_END || pkt_sum(buf, 1, 15) != buf[16]) {
      cameraStats.bad++;
      continue;
    }
    uint8_t flags = buf[2];
    decode(fieldData.ball,       flags & CAM_FLAG_BALL,  buf, 3);
    decode(fieldData.front_goal, flags & CAM_FLAG_FRONT, buf, 7);
    decode(fieldData.back_goal,  flags & CAM_FLAG_BACK,  buf, 11);

    uint8_t seq = buf[15];
    if (!first) cameraStats.lost += (uint8_t)(seq - fieldData.seq - 1);
    first = false;
    fieldData.seq = seq;
    fieldData.t_ms = millis();
    cameraStats.pkts++;
    got = true;
  }
  // Camera stopped or unplugged: forget everything so nothing old is used
  if (!got && fieldData.t_ms != 0 && millis() - fieldData.t_ms > CAM_TIMEOUT_MS) {
    fieldData.ball = FieldObj();
    fieldData.front_goal = FieldObj();
    fieldData.back_goal = FieldObj();
  }
  return got;
}

bool camera_fresh(uint32_t max_age_ms) {
  return fieldData.t_ms != 0 && millis() - fieldData.t_ms <= max_age_ms;
}

// =====================================================================
// Kicker
// =====================================================================
static KickState kickSt = KICK_CHARGING;
static uint32_t kickStart = 0;   // millis() when the state began
static uint32_t kickPulseStart = 0;   // micros() when DEAD / PULSE began
static uint32_t lastKick = 0;
static uint32_t kicks = 0;
static int chargeDuty = -1;
static uint8_t shotsLeft = KICK_BURST;  // kicks allowed before the rest
static bool resting = false;            // after the burst: soft charge for KICK_REST_MS
static uint32_t rampMs = RAMP_MS;       // ramp length of the current charge, 0 = full at once
static uint32_t readyAfterMs = KICK_MIN_CHARGE_MS; // charge time before the next kick


// ----- Software soft-charge PWM on pin 33 (IntervalTimer, no FlexPWM) -----
static IntervalTimer chargeTimer;
static volatile uint32_t chargeOnUs = 0, chargeOffUs = 0;
static volatile bool chargeLevel = false;  // true = P-MOS on
static bool chargeTimerRunning = false;

static inline void chargePin(bool on) {
  digitalWriteFast(KICK_CHARGE_PIN, on == KICK_CHARGE_ON_HIGH ? HIGH : LOW);
}

// One interrupt per edge. The PIT loads a new interval only at the NEXT timeout,
// so after switching to a level we set the length of the level after it.
static void chargeIsr() {
  chargeLevel = !chargeLevel;
  chargePin(chargeLevel);
  chargeTimer.update(chargeLevel ? chargeOffUs : chargeOnUs);
}

// d = 0..255. Very short on or off times are not worth an interrupt:
// the pin then just stays fully off / fully on.
static void chargeWrite(int d) {
  if (d == chargeDuty) return;
  chargeDuty = d;
  uint32_t period = 1000000UL / CHARGE_FREQ;
  uint32_t on = period * (uint32_t)constrain(d, 0, 255) / 255;
  uint32_t off = period - on;

  if (on < CHARGE_MIN_EDGE_US || off < CHARGE_MIN_EDGE_US) {  // steady level
    if (chargeTimerRunning) { chargeTimer.end(); chargeTimerRunning = false; }
    chargeLevel = on >= CHARGE_MIN_EDGE_US;
    chargePin(chargeLevel);
    return;
  }
  noInterrupts();
  chargeOnUs = on;
  chargeOffUs = off;
  interrupts();
  if (!chargeTimerRunning) {        // start with the pin off for one off-time
    chargeLevel = false;
    chargePin(false);
    chargeTimer.begin(chargeIsr, off);
    chargeTimer.update(on);         // length of the first on-time
    chargeTimerRunning = true;
  }
}

static void kickEnter(KickState s) {
  kickSt = s;
  kickStart = millis();
  kickPulseStart = micros();
}

void kicker_begin() {
  pinMode(KICK_CHARGE_PIN, OUTPUT);           // plain GPIO, never FlexPWM (B5)
  pinMode(KICK_DISCHARGE_PIN, OUTPUT);
  digitalWriteFast(KICK_DISCHARGE_PIN, LOW); // discharge off before anything else
  chargeDuty = -1;
  chargeWrite(0);
  pinMode(KICK_BALL_PIN, INPUT);             // HIGH = no ball, LOW = ball (as tested on the robot)
  shotsLeft = KICK_BURST;
  resting = false;
  rampMs = RAMP_MS;
  readyAfterMs = KICK_MIN_CHARGE_MS;
  kickEnter(KICK_CHARGING);
}

void kicker_update() {
  uint32_t t = millis() - kickStart;

  switch (kickSt) {
    case KICK_CHARGING:
      if (resting && t >= KICK_REST_MS) { resting = false; shotsLeft = KICK_BURST; }
      if (t >= CHARGE_TIMEOUT_MS) { chargeWrite(0); kickEnter(KICK_CHARGED); }
      else chargeWrite(t >= rampMs ? CHARGE_DUTY : (int)((uint32_t)CHARGE_DUTY * t / rampMs));
      break;
    case KICK_CHARGED:
      break;
    case KICK_DEAD:   // charge is off, wait before opening the discharge
      if (micros() - kickPulseStart >= DEAD_MS * 1000UL) {
        digitalWriteFast(KICK_DISCHARGE_PIN, HIGH);
        kickPulseStart = micros();
        kickSt = KICK_PULSE;
      }
      break;
    case KICK_PULSE:
      if (micros() - kickPulseStart >= KICK_MS * 1000UL) {
        digitalWriteFast(KICK_DISCHARGE_PIN, LOW);
        lastKick = millis();
        kicks++;
        if (shotsLeft > 0) shotsLeft--;
        if (shotsLeft == 0) {           // burst used up: rest and soft charge
          resting = true;
          rampMs = KICK_REST_MS;
          readyAfterMs = KICK_REST_MS;
        } else {                        // next shot of the burst: charge fully at once
          rampMs = 0;
          readyAfterMs = 0;
        }
        kickEnter(KICK_CHARGING);
      }
      break;
  }
}

bool kicker_ready() {
  if (resting || shotsLeft == 0) return false;
  if (lastKick != 0 && millis() - lastKick < MIN_KICK_GAP_MS) return false;
  if (kickSt == KICK_CHARGED) return true;
  return kickSt == KICK_CHARGING && millis() - kickStart >= readyAfterMs;
}

uint8_t kicker_shots_left() { return shotsLeft; }
bool kicker_resting() { return resting; }

bool kicker_fire() {
  if (!kicker_ready()) return false;
  chargeWrite(0);            // charge and discharge are never on together
  kickEnter(KICK_DEAD);
  return true;
}

bool has_ball() { return digitalReadFast(KICK_BALL_PIN) == LOW; }   // LOW = ball, no debounce
KickState kicker_state() { return kickSt; }
uint32_t kicker_count() { return kicks; }

const char* kicker_state_name() {
  switch (kickSt) {
    case KICK_CHARGING: return "CHARGING";
    case KICK_CHARGED:  return "CHARGED";
    case KICK_DEAD:     return "DEAD";
    default:            return "PULSE";
  }
}

// =====================================================================
// Strategy
// =====================================================================
static Vec2 polar(float speed, float deg) {
  Vec2 v;
  v.x = speed * cosf(deg * DEG_TO_RAD);
  v.y = speed * sinf(deg * DEG_TO_RAD);
  return v;
}

static float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

void strategy_begin() {
  robot.P_factor = HEADING_P;
  robot.D_factor = HEADING_D;
  robot.heading_threshold = HEADING_DEADBAND;
  robot.heading_limit = HEADING_LIMIT_DEG;
  robot.aim_P = AIM_HEADING_P;
  robot.aim_omega_max = AIM_OMEGA_MAX;
  robot.aiming = false;
  robot.robot_heading = 90.0f;
  robot.accel_max = ACCEL_MAX;
  robot.decel_max = DECEL_MAX;
  robot.ramp_on = true;
}

// Camera data in this FieldData is newer than CAM_TIMEOUT_MS
static bool field_fresh(const FieldData& f) {
  return f.t_ms != 0 && millis() - f.t_ms <= CAM_TIMEOUT_MS;
}

bool picked_up() { return fabsf(gyroData.pitch) > PICKUP_PITCH_DEG; }

__attribute__((weak)) UsData US_Sensor() { return UsData(); }

// ===== Line =====
LineCtl back_to_field(bool got_line_packet) {
  static bool active = false;
  static float first = -1;   // direction when the line was first seen
  static float away = 0;
  static float dir = -1;
  static uint32_t lastFlag = 0;

  LineCtl out;
  uint32_t now = millis();

  if (line_flag()) {
    lastFlag = now;
    if (!active) { active = true; first = -1; dir = -1; }
    if (got_line_packet) {
      if (first < 0) first = line_angle(lineData.state);
      float d = line_angle(lineData.state, first);
      if (d >= 0) { dir = d; away = d + 180.0f; }
    }
  } else if (active && now - lastFlag >= LINE_HOLD_MS) {
    active = false;
  }

  out.active = active;
  out.line_dir = dir;
  if (active && dir >= 0) out.v = polar(LINE_AVOID_SPEED, away);
  return out;
}

// ===== Ball =====
// Capture-mode state: filtered ball angle rate and hysteresis
static struct {
  bool init = false;
  bool capture = false;
  float last = 90, rate = 0, predErr = 0;
  uint32_t us = 0, camT = 0;
} cap;
static ChaseMode lastChase = CHASE_NONE;

static void capture_reset() { cap.init = false; cap.capture = false; cap.rate = 0; cap.predErr = 0; }

// Update once per new camera packet
static void capture_update(float angle, uint32_t cam_t) {
  if (cap.init && cam_t == cap.camT) return;
  uint32_t now = micros();
  if (!cap.init) {
    cap.init = true;
    cap.last = angle;
    cap.rate = 0;
  } else {
    float dt = constrain((now - cap.us) * 1e-6f, 0.005f, 0.1f);
    float measured = wrap180(angle - cap.last) / dt;
    cap.rate = CAPTURE_RATE_KEEP * cap.rate + (1.0f - CAPTURE_RATE_KEEP) * measured;
    cap.last = angle;
  }
  cap.us = now;
  cap.camT = cam_t;
  cap.predErr = wrap180(angle + cap.rate * CAPTURE_LOOKAHEAD_S - 90.0f);
  if (!cap.capture && fabsf(cap.predErr) < CAPTURE_ENTER_DEG) cap.capture = true;
  else if (cap.capture && fabsf(cap.predErr) > CAPTURE_EXIT_DEG) cap.capture = false;
}

Vec2 chase_ball(const MergedBall& ball, const FieldData& field) {
  bool camBall = ball.src == SRC_CAM || ball.src == SRC_FUSED;   // camera angle + pixel distance
  if (!camBall) capture_reset();

  if (ball.src == SRC_NONE) {
    const FieldObj& g = field.back_goal;
    if (field_fresh(field) && g.seen && g.dist > HOME_DIST_CM) {
      lastChase = CHASE_HOME;
      return polar(HOME_SPEED, g.angle);
    }
    lastChase = CHASE_NONE;
    return Vec2();
  }

  if (ball.src == SRC_IR) {
    lastChase = CHASE_IR;
    return polar(IR_SPEED_MIN + (IR_SPEED_MAX - IR_SPEED_MIN) * ball.closeness, ball.angle);
  }

  // Camera / FUSED ball. Distance from IR strength (CHASE_USE_IR_DIST 1, if IR sees the ball)
  // or from the camera pixel radius (reference code).
  float a = fmodf(ball.angle + 360.0f, 360.0f);
  float off = fabsf(a - 90.0f);                       // same as the reference: 0..270
  bool useIr = CHASE_USE_IR_DIST && ball.strength > 0;
  float d = ball.dist;                                // camera pixels, smaller = closer
  float st = ball.strength;                           // IR strength, bigger = closer

  bool pushNear  = useIr ? st >= IR_PUSH_STR : d <= PUSH_DIST;
  bool orbitNear = useIr ? st > IR_ORBIT_STR : d < CHASE_ORBIT_DIST;
  float speedT   = useIr ? (IR_CHASE_STR_NEAR - st) / (IR_CHASE_STR_NEAR - IR_CHASE_STR_FAR)
                         : (d - CHASE_DIST_NEAR) / (CHASE_DIST_FAR - CHASE_DIST_NEAR);  // 0 near, 1 far
  float ratio    = useIr ? (st >= IR_ORBIT_FULL_STR ? 1.0f : expf(-IR_ORBIT_DECAY * (IR_ORBIT_FULL_STR - st)))
                         : (d <= ORBIT_FULL_DIST ? 1.0f : expf(-ORBIT_DECAY * (d - ORBIT_FULL_DIST)));

  if (pushNear && off <= PUSH_DEG) {                   // right in front: push
    lastChase = CHASE_PUSH;
    Vec2 v;
    v.y = PUSH_SPEED;
    return v;
  }

  capture_update(a, field.t_ms);
  if (cap.capture) {                                   // straight-line approach
    lastChase = CHASE_CAPTURE;
    float lat = -(CAPTURE_KP * cap.predErr + CAPTURE_KD * cap.rate);
    lat = constrain(lat, -CAPTURE_MAX_LATERAL, CAPTURE_MAX_LATERAL);
    Vec2 v;
    v.x = lat;
    v.y = sqrtf(fmaxf(0.0f, CAPTURE_SPEED * CAPTURE_SPEED - lat * lat));
    return v;
  }

  lastChase = CHASE_ORBIT;
  float speed = CHASE_SPEED_MIN + (CHASE_SPEED_MAX - CHASE_SPEED_MIN) * clamp01(speedT);
  float dir = a;
  if (orbitNear) {
    speed *= 0.6f + 0.4f * (1.0f - clamp01(off / 90.0f));
    if (off <= CHASE_FRONT_DEG) {
      speed = CHASE_FRONT_SPEED;
    } else {
      float side = (a > 90.0f + CHASE_FRONT_DEG && a < 270.0f) ? 1.0f : -1.0f;
      dir = a + side * ORBIT_OFFSET_DEG * ratio;
    }
  }
  return polar(speed, dir);
}

ChaseMode chase_mode() { return lastChase; }
const char* chase_mode_name() {
  static const char* n[] = {"NONE", "HOME", "IR", "ORBIT", "CAPTURE", "PUSH"};
  return n[lastChase];
}

// ===== Heading =====
float rotate(const FieldData& field) {
  const FieldObj& g = field.front_goal;
  if (!field_fresh(field) || !g.seen) return 90.0f;
  // Turning CCW by d moves an object's angle by -d, so turn by (goal - 90)
  return current_heading() + wrap180(g.angle - 90.0f);
}

// ===== Shooting =====
bool in_corner(const FieldData& field) {
  const FieldObj& g = field.front_goal;
  return field_fresh(field) && g.seen && g.dist < CORNER_GOAL_DIST_CM &&
         fabsf(wrap180(g.angle - 90.0f)) > CORNER_GOAL_DEG;
}

enum CornerState : uint8_t { CORNER_IDLE, CORNER_BACKUP, CORNER_AIM, CORNER_SHOOT, CORNER_WAIT };

ShootCtl shoot_control(const FieldData& field) {
  static CornerState cs = CORNER_IDLE;
  static uint32_t ct = 0, holdStart = 0;
  static bool hadBall = false, kicked = false, cornerKicked = false;
  static float aimHeading = 90;

  ShootCtl out;
  uint32_t now = millis();
  bool ball = has_ball();
  const FieldObj& g = field.front_goal;
  bool goalOk = field_fresh(field) && g.seen;
  float goalErr = goalOk ? wrap180(g.angle - 90.0f) : 0;

  if (ball && !hadBall) { holdStart = now; kicked = false; }
  hadBall = ball;

  // ----- corner shot -----
  if (cs != CORNER_IDLE && !ball) cs = CORNER_IDLE;       // ball lost: give up
  if (cs == CORNER_IDLE && ball && in_corner(field)) { cs = CORNER_BACKUP; ct = now; aimHeading = 90; }

  if (cs != CORNER_IDLE) {
    out.active = true;
    switch (cs) {
      case CORNER_BACKUP:
        out.phase = "C-BACK";
        out.v.y = -CORNER_BACK_SPEED;
        if (now - ct >= CORNER_BACK_MS) { cs = CORNER_AIM; ct = now; }
        break;
      case CORNER_AIM:
        out.phase = "C-AIM";
        out.aiming = true;
        if (goalOk) aimHeading = current_heading() + goalErr;
        out.heading = aimHeading;
        if ((goalOk && fabsf(goalErr) < CORNER_AIM_OK_DEG) || now - ct >= CORNER_AIM_MAX_MS) {
          cs = CORNER_SHOOT; ct = now; cornerKicked = false;
        }
        break;
      case CORNER_SHOOT:
        out.phase = "C-SHOOT";
        out.aiming = true;
        out.heading = aimHeading;                           // keep the aimed heading
        out.v.y = CORNER_SHOOT_SPEED;
        if (!cornerKicked) cornerKicked = kicker_fire();
        if (now - ct >= CORNER_SHOOT_MS) cs = CORNER_WAIT;
        break;
      default:                                              // CORNER_WAIT
        out.phase = "C-WAIT";
        break;                                              // stop, heading 90
    }
    return out;
  }

  // ----- normal shot -----
  if (!ball) return out;
  out.active = true;
  if (goalOk) {
    out.phase = "AIM";
    out.aiming = true;
    out.heading = current_heading() + goalErr;
    out.v.y = AIM_PUSH_SPEED;
  } else {
    out.phase = "PUSH";
    out.v.y = NOGOAL_PUSH_SPEED;
  }
  if (!kicked && now - holdStart >= KICK_HOLD_MS) kicked = kicker_fire();
  return out;
}

// ===== Vector field =====
// Cut the part of v moving toward an object at from_deg, distance d: exponential deceleration
static void slow_toward(Vec2& v, float from_deg, float d, float stop, float lambda) {
  float ux = cosf(from_deg * DEG_TO_RAD), uy = sinf(from_deg * DEG_TO_RAD);
  float into = v.x * ux + v.y * uy;
  if (into <= 0) return;                         // moving away or along: keep
  float k = min(1.0f, expf(-(d - stop) / lambda));
  v.x -= k * into * ux;
  v.y -= k * into * uy;
}

Vec2 Vector_Field(Vec2 v, const FieldData& field, const UsData& us) {
  // Walls (R03): front 90, left 180, back 270, right 0. 0 = no reading.
  if (us.front > 0) slow_toward(v, 90,  us.front, WALL_STOP_CM, WALL_LAMBDA_CM);
  if (us.left  > 0) slow_toward(v, 180, us.left,  WALL_STOP_CM, WALL_LAMBDA_CM);
  if (us.back  > 0) slow_toward(v, 270, us.back,  WALL_STOP_CM, WALL_LAMBDA_CM);
  if (us.right > 0) slow_toward(v, 0,   us.right, WALL_STOP_CM, WALL_LAMBDA_CM);

  if (!field_fresh(field)) return v;
  const FieldObj& f = field.front_goal;
  const FieldObj& b = field.back_goal;
  if (f.seen && f.dist != PKT_NONE) slow_toward(v, f.angle, f.dist, GOAL_STOP_CM, GOAL_LAMBDA_CM);
  if (b.seen && b.dist != PKT_NONE) {
    slow_toward(v, b.angle, b.dist, GOAL_STOP_CM, GOAL_LAMBDA_CM);
    slow_toward(v, b.angle, b.dist, PENALTY_R_CM, PENALTY_LAMBDA_CM);   // own penalty area
    if (b.dist < PENALTY_R_CM) {                                         // inside: push out
      v.x -= VF_PENALTY_K * cosf(b.angle * DEG_TO_RAD);
      v.y -= VF_PENALTY_K * sinf(b.angle * DEG_TO_RAD);
    }
  }
  return v;
}

const char* ball_src_name(BallSrc s) {
  static const char* n[] = {"NONE", "CAM", "IR", "FUSED"};
  return n[s];
}
