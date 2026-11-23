#include <core_act.h>
#include <DMAChannel.h>
GyroData gyroData;
TopMaixData topmaixData;
RobotState robot;
// --- OLED OBJECT ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static const uint8_t motorDirPins[MOTOR_COUNT] = MOTOR_DIR_PINS;
static const uint8_t motorPwmPins[MOTOR_COUNT] = MOTOR_PWM_PINS;

void core_act_init() {
    Serial.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
    Serial3.setTimeout(50); // ms, camera answers once per frame
    Serial8.begin(921600);
    pinMode(LED_BUILTIN, OUTPUT);

    // Motor Initialization
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        pinMode(motorDirPins[i], OUTPUT);
        pinMode(motorPwmPins[i], OUTPUT);
        analogWrite(motorPwmPins[i], 0);
    }
    pinMode(SLP, OUTPUT);
    digitalWrite(SLP, HIGH);   // wake motor drivers

    pinMode(BALL_POSESSION, INPUT_PULLUP); // B6: same as the kicker, never floats
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
    // Keep reading: drain all queued packets so the newest one wins
  }
}

// Packet: CC DD found angleL angleH distL distH checksum EE, checksum = sum of bytes [1..6]
void readTopMaix_polar() {
  uint8_t buf[9];
  while (Serial3.available()) Serial3.read(); // drop stale bytes
  Serial3.write(0xDD);                        // request

  uint8_t checksum = 0;
  bool ok = Serial3.readBytes(buf, 9) == 9 &&
            buf[0] == 0xCC && buf[1] == 0xDD && buf[8] == 0xEE;
  for (int i = 1; i <= 6; i++) checksum += buf[i];

  topmaixData.valid = ok && checksum == buf[7];
  if (!topmaixData.valid) return;

  topmaixData.goal_found = buf[2];
  topmaixData.goal_angle = buf[3] | (buf[4] << 8);
  topmaixData.goal_dist  = buf[5] | (buf[6] << 8);
}

// ===== Top camera over DMA (Serial3 = LPUART2) =====
// DMA copies every received byte into a ring buffer, no CPU and no interrupt
#define CAM_BUF 64 // power of 2, buffer must be aligned to its size
static volatile uint8_t camRx[CAM_BUF] __attribute__((aligned(CAM_BUF)));
static DMAChannel camDma;

void camera_dma_begin() {
  Serial3.begin(115200);                                  // pins + baud rate
  NVIC_DISABLE_IRQ(IRQ_LPUART2);                          // Serial3 interrupt off,
  LPUART2_CTRL &= ~(LPUART_CTRL_RIE | LPUART_CTRL_ILIE);  // DMA takes the bytes
  LPUART2_WATER = LPUART_WATER_RXWATER(0);                // DMA request for every byte

  camDma.begin(true);
  camDma.source((volatile const unsigned char&)LPUART2_DATA);
  camDma.destinationCircular(camRx, CAM_BUF);             // wraps around, runs forever
  camDma.triggerAtHardwareEvent(DMAMUX_SOURCE_LPUART2_RX);
  camDma.enable();
  LPUART2_BAUD |= LPUART_BAUD_RDMAE;                      // UART RX -> DMA requests on
}

// Parse the bytes DMA received since the last call
bool read_camera() {
  static uint8_t buf[9];
  static int n = 0;
  static uint32_t tail = 0; // next byte to parse
  bool got = false;

  uint32_t head = (uint32_t)camDma.TCD->DADDR - (uint32_t)camRx; // next byte DMA writes
  while (tail != head) {
    uint8_t b = camRx[tail];
    tail = (tail + 1) % CAM_BUF;
    if (n == 0 && b != 0xCC) continue;                      // wait for start byte
    if (n == 1 && b != 0xDD) { n = (b == 0xCC); continue; } // resync
    buf[n++] = b;
    if (n < 9) continue;

    n = 0;
    uint8_t checksum = 0;
    for (int i = 1; i <= 6; i++) checksum += buf[i];
    if (buf[8] != 0xEE || checksum != buf[7]) continue;

    topmaixData.valid      = true;
    topmaixData.goal_found = buf[2];
    topmaixData.goal_angle = buf[3] | (buf[4] << 8);
    topmaixData.goal_dist  = buf[5] | (buf[6] << 8);
    got = true;
  }
  return got;
}

// ===== Line (from sense core) =====
bool line_flag() {
  return digitalReadFast(PIN_0) || digitalReadFast(PIN_1);
}

// Packet: AA s0 s1 s2 s3 checksum (sum of s0-s3)
// Uses the shared Serial8 parser, so ball packets (BB) are no longer thrown away
bool read_line_state(uint32_t& state) {
  if (!(read_sense() & SENSE_GOT_LINE)) return false;
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
void SetMotorSpeed(uint8_t port, float speed){
  if (port < 1 || port > MOTOR_COUNT) return;
  uint8_t i = port - 1;

  speed = constrain(speed, -MOTOR_SPEED_LIMIT, MOTOR_SPEED_LIMIT);
  if (speed == 0) {
    analogWrite(motorPwmPins[i], 0);
    return;
  }
  //digitalWrite(motorDirPins[i], speed > 0 ? LOW : HIGH);
  digitalWrite(motorDirPins[i], speed < 0 ? LOW : HIGH);
  analogWrite(motorPwmPins[i], (int)(fabsf(speed) * 255.0f / 100.0f));
}

void MotorStop(){
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
  robot.robot_heading = constrain(h, 45.0f, 135.0f);
}

// P controller with deadband: rotation command to reach target heading
static float headingCorrection(float target_heading) {
  float e = wrap180(target_heading - currentHeading());
  return (fabsf(e) > robot.heading_threshold) ? e * robot.P_factor : 0.0f;
}

void RobotIKControl(float vx, float vy, float omega) {
  // Inverse kinematics: wheel speed = projection of (vx, vy) on wheel axis + omega
  SetMotorSpeed(1, -0.643f * vx + 0.766f * vy + omega);
  SetMotorSpeed(2, -0.643f * vx - 0.766f * vy + omega);
  SetMotorSpeed(3,  0.707f * vx - 0.707f * vy + omega);
  SetMotorSpeed(4,  0.707f * vx + 0.707f * vy + omega);
}

// Robot-frame motion; rot_V nudges the target heading, clamped to 45..135
void Vector_Motion(float Vx, float Vy, float rot_V) {
  robot.robot_heading = constrain(robot.robot_heading + rot_V, 45.0f, 135.0f);
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

uint8_t read_sense() {
  static uint8_t buf[8];
  static uint8_t n = 0, need = 0;
  uint8_t got = 0;

  while (Serial8.available()) {
    uint8_t b = Serial8.read();

    if (n == 0) {                      // waiting for a header
      if (b == LINE_HDR)      need = LINE_LEN;
      else if (b == BALL_HDR) need = BALL_LEN;
      else { senseLinkStats.junk++; continue; }
    }
    buf[n++] = b;
    if (n < need) continue;
    n = 0;

    if (need == LINE_LEN) {
      if (pkt_sum(buf, 1, 4) != buf[5]) { senseLinkStats.bad++; continue; }
      lineData.state = buf[1] | (buf[2] << 8) | (buf[3] << 16) | ((uint32_t)buf[4] << 24);
      lineData.t_ms = millis();
      senseLinkStats.line_pkts++;
      got |= SENSE_GOT_LINE;
    } else {
      if (pkt_sum(buf, 1, 5) != buf[6]) { senseLinkStats.bad++; continue; }
      uint16_t a = pkt_u16(buf, 2);
      irBall.found = buf[1] && a < 360;
      if (irBall.found) irBall.angle = a;
      irBall.strength = pkt_u16(buf, 4);
      irBall.t_ms = millis();
      senseLinkStats.ball_pkts++;
      got |= SENSE_GOT_BALL;
    }
  }
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

// Same DMA setup as camera_dma_begin() in core_act.cpp, larger ring for 18-byte packets
#define CAM2_BUF 128 // power of 2, buffer aligned to its size
static volatile uint8_t cam2Rx[CAM2_BUF] __attribute__((aligned(CAM2_BUF)));
static DMAChannel cam2Dma;

void camera_v2_begin() {
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

bool read_camera_v2() {
  static uint8_t buf[CAM_LEN];
  static int n = 0;
  static uint32_t tail = 0;
  static bool first = true;
  bool got = false;

  uint32_t head = (uint32_t)cam2Dma.TCD->DADDR - (uint32_t)cam2Rx;
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

static bool ballStable = false;
static bool ballRaw = false;
static uint32_t ballChange = 0;

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
  pinMode(KICK_BALL_PIN, INPUT_PULLUP);      // B6: no ball = HIGH, never floats
  kickEnter(KICK_CHARGING);
}

static void updateBall() {
  bool raw = digitalReadFast(KICK_BALL_PIN) == LOW;
  uint32_t now = millis();
  if (raw != ballRaw) { ballRaw = raw; ballChange = now; }
  if (now - ballChange >= BALL_DEBOUNCE_MS) ballStable = ballRaw;
}

void kicker_update() {
  updateBall();
  uint32_t t = millis() - kickStart;

  switch (kickSt) {
    case KICK_CHARGING:
      if (t >= CHARGE_TIMEOUT_MS) { chargeWrite(0); kickEnter(KICK_CHARGED); }
      else chargeWrite(t >= RAMP_MS ? CHARGE_DUTY : (int)((uint32_t)CHARGE_DUTY * t / RAMP_MS));
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
        kickEnter(KICK_CHARGING);
      }
      break;
  }
}

bool kicker_ready() {
  if (millis() - lastKick < MIN_KICK_GAP_MS) return false;
  if (kickSt == KICK_CHARGED) return true;
  return kickSt == KICK_CHARGING && millis() - kickStart >= KICK_MIN_CHARGE_MS;
}

bool kicker_fire() {
  if (!kicker_ready()) return false;
  chargeWrite(0);            // charge and discharge are never on together
  kickEnter(KICK_DEAD);
  return true;
}

bool has_ball() { return ballStable; }
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
  robot.heading_threshold = HEADING_DEADBAND;
  robot.robot_heading = 90.0f;
}

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
MergedBall merge_ball_data(const FieldData& field, const IrBallData& ir) {
  MergedBall m;
  bool cam = camera_fresh(CAM_TIMEOUT_MS) && field.ball.seen;
  bool irOk = ir_ball_fresh(IR_TIMEOUT_MS);

  if (cam && (!irOk || fabsf(wrap180(field.ball.angle - ir.angle)) < MERGE_MAX_ERR_DEG)) {
    m.src = SRC_CAM;
    m.angle = field.ball.angle;
    m.closeness = clamp01((ORBIT_FAR_CM - field.ball.dist) / (ORBIT_FAR_CM - ORBIT_NEAR_CM));
  } else if (irOk) {
    m.src = SRC_IR;
    m.angle = ir.angle;
    m.closeness = clamp01((ir.strength - IR_FAR_STR) / (IR_NEAR_STR - IR_FAR_STR));
  }
  return m;
}

Vec2 chase_ball(const MergedBall& ball, const FieldData& field) {
  if (ball.src == SRC_NONE) {
    // No ball anywhere: go home toward our own goal, stop when close
    const FieldObj& g = field.back_goal;
    if (camera_fresh(CAM_TIMEOUT_MS) && g.seen && g.dist > HOME_DIST_CM)
      return polar(HOME_SPEED, g.angle);
    return Vec2();
  }

  float diff = wrap180(ball.angle - 90.0f);        // 0 = ball straight ahead
  if (fabsf(diff) <= CHASE_FRONT_DEG) return polar(CHASE_SPEED, ball.angle);

  // Ball to the side or behind: aim past it so the robot circles behind the ball.
  // Closer ball -> bigger offset.
  float offset = min(CHASE_OFFSET_MAX, CHASE_OFFSET_GAIN * fabsf(diff)) * ball.closeness;
  float dir = ball.angle + (diff > 0 ? offset : -offset);
  return polar(CHASE_SPEED, dir);
}

// ===== Heading =====
float rotate(const FieldData& field) {
  const FieldObj& g = field.front_goal;
  if (!camera_fresh(CAM_TIMEOUT_MS) || !g.seen) return 90.0f;
  // Turning CCW by d moves an object's angle by -d, so turn by (goal - 90)
  return current_heading() + wrap180(g.angle - 90.0f);
}

// Short turn when kicking
static uint32_t kickRotStart = 0;
static bool kickRotOn = false;
static float kickRotTarget = 90.0f;

float kick_rotate(bool kicked, const FieldData& field) {
  if (kicked) {
    float t = rotate(field);                         // 90 if the front goal is not seen
    float d = constrain(wrap180(t - current_heading()), -KICK_ROTATE_MAX_DEG, KICK_ROTATE_MAX_DEG);
    kickRotTarget = current_heading() + d;
    kickRotStart = millis();
    kickRotOn = true;
  }
  if (kickRotOn && millis() - kickRotStart >= KICK_ROTATE_MS) kickRotOn = false;
  return kickRotOn ? kickRotTarget : 90.0f;
}

bool kick_rotating() { return kickRotOn; }

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

  if (!camera_fresh(CAM_TIMEOUT_MS)) return v;
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
  return s == SRC_CAM ? "CAM" : (s == SRC_IR ? "IR" : "NONE");
}
