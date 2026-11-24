#include <core_sense.h>
#include <EEPROM.h>

uint32_t lsUsedMask = 0xFFFFFFFFUL;
bool lsUseMux2 = true;

void line_sensor_config(bool useMux2, uint32_t usedMask) {
    lsUseMux2 = useMux2;
    lsUsedMask = useMux2 ? usedMask : (usedMask & 0x0000FFFFUL);
}

// out 1-20 -> pin (sense_ir.cpp)
const uint8_t IR_PINS[IR_COUNT] = {
    12, 11, 10,  9,  8,  7,  6,  5,  4,  3, // out 1-10
     2, 23, 22, 21, 30, 29, 39, 38, 32, 33  // out 11-20
};
uint16_t irValue[IR_COUNT];

uint16_t lsRaw[32];
uint16_t avg_ls[32];
uint32_t lineState = 0xFFFFFFFF;

void core_sense_init() {
    Serial.begin(115200);
    Serial8.begin(SENSE_BAUD); // to act core, 921600, same constant on both boards
    pinMode(PIN_0, OUTPUT);
    pinMode(PIN_1, OUTPUT);
    digitalWrite(PIN_0, LOW);
    digitalWrite(PIN_1, LOW);
    pinMode(LINE_SCAN_BUTTON, INPUT_PULLUP);
    for (int i = 0; i < IR_COUNT; i++) {
        pinMode(IR_PINS[i], INPUT);
    }
    pinMode(LS_BRIGHTNESS, OUTPUT);
    analogWriteFrequency(LS_BRIGHTNESS, 200000); // 200 kHz, much faster than one ADC read

    pinMode(LS_S0, OUTPUT);
    pinMode(LS_S1, OUTPUT);
    pinMode(LS_S2, OUTPUT);
    pinMode(LS_S3, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(LS_M1, INPUT);
    pinMode(LS_M2, INPUT);

    // Load thresholds saved by calibrate_line_sensor(); erased EEPROM reads 0xFFFF
    // M1 = avg_ls[0-15], M2 = avg_ls[16-31] (saved by mux2_test)
    EEPROM.get(0, avg_ls);
    for (int i = 0; i < 32; i++) {
        if (avg_ls[i] == 0xFFFF) avg_ls[i] = 512;
    }
}

// Count LOW samples on each sensor over IR_WINDOW_US (one ball pulse cycle is ~833 us)
void update_ball_sensor() {
    for (int i = 0; i < IR_COUNT; i++) irValue[i] = 0;

    uint32_t start = micros();
    while (micros() - start < IR_WINDOW_US) {
        for (int i = 0; i < IR_COUNT; i++) {
            if (digitalRead(IR_PINS[i]) == LOW) irValue[i]++;
        }
    }
}

void fast_update_line_sensor() {
    static uint32_t prevRaw = 0xFFFFFFFF;
    uint32_t rawState       = 0xFFFFFFFF;

    for (uint8_t ch = 0; ch < 16; ch++) {
        digitalWriteFast(LS_S0, (ch >> 0) & 1);
        digitalWriteFast(LS_S1, (ch >> 1) & 1);
        digitalWriteFast(LS_S2, (ch >> 2) & 1);
        digitalWriteFast(LS_S3, (ch >> 3) & 1);
        // Mux settle time: sample the IR sensors here instead of waiting idle
        uint32_t t0 = micros();
        ir_sample();
        while (micros() - t0 < 10) {}

        uint16_t r1 = analogRead(LS_M1);
        lsRaw[ch] = r1;
        if (r1 < avg_ls[ch]) rawState &= ~(1UL << ch);

        if (lsUseMux2) {                       // attack robot: mux 2 not read at all
            uint16_t r2 = analogRead(LS_M2);
            lsRaw[ch + 16] = r2;
            if (r2 < avg_ls[ch+16]) rawState &= ~(1UL << (ch + 16));
        }
    }

    // Bit clears only if below threshold in this scan AND the previous one
    lineState = prevRaw | rawState | ~lsUsedMask;
    prevRaw   = rawState;
}

// Call every loop after fast_update_line_sensor().
// PIN_0/PIN_1 HIGH when a line is seen, then line + newest ball packet (every call)
// Returns true if a packet was sent
bool send_line_state() {
    return send_sense_packet(lineState, irBallLast);
}

IrBall irBallLast;
void update_ir_ball() { irBallLast = compute_ir_ball(); }

// Call every loop after fast_update_line_sensor().
// 1st press: LED13 blinks, record min/max of each sensor
// 2nd press: LED13 off, save avg_ls to EEPROM, show result for 2 s
// Returns true while scanning
bool calibrate_line_sensor() {
    static bool scanning = false;
    static bool lastBtn = HIGH;
    static uint32_t lastPress = 0;
    static uint32_t blinkTimer = 0;
    static uint16_t max_ls[32], min_ls[32];

    bool btn = digitalRead(LINE_SCAN_BUTTON);
    if (lastBtn == HIGH && btn == LOW && millis() - lastPress > 200) { // pressed, 200 ms debounce
        lastPress = millis();
        scanning = !scanning;

        if (scanning) {
            for (int i = 0; i < 32; i++) {
                max_ls[i] = 0;
                min_ls[i] = 0xFFFF;
            }
            Serial.println("Line scan start");
        } else {
            digitalWrite(LED_BUILTIN, LOW);
            for (int i = 0; i < 32; i++) {
                if (!(lsUsedMask & (1UL << i))) continue;  // no sensor on this channel
                avg_ls[i] = (max_ls[i] + min_ls[i]) / 2;
                Serial.printf("Sensor %d: min=%d, max=%d, avg=%d\n", i, min_ls[i], max_ls[i], avg_ls[i]);
            }
            EEPROM.put(0, avg_ls);
            Serial.println("Line scan saved");
            delay(2000); // keep result on screen
        }
    }
    lastBtn = btn;

    if (scanning) {
        for (int i = 0; i < 32; i++) { // both muxes
            if (lsRaw[i] > max_ls[i]) max_ls[i] = lsRaw[i];
            if (lsRaw[i] < min_ls[i]) min_ls[i] = lsRaw[i];
        }

        // Blink LED and print raw data every 100 ms
        if (millis() - blinkTimer > 50) {
            blinkTimer = millis();
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            Serial.print("RAW:");
            for (int i = 0; i < LS_COUNT; i++) {
                Serial.print(' ');
                Serial.print(lsRaw[i]);
            }
            Serial.println();
        }
    }
    return scanning;
}

// =====================================================================
// Sending to the act core
// =====================================================================
SenseTxStats senseTxStats;

// ----- IR ball (sense_ir.cpp) -----
struct IrSensor { uint8_t out; int16_t deg; };
static const IrSensor IR_NEAR[] = {
  {2, 0}, {4, 45}, {5, 60}, {6, 85}, {8, 95}, {9, 120}, {10, 135},
  {12, 180}, {14, 230}, {16, 260}, {18, 280}, {20, 310}
};
static const IrSensor IR_FAR[] = {
  {1, 0}, {3, 45}, {7, 90}, {11, 135}, {13, 180}, {15, 225}, {17, 270}, {19, 315}
};
static const int IR_NEAR_N = sizeof(IR_NEAR) / sizeof(IR_NEAR[0]);
static const int IR_FAR_N  = sizeof(IR_FAR)  / sizeof(IR_FAR[0]);

// Ball angle from one group, -1 if no sensor in the group sees the ball.
// Vector sum of the strongest sensor + its two neighbours (group sorted by angle), count-weighted
static float irGroupAngle(const IrSensor* g, int n) {
  int k = 0;
  uint16_t best = 0;
  for (int i = 0; i < n; i++) {
    uint16_t c = irValue[g[i].out - 1];
    if (c > best) { best = c; k = i; }
  }
  if (best <= IR_MIN) return -1;

  float x = 0, y = 0;
  for (int d = -1; d <= 1; d++) {
    const IrSensor& s = g[(k + d + n) % n];
    float c = irValue[s.out - 1];
    x += c * cosf(s.deg * DEG_TO_RAD);
    y += c * sinf(s.deg * DEG_TO_RAD);
  }
  float a = atan2f(y, x) * RAD_TO_DEG;
  return a < 0 ? a + 360.0f : a;
}

static IrBall irFromCounts() {
  IrBall b;
  float angle = irGroupAngle(IR_NEAR, IR_NEAR_N);          // near first
  if (angle < 0) angle = irGroupAngle(IR_FAR, IR_FAR_N);   // else far
  if (angle < 0) return b;
  b.found = true;
  b.angle = (uint16_t)lroundf(angle) % 360;
  for (int i = 0; i < IR_COUNT; i++) if (irValue[i] > IR_MIN) b.strength++;
  return b;
}

#if IR_NONBLOCKING
static uint16_t irAcc[IR_COUNT];
static uint32_t irWinStart = 0;
#endif

void ir_sample() {
#if IR_NONBLOCKING
  for (int i = 0; i < IR_COUNT; i++) if (digitalReadFast(IR_PINS[i]) == LOW) irAcc[i]++;
#endif
}

IrBall compute_ir_ball() {
#if IR_NONBLOCKING
  static IrBall last;
  if (micros() - irWinStart < IR_WINDOW_US) return last;   // window still open
  for (int i = 0; i < IR_COUNT; i++) { irValue[i] = irAcc[i]; irAcc[i] = 0; }
  irWinStart = micros();
  last = irFromCounts();
  return last;
#else
  update_ball_sensor();                                    // 1 ms, same as sense_ir.cpp
  return irFromCounts();
#endif
}

bool send_sense_packet(uint32_t state, const IrBall& ball) {
  bool line = state != 0xFFFFFFFF;
  digitalWriteFast(PIN_0, line);       // interrupt to the act core first
  digitalWriteFast(PIN_1, line);

  if (Serial8.availableForWrite() < SENSE_LEN) { senseTxStats.skipped_full++; return false; }

  uint8_t p[SENSE_LEN] = {SENSE_HDR,
                          (uint8_t)(state), (uint8_t)(state >> 8),
                          (uint8_t)(state >> 16), (uint8_t)(state >> 24),
                          (uint8_t)ball.found, 0, 0, 0, 0, 0};
  pkt_put16(p, 6, ball.found ? ball.angle % 360 : PKT_NONE);
  pkt_put16(p, 8, ball.found ? ball.strength : 0);
  p[10] = pkt_sum(p, 1, 9);
  Serial8.write(p, SENSE_LEN);
  senseTxStats.pkts++;
  if (line) senseTxStats.line_pkts++;
  return true;
}

bool send_us_packet() {
  if (Serial8.availableForWrite() < SENSE_LEN) return false;
  uint8_t p[SENSE_LEN] = {SENSE_US_HDR};
  for (int i = 0; i < US_COUNT; i++)
    pkt_put16(p, 1 + 2 * i, (uint16_t)constrain(lroundf(usDistCm[i] * 10.0f), 0L, 65535L));
  p[10] = pkt_sum(p, 1, 9);
  Serial8.write(p, SENSE_LEN);
  return true;
}

// ===== Ultrasonic (R03) =====
const uint8_t US_TRIG_PINS[US_COUNT] = {25, 37, 40, 41};   // H1-H4
const uint8_t US_ECHO_PINS[US_COUNT] = {26, 27, 28, 31};
float usDistCm[US_COUNT];
float usRawCm[US_COUNT];
uint32_t usReadings[US_COUNT], usMisses[US_COUNT];

static volatile uint32_t usEchoStart[US_COUNT], usEchoLen[US_COUNT];
static volatile bool usEchoDone[US_COUNT];
static bool usPending[US_COUNT];       // fired, echo not finished yet
static uint8_t usMissRun[US_COUNT];    // misses in a row
static uint8_t usCur = US_COUNT - 1;
static uint32_t usLastTrig = 0;

static void usEdge(uint8_t i) {
  if (digitalReadFast(US_ECHO_PINS[i])) usEchoStart[i] = micros();
  else { usEchoLen[i] = micros() - usEchoStart[i]; usEchoDone[i] = true; }
}
static void usIsr0() { usEdge(0); }
static void usIsr1() { usEdge(1); }
static void usIsr2() { usEdge(2); }
static void usIsr3() { usEdge(3); }

static void usMiss(uint8_t i) {
  usRawCm[i] = 0;
  usMisses[i]++;
  if (usMissRun[i] < US_INVALID_LIMIT) usMissRun[i]++;
  if (usMissRun[i] >= US_INVALID_LIMIT) usDistCm[i] = 0;
}

void us_begin() {
  void (*isr[US_COUNT])() = {usIsr0, usIsr1, usIsr2, usIsr3};
  for (int i = 0; i < US_COUNT; i++) {
    pinMode(US_TRIG_PINS[i], OUTPUT);
    digitalWriteFast(US_TRIG_PINS[i], LOW);
    pinMode(US_ECHO_PINS[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(US_ECHO_PINS[i]), isr[i], CHANGE);
  }
}

bool us_update() {
  bool fresh = false;
  for (int i = 0; i < US_COUNT; i++) {          // finished echoes
    if (!usEchoDone[i]) continue;
    noInterrupts();
    uint32_t len = usEchoLen[i];
    usEchoDone[i] = false;
    interrupts();
    if (!usPending[i]) continue;                 // edge from a sensor we did not fire: ignore
    usPending[i] = false;
    fresh = true;
    if (len > US_MIN_ECHO_US && len < US_MAX_ECHO_US) {
      float cm = len * 0.0343f / 2.0f;
      usRawCm[i] = cm;
      usDistCm[i] = usDistCm[i] > 0 ? usDistCm[i] * (1.0f - US_FILTER_ALPHA) + cm * US_FILTER_ALPHA : cm;
      usMissRun[i] = 0;
      usReadings[i]++;
    } else {
      usMiss(i);
    }
  }
  if (millis() - usLastTrig >= US_PERIOD_MS) {   // next sensor
    if (usPending[usCur]) { usPending[usCur] = false; usMiss(usCur); fresh = true; }   // no echo at all
    usCur = (usCur + 1) % US_COUNT;
    usLastTrig = millis();
    usEchoDone[usCur] = false;
    usPending[usCur] = true;
    digitalWriteFast(US_TRIG_PINS[usCur], HIGH); // 10 us trigger pulse
    delayMicroseconds(10);
    digitalWriteFast(US_TRIG_PINS[usCur], LOW);
  }
  return fresh;
}
