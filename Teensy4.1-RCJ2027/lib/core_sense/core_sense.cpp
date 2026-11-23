#include <core_sense.h>
#include <EEPROM.h>

const uint8_t IR_PINS[IR_COUNT] = {
    2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, // port 1  - 11
    23, 22, 21,                          // port 12 - 14
    28, 29                               // port 15 - 16
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

// Count LOW samples on each sensor over 1 ms (one ball pulse cycle is ~833 us)
void update_ball_sensor() {
    for (int i = 0; i < IR_COUNT; i++) irValue[i] = 0;

    uint32_t start = micros();
    while (micros() - start < 1000) {
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
        uint16_t r2 = analogRead(LS_M2);
        lsRaw[ch] = r1;
        lsRaw[ch + 16] = r2;

        if (r1 < avg_ls[ch])    rawState &= ~(1UL << ch);
        if (r2 < avg_ls[ch+16]) rawState &= ~(1UL << (ch + 16));
    }

    // Bit clears only if below threshold in this scan AND the previous one
    lineState = prevRaw | rawState | ~LS_USED_MASK;
    prevRaw   = rawState;
}

// Call every loop after fast_update_line_sensor().
// Line seen: PIN_0/PIN_1 HIGH, line + ball packet every call
// No line:   PIN_0/PIN_1 LOW, line + ball packet at SENSE_TX_HZ
// Returns true if a packet was sent
bool send_line_state() {
    return send_sense_packet(lineState, compute_ir_ball());
}

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
                if (!(LS_USED_MASK & (1UL << i))) continue; // no sensor on this channel
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

// Placeholders until new_ir_sensing is added (R02)
__attribute__((weak)) void ir_sample() {}
__attribute__((weak)) IrBall compute_ir_ball() { return IrBall(); }

bool send_sense_packet(uint32_t state, const IrBall& ball) {
  static uint32_t last = 0;
  bool line = state != 0xFFFFFFFF;
  digitalWriteFast(PIN_0, line);
  digitalWriteFast(PIN_1, line);

  uint32_t now = micros();
  if (!line && now - last < 1000000UL / SENSE_TX_HZ) return false;   // no line: 100 Hz
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
  last = now;
  return true;
}
