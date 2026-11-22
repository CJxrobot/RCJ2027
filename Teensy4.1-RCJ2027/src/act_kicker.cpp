#include <Arduino.h>

// Kicker test (act core), library not used
// 12 V -> XL6019 boost -> IRF5305 P-MOS (pin 33) -> 20000 uF cap -> solenoid, discharge (pin 32)
// Charge: PWM on the P-MOS, duty ramps 0 -> CHARGE_DUTY over RAMP_MS (soft charge, no big inrush),
//         then held until CHARGE_MS, then P-MOS off -> READY
// Kick:   charge off, short dead time, then discharge pin on for KICK_MS
// Charge and discharge are never on at the same time
//
// Serial commands: c = charge, k = kick, x = stop everything
// Button (pin 27): ready -> kick, otherwise -> start charging
//
// NOTE: pin 33 shares FlexPWM2.0 with pin 4 (motor 3 PWM),
//       analogWriteFrequency(33) also changes motor 3's PWM frequency

#define CHARGE_PIN    33
#define DISCHARGE_PIN 32
#define BUTTON_PIN    27   // pressed = LOW

#define CHARGE_ON_HIGH 1    // 1: pin HIGH = P-MOS on (NPN / N-MOS gate driver), 0: pin LOW = on
#define CHARGE_FREQ   1000  // Hz, low enough for the IRF5305 gate to switch fully
#define CHARGE_DUTY   255   // duty after the ramp, 0-255 (255 = fully on)
#define RAMP_MS       1000  // soft charge: time to ramp duty from 0 to CHARGE_DUTY
#define CHARGE_MS     10000  // total charge time, then P-MOS off
#define DEAD_MS       2     // charge off -> wait -> discharge on
#define KICK_MS       10    // discharge pulse length

enum State { IDLE, CHARGING, READY };
State state = IDLE;
uint32_t chargeStart = 0;
int duty = -1; // -1 = not set yet

// Charge P-MOS, duty 0 = off, 255 = fully on, between = PWM
void chargeWrite(int d) {
  if (d == duty) return;
  duty = d;
  if (d <= 0 || d >= 255) {             // steady level, no PWM
    bool on = d >= 255;
    pinMode(CHARGE_PIN, OUTPUT);
    digitalWrite(CHARGE_PIN, on == CHARGE_ON_HIGH ? HIGH : LOW);
  } else {
    analogWrite(CHARGE_PIN, CHARGE_ON_HIGH ? d : 255 - d);
  }
}

void stopAll() {
  chargeWrite(0);
  digitalWrite(DISCHARGE_PIN, LOW);
  state = IDLE;
  Serial.println("STOP");
}

void startCharge() {
  digitalWrite(DISCHARGE_PIN, LOW); // never both on
  chargeStart = millis();
  state = CHARGING;
  Serial.println("CHARGE start");
}

void kick() {
  chargeWrite(0);
  delay(DEAD_MS);
  digitalWrite(DISCHARGE_PIN, HIGH);
  delay(KICK_MS);
  digitalWrite(DISCHARGE_PIN, LOW);
  state = IDLE;
  Serial.println("KICK");
}

void setup() {
  // Both off before anything else
  pinMode(DISCHARGE_PIN, OUTPUT);
  digitalWrite(DISCHARGE_PIN, LOW);
  chargeWrite(0);
  analogWriteFrequency(CHARGE_PIN, CHARGE_FREQ);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("Kicker test: c = charge, k = kick, x = stop, button = charge / kick");
}

void loop() {

  // Serial commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') startCharge();
    else if (c == 'k') kick();
    else if (c == 'x') stopAll();
  }

  // Button: ready -> kick, otherwise -> charge
  static bool lastBtn = HIGH;
  static uint32_t lastPress = 0;
  bool btn = digitalRead(BUTTON_PIN);
  if (lastBtn == HIGH && btn == LOW && millis() - lastPress > 200) { // 200 ms debounce
    lastPress = millis();
    if (state == READY) kick();
    else if (state == IDLE) startCharge();
  }
  lastBtn = btn;

  // Charging: ramp duty, hold, then off
  if (state == CHARGING) {
    uint32_t t = millis() - chargeStart;
    if (t >= CHARGE_MS) {
      chargeWrite(0);
      state = READY;
      Serial.println("READY");
    } else if (t >= RAMP_MS) {
      chargeWrite(CHARGE_DUTY);
    } else {
      chargeWrite((int)((uint32_t)CHARGE_DUTY * t / RAMP_MS));
    }
  }

  // Status every 200 ms
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    const char* names[] = {"IDLE", "CHARGING", "READY"};
    Serial.printf("%s  duty %d/255  t %lu ms\n", names[state], duty,
                  state == CHARGING ? millis() - chargeStart : 0UL);
  }
}
