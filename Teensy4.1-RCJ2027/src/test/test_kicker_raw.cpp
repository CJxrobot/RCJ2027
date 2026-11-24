#include <Arduino.h>

// Kicker + ball possession hardware test (act core). Library NOT used: checks the circuit and
// the ball sensor alone, motors never touched. The game version is test_kicker (core_act kicker).
// 12 V -> XL6019 boost -> IRF5305 P-MOS (pin 33) -> 20000 uF cap -> solenoid, discharge (pin 32)
// Ball possession sensor pin 38: HIGH = no ball, LOW = ball (INPUT, as tested on the robot)
//
// Charge -> (charge timeout) -> charged -> ball -> kick -> charge -> ... keep kicking
//   Charging:  ball ignored, charge until CHARGE_TIMEOUT_MS, then charging stops
//   Charged:   wait for the ball, then kick and start charging again
//
// Charge: PWM on the P-MOS, duty ramps 0 -> CHARGE_DUTY over RAMP_MS (soft charge), then stays on
// Kick:   charge off, short dead time, then discharge pin on for KICK_MS
// Charge and discharge are never on at the same time
//
// NOTE: this test uses hardware PWM on pin 33 (shares FlexPWM2.0 with motor 3, pin 4).
//       Fine here because the motors are off. The game code (core_act) uses software PWM instead.

#define CHARGE_PIN    33
#define DISCHARGE_PIN 32
#define BALL_PIN      38   // LOW = ball

#define CHARGE_ON_HIGH    1     // 1: pin HIGH = P-MOS on, 0: pin LOW = on
#define CHARGE_FREQ       1000  // Hz
#define CHARGE_DUTY       255   // duty after the ramp, 0-255 (255 = fully on)
#define RAMP_MS           1000  // soft charge: time to ramp duty from 0 to CHARGE_DUTY
#define CHARGE_TIMEOUT_MS 5000  // charge time, then charging stops
#define DEAD_MS           2     // charge off -> wait -> discharge on
#define KICK_MS           15    // discharge pulse length (same as core_act)
#define MIN_KICK_GAP_MS   500   // last kick -> this kick shorter than this: no kick

bool charging = true;
uint32_t chargeStart = 0;
uint32_t lastKick = 0;
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

void startCharge() {
  charging = true;
  chargeStart = millis();
  Serial.println("CHARGE");
}

void kick() {
  chargeWrite(0);
  delay(DEAD_MS);
  digitalWrite(DISCHARGE_PIN, HIGH);
  delay(KICK_MS);
  digitalWrite(DISCHARGE_PIN, LOW);
  Serial.println("KICK");
}



void setup() {
  // Both off before anything else
  pinMode(DISCHARGE_PIN, OUTPUT);
  digitalWrite(DISCHARGE_PIN, LOW);
  chargeWrite(0);
  analogWriteFrequency(CHARGE_PIN, CHARGE_FREQ);
  pinMode(BALL_PIN, INPUT);        // same as core_act kicker_begin()

  Serial.begin(115200);
  Serial.println("Kicker + ball: charge -> ball -> kick -> charge ...");
  startCharge();
}

void loop() {
  bool ball = digitalRead(BALL_PIN) == LOW; // LOW = ball

  if (charging) {
    // charge: soft ramp, stay on, stop at timeout
    uint32_t t = millis() - chargeStart;
    if (t >= CHARGE_TIMEOUT_MS) {
      chargeWrite(0);
      charging = false;
      Serial.println("CHARGED");
    } else {
      chargeWrite(t >= RAMP_MS ? CHARGE_DUTY : (int)((uint32_t)CHARGE_DUTY * t / RAMP_MS));
    }
  } 
  if (ball && millis() - lastKick >= MIN_KICK_GAP_MS) { // less than MIN_KICK_GAP_MS since the last kick: no kick
    kick();        // ball -> kick
    lastKick = millis();
    startCharge(); // then charge again
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.printf("%s  duty %d/255  pin38 %d (%s)\n", charging ? "CHARGING" : "CHARGED", duty,
                  digitalRead(BALL_PIN), ball ? "ball" : "no ball");
  }
}
