#include <Arduino.h>

// Kicker with ball possession (act core), library not used
// 12 V -> XL6019 boost -> IRF5305 P-MOS (pin 33) -> 20000 uF cap -> solenoid, discharge (pin 32)
// Ball possession sensor pin 38: HIGH = no ball, LOW = ball
//
// Charge -> (charge timeout) -> charged -> ball -> kick -> charge -> ... keep kicking
//   Charging:  ball ignored, charge until CHARGE_TIMEOUT_MS, then charging stops
//   Charged:   wait for the ball, then kick and start charging again
//
// Charge: PWM on the P-MOS, duty ramps 0 -> CHARGE_DUTY over RAMP_MS (soft charge), then stays on
// Kick:   charge off, short dead time, then discharge pin on for KICK_MS
// Charge and discharge are never on at the same time
//
// NOTE: pin 33 shares FlexPWM2.0 with pin 4 (motor 3 PWM),
//       analogWriteFrequency(33) also changes motor 3's PWM frequency

#define CHARGE_PIN    33
#define DISCHARGE_PIN 32
#define BALL_PIN      38   // LOW = ball

#define CHARGE_ON_HIGH    1     // 1: pin HIGH = P-MOS on, 0: pin LOW = on
#define CHARGE_FREQ       1000  // Hz
#define CHARGE_DUTY       255   // duty after the ramp, 0-255 (255 = fully on)
#define RAMP_MS           1000  // soft charge: time to ramp duty from 0 to CHARGE_DUTY
#define CHARGE_TIMEOUT_MS 5000  // charge time, then charging stops
#define DEAD_MS           2     // charge off -> wait -> discharge on
#define KICK_MS           10    // discharge pulse length
#define BALL_MS           20    // pin must stay the same this long (debounce)
#define MIN_KICK_GAP_MS   500   // last kick -> this kick shorter than this: no kick

bool charging = true;
uint32_t chargeStart = 0;
uint32_t lastKick = 0;
int duty = -1; // -1 = not set yet

// Charge P-MOS, duty 0 = off, 255 = fully on, between = PWM
void chargeWrite(int d) {
  if (d == duty) return;
  duty = d;
  Serial.printf("chargeWrite %d\n", d);
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
  pinMode(BALL_PIN, INPUT_PULLUP); // no ball = HIGH, pin never floats

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
  if (ball && millis() - lastKick >= MIN_KICK_GAP_MS) { // < 300 ms since last kick: stop, no kick
    kick();        // ball -> kick
    lastKick = millis();
    startCharge(); // then charge again
  }
}
