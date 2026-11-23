#include <core_act.h>

static KickState state = KICK_CHARGING;
static uint32_t stateStart = 0;   // millis() when the state began
static uint32_t pulseStart = 0;   // micros() when DEAD / PULSE began
static uint32_t lastKick = 0;
static uint32_t kicks = 0;
static int duty = -1;

static bool ballStable = false;
static bool ballRaw = false;
static uint32_t ballChange = 0;

static void chargeWrite(int d) {
  if (d == duty) return;
  duty = d;
  if (d <= 0 || d >= 255) {
    bool on = d >= 255;
    pinMode(KICK_CHARGE_PIN, OUTPUT);
    digitalWriteFast(KICK_CHARGE_PIN, on == KICK_CHARGE_ON_HIGH ? HIGH : LOW);
  } else {
    analogWrite(KICK_CHARGE_PIN, KICK_CHARGE_ON_HIGH ? d : 255 - d); // default frequency
  }
}

static void enter(KickState s) {
  state = s;
  stateStart = millis();
  pulseStart = micros();
}

void kicker_begin() {
  pinMode(KICK_DISCHARGE_PIN, OUTPUT);
  digitalWriteFast(KICK_DISCHARGE_PIN, LOW); // discharge off before anything else
  duty = -1;
  chargeWrite(0);
  pinMode(KICK_BALL_PIN, INPUT_PULLUP);      // B6: no ball = HIGH, never floats
  enter(KICK_CHARGING);
}

static void updateBall() {
  bool raw = digitalReadFast(KICK_BALL_PIN) == LOW;
  uint32_t now = millis();
  if (raw != ballRaw) { ballRaw = raw; ballChange = now; }
  if (now - ballChange >= BALL_DEBOUNCE_MS) ballStable = ballRaw;
}

void kicker_update() {
  updateBall();
  uint32_t t = millis() - stateStart;

  switch (state) {
    case KICK_CHARGING:
      if (t >= CHARGE_TIMEOUT_MS) { chargeWrite(0); enter(KICK_CHARGED); }
      else chargeWrite(t >= RAMP_MS ? CHARGE_DUTY : (int)((uint32_t)CHARGE_DUTY * t / RAMP_MS));
      break;
    case KICK_CHARGED:
      break;
    case KICK_DEAD:   // charge is off, wait before opening the discharge
      if (micros() - pulseStart >= DEAD_MS * 1000UL) {
        digitalWriteFast(KICK_DISCHARGE_PIN, HIGH);
        pulseStart = micros();
        state = KICK_PULSE;
      }
      break;
    case KICK_PULSE:
      if (micros() - pulseStart >= KICK_MS * 1000UL) {
        digitalWriteFast(KICK_DISCHARGE_PIN, LOW);
        lastKick = millis();
        kicks++;
        enter(KICK_CHARGING);
      }
      break;
  }
}

bool kicker_ready() {
  if (millis() - lastKick < MIN_KICK_GAP_MS) return false;
  if (state == KICK_CHARGED) return true;
  return state == KICK_CHARGING && millis() - stateStart >= KICK_MIN_CHARGE_MS;
}

bool kicker_fire() {
  if (!kicker_ready()) return false;
  chargeWrite(0);            // charge and discharge are never on together
  enter(KICK_DEAD);
  return true;
}

bool has_ball() { return ballStable; }
KickState kicker_state() { return state; }
uint32_t kicker_count() { return kicks; }

const char* kicker_state_name() {
  switch (state) {
    case KICK_CHARGING: return "CHARGING";
    case KICK_CHARGED:  return "CHARGED";
    case KICK_DEAD:     return "DEAD";
    default:            return "PULSE";
  }
}
