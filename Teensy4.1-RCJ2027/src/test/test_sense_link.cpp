#include <Arduino.h>
#include <core_act.h>

// T02 (Act): Serial8 parser for the combined line + ball packet (AA, 11 bytes)
// and the ultrasonic packet (AB, 11 bytes).
// Sense runs test_sense_tx (no ultrasonic) or sense (with ultrasonic). Motors never move in this test.
// Pass: off the line ~100 pkts/s, on the line many more; bad 0; flag->line packet < 2000 us;
//       ball angle keeps sweeping 0-359 also while on the line
//       us/s ~33 with sense.cpp; F / R / B / L follow a board held at 10, 30, 100 cm (0 = no reading)

void setup() {
  core_act_init();
  MotorStop();
  drawMessage("T02 link");
  Serial.println("T02 test_sense_link");
}

void loop() {
  static uint32_t flagRiseUs = 0, worstLatency = 0;
  static bool lastFlag = false, waiting = false;

  bool flag = line_flag();
  if (flag && !lastFlag) { flagRiseUs = micros(); waiting = true; }
  lastFlag = flag;

  uint8_t got = read_sense();
  if ((got & SENSE_GOT_LINE) && waiting) {
    uint32_t lat = micros() - flagRiseUs;
    if (lat > worstLatency) worstLatency = lat;
    waiting = false;
  }
  digitalWriteFast(LED_BUILTIN, flag);

  static SenseLinkStats prev;
  EVERY_MS(tp, 1000) {
    SenseLinkStats s = senseLinkStats;
    Serial.printf("pkts/s %5lu | with line %5lu | bad %lu | junk bytes %lu | flag->pkt worst %lu us | "
                  "ball %s %3.0f deg str %u | line %08lX | us/s %3lu F %5.1f R %5.1f B %5.1f L %5.1f cm\n",
                  s.pkts - prev.pkts, s.line_pkts - prev.line_pkts, s.bad, s.junk,
                  worstLatency, irBall.found ? "seen" : "none", irBall.angle, irBall.strength,
                  lineData.state, s.us_pkts - prev.us_pkts,
                  senseUs.cm[US_N_FRONT], senseUs.cm[US_N_RIGHT], senseUs.cm[US_N_BACK], senseUs.cm[US_N_LEFT]);
    prev = s;
    worstLatency = 0;
  }
}
