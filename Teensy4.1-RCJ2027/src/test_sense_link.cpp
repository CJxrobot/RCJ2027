#include <Arduino.h>
#include <core_act.h>

// T02 (Act): one Serial8 parser for line (AA) and ball (BB) packets.
// Sense runs test_sense_tx. Motors never move in this test.
// Pass: ball ~100/s, bad 0, flag->line packet latency < 2000 us, ball angle sweeps 0-359

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
    Serial.printf("line/s %5lu | ball/s %4lu | bad %lu | junk bytes %lu | flag->pkt worst %lu us | "
                  "ball %s %3.0f deg str %u | line %08lX\n",
                  s.line_pkts - prev.line_pkts, s.ball_pkts - prev.ball_pkts, s.bad, s.junk,
                  worstLatency, irBall.found ? "seen" : "none", irBall.angle, irBall.strength,
                  lineData.state);
    prev = s;
    worstLatency = 0;
  }
}
