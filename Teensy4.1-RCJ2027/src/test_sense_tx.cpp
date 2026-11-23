#include <Arduino.h>
#include <core_sense.h>

// T01 (Sense): ONE packet with line + IR ball (AA, 11 bytes) on Serial8 at 921600.
//   no line:   ~SENSE_TX_HZ (100) packets/s
//   line seen: a packet every loop, each still carrying the ball
// Ball data is a FAKE sweep until new_ir_sensing (R02).
//
// Pass: off the line -> ~100 pkts/s, with line 0/s
//       on the line  -> pkts/s = loops/s, with line = pkts/s, tx full 0

static IrBall fakeBall() {
  static uint16_t angle = 0;
  static uint32_t last = 0;
  IrBall b;
  if (millis() - last >= 10) { last = millis(); angle = (angle + 3) % 360; }
  b.found = true;
  b.angle = angle;
  b.strength = 300;
  return b;
}

void setup() {
  core_sense_init();
  analogWrite(LS_BRIGHTNESS, 155);
  Serial.println("T01 test_sense_tx: line + fake IR ball in one packet");
}

void loop() {
  static uint32_t loops = 0, lastPrint = 0;
  static SenseTxStats prev;

  fast_update_line_sensor();
  send_sense_packet(lineState, fakeBall());
  digitalWriteFast(LED_BUILTIN, lineState != 0xFFFFFFFF);
  loops++;

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    SenseTxStats s = senseTxStats;
    Serial.printf("loops/s %5lu | pkts/s %5lu | with line %5lu | tx full %lu\n",
                  loops, s.pkts - prev.pkts, s.line_pkts - prev.line_pkts,
                  s.skipped_full - prev.skipped_full);
    prev = s;
    loops = 0;
  }
}
