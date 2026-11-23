#include <Arduino.h>
#include <core_sense.h>

// T01 (Sense): line packet first, then IR ball packet at up to BALL_TX_HZ, also on a line.
// Ball data is a FAKE sweep until new_ir_sensing (R02).
//
// Pass:  off the line -> ball ~100/s, line 0/s
//        on the line  -> line every loop AND ball still ~100/s, tx full 0

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
  Serial.println("T01 test_sense_tx: fake IR ball sweep + real line");
}

void loop() {
  static uint32_t loops = 0, lastPrint = 0;
  static SenseTxStats prev;

  fast_update_line_sensor();
  bool lineSent = send_line_state();
  send_ball_packet(fakeBall());                  // after the line packet, also on a line
  digitalWriteFast(LED_BUILTIN, lineSent);
  loops++;

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    SenseTxStats s = senseTxStats;
    Serial.printf("loops/s %5lu | line/s %5lu | ball/s %4lu | ball skipped (tx full) %lu\n",
                  loops, s.line_pkts - prev.line_pkts, s.ball_pkts - prev.ball_pkts,
                  s.ball_skipped_full - prev.ball_skipped_full);
    prev = s;
    loops = 0;
  }
}
