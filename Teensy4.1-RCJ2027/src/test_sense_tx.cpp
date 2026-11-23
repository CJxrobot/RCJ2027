#include <Arduino.h>
#include <core_sense.h>

// T01 (Sense): ONE packet with line + IR ball (AA, 11 bytes) on Serial8 at 921600.
//   no line:   ~SENSE_TX_HZ (100) packets/s
//   line seen: a packet every loop, each still carrying the ball
// USE_FAKE_BALL 1: fake ball sweep (+3 deg every 10 ms), 0: real IR from compute_ir_ball().
// The printout also shows the ball that went out, so you can check the IR angle.
//
// Pass: off the line -> ~100 pkts/s, with line 0/s
//       on the line  -> pkts/s = loops/s, with line = pkts/s, tx full 0

#define USE_FAKE_BALL 0

// Same as sense_ir.cpp / sense_defense.cpp: channels with no sensor and the outer ones
#define M2_UNUSED (0x1740UL << 16) // mux 2 channel 6, 8, 9, 10, 12 have no sensor
#define M2_IGNORE (0x8024UL << 16) // R_out 15, B_out 5, L_out 2: read but never reported

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
  analogWriteFrequency(LS_BRIGHTNESS, 20000);
  analogWrite(LS_BRIGHTNESS, 105);   // same as sense_ir.cpp / line_sensing.cpp calibration
  Serial.printf("T01 test_sense_tx: line + %s IR ball in one packet\n", USE_FAKE_BALL ? "FAKE" : "REAL");
}

void loop() {
  static uint32_t loops = 0, lastPrint = 0;
  static SenseTxStats prev;

  fast_update_line_sensor();
  lineState |= M2_UNUSED | M2_IGNORE;
  IrBall ball = USE_FAKE_BALL ? fakeBall() : compute_ir_ball();
  send_sense_packet(lineState, ball);
  digitalWriteFast(LED_BUILTIN, lineState != 0xFFFFFFFF);
  loops++;

  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    SenseTxStats s = senseTxStats;
    Serial.printf("loops/s %5lu | pkts/s %5lu | with line %5lu | tx full %lu | ball %s %3u deg str %u | line %08lX\n",
                  loops, s.pkts - prev.pkts, s.line_pkts - prev.line_pkts,
                  s.skipped_full - prev.skipped_full,
                  ball.found ? "seen" : "none", ball.angle, ball.strength, lineState);
    if (!USE_FAKE_BALL) {                       // raw counts, out 1-20
      Serial.print("   IR out1-20:");
      for (int i = 0; i < IR_COUNT; i++) Serial.printf(" %u", irValue[i]);
      Serial.println();
    }
    prev = s;
    loops = 0;
  }
}
