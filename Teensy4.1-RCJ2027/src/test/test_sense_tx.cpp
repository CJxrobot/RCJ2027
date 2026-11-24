#include <Arduino.h>
#include <core_sense.h>

// T01 (Sense): ONE packet with line + IR ball (AA, 11 bytes) on Serial8 at 921600,
// every loop, same order as sense.cpp: line scan -> packet -> IR ball.
// USE_FAKE_BALL 1: fake ball sweep (+3 deg every 10 ms), 0: real IR from compute_ir_ball().
// The printout also shows the ball that went out, so you can check the IR angle.
//
// Pass: pkts/s = loops/s always; off the line with line 0, on the line with line = pkts/s; tx full 0

#define USE_FAKE_BALL 0
#define USE_MUX2      1   // same switch as sense.cpp: 1 = defense robot, 0 = attack robot

// Same as sense.cpp: channels with no sensor and the outer ones
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
  line_sensor_config(USE_MUX2, ~(M2_UNUSED | M2_IGNORE));
  Serial.printf("T01 test_sense_tx: line + %s IR ball in one packet\n", USE_FAKE_BALL ? "FAKE" : "REAL");
}

void loop() {
  static uint32_t loops = 0, lastPrint = 0;
  static SenseTxStats prev;

  static IrBall ball;
  fast_update_line_sensor();                         // 1. line
  send_sense_packet(lineState, ball);                // 2. packet at once (line + newest ball)
  digitalWriteFast(LED_BUILTIN, lineState != 0xFFFFFFFF);
  ball = USE_FAKE_BALL ? fakeBall() : compute_ir_ball();   // 3. ball for the next packet
  loops++;

  if (millis() - lastPrint >= 100) {
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
