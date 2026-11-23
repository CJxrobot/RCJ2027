#include <core_sense.h>

SenseTxStats senseTxStats;

// Placeholders until new_ir_sensing is added (R02)
__attribute__((weak)) void ir_sample() {}
__attribute__((weak)) IrBall compute_ir_ball() { return IrBall(); }

bool send_line_packet(uint32_t state) {
  bool line = state != 0xFFFFFFFF;
  digitalWriteFast(PIN_0, line);
  digitalWriteFast(PIN_1, line);
  if (!line) return false;

  uint8_t p[LINE_LEN] = {LINE_HDR,
                         (uint8_t)(state), (uint8_t)(state >> 8),
                         (uint8_t)(state >> 16), (uint8_t)(state >> 24), 0};
  p[5] = pkt_sum(p, 1, 4);
  Serial8.write(p, LINE_LEN);
  senseTxStats.line_pkts++;
  return true;
}

bool send_ball_packet(const IrBall& ball, bool line_sent) {
  static uint32_t last = 0;
  uint32_t now = micros();
  if (now - last < 1000000UL / BALL_TX_HZ) return false; // not due yet

  if (line_sent) { senseTxStats.ball_skipped_line++; return false; }
  if (Serial8.availableForWrite() < BALL_LEN) { senseTxStats.ball_skipped_full++; return false; }

  uint8_t p[BALL_LEN] = {BALL_HDR, (uint8_t)ball.found, 0, 0, 0, 0, 0};
  pkt_put16(p, 2, ball.found ? ball.angle % 360 : PKT_NONE);
  pkt_put16(p, 4, ball.found ? ball.strength : 0);
  p[6] = pkt_sum(p, 1, 5);
  Serial8.write(p, BALL_LEN);
  senseTxStats.ball_pkts++;
  last = now;
  return true;
}
