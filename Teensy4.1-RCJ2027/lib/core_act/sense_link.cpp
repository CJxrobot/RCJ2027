#include <core_act.h>

LineData lineData;
IrBallData irBall;
SenseLinkStats senseLinkStats;

uint8_t read_sense() {
  static uint8_t buf[8];
  static uint8_t n = 0, need = 0;
  uint8_t got = 0;

  while (Serial8.available()) {
    uint8_t b = Serial8.read();

    if (n == 0) {                      // waiting for a header
      if (b == LINE_HDR)      need = LINE_LEN;
      else if (b == BALL_HDR) need = BALL_LEN;
      else { senseLinkStats.junk++; continue; }
    }
    buf[n++] = b;
    if (n < need) continue;
    n = 0;

    if (need == LINE_LEN) {
      if (pkt_sum(buf, 1, 4) != buf[5]) { senseLinkStats.bad++; continue; }
      lineData.state = buf[1] | (buf[2] << 8) | (buf[3] << 16) | ((uint32_t)buf[4] << 24);
      lineData.t_ms = millis();
      senseLinkStats.line_pkts++;
      got |= SENSE_GOT_LINE;
    } else {
      if (pkt_sum(buf, 1, 5) != buf[6]) { senseLinkStats.bad++; continue; }
      uint16_t a = pkt_u16(buf, 2);
      irBall.found = buf[1] && a < 360;
      if (irBall.found) irBall.angle = a;
      irBall.strength = pkt_u16(buf, 4);
      irBall.t_ms = millis();
      senseLinkStats.ball_pkts++;
      got |= SENSE_GOT_BALL;
    }
  }
  return got;
}

bool ir_ball_fresh(uint32_t max_age_ms) {
  return irBall.found && irBall.t_ms != 0 && millis() - irBall.t_ms <= max_age_ms;
}
