#ifndef CORE_SENSE_H
#define CORE_SENSE_H
#include <Arduino.h>

#define LINE_SCAN_BUTTON 36

// 16 IR ports, port N is index N-1
// odd ports  (1,3,..,15): TSOP4038 -> far
// even ports (2,4,..,16): TSSP     -> near
#define IR_COUNT 16
#define LS_BRIGHTNESS 24

// Line sensor: 16-channel mux, select pins S0-S3, analog outputs M1 / M2
#define LS_S0 16
#define LS_S1 17
#define LS_S2 18
#define LS_S3 19
#define LS_M1 14 // channel 0-15
#define LS_M2 15 // reserved, channel 16-31
#define LS_COUNT 16 // channels in use (M1 only), 32 when M2 is used
// Line channels that have a sensor (bit = channel, 16-31 = mux 2).
// Channels outside the mask are never calibrated and never reported as a line.
#ifndef LS_USED_MASK
#define LS_USED_MASK 0xFFFFFFFFUL
#endif

// Line flag to act core: HIGH while any sensor sees a line (faster than UART)
#define PIN_0 0
#define PIN_1 1

extern const uint8_t IR_PINS[IR_COUNT];
extern uint16_t irValue[IR_COUNT]; // higher = ball closer

extern uint16_t lsRaw[32];    // last analog reading per line sensor channel
extern uint16_t avg_ls[32];   // threshold per line sensor channel
extern uint32_t lineState;    // bit = 0 when reading below threshold for 2 scans in a row

void core_sense_init();
void update_ball_sensor();
void fast_update_line_sensor();
bool calibrate_line_sensor(); // true while scanning
bool send_line_state();       // set PIN_0/PIN_1, send lineState to act core (Serial8), true if sent

// =====================================================================
// Packet formats (keep the same as core_act.h and the Packets tab)
// =====================================================================
// Packet formats shared by Teensy Act, Teensy Sense and the Maix Cam
// Same layout as the "Packets" tab of the plan sheet. Change both together.
//
// Angles: 0-359 deg, robot frame, 0 = right, 90 = front, CCW. 0xFFFF = not seen.
// Multi-byte fields are little endian. Checksum = low 8 bits of the byte sum.

#define PKT_NONE 0xFFFF

// ===== /field_data v2: Maix Cam -> Act, Serial3, 115200, ~100 Hz =====
// CC DE flags ballA ballD frontA frontD backA backD seq chk EE   (18 bytes)
// chk = sum of bytes [1..15]
#define CAM_BAUD        115200
#define CAM_H1          0xCC
#define CAM_H2          0xDE
#define CAM_END         0xEE
#define CAM_LEN         18
#define CAM_FLAG_BALL   0x01
#define CAM_FLAG_FRONT  0x02
#define CAM_FLAG_BACK   0x04

// ===== Sense -> Act, Serial8, 921600 =====
#define SENSE_BAUD      921600
// /line_data, high priority: AA s0 s1 s2 s3 chk   (6 bytes), chk = sum of bytes [1..4]
#define LINE_HDR        0xAA
#define LINE_LEN        6
// /ir_ball_data, low priority (sent after the line packet, also while on a line):
//   BB found angL angH strL strH chk   (7 bytes), chk = sum of bytes [1..5]
#define BALL_HDR        0xBB
#define BALL_LEN        7

static inline uint8_t pkt_sum(const uint8_t* b, int from, int to) {
  uint8_t s = 0;
  for (int i = from; i <= to; i++) s += b[i];
  return s;
}
static inline uint16_t pkt_u16(const uint8_t* b, int i) {
  return (uint16_t)(b[i] | (b[i + 1] << 8));
}
static inline void pkt_put16(uint8_t* b, int i, uint16_t v) {
  b[i] = v & 0xFF;
  b[i + 1] = v >> 8;
}

// =====================================================================
// Sending to the act core
// =====================================================================
// IR ball result on the sense core (robot frame, 0 = right, 90 = front)
struct IrBall {
  bool found = false;
  uint16_t angle = 0;     // 0-359 deg
  uint16_t strength = 0;  // higher = closer
};

// ===== RESERVED for new_ir_sensing (task R02) =====
// Both are weak: define them in your new IR file and these empty versions go away.
//   ir_sample()       called once per mux channel inside fast_update_line_sensor(),
//                     during the settle time, so IR is sampled without a 1 ms wait
//   compute_ir_ball() called once per loop, returns the newest ball result
void ir_sample();
IrBall compute_ir_ball();

// Line packet (high priority). Sets PIN_0/PIN_1 and sends AA packet while a line is seen.
// Returns true if a packet was sent this call.
bool send_line_packet(uint32_t state);

// Ball packet (low priority). Always sent, also while a line is seen (the act core needs
// the ball position there too). In each loop the line packet goes out first. Sent only if:
//   - at least 1000/BALL_TX_HZ ms since the last ball packet
//   - Serial8 has room for the whole packet
// Returns true if a packet was sent.
#define BALL_TX_HZ 100
bool send_ball_packet(const IrBall& ball);

// Counters for test programs
struct SenseTxStats {
  uint32_t line_pkts = 0;
  uint32_t ball_pkts = 0;
  uint32_t ball_skipped_full = 0; // ball due but Serial8 TX buffer was full
};
extern SenseTxStats senseTxStats;

#endif
