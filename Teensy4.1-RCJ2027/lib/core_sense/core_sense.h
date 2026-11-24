#ifndef CORE_SENSE_H
#define CORE_SENSE_H
#include <Arduino.h>

#define LINE_SCAN_BUTTON 36

// 20 IR outputs (out 1-20, index = out - 1), same as sense_ir.cpp on main
//   near group (12 sensors) is used when any of them sees the ball, else the far group (8)
//   angle = count-weighted vector sum of the strongest sensor and its 2 neighbours
//   strength = how many of the 20 sensors see the ball (1 = farthest, more = closer)
#define IR_COUNT 20
#define IR_WINDOW_US 1000   // one ball pulse cycle is ~833 us
#define IR_MIN       0      // count above this = sensor sees the ball
// 0: compute_ir_ball() samples for IR_WINDOW_US like sense_ir.cpp (tested, ~1 ms per loop)
// 1: ir_sample() samples during the line scan, no wait (task T03 / R02)
#ifndef IR_NONBLOCKING
#define IR_NONBLOCKING 0
#endif
#define LS_BRIGHTNESS 24

// Line sensor: 16-channel mux, select pins S0-S3, analog outputs M1 / M2
#define LS_S0 16
#define LS_S1 17
#define LS_S2 18
#define LS_S3 19
#define LS_M1 14 // channel 0-15
#define LS_M2 15 // reserved, channel 16-31
#define LS_COUNT 16 // channels in use (M1 only), 32 when M2 is used
// Line channels that have a sensor (bit = channel, 16-31 = mux 2), set by line_sensor_config().
// Channels outside the mask are never calibrated and never reported as a line.
// useMux2 = false: mux 2 is not read at all (faster) and reports no line.
extern uint32_t lsUsedMask;
extern bool lsUseMux2;
void line_sensor_config(bool useMux2, uint32_t usedMask);

// Line flag to act core: HIGH while any sensor sees a line (faster than UART)
#define PIN_0 0
#define PIN_1 1

extern const uint8_t IR_PINS[IR_COUNT];
extern uint16_t irValue[IR_COUNT]; // LOW-sample count per sensor in the last window

extern uint16_t lsRaw[32];    // last analog reading per line sensor channel
extern uint16_t avg_ls[32];   // threshold per line sensor channel
extern uint32_t lineState;    // bit = 0 when reading below threshold for 2 scans in a row

void core_sense_init();
void update_ball_sensor();    // count LOW samples on all 20 sensors for IR_WINDOW_US (blocking)
void fast_update_line_sensor();
bool calibrate_line_sensor(); // true while scanning
bool send_line_state();       // set PIN_0/PIN_1, send line + newest ball (irBallLast) to act core, true if sent
void update_ir_ball();        // read the IR sensors (~1 ms) and store the result in irBallLast

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

// ===== /sense_data: Sense -> Act, Serial8, 921600 =====
// Line + IR ball in ONE packet (11 bytes), so every packet has both:
//   AA s0 s1 s2 s3 found angL angH strL strH chk     chk = sum of bytes [1..9]
//   s0-s3: lineState, bit = 0 when that sensor sees the line (all 1 = no line)
//   found: 1 = ball seen; angle 0-359 (0xFFFF if not seen); strength: higher = closer
// Sent every loop of the sense core (no rate limit): line first, ball = newest IR result.
#define SENSE_BAUD      921600
#define SENSE_HDR       0xAA
#define SENSE_LEN       11

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

// ===== IR ball (from sense_ir.cpp) =====
//   ir_sample()       called once per mux channel inside fast_update_line_sensor();
//                     samples the IR sensors there when IR_NONBLOCKING is 1
//   compute_ir_ball() reads the IR sensors and returns the ball (update_ir_ball() calls it)
void ir_sample();
IrBall compute_ir_ball();
extern IrBall irBallLast;      // newest ball, sent in every packet

// Send line + ball in one packet (see /sense_data above). Also sets PIN_0/PIN_1 first.
// Sent on every call, no rate limit. Skipped only if Serial8 has no room for the whole packet.
// send_line_state() calls this with lineState and irBallLast. Returns true if sent.
bool send_sense_packet(uint32_t state, const IrBall& ball);

// Counters for test programs
struct SenseTxStats {
  uint32_t pkts = 0;          // all packets
  uint32_t line_pkts = 0;     // packets with a line in them
  uint32_t skipped_full = 0;  // Serial8 TX buffer was full
};
extern SenseTxStats senseTxStats;

#endif
