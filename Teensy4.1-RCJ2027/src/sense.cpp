#include <Arduino.h>
#include <core_sense.h>

// Sense core: the ONE program for both robots. Only USE_MUX2 differs.
//
// Every loop, in this order:
//   1. scan the line sensors
//   2. PIN_0 / PIN_1 + packet to the act core at once (line + newest ball), no rate limit
//   3. read the IR ball (~1 ms), sent in the next packet
//   4. ultrasonic H1-H4 (non-blocking): AB packet when a reading finishes (~33/s)
// LINE_SCAN_BUTTON (pin 36): line calibration (only the channels in use)

#define USE_MUX2 1   // 1 = defense robot (mux 1 + mux 2), 0 = attack robot (mux 1 only, mux 2 not read)
#define DEBUG    0   // 1 = print all line sensors every 100 ms (USB serial)

// Mux 2 channels with no sensor, and the outer side sensors that are read but never reported
#define M2_UNUSED (0x1740UL << 16) // mux 2 channel 6, 8, 9, 10, 12 have no sensor
#define M2_IGNORE (0x8024UL << 16) // R_out 15, B_out 5, L_out 2
// Broken mux 1 ring channels (bit ch = ring channel ch, angle ch x 22.5): never "line".
// Emergency use: a channel that reads wrong (always line / never line) is masked here.
// Per robot (the two Sense boards share this file): ATK used with USE_MUX2 0, DEF with 1.
#define M1_IGNORE_ATK 0x0080UL       // attack robot: ch 7 (157.5 deg) broken (2026-11-25)
#define M1_IGNORE_DEF 0x0000UL       // defense robot: none
#define M1_IGNORE (USE_MUX2 ? M1_IGNORE_DEF : M1_IGNORE_ATK)

// Mux 2 sensor names for the debug print, same as line_sensing.cpp
struct M2Sensor { const char* name; uint8_t ch; };
const M2Sensor M2_MAP[] = {
  {"F_out", 11}, {"F_in",  7},
  {"B_out",  5}, {"B_mid", 4}, {"B_in",  3},
  {"L_out",  2}, {"L_mid", 1}, {"L_in",  0},
  {"R_out", 15}, {"R_mid", 14}, {"R_in", 13},
};
const int M2_N = sizeof(M2_MAP) / sizeof(M2_MAP[0]);

// L = line seen (bit = 0), . = no line
char mark(int i) { return (lineState >> i) & 1 ? '.' : 'L'; }

void printDebug() {
  Serial.printf("flag %d  state %08lX  ball %s %u deg str %u\n", lineState != 0xFFFFFFFF, lineState,
                irBallLast.found ? "seen" : "none", irBallLast.angle, irBallLast.strength);

  Serial.print("M1 deg :");
  for (int ch = 0; ch < 16; ch++) Serial.printf("%6d", (int)(ch * 22.5f));
  Serial.print("\nM1 raw :");
  for (int ch = 0; ch < 16; ch++) Serial.printf("%6d", lsRaw[ch]);
  Serial.print("\nM1 thr :");
  for (int ch = 0; ch < 16; ch++) Serial.printf("%6d", avg_ls[ch]);
  Serial.print("\nM1 line:");
  for (int ch = 0; ch < 16; ch++) Serial.printf("%6c", mark(ch));

  if (USE_MUX2) {
    Serial.print("\nM2     :");
    for (int i = 0; i < M2_N; i++) Serial.printf("%6s", M2_MAP[i].name);
    Serial.print("\nM2 raw :");
    for (int i = 0; i < M2_N; i++) Serial.printf("%6d", lsRaw[M2_MAP[i].ch + 16]);
    Serial.print("\nM2 thr :");
    for (int i = 0; i < M2_N; i++) Serial.printf("%6d", avg_ls[M2_MAP[i].ch + 16]);
    Serial.print("\nM2 line:");
    for (int i = 0; i < M2_N; i++) Serial.printf("%6c", mark(M2_MAP[i].ch + 16));
  }
  Serial.print("\n\n");
}

void setup() {
  core_sense_init();
  analogWriteFrequency(LS_BRIGHTNESS, 20000);
  analogWrite(LS_BRIGHTNESS, 105);   // same as line_sensing.cpp calibration (0-255)
  line_sensor_config(USE_MUX2, ~(M2_UNUSED | M2_IGNORE | M1_IGNORE));
  us_begin();
}

// Attack flash only (USE_MUX2 0), every 2 s on USB: the robot and the masked ring channels.
// The defense flash (USE_MUX2 1) prints nothing: unchanged.
static void printConfig() {
  static uint32_t last = 0;
  if (USE_MUX2 || millis() - last < 2000) return;
  last = millis();
  Serial.printf("SENSE %s robot: USE_MUX2 %d, ring mask 0x%04lX (masked: never line)\n",
                USE_MUX2 ? "DEFENSE" : "ATTACK", USE_MUX2, (unsigned long)M1_IGNORE);
}

void loop() {
  fast_update_line_sensor();                 // 1. line
  if (calibrate_line_sensor()) return;       // calibrating: send nothing

  send_line_state();                         // 2. PIN_0/PIN_1 + packet (line + newest ball)
  digitalWriteFast(LED_BUILTIN, lineState != 0xFFFFFFFF);

  printConfig();
  update_ir_ball();                          // 3. ball, goes out with the next packet
  if (us_update()) send_us_packet();         // 4. ultrasonic

  static uint32_t lastPrint = 0;
  if (DEBUG && millis() - lastPrint >= 100) {
    lastPrint = millis();
    printDebug();
  }
}
