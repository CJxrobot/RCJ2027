#pragma once
// Fake Arduino / Teensy for the PC simulator.
// Only what core_act.cpp and the act programs use. The robot code is NOT changed.
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <deque>
#include <string>
#include <vector>
using std::min;
using std::max;

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3
#define RISING 4
#define LED_BUILTIN 13
#define DEG_TO_RAD 0.017453292519943295f
#define RAD_TO_DEG 57.29577951308232f

template <class T, class L, class H>
T constrain(T x, L lo, H hi) { return x < lo ? (T)lo : (x > hi ? (T)hi : x); }

uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int  digitalRead(uint8_t pin);
#define digitalWriteFast digitalWrite
#define digitalReadFast  digitalRead
void analogWrite(uint8_t pin, int value);
inline void analogWriteFrequency(uint8_t, float) {}
inline void analogWriteResolution(int) {}
inline int  analogRead(uint8_t) { return 0; }
void attachInterrupt(uint8_t pin, void (*fn)(), int mode);
void detachInterrupt(uint8_t pin);
inline uint8_t digitalPinToInterrupt(uint8_t pin) { return pin; }
inline void noInterrupts() {}
inline void interrupts() {}

// Serial: robot reads bytes the simulator pushed; Serial (USB) text goes to the console log
struct FakeSerial {
  int id;
  std::deque<uint8_t> rx;
  std::string line;
  explicit FakeSerial(int i) : id(i) {}
  void begin(uint32_t) {}
  void setTimeout(uint32_t) {}
  operator bool() { return true; }
  int available() { return (int)rx.size(); }
  int read();
  int availableForWrite() { return 64; }
  size_t write(uint8_t c);
  size_t write(const uint8_t* b, size_t n) { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
  void flush() {}
  int printf(const char* fmt, ...);
  void print(const char* s);
  void print(char c)          { write((uint8_t)c); }
  void print(int v)           { printf("%d", v); }
  void print(unsigned v)      { printf("%u", v); }
  void print(long v)          { printf("%ld", v); }
  void print(unsigned long v) { printf("%lu", v); }
  void print(double v, int d = 2) { printf("%.*f", d, v); }
  template <class T> void println(T v) { print(v); write('\n'); }
  template <class T> void println(T v, int d) { print(v, d); write('\n'); }
  void println() { write('\n'); }
};
extern FakeSerial Serial, Serial2, Serial3, Serial8;

struct FakeCrashReport { operator bool() { return false; } };
extern FakeCrashReport CrashReport;

struct IntervalTimer {
  bool begin(void (*)(), uint32_t) { return true; }
  void update(uint32_t) {}
  void end() {}
};

// Camera DMA registers (camera_begin() writes them, nothing reads them)
extern uint32_t LPUART2_CTRL, LPUART2_WATER, LPUART2_BAUD;
extern volatile uint8_t LPUART2_DATA;
#define NVIC_DISABLE_IRQ(x)
#define LPUART_CTRL_RIE 1
#define LPUART_CTRL_ILIE 2
#define LPUART_WATER_RXWATER(x) (x)
#define LPUART_BAUD_RDMAE 4
#define IRQ_LPUART2 0
#define DMAMUX_SOURCE_LPUART2_RX 0

// ===== Simulator side (not Arduino) =====
extern uint64_t sim_us;                       // simulated time, microseconds
void sim_advance(uint32_t us);                // world.cpp: move time forward, run the world
int  sim_pin(uint8_t pin);                    // last value on a pin
void sim_set_pin(uint8_t pin, int value);     // world -> robot, runs attachInterrupt on RISING
void sim_camera_push(const uint8_t* b, int n);// bytes into the camera DMA ring
struct ConsoleLine { uint32_t ms; std::string text; };
extern std::vector<ConsoleLine> sim_console;  // everything the program printed on Serial
