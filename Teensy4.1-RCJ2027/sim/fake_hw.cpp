// Fake Teensy hardware for the simulator: time, pins, serial ports, camera DMA.
#include <Arduino.h>
#include <Wire.h>
#include <DMAChannel.h>

uint64_t sim_us = 0;
std::vector<ConsoleLine> sim_console;

FakeSerial Serial(0), Serial2(2), Serial3(3), Serial8(8);
FakeCrashReport CrashReport;
FakeWire Wire;
uint32_t LPUART2_CTRL, LPUART2_WATER, LPUART2_BAUD;
volatile uint8_t LPUART2_DATA;

// ----- time -----
uint32_t millis() { return (uint32_t)(sim_us / 1000); }
uint32_t micros() { return (uint32_t)sim_us; }
void delay(uint32_t ms) { sim_advance(ms * 1000); }
void delayMicroseconds(uint32_t us) { sim_advance(us); }

// ----- pins -----
static int pins[64];
static void (*isr[64])() = {};

void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t pin, uint8_t value) { pins[pin] = value; }
int digitalRead(uint8_t pin) { return pins[pin]; }
void analogWrite(uint8_t pin, int value) { pins[pin] = value; }
void attachInterrupt(uint8_t pin, void (*fn)(), int) { isr[pin] = fn; }
void detachInterrupt(uint8_t pin) { isr[pin] = nullptr; }

int sim_pin(uint8_t pin) { return pins[pin]; }

void sim_set_pin(uint8_t pin, int value) {
  bool rising = !pins[pin] && value;
  pins[pin] = value;
  if (rising && isr[pin]) isr[pin]();
}

// ----- serial -----
int FakeSerial::read() {
  if (rx.empty()) return -1;
  int b = rx.front();
  rx.pop_front();
  return b;
}

// Serial (USB) text is collected line by line with the time it was printed
size_t FakeSerial::write(uint8_t c) {
  if (id != 0) return 1;
  if (c == '\n') {
    sim_console.push_back({millis(), line});
    line.clear();
  } else if (c != '\r') {
    line += (char)c;
  }
  return 1;
}

void FakeSerial::print(const char* s) {
  while (*s) write((uint8_t)*s++);
}

int FakeSerial::printf(const char* fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  print(buf);
  return n;
}

// ----- camera DMA ring -----
static volatile uint8_t* camBuf = nullptr;
static unsigned camLen = 0, camPos = 0;
static FakeTCD* camTcd = nullptr;

void DMAChannel::destinationCircular(volatile uint8_t* buf, unsigned len) {
  camBuf = buf;
  camLen = len;
  camPos = 0;
  camTcd = &tcd;
  tcd.DADDR = (volatile void*)buf;
}

void sim_camera_push(const uint8_t* b, int n) {
  if (!camBuf) return;                        // camera_begin() not called
  for (int i = 0; i < n; i++) {
    camBuf[camPos] = b[i];
    camPos = (camPos + 1) % camLen;
  }
  camTcd->DADDR = (volatile void*)(camBuf + camPos);
}
