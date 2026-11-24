#pragma once
#include <Arduino.h>

// Fake DMA: the simulator writes camera bytes into the ring and moves DADDR, like the real DMA
struct FakeTCD { volatile void* DADDR = nullptr; };

struct DMAChannel {
  FakeTCD tcd;
  FakeTCD* TCD = &tcd;
  void begin(bool = false) {}
  void source(volatile const unsigned char&) {}
  void destinationCircular(volatile uint8_t* buf, unsigned len);   // fake_hw.cpp
  void triggerAtHardwareEvent(int) {}
  void enable() {}
};
