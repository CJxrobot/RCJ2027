#pragma once
#include <Arduino.h>
#include <Wire.h>

#define SSD1306_SWITCHCAPVCC 2
#define SSD1306_WHITE 1

// Fake OLED: accepts everything, shows nothing
struct Adafruit_SSD1306 {
  Adafruit_SSD1306(int, int, FakeWire*, int) {}
  bool begin(int, int) { return true; }
  void clearDisplay() {}
  void display() {}
  void setTextSize(int) {}
  void setTextColor(int) {}
  void setCursor(int, int) {}
  void print(const char*) {}
  void println(const char* = "") {}
  int printf(const char*, ...) { return 0; }
};
