#ifndef CORE_ACT_H
#define CORE_ACT_H
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- OLED Configuration ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

struct GyroData {
    float heading = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    bool exist = false;
};

extern GyroData gyroData;
extern Adafruit_SSD1306 display;


void update_gyro_sensor();
void drawMessage(const char* msg);
void core_act_init();
#endif