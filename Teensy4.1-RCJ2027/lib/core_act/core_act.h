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

// --- Motor Pins (index = SetMotorSpeed port) ---
//               port:  1   2   3   4
#define MOTOR_DIR_PINS { 6, 10,  9,  8}   // 方向控制腳
#define MOTOR_PWM_PINS { 2,  5,  4,  3}   // PWM 控制腳
#define MOTOR_COUNT 4
#define MOTOR_SPEED_LIMIT 75.0f           // |speed| cap in percent (RCAP 2027)

#define SLP 11
#define BALL_POSESSION 38

struct GyroData {
    float heading = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;
    bool exist = false;   // true only if a valid packet arrived this update
};

// Heading convention: 90 = facing forward (field frame), CCW positive
struct RobotState {
    float robot_heading = 90.0f;     // target heading used by Vector_Motion
    float heading_threshold = 2.0f;  // deg, no correction inside this band
    float P_factor = 0.5f;           // omega per degree of heading error
};

extern GyroData gyroData;
extern RobotState robot;
extern Adafruit_SSD1306 display;

void core_act_init();
void update_gyro_sensor();
void drawMessage(const char* msg);

// --- Motors ---
void SetMotorSpeed(uint8_t port, float speed);   // port 1-4, speed -100..100
void MotorStop();

// --- Motion ---
void RobotIKControl(float vx, float vy, float omega);
void Vector_Motion(float Vx, float Vy, float rot_V);
void FC_Vector_Motion(float WVx, float WVy, float target_heading);
#endif
