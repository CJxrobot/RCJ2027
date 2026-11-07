#include <Arduino.h>
#include <core_act.h>

// ===== UI v1 =====
// Screen layout (128x64):
//   y 0-10  : top bar, gyro heading + page number (always shown)
//   y 12-63 : page body, reserved for other data
#define UI_BODY_Y 12
int uiPage = 0;

void drawTopBar() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("H ");
  display.print(gyroData.heading, 1);
  display.setCursor(110, 0);
  display.print(uiPage + 1);
  display.drawFastHLine(0, 10, SCREEN_WIDTH, SSD1306_WHITE);
}

void drawPage() {
  display.clearDisplay();
  drawTopBar();
  switch (uiPage) {
    case 0: break; // body reserved, draw from y = UI_BODY_Y
  }
  display.display();
}

// ===== Main =====
void setup() {
  core_act_init();
}

void loop() {
  update_gyro_sensor();
  Serial.println("loop");
  if(digitalRead(38) == LOW){
    Serial.println("Ball Possession");
  }
  else{
    Serial.println("No Ball Possession");
  }
  /*
  SetMotorSpeed(1, 50);
  SetMotorSpeed(2, 50);
  SetMotorSpeed(3, 50);
  SetMotorSpeed(4, 50);
  */
  Vector_Motion(40, 40, 0);    
  // OLED refresh is slow (~25 ms), so only redraw every 100 ms
  static uint32_t lastDisplay = 0;
  if (millis() - lastDisplay >= 100) {
    lastDisplay = millis();
    drawPage();
  }
}