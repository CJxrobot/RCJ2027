#include <Arduino.h>
#include <core_act.h>

// Act core: camera v2 (ball + goal, Serial3) and IR ball (from sense core, Serial8) on the OLED
// Sense core must run sense_ir.cpp
// Compare: CAM ball angle vs IR ball angle, CAM ball distance in cm

void setup() {
  core_act_init();
}

void loop() {
  static uint32_t camPkt = 0, lastDraw = 0;
  static uint32_t lineState = 0xFFFFFFFF;

  if (read_topmaix_ball_goal()) camPkt++;
  read_line_state(lineState); // also fills robot.ir_*

  if (millis() - lastDraw < 100) return; // OLED is slow, redraw every 100 ms
  lastDraw = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("CAM ball");
  if (topmaixData.ball_found) display.printf(" A %3u  %u cm\n", topmaixData.ball_angle, topmaixData.ball_dist);
  else                        display.println(" none");
  display.println("IR ball");
  if (robot.ir_found) display.printf(" A %3u  N %u\n", robot.ir_angle, robot.ir_dist); // N = sensors seeing the ball
  else                display.println(" none");
  if (topmaixData.goal_found) display.printf("Goal A %3u D %u\n", topmaixData.goal_angle, topmaixData.goal_dist);
  else                        display.println("Goal none");
  display.printf("cam pkt %lu\n", camPkt);
  display.display();

  Serial.printf("CAM ball %d A %u %u cm | IR ball %d A %u N %u | goal %d A %u D %u | pkt %lu\n",
                topmaixData.ball_found, topmaixData.ball_angle, topmaixData.ball_dist,
                robot.ir_found, robot.ir_angle, robot.ir_dist,
                topmaixData.goal_found, topmaixData.goal_angle, topmaixData.goal_dist, camPkt);
}
