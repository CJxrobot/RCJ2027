#include <Arduino.h>
#include <core_sense.h>

// T11 (Sense): game program for the attack robot
//   line scan (IR sampled inside) -> ONE packet with line + IR ball
//   (every loop while a line is seen, otherwise 100 Hz)
//   LINE_SCAN_BUTTON (pin 36): calibration, same as before
// compute_ir_ball() is the empty placeholder until new_ir_sensing is added (R02)

void setup() {
  core_sense_init();
  analogWrite(LS_BRIGHTNESS, 155);
}

void loop() {
  fast_update_line_sensor();
  if (calibrate_line_sensor()) return;  // scanning: send nothing

  send_line_state();                     // line + compute_ir_ball() in one packet
  digitalWriteFast(LED_BUILTIN, lineState != 0xFFFFFFFF);
}
