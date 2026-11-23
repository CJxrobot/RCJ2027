#include <Arduino.h>
#include <core_sense.h>

// T11 (Sense): game program for the attack robot
//   line scan (IR sampled inside) -> line packet (high priority) -> IR ball packet (low priority)
//   LINE_SCAN_BUTTON (pin 36): calibration, same as before
// compute_ir_ball() is the empty placeholder until new_ir_sensing is added (R02)

void setup() {
  core_sense_init();
  analogWrite(LS_BRIGHTNESS, 155);
}

void loop() {
  fast_update_line_sensor();
  if (calibrate_line_sensor()) return;  // scanning: send nothing

  bool lineSent = send_line_state();
  send_ball_packet(compute_ir_ball(), lineSent);
  digitalWriteFast(LED_BUILTIN, lineSent);
}
