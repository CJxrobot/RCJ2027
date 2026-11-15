#include <Arduino.h>
#include <core_act.h>

// Defense, act core
// Front inner on the line -> move backward
// Front outer on the line -> move forward
// Back sensors (out/mid/in in a row):
//   some see the line  -> horizontal line behind -> move backward
//   all three see it   -> vertical line          -> stop completely
// Sideways: default speed, stop with the OUT sensor on the side line
//   out on the line       -> stop here
//   mid or in on the line -> back off until out sees the line again
// The side limits only cap speed toward that line, never push the robot into it
// While a side sensor sees the line, the robot never moves backward (curved side line)

#define SPEED       30  // forward / backward
#define SIDE_SPEED  30  // default sideways speed (+ = right)
#define BACK_SPEED  15  // backing off from a side line
#define LOST_MS     300 // stop backing if that side sees no line this long

// Sensors as lineState bits (mux 2 channel + 16)
#define M2(ch) (1UL << (16 + (ch)))
#define FRONT_IN  M2(14)
#define FRONT_OUT M2(15)
#define BACK  (M2(8) | M2(7) | M2(6)) // out, mid, in

// One side line: its sensors and backing-off memory
struct Side {
  uint32_t out, mid, in;
  bool backing;
  uint32_t lastSeen;
};
Side rightSide = {M2(0), M2(1), M2(2), false, 0};
Side leftSide  = {M2(5), M2(4), M2(3), false, 0};

void setup() {
  core_act_init();

  // Wait for button 1, then run until restart
  MotorStop();
  drawMessage("Press BTN");
  while (digitalRead(START_BUTTON) == HIGH) update_gyro_sensor();
  delay(200); // debounce
}

// true if any sensor in mask sees the line (bit = 0)
bool seen(uint32_t state, uint32_t mask) {
  return (~state & mask) != 0;
}

// Max speed toward a side line (negative = move away from it)
float sideLimit(Side& s, uint32_t state) {
  bool out = seen(state, s.out);
  bool deep = seen(state, s.mid | s.in);

  if (out || deep) s.lastSeen = millis();
  if (deep) s.backing = true;                          // too far over the line
  if (out && !deep) s.backing = false;                 // back at the out sensor
  if (millis() - s.lastSeen > LOST_MS) s.backing = false; // line lost, give up

  if (s.backing) return -BACK_SPEED; // back off until out sees the line
  if (out)       return 0;           // out on the line: stay here
  return SIDE_SPEED;                 // no line: full speed
}

void loop() {
  static uint32_t state = 0xFFFFFFFF; // bit = 0 when that sensor sees the line

  update_gyro_sensor();
  read_line_state(state);                // newest packet from sense core
  if (!line_flag()) state = 0xFFFFFFFF;  // flag LOW = no line anywhere

  // Back sensors: all three on the line = vertical line -> stop completely
  int back = __builtin_popcount(~state & BACK); // back sensors seeing the line
  if (back == 3) {
    Vector_Motion(0, 0, 0);
    return;
  }

  // Forward / backward from the front sensors, then the back sensors
  bool in  = seen(state, FRONT_IN);
  bool out = seen(state, FRONT_OUT);
  float vy = 0;
  if (in && !out)      vy = -SPEED; // backward
  else if (out && !in) vy =  SPEED; // forward
  else if (back > 0)   vy = -SPEED; // horizontal line behind -> backward

  // At a side line the curve also reaches the front sensors: never move backward there
  bool atSide = seen(state, rightSide.out | rightSide.mid | rightSide.in |
                            leftSide.out  | leftSide.mid  | leftSide.in) ||
                rightSide.backing || leftSide.backing;
  if (atSide) vy = max(vy, 0.0f);

  // Sideways: default speed, capped by each side line
  float vx = SIDE_SPEED;
  vx = min(vx,  sideLimit(rightSide, state)); // right line
  vx = max(vx, -sideLimit(leftSide,  state)); // left line

  Vector_Motion(vx, vy, 0);
}
