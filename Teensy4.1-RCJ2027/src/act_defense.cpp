#include <Arduino.h>
#include <core_act.h>

// Defense, act core
// Front inner on the line -> move backward
// Front outer on the line -> move forward
// Outer sensors R_out / B_out / L_out are ignored (sense core never reports them)
// Back sensors (mid/in in a row):
//   one sees the line  -> horizontal line behind -> move backward
//   both see it        -> vertical line          -> stop completely
// Sideways: default speed, stop with the MID sensor on the side line
//   mid on the line -> stop here
//   in on the line  -> back off until mid sees the line again
// The side limits only cap speed toward that line, never push the robot into it
// While a side sensor sees the line, the robot never moves backward (curved side line)
// Camera (goal behind, 270 = centre, 0 = right, 180 = left):
//   goal on the left  -> moving left slower, moving right faster
//   goal on the right -> moving right slower, moving left faster
//   the side-line limits are applied again afterwards, lines always win

#define SPEED       30  // forward / backward
#define SIDE_SPEED  30  // default sideways speed (+ = right)
#define BACK_SPEED  15  // backing off from a side line
#define LOST_MS     300 // stop backing if that side sees no line this long

#define GOAL_CENTER 270  // camera angle of the goal straight behind
#define GOAL_DEAD   5   // +- deg around the centre: no change
#define GOAL_SLOW   0.5f // speed factor toward the goal's side
#define GOAL_FAST   1.0f // speed factor away from it

// Sensors as lineState bits (mux 2 channel + 16)
#define M2(ch) (1UL << (16 + (ch)))
#define FRONT_IN  M2(7)
#define FRONT_OUT M2(11)
#define BACK  (M2(4) | M2(3)) // mid, in (B_out ignored)

// One side line: its sensors and backing-off memory
struct Side {
  uint32_t mid, in;
  bool backing;
  uint32_t lastSeen;
};
Side rightSide = {M2(14), M2(13), false, 0}; // mid, in (R_out ignored)
Side leftSide  = {M2(1),  M2(0),  false, 0}; // mid, in (L_out ignored)

// true if any sensor in mask sees the line (bit = 0)
bool seen(uint32_t state, uint32_t mask) {
  return (~state & mask) != 0;
}

// Max speed toward a side line (negative = move away from it)
float sideLimit(Side& s, uint32_t state) {
  bool mid = seen(state, s.mid);
  bool in  = seen(state, s.in);

  if (mid || in) s.lastSeen = millis();
  if (in) s.backing = true;                               // too far over the line
  if (mid && !in) s.backing = false;                      // back at the mid sensor
  if (millis() - s.lastSeen > LOST_MS) s.backing = false; // line lost, give up

  if (s.backing) return -BACK_SPEED; // back off until mid sees the line
  if (mid)       return 0;           // mid on the line: stay here
  return 1000;                       // no line: no limit
}

// Scale sideways speed by the goal direction from the camera
float goalAdjust(float vx) {
  if (!topmaixData.goal_found || topmaixData.goal_angle > 359) return vx;

  int diff = topmaixData.goal_angle - GOAL_CENTER; // < 0 = left, > 0 = right
  if (diff > 180)  diff -= 360;
  if (diff < -180) diff += 360;

  if (diff < -GOAL_DEAD) return vx * (vx < 0 ? GOAL_SLOW : GOAL_FAST); // goal left
  if (diff >  GOAL_DEAD) return vx * (vx > 0 ? GOAL_SLOW : GOAL_FAST); // goal right
  return vx;                                                           // centre
}

// Debug screen while waiting for the button. X = sensor sees the line
char mark(uint32_t state, uint32_t mask) { return seen(state, mask) ? 'X' : '.'; }

void drawDebug(uint32_t state) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Press BTN to start");
  display.printf("Head  %.1f\n", gyroData.heading);
  if (topmaixData.goal_found) display.printf("Goal  A%d D%d\n", topmaixData.goal_angle, topmaixData.goal_dist);
  else                        display.println("Goal  none");
  display.printf("Flag  %d\n", line_flag());
  display.printf("Front in %c out %c\n", mark(state, FRONT_IN), mark(state, FRONT_OUT));
  display.printf("Left  m%c i%c\n", mark(state, leftSide.mid), mark(state, leftSide.in));
  display.printf("Right m%c i%c\n", mark(state, rightSide.mid), mark(state, rightSide.in));
  display.printf("Back  m%c i%c\n", mark(state, M2(4)), mark(state, M2(3)));
  display.display();
}

void setup() {
  core_act_init();
  camera_dma_begin();

  // Wait for button 1 showing live sensor data, then run until restart
  MotorStop();
  uint32_t state = 0xFFFFFFFF, lastDraw = 0;
  while (digitalRead(START_BUTTON) == HIGH) {
    update_gyro_sensor();
    read_camera();
    read_line_state(state);
    if (!line_flag()) state = 0xFFFFFFFF;
    if (millis() - lastDraw >= 100) { // OLED is slow, redraw every 100 ms
      lastDraw = millis();
      drawDebug(state);
    }
  }
  drawMessage("GO");
  delay(200); // debounce
}

void loop() {
  static uint32_t state = 0xFFFFFFFF; // bit = 0 when that sensor sees the line

  update_gyro_sensor();
  read_camera();                         // newest goal data (DMA, no waiting)
  read_line_state(state);                // newest packet from sense core
  if (!line_flag()) state = 0xFFFFFFFF;  // flag LOW = no line anywhere

  // Back sensors: mid and in both on the line = vertical line -> stop completely
  int back = __builtin_popcount(~state & BACK); // back sensors seeing the line
  if (back == 2) {
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
  bool atSide = seen(state, rightSide.mid | rightSide.in | leftSide.mid | leftSide.in) ||
                rightSide.backing || leftSide.backing;
  if (atSide) vy = max(vy, 0.0f);

  // Sideways: default speed, capped by each side line, scaled by the goal, capped again
  float rightLim = sideLimit(rightSide, state);
  float leftLim  = sideLimit(leftSide,  state);
  float vx = SIDE_SPEED;
  vx = max(min(vx, rightLim), -leftLim);
  //vx = goalAdjust(vx);
  vx = max(min(vx, rightLim), -leftLim); // side lines always win

  Vector_Motion(vx, vy, 0);
}
