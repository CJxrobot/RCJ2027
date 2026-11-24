// Simple 2D world for the act programs: field, robot, ball, sensors.
// The robot program (setup/loop) runs unchanged; this file only
//   - sends it the same packets the real sensors send (gyro, sense, camera)
//   - reads its motor pins and moves the robot
//   - records everything into one HTML file (sim/out/...)
//
// Coordinates: cm, field centre = (0, 0), x = right, y = toward the opponent goal.
// Our goal is at the bottom (y < 0). Robot heading: 90 = facing the opponent goal, CCW.
#include <Arduino.h>
#include <core_act.h>

void setup();
void loop();

#ifdef _WIN32
// MinGW does not link the __attribute__((weak)) US_Sensor() in core_act.cpp: same empty one here
UsData US_Sensor() { return UsData(); }
#endif

// ===================== Numbers (change here) =====================
// Field (RCJ 2025 rules): playing area 158 x 219, out area 12, lines 2 cm wide.
// Line positions below are line centres.
const float LINE_X = 78.0f, LINE_Y = 108.5f;    // outer white line
const float WALL_X = 91.0f, WALL_Y = 121.5f;    // walls
const float GOAL_HALF = 30.0f;                  // goal 60 cm wide
const float PA_HALF = 39.0f, PA_DEPTH = 24.0f;  // penalty area 80 x 25, line centres
const float PA_R = 15.0f;                       // rounded front corners
const float LINE_SEEN_CM = 1.5f;                // sensor sees the line within this

// Robot (from the user): 100 % = 300 cm/s, body 22 cm
const float ROBOT_R = 11.0f;
const float CM_PER_PCT = 3.0f;                  // body speed cm/s per % (100 % -> 300 cm/s)
const float WHEEL_R = 9.0f;                     // wheel distance from the centre (guess)
const float MOTOR_TAU = 0.05f;                  // motor reaches 63 % of a step in 50 ms (guess)

// Line sensors in the robot frame (angle: 0 = right, 90 = front; radius cm)
// mux 1: ring of 32 sensors on a 15 cm circle, 2 sensors per channel.
//        Channel ch = the pair at ch*22.5 +- 5.625 deg (sees the line if either sensor does).
// mux 2 (defense): front F_in inside the ring, F_out outside it;
//        0 / 180 / 270 arms: in and mid used (out sensor not used, not modelled).
struct LineSensor { int bit; float deg, r; };
std::vector<LineSensor> lineSensors() {
  std::vector<LineSensor> s;
  for (int ch = 0; ch < 16; ch++) {
    s.push_back({ch, ch * 22.5f - 5.625f, 7.5f});
    s.push_back({ch, ch * 22.5f + 5.625f, 7.5f});
  }
#if SIM_MUX2
  s.push_back({16 + 7,  90, 5.5f});   // F_in  (inside the ring)
  s.push_back({16 + 11, 90, 9.5f});   // F_out (outside the ring)
  s.push_back({16 + 3, 270, 8.5f});   // B_in
  s.push_back({16 + 4, 270, 9.5f});   // B_mid
  s.push_back({16 + 0, 180, 8.5f});   // L_in
  s.push_back({16 + 1, 180, 9.5f});   // L_mid
  s.push_back({16 + 13,  0, 8.5f});   // R_in
  s.push_back({16 + 14,  0, 9.5f});   // R_mid
#endif
  return s;
}

// Ball
const float BALL_R = 2.1f;
const float BALL_FRICTION = 30.0f;              // cm/s^2
const float KICK_SPEED = 250.0f;                // cm/s
const float HOLD_DEG = 15.0f;                   // ball within 90 +- this and touching = held

// Sensors
const float IR_RANGE = 200.0f;                  // cm
const float IR_NOISE_DEG = 3.0f;
const uint32_t SENSE_US = 2000, CAMERA_US = 5000, GYRO_US = 10000;   // 500 / 200 / 100 Hz
const uint32_t LOOP_US = 100;                   // one loop() = 100 us of simulated time
const uint32_t FRAME_MS = 20;                   // recording rate

// IR strength (number of sensors that see the ball, 1-20). Guess: 20 at contact, about 8 at 40 cm.
int irStrength(float d) { return constrain((int)lroundf(20.0f * expf(-(d - 13.0f) / 45.0f)), 1, 20); }
// Camera ball distance in pixels. Guess, only used if the program uses the camera ball.
int camPixels(float d) { return constrain((int)lroundf(35.0f + d * 0.5f), 35, 150); }

// ===================== Scenarios =====================
struct Key { float t, x, y; bool visible; };     // ball position at time t (straight lines between)
struct Scenario {
  const char* name;
  const char* desc;
  float rx, ry, rh, secs;                       // robot start and length
  std::vector<Key> ball;
  float camOffFrom = -1, camOffTo = -1;         // our goal hidden from the camera in this time (s)
  float pushT = -1, pushX = 0, pushY = 0;       // at pushT the robot is moved to (pushX, pushY)
};

std::vector<Scenario> scenarios = {
  {"ball_front", "Ball still in front of the penalty area: DEFEND 3 s -> LEAVE -> take it, shoot -> RETURN",
   0, -93, 90, 14, {{0, 0, -30, true}}},
  {"ball_side", "Ball rolls left and right in front of the penalty area: robot follows sideways",
   0, -93, 90, 14, {{0, -60, -45, true}, {3, 60, -45, true}, {6, -60, -45, true}, {9, 60, -45, true}, {12, -60, -45, true}}},
  {"no_ball", "No ball, robot starts 25 cm right: it should centre on the goal (camera)",
   25, -93, 90, 6, {{0, 0, 0, false}}},
  {"corner_right", "Robot at the right corner, ball still straight in front of it: should LEAVE after 3 s",
   30, -93, 90, 10, {{0, 30, -45, true}}},
  {"corner_left", "Robot at the left corner, ball still straight in front of it: should LEAVE after 3 s",
   -30, -93, 90, 10, {{0, -30, -45, true}}},
  {"return_cam_off", "Corner LEAVE, our goal hidden from the camera 3.7-5.0 s (during RETURN): robot must stop, then go on",
   30, -93, 90, 10, {{0, 30, -45, true}}, 3.7f, 5.0f},
  {"side_far", "Ball far right outside the penalty area: robot must stop at the side, never cross the side line",
   0, -93, 90, 8, {{0, 75, -70, true}}},
  {"pushed_out", "Ball far right; at 2 s an opponent pushes the robot 60 cm out: RETURN, back behind the line",
   0, -93, 90, 7, {{0, 75, -70, true}}, -1, -1, 2.0f, 0, -33},
  {"ball_lost", "Ball in front, removed at 3.3 s (right after LEAVE): RETURN -> DEFEND",
   0, -93, 90, 8, {{0, 0, -30, true}, {3.3f, 0, -30, false}}},
};

// ===================== World state =====================
struct Robot { float x, y, h, vx, vy, w; float wheel[4]; } bot;   // h: deg; vx, vy: world cm/s; w: deg/s
struct Ball { float x, y, vx, vy; bool visible, scripted; } ball;
const Scenario* sc;
std::vector<LineSensor> sensors;
uint32_t lineState = 0xFFFFFFFF;
uint64_t nextSense = 0, nextCamera = 0, nextGyro = 0, nextFrame = 0;
uint8_t camSeq = 0, gyroIdx = 0;
int lastKickPin = 0;

struct Frame { float t, rx, ry, rh, bx, by; int bvis; float wheel[4]; uint32_t line; int flag, hold; };
std::vector<Frame> frames;

void simLog(const char* text) { sim_console.push_back({millis(), std::string("[SIM] ") + text}); }

float wrap360(float a) { a = fmodf(a, 360.0f); return a < 0 ? a + 360.0f : a; }
float randf(float a) { return a * (2.0f * rand() / RAND_MAX - 1.0f); }

// Direction of a world point in the robot frame (0 = right, 90 = front)
float robotAngle(float x, float y) {
  return wrap360(atan2f(y - bot.y, x - bot.x) * RAD_TO_DEG - bot.h + 90.0f);
}
float dist(float x, float y) { return hypotf(x - bot.x, y - bot.y); }

// Robot frame point -> world
void toWorld(float deg, float r, float& x, float& y) {
  float a = (bot.h - 90.0f + deg) * DEG_TO_RAD;
  x = bot.x + r * cosf(a);
  y = bot.y + r * sinf(a);
}

// ===================== Field lines =====================
float segDist(float px, float py, float ax, float ay, float bx, float by) {
  float dx = bx - ax, dy = by - ay;
  float t = constrain(((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy), 0.0f, 1.0f);
  return hypotf(px - ax - t * dx, py - ay - t * dy);
}

// Distance from (x, y) to the nearest white line
float lineDist(float x, float y) {
  float d = 1e9f;
  d = min(d, segDist(x, y, -LINE_X, -LINE_Y,  LINE_X, -LINE_Y));
  d = min(d, segDist(x, y, -LINE_X,  LINE_Y,  LINE_X,  LINE_Y));
  d = min(d, segDist(x, y, -LINE_X, -LINE_Y, -LINE_X,  LINE_Y));
  d = min(d, segDist(x, y,  LINE_X, -LINE_Y,  LINE_X,  LINE_Y));
  for (int s = -1; s <= 1; s += 2) {               // s = -1: our penalty area, +1: theirs
    float yGoal  = s * LINE_Y;
    float yFront = s * (LINE_Y - PA_DEPTH);
    float yArc   = s * (LINE_Y - PA_DEPTH + PA_R); // centre of the rounded corners
    d = min(d, segDist(x, y, -PA_HALF + PA_R, yFront, PA_HALF - PA_R, yFront));
    d = min(d, segDist(x, y, -PA_HALF, yGoal, -PA_HALF, yArc));
    d = min(d, segDist(x, y,  PA_HALF, yGoal,  PA_HALF, yArc));
    for (int k = -1; k <= 1; k += 2) {             // left / right corner (quarter circle)
      float dx = x - k * (PA_HALF - PA_R), dy = y - yArc;
      if (dx * k >= 0 && dy * -s >= 0) d = min(d, fabsf(hypotf(dx, dy) - PA_R));
    }
  }
  return d;
}

// ===================== Packets (same bytes as the real sensors) =====================
void sendGyro() {
  uint8_t p[19] = {0xAA, 0xAA, gyroIdx++};
  int16_t yaw = (int16_t)lroundf(wrap180(90.0f - bot.h) * 100.0f);   // current_heading() = 90 - yaw
  p[3] = yaw & 0xFF;  p[4] = (yaw >> 8) & 0xFF;                       // pitch, roll = 0
  p[13] = 1000 & 0xFF; p[14] = 1000 >> 8;                             // az = 1 g
  uint8_t sum = 0;
  for (int i = 2; i <= 16; i++) sum += p[i];
  p[18] = sum;
  for (uint8_t b : p) Serial2.rx.push_back(b);
}

void sendSense() {
  lineState = 0xFFFFFFFF;
  for (const LineSensor& s : sensors) {
    float x, y;
    toWorld(s.deg, s.r, x, y);
    if (lineDist(x, y) <= LINE_SEEN_CM) lineState &= ~(1UL << s.bit);
  }
  bool line = lineState != 0xFFFFFFFF;
  sim_set_pin(PIN_0, line);                        // interrupt first, like sense.cpp
  sim_set_pin(PIN_1, line);

  float d = dist(ball.x, ball.y);
  bool found = ball.visible && d < IR_RANGE;
  uint16_t ang = found ? (uint16_t)wrap360(lroundf(robotAngle(ball.x, ball.y) + randf(IR_NOISE_DEG))) : 0xFFFF;
  uint16_t str = found ? irStrength(d) : 0;
  uint8_t p[SENSE_LEN] = {SENSE_HDR};
  for (int i = 0; i < 4; i++) p[1 + i] = (lineState >> (8 * i)) & 0xFF;
  p[5] = found;
  pkt_put16(p, 6, ang);
  pkt_put16(p, 8, str);
  p[10] = pkt_sum(p, 1, 9);
  for (uint8_t b : p) Serial8.rx.push_back(b);
}

void putObj(uint8_t* p, int i, bool seen, float x, float y, uint16_t d) {
  pkt_put16(p, i, seen ? (uint16_t)lroundf(robotAngle(x, y)) % 360 : PKT_NONE);
  pkt_put16(p, i + 2, seen ? d : PKT_NONE);
}

void sendCamera() {
  bool b = ball.visible;
  uint8_t p[CAM_LEN] = {CAM_H1, CAM_H2};
  p[2] = (b ? CAM_FLAG_BALL : 0) | CAM_FLAG_FRONT | CAM_FLAG_BACK;
  putObj(p, 3, b, ball.x, ball.y, camPixels(dist(ball.x, ball.y)));
  putObj(p, 7, true, 0, LINE_Y + 1, (uint16_t)lroundf(dist(0, LINE_Y + 1)));     // front goal (theirs)
  float t = sim_us / 1e6f;
  bool ours = !(t >= sc->camOffFrom && t < sc->camOffTo);
  if (!ours) p[2] &= ~CAM_FLAG_BACK;
  putObj(p, 11, ours, 0, -LINE_Y - 1, (uint16_t)lroundf(dist(0, -LINE_Y - 1)));  // back goal (ours)
  p[15] = camSeq++;
  p[16] = pkt_sum(p, 1, 15);
  p[17] = CAM_END;
  sim_camera_push(p, CAM_LEN);
}

// ===================== Physics =====================
// Wheel speeds (%) from the motor pins: PWM 0-255, DIR HIGH = forward (SetMotorSpeed)
void readWheels(float cmd[4]) {
  const uint8_t dirPins[4] = MOTOR_DIR_PINS, pwmPins[4] = MOTOR_PWM_PINS;
  for (int i = 0; i < 4; i++)
    cmd[i] = sim_pin(pwmPins[i]) * 100.0f / 255.0f * (sim_pin(dirPins[i]) ? 1 : -1);
}

// Wheels -> body speed: least-squares inverse of RobotIKControl()
//   w0 = -0.643 vx + 0.766 vy + om,  w1 = -0.643 vx - 0.766 vy + om
//   w2 =  0.707 vx - 0.707 vy + om,  w3 =  0.707 vx + 0.707 vy + om
void wheelsToBody(const float w[4], float& vx, float& vy, float& om) {
  const float cx[4] = {-0.643f, -0.643f, 0.707f, 0.707f};
  const float cy[4] = { 0.766f, -0.766f, -0.707f, 0.707f};
  float sxx = 0, syy = 0, sx = 0, bx = 0, by = 0, bo = 0;
  for (int i = 0; i < 4; i++) {
    sxx += cx[i] * cx[i];  syy += cy[i] * cy[i];  sx += cx[i];
    bx += cx[i] * w[i];    by += cy[i] * w[i];    bo += w[i];
  }
  vy = by / syy;                                   // vy does not mix with the others
  float det = sxx * 4 - sx * sx;                   // solve [sxx sx; sx 4] [vx om] = [bx bo]
  vx = (bx * 4 - sx * bo) / det;
  om = (sxx * bo - sx * bx) / det;
}

void moveRobot(float dt) {
  float cmd[4];
  readWheels(cmd);
  for (int i = 0; i < 4; i++) bot.wheel[i] += (cmd[i] - bot.wheel[i]) * dt / MOTOR_TAU;

  float vx, vy, om;                                // robot frame, %
  wheelsToBody(bot.wheel, vx, vy, om);
  float a = (bot.h - 90.0f) * DEG_TO_RAD;          // robot frame -> world
  bot.vx = (vx * cosf(a) - vy * sinf(a)) * CM_PER_PCT;
  bot.vy = (vx * sinf(a) + vy * cosf(a)) * CM_PER_PCT;
  bot.w  = om * CM_PER_PCT / WHEEL_R * RAD_TO_DEG; // + = CCW
  bot.x = constrain(bot.x + bot.vx * dt, -WALL_X + ROBOT_R, WALL_X - ROBOT_R);
  bot.y = constrain(bot.y + bot.vy * dt, -WALL_Y + ROBOT_R, WALL_Y - ROBOT_R);
  bot.h = wrap360(bot.h + bot.w * dt);
}

bool ballHeld() {
  float rel = robotAngle(ball.x, ball.y);
  return ball.visible && dist(ball.x, ball.y) < ROBOT_R + BALL_R + 1.0f && fabsf(rel - 90.0f) < HOLD_DEG;
}

// Ball follows the scenario keys until the robot touches it, then rolls freely.
// A key with visible = false always applies (the ball is taken away).
void scriptBall(float t) {
  const std::vector<Key>& k = sc->ball;
  if (k.empty()) return;                          // live mode: the mouse moves the ball
  size_t i = 0;
  while (i + 1 < k.size() && k[i + 1].t <= t) i++;
  if (!k[i].visible) { ball.visible = false; ball.x = 999; ball.y = 999; return; }
  if (!ball.scripted) return;
  float x = k[i].x, y = k[i].y;
  if (i + 1 < k.size() && k[i + 1].visible) {
    float f = (t - k[i].t) / (k[i + 1].t - k[i].t);
    x += (k[i + 1].x - k[i].x) * f;
    y += (k[i + 1].y - k[i].y) * f;
  }
  ball.vx = ball.visible ? (x - ball.x) / 0.0001f : 0;   // one step = 100 us
  ball.vy = ball.visible ? (y - ball.y) / 0.0001f : 0;
  ball.x = x;
  ball.y = y;
  ball.visible = true;
}

bool mouseHoldsBall = false;                       // live mode: ball under the mouse, no physics

void moveBall(float dt) {
  if (!ball.visible || mouseHoldsBall) return;
  if (!ball.scripted) {
    float v = hypotf(ball.vx, ball.vy);
    float k = v > 0 ? max(0.0f, v - BALL_FRICTION * dt) / v : 0;
    ball.vx *= k;
    ball.vy *= k;
    ball.x += ball.vx * dt;
    ball.y += ball.vy * dt;
  }
  // Goal: ball over a goal line between the posts
  if (fabsf(ball.y) > LINE_Y + 1 && fabsf(ball.x) < GOAL_HALF) {
    simLog(ball.y > 0 ? "GOAL (opponent goal)" : "OWN GOAL");
    ball.visible = false;
    ball.x = ball.y = 999;
    return;
  }
  // Walls: bounce
  if (fabsf(ball.x) > WALL_X - BALL_R) { ball.x = copysignf(WALL_X - BALL_R, ball.x); ball.vx *= -0.5f; }
  if (fabsf(ball.y) > WALL_Y - BALL_R) { ball.y = copysignf(WALL_Y - BALL_R, ball.y); ball.vy *= -0.5f; }

  // Robot pushes the ball
  float d = dist(ball.x, ball.y), touch = ROBOT_R + BALL_R;
  if (d < touch && d > 0) {
    if (ball.scripted) { ball.scripted = false; simLog("robot touched the ball"); }
    float nx = (ball.x - bot.x) / d, ny = (ball.y - bot.y) / d;
    ball.x = bot.x + nx * touch;
    ball.y = bot.y + ny * touch;
    float robotN = bot.vx * nx + bot.vy * ny, ballN = ball.vx * nx + ball.vy * ny;
    if (robotN > ballN) { ball.vx += (robotN - ballN) * nx; ball.vy += (robotN - ballN) * ny; }
  }

  // Kick: discharge pin goes HIGH with the ball held
  int kickPin = sim_pin(KICK_DISCHARGE_PIN);
  if (kickPin && !lastKickPin) {
    if (ballHeld()) {
      float a = bot.h * DEG_TO_RAD;
      ball.vx = KICK_SPEED * cosf(a);
      ball.vy = KICK_SPEED * sinf(a);
      simLog("KICK (ball held)");
    } else {
      simLog("kick without the ball");
    }
  }
  lastKickPin = kickPin;
}

void record() {
  Frame f = {sim_us / 1e6f, bot.x, bot.y, bot.h, ball.x, ball.y, ball.visible,
             {bot.wheel[0], bot.wheel[1], bot.wheel[2], bot.wheel[3]}, lineState, line_flag(), has_ball()};
  frames.push_back(f);
}

// Called for every bit of simulated time (also inside delay())
void sim_advance(uint32_t us) {
  const uint32_t STEP = 100;
  for (uint32_t done = 0; done < us; done += STEP) {
    sim_us += STEP;
    float dt = STEP * 1e-6f;
    scriptBall(sim_us / 1e6f);
    if (sc->pushT >= 0 && sim_us == (uint64_t)(sc->pushT * 1e6f)) {
      bot.x = sc->pushX; bot.y = sc->pushY; simLog("robot pushed");
    }
    moveRobot(dt);
    moveBall(dt);
    sim_set_pin(BALL_POSESSION, ballHeld() ? LOW : HIGH);
    if (sim_us >= nextSense)  { nextSense  += SENSE_US;  sendSense(); }
    if (sim_us >= nextCamera) { nextCamera += CAMERA_US; sendCamera(); }
    if (sim_us >= nextGyro)   { nextGyro   += GYRO_US;   sendGyro(); }
    if (sim_us >= nextFrame)  { nextFrame  += FRAME_MS * 1000; record(); }
  }
}

// ===================== Output =====================
std::string jsonText(const std::string& s) {
  std::string o = "\"";
  for (char c : s) {
    if (c == '"' || c == '\\') o += '\\';
    if ((unsigned char)c >= 32) o += c;
  }
  return o + "\"";
}

std::string readFile(const char* path) {
  std::string text;
  FILE* in = fopen(path, "rb");
  if (!in) { printf("%s not found (run from the sim folder)\n", path); return text; }
  char buf[4096];
  size_t n;
  while ((n = fread(buf, 1, sizeof(buf), in)) > 0) text.append(buf, n);
  fclose(in);
  return text;
}

// viewer.html with the data put in (data = "null": live mode)
std::string viewerPage(const std::string& data) {
  std::string page = readFile("viewer.html");
  size_t at = page.find("__SIM_DATA__");
  if (at != std::string::npos) page.replace(at, 12, data);
  return page;
}

std::string sensorsJson() {
  std::string j = "[";
  char t[64];
  for (size_t i = 0; i < sensors.size(); i++) {
    snprintf(t, sizeof(t), "%s[%.1f,%.1f,%d]", i ? "," : "", sensors[i].deg, sensors[i].r, sensors[i].bit);
    j += t;
  }
  return j + "]";
}

std::string frameJson(const Frame& f) {
  char t[256];
  snprintf(t, sizeof(t), "[%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%.0f,%.0f,%.0f,%.0f,%u,%d,%d]",
           f.t, f.rx, f.ry, f.rh, f.bx, f.by, f.bvis, f.wheel[0], f.wheel[1], f.wheel[2], f.wheel[3],
           f.line, f.flag, f.hold);
  return t;
}

std::string consoleJson(size_t from) {
  std::string j = "[";
  char t[32];
  for (size_t i = from; i < sim_console.size(); i++) {
    snprintf(t, sizeof(t), "%s[%u,", i > from ? "," : "", sim_console[i].ms);
    j += t + jsonText(sim_console[i].text) + "]";
  }
  return j + "]";
}

void writeHtml(const char* path) {
  std::string j = "{\"prog\":" + jsonText(SIM_PROG) + ",\"name\":" + jsonText(sc->name) +
                  ",\"desc\":" + jsonText(sc->desc) + ",\"sensors\":" + sensorsJson() + ",\"frames\":[";
  for (size_t i = 0; i < frames.size(); i++) j += (i ? "," : "") + frameJson(frames[i]);
  j += "],\"console\":" + consoleJson(0) + "}";

  std::string page = viewerPage(j);
  FILE* out = fopen(path, "wb");
  fwrite(page.data(), 1, page.size(), out);
  fclose(out);
}

// ===================== Live mode =====================
// The browser calls /step about 50 times a second:
//   /step?ms=20&drag=1&x=10&y=-30   drag 1 = ball held by the mouse at (x, y), 2 = robot, 0 = none
//   &hide=1                          take the ball off the field
// The robot program runs for ms of simulated time, the answer is the newest frame + new console lines.
int live_serve(int port, std::string (*handle)(const std::string& path, std::string& type));
Scenario liveScenario = {"live", "Live: click = put the ball there, drag = move / throw it, drag the robot to move it",
                         0, -93, 90, 0, {}};
size_t liveLogSent = 0;

float param(const std::string& path, const char* key, float fallback) {
  for (const char* sep : {"?", "&"}) {
    size_t at = path.find(std::string(sep) + key + "=");
    if (at != std::string::npos) return atof(path.c_str() + at + strlen(key) + 2);
  }
  return fallback;
}

std::string liveHandle(const std::string& path, std::string& type) {
  if (path == "/" || path.rfind("/index", 0) == 0) { type = "text/html"; return viewerPage("null"); }
  if (path.rfind("/info", 0) == 0)
    return "{\"prog\":" + jsonText(SIM_PROG) + ",\"desc\":" + jsonText(sc->desc) + ",\"sensors\":" + sensorsJson() + "}";
  if (path.rfind("/step", 0) != 0) return "{}";

  float ms = constrain(param(path, "ms", 20), 0.0f, 100.0f);
  int drag = (int)param(path, "drag", 0);
  float x = param(path, "x", 0), y = param(path, "y", 0);
  if (param(path, "hide", 0)) { ball.visible = false; ball.x = ball.y = 999; ball.vx = ball.vy = 0; }
  // Ball held by the mouse: it stays under the mouse; let go = thrown with the mouse speed
  bool& held = mouseHoldsBall;
  static float throwVx = 0, throwVy = 0;
  if (drag == 1) {
    float dt = max(ms, 5.0f) / 1000.0f;
    if (held) {                                   // smoothed mouse speed
      throwVx = 0.5f * throwVx + 0.5f * constrain((x - ball.x) / dt, -400.0f, 400.0f);
      throwVy = 0.5f * throwVy + 0.5f * constrain((y - ball.y) / dt, -400.0f, 400.0f);
    } else {
      throwVx = throwVy = 0;
    }
    held = true;
    ball.x = x;
    ball.y = y;
    ball.vx = ball.vy = 0;
    ball.visible = true;
  } else if (held) {
    held = false;
    ball.vx = throwVx;
    ball.vy = throwVy;
  }
  if (drag == 2) { bot.x = x; bot.y = y; }

  uint64_t until = sim_us + (uint64_t)(ms * 1000);
  while (sim_us < until) {
    loop();
    sim_advance(LOOP_US);
  }
  frames.clear();
  record();
  std::string j = "{\"f\":" + frameJson(frames.back()) + ",\"log\":" + consoleJson(liveLogSent) + "}";
  liveLogSent = sim_console.size();
  return j;
}

// ===================== main =====================
// usage: sim <scenario | all | live>   (run from the sim folder, output in sim/out/)
// Each scenario runs in its own process (the robot program keeps static state).
int main(int argc, char** argv) {
  const char* want = argc > 1 ? argv[1] : "all";
  if (!strcmp(want, "all")) {
    int fails = 0;
    for (const Scenario& s : scenarios) {
      std::string cmd = std::string("\"") + argv[0] + "\" " + s.name;
      fails += system(cmd.c_str()) != 0;
    }
    return fails;
  }
  bool live = !strcmp(want, "live");
  if (live) sc = &liveScenario;
  for (const Scenario& s : scenarios) if (!strcmp(s.name, want)) sc = &s;
  if (!sc) {
    printf("unknown scenario '%s'. Use: all, live, or one of:\n", want);
    for (const Scenario& s : scenarios) printf("  %-12s %s\n", s.name, s.desc);
    return 1;
  }

  srand(1);
  sensors = lineSensors();
  bot = {sc->rx, sc->ry, sc->rh, 0, 0, 0, {0, 0, 0, 0}};
  ball = {999, 999, 0, 0, false, !live};
  if (live) { ball.x = 0; ball.y = -30; ball.visible = true; }
  sim_set_pin(BALL_POSESSION, HIGH);              // no ball
  sim_set_pin(START_BUTTON, LOW);                 // START already pressed
  scriptBall(0);

  setup();
  if (live) return live_serve(8765, liveHandle);

  while (sim_us < (uint64_t)(sc->secs * 1e6f)) {
    loop();
    sim_advance(LOOP_US);
  }

  char path[128];
  snprintf(path, sizeof(path), "out/%s_%s.html", SIM_PROG, sc->name);
  writeHtml(path);
  printf("%-12s %5.1f s  -> %s\n", sc->name, sc->secs, path);
  for (const ConsoleLine& c : sim_console)          // state changes and simulator events
    if (c.text.find("->") != std::string::npos || c.text.rfind("[SIM]", 0) == 0)
      printf("    %6.2f s  %s\n", c.ms / 1000.0f, c.text.c_str());
  return 0;
}
