#include <Arduino.h>
#include <core_act.h>

// Sprint test (Act): find the fastest speed ramp (ACCEL_MAX) that does not slip.
// No USB cable needed while driving: every run is stored in RAM, print it afterwards.
//
//   Each START press: 1 s countdown, sprint forward at SPRINT_SPEED for SPRINT_MS, then stop.
//   Every run uses the next ramp in ACCELS (shown on the OLED), last one = no ramp (old way).
//   Watch the wheels: the first value where they slip / the robot jumps is too high.
//   Afterwards plug USB back in, open the serial monitor and type  p  (+ Enter) -> CSV of all runs.
//   c = clear all runs.  Lift the robot or touch a line: the run stops at once.
//
// Logged every 5 ms: target speed, speed actually sent (after the ramp), IMU ax / ay, heading.
// Smooth "out" rising to the target and no spike in ax / ay = no slip.

#define SPRINT_SPEED 75.0f   // % (MOTOR_SPEED_LIMIT)
#define SPRINT_MS    500     // full-speed request time, then target 0
#define LOG_MS       1200    // logged time per run
#define LOG_EVERY_MS 5
#define LOG_N        (LOG_MS / LOG_EVERY_MS)

static const float ACCELS[] = {200, 300, 400, 600, 800, 0};   // %/s, 0 = no ramp
static const int RUNS = sizeof(ACCELS) / sizeof(ACCELS[0]);

struct Sample { uint16_t t; float target, out, ax, ay, heading; };
static Sample logBuf[RUNS][LOG_N];
static int logLen[RUNS];
static bool runAborted[RUNS];
static int nextRun = 0;

static void showIdle() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  if (nextRun < RUNS) {
    display.printf("SPRINT run %d/%d\n", nextRun + 1, RUNS);
    if (ACCELS[nextRun] > 0) display.printf("ramp %.0f %%/s\n", ACCELS[nextRun]);
    else                     display.println("NO ramp (old)");
    display.println("START = go (1 s)");
  } else {
    display.println("all runs done");
    display.println("USB + serial: p");
  }
  display.display();
}

static void showDone(int r) {
  float pax = 0, pay = 0;
  for (int i = 0; i < logLen[r]; i++) {
    pax = max(pax, fabsf(logBuf[r][i].ax));
    pay = max(pay, fabsf(logBuf[r][i].ay));
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.printf("run %d done%s\n", r + 1, runAborted[r] ? " ABORT" : "");
  display.printf("ramp %.0f\n", ACCELS[r]);
  display.printf("peak ax %.1f\n", pax);
  display.printf("peak ay %.1f m/s2\n", pay);
  display.println("put robot back");
  display.display();
  delay(1500);
}

static void printCsv() {
  Serial.println("run,ramp_pct_per_s,aborted,t_ms,target,out,ax,ay,heading");
  for (int r = 0; r < RUNS; r++)
    for (int i = 0; i < logLen[r]; i++) {
      const Sample& s = logBuf[r][i];
      Serial.printf("%d,%.0f,%d,%u,%.1f,%.1f,%.2f,%.2f,%.1f\n", r + 1, ACCELS[r], runAborted[r],
                    s.t, s.target, s.out, s.ax, s.ay, s.heading);
    }
  Serial.println("end");
}

static void runSprint(int r) {
  for (uint32_t t0 = millis(); millis() - t0 < 1000;) update_gyro_sensor();   // countdown

  robot.accel_max = ACCELS[r];
  robot.decel_max = ACCELS[r] * 1.5f;
  set_ramp(ACCELS[r] > 0);
  set_target_heading(90);
  MotorStop();

  logLen[r] = 0;
  runAborted[r] = false;
  uint32_t start = millis(), lastLog = 0;
  bool first = true;
  while (millis() - start < LOG_MS) {
    update_gyro_sensor();
    read_sense();
    if (picked_up() || line_flag()) { runAborted[r] = true; break; }   // safety

    uint32_t t = millis() - start;
    float target = t < SPRINT_MS ? SPRINT_SPEED : 0;
    Vector_Motion(0, target, 0);

    if ((first || t - lastLog >= LOG_EVERY_MS) && logLen[r] < LOG_N) {
      first = false;
      lastLog = t;
      Sample& s = logBuf[r][logLen[r]++];
      s.t = t;
      s.target = target;
      s.out = motion_output().y;
      s.ax = gyroData.ax;
      s.ay = gyroData.ay;
      s.heading = current_heading();
    }
  }
  MotorStop();
  robot.accel_max = ACCEL_MAX;
  robot.decel_max = DECEL_MAX;
}

void setup() {
  core_act_init();
  strategy_begin();
  MotorStop();
  wait_start("SPRINT");
  showIdle();
}

void loop() {
  update_gyro_sensor();
  read_sense();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'p') printCsv();
    if (c == 'c') { for (int r = 0; r < RUNS; r++) logLen[r] = 0; nextRun = 0; showIdle(); Serial.println("cleared"); }
  }

  if (start_pressed() && nextRun < RUNS) {
    int r = nextRun++;
    runSprint(r);
    showDone(r);
    showIdle();
  }
}
