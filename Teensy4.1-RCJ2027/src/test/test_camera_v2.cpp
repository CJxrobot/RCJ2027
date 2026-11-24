#include <Arduino.h>
#include <core_act.h>

// T04 (Act): /field_data v2 over Serial3 DMA.
// Maix Cam runs main.py (or camera_test/cam_fake_v2.py for fake data). Motors never move.
// Pass: >100 pkts/s (main.py measured ~120), lost and bad not increasing
//       ball front/right/back/left -> about 90 / 0 / 270 / 180 deg

static void printObj(const char* name, const FieldObj& o, const char* unit) {
  if (o.seen) Serial.printf("%s %3.0f deg %4u %s  ", name, o.angle, o.dist, unit);
  else        Serial.printf("%s ---            ", name);
}

void setup() {
  core_act_init();
  camera_begin();
  MotorStop();
  drawMessage("T04 cam");
  Serial.println("T04 test_camera_v2");
}

void loop() {
  read_camera();

  static CameraStats prev;
  EVERY_MS(tp, 100) {
    printObj("ball", fieldData.ball, "px");
    printObj("front", fieldData.front_goal, "cm");
    printObj("back", fieldData.back_goal, "cm");
    Serial.printf("age %lu ms\n", fieldData.t_ms ? millis() - fieldData.t_ms : 0);
  }
  EVERY_MS(ts, 1000) {
    CameraStats s = cameraStats;
    Serial.printf("== pkts/s %lu | lost total %lu | bad total %lu\n", s.pkts - prev.pkts, s.lost, s.bad);
    prev = s;
  }
}
