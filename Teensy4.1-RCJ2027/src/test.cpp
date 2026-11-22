#include <Arduino.h>

// Top camera decode test (act core), library not used
// MaixCam on Serial3 (pins 14/15), 115200
// Request: send 0xDD, camera answers once (after its current frame):
//   CC DD found angleL angleH distL distH checksum EE
//   checksum = (DD + found + angleL + angleH + distL + distH) & 0xFF
//   angle 0-359 deg, dist from the distance formula, both 0xFFFF when no goal

#define CAM          Serial3
#define TIMEOUT_MS   100 // wait this long for an answer
#define REQUEST_MS   50  // send a request every 50 ms

struct Goal {
  bool found;
  uint16_t angle, dist;
};

uint32_t okCount = 0, timeoutCount = 0, badCount = 0;

// Wait for one packet, true if a valid one arrived within TIMEOUT_MS
bool readPacket(Goal& g) {
  uint8_t b[9];
  int n = 0;
  uint32_t start = millis();

  while (millis() - start < TIMEOUT_MS) {
    if (!CAM.available()) continue;
    uint8_t c = CAM.read();

    if (n == 0 && c != 0xCC) continue;                      // wait for start byte
    if (n == 1 && c != 0xDD) { n = (c == 0xCC); continue; } // resync
    b[n++] = c;
    if (n < 9) continue;

    n = 0;
    uint8_t sum = 0;
    for (int i = 1; i <= 6; i++) sum += b[i];
    if (b[8] != 0xEE || sum != b[7]) { badCount++; continue; } // broken packet, keep looking

    g.found = b[2];
    g.angle = b[3] | (b[4] << 8);
    g.dist  = b[5] | (b[6] << 8);
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  CAM.begin(115200);
  Serial.println("Camera decode test: send 0xDD, read CC DD ... EE");
}

void loop() {
  while (CAM.available()) CAM.read(); // drop stale bytes
  CAM.write(0xDD);                    // request

  uint32_t t = millis();
  Goal g;
  bool ok = readPacket(g);
  t = millis() - t;

  if (!ok) {
    timeoutCount++;
    Serial.printf("timeout                          | ok %lu  timeout %lu  bad %lu\n",
                  okCount, timeoutCount, badCount);
  } else {
    okCount++;
    if (g.found) Serial.printf("found 1  angle %3u  dist %5u  %3lu ms", g.angle, g.dist, t);
    else         Serial.printf("found 0  (no goal)             %3lu ms", t);
    Serial.printf(" | ok %lu  timeout %lu  bad %lu\n", okCount, timeoutCount, badCount);
  }

  delay(REQUEST_MS);
}
