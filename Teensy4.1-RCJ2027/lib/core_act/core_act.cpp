#include <core_act.h>
GyroData gyroData;
// --- OLED OBJECT ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void core_act_init() {
    Serial.begin(115200);
    Serial2.begin(115200);
    Serial8.begin(921600);
    pinMode(LED_BUILTIN, OUTPUT);

    Wire.begin();
    Wire.setClock(400000); // Fast I2C for OLED

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        for(;;); // Lock if OLED fails
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    drawMessage("Starting");
}

void update_gyro_sensor(){
  const int PACKET_SIZE = 19;
  uint8_t buffer[PACKET_SIZE];
  gyroData.exist = false; // Reset flag before read attempt

  while (Serial2.available() >= PACKET_SIZE){
    buffer[0] = Serial2.read();
    if(buffer[0] != 0xAA) continue;
    buffer[1] = Serial2.read();
    if(buffer[1] != 0xAA) continue;

    // Read remaining 17 bytes
    for (int i = 2; i < PACKET_SIZE; i++){
      buffer[i] = Serial2.read();
    }

    // --- Checksum: sum of bytes [2..16], mod 256 ---
    uint8_t esti_checksum = 0;
    for (int i = 2; i <= 16; i++){
      esti_checksum += buffer[i];
    }
    esti_checksum %= 256;

    // Compare with buffer[18]
    if(esti_checksum != buffer[18]){
      //Serial.println("Checksum error");
      continue;
    }

    // --- Extract yaw (Little Endian) ---
    int16_t yaw_raw = (int16_t)((buffer[4] << 8) | buffer[3]);
    int16_t pitch_raw = (int16_t)((buffer[6] << 8) | buffer[5]);
    int16_t roll_raw = (int16_t)((buffer[8] << 8) | buffer[7]);

    //Serial.print("yaw_raw: ");
    //Serial.println(yaw_raw);

    // Convert to degrees if within range
    if(abs(yaw_raw) <= 18000){
      gyroData.heading = yaw_raw * 0.01f;
      gyroData.exist = true;
    }

    if(abs(pitch_raw) <= 18000){
      gyroData.pitch = pitch_raw * 0.01f;
    }

    if(abs(roll_raw) <= 18000){
      gyroData.roll = roll_raw * 0.01f;
    }
    // Keep reading: drain all queued packets so the newest one wins
  }
}

void drawMessage(const char* msg) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println(msg);
    display.display();
}