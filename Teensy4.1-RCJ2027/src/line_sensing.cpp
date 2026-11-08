#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  Serial.println("Buffer Test");
  Serial3.begin(115200);
  pinMode(40, OUTPUT);
  digitalWrite(40, HIGH); // Set pin 40 high to indicate the start of the test
}

void loop() {
    // Add your main code here
    if(Serial3.available() > 0) {
        uint8_t byteReceived = Serial3.read();
        Serial.print("Received byte: ");
        Serial.println(byteReceived, HEX);
    }
}