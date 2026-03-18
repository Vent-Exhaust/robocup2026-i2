#include <Arduino.h>

// L2 receives on Serial4 (TX pin 17, RX pin 16)

void setup() {
    Serial.begin(115200);   // USB monitor
    Serial4.begin(115200);  // from L1
    Serial.println("L2 serial debug ready");
}

void loop() {
    if (Serial4.available()) {
        uint8_t b = Serial4.read();
        Serial.print("L2 received: ");
        Serial.println(b);
    }
}