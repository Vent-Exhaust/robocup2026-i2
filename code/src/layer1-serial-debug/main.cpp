#include <Arduino.h>

// L1 sends over Serial1 (TX pin 1, RX pin 0)

uint8_t counter = 0;

void setup() {
    Serial.begin(115200);   // USB monitor
    Serial1.begin(115200);  // to L2
    Serial.println("L1 serial debug ready");
}

void loop() {
    Serial1.write(counter);

    Serial.print("L1 sent: ");
    Serial.println(counter);

    counter++;
    delay(500);
}
