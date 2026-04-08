#include <Arduino.h>
#include "layer3_comm.h"

void setup() {
    Serial.begin(115200);
    setupL3Comm();
    Serial.println("[IR DEBUG] ready — printing L3 IR + switch data");
}

void loop() {
    if (readL3()) {
        debugL3Readings();
    }
}
