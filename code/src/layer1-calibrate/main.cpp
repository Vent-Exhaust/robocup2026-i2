#include <Arduino.h>
#include "light_ring.h"
#include "calibration.h"

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);

    pinMode(M1, INPUT);
    pinMode(M2, INPUT);
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);

    // while (!Serial);  // wait for Serial Monitor to open
}

void loop() {
    calibrateLightRing();
}
