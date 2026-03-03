#include "main.h"

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);

    pinMode(M1, INPUT);
    pinMode(M2, INPUT);
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);

    Serial.println("System Initialized");
}

void loop() {
    checkLightRing();
    // debugLDRValues();
    auto [angle, size] = findLine();

    if (!std::isnan(angle)) {
        Serial.print("Line Detected! Angle: ");
        Serial.print(angle);
        Serial.print(" Size: ");
        Serial.println(size);
    } else {
        Serial.println("Searching for line...");
    }

    delay(LOOP_DELAY_MS);
}
