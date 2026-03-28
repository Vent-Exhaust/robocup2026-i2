#include "main.h"

SerialComm l1Comm(L1_TO_L2_SERIAL);

static const bool DEBUG = false;

void setup() {
    Serial.begin(115200);
    L1_TO_L2_SERIAL.begin(115200);

    analogReadResolution(12);

    pinMode(M1, INPUT);
    pinMode(M2, INPUT);
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);

    if (DEBUG) Serial.println("System Initialised");
}

void loop() {
    // selectMuxChannel(3);
    checkLightRing();
    if (DEBUG) debugLDRValues();
    auto [angle, size] = findLine();

    if (!std::isnan(angle)) {
        if (DEBUG) {
            Serial.print("Line Detected! Angle: ");
            Serial.print(angle);
            Serial.print(" Size: ");
            Serial.println(size);
        }

        int16_t tx[] = { 1, (int16_t)(angle * 10), (int16_t)(size * 10) };
        l1Comm.write(tx, 3);
    } else {
        if (DEBUG) Serial.println("Searching for line...");

        int16_t tx[] = { 0, 0, 0 };
        l1Comm.write(tx, 3);
    }

    delay(LOOP_DELAY_MS);
}
