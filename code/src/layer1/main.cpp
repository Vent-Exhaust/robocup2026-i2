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
    LineResult line = findLine();

    if (!std::isnan(line.angle)) {
        if (DEBUG) {
            Serial.print("Line Detected! Angle: ");
            Serial.print(line.angle);
            Serial.print(" Size: ");
            Serial.print(line.size);
            Serial.print(" Start: ");
            Serial.print(line.startLdr);
            Serial.print(" End: ");
            Serial.println(line.endLdr);
        }

        int16_t tx[] = {
            1,
            (int16_t)(line.angle * 10),
            (int16_t)(line.size * 10),
            (int16_t)line.startLdr,
            (int16_t)line.endLdr
        };
        l1Comm.write(tx, 5);
    } else {
        if (DEBUG) Serial.println("Searching for line...");

        int16_t tx[] = { 0, 0, 0, -1, -1 };
        l1Comm.write(tx, 5);
    }

    delay(LOOP_DELAY_MS);
}
