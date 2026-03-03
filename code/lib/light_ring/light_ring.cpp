#include "light_ring.h"
#include "angle_utils.h"
#include <vector>

// --- GLOBAL VARIABLE DEFINITIONS ---

int ldr_values[32];
bool ldr_threshold_pass[32];
int filtered_ldr_values[16];
bool filtered_ldr_threshold_pass[16];

// --- MUX CONTROL ---

void selectMuxChannel(int n) {
    int binaryNum[4] = {0, 0, 0, 0};
    int tempN = n;
    int i = 0;

    while (tempN > 0 && i < 4) {
        binaryNum[i] = tempN % 2;
        tempN = tempN / 2;
        i++;
    }

    // S0 = LSB, S3 = MSB
    digitalWrite(S0, binaryNum[0]);
    digitalWrite(S1, binaryNum[1]);
    digitalWrite(S2, binaryNum[2]);
    digitalWrite(S3, binaryNum[3]);
}

// --- SENSOR PROCESSING ---

void checkLightRing() {
    // Mux 1 (indices 0-15)
    for (int i = 0; i < 16; i++) {
        selectMuxChannel(i);
        delayMicroseconds(MUX_SETTLE_US);
        int ldrVal = analogRead(M1);
        ldr_values[i] = ldrVal;
        ldr_threshold_pass[i] = (ldrVal >= LDR_THRESHOLD);
    }

    // Mux 2 (indices 16-31)
    for (int i = 0; i < 16; i++) {
        selectMuxChannel(i);
        delayMicroseconds(MUX_SETTLE_US);
        int ldrVal = analogRead(M2);
        ldr_values[i + 16] = ldrVal;
        ldr_threshold_pass[i + 16] = (ldrVal >= LDR_THRESHOLD);
    }

    // Average adjacent pairs to produce 16 logical readings
    for (int i = 0; i < 16; i++) {
        filtered_ldr_values[i] = (ldr_values[i * 2] + ldr_values[i * 2 + 1]) / 2;
        filtered_ldr_threshold_pass[i] = (filtered_ldr_values[i] >= LDR_THRESHOLD);
    }
}

std::pair<double, double> findLine() {
    std::vector<uint8_t> matches;
    for (int i = 0; i < 16; i++) {
        if (filtered_ldr_threshold_pass[i]) matches.push_back(i);
    }

    if (matches.size() <= 1) return {NAN, NAN};

    double maxAngleDifference = 0;
    double lineStartAngle = NAN, lineEndAngle = NAN;

    for (size_t i = 0; i < matches.size() - 1; i++) {
        for (size_t j = i + 1; j < matches.size(); j++) {
            double angleI = matches[i] * 22.5;
            double angleJ = matches[j] * 22.5;
            double diff = smallerAngleDifference(angleI, angleJ);

            if (diff > maxAngleDifference) {
                maxAngleDifference = diff;
                lineStartAngle = angleI;
                lineEndAngle = angleJ;
            }
        }
    }

    if (std::isnan(lineStartAngle)) return {NAN, NAN};

    double bisector = angleBisector(lineStartAngle, lineEndAngle);
    double lineSize = maxAngleDifference / 180.0;

    return {bisector, lineSize};
}

// --- DEBUG ---

void debugLDRValues() {
    Serial.println("--- LDR Averaged Values ---");
    for (int i = 0; i < 16; i++) {
        Serial.print("LDR[");
        Serial.print(i);
        Serial.print("]: ");
        Serial.print(filtered_ldr_values[i]);
        Serial.print(" (");
        Serial.print(filtered_ldr_threshold_pass[i] ? "PASS" : "FAIL");
        Serial.print(")");
        if (i < 15) Serial.print(" | ");
    }
    Serial.println();
}
