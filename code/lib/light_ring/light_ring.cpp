#include "light_ring.h"
#include "angle_utils.h"
#include <vector>

// --- GLOBAL VARIABLE DEFINITIONS ---

int ldr_values[32];
bool ldr_threshold_pass[32];

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
    // Mux 1 (indices 0–15)
    for (int i = 0; i < 16; i++) {
        selectMuxChannel(i);
        delayMicroseconds(MUX_SETTLE_US);
        int sum = 0;
        for (int s = 0; s < LDR_SAMPLES; s++) sum += analogRead(M1);
        ldr_values[i] = sum / LDR_SAMPLES;
        ldr_threshold_pass[i] = (ldr_values[i] >= LDR_THRESHOLDS[i]);
    }

    // Mux 2 (indices 16–31)
    for (int i = 0; i < 16; i++) {
        selectMuxChannel(i);
        delayMicroseconds(MUX_SETTLE_US);
        int sum = 0;
        for (int s = 0; s < LDR_SAMPLES; s++) sum += analogRead(M2);
        ldr_values[i + 16] = sum / LDR_SAMPLES;
        ldr_threshold_pass[i + 16] = (ldr_values[i + 16] >= LDR_THRESHOLDS[i + 16]);
    }
}

// --- LINE DETECTION ---

// 32 sensors evenly spaced at 11.25° each
std::pair<double, double> findLine() {
    std::vector<uint8_t> matches;
    for (int i = 0; i < 32; i++) {
        if (ldr_threshold_pass[i]) matches.push_back(i);
    }

    if (matches.size() <= 1) return {NAN, NAN};

    double maxAngleDifference = 0;
    double lineStartAngle = NAN, lineEndAngle = NAN;

    for (size_t i = 0; i < matches.size() - 1; i++) {
        for (size_t j = i + 1; j < matches.size(); j++) {
            double angleI = matches[i] * 11.25;
            double angleJ = matches[j] * 11.25;
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
    for (int i = 0; i < 32; i++) {
        Serial.print(ldr_threshold_pass[i] ? 1 : 0);
        if (i < 31) Serial.print(" ");
    }
    Serial.println();
}
