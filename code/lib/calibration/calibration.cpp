#include "calibration.h"
#include "light_ring.h"
#include "config.h"
#include <Arduino.h>

void calibrateLightRing() {
    int minVal[32], maxVal[32];
    for (int i = 0; i < 32; i++) {
        minVal[i] = 0x7FFF;
        maxVal[i] = 0;
    }

    Serial.println("=== Light Ring Calibration ===");
    Serial.println("Press Enter to take a sample, 'd' + Enter when done.");

    int sampleCount = 0;

    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            while (Serial.available()) Serial.read();  // flush rest of line

            if (c == 'd') {
                break;
            } else if (c == '\n' || c == '\r') {
                for (int r = 0; r < CALIBRATION_READS_PER_SAMPLE; r++) {
                    checkLightRing();
                    for (int i = 0; i < 32; i++) {
                        if (ldr_values[i] < minVal[i]) minVal[i] = ldr_values[i];
                        if (ldr_values[i] > maxVal[i]) maxVal[i] = ldr_values[i];
                    }
                }
                sampleCount++;
                Serial.print("Sample ");
                Serial.print(sampleCount);
                Serial.println(" — current thresholds:");
                for (int i = 0; i < 32; i++) {
                    int threshold = (minVal[i] + maxVal[i]) / 2;
                    bool pass = ldr_values[i] >= threshold;
                    Serial.print("  [");
                    Serial.print(i);
                    Serial.print("] ");
                    Serial.print(minVal[i]);
                    Serial.print(" / ");
                    Serial.print(threshold);
                    Serial.print(" / ");
                    Serial.print(maxVal[i]);
                    Serial.print("  →  ");
                    Serial.println(pass ? "PASS" : "fail");
                }
            }
        }
    }

    Serial.println("\nDone. Paste into config.h:");
    Serial.print("constexpr int LDR_THRESHOLDS[32] = {");
    for (int i = 0; i < 32; i++) {
        int threshold = (minVal[i] + maxVal[i]) / 2;
        Serial.print(threshold);
        if (i < 31) Serial.print(", ");
    }
    Serial.println("};");

    Serial.println("\nPer-sensor detail (min / threshold / max):");
    for (int i = 0; i < 32; i++) {
        Serial.print("  [");
        Serial.print(i);
        Serial.print("] ");
        Serial.print(minVal[i]);
        Serial.print(" / ");
        Serial.print((minVal[i] + maxVal[i]) / 2);
        Serial.print(" / ");
        Serial.println(maxVal[i]);
    }

    while (true);  // halt — reflash to exit
}
