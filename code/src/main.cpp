#include <Arduino.h>
#include <algorithm>
#include <vector>
#include <cmath>

// Pin Definitions from your new snippet
#define M1 19
#define M2 18
#define S0 14
#define S1 15
#define S2 16
#define S3 17

// Global variables (assuming these exist in your main.h or light_ring.h)
int ldr_values[32];
bool ldr_threshold_pass[32];
int filtered_ldr_values[16];
bool filtered_ldr_threshold_pass[16];
int ldr_threshold = 500; // Adjust based on calibration

// --- MUX LOGIC FROM YOUR SECOND SNIPPET ---

void selectMuxChannel(int n) {
    // This replicates your logic: S0 gets bit 3, S1 gets bit 2, etc.
    int binaryNum[4] = {0, 0, 0, 0}; 
    int tempN = n;
    int i = 0;

    while (tempN > 0 && i < 4) {
        binaryNum[i] = tempN % 2;
        tempN = tempN / 2;
        i++;
    }
    
    // Applying the specific mapping from your snippet
    digitalWrite(S0, binaryNum[3]);
    digitalWrite(S1, binaryNum[2]);
    digitalWrite(S2, binaryNum[1]);
    digitalWrite(S3, binaryNum[0]);
}

// --- CORE UTILITIES ---

double clipAngleTo360(double angle) {
    angle = fmod(angle, 360);
    return angle < 0 ? angle + 360 : angle;
}

double smallerAngleDifference(double leftAngle, double rightAngle) {
    double diff = fmod(fabs(rightAngle - leftAngle), 360);
    return diff > 180 ? 360 - diff : diff;
}

double angleBisector(double leftAngle, double rightAngle) {
    double diff = smallerAngleDifference(leftAngle, rightAngle);
    return clipAngleTo360(leftAngle + diff / 2.0);
}

// --- SENSOR PROCESSING ---

void checkLightRing() {
    // Mux 1 (Indices 0-15)
    for (int i = 0; i < 16; i++) {
        selectMuxChannel(i);
        delayMicroseconds(10); // Small delay for signal settling
        int ldrVal = analogRead(M1);
        ldr_values[i] = ldrVal;
        ldr_threshold_pass[i] = (ldrVal >= ldr_threshold);
    }

    // Mux 2 (Indices 16-31)
    for (int i = 0; i < 16; i++) {
        selectMuxChannel(i);
        delayMicroseconds(10);
        int ldrVal = analogRead(M2);
        ldr_values[i + 16] = ldrVal;
        ldr_threshold_pass[i + 16] = (ldrVal >= ldr_threshold);
    }

    // Downsampling/Filtering to 16 sensors if needed
    for (int i = 0; i < 16; i++) {
        filtered_ldr_threshold_pass[i] = ldr_threshold_pass[i * 2];
        filtered_ldr_values[i] = ldr_values[i * 2];
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

// --- SETUP & LOOP ---

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    
    pinMode(M1, INPUT);
    pinMode(M2, INPUT);
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    
    Serial.println("System Initialized with New Pinout");
}

void loop() {
    checkLightRing();
    auto [angle, size] = findLine();

    if (!std::isnan(angle)) {
        Serial.print("Line Detected! Angle: ");
        Serial.print(angle);
        Serial.print(" Size: ");
        Serial.println(size);
    } else {
        Serial.println("Searching for line...");
    }
    
    delay(50); 
}