#include "main.h"

using namespace std;
#define ll long long

void setupMotors() {
    //MOTOR 1
    pinMode(M1_IN_A, OUTPUT);
    pinMode(M1_IN_B, OUTPUT);
    pinMode(M1_PWM, OUTPUT);
    analogWriteFrequency(M1_PWM, 146484);

    //MOTOR 2
    pinMode(M2_IN_A, OUTPUT);
    pinMode(M2_IN_B, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    analogWriteFrequency(M2_PWM, 146484);

    //MOTOR 3
    pinMode(M3_IN_A, OUTPUT);
    pinMode(M3_IN_B, OUTPUT);
    pinMode(M3_PWM, OUTPUT);
    analogWriteFrequency(M3_PWM, 146484);

    //MOTOR 4
    pinMode(M4_IN_A, OUTPUT);
    pinMode(M4_IN_B, OUTPUT);
    pinMode(M4_PWM, OUTPUT);
    analogWriteFrequency(M4_PWM, 146484);

    Serial.println("Motors pinModes defined (by right...)");
}

void setup() {
    Serial.begin(115200);
    setupMotors();
}

void loop() {
}