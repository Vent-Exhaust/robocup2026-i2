#include "main.h"

using namespace std;
#define ll long long

bool isActuated = false;

void setupMotors() {
    // MOTOR 1
    pinMode(M1_IN_A, OUTPUT);
    pinMode(M1_IN_B, OUTPUT);
    pinMode(M1_PWM, OUTPUT);
    analogWriteFrequency(M1_PWM, 146484);

    // MOTOR 2
    pinMode(M2_IN_A, OUTPUT);
    pinMode(M2_IN_B, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    analogWriteFrequency(M2_PWM, 146484);

    // MOTOR 3
    pinMode(M3_IN_A, OUTPUT);
    pinMode(M3_IN_B, OUTPUT);
    pinMode(M3_PWM, OUTPUT);
    analogWriteFrequency(M3_PWM, 146484);

    // MOTOR 4
    pinMode(M4_IN_A, OUTPUT);
    pinMode(M4_IN_B, OUTPUT);
    pinMode(M4_PWM, OUTPUT);
    analogWriteFrequency(M4_PWM, 146484);

    Serial.println("Motors pinModes defined (by right...)");
}

void setupSol() {
    pinMode(SOL, OUTPUT);
    digitalWrite(SOL, LOW);
    Serial.println("Sol pinMode defined (by right...)");
}

void setupLightgate() {
    pinMode(LIGHTGATE, INPUT);
    Serial.println("Lightgate pinMode defined (by right...)");
}

void setup() {
    Serial.begin(115200);

    // Setup pinModes
    setupMotors();
    setupSol();
    setupLightgate();

    // Set all motors forward
    digitalWrite(M1_IN_A, HIGH); digitalWrite(M1_IN_B, LOW);
    digitalWrite(M2_IN_A, HIGH); digitalWrite(M2_IN_B, LOW);
    digitalWrite(M3_IN_A, HIGH); digitalWrite(M3_IN_B, LOW);
    digitalWrite(M4_IN_A, HIGH); digitalWrite(M4_IN_B, LOW);
}

void loop() {

    // isActuated = !isActuated;
    // Serial.println(isActuated);
    // digitalWrite(SOL, isActuated);

    // Accelerate
    // for (int speed = 0; speed <= 76; speed++) {
    //     analogWrite(M1_PWM, speed);
    //     analogWrite(M2_PWM, speed);
    //     analogWrite(M3_PWM, speed);
    //     analogWrite(M4_PWM, speed);

    //     delay(10);
    // }

    // delay(1000);

    // // Decelerate
    // for (int speed = 76; speed >= 0; speed--) {
    //     analogWrite(M1_PWM, speed);
    //     analogWrite(M2_PWM, speed);
    //     analogWrite(M3_PWM, speed);
    //     analogWrite(M4_PWM, speed);

    //     delay(10);
    // }



    // delay(2000);

    Serial.println(analogRead(LIGHTGATE));
}