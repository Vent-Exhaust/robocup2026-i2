#include "scoring.h"

Servo dribbler;
bool isActuated = false;

void setupESC() {
    dribbler.attach(DRIBBLER_PWM, 1000, 2000);  // min/max pulse width

    // Arm ESC
    dribbler.writeMicroseconds(1000);  // stop signal
    delay(3000);
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
