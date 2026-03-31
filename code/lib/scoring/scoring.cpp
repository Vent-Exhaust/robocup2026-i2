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

void spinDribbler(int speed) {
    int pulse = map(speed, 0, 100, 1000, 2000);
    dribbler.writeMicroseconds(pulse);
}

bool checkCatchment() {
  int count = 0;
  for (int i = 0; i < 20; i++) {
    count += analogRead(LIGHTGATE) < 1021 ? 1 : 0;
    delayMicroseconds(5);
  }
  // Serial.println(count);
  return count > 2;
}