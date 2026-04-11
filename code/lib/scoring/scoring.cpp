#include "scoring.h"

Servo dribbler;
bool isActuated = false;
static unsigned long lastKickTime = 0;

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

void kickSol() {
  spinDribbler(0);

  // delay(100);

  digitalWrite(SOL, HIGH);
  delay(50);
  digitalWrite(SOL, LOW);

  lastKickTime = millis();
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
  // Ignore lightgate during cooldown after kick — solenoid triggers false positive
  if (millis() - lastKickTime < KICK_COOLDOWN_MS) return false;

  int count = 0;
  for (int i = 0; i < 20; i++) {
    count += analogRead(LIGHTGATE) < 995 ? 1 : 0;
    delayMicroseconds(5);
  }
  // Serial.println(count);
  return count > 4;
}