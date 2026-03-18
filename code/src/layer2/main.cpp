#include "main.h"

using namespace std;
#define ll long long

bool isActuated = false;

SerialComm l1Comm(L1_TO_L2_SERIAL);
bool l1LineDetected = false;
float l1Angle = 0;
float l1Size = 0;

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

void setupESC() {
    dribbler.attach(DRIBBLER_PWM, 1000, 2000);  // min/max pulse width

    // Arm ESC
    dribbler.writeMicroseconds(1000);  // stop signal
    delay(3000);      
}

void debugL1Readings() {
    if (l1LineDetected) {
        Serial.print("[L1] Line | Angle: ");
        Serial.print(l1Angle, 1);
        Serial.print(" deg | Size: ");
        Serial.println(l1Size, 1);
    } else {
        Serial.println("[L1] No line");
    }
}

void setup() {
    setupESC();
    Serial.begin(115200);
    L1_TO_L2_SERIAL.begin(115200);

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
    int16_t rx[3];
    int count = l1Comm.read(rx, 3);
    if (count == 3) {
        l1LineDetected = rx[0];
        l1Angle = rx[1] / 10.0f;
        l1Size  = rx[2] / 10.0f;
        debugL1Readings();
    }

    // digitalWrite(SOL, LOW);
    // delay(3000);
    // digitalWrite(SOL, HIGH);
    // delay(100);

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

    // dribbler.writeMicroseconds(1700);

    // delay(5000);

    // // stop
    // dribbler.writeMicroseconds(1000);
    // delay(3000);


    // delay(2000);

    // Serial.println(analogRead(LIGHTGATE));
}