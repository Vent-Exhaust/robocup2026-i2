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

}

void setMotor(int inA, int inB, int pwmPin, double power) {
    bool forward = (power >= 0);
    int pwmVal = (int)(abs(power) * 255);
    pwmVal = min(pwmVal, 255);
    digitalWrite(inA, forward ? HIGH : LOW);
    digitalWrite(inB, forward ? LOW : HIGH);
    analogWrite(pwmPin, pwmVal);
}

// angleDeg: movement direction (0° = right, 90° = forward), math convention
// speed: 0.0 to 1.0
// omega: rotational rate (-1.0 to 1.0, positive = CCW)
void moveRobot(double angleDeg, double speed, double omega) {
    double angleRad = angleDeg * DEG_TO_RAD;
    double vx = speed * cos(angleRad);  // rightward
    double vy = speed * sin(angleRad);  // forward

    // X-drive: motors at 45°, 135°, 225°, 315°
    double m1 =  vy + vx + omega;  // 45°
    double m2 =  vy - vx + omega;  // 135°
    double m3 =  vy + vx - omega;  // 225°
    double m4 =  vy - vx - omega;  // 315°

    // Normalize so no value exceeds 1.0
    double maxMag = max({abs(m1), abs(m2), abs(m3), abs(m4), 1.0});
    m1 /= maxMag;
    m2 /= maxMag;
    m3 /= maxMag;
    m4 /= maxMag;

    // Invert m1 and m4 to fix physical mounting direction
    m1 = -m1;
    m4 = -m4;

    // Apply trim
    m1 *= M1_TRIM;
    m2 *= M2_TRIM;
    m3 *= M3_TRIM;
    m4 *= M4_TRIM;

    setMotor(M1_IN_A, M1_IN_B, M1_PWM, m1);
    setMotor(M2_IN_A, M2_IN_B, M2_PWM, m2);
    setMotor(M3_IN_A, M3_IN_B, M3_PWM, m3);
    setMotor(M4_IN_A, M4_IN_B, M4_PWM, m4);
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

    // Accelerate from 0 to 0.4 over 1s
    for (int i = 0; i <= 40; i++) {
        moveRobot(0, i / 100.0, 0);
        delay(25);
    }

    // Hold at 0.4 for 1s
    delay(1000);

    // Decelerate back to 0 over 1s
    for (int i = 40; i >= 0; i--) {
        moveRobot(0, i / 100.0, 0);
        delay(25);
    }

    moveRobot(0, 0, 0);
    delay(1000);
}