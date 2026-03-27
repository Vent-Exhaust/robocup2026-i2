#include "movement.h"

// IMU objects
Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;

// IMU state
float imuYaw = 0;
float imuAccelX = 0, imuAccelY = 0, imuAccelZ = 0;

// IMU health tracking
static unsigned long lastImuReadMs = 0;

// Yaw heading-hold state
static float targetYaw    = 0;
static bool  targetYawSet = false;
static PID   yawPID       = { YAW_KP, YAW_KI, YAW_KD, YAW_I_MAX };

void setupMotors() {
    // Front Right (M1)
    pinMode(FR_IN_A, OUTPUT);
    pinMode(FR_IN_B, OUTPUT);
    pinMode(FR_PWM, OUTPUT);
    analogWriteFrequency(FR_PWM, 146484);

    // Back Right (M2)
    pinMode(BR_IN_A, OUTPUT);
    pinMode(BR_IN_B, OUTPUT);
    pinMode(BR_PWM, OUTPUT);
    analogWriteFrequency(BR_PWM, 146484);

    // Back Left (M3)
    pinMode(BL_IN_A, OUTPUT);
    pinMode(BL_IN_B, OUTPUT);
    pinMode(BL_PWM, OUTPUT);
    analogWriteFrequency(BL_PWM, 146484);

    // Front Left (M4)
    pinMode(FL_IN_A, OUTPUT);
    pinMode(FL_IN_B, OUTPUT);
    pinMode(FL_PWM, OUTPUT);
    analogWriteFrequency(FL_PWM, 146484);

    Serial.println("Motors pinModes defined (by right...)");
}

static float quaternionToYaw(float r, float i, float j, float k) {
    float sinYaw = 2.0f * (r * k + i * j);
    float cosYaw = 1.0f - 2.0f * (j * j + k * k);
    return atan2(sinYaw, cosYaw) * (180.0f / M_PI);
}

void setupIMU() {
    Wire.begin();
    Wire.setClock(1000000);

    if (!bno.begin_I2C()) {
        Serial.println("[IMU] Failed to find BNO085 — check wiring");
        while (1) delay(10);
    }

    bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);  // 100Hz yaw (no magnetometer)
    bno.enableReport(SH2_LINEAR_ACCELERATION, 10000);    // 100Hz linear accel
    Serial.println("[IMU] BNO085 ready");
}

void readIMU() {
    static float accelXFilt = 0;
    static float accelYFilt = 0;
    const float ACCEL_ALPHA = 0.1f;

    bool gotData = false;
    int maxReads = 10;  // don't drain forever if bus is spamming
    while (maxReads-- > 0 && bno.getSensorEvent(&sensorValue)) {
        gotData = true;
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
            imuYaw = quaternionToYaw(
                sensorValue.un.gameRotationVector.real,
                sensorValue.un.gameRotationVector.i,
                sensorValue.un.gameRotationVector.j,
                sensorValue.un.gameRotationVector.k
            );
        } else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
            accelXFilt = ACCEL_ALPHA * sensorValue.un.linearAcceleration.x + (1.0f - ACCEL_ALPHA) * accelXFilt;
            accelYFilt = ACCEL_ALPHA * sensorValue.un.linearAcceleration.y + (1.0f - ACCEL_ALPHA) * accelYFilt;
            imuAccelX = accelXFilt;
            imuAccelY = accelYFilt;
            imuAccelZ = sensorValue.un.linearAcceleration.z;
        }
    }
    if (gotData) lastImuReadMs = millis();
}

// void readIMU() {
//     bool gotData = false;
//     while (bno.getSensorEvent(&sensorValue)) {
//         gotData = true;
//         if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
//             imuYaw = quaternionToYaw(
//                 sensorValue.un.rotationVector.real,
//                 sensorValue.un.rotationVector.i,
//                 sensorValue.un.rotationVector.j,
//                 sensorValue.un.rotationVector.k
//             );
//         } else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
//             imuAccelX = sensorValue.un.linearAcceleration.x;
//             imuAccelY = sensorValue.un.linearAcceleration.y;
//             imuAccelZ = sensorValue.un.linearAcceleration.z;
//         }
//     }
//     if (gotData) lastImuReadMs = millis();
// }

bool isImuHealthy() {
    if (lastImuReadMs == 0) return true;  // haven't started yet
    return (millis() - lastImuReadMs) < IMU_TIMEOUT_MS;
}

bool reinitIMU() {
    Serial.println("[IMU] Attempting reinit...");

    // End I2C bus
    Wire.end();
    delay(10);

    // Bit-bang SCL to unstick a hung I2C slave holding SDA low
    pinMode(SCL_IMU, OUTPUT);
    pinMode(SDA_IMU, INPUT);
    for (int i = 0; i < 16; i++) {
        digitalWrite(SCL_IMU, HIGH);
        delayMicroseconds(5);
        digitalWrite(SCL_IMU, LOW);
        delayMicroseconds(5);
    }
    pinMode(SCL_IMU, INPUT); // release

    delay(100);

    // Restart I2C
    Wire.begin();
    Wire.setClock(1000000);
    delay(100);

    if (!bno.begin_I2C()) {
        Serial.println("[IMU] Reinit failed — could not find BNO085");
        return false;
    }

    bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
    bno.enableReport(SH2_LINEAR_ACCELERATION, 10000);

    lastImuReadMs = millis();
    Serial.println("[IMU] Reinit successful");
    return true;
}

void stopMotors() {
    setMotor(FR_IN_A, FR_IN_B, FR_PWM, 0);
    setMotor(BR_IN_A, BR_IN_B, BR_PWM, 0);
    setMotor(BL_IN_A, BL_IN_B, BL_PWM, 0);
    setMotor(FL_IN_A, FL_IN_B, FL_PWM, 0);
}

void debugIMU() {
    Serial.print("[IMU] Yaw: ");
    Serial.print(imuYaw, 2);
    Serial.print(" deg | Accel X: ");
    Serial.print(imuAccelX, 3);
    Serial.print(" Y: ");
    Serial.print(imuAccelY, 3);
    Serial.print(" Z: ");
    Serial.print(imuAccelZ, 3);
    Serial.println(" m/s2");
}

void setMotor(int inA, int inB, int pwmPin, double power) {
    bool forward = (power >= 0);
    int pwmVal = (int)(abs(power) * 255);
    pwmVal = min(pwmVal, 255);
    digitalWrite(inA, forward ? HIGH : LOW);
    digitalWrite(inB, forward ? LOW : HIGH);
    analogWrite(pwmPin, pwmVal);
}

// Call this to re-lock the target heading (e.g. after an intentional turn)
void resetYawTarget() {
    targetYawSet = false;
    yawPID.reset();
}

// angleDeg: movement direction (0° = forward, 90° = right, 180° = backward, 270° = left)
// speed: 0.0 to 1.0
// omega: rotational rate (-1.0 to 1.0, positive = CW)
void moveRobot(double angleDeg, double speed, double omega) {
    omega = -omega;
    double angleRad = angleDeg * DEG_TO_RAD;
    double vx =  speed * cos(angleRad);  // rightward
    double vy = -speed * sin(angleRad);  // forward

    // --- Yaw heading-hold PID ---
    if (omega != 0.0) {
        // Intentionally turning: track current yaw so we hold the new heading after
        targetYaw = imuYaw;
        yawPID.reset();
    } else {
        if (!targetYawSet) {
            targetYaw    = imuYaw;
            targetYawSet = true;
        }
        float err = targetYaw - imuYaw;
        // Wrap error to [-180, 180]
        while (err >  180.0f) err -= 360.0f;
        while (err < -180.0f) err += 360.0f;
        double yawCorr = constrain(yawPID.compute(err, imuYaw), -YAW_CORRECTION_MAX, YAW_CORRECTION_MAX);
        omega += yawCorr;
    }

    // --- Accel drift correction (P) ---
    // Counteract unexpected acceleration in robot frame
    vx += constrain(-imuAccelX * ACCEL_KP, -ACCEL_CORRECTION_MAX, ACCEL_CORRECTION_MAX);
    vy += constrain(-imuAccelY * ACCEL_KP, -ACCEL_CORRECTION_MAX, ACCEL_CORRECTION_MAX);

    // X-drive: motors at 45°, 135°, 225°, 315°
    double mFR =  vy + vx + omega;  // 45°  (M1)
    double mBR =  vy - vx - omega;  // 135° (M2)
    double mBL =  vy + vx - omega;  // 225° (M3)
    double mFL =  vy - vx + omega;  // 315° (M4)

    // Normalize so no value exceeds 1.0
    double maxMag = max(max(max(max(abs(mFR), abs(mBR)), abs(mBL)), abs(mFL)), 1.0);
    mFR /= maxMag;
    mBR /= maxMag;
    mBL /= maxMag;
    mFL /= maxMag;

    // Invert mFR and mFL to fix physical mounting direction
    mFR = -mFR;
    mFL = -mFL;

    // Apply trim
    mFR *= FR_TRIM;
    mBR *= BR_TRIM;
    mBL *= BL_TRIM;
    mFL *= FL_TRIM;

    setMotor(FR_IN_A, FR_IN_B, FR_PWM, mFR);
    setMotor(BR_IN_A, BR_IN_B, BR_PWM, mBR);
    setMotor(BL_IN_A, BL_IN_B, BL_PWM, mBL);
    setMotor(FL_IN_A, FL_IN_B, FL_PWM, mFL);
}
