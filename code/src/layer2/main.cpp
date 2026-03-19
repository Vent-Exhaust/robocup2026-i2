#include "main.h"

using namespace std;
#define ll long long

bool isActuated = false;

// IMU
Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;
float imuYaw = 0;
float imuAccelX = 0, imuAccelY = 0, imuAccelZ = 0;

// IMU correction state
static float targetYaw    = 0;
static bool  targetYawSet = false;
static PID   yawPID       = { YAW_KP, YAW_KI, YAW_KD, YAW_I_MAX };

SerialComm l1Comm(L1_TO_L2_SERIAL);
bool l1LineDetected = false;
float l1Angle = 0;
float l1Size = 0;

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

static float quaternionToYaw(float r, float i, float j, float k) {
    float sinYaw = 2.0f * (r * k + i * j);
    float cosYaw = 1.0f - 2.0f * (j * j + k * k);
    return atan2(sinYaw, cosYaw) * (180.0f / M_PI);
}

void setupIMU() {
    Wire.setSCL(SCL_IMU);
    Wire.setSDA(SDA_IMU);
    Wire.begin();
    Wire.setClock(400000);

    if (!bno.begin_I2C()) {
        Serial.println("[IMU] Failed to find BNO085 — check wiring");
        while (1) delay(10);
    }

    bno.enableReport(SH2_ARVR_STABILIZED_RV, 10000);   // 100Hz yaw
    bno.enableReport(SH2_LINEAR_ACCELERATION, 10000);   // 100Hz linear accel
    Serial.println("[IMU] BNO085 ready");
}

void readIMU() {
    while (bno.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
            imuYaw = quaternionToYaw(
                sensorValue.un.arvrStabilizedRV.real,
                sensorValue.un.arvrStabilizedRV.i,
                sensorValue.un.arvrStabilizedRV.j,
                sensorValue.un.arvrStabilizedRV.k
            );
        } else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
            imuAccelX = sensorValue.un.linearAcceleration.x;
            imuAccelY = sensorValue.un.linearAcceleration.y;
            imuAccelZ = sensorValue.un.linearAcceleration.z;
        }
    }
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
// omega: rotational rate (-1.0 to 1.0, positive = CCW)
void moveRobot(double angleDeg, double speed, double omega) {
    double angleRad = angleDeg * DEG_TO_RAD;
    double vx = speed * cos(angleRad);  // rightward
    double vy = speed * sin(angleRad);  // forward

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
        double yawCorr = constrain(yawPID.compute(err), -YAW_CORRECTION_MAX, YAW_CORRECTION_MAX);
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
    double maxMag = max({abs(mFR), abs(mBR), abs(mBL), abs(mFL), 1.0});
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

void setup() {
    setupESC();
    Serial.begin(115200);
    L1_TO_L2_SERIAL.begin(115200);

    // Setup pinModes
    setupMotors();
    setupSol();
    setupLightgate();
    setupIMU();

    delay(3000);
    resetYawTarget();

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

    readIMU();
    moveRobot(0, 0, 0);

    // // Square pattern: forward -> right -> backward -> left
    // static const double   SQUARE_ANGLES[4] = { 0, 90, 180, 270 };
    // static const uint32_t SIDE_MS    = 2000;  // ms per side (cruise + ramps)
    // static const uint32_t RAMP_MS    = 400;   // ms to accel / decel
    // static const double   MAX_SPEED  = 0.2;
    // static uint8_t        squareState  = 0;
    // static uint32_t       stateStartMs = 0;

    // uint32_t now = millis();
    // if (stateStartMs == 0) stateStartMs = now;

    // uint32_t elapsed = now - stateStartMs;
    // if (elapsed >= SIDE_MS) {
    //     squareState  = (squareState + 1) % 4;
    //     stateStartMs = now;
    //     elapsed      = 0;
    //     resetYawTarget();
    // }

    // // Ramp up at start, ramp down at end
    // double speed;
    // if (elapsed < RAMP_MS) {
    //     speed = MAX_SPEED * (double)elapsed / RAMP_MS;
    // } else if (elapsed > SIDE_MS - RAMP_MS) {
    //     speed = MAX_SPEED * (double)(SIDE_MS - elapsed) / RAMP_MS;
    // } else {
    //     speed = MAX_SPEED;
    // }

    // moveRobot(SQUARE_ANGLES[squareState], speed, 0);
}