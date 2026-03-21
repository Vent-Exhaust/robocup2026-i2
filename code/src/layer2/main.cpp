#include "main.h"

void setup() {
    setupESC();
    Serial.begin(115200);
    setupL1Comm();
    setupCamComm();

    setupMotors();
    setupSol();
    setupLightgate();
    setupIMU();

    delay(3000);
    resetYawTarget();
}

void loop() {
    dribbler.writeMicroseconds(1500);

    // if (readL1()) debugL1Readings();
    if (readCam()) debugCamReadings();

    readIMU();
    Serial.println(imuYaw);

    // if (!isImuHealthy()) {
    //     Serial.println("[IMU] Timeout — stopping motors and reinitialising");
    //     stopMotors();

    //     while (!reinitIMU()) {
    //         delay(500);
    //     }

    //     delay(IMU_REINIT_SETTLE_MS);
    //     resetYawTarget();
    //     Serial.println("[IMU] Recovered — resuming");
    //     return;
    // }

    // // moveRobot(0, 0, 0);

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
