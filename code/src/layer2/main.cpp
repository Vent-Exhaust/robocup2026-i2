#include "main.h"

void setup() {
    setupESC();
    Serial.begin(115200);
    setupL1Comm();
    setupCamComm();
    setupL3Comm();

    setupMotors();
    setupSol();
    setupLightgate();
    setupIMU();

    delay(500);
    resetYawTarget();
}

void loop() {
    readL3();
    readCam();
    readIMU();

    // moveRobot(0, 0.3, 0);

    // // Determine ball angle — prefer IR (means ball is close)
    bool ballFound = false;
    float ballAngle = 0;

    if (l3BallAngle) {
        ballFound = true;
        ballAngle = l3BallAngle;
        Serial.println(l3BallAngle);
    } else if (camBallDetected) {
        ballFound = true;
        ballAngle = camBallAngle;
    }

    // Serial.println(l3BallAngle);
    // moveRobot(l3BallAngle, 0.3, 0);

    if (ballFound) {
        // Rotation error: how far ball is from front (0°), in [-180, 180]
        float rotError = ballAngle;
        if (rotError > 180.0f) rotError -= 360.0f;

        // P-control rotation to face the ball
        // Positive rotError (ball on right) → negative omega (CW)
        static const float ROT_KP = 0.001;
        float omega = rotError * ROT_KP;
        omega = constrain(omega, -0.5f, 0.5f);

        // If nearly facing ball, let heading-hold PID take over
        if (abs(rotError) < 5.0f) {
            omega = 0;
        }

        // Move toward ball while rotating to face it
        static const float MOVE_SPEED = 0.3f;
        moveRobot(ballAngle, MOVE_SPEED, omega);

        Serial.printf("[TRACK] angle=%.1f rotErr=%.1f omega=%.3f src=%s\n",
                      ballAngle, rotError, omega,
                      l3BallDetected ? "IR" : "CAM");
    } else {
        stopMotors();
        resetYawTarget();
    }
}
