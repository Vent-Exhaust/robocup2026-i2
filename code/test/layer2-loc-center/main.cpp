#include <Arduino.h>
#include "movement.h"
#include "cam_comm.h"
#include "config.h"

// --- Tuning ---
static const float LOC_KP        = 0.008f; // speed per cm of error
static const float LOC_SPEED_MIN = 0.08f;
static const float LOC_SPEED_MAX = 0.3f;
static const float GOAL_SEP      = 194.0f; // cm between the two goals
static const float HALF_FIELD    = 97.0f;
static const float DEADZONE_CM   = 3.0f;   // stop within this radius of centre

// IMU-to-world heading offset, calibrated when both goals are visible
static float imuOffset       = 0;
static bool  imuOffsetValid  = false;

// Compute position from a single goal + IMU heading
// goalWorldY: -97 for blue, +97 for yellow
static void locFromOneGoal(float dist, float angle, float goalWorldY,
                           float headingDeg, float& posX, float& posY) {
    float a  = angle * DEG_TO_RAD;
    float rx = dist * sinf(a);   // goal in robot frame (x = right)
    float ry = dist * cosf(a);   // goal in robot frame (y = forward)

    float th = headingDeg * DEG_TO_RAD;
    float cosT = cosf(th), sinT = sinf(th);

    // world = robotPos + R * robotFrame  →  robotPos = goalWorld - R * robotFrame
    posX = -(rx * cosT + ry * sinT);
    posY = goalWorldY + rx * sinT - ry * cosT;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    setupMotors();
    setupIMU();
    setupCamComm();

    delay(500);
    resetYawTarget();

    Serial.println("=== Localisation — hold centre ===");
}

void loop() {
    readCam();
    readIMU();

    float posX, posY, headingDeg;
    bool locValid = false;
    const char* locSrc = "";

    if (camBlueDetected && camYellowDetected) {
        // --- Two-goal localisation (heading from geometry) ---
        float a1 = camBlueAngle   * DEG_TO_RAD;
        float a2 = camYellowAngle * DEG_TO_RAD;

        float bx = camBlueDist   * sinf(a1);
        float by = camBlueDist   * cosf(a1);
        float gx = camYellowDist * sinf(a2);
        float gy = camYellowDist * cosf(a2);

        float dx = gx - bx;
        float dy = gy - by;
        float r  = sqrtf(dx * dx + dy * dy);

        if (r > 1.0f) {
            float scale = GOAL_SEP / r;
            headingDeg = atan2f(-dx, dy) * RAD_TO_DEG;
            posX = (by * gx - bx * gy) / r * scale;
            posY = (-(bx * dx + by * dy) / r - r / 2.0f) * scale;

            // Calibrate IMU offset while we have both goals
            imuOffset = headingDeg - imuYaw;
            imuOffsetValid = true;

            locValid = true;
            locSrc = "2G";
        }
    } else if (imuOffsetValid && (camBlueDetected || camYellowDetected)) {
        // --- Single-goal localisation (heading from IMU) ---
        headingDeg = imuYaw + imuOffset;

        if (camBlueDetected) {
            locFromOneGoal(camBlueDist, camBlueAngle, -HALF_FIELD,
                           headingDeg, posX, posY);
            locSrc = "1G-B";
        } else {
            locFromOneGoal(camYellowDist, camYellowAngle, HALF_FIELD,
                           headingDeg, posX, posY);
            locSrc = "1G-Y";
        }
        locValid = true;
    }

    if (!locValid) {
        stopMotors();
        resetYawTarget();
        Serial.println("[LOC] no fix");
        return;
    }

    // --- Drive toward centre (0, 0) ---
    float distToCenter = sqrtf(posX * posX + posY * posY);

    if (distToCenter < DEADZONE_CM) {
        stopMotors();
        resetYawTarget();
        Serial.printf("[LOC:%s] at centre  x=%.1f y=%.1f dist=%.1f\n",
                      locSrc, posX, posY, distToCenter);
        return;
    }

    // World-frame angle from robot to centre (clockwise from north)
    float worldAngleDeg = atan2f(-posX, -posY) * RAD_TO_DEG;

    // Convert to robot-frame movement angle for moveRobot()
    float moveAngle = worldAngleDeg - headingDeg;
    moveAngle = fmodf(moveAngle + 360.0f, 360.0f);

    float speed = constrain(distToCenter * LOC_KP, LOC_SPEED_MIN, LOC_SPEED_MAX);

    moveRobot(moveAngle, speed, 0);

    Serial.printf("[LOC:%s] x=%.1f y=%.1f dist=%.1f move=%.1f spd=%.2f hdg=%.1f\n",
                  locSrc, posX, posY, distToCenter, moveAngle, speed, headingDeg);
}
