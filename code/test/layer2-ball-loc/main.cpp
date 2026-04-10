#include <Arduino.h>
#include "movement.h"
#include "cam_comm.h"
#include "config.h"

// --- Localisation constants ---
static const float GOAL_SEP   = 194.0f;
static const float HALF_FIELD = 97.0f;

// IMU-to-world heading offset, calibrated when both goals are visible
static float imuOffset      = 0;
static bool  imuOffsetValid = false;

// Robot pose
static float posX = 0, posY = 0, headingDeg = 0;
static bool  locValid = false;
static const char* locSrc = "";

// Ball field position
static float ballFieldX = 0, ballFieldY = 0;
static bool  ballLocValid = false;

// Compute position from a single goal + IMU heading
static void locFromOneGoal(float dist, float angle, float goalWorldY,
                           float heading, float& px, float& py) {
    float a  = angle * DEG_TO_RAD;
    float rx = dist * sinf(a);
    float ry = dist * cosf(a);
    float th = heading * DEG_TO_RAD;
    float cosT = cosf(th), sinT = sinf(th);
    px = -(rx * cosT + ry * sinT);
    py = goalWorldY + rx * sinT - ry * cosT;
}

static void updateLocalisation() {
    locValid = false;

    if (camBlueDetected && camYellowDetected) {
        float a1 = camBlueAngle   * DEG_TO_RAD;
        float a2 = camYellowAngle * DEG_TO_RAD;
        float bx = camBlueDist   * sinf(a1);
        float by = camBlueDist   * cosf(a1);
        float gx = camYellowDist * sinf(a2);
        float gy = camYellowDist * cosf(a2);
        float dx = gx - bx, dy = gy - by;
        float r  = sqrtf(dx * dx + dy * dy);
        if (r < 1.0f) return;

        float scale = GOAL_SEP / r;
        headingDeg = atan2f(-dx, dy) * RAD_TO_DEG;
        posX = (by * gx - bx * gy) / r * scale;
        posY = (-(bx * dx + by * dy) / r - r / 2.0f) * scale;

        imuOffset = headingDeg - imuYaw;
        imuOffsetValid = true;
        locValid = true;
        locSrc = "2G";
    } else if (imuOffsetValid && (camBlueDetected || camYellowDetected)) {
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
}

static void updateBallLoc() {
    ballLocValid = false;
    if (!locValid || !camBallDetected) return;

    float headRad = headingDeg * DEG_TO_RAD;
    float ballRad = camBallAngle * DEG_TO_RAD;

    // Ball in robot frame (0 deg = forward, CW positive)
    float relX = camBallDist * sinf(ballRad);
    float relY = camBallDist * cosf(ballRad);

    // Rotate into field frame and add robot position
    float cosH = cosf(headRad), sinH = sinf(headRad);
    ballFieldX = posX + relX * cosH - relY * sinH;
    ballFieldY = posY + relX * sinH + relY * cosH;
    ballLocValid = true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    setupIMU();
    setupCamComm();

    delay(500);
    Serial.println("=== Ball Localisation Debug ===");
}

void loop() {
    readCam();
    readIMU();

    updateLocalisation();
    updateBallLoc();

    if (!locValid) {
        Serial.println("[LOC] no fix");
        delay(100);
        return;
    }

    Serial.printf("[LOC:%s] robot x=%.1f y=%.1f hdg=%.1f", locSrc, posX, posY, headingDeg);

    if (ballLocValid) {
        Serial.printf("  |  [BALL] field x=%.1f y=%.1f  (cam dist=%.1f ang=%.1f)\n",
                      ballFieldX, ballFieldY, camBallDist, camBallAngle);
    } else {
        Serial.println("  |  [BALL] not visible");
    }

    delay(LOOP_DELAY_MS);
}
