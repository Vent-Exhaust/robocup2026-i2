#include "main.h"

// Localisation state
static const float GOAL_SEP     = 194.0f;
static const float HALF_FIELD   = 97.0f;
static float imuOffset          = 0;
static bool  imuOffsetValid     = false;
float locX = 0, locY = 0, locHeading = 0;
bool  locValid = false;

// Return-to-centre tuning
static const float LOC_KP        = 0.008f;
static const float LOC_SPEED_MIN = 0.08f;
static const float LOC_SPEED_MAX = 0.3f;
static const float LOC_DEADZONE  = 3.0f;

// Field boundary speed cap
// Field is 219 x 158 cm → half-widths 109.5 (Y) and 79 (X) from centre
static const float FIELD_HALF_Y     = 109.5f; // along goal axis
static const float FIELD_HALF_X     = 79.0f;  // perpendicular to goals
static const float EDGE_MARGIN      = 20.0f;  // cm from edge where cap kicks in
static const float EDGE_SPEED_CAP   = 0.15f;   // max speed when moving outward near edge

static void locFromOneGoal(float dist, float angle, float goalWorldY,
                           float headingDeg, float& posX, float& posY) {
    float a  = angle * DEG_TO_RAD;
    float rx = dist * sinf(a);
    float ry = dist * cosf(a);
    float th = headingDeg * DEG_TO_RAD;
    float cosT = cosf(th), sinT = sinf(th);
    posX = -(rx * cosT + ry * sinT);
    posY = goalWorldY + rx * sinT - ry * cosT;
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
        locHeading = atan2f(-dx, dy) * RAD_TO_DEG;
        locX = (by * gx - bx * gy) / r * scale;
        locY = (-(bx * dx + by * dy) / r - r / 2.0f) * scale;

        imuOffset = locHeading - imuYaw;
        imuOffsetValid = true;
        locValid = true;
    } else if (imuOffsetValid && (camBlueDetected || camYellowDetected)) {
        locHeading = imuYaw + imuOffset;
        if (camBlueDetected)
            locFromOneGoal(camBlueDist, camBlueAngle, -HALF_FIELD, locHeading, locX, locY);
        else
            locFromOneGoal(camYellowDist, camYellowAngle, HALF_FIELD, locHeading, locX, locY);
        locValid = true;
    }
}

// Returns a speed cap if the robot is near the field edge AND the movement
// direction would take it further out. Returns LOC_SPEED_MAX otherwise.
static float edgeSpeedCap(float moveAngleDeg, float headingDeg) {
    if (!locValid) return LOC_SPEED_MAX;

    // World-frame movement direction
    float worldMoveDeg = headingDeg + moveAngleDeg;
    float worldMoveRad = worldMoveDeg * DEG_TO_RAD;
    float moveDirX = sinf(worldMoveRad);  // world X component of movement
    float moveDirY = cosf(worldMoveRad);  // world Y component of movement

    // Distance from each edge
    float distToEdgeXp = FIELD_HALF_X - locX;  // +X edge
    float distToEdgeXn = FIELD_HALF_X + locX;  // -X edge
    float distToEdgeYp = FIELD_HALF_Y - locY;  // +Y edge
    float distToEdgeYn = FIELD_HALF_Y + locY;  // -Y edge

    bool nearEdge = false;

    // Only cap if moving TOWARD the nearby edge
    if (distToEdgeXp < EDGE_MARGIN && moveDirX > 0) nearEdge = true;
    if (distToEdgeXn < EDGE_MARGIN && moveDirX < 0) nearEdge = true;
    if (distToEdgeYp < EDGE_MARGIN && moveDirY > 0) nearEdge = true;
    if (distToEdgeYn < EDGE_MARGIN && moveDirY < 0) nearEdge = true;

    return nearEdge ? EDGE_SPEED_CAP : LOC_SPEED_MAX;
}

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
    readL1();
    readL3();
    readCam();
    readIMU();

    // Localisation
    updateLocalisation();

    bool ballFound = false;
    float ballAngle = 0;

    if (camBallDetected) {
        ballFound = true;
        ballAngle = camBallAngle;
    } else if (l3BallAngle) {
        ballFound = true;
        ballAngle = l3BallAngle;
    }

    // Line avoidance takes priority over everything
    if (l1LineDetected) {
        float escapeAngle = fmod(l1Angle + 180.0f, 360.0f);
        static const float LINE_KP = 3.0f;
        float escapeSpeed = constrain(l1Size * LINE_KP, 0.1f, 0.7f);
        moveRobot(escapeAngle, escapeSpeed, 0);
    } else if (ballFound) {
        float rotError = ballAngle;
        if (rotError > 180.0f) rotError -= 360.0f;

        static const float ROT_KP = 0.001;
        float omega = rotError * ROT_KP;
        omega = constrain(omega, -0.5f, 0.5f);
        if (abs(rotError) < 5.0f) omega = 0;

        static const float BALL_SPEED_KP  = 0.001f;
        static const float BALL_SPEED_MIN = 0.05f;
        static const float BALL_SPEED_MAX = 0.2f;
        float moveSpeed;
        if (camBallDetected) {
            moveSpeed = constrain(camBallDist * BALL_SPEED_KP, BALL_SPEED_MIN, BALL_SPEED_MAX);
        } else {
            moveSpeed = BALL_SPEED_MAX;
        }

        // Cap speed near field edge if moving outward
        float cap = edgeSpeedCap(ballAngle, locHeading);
        moveSpeed = min(moveSpeed, cap);

        moveRobot(ballAngle, moveSpeed, omega);
    } else if (locValid) {
        // No ball — return to centre
        float distToCenter = sqrtf(locX * locX + locY * locY);

        if (distToCenter < LOC_DEADZONE) {
            stopMotors();
        } else {
            float worldAngleDeg = atan2f(-locX, -locY) * RAD_TO_DEG;
            float moveAngle = worldAngleDeg - locHeading;
            moveAngle = fmodf(moveAngle + 360.0f, 360.0f);

            float speed = constrain(distToCenter * LOC_KP, LOC_SPEED_MIN, LOC_SPEED_MAX);
            moveRobot(moveAngle, speed, 0);
        }
    } else {
        stopMotors();
    }

    if (locValid) {
        Serial.printf("[LOC] x=%.1f y=%.1f heading=%.1f\n", locX, locY, locHeading);
    }
}
