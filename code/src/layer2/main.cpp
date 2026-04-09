#include "main.h"

// ── Localisation state ──────────────────────────────────────────────────────
static const float GOAL_SEP     = 194.0f;
static const float HALF_FIELD   = 97.0f;
static float imuOffset          = 0;
static bool  imuOffsetValid     = false;
float locX = 0, locY = 0, locHeading = 0;
bool  locValid = false;

// Return-to-centre tuning
static const float LOC_KP        = 0.003f;
static const float LOC_SPEED_MIN = 0.08f * LOCATION_SPEED_MULT;
static const float LOC_SPEED_MAX = 0.3f;
static const float LOC_DEADZONE  = 3.0f;

// Field boundary speed cap
static const float FIELD_HALF_Y     = 109.5f;
static const float FIELD_HALF_X     = 79.0f;
static const float EDGE_MARGIN      = 20.0f;
static const float EDGE_SPEED_CAP   = 0.15f;

// ── Localisation ────────────────────────────────────────────────────────────

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

static float edgeSpeedCap(float moveAngleDeg, float headingDeg) {
    if (!locValid) return LOC_SPEED_MAX;

    float worldMoveDeg = headingDeg + moveAngleDeg;
    float worldMoveRad = worldMoveDeg * DEG_TO_RAD;
    float moveDirX = sinf(worldMoveRad);
    float moveDirY = cosf(worldMoveRad);

    float distToEdgeXp = FIELD_HALF_X - locX;
    float distToEdgeXn = FIELD_HALF_X + locX;
    float distToEdgeYp = FIELD_HALF_Y - locY;
    float distToEdgeYn = FIELD_HALF_Y + locY;

    bool nearEdge = false;
    if (distToEdgeXp < EDGE_MARGIN && moveDirX > 0) nearEdge = true;
    if (distToEdgeXn < EDGE_MARGIN && moveDirX < 0) nearEdge = true;
    if (distToEdgeYp < EDGE_MARGIN && moveDirY > 0) nearEdge = true;
    if (distToEdgeYn < EDGE_MARGIN && moveDirY < 0) nearEdge = true;

    return nearEdge ? EDGE_SPEED_CAP : LOC_SPEED_MAX;
}

// ── Main ────────────────────────────────────────────────────────────────────

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
    // ── Read sensors ────────────────────────────────────────────────────
    readL1();
    readL3();
    readCam();
    readIMU();
    updateLocalisation();

    // ── Face goal ────────────────────────────────────────────────────────
    #if ATTACK_GOAL == 0
    bool  goalVisible = camBlueDetected;
    float goalAngle   = camBlueAngle;
    #else
    bool  goalVisible = camYellowDetected;
    float goalAngle   = camYellowAngle;
    #endif

    float err;
    if (goalVisible) {
        err = goalAngle;
    } else {
        err = -imuYaw;
    }
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;

    static const float FACE_KP        = 0.005f;
    static const float FACE_KD        = 0.2f;
    static const float FACE_DEADZONE_FAR  = 5.0f;   // deadzone when far from goal
    static const float FACE_DEADZONE_NEAR = 2.0f;   // deadzone when close to goal
    static const float FACE_NEAR_DIST     = 60.0f;  // distance threshold (cm)
    static const float FACE_OMEGA_MIN = 0.12f;
    static const float FACE_OMEGA_MAX = 0.18f;

    #if ATTACK_GOAL == 0
    float goalDist = camBlueDist;
    #else
    float goalDist = camYellowDist;
    #endif

    float deadzone = (goalVisible && goalDist < FACE_NEAR_DIST)
                     ? FACE_DEADZONE_NEAR : FACE_DEADZONE_FAR;

    static float prevErr = 0;

    float omega = 0;
    if (fabsf(err) > deadzone) {
        float derivative = err - prevErr;
        float raw = err * FACE_KP + derivative * FACE_KD;
        float sign = (raw > 0) ? 1.0f : -1.0f;
        omega = sign * constrain(fabsf(raw), FACE_OMEGA_MIN, FACE_OMEGA_MAX);
    }
    prevErr = err;

    moveRobot(0, 0, omega);
    Serial.printf("[FACE] goal=%d err=%.1f omega=%.2f\n", goalVisible, err, omega);
}
