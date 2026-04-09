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
static const float LOC_SPEED_MIN = 0.1f;
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

    static const float FACE_KP        = 0.0001f;
    static const float FACE_KD        = 0.0008f;
    static const float FACE_DEADZONE_FAR  = 10.0f;   // deadzone when far from goal
    static const float FACE_DEADZONE_NEAR = 8.0f;   // deadzone when close to goal
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

    // ── Scoring ──────────────────────────────────────────────────────
    if (checkCatchment() && goalVisible && goalDist < SCORE_DIST_THRESH) {
        kickSol();
        Serial.printf("[KICK] goalDist=%.1f\n", goalDist);
    }

    // ── IR orbit ──────────────────────────────────────────────────────
    if (l3BallDetected) {
        // Normalise ball angle to -180..+180
        float ballAngle = l3BallAngle;
        if (ballAngle > 180.0f) ballAngle -= 360.0f;

        static unsigned long lastCamBallMs = 0;
        if (camBallDetected) lastCamBallMs = millis();
        bool closeEnough = (millis() - lastCamBallMs < 2000);
        float moveAngle, speed;

        if (!closeEnough) {
            // Far: chase straight at ball
            moveAngle = ballAngle;
            speed = IR_CHASE_SPEED;
            Serial.printf("[IR CHASE] ball=%.1f\n", l3BallAngle);
        } else {
            // Close: orbit around ball
            float absBall = fabsf(ballAngle);
            float offsetScale = fminf(absBall / IR_ORBIT_FADE_DEG, 1.0f);
            float sign = (ballAngle > 0) ? 1.0f : -1.0f;
            moveAngle = ballAngle + sign * IR_ORBIT_OFFSET * offsetScale;
            moveAngle = constrain(moveAngle, -180.0f, 180.0f);
            speed = IR_ORBIT_SPEED_MIN
                  + (IR_ORBIT_SPEED_MAX - IR_ORBIT_SPEED_MIN) * (1.0f - offsetScale);
        }

        // Cap total output so translation + rotation never exceeds max
        float capSpeed = closeEnough ? IR_ORBIT_SPEED_MAX : IR_CHASE_SPEED;
        float total = speed + fabsf(omega);
        if (total > capSpeed) {
            float scale = capSpeed / total;
            speed *= scale;
            omega *= scale;
        }

        // Nudge away from X-drive dead angles (±45°, ±135°) so all 4 motors engage
        float wrapped = fmodf(moveAngle + 360.0f, 90.0f);  // distance into each 90° quadrant
        float distToDead = fabsf(wrapped - 45.0f);          // distance from the 45° dead spot
        if (distToDead < XDRIVE_DEAD_NUDGE) {
            float nudge = XDRIVE_DEAD_NUDGE - distToDead;
            moveAngle += (wrapped < 45.0f) ? -nudge : nudge;
        }

        // Line avoidance: drive toward field centre using localisation
        if (l1LineDetected) {
            if (locValid) {
                float towardCentreWorld = atan2f(-locX, -locY) * RAD_TO_DEG;
                moveAngle = towardCentreWorld - locHeading;
                if (moveAngle >  180.0f) moveAngle -= 360.0f;
                if (moveAngle < -180.0f) moveAngle += 360.0f;
                speed = LINE_PUSH_SPEED;
                Serial.printf("[LINE] loc x=%.1f y=%.1f move=%.1f\n", locX, locY, moveAngle);
            } else {
                // No localisation — fall back to L1 line angle rejection
                float lineRad = l1Angle * DEG_TO_RAD;
                float moveRad = moveAngle * DEG_TO_RAD;
                float mvx = speed * sinf(moveRad);
                float mvy = speed * cosf(moveRad);
                float lx  = sinf(lineRad);
                float ly  = cosf(lineRad);
                float dot = mvx * lx + mvy * ly;
                if (dot > 0) { mvx -= dot * lx; mvy -= dot * ly; }
                mvx -= LINE_PUSH_SPEED * lx;
                mvy -= LINE_PUSH_SPEED * ly;
                speed = sqrtf(mvx * mvx + mvy * mvy);
                moveAngle = atan2f(mvx, mvy) * RAD_TO_DEG;
                Serial.printf("[LINE] no loc, l1 fallback angle=%.1f\n", l1Angle);
            }
        }

        moveRobot(moveAngle, speed, omega);
        if (closeEnough)
            Serial.printf("[IR ORBIT] ball=%.1f move=%.1f spd=%.2f omega=%.2f count=%d\n",
                          l3BallAngle, moveAngle, speed, omega, l3BallCount);
    } else if (locValid) {
        // No ball: drive to centre using localisation
        float dist = sqrtf(locX * locX + locY * locY);
        if (dist > LOC_DEADZONE) {
            float towardCentreWorld = atan2f(-locX, -locY) * RAD_TO_DEG;
            float moveAngle = towardCentreWorld - locHeading;
            if (moveAngle >  180.0f) moveAngle -= 360.0f;
            if (moveAngle < -180.0f) moveAngle += 360.0f;
            float speed = constrain(dist * LOC_KP, LOC_SPEED_MIN, LOC_SPEED_MAX);
            moveRobot(moveAngle, speed, omega);
            Serial.printf("[RTC] x=%.1f y=%.1f dist=%.1f move=%.1f spd=%.2f\n",
                          locX, locY, dist, moveAngle, speed);
        } else {
            moveRobot(0, 0, omega);
            Serial.printf("[RTC] at centre, dist=%.1f\n", dist);
        }
    } else {
        moveRobot(0, 0, omega);
        Serial.printf("[IDLE] goal=%d err=%.1f omega=%.2f\n", goalVisible, err, omega);
    }
}
