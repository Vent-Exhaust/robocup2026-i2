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

// ── Striker face-goal heading correction ────────────────────────────────────

static const float FACE_KP             = 0.0001f;
static const float FACE_KD             = 0.0008f;
static const float FACE_DEADZONE_FAR   = 10.0f;
static const float FACE_DEADZONE_NEAR  = 8.0f;
static const float FACE_NEAR_DIST      = 60.0f;
static const float FACE_OMEGA_MIN      = 0.12f;
static const float FACE_OMEGA_MAX      = 0.18f;

static float prevFaceErr = 0;

static float computeFaceOmega(bool goalVis, float goalAng, float goalDst) {
    float err;
    if (goalVis) {
        err = goalAng;
    } else {
        err = -imuYaw;
    }
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;

    float deadzone = (goalVis && goalDst < FACE_NEAR_DIST)
                     ? FACE_DEADZONE_NEAR : FACE_DEADZONE_FAR;

    float omega = 0;
    if (fabsf(err) > deadzone) {
        float derivative = err - prevFaceErr;
        float raw = err * FACE_KP + derivative * FACE_KD;
        float sign = (raw > 0) ? 1.0f : -1.0f;
        omega = sign * constrain(fabsf(raw), FACE_OMEGA_MIN, FACE_OMEGA_MAX);
    }
    prevFaceErr = err;
    return omega;
}

// ── Line avoidance helper ──────────────────────────────────────────────────

static void applyLineAvoidance(float& moveAngle, float& speed, float& omega) {
    if (!l1LineDetected) return;
    omega = 0;
    if (locValid) {
        // Back-line override: rear LDR sensors report ~90° instead of 180°,
        // so when localisation shows we're near/behind the goalie line,
        // ignore L1 angle and drive straight forward (toward centre Y).
        #if ATTACK_GOAL == 0
        bool nearBackLine = locY > fabsf(GOALIE_CURVE_Y0);   // defending positive Y side
        float forwardWorld = 180.0f;                          // world heading toward negative Y
        #else
        bool nearBackLine = locY < -fabsf(GOALIE_CURVE_Y0);  // defending negative Y side
        float forwardWorld = 0.0f;                            // world heading toward positive Y
        #endif

        float escapeWorld;
        if (nearBackLine) {
            escapeWorld = forwardWorld;
            Serial.printf("[LINE] back-line override y=%.1f\n", locY);
        } else {
            escapeWorld = atan2f(-locX, -locY) * RAD_TO_DEG;
            Serial.printf("[LINE] loc x=%.1f y=%.1f\n", locX, locY);
        }

        moveAngle = escapeWorld - locHeading;
        if (moveAngle >  180.0f) moveAngle -= 360.0f;
        if (moveAngle < -180.0f) moveAngle += 360.0f;
        speed = LINE_PUSH_SPEED;
        Serial.printf("[LINE] move=%.1f\n", moveAngle);
    } else {
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

// ── Striker ────────────────────────────────────────────────────────────────

static void strikerLoop() {
    #if ATTACK_GOAL == 0
    bool  goalVisible = camBlueDetected;
    float goalAngle   = camBlueAngle;
    float goalDist    = camBlueDist;
    #else
    bool  goalVisible = camYellowDetected;
    float goalAngle   = camYellowAngle;
    float goalDist    = camYellowDist;
    #endif

    float omega = computeFaceOmega(goalVisible, goalAngle, goalDist);

    // ── Scoring ──────────────────────────────────────────────────────
    if (checkCatchment() && goalVisible && goalDist < SCORE_DIST_THRESH) {
        kickSol();
        Serial.printf("[KICK] goalDist=%.1f\n", goalDist);
    }

    // ── IR orbit ──────────────────────────────────────────────────────
    if (l3BallDetected) {
        float ballAngle = l3BallAngle;
        if (ballAngle > 180.0f) ballAngle -= 360.0f;

        static unsigned long lastCamBallMs = 0;
        if (camBallDetected) lastCamBallMs = millis();
        bool closeEnough = (millis() - lastCamBallMs < 2000);
        float moveAngle, speed;

        if (!closeEnough) {
            moveAngle = ballAngle;
            speed = IR_CHASE_SPEED;
            Serial.printf("[IR CHASE] ball=%.1f\n", l3BallAngle);
        } else {
            float absBall = fabsf(ballAngle);
            float offsetScale = fminf(absBall / IR_ORBIT_FADE_DEG, 1.0f);
            float sign = (ballAngle > 0) ? 1.0f : -1.0f;
            moveAngle = ballAngle + sign * IR_ORBIT_OFFSET * offsetScale;
            moveAngle = constrain(moveAngle, -180.0f, 180.0f);
            speed = IR_ORBIT_SPEED_MIN
                  + (IR_ORBIT_SPEED_MAX - IR_ORBIT_SPEED_MIN) * (1.0f - offsetScale);
        }

        float capSpeed = closeEnough ? IR_ORBIT_SPEED_MAX : IR_CHASE_SPEED;
        float total = speed + fabsf(omega);
        if (total > capSpeed) {
            float scale = capSpeed / total;
            speed *= scale;
            omega *= scale;
        }

        float wrapped = fmodf(moveAngle + 360.0f, 90.0f);
        float distToDead = fabsf(wrapped - 45.0f);
        if (distToDead < XDRIVE_DEAD_NUDGE) {
            float nudge = XDRIVE_DEAD_NUDGE - distToDead;
            moveAngle += (wrapped < 45.0f) ? -nudge : nudge;
        }

        applyLineAvoidance(moveAngle, speed, omega);

        moveRobot(moveAngle, speed, omega);
        if (closeEnough)
            Serial.printf("[IR ORBIT] ball=%.1f move=%.1f spd=%.2f omega=%.2f count=%d\n",
                          l3BallAngle, moveAngle, speed, omega, l3BallCount);
    } else if (locValid) {
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
        Serial.printf("[IDLE] goal=%d err=%.1f omega=%.2f\n", goalVisible, prevFaceErr, omega);
    }
}

// ── Goalie ─────────────────────────────────────────────────────────────────
// Curve-following goalie ported from robocup-2025.
// Robot tracks along y = A*x^6 + Y0 in front of own goal.
// Uses localisation for position, IR for ball direction.

// Sign flip: ATTACK_GOAL==1 → defend blue (negative Y), ATTACK_GOAL==0 → defend yellow (positive Y)
#if ATTACK_GOAL == 1
static const float GK_SIGN = 1.0f;
#else
static const float GK_SIGN = -1.0f;
#endif

static float gkPrevError   = 0;
static float gkLastBallX   = 0;  // last-seen ball side for when ball lost

// The goalie curve: y = GK_SIGN * (A * x^6 + Y0)
static float gkCurveY(float x) {
    float x2 = x * x;
    float x6 = x2 * x2 * x2;
    return GK_SIGN * (GOALIE_CURVE_A * x6 + GOALIE_CURVE_Y0);
}

// First derivative dy/dx = GK_SIGN * 6A * x^5
static float gkCurveDY(float x) {
    float x2 = x * x;
    float x5 = x2 * x2 * x;
    return GK_SIGN * 6.0f * GOALIE_CURVE_A * x5;
}

// Newton's method: find closest x on curve to point (px, py)
static float gkClosestX(float px, float py) {
    float x = constrain(px, -GOALIE_X_MAX, GOALIE_X_MAX);
    for (int i = 0; i < 30; i++) {
        float y  = gkCurveY(x);
        float dy = gkCurveDY(x);
        float x2 = x * x, x4 = x2 * x2;
        float d2y = GK_SIGN * 30.0f * GOALIE_CURVE_A * x4;
        float grad = 2.0f * (x - px) + 2.0f * (y - py) * dy;
        float hess = 2.0f + 2.0f * dy * dy + 2.0f * (y - py) * d2y;
        if (fabsf(grad) < 1e-4f) break;
        x -= grad / hess;
    }
    return constrain(x, -GOALIE_X_MAX, GOALIE_X_MAX);
}

// Step along curve from current toward target by stepSize cm
static void gkStepAlongCurve(float curX, float tgtX, float stepSize,
                              float& outX, float& outY) {
    float slope = gkCurveDY(curX);
    float dx = stepSize / sqrtf(1.0f + slope * slope);
    dx = (tgtX > curX) ? dx : -dx;
    // Don't overshoot
    if (fabsf(dx) > fabsf(tgtX - curX)) {
        dx = tgtX - curX;
    }
    outX = constrain(curX + dx, -GOALIE_X_MAX, GOALIE_X_MAX);
    outY = gkCurveY(outX);
}

// Edge speed scaling (from 2025): uses average goal x to slow near edges
static float gkEdgeSpeedCap() {
    float avgGoalX = 0;
    if (camBlueDetected && camYellowDetected) {
        avgGoalX = fabsf((camBlueAngle + camYellowAngle) * 0.5f);
    } else if (camBlueDetected) {
        avgGoalX = fabsf(camBlueAngle);
    } else if (camYellowDetected) {
        avgGoalX = fabsf(camYellowAngle);
    }
    return fminf(GOALIE_EDGE_A * expf(-GOALIE_EDGE_B * avgGoalX + GOALIE_EDGE_C)
                 + GOALIE_EDGE_D, GOALIE_SPEED_MAX);
}

static void goalieLoop() {
    if (!locValid) {
        moveRobot(0, 0, 0);
        Serial.println("[GK] no loc, holding");
        return;
    }

    // 1. Project robot onto curve
    float curX = gkClosestX(locX, locY);
    float curY = gkCurveY(curX);

    // 2. Determine target X from IR ball angle
    float tgtX = 0;
    if (l3BallDetected) {
        float ballAng = l3BallAngle;
        if (ballAng > 180.0f) ballAng -= 360.0f;
        tgtX = GK_SIGN * GOALIE_BALL_X_SCALE * sinf(ballAng * DEG_TO_RAD);
        gkLastBallX = tgtX;
    } else {
        // No ball: drift to last-seen side (like 2025 raffles mode)
        tgtX = (gkLastBallX > 0) ? GOALIE_X_MAX * 0.5f : -GOALIE_X_MAX * 0.5f;
    }
    tgtX = constrain(tgtX, -GOALIE_X_MAX, GOALIE_X_MAX);

    // 3. Compute step multiplier from ball angle (faster step when ball far off-center)
    float ballAbsAng = 0;
    if (l3BallDetected) {
        ballAbsAng = l3BallAngle;
        if (ballAbsAng > 180.0f) ballAbsAng -= 360.0f;
        ballAbsAng = fabsf(ballAbsAng);
        if (ballAbsAng > 90.0f) ballAbsAng = 180.0f - ballAbsAng;
    }
    float stepMult = (ballAbsAng > 35.0f) ? 3.0f : 1.0f;

    // 4. Step along curve toward target
    float nextX, nextY;
    gkStepAlongCurve(curX, tgtX, stepMult * GOALIE_STEP_SIZE, nextX, nextY);
    if (GK_SIGN > 0)
        nextY = constrain(nextY, GOALIE_Y_MIN, 0.0f);      // defend blue: negative Y side
    else
        nextY = constrain(nextY, 0.0f, -GOALIE_Y_MIN);     // defend yellow: positive Y side

    // 5. PD speed from ball angle error
    float error = 0;
    if (l3BallDetected && fabsf(l3BallAngle > 180.0f ? l3BallAngle - 360.0f : l3BallAngle) > 0.1f) {
        float ang = l3BallAngle;
        if (ang > 180.0f) ang -= 360.0f;
        ang = fabsf(ang);
        error = constrain((15.0f * expf(0.2f * ang - 5.0f)) + 4.0f * ang, 0.0f, 300.0f);
    }
    float pTerm = GOALIE_SPEED_KP * error;
    float dTerm = GOALIE_SPEED_KD * (error - gkPrevError);
    gkPrevError = error;

    float maxSpd = gkEdgeSpeedCap();
    float speed  = constrain(pTerm + dTerm, GOALIE_SPEED_MIN, maxSpd);

    if (!l3BallDetected) {
        speed = constrain(speed, 0.10f, 0.20f);
    }

    // 6. Drive toward next point on curve
    float errX = nextX - locX;
    float errY = nextY - locY;
    float dist = sqrtf(errX * errX + errY * errY);

    if (dist < 2.0f) {
        moveRobot(0, 0, 0);
        Serial.printf("[GK] on curve, dist=%.1f\n", dist);
        return;
    }

    float toTargetWorld = atan2f(errX, errY) * RAD_TO_DEG;
    float moveAngle = toTargetWorld - locHeading;
    if (moveAngle >  180.0f) moveAngle -= 360.0f;
    if (moveAngle < -180.0f) moveAngle += 360.0f;

    float omega = 0;
    applyLineAvoidance(moveAngle, speed, omega);

    moveRobot(moveAngle, speed, omega);
    Serial.printf("[GK] cur=(%.0f,%.0f) tgt=(%.0f,%.0f) nxt=(%.0f,%.0f) spd=%.2f\n",
                  curX, curY, tgtX, gkCurveY(tgtX), nextX, nextY, speed);
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

    Serial.println(imuYaw);

    #if ROLE == 0
    strikerLoop();
    #else
    goalieLoop();
    #endif

    // moveRobot(0, 0.2, -0.2);
}
