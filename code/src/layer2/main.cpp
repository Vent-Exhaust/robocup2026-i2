#include "main.h"

// ── Task state machine ──────────────────────────────────────────────────────
enum Task { ORBIT_TO_BALL, SCORE, GOALIE, RETURN_TO_CENTRE, IDLE };
static Task task = IDLE;

// ── Striker state ───────────────────────────────────────────────────────────
static unsigned long noCatchmentStart = 0;
static bool wasCaught = false;
static const unsigned long CATCHMENT_TIMEOUT = 500; // ms

// ── Movement output (set by task functions, applied at end of loop) ─────────
static float moveAngle = 0;
static float moveSpeed = 0;
static float moveOmega = 0;
static float lastMoveAngle = 0;

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

// ── Ball detection helper ───────────────────────────────────────────────────

static bool ballFound = false;
static float ballAngle = 0;

static void updateBallState() {
    ballFound = false;
    ballAngle = 0;

    if (l3BallDetected) {
        ballFound = true;
        ballAngle = l3BallAngle;
    }
    // Camera ball is unreliable at distance — only use as fallback if needed
    // else if (camBallDetected) {
    //     ballFound = true;
    //     ballAngle = camBallAngle;
    // }
}

// ── Task functions ──────────────────────────────────────────────────────────

static void doLineEscape() {
    Serial.printf("[MODE] LINE DETECTED angle=%.1f size=%.1f\n", l1Angle, l1Size);
    float l1Escape = fmod(l1Angle + 180.0f, 360.0f);
    float reverseMove = fmod(lastMoveAngle + 180.0f, 360.0f);

    float l1Rad = l1Escape * DEG_TO_RAD;
    float revRad = reverseMove * DEG_TO_RAD;
    float bx = 0.5f * cosf(l1Rad) + 0.5f * cosf(revRad);
    float by = 0.5f * sinf(l1Rad) + 0.5f * sinf(revRad);
    float escapeAngle = atan2f(by, bx) * RAD_TO_DEG;
    escapeAngle = fmod(escapeAngle + 360.0f, 360.0f);

    static const float LINE_KP = 1.0f;
    float escapeSpeed = constrain(l1Size * LINE_KP, 0.08f * LOCATION_SPEED_MULT, 0.3f);
    moveAngle = escapeAngle;
    moveSpeed = escapeSpeed;
    moveOmega = 0;
}

static void doOrbitToBall() {
    if (camBallDetected && l3BallDetected) {
        Serial.println("[MODE] ORBIT (cam+IR)");
        float irAngle = l3BallAngle;

        // Goal angle in robot frame
        float goalAngle;
        if (camBlueDetected) {
            goalAngle = camBlueAngle;
        } else {
            goalAngle = -imuYaw;
        }
        if (goalAngle >  180.0f) goalAngle -= 360.0f;
        if (goalAngle < -180.0f) goalAngle += 360.0f;

        float ballAng = irAngle;
        if (ballAng > 180.0f) ballAng -= 360.0f;

        float alignError = goalAngle - ballAng;
        if (alignError >  180.0f) alignError -= 360.0f;
        if (alignError < -180.0f) alignError += 360.0f;

        float tangentDir = (alignError > 0) ? -90.0f : 90.0f;

        static float orbitIntegral = 0;
        static float prevAlignError = 0;
        static bool headingLocked = false;
        static float lockedHeading = 0;

        if (fabsf(alignError) <= CAM_ORBIT_DEADZONE) {
            // Aligned — lock ball heading once then drive straight
            if (!headingLocked) {
                lockedHeading = irAngle;
                headingLocked = true;
            }

            orbitIntegral = 0;
            prevAlignError = alignError;

            static const float CREEP_SPEED = 0.15f * LOCATION_SPEED_MULT;
            moveAngle = lockedHeading;
            moveSpeed = CREEP_SPEED;

            // Rotate to face the goal while driving straight
            float rotError = goalAngle;
            if (rotError >  180.0f) rotError -= 360.0f;
            if (rotError < -180.0f) rotError += 360.0f;
            static const float ALIGN_ROT_KP = 0.003f;
            moveOmega = constrain(rotError * ALIGN_ROT_KP, -0.5f, 0.5f);
            if (fabsf(rotError) < 5.0f) moveOmega = 0;

            Serial.printf("[ORBIT] ALIGNED locked=%.1f alignErr=%.1f rot=%.1f\n",
                          lockedHeading, alignError, rotError);
            return;
        }

        // Outside deadzone — orbit normally, reset lock
        headingLocked = false;

        orbitIntegral = constrain(orbitIntegral + fabsf(alignError) * CAM_ORBIT_KI,
                                  0.0f, CAM_ORBIT_I_MAX);

        float derivative = alignError - prevAlignError;
        prevAlignError = alignError;

        float orbitSpeed = constrain(fabsf(alignError) * CAM_ORBIT_KP + orbitIntegral
                                     + fabsf(derivative) * CAM_ORBIT_KD,
                                     0.0f, CAM_ORBIT_SPEED * LOCATION_SPEED_MULT);

        float radiusError = camBallDist - CAM_ORBIT_RADIUS;
        float radiusAdjust = constrain(radiusError * CAM_ORBIT_RADIUS_KP, -30.0f, 30.0f);

        float tangentRad = (irAngle + tangentDir) * DEG_TO_RAD;
        float forwardRad = irAngle * DEG_TO_RAD;

        static const float CREEP_SPEED  = 0.15f * LOCATION_SPEED_MULT;
        static const float ALIGN_THRESH = 5.0f;
        float forwardSpeed = 0;
        if (fabsf(alignError) < ALIGN_THRESH) {
            forwardSpeed = CREEP_SPEED;
        }

        float mx = orbitSpeed * sinf(tangentRad) + forwardSpeed * sinf(forwardRad);
        float my = orbitSpeed * cosf(tangentRad) + forwardSpeed * cosf(forwardRad);

        moveAngle = fmod(atan2f(mx, my) * RAD_TO_DEG + 360.0f, 360.0f);
        moveSpeed = sqrtf(mx * mx + my * my);
        moveAngle = fmod(moveAngle - radiusAdjust + 360.0f, 360.0f);

        float rotError = irAngle;
        if (rotError > 180.0f) rotError -= 360.0f;
        static const float ORBIT_ROT_KP = 0.004f;
        moveOmega = constrain(rotError * ORBIT_ROT_KP, -0.5f, 0.5f);
        if (fabsf(rotError) < 5.0f) moveOmega = 0;

        Serial.printf("[ORBIT] alignErr=%.1f spd=%.2f creep=%.2f camDist=%.1f\n",
                      alignError, moveSpeed, forwardSpeed, camBallDist);
    } else {
        Serial.println("[MODE] IR CHASE (IR only)");
        moveAngle = ballAngle;

        static const float IR_CHASE_SPEED = 0.15f * LOCATION_SPEED_MULT;
        static const float IR_ROT_KP = 0.003f;

        float rotError = ballAngle;
        if (rotError > 180.0f) rotError -= 360.0f;
        moveOmega = constrain(rotError * IR_ROT_KP, -0.5f, 0.5f);
        if (fabsf(rotError) < 5.0f) moveOmega = 0;

        moveSpeed = IR_CHASE_SPEED;
        Serial.printf("[IR] chasing ball at %.1f rot=%.2f\n", ballAngle, moveOmega);
    }
}

static void doScore() {
    // For now: same as old scoring — drive forward + kick when close
    // Phase 3 will replace this with a proper scoring sequence
    Serial.printf("[MODE] SCORING dist=%.1f\n", camBlueDist);

    float goalError = camBlueAngle;
    if (goalError > 180.0f) goalError -= 360.0f;

    kickSol();

    moveAngle = 0;
    moveSpeed = 0.3f;
    moveOmega = 0;
    Serial.printf("[SCORE] dist=%.1f err=%.1f\n", camBlueDist, goalError);
}

static void doReturnToCentre() {
    Serial.println("[MODE] RETURN TO CENTRE");
    float distToCenter = sqrtf(locX * locX + locY * locY);

    if (distToCenter < LOC_DEADZONE) {
        moveSpeed = 0;
        moveAngle = 0;
        moveOmega = 0;
        Serial.printf("[RTC] at centre (dist=%.1f)\n", distToCenter);
    } else {
        float worldAngleDeg = atan2f(-locX, -locY) * RAD_TO_DEG;
        moveAngle = worldAngleDeg - locHeading;
        moveAngle = fmodf(moveAngle, 360.0f);
        if (moveAngle < 0) moveAngle += 360.0f;

        moveSpeed = constrain(distToCenter * LOC_KP, LOC_SPEED_MIN, LOC_SPEED_MAX);
        moveOmega = 0;
        Serial.printf("[RTC] dist=%.1f angle=%.1f spd=%.2f heading=%.1f\n",
                      distToCenter, moveAngle, moveSpeed, locHeading);
    }
}

static void doIdle() {
    Serial.println("[MODE] IDLE");
    moveSpeed = 0;
    moveAngle = 0;
    moveOmega = 0;
    Serial.println("[IDLE] no ball, no loc");
}

// ── Role functions ──────────────────────────────────────────────────────────

static void striker(bool ballCaught) {
    if (ballCaught) {
        noCatchmentStart = millis();
        wasCaught = true;

        if (camBlueDetected && camBlueDist < SCORE_DIST_THRESH) {
            task = SCORE;
        } else {
            // Ball caught but not close to goal — keep orbiting to get closer
            task = ballFound ? ORBIT_TO_BALL : RETURN_TO_CENTRE;
        }
    } else if (wasCaught) {
        // Ball was in catchment recently — give it a grace period
        if (millis() - noCatchmentStart > CATCHMENT_TIMEOUT) {
            task = ballFound ? ORBIT_TO_BALL : (locValid ? RETURN_TO_CENTRE : IDLE);
            wasCaught = false;
        } else {
            // Stay on current task (SCORE or ORBIT) during grace period
            if (task == SCORE && camBlueDetected && camBlueDist < SCORE_DIST_THRESH) {
                task = SCORE;
            } else {
                task = ballFound ? ORBIT_TO_BALL : (locValid ? RETURN_TO_CENTRE : IDLE);
            }
        }
    } else {
        task = ballFound ? ORBIT_TO_BALL : (locValid ? RETURN_TO_CENTRE : IDLE);
    }
}

static void goalie() {
    task = GOALIE;
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
    spinDribbler(DRIBBLER_SPEED);

    // ── Read sensors ────────────────────────────────────────────────────
    readL1();
    readL3();
    readCam();
    readIMU();

    // bool ballCaught = checkCatchment();
    // updateLocalisation();
    // updateBallState();

    // // ── Role selection ──────────────────────────────────────────────────
    // // Goalie disabled — developed on separate branch
    // striker(ballCaught);

    // // ── Line avoidance overrides everything ─────────────────────────────
    // if (l1LineDetected) {
    //     doLineEscape();
    // } else {
    //     // ── Execute current task ────────────────────────────────────────
    //     switch (task) {
    //     case ORBIT_TO_BALL:    doOrbitToBall();      break;
    //     case SCORE:            doScore();            break;
    //     case GOALIE:           /* Phase 4 */         doIdle(); break;
    //     case RETURN_TO_CENTRE: doReturnToCentre();   break;
    //     case IDLE:             doIdle();             break;
    //     }
    // }

    // // ── Apply movement ──────────────────────────────────────────────────
    // lastMoveAngle = moveAngle;

    // if (moveSpeed == 0) {
    //     stopMotors();
    // } else {
    //     moveRobot(moveAngle, moveSpeed, moveOmega);
    // }

    // if (locValid) {
    //     Serial.printf("[LOC] x=%.1f y=%.1f heading=%.1f\n", locX, locY, locHeading);
    // }

    moveRobot(0, 0.0125, -0.075);
}
