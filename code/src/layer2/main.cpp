#include "main.h"

// Localisation state
static const float GOAL_SEP     = 194.0f;
static const float HALF_FIELD   = 97.0f;
static float imuOffset          = 0;
static bool  imuOffsetValid     = false;
float locX = 0, locY = 0, locHeading = 0;
bool  locValid = false;

// Return-to-centre tuning
static const float LOC_KP        = 0.003f;
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
    spinDribbler(DRIBBLER_SPEED);

    readL1();
    readL3();
    readCam();
    readIMU();

    bool ballCaught = checkCatchment();

    // Localisation
    updateLocalisation();

    bool ballFound = false;
    float ballAngle = 0;

    if (l3BallDetected) {
        ballFound = true;
        ballAngle = l3BallAngle;
    } else if (camBallDetected) {
        // ballFound = true;
        // ballAngle = camBallAngle;
    }

    // Line avoidance takes priority over everything
    static float lastMoveAngle = 0;
    if (l1LineDetected) {
        // Weighted escape: 50% reverse of prior movement, 50% L1 line-opposite
        float l1Escape = fmod(l1Angle + 180.0f, 360.0f);
        float reverseMove = fmod(lastMoveAngle + 180.0f, 360.0f);

        // Blend angles using vector averaging to handle wraparound
        float l1Rad = l1Escape * DEG_TO_RAD;
        float revRad = reverseMove * DEG_TO_RAD;
        float bx = 0.5f * cosf(l1Rad) + 0.5f * cosf(revRad);
        float by = 0.5f * sinf(l1Rad) + 0.5f * sinf(revRad);
        float escapeAngle = atan2f(by, bx) * RAD_TO_DEG;
        escapeAngle = fmod(escapeAngle + 360.0f, 360.0f);

        static const float LINE_KP = 1.0f;
        float escapeSpeed = constrain(l1Size * LINE_KP, 0.08f, 0.3f);
        moveRobot(escapeAngle, escapeSpeed, 0);
    } else if (false) {
    // } else if (ballCaught) {
        // Score towards target goal (swap to camYellow* to change target)
        bool targetDetected = camBlueDetected;
        float targetAngle   = camBlueAngle;
        float targetDist    = camBlueDist;

        if (targetDetected) {
            float goalError = targetAngle;
            if (goalError > 180.0f) goalError -= 360.0f;

            // Smooth rotation to keep ball in dribbler
            // Quadratic response: gentle near center, stronger at large errors
            static const float SCORE_ROT_KP  = 0.00004f;
            static const float SCORE_ROT_MAX = 0.12f;
            float sign = (goalError > 0) ? 1.0f : -1.0f;
            float omega = constrain(sign * goalError * goalError * SCORE_ROT_KP,
                                    -SCORE_ROT_MAX, SCORE_ROT_MAX);

            // Move forward towards goal
            static const float SCORE_SPEED = 0.2f;
            lastMoveAngle = 0;

            // Kick when close and roughly aligned
            if (targetDist < 70.0f && fabsf(goalError) < 20.0f) {
                kickSol();
            }

            moveRobot(0, SCORE_SPEED, omega);
            Serial.printf("[SCORE] goal err=%.1f dist=%.1f\n", goalError, targetDist);
        } else {
            // Goal not visible — creep forward and hope camera picks it up
            lastMoveAngle = 0;
            moveRobot(0, 0.15f, 0);
            Serial.println("[SCORE] goal not visible, creeping forward");
        }
    // } else if (false) {
    } else if (ballFound) {
        if (camBallDetected && l3BallDetected) {
            // === Orbit mode: IR angle for direction, cam distance for radius ===
            float irAngle = l3BallAngle;

            // Alignment: determine orbit direction (bang-bang)
            // Goal angle in robot frame
            float goalAngle;
            if (camBlueDetected) {
                goalAngle = camBlueAngle;
            } else {
                goalAngle = -imuYaw;  // world 0° in robot frame
            }
            if (goalAngle >  180.0f) goalAngle -= 360.0f;
            if (goalAngle < -180.0f) goalAngle += 360.0f;

            float ballAng = irAngle;
            if (ballAng > 180.0f) ballAng -= 360.0f;

            // If goal is to the right of ball, orbit CW (-90); else CCW (+90)
            float alignError = goalAngle - ballAng;
            if (alignError >  180.0f) alignError -= 360.0f;
            if (alignError < -180.0f) alignError += 360.0f;

            float tangentDir = (alignError > 0) ? -90.0f : 90.0f;

            // Radius maintenance: adjust angle toward/away from ball
            float radiusError = camBallDist - CAM_ORBIT_RADIUS;
            float radiusAdjust = constrain(radiusError * CAM_ORBIT_RADIUS_KP, -30.0f, 30.0f);

            float moveAngle = fmod(irAngle + tangentDir - radiusAdjust + 360.0f, 360.0f);

            // Face the ball: rotate toward IR ball angle
            float rotError = irAngle;
            if (rotError > 180.0f) rotError -= 360.0f;
            static const float ORBIT_ROT_KP = 0.003f;
            float omega = constrain(rotError * ORBIT_ROT_KP, -0.5f, 0.5f);
            if (fabsf(rotError) < 5.0f) omega = 0;

            lastMoveAngle = moveAngle;
            moveRobot(moveAngle, CAM_ORBIT_SPEED, omega);
            Serial.printf("[ORBIT] irAng=%.1f camDist=%.1f radErr=%.1f\n",
                          irAngle, camBallDist, radiusError);
        } else {
            // === IR chase mode — camera can't see ball, go straight at it ===
            float moveAngle = ballAngle;

            static const float IR_CHASE_SPEED = 0.15f;

            lastMoveAngle = moveAngle;
            moveRobot(moveAngle, IR_CHASE_SPEED, 0);
            Serial.printf("[IR] chasing ball at %.1f\n", ballAngle);
        }
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
            lastMoveAngle = moveAngle;
            moveRobot(moveAngle, speed, 0);
        }
    } else {
        stopMotors();
    }

    if (locValid) {
        Serial.printf("[LOC] x=%.1f y=%.1f heading=%.1f\n", locX, locY, locHeading);
    }
}
