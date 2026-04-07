#include "main.h"

// Localisation state
static const float GOAL_SEP     = 194.0f;
static const float HALF_FIELD   = 97.0f;
static float imuOffset          = 0;
static bool  imuOffsetValid     = false;
float locX = 0, locY = 0, locHeading = 0;
bool  locValid = false;

// Goalie tuning
static const float GOALIE_LATERAL_DEADZONE_DEG = 20.0f;  // ball within this many degrees of straight behind → stop sliding

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

    // IR is primary. Camera bearing only used when IR can't see the ball
    // and the ball is close enough for the camera to be reliable.
    static const float CAM_BALL_NEAR_DIST = 60.0f;  // cm
    if (l3BallDetected) {
        ballFound = true;
        ballAngle = l3BallAngle;
    } else if (camBallDetected && camBallDist < CAM_BALL_NEAR_DIST) {
        ballFound = true;
        ballAngle = camBallAngle;
    }

    static float lastMoveAngle = 0;
    static bool  goalParked    = false;

    // Latch onto line when first detected
    if (l1LineDetected && !goalParked) {
        goalParked = true;
        Serial.println("[GOALIE] reached blue goal line");
    }

    if (goalParked) {
        // === ON LINE: track ball laterally, re-acquire line if lost ===
        // Ref: open/software design teensy1/robot.cpp — goalieTrack(), trackLineGoalie()
        static const float GOALIE_TRACK_SPEED     = 0.5f;
        static const float GOALIE_TRACK_MIN       = 0.1f;
        static const float GOALIE_REACQUIRE_SPEED = 0.10f;
        static const float GOALIE_CENTRE_SPEED    = 0.2f;
        static const float GOALIE_CENTRE_DEADZONE = 5.0f;   // cm

        // Track last seen ball side for when ball disappears (ref: goalieTrack raffles_goalie)
        static float lastBallLateral = 0.0f;

        if (!l1LineDetected) {
            // Drifted forward off the line — back up to re-acquire
            lastMoveAngle = 180.0f;
            moveRobot(180.0f, GOALIE_REACQUIRE_SPEED, 0);
            Serial.println("[GOALIE] re-acquiring line");
        } else if (ballFound) {
            // Lateral component of ball angle — ref: goalieTrack uses world-frame ball.x
            float ballRad     = ballAngle * DEG_TO_RAD;
            float ballLateral = sinf(ballRad);  // -1 = left, +1 = right
            lastBallLateral   = ballLateral;

            // Dead zone: if ball is nearly straight behind (within GOALIE_LATERAL_DEADZONE_DEG of 180°),
            // stop lateral — ref: trackLineGoalie abs(correction - 180) < 20
            float angleFrom180 = fabsf(fmodf(ballAngle + 180.0f, 360.0f) - 180.0f);
            if (angleFrom180 < GOALIE_LATERAL_DEADZONE_DEG) {
                stopMotors();
                Serial.printf("[GOALIE] dead zone ball=%.1f\n", ballAngle);
            } else {
                float trackAngle = (ballLateral > 0) ? 90.0f : 270.0f;
                float trackSpeed = constrain(fabsf(ballLateral) * GOALIE_TRACK_SPEED,
                                             GOALIE_TRACK_MIN, GOALIE_TRACK_SPEED);
                lastMoveAngle = trackAngle;
                moveRobot(trackAngle, trackSpeed, 0);
                Serial.printf("[GOALIE] tracking ball=%.1f lateral=%.2f spd=%.2f\n",
                              ballAngle, ballLateral, trackSpeed);
            }
        } else if (locValid && fabsf(locX) > GOALIE_CENTRE_DEADZONE) {
            // No ball — ref: goalieTrack returns to centre (raffles_goalie biases to last seen side,
            // we always return to centre since locX is reliable)
            float centreAngle = (locX > 0) ? 270.0f : 90.0f;
            float centreSpeed = constrain(fabsf(locX) * 0.005f,
                                          0.05f, GOALIE_CENTRE_SPEED);
            lastMoveAngle = centreAngle;
            moveRobot(centreAngle, centreSpeed, 0);
            Serial.printf("[GOALIE] centring locX=%.1f\n", locX);
        } else if (!locValid && fabsf(lastBallLateral) > 0.1f) {
            // No localisation, no ball — bias toward last seen ball side briefly
            // ref: goalieTrack — when no ball, use ball_last_seen
            float biasAngle = (lastBallLateral > 0) ? 90.0f : 270.0f;
            moveRobot(biasAngle, GOALIE_TRACK_MIN, 0);
            Serial.println("[GOALIE] biasing to last seen side");
        } else {
            stopMotors();
        }
    } else {
        // === DRIVE to blue goal line ===
        static const float GOALIE_SPEED = 0.15f;
        float moveAngle = 180.0f;  // fallback: straight back
        float speed     = GOALIE_SPEED;

        if (locValid) {
            float dx = -locX;
            float dy = -HALF_FIELD - locY;  // blue goal at world Y = -HALF_FIELD
            float dist = sqrtf(dx * dx + dy * dy);
            float worldAngle = atan2f(dx, dy) * RAD_TO_DEG;
            moveAngle = fmodf(worldAngle - locHeading + 360.0f, 360.0f);
            speed = constrain(dist * 0.005f, 0.08f, GOALIE_SPEED);
            Serial.printf("[GOALIE] loc dist=%.1f\n", dist);
        } else if (camBlueDetected) {
            moveAngle = camBlueAngle;
            speed = constrain(camBlueDist * 0.005f, 0.08f, GOALIE_SPEED);
            Serial.printf("[GOALIE] cam → blue dist=%.1f\n", camBlueDist);
        }

        lastMoveAngle = moveAngle;
        moveRobot(moveAngle, speed, 0);
    }

    if (false) {  // striker disabled
        // === STRIKER MODE ===
        if (ballCaught && camBlueDetected && camBlueDist < 50.0f) {
            // Ball caught and near goal — charge and kick
            float goalError = camBlueAngle;
            if (goalError > 180.0f) goalError -= 360.0f;

            kickSol();

            lastMoveAngle = 0;
            moveRobot(0, 0.3f, 0);
            Serial.printf("[SCORE] dist=%.1f err=%.1f\n", camBlueDist, goalError);
        } else if (ballFound) {
            if (camBallDetected && l3BallDetected) {
                // === Orbit mode: IR angle for direction, cam distance for radius ===
                float irAngle = l3BallAngle;

                // Alignment: determine orbit direction
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

                // Alignment error: goal vs ball angle from robot's POV
                float alignError = goalAngle - ballAng;
                if (alignError >  180.0f) alignError -= 360.0f;
                if (alignError < -180.0f) alignError += 360.0f;

                // KP control: orbit speed & direction proportional to alignment error
                // Positive error → CW (-90°), negative → CCW (+90°)
                float tangentDir = (alignError > 0) ? -90.0f : 90.0f;

                float orbitSpeed = constrain(fabsf(alignError) * CAM_ORBIT_KP,
                                             0.0f, CAM_ORBIT_SPEED);

                // Radius maintenance: adjust angle toward/away from ball
                float radiusError = camBallDist - CAM_ORBIT_RADIUS;
                float radiusAdjust = constrain(radiusError * CAM_ORBIT_RADIUS_KP, -30.0f, 30.0f);

                // When aligned, blend in a forward creep toward ball
                static const float ALIGN_THRESH = 5.0f;
                static const float CREEP_SPEED  = 0.15f;
                float forwardSpeed = 0;
                if (fabsf(alignError) < ALIGN_THRESH) {
                    forwardSpeed = CREEP_SPEED;
                }

                // Combine orbit tangent + forward creep via vector sum
                float tangentRad = (irAngle + tangentDir) * DEG_TO_RAD;
                float forwardRad = irAngle * DEG_TO_RAD;  // toward ball

                float mx = orbitSpeed * sinf(tangentRad) + forwardSpeed * sinf(forwardRad);
                float my = orbitSpeed * cosf(tangentRad) + forwardSpeed * cosf(forwardRad);

                float moveAngle = fmod(atan2f(mx, my) * RAD_TO_DEG + 360.0f, 360.0f);
                float moveSpeed  = sqrtf(mx * mx + my * my);

                // Apply radius adjustment on top
                moveAngle = fmod(moveAngle - radiusAdjust + 360.0f, 360.0f);

                // Face the ball: rotate toward IR ball angle
                float rotError = irAngle;
                if (rotError > 180.0f) rotError -= 360.0f;
                static const float ORBIT_ROT_KP = 0.003f;
                float omega = constrain(rotError * ORBIT_ROT_KP, -0.5f, 0.5f);
                if (fabsf(rotError) < 5.0f) omega = 0;

                lastMoveAngle = moveAngle;
                moveRobot(moveAngle, moveSpeed, omega);
                Serial.printf("[ORBIT] alignErr=%.1f spd=%.2f creep=%.2f camDist=%.1f\n",
                              alignError, moveSpeed, forwardSpeed, camBallDist);
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
    }

    if (locValid) {
        Serial.printf("[LOC] x=%.1f y=%.1f heading=%.1f\n", locX, locY, locHeading);
    }
}
