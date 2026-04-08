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

// ---------------------------------------------------------------------------
// Line-following: sensor-to-world-angle lookup table
// Ref: open/software design teensy1/main.h — ldr_angles[chord_depth][sensor]
// 15 rows = chord depth tiers (0 = shallow, 14 = deep crossing).
// 32 columns = sensor index 0–31 (11.25° spacing, 0° = forward).
// Usage: ldrAngles[tier][31 - sensorIndex] + imuYaw → world-frame angle to
//        that sensor edge.  Select tier via round(l1Size * 14).
// ---------------------------------------------------------------------------
static const float LDR_ANGLES[15][32] = {
    { 318.43f,324.19f,329.99f,335.86f,341.87f,348.16f,355.30f,  8.36f,
      171.64f,184.70f,191.84f,198.13f,204.14f,210.01f,215.81f,221.57f,
      227.31f,233.03f,238.73f,244.42f,250.11f,255.80f,261.48f,267.16f,
      272.84f,278.52f,284.20f,289.89f,295.58f,301.27f,306.97f,312.69f },
    { 320.31f,326.49f,332.84f,339.47f,346.62f,354.91f,  6.50f, 36.04f,
      143.96f,173.50f,185.09f,193.38f,200.53f,207.16f,213.51f,219.69f,
      225.76f,231.76f,237.70f,243.62f,249.50f,255.37f,261.23f,267.08f,
      272.92f,278.77f,284.63f,290.50f,296.38f,302.30f,308.24f,314.24f },
    { 323.61f,330.51f,337.79f,345.70f,354.74f,  6.11f, 23.38f, 59.09f,
      120.91f,156.62f,173.89f,185.26f,194.30f,202.21f,209.49f,216.39f,
      223.04f,229.53f,235.91f,242.20f,248.43f,254.62f,260.78f,266.93f,
      273.07f,279.22f,285.38f,291.57f,297.80f,304.09f,310.47f,316.96f },
    { 328.53f,336.46f,345.04f,354.63f,  5.93f, 20.35f, 40.72f, 71.21f,
      108.79f,139.28f,159.65f,174.07f,185.37f,194.96f,203.54f,211.47f,
      218.97f,226.19f,233.19f,240.05f,246.80f,253.48f,260.10f,266.70f,
      273.30f,279.90f,286.52f,293.20f,299.95f,306.81f,313.81f,321.03f },
    { 335.31f,344.51f,354.55f,  5.82f, 18.92f, 34.70f, 54.12f, 77.43f,
      102.57f,125.88f,145.30f,161.08f,174.18f,185.45f,195.49f,204.69f,
      213.29f,221.47f,229.35f,236.99f,244.48f,251.84f,259.14f,266.38f,
      273.62f,280.86f,288.16f,295.52f,303.01f,310.65f,318.53f,326.71f },
    { 344.04f,354.48f,  5.74f, 18.04f, 31.60f, 46.64f, 63.18f, 80.91f,
       99.09f,116.82f,133.36f,148.40f,161.96f,174.26f,185.52f,195.96f,
      205.78f,215.12f,224.08f,232.76f,241.24f,249.56f,257.77f,265.93f,
      274.07f,282.23f,290.44f,298.76f,307.24f,315.92f,324.88f,334.22f },
    { 354.43f,  5.68f, 17.40f, 29.61f, 42.33f, 55.54f, 69.14f, 83.02f,
       96.98f,110.86f,124.46f,137.67f,150.39f,162.60f,174.32f,185.57f,
      196.41f,206.89f,217.08f,227.02f,236.76f,246.36f,255.86f,265.29f,
      274.71f,284.14f,293.64f,303.24f,312.98f,322.92f,333.11f,343.59f },
    {   5.63f, 16.88f, 28.12f, 39.37f, 50.63f, 61.87f, 73.13f, 84.38f,
       95.62f,106.88f,118.12f,129.37f,140.63f,151.87f,163.13f,174.38f,
      185.62f,196.87f,208.13f,219.37f,230.62f,241.88f,253.13f,264.37f,
      275.62f,286.87f,298.12f,309.38f,320.63f,331.87f,343.12f,354.37f },
    {  16.41f, 26.89f, 37.08f, 47.02f, 56.76f, 66.36f, 75.86f, 85.29f,
       94.71f,104.14f,113.64f,123.24f,132.98f,142.92f,153.11f,163.59f,
      174.43f,185.68f,197.40f,209.61f,222.33f,235.54f,249.14f,263.02f,
      276.98f,290.86f,304.46f,317.67f,330.39f,342.60f,354.32f,  5.57f },
    {  25.78f, 35.12f, 44.08f, 52.76f, 61.24f, 69.56f, 77.77f, 85.93f,
       94.07f,102.23f,110.44f,118.76f,127.24f,135.92f,144.88f,154.22f,
      164.04f,174.48f,185.74f,198.04f,211.60f,226.64f,243.18f,260.91f,
      279.09f,296.82f,313.36f,328.40f,341.96f,354.26f,  5.52f, 15.96f },
    {  33.29f, 41.47f, 49.35f, 56.99f, 64.48f, 71.84f, 79.14f, 86.38f,
       93.62f,100.86f,108.16f,115.52f,123.01f,130.65f,138.53f,146.71f,
      155.31f,164.51f,174.55f,185.82f,198.92f,214.70f,234.12f,257.43f,
      282.57f,305.88f,325.30f,341.08f,354.18f,  5.45f, 15.49f, 24.69f },
    {  38.97f, 46.19f, 53.19f, 60.05f, 66.80f, 73.48f, 80.10f, 86.70f,
       93.30f, 99.90f,106.52f,113.20f,119.95f,126.81f,133.81f,141.03f,
      148.53f,156.46f,165.04f,174.63f,185.93f,200.35f,220.72f,251.21f,
      288.79f,319.28f,339.65f,354.07f,  5.37f, 14.96f, 23.54f, 31.47f },
    {  43.04f, 49.53f, 55.91f, 62.20f, 68.43f, 74.62f, 80.78f, 86.93f,
       93.07f, 99.22f,105.38f,111.57f,117.80f,124.09f,130.47f,136.96f,
      143.61f,150.51f,157.79f,165.70f,174.74f,186.11f,203.38f,239.09f,
      300.91f,336.62f,353.89f,  5.26f, 14.30f, 22.21f, 29.49f, 36.39f },
    {  45.76f, 51.76f, 57.70f, 63.62f, 69.50f, 75.37f, 81.23f, 87.08f,
       92.92f, 98.77f,104.63f,110.50f,116.38f,122.30f,128.24f,134.24f,
      140.31f,146.49f,152.84f,159.47f,166.62f,174.91f,186.50f,216.03f,
      323.96f,353.50f,  5.09f, 13.38f, 20.53f, 27.16f, 33.51f, 39.69f },
    {  47.31f, 53.03f, 58.73f, 64.42f, 70.11f, 75.80f, 81.48f, 87.16f,
       92.84f, 98.52f,104.20f,109.89f,115.58f,121.27f,126.97f,132.69f,
      138.43f,144.19f,149.99f,155.86f,161.87f,168.16f,175.30f,188.36f,
      351.64f,  4.70f, 11.84f, 18.13f, 24.14f, 30.01f, 35.81f, 41.57f },
};

// Curve-following line tracker.
// Ref: open/software design teensy1/robot.cpp — trackLine()
// targetAngle: direction we want to travel (robot frame, 0° = forward).
//              Pass l3BallAngle (IR) as the target when chasing the ball.
// speed:       desired movement speed.
// Outputs moveRobot() call — moves along the line edge closest to targetAngle.
static void trackLine(float targetAngle, float speed) {
    if (l1StartLdr < 0 || l1EndLdr < 0) return;

    // Chord depth tier: maps l1Size (0–1) → row 0–14
    int tier = (int)constrain(roundf(l1Size * 14.0f), 0, 14);

    // Real-world angle to each edge sensor (ref: ldr_angles[7+offset][31-ldr])
    // offset here is 0 since we use tier directly
    float startAngle = fmodf(LDR_ANGLES[tier][31 - l1StartLdr] + imuYaw + 360.0f, 360.0f);
    float endAngle   = fmodf(LDR_ANGLES[tier][31 - l1EndLdr]   + imuYaw + 360.0f, 360.0f);

    // Angular error of each edge from the desired travel direction
    float errStart = fabsf(fmodf(targetAngle - startAngle + 540.0f, 360.0f) - 180.0f);
    float errEnd   = fabsf(fmodf(targetAngle - endAngle   + 540.0f, 360.0f) - 180.0f);

    // Move toward whichever edge is closest to the desired direction
    float correction = (errStart < errEnd) ? startAngle : endAngle;

    moveRobot(correction, speed, 0);
    Serial.printf("[LINE] tier=%d start=%.1f end=%.1f corr=%.1f\n",
                  tier, startAngle, endAngle, correction);
}

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
