#include <Arduino.h>
#include "movement.h"
#include "cam_comm.h"
#include "layer3_comm.h"
#include "config.h"

// === Localisation (copied from production) ===
static const float GOAL_SEP    = 194.0f;
static const float HALF_FIELD  = 97.0f;
static float imuOffset         = 0;
static bool  imuOffsetValid    = false;
float locX = 0, locY = 0, locHeading = 0;
bool  locValid = false;

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

// === Goalie curve (hardcoded to defend yellow: GK_SIGN = -1) ===
static const float GK_SIGN = -1.0f;  // defend yellow goal (positive Y side)

static float gkCurveY(float x) {
    float x2 = x * x;
    float x6 = x2 * x2 * x2;
    return GK_SIGN * (GOALIE_CURVE_A * x6 + GOALIE_CURVE_Y0);
}

static float gkCurveDY(float x) {
    float x2 = x * x;
    float x5 = x2 * x2 * x;
    return GK_SIGN * 6.0f * GOALIE_CURVE_A * x5;
}

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

static void gkStepAlongCurve(float curX, float tgtX, float stepSize,
                              float& outX, float& outY) {
    float slope = gkCurveDY(curX);
    float dx = stepSize / sqrtf(1.0f + slope * slope);
    dx = (tgtX > curX) ? dx : -dx;
    if (fabsf(dx) > fabsf(tgtX - curX)) {
        dx = tgtX - curX;
    }
    outX = constrain(curX + dx, -GOALIE_X_MAX, GOALIE_X_MAX);
    outY = gkCurveY(outX);
}

// === State ===
static float gkPrevError = 0;
static float gkLastBallX = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    setupMotors();
    setupIMU();
    setupCamComm();
    setupL3Comm();

    delay(500);
    resetYawTarget();

    Serial.println("=== Goalie Debug (Yellow Goal) ===");
    Serial.printf("GK_SIGN=%.0f  CURVE_A=%.2e  CURVE_Y0=%.1f  X_MAX=%.0f\n",
                  GK_SIGN, GOALIE_CURVE_A, GOALIE_CURVE_Y0, GOALIE_X_MAX);

    // Print curve shape
    Serial.println("Curve: x -> y");
    for (float x = -GOALIE_X_MAX; x <= GOALIE_X_MAX; x += 20.0f) {
        Serial.printf("  x=%6.1f  y=%6.1f\n", x, gkCurveY(x));
    }
}

void loop() {
    readCam();
    readIMU();
    readL3();
    updateLocalisation();

    // --- Print sensor state ---
    Serial.printf("[LOC] valid=%d x=%.1f y=%.1f hdg=%.1f | ",
                  locValid, locX, locY, locHeading);
    Serial.printf("[IR] det=%d ang=%.1f | ", l3BallDetected, l3BallAngle);
    Serial.printf("[CAM] blue=%d yellow=%d\n", camBlueDetected, camYellowDetected);

    if (!locValid) {
        moveRobot(0, 0, 0);
        Serial.println("[GK] no loc, stopping");
        return;
    }

    // 1. Project onto curve
    float curX = gkClosestX(locX, locY);
    float curY = gkCurveY(curX);

    // 2. Target X from IR
    float tgtX = 0;
    if (l3BallDetected) {
        float ballAng = l3BallAngle;
        if (ballAng > 180.0f) ballAng -= 360.0f;
        tgtX = GOALIE_BALL_X_SCALE * sinf(ballAng * DEG_TO_RAD);
        gkLastBallX = tgtX;
    } else {
        tgtX = (gkLastBallX > 0) ? GOALIE_X_MAX * 0.5f : -GOALIE_X_MAX * 0.5f;
    }
    tgtX = constrain(tgtX, -GOALIE_X_MAX, GOALIE_X_MAX);

    // 3. Step multiplier
    float ballAbsAng = 0;
    if (l3BallDetected) {
        ballAbsAng = l3BallAngle;
        if (ballAbsAng > 180.0f) ballAbsAng -= 360.0f;
        ballAbsAng = fabsf(ballAbsAng);
        if (ballAbsAng > 90.0f) ballAbsAng = 180.0f - ballAbsAng;
    }
    float stepMult = (ballAbsAng > 35.0f) ? 3.0f : 1.0f;

    // 4. Step along curve
    float nextX, nextY;
    gkStepAlongCurve(curX, tgtX, stepMult * GOALIE_STEP_SIZE, nextX, nextY);
    // Defend yellow: positive Y side
    nextY = constrain(nextY, 0.0f, -GOALIE_Y_MIN);

    // 5. Drive toward next point
    float errX = nextX - locX;
    float errY = nextY - locY;
    float dist = sqrtf(errX * errX + errY * errY);

    Serial.printf("[GK] curvePos=(%.0f,%.0f) tgt=(%.0f,%.0f) next=(%.0f,%.0f) err=(%.1f,%.1f) dist=%.1f\n",
                  curX, curY, tgtX, gkCurveY(tgtX), nextX, nextY, errX, errY, dist);

    if (dist < 2.0f) {
        moveRobot(0, 0, 0);
        Serial.println("[GK] on curve");
        return;
    }

    float toTargetWorld = atan2f(errX, errY) * RAD_TO_DEG;
    float moveAngle = toTargetWorld - locHeading;
    if (moveAngle >  180.0f) moveAngle -= 360.0f;
    if (moveAngle < -180.0f) moveAngle += 360.0f;

    float speed = constrain(dist * 0.005f, GOALIE_SPEED_MIN, GOALIE_SPEED_MAX);

    moveRobot(moveAngle, speed, 0);
    Serial.printf("[GK] moveAng=%.1f spd=%.2f worldAng=%.1f locHdg=%.1f\n",
                  moveAngle, speed, toTargetWorld, locHeading);
}
