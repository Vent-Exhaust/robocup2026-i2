#include "cam_comm.h"

// CSV buffer
static char camBuf[192];
static uint8_t camBufIdx = 0;

// Blue goal
bool  camBlueDetected = false;
float camBlueX = 0, camBlueY = 0, camBlueDist = 0, camBlueAngle = 0;
float camBlueLBDist = 0, camBlueLBAngle = 0;

// Yellow goal
bool  camYellowDetected = false;
float camYellowX = 0, camYellowY = 0, camYellowDist = 0, camYellowAngle = 0;
float camYellowLBDist = 0, camYellowLBAngle = 0;

// Ball
bool  camBallDetected = false;
float camBallX = 0, camBallY = 0, camBallDist = 0, camBallAngle = 0;

void setupCamComm() {
    CAM_SERIAL.begin(115200);
}

// Parse a CSV field: returns the float value and sets `detected` to false if "none"
static float parseField(const char* token, bool& detected) {
    if (token[0] == 'n') { // "none"
        detected = false;
        return 0.0f;
    }
    return atof(token);
}

static void parseLine(char* line) {
    // Expected: BLUE_X,BLUE_Y,BLUE_DIST,BLUE_ANGLE,BLUE_LB_DIST,BLUE_LB_ANGLE,
    //           YELLOW_X,YELLOW_Y,YELLOW_DIST,YELLOW_ANGLE,YELLOW_LB_DIST,YELLOW_LB_ANGLE,
    //           BALL_X,BALL_Y,BALL_DIST,BALL_ANGLE
    char* tokens[16];
    uint8_t count = 0;

    char* tok = strtok(line, ",");
    while (tok && count < 16) {
        tokens[count++] = tok;
        tok = strtok(NULL, ",");
    }
    if (count < 16) return;

    // Blue goal (fields 0-5)
    camBlueDetected = true;
    camBlueX       = parseField(tokens[0], camBlueDetected);
    camBlueY       = parseField(tokens[1], camBlueDetected);
    camBlueDist    = parseField(tokens[2], camBlueDetected);
    camBlueAngle   = parseField(tokens[3], camBlueDetected);
    camBlueLBDist  = parseField(tokens[4], camBlueDetected);
    camBlueLBAngle = parseField(tokens[5], camBlueDetected);

    // Yellow goal (fields 6-11)
    camYellowDetected = true;
    camYellowX       = parseField(tokens[6],  camYellowDetected);
    camYellowY       = parseField(tokens[7],  camYellowDetected);
    camYellowDist    = parseField(tokens[8],  camYellowDetected);
    camYellowAngle   = parseField(tokens[9],  camYellowDetected);
    camYellowLBDist  = parseField(tokens[10], camYellowDetected);
    camYellowLBAngle = parseField(tokens[11], camYellowDetected);

    // Ball (fields 12-15)
    camBallDetected = true;
    camBallX     = parseField(tokens[12], camBallDetected);
    camBallY     = parseField(tokens[13], camBallDetected);
    camBallDist  = parseField(tokens[14], camBallDetected);
    camBallAngle = parseField(tokens[15], camBallDetected);
}

bool readCam() {
    while (CAM_SERIAL.available()) {
        char c = CAM_SERIAL.read();
        if (c == '\n' || c == '\r') {
            if (camBufIdx > 0) {
                camBuf[camBufIdx] = '\0';
                parseLine(camBuf);
                camBufIdx = 0;
                return true;
            }
        } else if (camBufIdx < sizeof(camBuf) - 1) {
            camBuf[camBufIdx++] = c;
        }
    }
    return false;
}

void debugCamReadings() {
    if (camBlueDetected) {
        Serial.print("[CAM] Blue   | X: "); Serial.print(camBlueX, 1);
        Serial.print(" Y: "); Serial.print(camBlueY, 1);
        Serial.print(" Dist: "); Serial.print(camBlueDist, 1);
        Serial.print(" Angle: "); Serial.println(camBlueAngle, 1);
    } else {
        Serial.println("[CAM] Blue   | none");
    }

    if (camYellowDetected) {
        Serial.print("[CAM] Yellow | X: "); Serial.print(camYellowX, 1);
        Serial.print(" Y: "); Serial.print(camYellowY, 1);
        Serial.print(" Dist: "); Serial.print(camYellowDist, 1);
        Serial.print(" Angle: "); Serial.println(camYellowAngle, 1);
    } else {
        Serial.println("[CAM] Yellow | none");
    }

    if (camBallDetected) {
        Serial.print("[CAM] Ball   | X: "); Serial.print(camBallX, 1);
        Serial.print(" Y: "); Serial.print(camBallY, 1);
        Serial.print(" Dist: "); Serial.print(camBallDist, 1);
        Serial.print(" Angle: "); Serial.println(camBallAngle, 1);
    } else {
        Serial.println("[CAM] Ball   | none");
    }
}
