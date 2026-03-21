#include "layer3_comm.h"

static SerialComm l3Comm(L3_TO_L2_SERIAL);

// Ball (from IR ring)
bool  l3BallDetected = false;
float l3BallAngle = 0;
int   l3BallCount = 0;

// Switches
bool l3SwitchGoal   = false;
bool l3SwitchRole   = false;
bool l3SwitchStrat0 = false;
bool l3SwitchStrat1 = false;

void setupL3Comm() {
    L3_TO_L2_SERIAL.begin(115200);
}

bool readL3() {
    int16_t rx[7];
    if (l3Comm.read(rx, 7) == 7) {
        l3BallDetected = rx[0];
        l3BallAngle    = rx[1] / 10.0f;
        l3BallCount    = rx[2];
        l3SwitchGoal   = rx[3];
        l3SwitchRole   = rx[4];
        l3SwitchStrat0 = rx[5];
        l3SwitchStrat1 = rx[6];
        return true;
    }
    return false;
}

void debugL3Readings() {
    if (l3BallDetected) {
        Serial.printf("[L3] Ball | Angle: %6.1f deg | Count: %2d\n",
                      l3BallAngle, l3BallCount);
    } else {
        Serial.printf("[L3] Ball |              none              \n");
    }
    Serial.printf("[L3] SW   | goal=%d role=%d strat0=%d strat1=%d\n",
                  l3SwitchGoal, l3SwitchRole, l3SwitchStrat0, l3SwitchStrat1);
}
