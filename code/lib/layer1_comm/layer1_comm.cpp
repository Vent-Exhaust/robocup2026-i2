#include "layer1_comm.h"

static SerialComm l1Comm(L1_TO_L2_SERIAL);

bool  l1LineDetected = false;
float l1Angle        = 0;
float l1Size         = 0;
int   l1StartLdr     = -1;
int   l1EndLdr       = -1;

void setupL1Comm() {
    L1_TO_L2_SERIAL.begin(115200);
}

bool readL1() {
    int16_t rx[5];
    if (l1Comm.read(rx, 5) == 5) {
        l1LineDetected = rx[0];
        l1Angle        = rx[1] / 10.0f;
        l1Size         = rx[2] / 10.0f;
        l1StartLdr     = (int)rx[3];
        l1EndLdr       = (int)rx[4];
        return true;
    }
    return false;
}

void debugL1Readings() {
    if (l1LineDetected) {
        Serial.print("[L1] Line | Angle: ");
        Serial.print(l1Angle, 1);
        Serial.print(" deg | Size: ");
        Serial.println(l1Size, 1);
    } else {
        Serial.println("[L1] No line");
    }
}
