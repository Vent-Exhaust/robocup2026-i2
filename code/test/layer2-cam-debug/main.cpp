#include <Arduino.h>
#include "cam_comm.h"

void setup() {
    Serial.begin(115200);
    setupCamComm();
    Serial.println("[CAM DEBUG] ready — printing camera data");
}

void loop() {
    if (readCam()) {
        debugCamReadings();
    }
}
