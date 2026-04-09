#ifndef CAM_COMM_H
#define CAM_COMM_H

#include <Arduino.h>

#define CAM_SERIAL Serial3

// Blue goal
extern bool  camBlueDetected;
extern float camBlueX, camBlueY, camBlueDist, camBlueAngle;
extern float camBlueLBDist, camBlueLBAngle;  // largest blob (scoring aim)

// Yellow goal
extern bool  camYellowDetected;
extern float camYellowX, camYellowY, camYellowDist, camYellowAngle;
extern float camYellowLBDist, camYellowLBAngle;  // largest blob (scoring aim)

// Ball
extern bool  camBallDetected;
extern float camBallX, camBallY, camBallDist, camBallAngle;

void setupCamComm();
bool readCam();         // returns true when a new line was parsed
void debugCamReadings();

#endif // CAM_COMM_H
