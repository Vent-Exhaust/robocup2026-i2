#ifndef LAYER3_COMM_H
#define LAYER3_COMM_H

#include <Arduino.h>
#include "serial_comm.h"

#define L3_TO_L2_SERIAL Serial5

// Ball (from IR ring)
extern bool  l3BallDetected;
extern float l3BallAngle;
extern int   l3BallCount;

// Switches
extern bool l3SwitchGoal;
extern bool l3SwitchRole;
extern bool l3SwitchStrat0;
extern bool l3SwitchStrat1;

void setupL3Comm();
bool readL3();          // returns true when a new packet was parsed
void debugL3Readings();

#endif // LAYER3_COMM_H
