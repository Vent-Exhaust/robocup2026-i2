#ifndef LAYER1_COMM_H
#define LAYER1_COMM_H

#include <Arduino.h>
#include "serial_comm.h"

#define L1_TO_L2_SERIAL Serial4

extern bool  l1LineDetected;
extern float l1Angle;
extern float l1Size;
extern int   l1StartLdr;   // index of first triggered sensor (0–31), -1 if no line
extern int   l1EndLdr;     // index of last triggered sensor  (0–31), -1 if no line

void setupL1Comm();
bool readL1();       // returns true when new data arrived
void debugL1Readings();

#endif // LAYER1_COMM_H
