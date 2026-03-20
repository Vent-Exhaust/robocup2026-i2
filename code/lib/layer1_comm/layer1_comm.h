#ifndef LAYER1_COMM_H
#define LAYER1_COMM_H

#include <Arduino.h>
#include "serial_comm.h"

#define L1_TO_L2_SERIAL Serial4

extern bool  l1LineDetected;
extern float l1Angle;
extern float l1Size;

void setupL1Comm();
bool readL1();       // returns true when new data arrived
void debugL1Readings();

#endif // LAYER1_COMM_H
