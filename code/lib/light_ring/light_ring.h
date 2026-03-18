#ifndef LIGHT_RING_H
#define LIGHT_RING_H

#include <Arduino.h>
#include <utility>
#include "config.h"

// Multiplexer pin definitions
#define M1 19
#define M2 18
#define S0 14
#define S1 15
#define S2 16
#define S3 17

// Raw readings from both muxes (32 sensors)
extern int ldr_values[32];
extern bool ldr_threshold_pass[32];

void selectMuxChannel(int n);
void checkLightRing();

// Returns {angle, size} — both NAN if no line found
std::pair<double, double> findLine();

void debugLDRValues();

#endif // LIGHT_RING_H
