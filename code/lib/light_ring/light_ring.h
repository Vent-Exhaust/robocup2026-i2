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

struct LineResult {
    double angle;     // escape direction (away from line), NAN if no line
    double size;      // normalised span 0–1
    int    startLdr;  // index of first triggered sensor (-1 if no line)
    int    endLdr;    // index of last triggered sensor  (-1 if no line)
};

// Returns line data — angle/size are NAN and startLdr/endLdr are -1 if no line found
LineResult findLine();

void debugLDRValues();

#endif // LIGHT_RING_H
