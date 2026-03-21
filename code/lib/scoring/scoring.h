#ifndef SCORING_H
#define SCORING_H

#include <Arduino.h>
#include <Servo.h>

// Dribbler
#define DRIBBLER_PWM 12
extern Servo dribbler;

// Kicker (solenoid)
#define SOL 13
extern bool isActuated;

// Lightgate
#define LIGHTGATE 22

// Function prototypes
void setupESC();
void setupSol();
void setupLightgate();
void spinDribbler(int speed);

#endif // SCORING_H
