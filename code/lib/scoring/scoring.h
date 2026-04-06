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

// Kick cooldown (ms) — lightgate reads false positive after solenoid fires
#define KICK_COOLDOWN_MS 1000

// Function prototypes
void setupESC();
void setupSol();
void kickSol();
void setupLightgate();
void spinDribbler(int speed);
bool checkCatchment();

#endif // SCORING_H
