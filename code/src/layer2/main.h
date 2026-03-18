#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Servo.h>
#include "serial_comm.h"

// Motor Initialisation

// Motor 1
#define M1_IN_A 0
#define M1_PWM 1
#define M1_IN_B 2

// Motor 2
#define M2_IN_A 3
#define M2_PWM 4
#define M2_IN_B 5

// Motor 3
#define M3_IN_A 6
#define M3_PWM 7
#define M3_IN_B 8

// Motor 4
#define M4_IN_A 9
#define M4_PWM 10
#define M4_IN_B 11

// Dribbler
Servo dribbler;
#define DRIBBLER_PWM 12


// Lightgate
#define LIGHTGATE 22

// Solenoid
#define SOL 13

// L1 - L2 Serial
#define L1_TO_L2_SERIAL Serial4

// L3-Esp32c3 UART
#define RX_L3 21
#define TX_L3 20

// Cam UART
#define RX_CAM 15
#define TX_CAM 14

// IMU I2C
#define SCL_IMU 19
#define SDA_IMU 18

// Function Prototypes
void setupMotors();
void setupSol();
void setupLightgate();
void setupESC();

#endif // MAIN_H