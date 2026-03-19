#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_BNO08x.h>
#include "serial_comm.h"
#include "config.h"

struct PID {
    double kp, ki, kd, iMax;
    double integral  = 0;
    double prevError = 0;
    unsigned long lastUs = 0;

    double compute(double error) {
        unsigned long now = micros();
        double dt = (lastUs == 0) ? 0.01 : (now - lastUs) * 1e-6;
        lastUs = now;
        if (dt > 0.5) dt = 0.01;
        integral  = constrain(integral + error * dt, -iMax, iMax);
        double d  = (error - prevError) / dt;
        prevError = error;
        return kp * error + ki * integral + kd * d;
    }

    void reset() { integral = 0; prevError = 0; lastUs = 0; }
};

// Motor Initialisation

// Front Right (M1, 45°)
#define FR_IN_A 0
#define FR_PWM  1
#define FR_IN_B 2

// Back Right (M2, 135°)
#define BR_IN_A 3
#define BR_PWM  4
#define BR_IN_B 5

// Back Left (M3, 225°)
#define BL_IN_A 6
#define BL_PWM  7
#define BL_IN_B 8

// Front Left (M4, 315°)
#define FL_IN_A 9
#define FL_PWM  10
#define FL_IN_B 11

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
void setupIMU();
void readIMU();
void debugIMU();
void resetYawTarget();
void setMotor(int inA, int inB, int pwmPin, double power);
void moveRobot(double angleDeg, double speed, double omega);

#endif // MAIN_H