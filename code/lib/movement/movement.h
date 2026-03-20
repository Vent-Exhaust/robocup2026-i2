#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "config.h"

struct PID {
    double kp, ki, kd, iMax;
    double integral  = 0;
    double prevMeasurement = 0;
    unsigned long lastUs = 0;

    double compute(double error, double measurement) {
        unsigned long now = micros();
        double dt = (lastUs == 0) ? 0.01 : (now - lastUs) * 1e-6;
        lastUs = now;
        if (dt > 0.5) dt = 0.01;
        integral  = constrain(integral + error * dt, -iMax, iMax);
        // Derivative on measurement avoids spikes when the robot is
        // repositioned or the target changes suddenly.
        double dMeas = measurement - prevMeasurement;
        while (dMeas >  180.0) dMeas -= 360.0;
        while (dMeas < -180.0) dMeas += 360.0;
        double d  = -dMeas / dt;   // negative: increasing measurement = decreasing error
        prevMeasurement = measurement;
        return kp * error + ki * integral + kd * d;
    }

    void reset() { integral = 0; prevMeasurement = 0; lastUs = 0; }
};

// Motor pins
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

// IMU I2C
#define SCL_IMU 19
#define SDA_IMU 18

// IMU state (updated by readIMU)
extern float imuYaw;
extern float imuAccelX, imuAccelY, imuAccelZ;

// Function prototypes
void setupMotors();
void setupIMU();
void readIMU();
void debugIMU();
void resetYawTarget();
void setMotor(int inA, int inB, int pwmPin, double power);
void moveRobot(double angleDeg, double speed, double omega);

#endif // MOVEMENT_H
