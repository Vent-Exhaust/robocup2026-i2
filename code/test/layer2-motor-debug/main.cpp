#include <Arduino.h>
#include "movement.h"

static const double TEST_SPEED = 0.3;
static const unsigned long RUN_TIME_MS = 2000;
static const unsigned long PAUSE_TIME_MS = 1000;

struct Motor {
    const char *name;
    int inA, inB, pwm;
};

static const Motor motors[] = {
    { "M1 - Front Right (FR)", FR_IN_A, FR_IN_B, FR_PWM },
    { "M2 - Back Right  (BR)", BR_IN_A, BR_IN_B, BR_PWM },
    { "M3 - Back Left   (BL)", BL_IN_A, BL_IN_B, BL_PWM },
    { "M4 - Front Left  (FL)", FL_IN_A, FL_IN_B, FL_PWM },
};
static const int NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

static int currentMotor = 0;
static unsigned long stateStartMs = 0;
static bool running = false;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    setupMotors();
    stopMotors();

    Serial.println("=== Motor Debug ===");
    Serial.println("Cycles through each motor one by one.");
    Serial.printf("Speed: %.0f%%  |  Run: %lums  |  Pause: %lums\n",
                  TEST_SPEED * 100, RUN_TIME_MS, PAUSE_TIME_MS);
    Serial.println();

    currentMotor = 0;
    running = true;
    stateStartMs = millis();
    Serial.printf(">>> Running: %s\n", motors[currentMotor].name);
    setMotor(motors[currentMotor].inA, motors[currentMotor].inB,
             motors[currentMotor].pwm, TEST_SPEED);
}

void loop() {
    unsigned long elapsed = millis() - stateStartMs;

    if (running) {
        if (elapsed >= RUN_TIME_MS) {
            stopMotors();
            Serial.printf("    Stopped: %s\n", motors[currentMotor].name);
            running = false;
            stateStartMs = millis();
        }
    } else {
        if (elapsed >= PAUSE_TIME_MS) {
            currentMotor = (currentMotor + 1) % NUM_MOTORS;
            if (currentMotor == 0) {
                Serial.println("\n--- Cycle complete, restarting ---\n");
            }
            Serial.printf(">>> Running: %s\n", motors[currentMotor].name);
            setMotor(motors[currentMotor].inA, motors[currentMotor].inB,
                     motors[currentMotor].pwm, TEST_SPEED);
            running = true;
            stateStartMs = millis();
        }
    }
}
