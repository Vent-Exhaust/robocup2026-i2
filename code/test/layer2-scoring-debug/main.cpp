#include <Arduino.h>
#include "scoring.h"
#include "cam_comm.h"
#include "movement.h"
#include "config.h"

// --- Scoring tuning (mirror these in main layer2 code) ---
static const float SCORE_ROT_KP  = 0.002f;
static const float SCORE_ROT_MAX = 0.15f;
static const float SCORE_SPEED   = 0.2f;
static const float KICK_DIST     = 50.0f;
static const float KICK_ANGLE    = 20.0f;

enum Mode { M_MENU, M_LIGHTGATE, M_DRIBBLER, M_SOLENOID, M_CAM_GOAL, M_SCORE_RUN };
static Mode mode = M_MENU;

static void printMenu() {
    Serial.println();
    Serial.println("=== Scoring Debug ===");
    Serial.println("1 — Lightgate (catchment) live reading");
    Serial.println("2 — Dribbler speed sweep");
    Serial.println("3 — Solenoid kick test");
    Serial.println("4 — Camera goal tracking (read-only)");
    Serial.println("5 — Full score run (rotate + drive + kick)");
    Serial.println("0 — Stop / back to menu");
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    setupESC();
    setupSol();
    setupLightgate();
    setupMotors();
    setupIMU();
    setupCamComm();

    delay(500);
    resetYawTarget();
    printMenu();
}

void loop() {
    // Check for serial commands
    if (Serial.available()) {
        char c = Serial.read();
        // Flush remaining chars on the line
        while (Serial.available() && Serial.peek() != '\n') Serial.read();

        if (c == '0') {
            stopMotors();
            spinDribbler(0);
            mode = M_MENU;
            printMenu();
            return;
        }

        switch (c) {
            case '1': mode = M_LIGHTGATE;  Serial.println(">> Lightgate monitor (send 0 to stop)"); break;
            case '2': mode = M_DRIBBLER;   Serial.println(">> Dribbler sweep (send 0 to stop)");    break;
            case '3': mode = M_SOLENOID;   break;
            case '4': mode = M_CAM_GOAL;   Serial.println(">> Camera goal tracking (send 0 to stop)"); break;
            case '5': mode = M_SCORE_RUN;  Serial.println(">> Score run active (send 0 to stop)");  break;
            default: break;
        }
    }

    readIMU();
    readCam();

    switch (mode) {

    case M_LIGHTGATE: {
        int raw = analogRead(LIGHTGATE);
        bool caught = checkCatchment();
        Serial.printf("[LG] raw=%d  caught=%s\n", raw, caught ? "YES" : "no");
        delay(100);
        break;
    }

    case M_DRIBBLER: {
        // Ramp 0→30→0 so you can observe at each speed
        static int dribbSpeed = 0;
        static int dir = 1;
        static unsigned long lastStep = 0;

        if (millis() - lastStep > 1000) {
            lastStep = millis();
            dribbSpeed += dir * 2;
            if (dribbSpeed >= 30) dir = -1;
            if (dribbSpeed <= 0)  { dir = 1; dribbSpeed = 0; }
            spinDribbler(dribbSpeed);
            bool caught = checkCatchment();
            Serial.printf("[DRIBB] speed=%d  caught=%s\n", dribbSpeed, caught ? "YES" : "no");
        }
        break;
    }

    case M_SOLENOID: {
        Serial.println("[SOL] Firing in 1s...");
        delay(1000);
        kickSol();
        Serial.println("[SOL] Fired!");
        mode = M_MENU;
        printMenu();
        break;
    }

    case M_CAM_GOAL: {
        if (camBlueDetected)
            Serial.printf("[CAM] BLUE  angle=%.1f  dist=%.1f\n", camBlueAngle, camBlueDist);
        if (camYellowDetected)
            Serial.printf("[CAM] YELLOW angle=%.1f  dist=%.1f\n", camYellowAngle, camYellowDist);
        if (!camBlueDetected && !camYellowDetected)
            Serial.println("[CAM] no goal detected");

        bool caught = checkCatchment();
        Serial.printf("[LG] caught=%s\n", caught ? "YES" : "no");
        delay(200);
        break;
    }

    case M_SCORE_RUN: {
        spinDribbler(DRIBBLER_SPEED);
        bool caught = checkCatchment();

        // Target: blue goal (swap to camYellow* to change)
        bool targetDetected = camBlueDetected;
        float targetAngle   = camBlueAngle;
        float targetDist    = camBlueDist;

        if (!caught) {
            stopMotors();
            Serial.println("[SCORE] waiting for ball...");
            delay(200);
            break;
        }

        if (targetDetected) {
            float goalError = targetAngle;
            if (goalError > 180.0f) goalError -= 360.0f;

            float omega = constrain(goalError * SCORE_ROT_KP, -SCORE_ROT_MAX, SCORE_ROT_MAX);
            if (fabsf(goalError) < 3.0f) omega = 0;

            if (targetDist < KICK_DIST && fabsf(goalError) < KICK_ANGLE) {
                Serial.println("[SCORE] KICKING!");
                kickSol();
                stopMotors();
                mode = M_MENU;
                printMenu();
                break;
            }

            moveRobot(0, SCORE_SPEED, omega);
            Serial.printf("[SCORE] err=%.1f dist=%.1f omega=%.3f\n", goalError, targetDist, omega);
        } else {
            moveRobot(0, 0.15f, 0);
            Serial.println("[SCORE] goal not visible, creeping");
        }
        delay(50);
        break;
    }

    case M_MENU:
    default:
        break;
    }
}
