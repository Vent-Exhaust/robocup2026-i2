#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "serial_comm.h"

// ── Fill in after first flash ─────────────────────────────────────────────
// Open Serial Monitor on each board to get its MAC, then enter both below.
// Flash both boards again with the same code — they auto-detect their role.
static const uint8_t PEER_A_MAC[6] = {0x80, 0xF1, 0xB2, 0x64, 0x5E, 0x7C};
static const uint8_t PEER_B_MAC[6] = {0x80, 0xF1, 0xB2, 0x64, 0x5A, 0x2C};
// ─────────────────────────────────────────────────────────────────────────

static const uint32_t TX_INTERVAL_MS = 200;

struct Packet {
    uint32_t id;
    uint32_t timestamp_ms;
};

// L3 → L2 UART (Serial1 with custom pins)
#define L3_TO_L2_SERIAL Serial1
#define TX_L2 D6
#define RX_L2 D7

// Mux Pin Declarations
#define MUX_1 D8
#define MUX_2 D1
#define S0 D2
#define S1 D3
#define S2 D4
#define S3 D5

// IR Readings
#define IR_COUNT 28
#define IR_THRESHOLD 1
#define IR_SAMPLES 250
#define IR_TRIM 1  // sensors to trim from each edge of a cluster
int IR[IR_COUNT] = {};

struct BallData {
    float angle;      // degrees: 0=front, 90=right, 180=back, 270=left
    int activeCount;  // number of sensors detecting the ball
    bool detected;
};

// Switch Readings (mux channels 14–15, beyond the 0–13 used by IR)
// GOAL:    MUX_1 ch14 | ROLE:    MUX_1 ch15 (!note: silkscreen writes side)
// STRAT_0: MUX_2 ch14 | STRAT_1: MUX_2 ch15
bool switchGoal = false;
bool switchRole = false;
bool switchStrat0 = false;
bool switchStrat1 = false;

#endif // MAIN_H
