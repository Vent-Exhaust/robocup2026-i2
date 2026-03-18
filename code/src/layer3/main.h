#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

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

// Mux Pin Declarations
#define MUX_1 D8
#define MUX_2 D1
#define S0 D2
#define S1 D3
#define S2 D4
#define S3 D5

// IR readings
#define IR_THRESHOLD 3
#define IR_SAMPLES 200
int IR[28] = {};

#endif // MAIN_H
