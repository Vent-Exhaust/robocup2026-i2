// #include "main.h"

// static uint8_t myMac[6];
// static uint8_t peerMac[6];

// static uint32_t txCount    = 0;
// static uint32_t rxCount    = 0;
// static uint32_t lastTxMs   = 0;
// static uint32_t lastPrintMs = 0;
// static const char *myRole  = nullptr;

// // ── Callbacks run on the WiFi task (core 0) — independent of loop() ───────

// void onSent(const uint8_t *mac, esp_now_send_status_t status) {
//     Serial.printf("[TX] id=%-4lu  %s\n", txCount,
//                   status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
// }

// void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
//     if (len < (int)sizeof(Packet)) return;
//     Packet pkt;
//     memcpy(&pkt, data, sizeof(pkt));
//     rxCount++;
//     uint32_t latency = millis() - pkt.timestamp_ms;
//     Serial.printf("[RX] id=%-4lu  latency=%lums  total_rx=%lu\n",
//                   pkt.id, latency, rxCount);
// }

// // ─────────────────────────────────────────────────────────────────────────

// void setup() {
//     Serial.begin(115200);
//     delay(500);

//     WiFi.mode(WIFI_STA);
//     WiFi.macAddress(myMac);

//     Serial.printf("\nMAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
//                   myMac[0], myMac[1], myMac[2],
//                   myMac[3], myMac[4], myMac[5]);

//     if (memcmp(myMac, PEER_A_MAC, 6) == 0) {
//         memcpy(peerMac, PEER_B_MAC, 6);
//         myRole = "PEER A";
//         Serial.println("Role: PEER A");
//     } else if (memcmp(myMac, PEER_B_MAC, 6) == 0) {
//         memcpy(peerMac, PEER_A_MAC, 6);
//         myRole = "PEER B";
//         Serial.println("Role: PEER B");
//     } else {
//         Serial.println("WARNING: MAC not recognised — fill in PEER_A/B_MAC in main.h");
//         while (true) {
//             Serial.printf("MY MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
//                           myMac[0], myMac[1], myMac[2],
//                           myMac[3], myMac[4], myMac[5]);
//             delay(1000);
//         }
//     }

//     if (esp_now_init() != ESP_OK) {
//         Serial.println("ESP-NOW init failed");
//         while (true) delay(1000);
//     }

//     esp_now_register_send_cb(onSent);
//     esp_now_register_recv_cb(onRecv);

//     esp_now_peer_info_t peer = {};
//     memcpy(peer.peer_addr, peerMac, 6);
//     peer.channel = 0;
//     peer.encrypt = false;
//     if (esp_now_add_peer(&peer) != ESP_OK) {
//         Serial.println("Add peer failed");
//         while (true) delay(1000);
//     }

//     Serial.println("ESP-NOW ready.\n");
// }

// void loop() {
//     uint32_t now = millis();

//     // Non-blocking TX — fires every TX_INTERVAL_MS
//     // RX always works regardless, since onRecv() runs on a separate task
//     if (now - lastTxMs >= TX_INTERVAL_MS) {
//         lastTxMs = now;
//         Packet pkt = {++txCount, now};
//         esp_now_send(peerMac, (uint8_t *)&pkt, sizeof(pkt));
//     }

//     if (now - lastPrintMs >= 1000) {
//         lastPrintMs = now;
//         Serial.printf("I am %s  |  time=%lums\n", myRole, now);
//     }

//     // ── Simulate 1-2s blocking work ───────────────────────────────────────
//     delay(2000);
//     //
//     // RX callbacks still fire during this block (WiFi task, core 0).
//     // TX will resume after the block; millis() timer catches up correctly.
//     // ─────────────────────────────────────────────────────────────────────
// }

#include "main.h"

static SerialComm l3Comm(L3_TO_L2_SERIAL);

void setupL3Comm() {
    L3_TO_L2_SERIAL.begin(115200, SERIAL_8N1, RX_L2, TX_L2);
}

void sendL3Data(BallData &ball) {
    int16_t tx[7] = {
        (int16_t)ball.detected,
        (int16_t)(ball.angle * 10),
        (int16_t)ball.activeCount,
        (int16_t)switchGoal,
        (int16_t)switchRole,
        (int16_t)switchStrat0,
        (int16_t)switchStrat1
    };
    l3Comm.write(tx, 7);
}

void selectMuxChannel(int n) {
    int binaryNum[4] = {0, 0, 0, 0};
    int tempN = n;
    int i = 0;

    while (tempN > 0 && i < 4) {
        binaryNum[i] = tempN % 2;
        tempN = tempN / 2;
        i++;
    }

    // S0 = LSB, S3 = MSB
    digitalWrite(S0, binaryNum[0]);
    digitalWrite(S1, binaryNum[1]);
    digitalWrite(S2, binaryNum[2]);
    digitalWrite(S3, binaryNum[3]);
}

bool readIR(int pin) {
    int lowCount = 0;
    for (int i = 0; i < IR_SAMPLES; i++) {
        if (digitalRead(pin) == LOW) {
            lowCount++;
        }
        delayMicroseconds(2);
    }
    return lowCount > IR_THRESHOLD;
}

void readIRs() {
    for (int i = 0; i < 14; i++) {
        selectMuxChannel(i);
        IR[i] = readIR(MUX_1);
        IR[i+14] = readIR(MUX_2);
    }
}

BallData calculateBall() {
    // Find the largest consecutive cluster of active sensors (wrapping around)
    int bestStart = -1, bestLen = 0;
    int curStart = -1, curLen = 0;

    // Double the loop to handle wraparound clusters
    for (int i = 0; i < IR_COUNT * 2; i++) {
        if (IR[i % IR_COUNT]) {
            if (curLen == 0) curStart = i;
            curLen++;
            if (curLen > bestLen) {
                bestLen = curLen;
                bestStart = curStart;
            }
        } else {
            curLen = 0;
        }
    }
    // Cap length to IR_COUNT (all sensors active edge case)
    if (bestLen > IR_COUNT) bestLen = IR_COUNT;

    // Trim noisy edge sensors if cluster is large enough, otherwise use all
    int useStart = bestStart;
    int useLen = bestLen;
    if (bestLen > 2 * IR_TRIM) {
        useStart = bestStart + IR_TRIM;
        useLen = bestLen - 2 * IR_TRIM;
    }

    BallData ball;
    ball.activeCount = bestLen;
    ball.detected = bestLen > 0;

    if (ball.detected) {
        float sumX = 0, sumY = 0;
        for (int j = 0; j < useLen; j++) {
            int idx = (useStart + j) % IR_COUNT;
            float angleDeg = fmod(270.0f - idx * (360.0f / IR_COUNT) + 360.0f, 360.0f);
            float angleRad = angleDeg * DEG_TO_RAD;
            sumX += sinf(angleRad);
            sumY += cosf(angleRad);
        }
        float angleRad = atan2f(sumX, sumY);
        ball.angle = angleRad * RAD_TO_DEG;
        if (ball.angle < 0) ball.angle += 360.0f;
    } else {
        ball.angle = 0;
    }

    return ball;
}

void debugIR() {
    for (int ir : IR) {
        Serial.print(ir);
        Serial.print(" ");
    }
    Serial.println();
}

void debugBall(BallData &ball) {
    if (ball.detected) {
        Serial.printf("[BALL] angle=%.1f  sensors=%d\n",
                      ball.angle, ball.activeCount);
    } else {
        Serial.println("[BALL] not detected");
    }
}

void readSwitches() {
    selectMuxChannel(14);
    switchGoal   = digitalRead(MUX_1);
    switchStrat0 = digitalRead(MUX_2);

    selectMuxChannel(15);
    switchRole   = digitalRead(MUX_1);
    switchStrat1 = digitalRead(MUX_2);
}

void setupMux() {
    pinMode(MUX_1, INPUT);
    pinMode(MUX_2, INPUT);
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
}

void setup() {
    Serial.begin(115200);
    setupMux();
    setupL3Comm();
}

void loop() {
    readIRs();
    readSwitches();
    BallData ball = calculateBall();
    sendL3Data(ball);
    Serial.print(ball.angle);
    Serial.print(" ");
    debugIR();
    // debugBall(ball);
    // Serial.printf("[SW] goal=%d role=%d strat0=%d strat1=%d\n",
    //               switchGoal, switchRole, switchStrat0, switchStrat1);
}