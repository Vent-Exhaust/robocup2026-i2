#include "main.h"

static uint8_t myMac[6];
static uint8_t peerMac[6];

static uint32_t txCount    = 0;
static uint32_t rxCount    = 0;
static uint32_t lastTxMs   = 0;
static uint32_t lastPrintMs = 0;
static const char *myRole  = nullptr;

// ── Callbacks run on the WiFi task (core 0) — independent of loop() ───────

void onSent(const uint8_t *mac, esp_now_send_status_t status) {
    Serial.printf("[TX] id=%-4lu  %s\n", txCount,
                  status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void onRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < (int)sizeof(Packet)) return;
    Packet pkt;
    memcpy(&pkt, data, sizeof(pkt));
    rxCount++;
    uint32_t latency = millis() - pkt.timestamp_ms;
    Serial.printf("[RX] id=%-4lu  latency=%lums  total_rx=%lu\n",
                  pkt.id, latency, rxCount);
}

// ─────────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);

    WiFi.mode(WIFI_STA);
    WiFi.macAddress(myMac);

    Serial.printf("\nMAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  myMac[0], myMac[1], myMac[2],
                  myMac[3], myMac[4], myMac[5]);

    if (memcmp(myMac, PEER_A_MAC, 6) == 0) {
        memcpy(peerMac, PEER_B_MAC, 6);
        myRole = "PEER A";
        Serial.println("Role: PEER A");
    } else if (memcmp(myMac, PEER_B_MAC, 6) == 0) {
        memcpy(peerMac, PEER_A_MAC, 6);
        myRole = "PEER B";
        Serial.println("Role: PEER B");
    } else {
        Serial.println("WARNING: MAC not recognised — fill in PEER_A/B_MAC in main.h");
        while (true) {
            Serial.printf("MY MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                          myMac[0], myMac[1], myMac[2],
                          myMac[3], myMac[4], myMac[5]);
            delay(1000);
        }
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while (true) delay(1000);
    }

    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onRecv);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, peerMac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("Add peer failed");
        while (true) delay(1000);
    }

    Serial.println("ESP-NOW ready.\n");
}

void loop() {
    uint32_t now = millis();

    // Non-blocking TX — fires every TX_INTERVAL_MS
    // RX always works regardless, since onRecv() runs on a separate task
    if (now - lastTxMs >= TX_INTERVAL_MS) {
        lastTxMs = now;
        Packet pkt = {++txCount, now};
        esp_now_send(peerMac, (uint8_t *)&pkt, sizeof(pkt));
    }

    if (now - lastPrintMs >= 1000) {
        lastPrintMs = now;
        Serial.printf("I am %s  |  time=%lums\n", myRole, now);
    }

    // ── Simulate 1-2s blocking work ───────────────────────────────────────
    delay(2000);
    //
    // RX callbacks still fire during this block (WiFi task, core 0).
    // TX will resume after the block; millis() timer catches up correctly.
    // ─────────────────────────────────────────────────────────────────────
}
