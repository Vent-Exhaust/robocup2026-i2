#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// COMPETITION TUNING — edit these values on game day
// =============================================================================

// --- Light Ring ---
// ADC value (0–4095) above which a sensor is considered "on the line"
constexpr int LDR_THRESHOLD = 1900;

// Microseconds to wait after switching mux channel before reading
constexpr int MUX_SETTLE_US = 50;

// --- Main Loop ---
// Polling period in milliseconds
constexpr int LOOP_DELAY_MS = 50;

// =============================================================================

#endif // CONFIG_H
