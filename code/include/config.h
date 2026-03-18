#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// COMPETITION TUNING — edit these values on game day
// =============================================================================

// --- Light Ring ---
// Per-sensor thresholds (0–4095). Run calibrateLightRing() to generate these.

// PSU 3v3 (Lab)
// constexpr int LDR_THRESHOLDS[32] = {2590, 2673, 2634, 2713, 2647, 2759, 2220, 2772, 2286, 2271, 2096, 2570, 2309, 2538, 2638, 2678, 2560, 2300, 2192, 2230, 1756, 2153, 2062, 2273, 2518, 1396, 2031, 2209, 2236, 2337, 2250, 2472};

// PSU 12v (Lab)
constexpr int LDR_THRESHOLDS[32] = {3391, 3388, 3372, 3367, 3354, 3350, 3307, 3325, 3307, 3253, 3300, 3291, 3265, 3300, 3300, 3264, 3257, 3330, 3192, 3238, 3219, 3224, 3180, 3221, 3217, 3200, 3175, 3201, 3198, 3206, 3187, 3208};

// Samples to average per sensor reading
constexpr int LDR_SAMPLES = 25;

// Microseconds to wait after switching mux channel before reading
constexpr int MUX_SETTLE_US = 50;

// --- Calibration ---
// How many checkLightRing() calls to make per Enter press
constexpr int CALIBRATION_READS_PER_SAMPLE = 50;

// --- Main Loop ---
// Polling period in milliseconds
constexpr int LOOP_DELAY_MS = 50;

// =============================================================================

#endif // CONFIG_H
