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
// constexpr int LDR_THRESHOLDS[32] = {3391, 3388, 3372, 3367, 3354, 3350, 3307, 3325, 3307, 3253, 3300, 3291, 3265, 3300, 3300, 3264, 3257, 3330, 3192, 3238, 3219, 3224, 3180, 3221, 3217, 3200, 3175, 3201, 3198, 3206, 3187, 3208};

// Lipo (Cx home)
constexpr int LDR_THRESHOLDS[32] = {3415, 3409, 3393, 3384, 3372, 3365, 3326, 3339, 3323, 3311, 3303, 3307, 3294, 3284, 3276, 3277, 3272, 3254, 3251, 3258, 3243, 3242, 3235, 3240, 3236, 3222, 3217, 3227, 3220, 3227, 3217, 3230};

// Samples to average per sensor reading
constexpr int LDR_SAMPLES = 25;

// Microseconds to wait after switching mux channel before reading
constexpr int MUX_SETTLE_US = 50;

// --- Calibration ---
// How many checkLightRing() calls to make per Enter press
constexpr int CALIBRATION_READS_PER_SAMPLE = 50;

// --- Main Loop ---
// Polling period in milliseconds
constexpr int LOOP_DELAY_MS = 30;

// --- Layer 2: Motor Trim ---
// Per-motor speed multipliers (0.0–1.0). Reduce a motor's value if it spins
// faster than the others. The slowest motor should stay at 1.0.
constexpr double FR_TRIM = 1.0;  // M1
constexpr double BR_TRIM = 1.0;   // M2
constexpr double BL_TRIM = 1.0;  // M3
constexpr double FL_TRIM = 1.0;   // M4

// --- Layer 2: IMU Correction ---
// Yaw heading-hold PID (output added to omega)
constexpr double YAW_KP             = 0.008;
constexpr double YAW_KI             = 0.0;
constexpr double YAW_KD             = 0.0021;
constexpr double YAW_I_MAX          = 0.35;    // integral windup limit
constexpr double YAW_CORRECTION_MAX = 1.0;    // max omega correction (0–1 scale)

// Accel drift correction — P only (output added to vx/vy)
constexpr double ACCEL_KP             = 0.005;
constexpr double ACCEL_CORRECTION_MAX = 0.2;

// --- IMU Health ---
// If no new IMU reading arrives within this period, consider it disconnected
constexpr unsigned long IMU_TIMEOUT_MS = 200;
// How long to wait after reinit before resuming movement
constexpr unsigned long IMU_REINIT_SETTLE_MS = 500;

// =============================================================================

#endif // CONFIG_H
