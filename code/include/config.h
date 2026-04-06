#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// BOT SELECTION — change to 1 or 2 before uploading
// =============================================================================
#define BOT 2

// =============================================================================
// COMPETITION TUNING — edit these values on game day
// =============================================================================

// --- Light Ring ---
// Per-sensor thresholds (0–4095). Run calibrateLightRing() to generate these.
#if BOT == 1
constexpr int LDR_THRESHOLDS[32] = {3415, 3409, 3393, 3384, 3372, 3365, 3326, 3339, 3323, 3311, 3303, 3307, 3294, 3284, 3276, 3277, 3272, 3254, 3251, 3258, 3243, 3242, 3235, 3240, 3236, 3222, 3217, 3227, 3220, 3227, 3217, 3230};
#elif BOT == 2
constexpr int LDR_THRESHOLDS[32] = {3403, 3370, 3346, 3357, 3365, 3340, 3351, 3325, 3313, 3293, 3308, 3299, 3278, 3280, 3273, 3271, 3262, 3255, 3250, 3221, 3249, 3241, 3204, 3227, 3233, 3229, 3225, 3211, 3209, 3196, 3201, 3219};
#endif

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
#if BOT == 1
constexpr double FR_TRIM = 1.0;  // M1
constexpr double BR_TRIM = 1.0;  // M2
constexpr double BL_TRIM = 1.0;  // M3
constexpr double FL_TRIM = 1.0;  // M4
#elif BOT == 2
constexpr double FR_TRIM = 1.0;  // M1
constexpr double BR_TRIM = 1.0;  // M2
constexpr double BL_TRIM = -1.0;  // M3
constexpr double FL_TRIM = 1.0;  // M4
#endif

// --- Layer 2: IMU Correction ---
// Yaw heading-hold PID (output added to omega)
#if BOT == 1
constexpr double YAW_KP             = 0.0025;
constexpr double YAW_KI             = 0.0;
constexpr double YAW_KD             = 0.0;
constexpr double YAW_I_MAX          = 0.35;    // integral windup limit
constexpr double YAW_CORRECTION_MAX = 0.4;     // max omega correction (0–1 scale)
#elif BOT == 2
constexpr double YAW_KP             = 0.0025;
constexpr double YAW_KI             = 0.0;
constexpr double YAW_KD             = 0.0;
constexpr double YAW_I_MAX          = 0.35;
constexpr double YAW_CORRECTION_MAX = 0.4;
#endif

// Accel drift correction — P only (output added to vx/vy)
constexpr double ACCEL_KP             = 0.005;
constexpr double ACCEL_CORRECTION_MAX = 0.2;

// --- IMU Health ---
// If no new IMU reading arrives within this period, consider it disconnected
constexpr unsigned long IMU_TIMEOUT_MS = 200;
// How long to wait after reinit before resuming movement
constexpr unsigned long IMU_REINIT_SETTLE_MS = 500;

// --- Motor Smoothing ---
// EMA alpha for motor power (0.0 = frozen, 1.0 = no smoothing)
// Lower = smoother but more sluggish. 0.15–0.3 is a good range.
constexpr double MOTOR_SMOOTH_ALPHA = 0.3;

// --- Dribbler Speed ---
constexpr int DRIBBLER_SPEED = 12;

// --- Camera Orbit ---
constexpr float CAM_ORBIT_RADIUS      = 20.0f;  // desired distance from ball (cm)
constexpr float CAM_ORBIT_RADIUS_KP   = 1.2f;   // radius maintenance gain (deg per cm error, capped ±30°)
constexpr float CAM_ORBIT_SPEED       = 0.1f;    // max orbit speed
constexpr float CAM_ORBIT_KP          = 0.01f;   // orbit speed gain (speed per deg of alignment error)

// =============================================================================

#endif // CONFIG_H
