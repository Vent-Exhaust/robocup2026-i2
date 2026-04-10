#ifndef CONFIG_H
#define CONFIG_H

// =============================================================================
// BOT SELECTION — change to 1 or 2 before uploading
// =============================================================================
#define BOT 1

// =============================================================================
// LOCATION SELECTION — change per venue (affects min speeds, thresholds, etc.)
// 1 = Home    2 = Competition
// =============================================================================
#define LOCATION 2

// --- Strategy ---
// Which goal to attack: 0 = blue, 1 = yellow
#define ATTACK_GOAL 1

// Robot role: 0 = striker, 1 = goalie
#define ROLE 1

// =============================================================================
// COMPETITION TUNING — edit these values on game day
// =============================================================================

// --- Light Ring ---
// Per-sensor thresholds (0–4095). Run calibrateLightRing() to generate these.
#if BOT == 1
// constexpr int LDR_THRESHOLDS[32] = {3415, 3409, 3393, 3384, 3372, 3365, 3326, 3339, 3323, 3311, 3303, 3307, 3294, 3284, 3276, 3277, 3272, 3254, 3251, 3258, 3243, 3242, 3235, 3240, 3236, 3222, 3217, 3227, 3220, 3227, 3217, 3230};
constexpr int LDR_THRESHOLDS[32] = {3418, 3411, 3396, 3390, 3376, 3370, 3333, 3346, 3332, 3319, 3307, 3308, 3300, 3294, 3284, 3284, 3281, 3264, 3257, 3259, 3077, 3244, 3240, 3241, 3238, 3228, 3212, 3212, 3217, 3234, 3225, 3236};
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
constexpr int DRIBBLER_SPEED = 20;

// --- Location Speed Multiplier ---
// Percentage increase applied to all min speeds (0 = no change, 50 = 50% faster)
#if LOCATION == 1
constexpr float LOCATION_SPEED_PCT = 0.0f;
#elif LOCATION == 2
constexpr float LOCATION_SPEED_PCT = -10.0f;
#endif
constexpr float LOCATION_SPEED_MULT = 1.0f + LOCATION_SPEED_PCT / 100.0f;

// --- Camera Orbit ---
constexpr float CAM_ORBIT_RADIUS      = 20.0f;  // desired distance from ball (cm)
constexpr float CAM_ORBIT_RADIUS_KP   = 1.2f;   // radius maintenance gain (deg per cm error, capped ±30°)
constexpr float CAM_ORBIT_SPEED       = 0.15f;   // max orbit speed
constexpr float CAM_ORBIT_KP          = 0.02f;   // orbit speed gain (speed per deg of alignment error)
constexpr float CAM_ORBIT_KD          = 0.005f;   // orbit derivative gain (damps oscillation near alignment)
constexpr float CAM_ORBIT_KI          = 0.0005f;  // orbit integral gain (ramps up when alignment error persists)
constexpr float CAM_ORBIT_I_MAX       = 0.05f;    // integral windup limit
constexpr float CAM_ORBIT_DEADZONE    = 20.0f;    // alignment error deadzone (deg) — no orbit when aligned within this

// --- IR Chase ---
constexpr float IR_CHASE_SPEED        = 0.18f;   // speed when chasing ball (cam can't see = far away)

// --- IR Orbit ---
constexpr float IR_ORBIT_OFFSET       = 70.0f;   // max perpendicular offset (deg) — larger = wider orbit
constexpr float IR_ORBIT_FADE_DEG     = 45.0f;   // ball angle at which offset reaches full strength
constexpr float IR_ORBIT_SPEED_MAX    = 0.15f;   // speed when aligned (ball ahead)
constexpr float IR_ORBIT_SPEED_MIN    = 0.3f;    // speed when orbiting sideways
constexpr float XDRIVE_DEAD_NUDGE    = 10.0f;   // degrees to nudge away from X-drive dead angles (±45°, ±135°)

// --- Line Avoidance ---
constexpr float LINE_PUSH_SPEED      = 0.2f;   // speed to push away from detected line

// --- Scoring ---
constexpr float SCORE_DIST_THRESH     = 70.0f;    // goal distance (cm) to trigger scoring when ball caught

// --- Goalie ---
// Curve shape: y = CURVE_A * x^6 + CURVE_Y0  (cm, field coords)
constexpr float GOALIE_CURVE_A         = -2.0e-11f; // x^6 coeff — curve dips at edges (2x wider)
constexpr float GOALIE_CURVE_Y0        = -55.0f;    // baseline y at center (cm from field center)
constexpr float GOALIE_STEP_SIZE       = 20.0f;     // cm per step along curve
// Speed PD
constexpr float GOALIE_SPEED_KP        = 0.001f;  // P gain for speed from error
constexpr float GOALIE_SPEED_KD        = 0.00002f;// D gain for speed
constexpr float GOALIE_SPEED_MIN       = 0.20f;   // min move speed
constexpr float GOALIE_SPEED_MAX       = 0.35f;   // max move speed
// Ball angle → target X mapping
constexpr float GOALIE_BALL_X_SCALE    = 120.0f;  // max target X offset from ball angle (cm)
// Edge speed scaling (exponential, from 2025)
constexpr float GOALIE_EDGE_A          = 3.4f;
constexpr float GOALIE_EDGE_B          = 0.07f;   // scaled for cm (was 0.007 in mm)
constexpr float GOALIE_EDGE_C          = 0.0f;
constexpr float GOALIE_EDGE_D          = 0.2f;
// Clamp
constexpr float GOALIE_X_MAX           = 100.0f;  // max X range on curve (cm)
constexpr float GOALIE_Y_MIN           = -100.0f; // don't drive past this Y (cm)

// =============================================================================

#endif // CONFIG_H
