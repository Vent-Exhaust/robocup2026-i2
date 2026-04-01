# CLAUDE.md

## Project

RoboCup 2026 soccer robot firmware — 3-layer architecture on PlatformIO + Arduino.

## Build

- PlatformIO project. Build with `pio run -e <env>`.
- Production environments: `layer1-teensy40`, `layer2-teensy40`, `layer3-seeed_xiao_esp32c3`
- Debug/test environments in `test/` directory (e.g. `layer1-calibrate`, `layer2-imu-debug`)
- All serial at 115200 baud

## Architecture

Three processors communicate over UART using a custom SerialComm packet protocol (start byte 0xAA, big-endian int16_t values, XOR checksum):

- **Layer 1** (Teensy 4.0): 32-LDR ring for white line detection. Sends line angle & width to L2 via Serial1→Serial4.
- **Layer 2** (Teensy 4.0): Main brain — strategy state machine, X-drive motor control (4 mecanum), BNO085 IMU heading hold (PID), camera-based localisation (dual-goal triangulation), dribbler/kicker scoring. Receives from L1 (Serial4), L3 (Serial5), camera (Serial3).
- **Layer 3** (XIAO ESP32-C3): 28-IR sensor ring for ball detection (cluster-based), 4 DIP switches. Sends ball angle + switches to L2 via Serial1→Serial5.

## Key Files

- `include/config.h` — all tuning constants (LDR thresholds, PID gains, motor trim, pin mappings)
- `src/layer1/main.cpp` — line detection loop
- `src/layer2/main.cpp` — strategy, movement, localisation, scoring
- `src/layer3/main.cpp` — IR ball detection, switches
- `lib/serial_comm/` — shared UART packet framing
- `lib/movement/` — motor control, IMU, heading hold PID
- `lib/cam_comm/` — camera CSV parsing (goal + ball positions)
- `lib/scoring/` — dribbler ESC, solenoid kicker, lightgate
- `lib/light_ring/` — LDR mux reading and line-finding algorithm

## Code Style

- 4-space indentation
- `camelCase` for functions and variables
- `UPPER_SNAKE_CASE` for `#define` constants and pin names
- Floating-point values scaled by 10 for int16_t serial transmission
- Debug prints prefixed with tags like `[IMU]`, `[L1]`, `[CAM]`
- Include order: `<Arduino.h>` → external libs → project headers
- Header guards: `#ifndef HEADER_H` style

## Providing Answers and Writing Code

- Ensure file organisation is consistent with pre-existing code
- Aim for non-desctructive edits, assume all existing code works
- Always reference pre-existing code before writing
- Write non-blocking code as much as possible