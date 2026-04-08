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
- Do not and never skip steps. Break code writing into small, individually testable functions before going on to the next feature which builds upon it
- Always break down instructions into smaller, more feasible steps
- Suggest hardware checks when it may be a possible root cause, instead of diving into code-based rabbit holes

## Testing After Every Change

After every code change to `src/layer2/main.cpp` (or any lib it depends on), follow this checklist:

### 1. Build check (always)
```
pio run -e layer2-teensy40
```
Must compile with **zero errors**. Warnings are acceptable only for intentionally unused code (e.g. functions added for a later phase).

### 2. Serial monitor smoke test (when hardware is available)
Upload and open serial monitor:
```
pio run -e layer2-teensy40 -t upload && pio device monitor
```
**What to expect in serial output:**
- `[MODE] ...` prints every loop showing which task is active (ORBIT, IR CHASE, SCORING, RETURN TO CENTRE, IDLE, LINE DETECTED)
- `[LOC] x=... y=... heading=...` when both/one goal is visible
- `[ORBIT]`, `[IR]`, `[SCORE]`, `[RTC]`, `[IDLE]` prefixed lines with live values
- If nothing prints: check USB connection, baud 115200
- If `[MODE]` flickers rapidly between states: a sensor is noisy — check that specific sensor's debug env

### 3. Subsystem isolation tests (when debugging a specific issue)
Use the dedicated test environments to isolate problems:

| Issue | Test env | What to check |
|-------|----------|---------------|
| Motors wrong direction/speed | `layer2-motor-debug` | Each motor runs individually, verify direction + label match |
| IMU drift / no heading | `layer2-imu-debug` | Yaw value updates smoothly, no jumps |
| Camera not detecting goals | `layer2-serial-debug` | Raw cam CSV appears on Serial3 |
| Dribbler/kicker/lightgate | `layer2-scoring-debug` | Interactive menu: test each subsystem individually |
| Localisation drift | `layer2-loc-center` | Robot drives to centre, `[LOC]` prints stable x/y |
| LDR thresholds wrong | `layer1-calibrate` | Run on L1 Teensy, recalibrate thresholds |
| L1 not sending data | `layer1-serial-debug` | Verify packets arrive on L2's Serial4 |

### 4. On-field functional test (after uploading production code)
Place robot on field and verify in order:
1. **Idle** — no ball, no goals visible → robot should stop, serial prints `[IDLE]`
2. **Return to centre** — show both goals to camera, move robot off-centre → robot drives toward centre, prints `[RTC]`
3. **IR chase** — place ball in IR range but out of camera range → robot chases ball, prints `[IR]`
4. **Orbit** — ball visible to both camera + IR → robot orbits behind ball, prints `[ORBIT]`
5. **Line escape** — push robot onto white line → robot reverses off, prints `[MODE] LINE DETECTED`
6. **Score** — feed ball into catchment near goal → robot drives forward and kicks
7. **DIP switches** — flip role switch → robot should switch between striker/goalie behaviour

### 5. What "working" looks like per phase
- **Phase 1 (state machine)**: Behaviour identical to before refactor. Serial output now shows `[MODE]` tags consistently. Task transitions visible in serial.
- **Phase 2 (orbit)**: Robot curves around ball instead of chasing head-on. Approaches from goal-side. Slows when close.
- **Phase 3 (scoring)**: Robot drives forward with ball, rotates to face goal, kicks when aligned. No more "kick immediately" on catchment.
- **Phase 4 (goalie)**: Flipping role DIP switch makes robot sit in front of own goal and track ball side-to-side. Rushes ball when close.
- **Phase 5 (polish)**: Heading snaps faster during scoring. Robot goes to last-seen-ball side when ball lost. Line escape resumes previous task.