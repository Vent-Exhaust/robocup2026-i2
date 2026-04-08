# Strategy Implementation TODO

Porting the 2025 strategy into the 2026 i2 codebase. The 2025 code lives in
`ducc/robocup-2025/open/software design/microcontrollers/src/teensy1/`.

Key hardware differences from 2025:
- **We have an IR ring** (28 sensors on L3) — 2025 did not, they relied on camera for ball.
- **Our camera is weak at distance** — use IR as primary ball bearing, camera only when close.
- **No TOFs** — rules prohibit them. Localisation is camera-only (dual-goal triangulation), which we already have.
- **Field units are cm** — 2025 used pixel coordinates (~±800 x-range). Our field is 219x158 cm.

---

## Phase 1: State Machine Foundation

Refactor `src/layer2/main.cpp` from the current flat if-else chain into a task-based state machine.

- [ ] **1.1 Define task enum**
  Add to `src/layer2/main.cpp` (or a new header):
  ```cpp
  enum Task { ORBIT_TO_BALL, SCORE, GOALIE, RETURN_TO_CENTRE, IDLE };
  static Task task = IDLE;
  ```
  **2025 ref**: `teensy1/main.h:694` — `int task;` with values 0/1/2/3
  **Our code**: `src/layer2/main.cpp:111-276` — the current `loop()` if-else chain

- [ ] **1.2 Write `striker()` role function**
  Runs every loop to decide the current task based on catchment state.
  ```
  if kicked recently         -> ORBIT_TO_BALL (reset catchment state)
  if ball in catchment       -> SCORE (record timestamp)
  if was in catchment <500ms -> stay SCORE
  if ball found (IR or cam)  -> ORBIT_TO_BALL
  if locValid                -> RETURN_TO_CENTRE
  else                       -> IDLE
  ```
  **2025 ref**: `teensy1/main.cpp:36-67` — `striker()` function with `catchment_timeout = 500ms`
  **Our code**: `src/layer2/main.cpp:119` — `bool ballCaught = checkCatchment();` currently only used in the if-else

- [ ] **1.3 Write `goalie()` role function**
  Simply sets `task = GOALIE` (goalie logic is in Phase 4).
  **2025 ref**: `teensy1/main.cpp:69-72` — `void goalie()`

- [ ] **1.4 Use DIP switches for role selection**
  L3 already sends 4 DIP switches (`l3SwitchGoal`, `l3SwitchRole`, `l3SwitchStrat0`, `l3SwitchStrat1`).
  Use `l3SwitchRole` to pick striker vs goalie each loop.
  **Our code**: `lib/layer3_comm/layer3_comm.h:17-20` — switch externs

- [ ] **1.5 Refactor `loop()` into `switch(task)`**
  ```cpp
  void loop() {
      // ... read sensors, update localisation (unchanged) ...

      if (l3SwitchRole) striker(); else goalie();

      switch (task) {
      case ORBIT_TO_BALL:  orbitToBall();      break;
      case SCORE:          orbitScore();        break;
      case GOALIE:         defendGoal();        break;
      case RETURN_TO_CENTRE: returnToCentre(); break;
      case IDLE:           stopMotors();        break;
      }

      moveRobot(moveAngle, moveSpeed, omega);
  }
  ```
  Extract each current if-else branch into its own function. No behaviour change yet.
  **2025 ref**: `teensy1/main.cpp:279-332` — the `switch(robot.task)` block

---

## Phase 2: Orbit Refinement

Improve the orbit so the robot approaches the ball from behind (between ball and our goal).

- [ ] **2.1 Determine orbit target bearing**
  The robot should orbit the ball so it ends up between the ball and the opponent's goal.
  - If `camBlueDetected`: target bearing = `camBlueAngle` (goal we're attacking)
  - Else: target bearing = world 0 relative to robot = `-imuYaw` (fallback)
  **2025 ref**: `teensy1/main.cpp:284-291` — uses `blue_open.current_pose.bearing`
  **Our code**: `src/layer2/main.cpp:173-178` — similar logic already exists

- [ ] **2.2 Replace orbit with angular-offset approach**
  2025's orbit adds an angular offset to the ball bearing that shrinks as the robot gets behind the ball.
  Since our camera is bad at distance, simplify:
  - Use IR bearing as the ball direction (reliable at all distances)
  - Compute `alignError` = goal bearing - ball bearing (how far off we are from being behind the ball)
  - Offset = `alignError * scaleFactor` (capped at ±90 degrees)
  - Move direction = IR ball bearing + offset
  - This makes the robot curve around the ball instead of chasing head-on
  **2025 ref**: `teensy1/algorithm.cpp:93-218` — `orbitToBall()`, especially lines 98-108 for the offset calc and 120-128 for the exponential orbit factor
  **Our code**: `src/layer2/main.cpp:164-234` — current orbit uses tangent+creep vector blend

- [ ] **2.3 Speed control without reliable distance**
  Since we can't trust camera ball distance at range:
  - Use `l3BallCount` (number of active IR sensors) as a rough proximity signal — more sensors = closer ball
  - When cam ball is available AND close (< 30cm): use `camBallDist` for fine deceleration
  - Otherwise: use a fixed moderate speed (e.g. 0.15) that slows when IR count is high
  **2025 ref**: `teensy1/algorithm.cpp:130-144` — deceleration curve based on `ball.distance_from_robot`
  **Our code**: `src/layer2/main.cpp:197-198` — `orbitSpeed` is currently based on alignment error only

- [ ] **2.4 Forward creep when aligned**
  When the robot is behind the ball (alignError small), blend in a forward component toward the ball.
  Keep the existing concept but tie it to the new orbit.
  **2025 ref**: implicit — when orbit offset is small, the robot naturally moves toward the ball
  **Our code**: `src/layer2/main.cpp:206-209` — `CREEP_SPEED` logic (keep this)

- [ ] **2.5 Edge speed capping during orbit**
  Already implemented — just make sure it's applied in the orbit function.
  **Our code**: `src/layer2/main.cpp:69-93` — `edgeSpeedCap()`

---

## Phase 3: Scoring Sequence

When the ball is caught, drive toward the goal and kick when aligned.

- [ ] **3.1 Implement `orbitScore()` function**
  Called when `task == SCORE`:
  1. Drive forward (angle = 0 in robot frame) at scoring speed
  2. Set yaw target to face the blue goal: `yawTarget = imuYaw + camBlueAngle`
  3. The existing PID heading hold will rotate the robot to face the goal while moving
  **2025 ref**: `teensy1/algorithm.cpp:263-305` — `orbitScore()` and `scoringStrategyOne()`
  **Our code**: `src/layer2/main.cpp:154-163` — current scoring is just "kick if close to goal"

- [ ] **3.2 Kick condition**
  Kick when:
  - `camBlueDetected` AND `camBlueDist < SCORE_KICK_DIST` (e.g. 50cm)
  - AND `abs(camBlueAngle) < SCORE_ALIGN_THRESH` (e.g. 10 degrees)
  If goal not visible, drive forward for a max time (e.g. 2 seconds) then kick blindly.
  **2025 ref**: `teensy1/algorithm.cpp:335-341` — kicks when `y > 100` and bearing error < 5
  **Our code**: `src/layer2/main.cpp:155` — currently checks `camBlueDist < 50` only

- [ ] **3.3 Add scoring constants to `config.h`**
  ```cpp
  constexpr float SCORE_SPEED         = 0.3f;
  constexpr float SCORE_KICK_DIST     = 50.0f;   // cm
  constexpr float SCORE_ALIGN_THRESH  = 10.0f;   // degrees
  constexpr unsigned long SCORE_TIMEOUT_MS = 2000; // blind kick fallback
  ```
  **Our code**: `include/config.h` — add alongside existing tuning constants

- [ ] **3.4 Line rejection during scoring**
  If line detected while scoring, run the escape logic but keep `task = SCORE` so scoring resumes.
  **2025 ref**: `teensy1/algorithm.cpp:278-282` — `rejectLine(target_bearing)` during score
  **Our code**: `src/layer2/main.cpp:137-153` — current line escape doesn't preserve task state

- [ ] **3.5 Post-kick reset**
  After kicking: stop briefly, reset task to `ORBIT_TO_BALL`, reset catchment state.
  **2025 ref**: `teensy1/main.cpp:38-43` — `if (robot.kicker.kicked) { task = 0; }`
  **Our code**: `lib/scoring/scoring.h:19` — `KICK_COOLDOWN_MS = 1000`

---

## Phase 4: Goalie Mode

Defend goal by tracking the ball along a curved line in front of goal.

- [ ] **4.1 Define the defensive curve**
  2025 used `f(x) = -5e-15 * x^6 - 500` in pixel coords (x range ~±800, y ~ -500 to -700).
  Scale to our cm coords (x range ~±79, field half-Y = 109.5):
  A starting point: `f(x) = -a * x^6 - 30` where the curve sits ~30cm in front of our goal.
  The exact shape needs field tuning — the idea is flat in the middle, curves back at the edges.
  **2025 ref**: `teensy1/robot.cpp:12-14` — `f(x)`, `df(x)`, `d2f(x)`
  Skip Newton's method for now — just clamp target_x to ball's x projected onto the curve.

- [ ] **4.2 Implement `goalieTrack()`**
  1. Use localisation (`locX`, `locY`) for robot position
  2. Estimate ball world position: `ballWorldX = locX + ballRelX` (from IR bearing + rough distance estimate, or just use `locX` offset by IR angle direction)
  3. Find target point on curve: `targetX = clamp(ballWorldX, -60, 60)`, `targetY = f(targetX)`
  4. Move toward that point using `moveToPoint` style logic (angle + speed from distance)
  **2025 ref**: `teensy1/robot.cpp:107-237` — full `goalieTrack()` with Newton's method, PID speed, step-based movement
  **Our code**: `src/layer2/main.cpp:252-270` — `RETURN_TO_CENTRE` logic is similar (move to a world point) — reuse that pattern

- [ ] **4.3 Implement `goalieRush()`**
  When ball is close and in front, charge straight at it:
  - Trigger: IR ball count high (close) AND ball bearing is roughly forward (< 20 degrees off centre)
  - Move direction = ball bearing, speed = high (0.35-0.5)
  **2025 ref**: `teensy1/robot.cpp:239-260` — `goalieRush()`
  **2025 ref**: `teensy1/algorithm.cpp:26-91` — `defendGoal()` with timeout logic (100ms before rushing)

- [ ] **4.4 Implement `defendGoal()` with rush timeout**
  Combines track + rush:
  ```
  if on_line                       -> rejectLine
  if ball in front AND close:
      if close for > 100ms         -> goalieRush()
      else                         -> goalieTrack()
  else                             -> goalieTrack()
  ```
  **2025 ref**: `teensy1/algorithm.cpp:26-91` — full `defendGoal()` state machine

- [ ] **4.5 Add goalie constants to `config.h`**
  ```cpp
  constexpr float GOALIE_CURVE_A       = ???;   // tune on field
  constexpr float GOALIE_CURVE_OFFSET  = -30.0f; // cm in front of goal
  constexpr float GOALIE_X_CLAMP       = 60.0f;  // max x tracking range
  constexpr float GOALIE_RUSH_SPEED    = 0.4f;
  constexpr float GOALIE_TRACK_MIN_SPD = 0.1f;
  constexpr float GOALIE_TRACK_MAX_SPD = 0.35f;
  constexpr unsigned long GOALIE_RUSH_TIMEOUT_MS = 100;
  constexpr float GOALIE_RUSH_ANGLE_THRESH = 20.0f; // degrees
  ```

---

## Phase 5: Polish

- [ ] **5.1 Per-task PID tuning**
  Orbit needs gentle heading hold, scoring needs aggressive snapping.
  Adjust `YAW_KP` dynamically based on current task (or use separate PID instances).
  **2025 ref**: `teensy1/main.cpp:274-276` — `kp` changes per task case
  **Our code**: `include/config.h:58-70` — single `YAW_KP`

- [ ] **5.2 Return-to-centre improvements**
  When ball is lost, go to the side it was last seen instead of dead centre.
  Track `lastBallSide` (positive/negative x when ball was last detected).
  Move to `(sign(lastBallSide) * 30, 0)` instead of `(0, 0)`.
  **2025 ref**: `teensy1/algorithm.cpp:221-258` — uses `ball.ball_last_seen.x` to pick side

- [ ] **5.3 Line rejection preserves task**
  Currently line escape overrides everything. Change so that after escaping, the robot returns to whatever task it was doing (especially important for scoring).
  **2025 ref**: `teensy1/algorithm.cpp:186-196` — `rejectLine(bearing)` during orbit preserves orbit bearing

- [ ] **5.4 Kick cooldown handling**
  After a kick, ignore catchment for `KICK_COOLDOWN_MS` to avoid the lightgate false positive.
  **Our code**: `lib/scoring/scoring.h:19` — `KICK_COOLDOWN_MS = 1000` (already defined, wire it in)

---

## File Reference

| File | Purpose |
|------|---------|
| `src/layer2/main.cpp` | Main strategy loop — most changes go here |
| `include/config.h` | Tuning constants — add scoring/goalie params |
| `lib/movement/movement.h` | Motor control, PID, IMU — mostly unchanged |
| `lib/scoring/scoring.h` | Dribbler, kicker, lightgate — mostly unchanged |
| `lib/cam_comm/cam_comm.h` | Camera data — unchanged |
| `lib/layer1_comm/layer1_comm.h` | Line data from L1 — unchanged |
| `lib/layer3_comm/layer3_comm.h` | IR ball + switches from L3 — unchanged |

## 2025 Reference Files

All under `ducc/robocup-2025/open/software design/microcontrollers/src/teensy1/`:

| File | What to look at |
|------|-----------------|
| `main.cpp` | `striker()`, `goalie()`, main loop task switch |
| `main.h` | `Robot` class, `MoveData`, `LineData` structs |
| `algorithm.cpp` | `orbitToBall()`, `orbitScore()`, `scoringStrategyOne()`, `defendGoal()` |
| `robot.cpp` | `goalieTrack()`, `goalieRush()`, `moveToPoint()`, `rejectLine()` |
| `camera.cpp` | `storeRobotPose()` localisation (we already have this) |
| `base.cpp` | Motor control, PID heading hold (we already have this) |
| `serial.cpp` | Serial handlers, ball distance regression |
