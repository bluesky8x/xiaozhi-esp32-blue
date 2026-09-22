# Blue V4 — ESP32-S3 quadruped

Voice-controlled quadruped robot: same audio/display stack as Blue V2, but the wheel
motors are replaced by **8 × MG90S servos** (4 legs × hip + knee) driven by a
**PCA9685** I2C PWM expander.

| Item | Part |
|---|---|
| MCU | ESP32-S3-WROOM-1-N16R8 (16 MB flash, 8 MB octal PSRAM) |
| Servos | 8 × MG90S, via PCA9685 @ `0x40` on I2C (50 Hz, 12-bit) |
| Display | ST7789 1.54" 240×240 SPI, **Otto GIF face** |
| Audio | INMP441 mic + MAX98357 amp (shared duplex I2S, amp on 5 V) |
| Touch | TTP223 on GPIO 16 |
| Range | VL53L0X ToF on the shared I2C bus (0x29) |
| Status | LED on GPIO 48 |
| Power | 1S 18650 + Type-C charge/discharge module (5 V/2 A), servo rail on 5 V |

Full pinout, PCA9685 channel map, power tree and the bring-up checklist:
**[WIRING.md](./WIRING.md)**.

## Self-contained board

This directory is independent of `blue-v2`: every board support file is a clone
(`blue_v4_face_display`, `blue_v4_cloud_guard`, `power_controller`, plus the new
`pca9685` / `servo_controller` / `gait_engine`). Only the shared Otto icon font and emoji
display assets come from `boards/otto-robot/`.

## Architecture

```
MCP tools (self.gait.*, self.servo.*, self.tof.*, self.motor.* compat)
        │
        ├── GaitEngine    IK + crawl gait + poses  →  publishes joint targets
        │
        └── ServoController  50 Hz interpolation, slew limit, NVS trim, OE# safety
                    │
                Pca9685 (I2C, board-owned bus)  ──►  8 × MG90S
```

### 4 legs × 2 servos (hip + knee)

Each leg is driven by **two** servos that always move as a pair, solved from one foot
position by inverse kinematics — never commanded independently:

- `GaitEngine::SolveLeg(foot_x_mm, foot_z_mm, &hip_deg, &knee_deg)` solves the
  femur/tibia triangle (`LEG_FEMUR_MM`, `LEG_TIBIA_MM`) for both joints at once.
- `PublishLegs()` maps leg `i` to **hip joint `i*2`** and **knee joint `i*2+1`**
  (PCA9685 CH0/CH1 = front-left, CH2/CH3 = front-right, CH4/CH5 = rear-left,
  CH6/CH7 = rear-right); `ServoController` then slews both toward the target.
- The crawl gait moves **one leg at a time** (lift → swing → plant) while the other three
  stay planted, then the planted legs push back a quarter stride, so the body advances one
  stride per four-leg cycle and the robot is statically stable throughout.
- Poses (`stand`, `sit`, `body(height, pitch, roll)`) are expressed as foot positions for
  all four legs, so both servos of every leg stay coordinated: pitch/roll tilt the body by
  changing each leg's reach.

**2-DoF limitation:** there is no abduction (sideways) joint, so a foot cannot be placed
laterally. Turning uses a differential stride (left and right legs move in opposite
directions) plus body roll. If your frame has a different joint arrangement (for example
hip *yaw* + knee pitch), adjust `kHipServoSign` / `kKneeServoSign` in `gait_engine.cc`.

- **ServoController** is the only task that touches the PCA9685. Producers publish targets;
  discrete commands (relax/enable/trim) go through a bounded queue.
- **GaitEngine** owns the inverse kinematics and commands motion; motion is intentionally
  conservative so the 5 V/2 A rail stays within budget (one leg moves at a time).
- **OE# on GPIO 1** is pulled up, so the servos are limp through boot/reset/brownout and only
  energise after a clean init.

## MCP tools

| Tool | Purpose |
|---|---|
| `self.gait.stand` | Stand and hold posture (optional height) |
| `self.gait.sit` | Fold down into a resting sit |
| `self.gait.walk` | Crawl-gait walk (`direction`, `steps`, `stride_mm`, `step_ms`) |
| `self.gait.turn` | Differential-stride turn in place |
| `self.gait.body` | Posture: height, pitch, roll |
| `self.gait.stop` | Safety stop + release torque |
| `self.gait.leg_test` | Bring-up: move ONE leg by IK (both its servos) while the others stand |
| `self.gait.status` | Gait/driver state JSON |
| `self.servo.set` / `self.servo.set_all` | Direct joint angles (testing/corrections) |
| `self.servo.get_positions` | Commanded angles + trims JSON |
| `self.servo.trim` / `self.servo.invert` | Per-joint calibration, persisted in NVS |
| `self.servo.relax` / `self.servo.enable` / `self.servo.stop` | Torque control |
| `self.motor.move` / `forward` / `backward` / `turn_left` / `turn_right` / `circle` / `stop` / `dance` | blue-v2 compatible layer — see below |
| `self.tof.calibrate` | Calibrate the front VL53L0X on open floor (saved to NVS) |
| `self.tof.get_distance` | Read the front distance in mm (debug) |
| `self.tof.clear_calibration` | Drop the saved calibration |
| `self.power.enter_sleep` | Sleep: dim screen + relax servos |

Cloud server URLs are blocked (`BLUE_V4_BLOCK_CLOUD_SERVERS`) — the OTA/websocket endpoint
must come from the WiFi portal.

## Server compatibility (no server change needed)

The xiaozhi server maps its `mv:*` tags to device tools **by name**
(`core/utils/robot_move_codec.py: MOVE_CODE_TO_MCP`, trying `self.motor.*` first). So
blue-v4 exposes the same `self.motor.*` tool names as blue-v2 and translates the wheel
semantics into leg motion:

| Server command | blue-v2 (wheels) | blue-v4 (crawl gait) |
|---|---|---|
| `self.motor.move` left=right>0 | both wheels forward | walk forward |
| `self.motor.move` left=right<0 | both wheels back | walk backward |
| `self.motor.move` left>0, right<0 | turn left | turn left (differential stride) |
| `self.motor.move` left<0, right>0 | turn right | turn right |
| `self.motor.move` 0, 0 | stop | stop + relax |
| `self.motor.forward` / `backward` / `turn_left` / `turn_right` | wheel burst | one crawl burst |
| `self.motor.circle` | arc | forward + steady turn |
| `self.motor.dance` (`track`, `segment_ms`, `timeline`) | wheel dance | servo gestures: `D` bounce, `g` sway, `v` leg wave, `c` crouch |
| `self.motor.stop` | stop | stop + relax |

`duration_ms` maps to a crawl-step count (one step = 4 legs), and the speed magnitude maps
to `step_ms`. Turn/motor direction is a mechanical convention: if a turn comes out
reversed after assembly, flip the hip joints with `self.servo.invert` rather than changing
the server.

Set `BLUE_V4_MOTOR_COMPAT_TOOLS 0` in `config.h` to hide this layer and use only the native
`self.gait.*` / `self.servo.*` tools.

## Build

```bash
source ~/esp/esp-idf/export.sh
python scripts/build.py blue-v4 --name blue-v4        # canonical
idf.py -p /dev/cu.usbmodem* flash monitor
```

menuconfig: **Board type → Blue V4 (ESP32-S3 quadruped, PCA9685 servos)** and
**LCD type → ST7789 240×240**.

## Tuning values in `config.h`

`LEG_FEMUR_MM`, `LEG_TIBIA_MM`, `LEG_HIP_OFFSET_X_MM`, `LEG_HIP_OFFSET_Y_MM`,
`BODY_STAND_HEIGHT_MM`, `STRIDE_LENGTH_MM`, `SERVO_SLEW_DEG_PER_SEC`,
`SERVO_IDLE_RELAX_MS`, `SERVO_MAX_ACTIVE_MOVING`.

## Pulse band = angle gain (đọc trước khi chỉnh bất kỳ số nào)

The PCA9685 maps `pulse = min_us + (angle/180) * (max_us - min_us)`, so **the pulse band is
the gain of the whole angle model**. `SERVO_MIN/MAX_PULSE_US` defaults to **500..2500 µs**
(the MG90S datasheet band: 500 µs = 0°, 1500 µs = 90°, 2500 µs = 180°), which makes the
commanded angles equal to real physical degrees — that is what all the geometry maths
(`LEG_*_MM`, the crouch table, α = 43.3°) assumes.

| Band | µs/deg | Effect |
|---|---|---|
| 500..2500 (default) | 11.1 | 0..180° commanded **= 0..180° physical**; full servo range |
| 1000..2000 | 5.6 | every motion is **half** the physical size (the servo never uses its ends) |

Change it at runtime with `srv:range=500-2500` (persisted in NVS `pmin`/`pmax`, which wins
over `config.h`), and prove the servo really reaches both ends with `srv:travel=N`
(`0°→180°→0°` twice, then back to neutral 90°). If a servo buzzes or binds before the ends,
use a narrower band **and double every angle delta** in `config.h` to keep the same motion.

Key joint-space values (all in **physical** degrees with the default band): stand = all
joints at 90°; sit = knee 90+18.9° (`GAIT_JOINT_CROUCH_DEG_PER_MM`); tilt 12° = knees ±19.2°
(`GAIT_JOINT_TILT_DEG_PER_DEG`); walk = hip ±22.5° and knee lift 36°; posture motion runs at
`POSTURE_RATE_DEG_PER_SEC` = 70 °/s physical.

The hip/knee servo mapping constants live at the top of `gait_engine.cc`
(`kHipServoSign`, `kKneeServoSign`, centers) and must be validated on the real frame —
see the bring-up checklist in `WIRING.md`.

## Test procedure after flashing

Robot on **four blocks** (feet off the ground) for steps 1-3. Servo rail powered separately,
common GND, `OE#` on IO1 with its 10 kΩ pull-up, MX1508 motor supply disconnected.

| Step | Do | Expect |
|---|---|---|
| 0 | Power up, watch serial | Servos **must not move**. Log: `I2C bus on SDA 41 / SCL 42`, `PCA9685 0x40 ready (outputs OFF)`, `ServoController: ready (… hardware=1)`, `Blue-v2 compatible motor tools registered (8)`, `GaitEngine: ready`, `ToF ready`, `gait guard active`, `Robot tools initialized` |
| 1 | `self.servo.set joint=0 angle=90`, then sweep 60/120 | One servo moves smoothly; use `self.servo.trim` to square the horn, `self.servo.invert` if mirrored |
| 2 | `self.gait.stand` | All 8 energise to a square stance; `self.gait.status` shows `holding` |
| 3 | `self.gait.leg_test leg=0 foot_z=70…140`, then `foot_x=-30…+30` | Both servos of that leg fold/extend and move fore/aft **together**; repeat per leg |
| 4 | `self.gait.walk steps=1 stride_mm=10 step_ms=800` | Crawl order FR → RL → FL → RR: lift → swing → plant, one leg at a time |
| 5 | `self.gait.turn steps=1 step_ms=800`, `self.gait.sit`, `self.gait.body height_mm=80 pitch_deg=10` | Differential turn, sit, posture tilt |
| 6 | Feet on the floor: `self.gait.stand` then walk 2-3 steps | No dragging/collapse; tune `STRIDE_LENGTH_MM`, `BODY_STAND_HEIGHT_MM`, `step_ms` |
| 7 | Voice (tap TTP223 to start chat): say a movement request | Server emits `mv:*` → `self.motor.*` → gait moves the legs. No server change needed |
| 8 | `self.tof.calibrate` on open floor, then walk toward a wall / table edge | `BlueV4GaitGuard: STOP (obstacle)` / `STOP (cliff_far)` + gait stops |
| 9 | `self.gait.stop`, tap during a walk, hold BOOT 5 s | Servos go limp; walk cancels; factory reset relaxes servos before reboot |

| Symptom | Fix |
|---|---|
| Servos twitch at boot or hold position after a reset | `OE#` not wired to IO1 (or pull-up missing) |
| `PCA9685` not found | I2C wiring, 4.7 kΩ pull-ups, address 0x40 |
| One leg moves backwards | `self.servo.invert` on that hip/knee joint |
| Legs buzz/jitter while holding | Normal servo load; lower `BODY_STAND_HEIGHT_MM` or relax when idle |
| `Reset: BROWNOUT` while walking | Servo rail sagging: feed `V+` from the cell, add bulk caps, increase `step_ms` |
| Robot drags feet / tips over | Geometry constants wrong (`LEG_FEMUR_MM`, `LEG_TIBIA_MM`, offsets) or stride too long |
| Guard stops on nothing | Re-run `self.tof.calibrate`; `TOF_OBSTACLE_GUARD_ENABLE 0` disables it |

## Status

- ✅ Board scaffolding, pin map, PCA9685 driver, servo controller, gait engine, Otto GIF
  display, front ToF (board-owned bus injection), ToF gait guard, cloud guard, build-chain
  integration.
- ⏳ Server-side tags/prompt/docs in `esp32-server-blue` (the `mv:*` wheel tags still need
  a servo/gait equivalent).
- ⚠️ Not yet validated on hardware: servo mapping signs, leg geometry, current draw per
  crawl step, and the real gait stability.
