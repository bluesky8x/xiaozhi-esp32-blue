#ifndef _BLUE_V4_CONFIG_H_
#define _BLUE_V4_CONFIG_H_

#include <driver/gpio.h>

// Blue V4 — ESP32-S3-WROOM-1-N16R8 quadruped robot.
//
// Same peripheral stack as Blue V2 (ST7789 1.54" 240x240, INMP441 I2S mic,
// MAX98357 I2S amp, TTP223 touch, VL53L0X ToF, status LED) but the MX1508 DC
// motors are replaced by 8x MG90S servos driven by a PCA9685 I2C PWM expander:
// 4 legs x 2 degrees of freedom (hip + knee).
//
// This board directory is SELF-CONTAINED: all board support files are clones
// living in main/boards/blue-v4/ (see README.md), independent of blue-v2.
// Wiring reference: WIRING.md.

#define AUDIO_INPUT_SAMPLE_RATE 16000
// Duplex I2S uses one BCLK/WS clock pair (see no_audio_codec.cc) — matches mic + Opus 16 kHz.
#define AUDIO_OUTPUT_SAMPLE_RATE 16000

// Shared I2S bus: INMP441 + MAX98357 (duplex)
#define AUDIO_I2S_GPIO_WS GPIO_NUM_4
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_5
#define AUDIO_I2S_GPIO_DIN GPIO_NUM_6   // mic SD
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_7  // speaker DIN

// INMP441 (L/R->GND): AGC + soft compressor in no_audio_codec (see AUDIO_MIC_AGC_* in CMakeLists).
// AudioCodec::input_gain_ defaults to 0.0, so GetAudioCodec() must call SetInputGain() with
// this value or the uplink stays silent (blue-v2 does exactly the same).
#define AUDIO_MIC_SHIFT_BITS 12
#define AUDIO_MIC_INPUT_GAIN 1.0f
#define AUDIO_MIC_SOFT_LIMIT 24000

// Mic level debug on serial (~1 Hz) while I2S input is active (bring-up).
#define AUDIO_MIC_DEBUG_LOG 1

#define BUILTIN_LED_GPIO GPIO_NUM_48
#define BOOT_BUTTON_GPIO GPIO_NUM_0
#define TOUCH_BUTTON_GPIO GPIO_NUM_16
// TTP223 SIG goes HIGH when touched (most modules). Set 0 if yours is active-low.
#define TOUCH_BUTTON_ACTIVE_HIGH 1
#define TOUCH_DEBOUNCE_MS 350

#define RESET_NVS_BUTTON_GPIO GPIO_NUM_NC
#define RESET_FACTORY_BUTTON_GPIO GPIO_NUM_NC
#define FACTORY_RESET_LONG_PRESS_MS 5000

// -1 = no auto sleep while the quadruped is on the bench; set 60 for battery use.
#define POWER_SAVE_SLEEP_SECS (-1)
// Minimum backlight when entering sleep via MCP (not 1% — nearly invisible on 1.54" LCD).
#define POWER_SAVE_DIM_BRIGHTNESS 25

// LVGL draw buffer height (lines). Internal DMA SRAM — avoid full-frame PSRAM flush
// (SPI DMA OOM + WDT). See main/display/lcd_display.cc.
#define DISPLAY_LVGL_BUFFER_LINES 80

#define DISPLAY_MOSI_PIN GPIO_NUM_10
#define DISPLAY_CLK_PIN GPIO_NUM_9
#define DISPLAY_DC_PIN GPIO_NUM_8
// Breadboard: module RES → GPIO 18. PCB with RES tied to 3.3 V: use GPIO_NUM_NC.
#define DISPLAY_RST_PIN GPIO_NUM_18
#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_17
#define DISPLAY_CS_PIN GPIO_NUM_NC

// No separate decor LED on Blue V4.
#define DECOR_LED_GPIO GPIO_NUM_NC

// ---------------------------------------------------------------------------
// I2C bus — shared by VL53L0X (0x29) and PCA9685 (0x40).
// The board creates this bus and injects the handle into both drivers.
// ---------------------------------------------------------------------------
#define I2C_SENSOR_SDA_PIN GPIO_NUM_41
#define I2C_SENSOR_SCL_PIN GPIO_NUM_42
#define I2C_SENSOR_SPEED_HZ 400000
#define I2C_SENSOR_PORT I2C_NUM_0

// ---------------------------------------------------------------------------
// PCA9685 servo driver — 4 legs x 2 joints (hip + knee) = 8 of 16 channels.
// ---------------------------------------------------------------------------
#define PCA9685_I2C_ADDR 0x40
// OE# (active LOW = outputs enabled). Pulled up in hardware so the servos stay
// limp through any reset/brownout and only energise after a clean init.
#define PCA9685_OE_GPIO GPIO_NUM_1

// Logical joint -> PCA9685 channel. Joint order: 0/1 = leg 1 FL hip/knee, 2/3 = leg 2 FR,
// 4/5 = leg 3 RL, 6/7 = leg 4 RR. Only change an entry when a driver channel is damaged:
// re-plug that servo into a free channel (8..15) and point the joint there.
#define SERVO_CHANNEL_MAP {0, 1, 2, 3, 4, 5, 6, 7}

// Default direction inversion per joint (bit i = joint i): a "mount" correction applied ON TOP
// of the NVS calibration (effective = NVS value XOR mask bit). Robot geometry: hip = VERTICAL
// yaw axis (shaft down), 4 legs at the 4 corners of the square body, tibia on the body diagonal
// at neutral. Both sides are mirror images, so the right-hand hips (joint 2 = FR, 6 = RR) must be
// inverted; add bits 3 and 7 (0xCC) if the right-hand knees fold down instead of up.
#define SERVO_INVERT_DEFAULT_MASK 0x44

#define SERVO_COUNT 8
#define SERVO_PWM_FREQ_HZ 50
#define SERVO_PWM_RESOLUTION 4096
// MG90S (and SG90-class) servos take ~1000..2000 us for 0..180 deg. The mapping in
// ServoController is pulse = min + (angle/180) * (max - min), so a wider default (500..2500)
// stretches every commanded angle 2x in the physical world AND drives the ends past the servo's
// mechanical stops (the servo then just ticks/buzzes, stalls and sags the 5 V rail).
// Keep these at the servo's real band; per-robot differences are handled by trim/invert
// (persisted in NVS) instead. Overridable at runtime with self.servo.pulse_range.
#define SERVO_MIN_PULSE_US 1000
#define SERVO_MAX_PULSE_US 2000
#define SERVO_UPDATE_PERIOD_MS 10  // 100 Hz interpolation tick (dày hơn = đường đi mượt hơn)
// 1 = log the servo task rate/state once per second (bring-up diagnostics).
#define SERVO_TICK_DEBUG_LOG 1
#define SERVO_CMD_QUEUE_DEPTH 8

// Servo calibration / motion defaults (per-servo trim lives in NVS, see servo_controller).
#define SERVO_DEFAULT_MIN_DEG 0.0f
#define SERVO_DEFAULT_MAX_DEG 180.0f
#define SERVO_DEFAULT_NEUTRAL_DEG 90.0f
// Motion profile: slow and soft. Speed is ramped with an acceleration limit, so moves
// start/stop gently and arrive without overshoot (smooth, no jitter, clean stop).
#define SERVO_SLEW_DEG_PER_SEC 90.0f
#define SERVO_ACCEL_DEG_PER_SEC2 240.0f
#define SERVO_STOW_SLEW_DEG_PER_SEC 90.0f

// Boot pose: the joints stay limp after power-up, then (after the delay) every joint travels
// to the neutral point and holds there, so the legs settle instead of snapping at reset.
// Any pose published before the deadline cancels this.
#define SERVO_BOOT_NEUTRAL_ENABLE 1
#define SERVO_BOOT_NEUTRAL_DELAY_MS 1500
#define SERVO_BOOT_NEUTRAL_SLEW_DEG_PER_SEC 45.0f
// Keep torque only this long after the boot pose, then let the idle timeout release the servos
// (8 stalled joints draw a lot of current and can sag the servo rail).
#define SERVO_BOOT_NEUTRAL_HOLD_MS 3000

// Relax (PWM off) after this long with no new target WHILE NOT HOLDING A POSE.
// Holding a stand pose keeps torque; only idle/relaxed states time out.
#define SERVO_IDLE_RELAX_MS 8000
// 1 = drop mic frames while any servo is slewing (same idea as blue-v2's motor uplink
// pause: keeps servo whine/EMI out of the uplink). 0 = always keep the mic live.
#define BLUE_V4_PAUSE_UPLINK_WHILE_MOVING 1

// Motion command queue depth — the server sends mv:* sequences, so a few commands may
// arrive while one is still running.
#define BLUE_V4_GAIT_QUEUE_DEPTH 8

// 1 = also expose blue-v2's self.motor.* tool names so the existing server drives the
// servos with no server-side change (mv:* tags keep working). 0 = only self.gait.*/self.servo.*.
#define BLUE_V4_MOTOR_COMPAT_TOOLS 1
// Current budget guard: the Type-C 1S module supplies 5 V/2 A for logic + servos.
// The crawl gait moves one leg at a time, so at most 2 joints are loaded at once.
#define SERVO_MAX_ACTIVE_MOVING 2

// --------------------------------------------------------------------------
// Legacy blue-v2 wheel driver (MX1508) on IO11-IO14.
// blue-v4 never drives these pins, but if the old wiring is still connected the
// floating H-bridge inputs can chatter (wheels twitch, current spikes, EMI on the
// shared I2C/servo rail). 1 = drive them LOW at boot.
// Set 0 only if you repurpose those GPIOs for something else on blue-v4.
// --------------------------------------------------------------------------
#define BLUE_V4_LEGACY_MX1508_PINS_LOW 1
#define LEGACY_MX1508_IN1 GPIO_NUM_14
#define LEGACY_MX1508_IN2 GPIO_NUM_13
#define LEGACY_MX1508_IN3 GPIO_NUM_12
#define LEGACY_MX1508_IN4 GPIO_NUM_11

// ---------------------------------------------------------------------------
// VL53L0X front ToF (same behaviour/tuning as blue-v2).
// Shares the board-owned I2C bus with the PCA9685 (0x29 vs 0x40).
// ---------------------------------------------------------------------------
#define TOF_FRONT_XSHUT_GPIO GPIO_NUM_NC  // single sensor: tie XSHUT -> 3.3 V
#define TOF_FRONT_I2C_ADDR 0x29

// Safety guard while the gait runs (requires self.tof.calibrate on open floor first).
// Set BOTH to 0 to disable the stop logic for bench testing — the ToF sensor keeps
// working and self.tof.get_distance still reports.
#define TOF_OBSTACLE_GUARD_ENABLE 0
#define TOF_CLIFF_GUARD_ENABLE 0
#define TOF_GUARD_POLL_MS 50
#define TOF_MAX_VALID_MM 2000

// When calibrated: stop if dist deviates from the saved cal distance (% + absolute, stricter wins).
#define TOF_CAL_NEAR_MARGIN_PCT 25   // obstacle: dist < cal * 75%
#define TOF_CAL_FAR_MARGIN_PCT 25    // cliff: dist > cal * 125%
#define TOF_CAL_NEAR_MARGIN_MM 30    // also dist < cal - 30 mm (whichever stops sooner)
#define TOF_CAL_FAR_MARGIN_MM 50     // also dist > cal + 50 mm
#define TOF_CAL_APPROACH_STEP_MM 20  // closing fast: dist dropped this much in one poll
#define TOF_CAL_JUMP_MARGIN_MM 80    // sudden extra deviation from cal in one step

// Fallback cliff: jump from nearest reading this move leg (mm).
#define TOF_CLIFF_JUMP_MM 80
#define TOF_CLIFF_NEAR_MAX_MM 160  // jump/lost-signal cliff only if was this close first

// Ignore the obstacle guard for this long after the gait starts (I2C/vibration settle).
#define TOF_MOVE_GRACE_MS 150

// Fallback fixed thresholds when NOT calibrated yet (sensor ~128 mm to open floor).
#define TOF_OBSTACLE_STOP_MM 190
#define TOF_CLIFF_VOID_MM 200
#define TOF_CALIBRATION_DISTANCE_MM 128

// Verbose ToF / guard logs on serial (set 0 to reduce spam).
#define TOF_DEBUG_LOG 1

// ---------------------------------------------------------------------------
// Leg geometry (mm) — used by the gait engine inverse kinematics.
// Defaults suit a typical 3D-printed micro quadruped with MG90S joints.
// Tune with the real frame; these are compile-time constants, not hardcoded poses.
// ---------------------------------------------------------------------------
#define LEG_FEMUR_MM 60.0f          // hip axis -> knee axis
#define LEG_TIBIA_MM 90.0f          // knee axis -> foot contact
#define LEG_HIP_OFFSET_X_MM 45.0f   // body centre -> hip axis, fore/aft
#define LEG_HIP_OFFSET_Y_MM 55.0f   // body centre -> hip axis, left/right
#define BODY_STAND_HEIGHT_MM 95.0f  // default hip->foot vertical distance
#define BODY_MIN_HEIGHT_MM 70.0f
#define BODY_MAX_HEIGHT_MM 120.0f
#define STRIDE_LENGTH_MM 40.0f  // default crawl stride

// ---------------------------------------------------------------------------
// Joint-space crawl gait (hip/knee driven directly, no IK reach clamping).
// Gives a large, predictable travel per step for bring-up and for a simpler walk.
// 0 = use the inverse-kinematics crawl instead.
// ---------------------------------------------------------------------------
#define BLUE_V4_JOINT_SPACE_GAIT 1
#define GAIT_JOINT_HIP_NEUTRAL_DEG 90.0f  // centre of the hip sweep (mount neutral)
// Hip servo = VERTICAL yaw axis (shaft pointing down, arm pointing to the body centre, leg
// pointing outward on the body diagonal): the hip swings the foot sideways/fore-aft, so the
// FORWARD travel per step is sqrt(2) * R * sin(travel/2). With R = 70 mm: 40 deg = ~34 mm,
// 70 deg = ~57 mm, 90 deg = ~70 mm per step (90 deg = 45..135 deg on the servo, still inside
// the 1000..2000 us band), 120 deg = ~86 mm (bench only).
#define GAIT_JOINT_HIP_TRAVEL_DEG 90.0f   // walking default (override per call with hip_deg=)
#define GAIT_JOINT_KNEE_TRAVEL_DEG 60.0f  // knee fold, lifts the foot (override with knee_deg=)
#define GAIT_JOINT_STEP_MS 1400           // nominal ms per leg cycle (slow)

// Chiều đi tới: +1 = hip quét theo chiều servo tăng, -1 = đảo lại. Đổi dấu ở đây nếu robot
// đi lùi khi được lệnh đi tới (1 bước vẫn là 4 chân/8 servo, chỉ đảo chiều quét hip).
#define GAIT_JOINT_FORWARD_SIGN (-1.0f)

// --- Phase timing (per leg, in the crawl order) ---
// The commanded ramp ends on schedule but a loaded servo lands later, so every phase gets a
// short dwell before the next joint moves. Without the plant dwell the hips start rotating back
// while the foot is still in the air: the leg drags instead of pushing the body.
#define GAIT_JOINT_LIFT_DWELL_MS 40       // hold after the knee lifts, before the hip sweeps
#define GAIT_JOINT_PLANT_DWELL_MS 120     // hold after the knee lowers — the foot must be ON the floor
#define GAIT_JOINT_SETTLE_TIMEOUT_MS 700  // hard cap on a settle wait (safety)
#define GAIT_JOINT_PLANT_SLOWDOWN 1.25f   // the knee descends this much slower than it lifts
// Knee fold -> foot lift: the tibia (knee axis -> foot tip) is 60 mm, so a fold of delta raises
// the foot by about 60 * (sin(alpha + delta) - sin(alpha)) with alpha ~ the tibia angle below
// horizontal at neutral: 20 deg ~ 7 mm, 30 deg ~ 15 mm, 40 deg ~ 17 mm.
#define LEG_TIBIA_LEN_MM 60.0f
// Joint-space posture (spider geometry): extra knee fold per mm of body-height reduction.
// Approximation for the 60 mm tibia at ~45 deg: about 1.3 deg of fold per mm.
#define GAIT_JOINT_CROUCH_DEG_PER_MM 1.30f
// Knee-fold bias per degree of body pitch (positive = nose down) / roll (positive = left down).
#define GAIT_JOINT_TILT_DEG_PER_DEG 0.6f
// Distance from the hip yaw axis to the foot tip at neutral, in mm (documentation / travel maths).
#define LEG_FOOT_RADIUS_MM 70.0f

// ---------------------------------------------------------------------------
// Board behaviour
// ---------------------------------------------------------------------------
// 1 = OTA URL must come from the WiFi portal; block tenclass.net / xiaozhi.me fallback.
#define BLUE_V4_BLOCK_CLOUD_SERVERS 1
#define BLUE_V4_DEFAULT_EMOTION "neutral"

// N16R8 — DO NOT USE (module / strapping / USB-JTAG):
//   3, 19–20, 26–34, 35–37, 45–46
// GPIO 0 is BOOT (strapping, used). GPIO 1 = PCA9685 OE#. See WIRING.md.

#if defined(CONFIG_LCD_ST7789_240X240)
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 240
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR true
#define DISPLAY_RGB_ORDER LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 3
#else
#error "Blue V4 requires CONFIG_LCD_ST7789_240X240 (see main/Kconfig.projbuild -> DISPLAY_LCD_TYPE)"
#endif

#endif  // _BLUE_V4_CONFIG_H_
