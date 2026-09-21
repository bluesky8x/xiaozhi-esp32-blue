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

#define SERVO_COUNT 8
#define SERVO_PWM_FREQ_HZ 50
#define SERVO_PWM_RESOLUTION 4096
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO_UPDATE_PERIOD_MS 20  // 50 Hz interpolation tick
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
#define GAIT_JOINT_HIP_NEUTRAL_DEG 60.0f  // gait neutral (the MOUNT neutral is still 90)
#define GAIT_JOINT_HIP_TRAVEL_DEG 120.0f  // hip travel per step (forward sweep)
#define GAIT_JOINT_KNEE_TRAVEL_DEG 60.0f  // knee fold while the leg swings
#define GAIT_JOINT_STEP_MS 1200           // default ms per leg cycle (slow)

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
