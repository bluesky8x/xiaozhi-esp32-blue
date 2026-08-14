#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// Blue V2 — ESP32-S3-WROOM-1-N16R8 (16 MB flash, 8 MB Octal PSRAM)
// Same hardware as Blue V1 (ST7789 1.54", INMP441, MAX98357, MX1508, TTP223)
// with a pin-minimized map — see WIRING.md vs blue-v1.
//
// Optimizations vs V1:
//   • I2S duplex (4 pins) instead of simplex mic+spk (6 pins)
//   • LCD RST tied to 3.3 V on PCB (no GPIO)
//   • Decor LED dropped — status RGB on GPIO 48 only
//   • ToF XSHUT on GPIO 1 / 2 (frees 47 for battery / expansion)

#define AUDIO_INPUT_SAMPLE_RATE  16000
// Duplex I2S uses one BCLK/WS clock (see no_audio_codec.cc) — must match mic + Xiaozhi Opus (16 kHz).
// 24 kHz caused server→device resampling and noisy TTS on MAX98357.
#define AUDIO_OUTPUT_SAMPLE_RATE 16000

// Shared I2S bus: INMP441 + MAX98357 (duplex)
#define AUDIO_I2S_GPIO_WS   GPIO_NUM_4
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_5
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_6   // mic SD
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_7   // speaker DIN

#define BUILTIN_LED_GPIO        GPIO_NUM_48
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define TOUCH_BUTTON_GPIO       GPIO_NUM_16
// TTP223 SIG goes HIGH when touched (most modules). Set 0 if your module is active-low.
#define TOUCH_BUTTON_ACTIVE_HIGH 1
#define TOUCH_DEBOUNCE_MS       350

#define RESET_NVS_BUTTON_GPIO       GPIO_NUM_NC
#define RESET_FACTORY_BUTTON_GPIO   GPIO_NUM_NC
#define FACTORY_RESET_LONG_PRESS_MS 5000

// -1 = no auto sleep (backlight dim looked like "display off" on ST7789 during bring-up).
// Set to 60 for battery use when display sleep is desired.
#define POWER_SAVE_SLEEP_SECS (-1)
// Minimum backlight when entering sleep via MCP (not 1% — nearly invisible on 1.54" LCD).
#define POWER_SAVE_DIM_BRIGHTNESS 25

// LVGL draw buffer height (lines). Internal DMA SRAM — avoid full-frame PSRAM flush (SPI DMA OOM + WDT).
#define DISPLAY_LVGL_BUFFER_LINES 80

// 1 = animated color-bar test (same as blue-v3, use while LCD-only on bench).
// 0 = Otto emoji UI (full robot — mic/motor must be wired).
#define BLUE_V2_LCD_TEST_SCREEN 0

// 1 = BlueV3TestDisplay driver (proven visible on this ST7789 + INVERT hardware).
// 0 = experimental emoji/face UI.
#define BLUE_V2_USE_V3_DISPLAY 0

// 1 = MCU + LCD bench: Otto GIF — no motor, ToF, or robot MCP tools.
// WiFi/OTA still run; use to verify Otto assets without SPI/DMA interference.
#define BLUE_V2_OTTO_LCD_ONLY 1

// 1 = add INMP441 + MAX98357 (I2S deferred until after activation — protects LCD SPI).
// Requires BLUE_V2_OTTO_LCD_ONLY=1. Wake word stays off; use boot/touch to start chat.
#define BLUE_V2_OTTO_AUDIO_ENABLE 1
// Requires BLUE_V2_OTTO_LCD_ONLY=1. Registers self.motor.* / self.chassis.* MCP tools.
#define BLUE_V2_OTTO_MOTOR_ENABLE 1
// Requires BLUE_V2_OTTO_LCD_ONLY=1. VL53L0X on I2C 41/42 + self.tof.* MCP; guard needs motor.
#define BLUE_V2_OTTO_TOF_ENABLE     1

// 1 = TTP223 on GPIO 16 — toggle chat (same as BOOT). Works without wake word.
#define BLUE_V2_OTTO_TOUCH_ENABLE 1
#define BLUE_V2_OTTO_TOUCH_ENABLE 1

// 1 = OTA URL from WiFi portal only; block tenclass.net / xiaozhi.me fallback (no cloud Chinese TTS).
#define BLUE_V2_BLOCK_CLOUD_SERVERS 1

#if BLUE_V2_OTTO_LCD_ONLY && BLUE_V2_OTTO_AUDIO_ENABLE
#define BLUE_V2_OTTO_DEFER_HEAVY 1
#endif

#define BLUE_V2_DEFAULT_EMOTION "neutral"

#define DISPLAY_MOSI_PIN      GPIO_NUM_10
#define DISPLAY_CLK_PIN       GPIO_NUM_9
#define DISPLAY_DC_PIN        GPIO_NUM_8
// Breadboard: module pin 5 (RES) → GPIO 18. PCB with RES→3.3V: use GPIO_NUM_NC.
#define DISPLAY_RST_PIN       GPIO_NUM_18
#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_17
#define DISPLAY_CS_PIN        GPIO_NUM_NC

// No separate decor GPIO — use RGB 48 for status; MCP self.lamp optional on future PCB
#define DECOR_LED_GPIO        GPIO_NUM_NC

// MX1508 — IN1–IN4 as digital GPIO (no LEDC — avoids ST7789 SPI conflict on Blue V2).
#define MOTOR_USE_GPIO_ONLY 1
#define MOTOR_LEFT_IN1   GPIO_NUM_14
#define MOTOR_LEFT_IN2   GPIO_NUM_13
#define MOTOR_RIGHT_IN1  GPIO_NUM_12
#define MOTOR_RIGHT_IN2  GPIO_NUM_11
#define MOTOR_AUTO_STOP_MS 5000
// Embedded dance1.ogg / dance2.ogg — keep server mv:d / mv:d2 cooldown in sync.
#define MOTOR_DANCE1_DURATION_MS 23500
#define MOTOR_DANCE2_DURATION_MS 99500
#define MOTOR_DANCE3_DURATION_MS 25100
#define MOTOR_DANCE_DURATION_MS MOTOR_DANCE1_DURATION_MS
// Active brake (IN1=IN2=HIGH) before coast on stop — FS_MX1508 motorBrake() pattern, reduces mechanical jitter.
#define MOTOR_BRAKE_ENABLE 1
#define MOTOR_BRAKE_MS     50
// Cap for future LEDC/SLOW_DECAY (Phase B). GPIO on/off still full speed when non-zero.
#define MOTOR_MAX_DUTY_PCT 70

// INMP441 (L/R→GND): AGC + soft compressor in no_audio_codec (see AUDIO_MIC_AGC_* in CMakeLists).
#define AUDIO_MIC_SHIFT_BITS      12
#define AUDIO_MIC_INPUT_GAIN      1.0f
#define AUDIO_MIC_SOFT_LIMIT      24000

// Mic level debug on serial (~1 Hz): peak/RMS while I2S input active (Blue V2 bring-up).
#define AUDIO_MIC_DEBUG_LOG         1

// VL53L0X front sensor (set NC to skip I2C init).
#define I2C_SENSOR_SDA_PIN GPIO_NUM_41
#define I2C_SENSOR_SCL_PIN GPIO_NUM_42
#define TOF_FRONT_XSHUT_GPIO GPIO_NUM_NC   // single sensor: tie XSHUT → 3.3 V; dual: use GPIO 1 / 2
#define TOF_REAR_XSHUT_GPIO  GPIO_NUM_2
#define TOF_FRONT_I2C_ADDR   0x29
#define TOF_REAR_I2C_ADDR    0x2A

// Safety guard while motors run (requires self.tof.calibrate on open floor first).
#define TOF_OBSTACLE_GUARD_ENABLE 1
#define TOF_CLIFF_GUARD_ENABLE    1

// When calibrated: stop if dist deviates from saved cal distance (% + absolute, stricter wins).
#define TOF_CAL_NEAR_MARGIN_PCT   25    // obstacle: dist < cal * 75%
#define TOF_CAL_FAR_MARGIN_PCT    25    // cliff: dist > cal * 125%
#define TOF_CAL_NEAR_MARGIN_MM    30    // also dist < cal - 30 mm (whichever stops sooner)
#define TOF_CAL_FAR_MARGIN_MM     50    // also dist > cal + 50 mm
#define TOF_CAL_APPROACH_STEP_MM  20    // closing fast: dist dropped this much in one poll
#define TOF_CAL_JUMP_MARGIN_MM    80    // sudden extra deviation from cal in one step

// Fallback cliff: jump from nearest reading this move leg (mm).
#define TOF_CLIFF_JUMP_MM         150
#define TOF_CLIFF_NEAR_MAX_MM     280   // jump/lost-signal cliff only if was this close first

// Ignore guard for this long after motor start (I2C/vibration settle); cliff still checked.
#define TOF_MOVE_GRACE_MS         150

// Fallback fixed thresholds when NOT calibrated yet:
#define TOF_OBSTACLE_STOP_MM      120
#define TOF_CLIFF_VOID_MM         450

// Rear/down sensor (optional):
#define TOF_CLIFF_FLOOR_MAX_MM    120
#define TOF_REAR_SENSOR_ENABLE    0

#define TOF_GUARD_POLL_MS         50
#define TOF_MAX_VALID_MM          2000

// Verbose ToF / obstacle-guard logs on serial monitor (set 0 to reduce spam).
#define TOF_DEBUG_LOG             1
#define TOF_DEBUG_IDLE_LOG_MS     1000   // log range while motors idle
#define TOF_DEBUG_MOVE_LOG_MS     150    // log range while any motor motion

// Calibrate on open floor at normal travel distance (e.g. 300–450 mm on a table).
#define TOF_CALIBRATION_DISTANCE_MM 400

// N16R8 — DO NOT USE (module / strapping / USB-JTAG):
//   3, 19–20, 26–34, 35–37, 45–46
// GPIO 0 = BOOT (strapping, used). See WIRING.md.

#ifdef CONFIG_LCD_ST7789_240X240
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  240
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 3
#endif

#ifdef CONFIG_LCD_ST7789_240X135
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  135
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY true
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  40
#define DISPLAY_OFFSET_Y  53
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

#ifndef DISPLAY_WIDTH
#error "Select LCD type in menuconfig: ST7789 240*240 or ST7789 240*135"
#endif

#endif  // _BOARD_CONFIG_H_
