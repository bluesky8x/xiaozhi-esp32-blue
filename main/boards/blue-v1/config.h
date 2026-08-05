#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

// Blue V1 — ESP32-S3-WROOM-1-N16R8 (16 MB flash, 8 MB Octal PSRAM)
// TFT ST7789 1.54" 240×240 — VuiTV wiring + touch GPIO21 + optional ToF I2C 41/42.
// OLED used touch on 17; TFT keeps BL on 17 — touch moved to 21 to avoid conflict.

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

#define AUDIO_I2S_METHOD_SIMPLEX

#ifdef AUDIO_I2S_METHOD_SIMPLEX

#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_4
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_5
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_7
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_15
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_16

#else

#define AUDIO_I2S_GPIO_WS GPIO_NUM_4
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_5
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_6
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_7

#endif

#define BUILTIN_LED_GPIO        GPIO_NUM_48
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define TOUCH_BUTTON_GPIO       GPIO_NUM_21

// SystemReset — BOOT long-press only (touch is used for mic / wake)
#define RESET_NVS_BUTTON_GPIO       GPIO_NUM_NC
#define RESET_FACTORY_BUTTON_GPIO   GPIO_NUM_NC
#define FACTORY_RESET_LONG_PRESS_MS 5000

// Power save — idle 60s → sleepy face + dim BL; touch wakes like wake word
#define POWER_SAVE_SLEEP_SECS       60

// ST7789 1.54" 240×240 SPI — CS tied to GND on module
#define DISPLAY_MOSI_PIN      GPIO_NUM_10
#define DISPLAY_CLK_PIN       GPIO_NUM_9
#define DISPLAY_DC_PIN        GPIO_NUM_8
#define DISPLAY_RST_PIN       GPIO_NUM_18
#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_17
#define DISPLAY_CS_PIN        GPIO_NUM_NC

// Decorative light (MCP self.lamp.*)
#define DECOR_LED_GPIO        GPIO_NUM_38

// MX1508 / L9110S — PWM on IN1–IN4 (no separate ENA/ENB). See motor_controller.h.
#define MOTOR_LEFT_IN1   GPIO_NUM_11
#define MOTOR_LEFT_IN2   GPIO_NUM_12
#define MOTOR_RIGHT_IN1  GPIO_NUM_13
#define MOTOR_RIGHT_IN2  GPIO_NUM_14
#define MOTOR_PWM_FREQ_HZ  20000
#define MOTOR_AUTO_STOP_MS 5000

// Optional VL53L0X / VL53L1X / VL6180X (I2C @ 0x29) — same bus as VuiTV OLED optional ToF
#define I2C_SENSOR_SDA_PIN GPIO_NUM_41
#define I2C_SENSOR_SCL_PIN GPIO_NUM_42
// Dual VL53L0X: each sensor needs its own XSHUT (cannot tie both to 3.3 V — same default addr 0x29)
#define TOF_FRONT_XSHUT_GPIO GPIO_NUM_2   // front — GPIO 2 (not a strapping pin; prefer over GPIO 3)
#define TOF_REAR_XSHUT_GPIO  GPIO_NUM_47  // rear / floor
#define TOF_FRONT_I2C_ADDR   0x29
#define TOF_REAR_I2C_ADDR    0x2A
// Single sensor: set unused XSHUT to GPIO_NUM_NC and tie that module's XSHUT → 3.3 V
//
// N16R8 avoid: 35-37 (PSRAM), 26-34 (flash/PSRAM), 0/3/45/46 (strapping), 19-20 (USB-JTAG)
// Free expand: 1, 2, 39-40, 43-44, 47  — see WIRING.md § Free GPIO

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
#define DISPLAY_SPI_MODE 0
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
