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
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// Shared I2S bus: INMP441 + MAX98357 (duplex)
#define AUDIO_I2S_GPIO_WS   GPIO_NUM_4
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_5
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_6   // mic SD
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_7   // speaker DIN

#define BUILTIN_LED_GPIO        GPIO_NUM_48
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define TOUCH_BUTTON_GPIO       GPIO_NUM_16

#define RESET_NVS_BUTTON_GPIO       GPIO_NUM_NC
#define RESET_FACTORY_BUTTON_GPIO   GPIO_NUM_NC
#define FACTORY_RESET_LONG_PRESS_MS 5000

#define POWER_SAVE_SLEEP_SECS 60

// ST7789 1.54" 240×240 SPI — CS tied to GND; RST tied to 3.3 V on PCB
#define DISPLAY_MOSI_PIN      GPIO_NUM_10
#define DISPLAY_CLK_PIN       GPIO_NUM_9
#define DISPLAY_DC_PIN        GPIO_NUM_8
#define DISPLAY_RST_PIN       GPIO_NUM_NC
#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_17
#define DISPLAY_CS_PIN        GPIO_NUM_NC

// No separate decor GPIO — use RGB 48 for status; MCP self.lamp optional on future PCB
#define DECOR_LED_GPIO        GPIO_NUM_NC

// MX1508 — PWM on IN1–IN4
#define MOTOR_LEFT_IN1   GPIO_NUM_11
#define MOTOR_LEFT_IN2   GPIO_NUM_12
#define MOTOR_RIGHT_IN1  GPIO_NUM_13
#define MOTOR_RIGHT_IN2  GPIO_NUM_14
#define MOTOR_PWM_FREQ_HZ  20000
#define MOTOR_AUTO_STOP_MS 5000

// Optional VL53L0X / VL6180X — shared I2C
#define I2C_SENSOR_SDA_PIN GPIO_NUM_41
#define I2C_SENSOR_SCL_PIN GPIO_NUM_42
#define TOF_FRONT_XSHUT_GPIO GPIO_NUM_1
#define TOF_REAR_XSHUT_GPIO  GPIO_NUM_2
#define TOF_FRONT_I2C_ADDR   0x29
#define TOF_REAR_I2C_ADDR    0x2A

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
