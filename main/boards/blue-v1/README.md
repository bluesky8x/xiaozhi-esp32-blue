# Blue V1

Xiaozhi board profile for **Blue V1** — ESP32-S3 voice robot, **ST7789 1.54" TFT 240×240**, **MX1508** motor driver, no camera.

## MCU

**ESP32-S3-WROOM-1-N16R8** (same module family as Kita V1 Pro)

| Spec | Value |
|------|--------|
| Flash | 16 MB Quad SPI |
| PSRAM | **8 MB Octal SPI** (OPI) |
| Supply | 3.0–3.6 V (**3.3 V**) |
| Temp | –40 ~ 65 °C (R8 series) |

Datasheet: [ESP32-S3-WROOM-1](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf)

Firmware: `CONFIG_SPIRAM_MODE_OCT=y` in `sdkconfig.defaults.esp32s3`.

**Module constraint (N16R8):** tránh **35–37** (PSRAM), **26–34** (flash/PSRAM), **0/3/45/46** (strapping), **19–20** (USB-JTAG khi debug). Chi tiết: [WIRING.md § Free GPIO](./WIRING.md#free-gpio-n16r8).

Pin map based on [VuiTV Robot TFT wiring](https://iot.vuitv.dev/wiring) with touch on **GPIO 21** and optional ToF on **41/42**.

## So sánh với preset VuiTV

| Chức năng | OLED | TFT (VuiTV) | **Blue V1** |
|-----------|------|-------------|-------------|
| Mic / loa / motor / BOOT / RGB / đèn 38 | ✅ | ✅ | ✅ |
| ST7789 SPI + BL **17** | — | ✅ | ✅ |
| Touch TTP223 | **17** | ❌ | **21** |
| ToF I2C (VL53L0X…) | **41 / 42** | — | **41 / 42** |
| Đèn trang trí 2 | **18** | → RST | **18 = RST** |

## Hardware

| Component | GPIO |
|-----------|------|
| INMP441 WS / SCK / SD | 4 / 5 / 6 |
| MAX98357 DOUT / BCLK / LRCK | 7 / 15 / 16 |
| ST7789 **1.54"** MOSI / SCK / DC / RST / BL | 10 / 9 / 8 / 18 / 17 |
| Display face | **Otto GIF** (otto-gif assets) |
| TTP223 (touch) | **21** |
| VL53L0X / VL6180X SDA / SCL *(optional)* | **41 / 42** |
| MX1508 IN1–IN4 (PWM) | 11 / 12 / 13 / 14 |
| Decorative LED 1 | 38 |
| BOOT | 0 |
| RGB LED | 48 |

Full wiring: [WIRING.md](./WIRING.md) — includes **free GPIO** map for N16R8.

## Build

```bash
cd esp32-blue
python scripts/build.py blue-v1
```

menuconfig: **Board type → Blue V1 (TFT robot)**, **LCD → ST7789 240×240** (1.54" panel).

## QEMU (no hardware)

See [QEMU.md](./QEMU.md). Quick start after build:

```bash
pkill -9 qemu-system-xtensa 2>/dev/null
pkill -f "idf.py qemu" 2>/dev/null
export IDF_PYTHON_ENV_PATH=~/.espressif/python_env/idf6.0_py3.12_env
source ~/esp/esp-idf/export.sh
cd ~/work/xiaozhi-esp32
idf.py qemu
```

Backend: [esp32-server-blue/BLUE.md](../../../../esp32-server-blue/BLUE.md).

## Interaction

- **BOOT (0):** click → chat / Wi‑Fi config at startup; **hold 5 s** → factory reset
- **Touch (21):** click → **mute/unmute mic**; in **sleep mode** → wake (like wake word)
- **Sleep:** idle **60 s** → sleepy face + dim backlight (`PowerSaveTimer`)

## Factory reset (`SystemReset`)

| Cách | Thao tác |
|------|----------|
| **Factory reset** | Giữ **BOOT (0)** **≥ 5 s** khi đang chạy |

**Không** dùng chạm lúc bật nguồn để reset (TTP223 dễ kích hoạt nhầm). Treo cứng: `esptool erase_flash` + flash lại.

## MCP tools

- Motor: `mv:t|p|f|b|s` in LLM reply → server parses → MCP motor (see [esp32-server-blue BLUE.md](../../../../esp32-server-blue/BLUE.md#robot-motion-mv))
- Multi-step: max **3** moves per reply, **5 s** between steps (server config)
- **Web test** (`http://127.0.0.1:8006/index.html`): motor tools are simulated in browser — refresh page + reconnect WebSocket after updates
- Decor: `self.lamp.*`
- Screen: `self.screen.set_brightness`
- ToF: driver **TODO**

## vs Kita V1 Pro

Do not use on Kita hardware (OV3660 FPC).
