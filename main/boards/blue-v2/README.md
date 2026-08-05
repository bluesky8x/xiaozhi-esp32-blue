# Blue V2

Pin-optimized successor to [Blue V1](../blue-v1/README.md) — **same hardware stack**, fewer GPIO used, more pins free for expansion.

Based on [ESP32-S3-WROOM-1-N16R8](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf) constraints (avoid **35–37**, **26–34**, strapping **3/45/46**, USB-JTAG **19–20**).

## What changed vs V1

| Optimization | Benefit |
|--------------|---------|
| I2S **duplex** (GPIO 4–7) | Frees **15, 16** |
| LCD RST → **3.3 V** on PCB | Frees **18** |
| Touch → **GPIO 16** | Frees **21** |
| No decor LED (RGB **48** only) | Frees **38** |
| ToF XSHUT → **1 / 2** | Frees **47** |

Full wiring: [WIRING.md](./WIRING.md)

## Hardware (unchanged components)

| Component | Notes |
|-----------|--------|
| MCU | ESP32-S3-WROOM-1-N16R8 |
| Display | ST7789 1.54" 240×240, Otto GIF |
| Audio | INMP441 + MAX98357 (shared I2S) |
| Motor | MX1508 PWM on GPIO 11–14 |
| Touch | TTP223 on **GPIO 16** |
| ToF *(opt.)* | I2C 41/42, XSHUT 1/2 |

## Build

```bash
cd esp32-blue
source ~/esp/esp-idf/export.sh
python scripts/build.py blue-v2
```

menuconfig: **Board type → Blue V2 (ST7789 1.54" robot)**

Flash:

```bash
idf.py -p /dev/cu.usbmodem* flash monitor
```

## QEMU

See [QEMU.md](./QEMU.md) — same as V1 but `python scripts/build.py blue-v2`.

## MCP / server

Same motor protocol as V1 — [esp32-server-blue/BLUE.md](../../../../esp32-server-blue/BLUE.md).

## Migration from V1 PCB

**Not pin-compatible.** Rewire per [WIRING.md § V1 → V2](./WIRING.md#v1--v2-pin-changes-pcb-rewire-required). Existing V1 boards keep using `blue-v1` firmware.
