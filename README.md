# esp32-blue

Firmware for **Blue V1** — ESP32-S3 voice robot (Xiaozhi / 小智).

| Spec | Value |
|------|--------|
| MCU | ESP32-S3-WROOM-1-N16R8 (16 MB flash, 8 MB Octal PSRAM) |
| Display | ST7789 1.54" 240×240, Otto GIF face |
| Motor | MX1508 (PWM on GPIO 11–14) |
| Audio | INMP441 + MAX98357 |

## Build

```bash
cd esp32-blue
source ~/esp/esp-idf/export.sh   # ESP-IDF 5.5+ / 6.x
python scripts/build.py blue-v1
```

Flash:

```bash
idf.py -p /dev/cu.usbmodem* flash monitor
```

## QEMU (no hardware)

Run in ESP-IDF QEMU — full steps in [main/boards/blue-v1/QEMU.md](main/boards/blue-v1/QEMU.md).

```bash
pkill -9 qemu-system-xtensa 2>/dev/null
pkill -f "idf.py qemu" 2>/dev/null
export IDF_PYTHON_ENV_PATH=~/.espressif/python_env/idf6.0_py3.12_env
source ~/esp/esp-idf/export.sh
cd ~/work/xiaozhi-esp32   # or ~/work/esp32-blue after build
idf.py qemu
```

Pair with [esp32-server-blue](../esp32-server-blue/BLUE.md) WebSocket on your LAN.

## Docs

- [Blue V2 board](main/boards/blue-v2/README.md) — **new PCB (recommended)**
- [Blue V2 wiring / pin map](main/boards/blue-v2/WIRING.md)
- [Blue V1 board](main/boards/blue-v1/README.md) — legacy pinout
- [QEMU simulator](main/boards/blue-v2/QEMU.md)
- [Wiring / GPIO (V1)](main/boards/blue-v1/WIRING.md)
- [Backend server](../esp32-server-blue/BLUE.md) — Xiaozhi + Kira character (`esp32-server-blue`)

## Stack

| Component | Path |
|-----------|------|
| Firmware (this repo) | `esp32-blue` — boards **`blue-v1`** (legacy PCB) · **`blue-v2`** (pin-optimized, recommended for new PCB) |
| Python server | `esp32-server-blue` — run `./run.sh` after config |

## Ported from

Board profile migrated from `xiaozhi-esp32/main/boards/blue-v1`.
