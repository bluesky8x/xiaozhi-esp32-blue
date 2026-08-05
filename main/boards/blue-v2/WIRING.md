# Blue V2 — Wiring

Firmware board: **`blue-v2`**

Build: `python scripts/build.py blue-v2` (from repo root `esp32-blue`)

**MCU:** [ESP32-S3-WROOM-1-N16R8](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf) — 16 MB flash, 8 MB Octal PSRAM.

Same **components** as [Blue V1](../blue-v1/WIRING.md) (INMP441, MAX98357, ST7789 1.54", MX1508, TTP223, optional VL53L0X) with a **pin-minimized** map so more GPIO stay free for sensors, UART, battery, etc.

---

## V1 → V2 pin changes (PCB rewire required)

| Function | Blue V1 | **Blue V2** | Why |
|----------|---------|-------------|-----|
| I2S mic WS/SCK/SD | 4 / 5 / 6 | **4 / 5 / 6** | Shared bus |
| I2S spk BCLK/LRCK/DOUT | 15 / 16 / 7 | **5 / 4 / 7** | Duplex — BCLK+WS shared with mic |
| ST7789 RST | **18** | **→ 3.3 V** (PCB) | Frees GPIO 18 |
| TTP223 touch | **21** | **16** | Frees GPIO 21 |
| Decor LED | **38** | *(removed)* | Use RGB **48** only |
| ToF XSHUT #1 / #2 | 2 / 47 | **1 / 2** | Frees GPIO 47 |
| Motor IN1–IN4 | 11–14 | **11–14** | unchanged |
| ST7789 SPI + BL | 8–10, 17 | **8–10, 17** | unchanged |
| ToF I2C | 41 / 42 | **41 / 42** | unchanged |
| BOOT / RGB | 0 / 48 | **0 / 48** | unchanged |

---

## Chân KHÔNG được dùng (N16R8)

Theo [ESP32-S3-WROOM-1 datasheet](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf):

| GPIO | Lý do |
|------|--------|
| **35, 36, 37** | Octal PSRAM nội bộ — **cấm** |
| **26–34** | Flash / PSRAM trong module — **không dùng** |
| **3** | Strapping — **không gán peripheral** (floating, no internal pull) |
| **45, 46** | Strapping — tránh mức sai lúc reset |
| **19, 20** | USB-JTAG — tránh khi debug qua USB |
| **0** | Strapping — **đã dùng cho BOOT** (pull-up OK) |

Blue V2 **không** dùng bất kỳ GPIO nào trong bảng trên (trừ GPIO 0 = BOOT).

---

## Audio — I2S duplex (4 wires + power)

INMP441 và MAX98357 **dùng chung** WS + BCLK:

| Signal | INMP441 | MAX98357 | ESP32-S3 |
|--------|---------|----------|----------|
| WS / LRCLK | WS | LRC | **4** |
| BCLK / SCK | SCK | BCLK | **5** |
| Data in | SD | — | **6** |
| Data out | — | DIN | **7** |

L/R mic → GND (left). Speaker SD → 3.3 V (always on).

---

## ST7789 1.54" 240×240 (SPI)

| Module | GPIO |
|--------|------|
| MOSI / SDA | **10** |
| SCK / SCL | **9** |
| DC | **8** |
| BLK | **17** |
| RST / RES | **3.3 V** *(not a GPIO — 10 kΩ to 3.3 V optional)* |
| CS | **GND** |

## TTP223 — GPIO **16**

| Chạm (click) | Hành vi |
|--------------|---------|
| Bình thường | Bật/tắt **mic** |
| Sleep (idle 60s) | Wake + gọi AI |

## MX1508 — GPIO 11 / 12 / 13 / 14

Same as V1 — LEDC PWM 20 kHz on IN1–IN4.

## Optional ToF (VL53L0X)

### One sensor

I2C **41 / 42**, XSHUT → **3.3 V**, address **0x29**.

### Two sensors

| | Front | Rear |
|--|-------|------|
| SDA / SCL | **41 / 42** | **41 / 42** |
| XSHUT | **GPIO 1** | **GPIO 2** |
| I2C addr | **0x29** | **0x2A** |

Driver ToF: **TODO** (same as V1).

## Status RGB — **48** only

No separate decor LED on GPIO 38. MCP `self.lamp.*` is disabled unless you add `DECOR_LED_GPIO` in `config.h`.

## BOOT **0** · Factory reset

Hold **BOOT ≥ 5 s** while running → factory reset.

---

## GPIO map (Blue V2)

| GPIO | Function |
|------|----------|
| 0 | BOOT |
| 4–7 | I2S duplex (mic + speaker) |
| 8–10, 17 | ST7789 SPI + BL |
| 11–14 | MX1508 motor |
| 16 | TTP223 touch |
| 1, 2 | ToF XSHUT *(optional, 2× VL53L0X)* |
| 41–42 | ToF I2C *(optional)* |
| 48 | RGB status LED |

---

## Free GPIO for expansion

Pins **not used** by Blue V2 firmware and **safe** on N16R8:

| GPIO | Suggested use |
|------|----------------|
| **15** | UART / GPIO *(freed from V1 speaker LRCK)* |
| **18** | UART / GPIO *(freed — RST on PCB)* |
| **21** | I2C / GPIO *(freed from V1 touch)* |
| **38** | GPIO / decor *(freed from V1 decor)* |
| **39–40** | UART (HC-05 / GNSS / co-processor) |
| **43–44** | General GPIO / SPI spare |
| **47** | Battery `CHRG` / ADC enable *(freed from V1 ToF XSHUT)* |

Also free when optional hardware omitted:

| GPIO | If unused |
|------|-----------|
| **1, 2** | No dual ToF XSHUT |
| **41–42** | No ToF I2C bus |

### Quick summary

```
AVOID:    0*, 3, 19–20†, 26–34, 35–37, 45, 46
USED:     0, 4–14, 16–17, 48  (+ 1, 2, 41–42 if ToF)
FREE:     15, 18, 21, 38, 39–40, 43–44, 47

*  GPIO 0 = BOOT
†  Tránh 19–20 khi debug USB
```

Configure optional pins in `config.h` — set `I2C_SENSOR_*`, `TOF_*`, `DECOR_LED_GPIO` to `GPIO_NUM_NC` when not wired.

---

## Pin budget vs Blue V1

| | V1 used | V2 used | V2 free (extra) |
|--|---------|---------|-----------------|
| Core GPIO | ~18 | ~14 | **+4 always** |
| With 2× ToF | +4 (2,41,42,47) | +4 (1,2,41,42) | **+47, +18, +21, +38, +15** |

Blue V2 saves **4 GPIO** in the minimum build and leaves **7 high-value expansion pins** (15, 18, 21, 38, 39–40, 43–44, 47) vs V1’s scattered free pool.
