# Blue V1 — Wiring

Firmware board: **`blue-v1`**

Build: `python scripts/build.py blue-v1` (from repo root `esp32-blue`)

**MCU:** [ESP32-S3-WROOM-1-N16R8](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf) — 16 MB flash, 8 MB Octal PSRAM. Xem [Free GPIO § Chân cần tránh](./WIRING.md#free-gpio-n16r8): **35–37** cấm; **0, 3, 45, 46** strapping; **19–20** USB-JTAG; **26–34** flash/PSRAM nội bộ.

Reference: [VuiTV Robot TFT wiring](https://iot.vuitv.dev/wiring)

---

## Map vs VuiTV official

| | VuiTV OLED | VuiTV TFT | **Blue V1** |
|--|------------|-----------|-------------|
| Mic / loa / motor | 4–6, 7/15/16, 11–14 | same | same |
| Display | I2C 41/42 | SPI 8–10, 17 BL, 18 RST | SPI + **BL 17** |
| Touch | **17** | — | **21** |
| ToF I2C | 41/42 *(opt.)* | — | **41/42** |
| Decor 1 / RGB | 38 / 48 | 38 / 48 | 38 / 48 |

**GPIO 17 = BL**, **GPIO 18 = RST** — do not wire touch or decor LED 2 to these pins.

---

## INMP441 — GPIO 4 / 5 / 6

## MAX98357 — GPIO 7 / 15 / 16

## ST7789 1.54" 240×240 (SPI)

Module labels **SCL / SDA** are SPI (not I2C).

| Module | GPIO |
|--------|------|
| MOSI / SDA | **10** |
| SCK / SCL | **9** |
| DC | **8** |
| RST / RES | **18** |
| BLK | **17** |
| CS | **GND** |

## TTP223 — GPIO **21**

| Chạm (click) | Hành vi |
|--------------|---------|
| Bình thường | Bật/tắt **mic** (mute/unmute) |
| **Sleep** (idle 60s) | Đánh thức + gọi AI (như wake word) |

## Display — Otto GIF face (240×240)

Firmware uses **otto-gif** emoji assets (animated face). Emotions change with device state and LLM replies.

| Trạng thái | GIF |
|------------|-----|
| Standby / Idle | `neutral` |
| Connecting | `thinking` |
| Listening | `thinking` |
| Speaking | `happy` |
| Power save | `sleepy` |
| LLM trả lời (emoji) | `happy`, `sad`, `angry`, … (override tạm thời) |

Build embeds GIFs from `managed_components/txp666__otto-emoji-gif-component/gifs/`.

## MX1508 — GPIO 11 / 12 / 13 / 14 (PWM on IN1–IN4)

Firmware drives speed via **LEDC PWM** on all four IN pins (20 kHz, 10-bit). No ENA/ENB on this module.

| MX1508 | ESP32-S3 | Motor |
|--------|----------|-------|
| IN1 | **11** | Left A |
| IN2 | **12** | Left B |
| IN3 | **13** | Right A |
| IN4 | **14** | Right B |

## Optional ToF (VL53L0X / VL6180X)

### Một cảm biến

Chung bus I2C **41 / 42**. **XSHUT → 3.3 V** (luôn bật). Địa chỉ mặc định **0x29**.

| VL53L0X | ESP32-S3 |
|---------|----------|
| VCC | 3.3 V |
| GND | GND |
| SDA | **41** |
| SCL | **42** |
| XSHUT | **3.3 V** |

Thêm **4.7 kΩ** pull-up SDA/SCL → 3.3 V (một cặp cho cả bus).

### Hai cảm biến VL53L0X (cùng bus)

Hai module **cùng địa chỉ 0x29** khi reset — **bắt buộc** mỗi con một chân **XSHUT** riêng, firmware bật từng con lần lượt rồi gán địa chỉ khác nhau.

| | Cảm biến 1 (trước) | Cảm biến 2 (sau) |
|--|-------------------|------------------|
| SDA / SCL | **41 / 42** *(chung)* | **41 / 42** *(chung)* |
| XSHUT | **GPIO 2** | **GPIO 47** |
| I2C sau init | **0x29** | **0x2A** |
| VCC / GND | 3.3 V / GND | 3.3 V / GND |

**Thứ tự khởi tạo (firmware):**

1. Kéo **cả hai XSHUT = LOW** → cả hai tắt.
2. **XSHUT1 = HIGH** → init sensor 1, giữ **0x29**.
3. **XSHUT2 = HIGH** → init sensor 2, đổi sang **0x2A** (`setAddress()`).
4. Đọc đo bình thường qua hai địa chỉ.

**GPIO thêm so với 1 sensor:** **2** và **47** (trước đó rảnh). Không dùng GPIO 35–37.

Gợi ý lắp: sensor 1 hướng **trước** (tránh va), sensor 2 **sàn / sau** (phát hiện bậc, theo dõi lùi).

> Driver ToF trong firmware Blue V1 vẫn **TODO** — đấu dây theo bảng trên; khi có driver sẽ đọc `TOF_*` trong `config.h`.

## Decor LED — **38** · BOOT **0** · RGB **48**

## Factory reset (`SystemReset`)

| Cách | Thao tác |
|------|----------|
| **Factory reset** | Giữ **BOOT (0)** **≥ 5 s** khi đang chạy |

Treo / không phản hồi: `esptool erase_flash` + flash lại (README).

---

## GPIO map

| GPIO | Function |
|------|----------|
| 0 | BOOT |
| 4–6 | INMP441 |
| 7, 15–16 | MAX98357 |
| 8–10, 17–18 | ST7789 |
| 11–14 | MX1508 (PWM IN1–IN4) |
| 21 | TTP223 |
| 38 | Decor LED 1 |
| 41–42 | ToF I2C *(optional)* |
| 2, 47 | ToF XSHUT *(2× VL53L0X)* |
| 48 | RGB LED |

Optional BT (VuiTV): UART **39 / 40**.

---

## Free GPIO (N16R8)

**Module:** [ESP32-S3-WROOM-1-N16R8](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf) — 16 MB flash, 8 MB Octal PSRAM.

### Chân thật sự cần tránh (N16R8)

| GPIO | Lý do |
|------|--------|
| **35, 36, 37** | Octal PSRAM nội bộ — **cấm** |
| **0, 3** | Strapping + BOOT *(Blue V1 dùng **0** cho nút BOOT)* |
| **45, 46** | Strapping |
| **19, 20** | USB-JTAG *(tránh nếu dùng USB debug)* |
| **26–34** | Flash / PSRAM trong module — không nên dùng ngoài |

Blue V1 pin map **không** dùng các nhóm trên (trừ **GPIO 0** = BOOT). Khi mở rộng, ưu tiên chân **không** nằm trong bảng này.

**Strapping chi tiết (GPIO 0, 3, 45, 46):** chỉ lấy mẫu lúc reset; sau boot hoạt động như GPIO thường. **GPIO 3** floating, không pull nội bộ — **không** gán peripheral mới. **GPIO 45/46** — tránh kéo mức sai lúc reset (đặc biệt GPIO0=LOW + GPIO46=HIGH = invalid).

### Used by Blue V1 firmware

| GPIO | Function | Notes |
|------|----------|--------|
| **0** | BOOT button | Strapping — đã dùng, OK với pull-up DevKit |
| **4–6** | INMP441 I2S mic | WS / SCK / SD |
| **7, 15–16** | MAX98357 I2S speaker | DOUT / BCLK / LRCK |
| **8–10** | ST7789 SPI | DC / SCK / MOSI (CS → GND) |
| **11–14** | MX1508 motor | IN1–IN4, LEDC PWM |
| **17** | ST7789 BL | PWM backlight |
| **18** | ST7789 RST | |
| **21** | TTP223 touch | Mic mute / wake from sleep |
| **38** | Decor LED | MCP `self.lamp.*` — optional, can set `NC` |
| **41–42** | ToF I2C *(optional)* | Free if no VL53L0X / VL6180X |
| **2, 47** | ToF XSHUT *(2× VL53L0X)* | Free if single ToF (XSHUT → 3.3 V) |
| **48** | Status RGB | DevKitC-1 v1.0 may use **38** for onboard RGB instead |

### Free for expansion (recommended)

Chỉ các chân **không** nằm trong bảng “cần tránh” ở trên.

| GPIO | Suggested use | Notes |
|------|---------------|--------|
| **1** | Battery ADC | Divider 100k/100k từ TP4056 |
| **2** | ToF XSHUT #1 | VL53L0X trước *(ưu tiên thay GPIO 3)* |
| **39–40** | UART BT *(VuiTV)* | HC-05 / serial |
| **43–44** | Spare | GPIO tổng quát |
| **47** | ToF XSHUT #2 / CHRG | VL53L0X sau; hoặc TP4056 `CHRG` |

### Free only if optional hardware omitted

| GPIO | If unused |
|------|-----------|
| **38** | No decor LED → free (DevKit onboard RGB on v1.1) |
| **41–42** | No ToF sensor → free I2C bus |
| **48** | No external RGB → free if using **38** for status |

### Quick summary

```
AVOID:    0*, 3, 19–20†, 26–34, 35–37, 45, 46
USED:     0, 4–18, 21, 38, 41–42‡, 48  (+ 2, 47 if 2× VL53L0X)
FREE:     1, 2, 39–40, 43–44, 47  (2, 47 used if 2× ToF)
OPTIONAL: 38, 41, 42, 48  (‡41–42 free if no ToF)

*  GPIO 0 = BOOT (strapping, đã dùng)
†  Tránh 19–20 khi debug qua USB
```

\*Configure in `config.h` — set `DECOR_LED_GPIO`, `I2C_SENSOR_*` to `GPIO_NUM_NC` when not wired.

### Planned / discussed (not in firmware yet)

| Feature | Suggested GPIO |
|---------|----------------|
| Battery ADC (TP4056 divider) | **1** |
| Charge status `CHRG` | **47** or **38** |
| Motor ENA/ENB (L298N only) | **38** + **47** — N/A for MX1508 |
| ToF `XSHUT` (2× VL53L0X) | **2** + **47** |
