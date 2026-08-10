# Blue V2 — Pinout & Wiring

Firmware board: **`blue-v2`**

Build: `python scripts/build.py blue-v2` (from repo root `esp32-blue`)

**MCU:** [ESP32-S3-WROOM-1-N16R8](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf) — 16 MB flash, 8 MB Octal PSRAM.

Cùng **linh kiện** với [Blue V1](../blue-v1/WIRING.md) (INMP441, MAX98357, ST7789 1.54", MX1508, TTP223, VL53L0X) nhưng **sơ đồ chân tối ưu** — xem [V1 → V2](#v1--v2-thay-doi-chân).

Nguồn tham chiếu firmware: `main/boards/blue-v2/config.h`

---

## Tổng hợp pinout (MCU ↔ thiết bị)

| # | Thiết bị | Tín hiệu module | ESP32-S3 GPIO | Ghi chú |
|---|----------|-----------------|---------------|---------|
| 1 | **INMP441** (mic I2S) | WS / LRC | **4** | Bus I2S duplex — dùng chung với loa |
| | | SCK / BCLK | **5** | |
| | | SD (data out) | **6** | |
| | | L/R | **GND** | Chọn kênh trái |
| | | VDD / GND | **3.3 V / GND** | |
| 2 | **MAX98357** (loa I2S) | LRC / WS | **4** | Cùng WS với mic |
| | | BCLK | **5** | Cùng BCLK với mic |
| | | DIN (data in) | **7** | |
| | | SD (shutdown) | **3.3 V** | Luôn bật |
| | | GAIN | GND hoặc NC | Mức gain mặc định |
| | | VIN / GND | **3.3 V / GND** | |
| 3 | **ST7789** 1.54" 240×240 (SPI) | SDA / MOSI | **10** | SPI3 — nhãn SDA trên module = MOSI |
| | | SCL / SCK | **9** | |
| | | DC | **8** | |
| | | BLK (backlight) | **17** | PWM backlight |
| | | RES / RST | **18** *hoặc* **3.3 V** | Xem [LCD reset](#3-st7789-154-240240-spi) |
| | | CS | **GND** | CS nối mass — firmware `DISPLAY_CS_PIN = NC` |
| | | VCC / GND | **3.3 V / GND** | |
| 4 | **TTP223** (touch) | SIG / OUT | **16** | Click = toggle chat / wake |
| | | VCC / GND | **3.3 V / GND** | |
| 5 | **MX1508** (motor driver) | IN1 | **14** | Bánh trái A — GPIO on/off (no PWM) |
| | | IN2 | **13** | Bánh trái B |
| | | IN3 | **12** | Bánh phải A |
| | | IN4 | **11** | Bánh phải B |
| | | VCC / GND | Nguồn motor riêng + GND chung MCU | **Không** cấp 5 V motor từ 3.3 V MCU |
| 6 | **VL53L0X** (ToF, 1 cảm biến) | SDA | **41** | I2C bus sensor |
| | | SCL | **42** | |
| | | XSHUT | **3.3 V** | Luôn HIGH — firmware `TOF_FRONT_XSHUT_GPIO = NC` |
| | | VCC / GND | **3.3 V / GND** | |
| | | I2C addr | **0x29** | |
| 7 | **VL53L0X #2** *(tùy chọn, chưa lắp)* | SDA / SCL | **41 / 42** | Chung bus với sensor 1 |
| | | XSHUT (trước) | **GPIO 1** | Bật `TOF_FRONT_XSHUT_GPIO = 1` |
| | | XSHUT (sau, hướng xuống) | **GPIO 2** | Bật `TOF_REAR_SENSOR_ENABLE = 1` |
| | | I2C addr | **0x29 / 0x2A** | Sau init firmware |
| 8 | **WS2812 RGB** (status) | DIN | **48** | 1 pixel GRB — `SingleLed` |
| | | VCC / GND | **3.3 V / GND** | |
| 9 | **BOOT** (nút trên module) | — | **0** | Giữ ≥ 5 s → factory reset |

**Pull-up I2C (41/42):** thêm **4.7 kΩ** SDA→3.3 V và SCL→3.3 V (một cặp cho cả bus ToF).

---

## Chi tiết từng thiết bị

### 1. INMP441 — microphone I2S

| Chân INMP441 | Nối tới |
|--------------|---------|
| WS | GPIO **4** |
| SCK | GPIO **5** |
| SD | GPIO **6** |
| L/R | **GND** (left channel) |
| VDD | **3.3 V** |
| GND | **GND** |

### 2. MAX98357 — amplifier I2S

| Chân MAX98357 | Nối tới |
|---------------|---------|
| LRC | GPIO **4** *(chung WS mic)* |
| BCLK | GPIO **5** *(chung BCLK mic)* |
| DIN | GPIO **7** |
| SD | **3.3 V** *(always on)* |
| VIN | **3.3 V** |
| GND | **GND** |

> **Blue V2 vs V1:** V1 dùng LRCK/BCLK riêng trên GPIO **15/16** cho loa. V2 gộp WS+BCLK (**4/5**) — duplex 4 dây + nguồn.

### 3. ST7789 1.54" 240×240 (SPI)

Nhãn **SCL / SDA** trên module là **SPI**, không phải I2C.

| Chân module | ESP32-S3 | Ghi chú |
|-------------|----------|---------|
| SDA (MOSI) | **10** | `DISPLAY_MOSI_PIN` |
| SCL (SCK) | **9** | `DISPLAY_CLK_PIN` |
| DC | **8** | `DISPLAY_DC_PIN` |
| BLK | **17** | `DISPLAY_BACKLIGHT_PIN` |
| CS | **GND** | `DISPLAY_CS_PIN = NC` |
| RES / RST | **GPIO 18** *hoặc* **3.3 V** | Tùy cách lắp |
| VCC | **3.3 V** | |
| GND | **GND** | |

**LCD reset — chọn một:**

| Cách lắp | RES nối | `config.h` |
|----------|---------|------------|
| **Breadboard** *(hiện tại)* | GPIO **18** | `DISPLAY_RST_PIN GPIO_NUM_18` |
| **PCB** *(RES kéo 3.3 V)* | **3.3 V** (+ 10 kΩ tùy chọn) | `DISPLAY_RST_PIN GPIO_NUM_NC` |

Firmware gọi reset phần cứng qua GPIO 18 khi pin ≠ `NC`; nếu RES đã nối 3.3 V thì đặt `NC`.

### 4. TTP223 — cảm ứng điện dung

| Chân TTP223 | Nối tới |
|-------------|---------|
| SIG / I/O | GPIO **16** |
| VCC | **3.3 V** |
| GND | **GND** |

| Chạm (click) | Hành vi firmware |
|--------------|------------------|
| Mọi lúc (trừ sleep) | Bắt đầu / dừng trò chuyện *(giống BOOT)* |
| Sleep (idle 60 s) | Wake + gọi AI |

> Không dùng chạm để mute mic khi đang nghe — tránh treo I2S duplex trên breadboard.

### 5. MX1508 — motor driver (GPIO on/off)

| Chân MX1508 | ESP32-S3 | Motor |
|-------------|----------|-------|
| IN1 | **14** | Trái A |
| IN2 | **13** | Trái B |
| IN3 | **12** | Phải A |
| IN4 | **11** | Phải B |

- Firmware bật/tắt IN1–IN4 bằng GPIO (`MOTOR_USE_GPIO_ONLY=1`) — không LEDC, tránh xung đột SPI màn hình
- Dấu speed chỉ chọn chiều quay; không điều tốc (full ON khi di chuyển)
- Tự dừng sau **5 s** mặc định (`MOTOR_AUTO_STOP_MS`)
- OUT1/OUT2 → motor trái; OUT3/OUT4 → motor phải

### 6. VL53L0X — cảm biến khoảng cách (ToF)

#### Setup hiện tại — **1 cảm biến phía trước**

| Chân VL53L0X | Nối tới |
|--------------|---------|
| VCC | **3.3 V** |
| GND | **GND** |
| SDA | GPIO **41** |
| SCL | GPIO **42** |
| XSHUT | **3.3 V** *(luôn bật)* |
| GPIO1 | *(không dùng)* |
| I2C address | **0x29** |

`config.h`:
```c
#define I2C_SENSOR_SDA_PIN      GPIO_NUM_41
#define I2C_SENSOR_SCL_PIN      GPIO_NUM_42
#define TOF_FRONT_XSHUT_GPIO    GPIO_NUM_NC
#define TOF_REAR_SENSOR_ENABLE  0
```

#### Setup tương lai — **2 cảm biến** (trước + sau hướng xuống)

| | Sensor trước | Sensor sau (sàn) |
|--|-------------|------------------|
| SDA / SCL | **41 / 42** *(chung)* | **41 / 42** *(chung)* |
| XSHUT | **GPIO 1** | **GPIO 2** |
| Hướng lắp | Phía trước (va chạm) | Hướng xuống sàn (hố sâu) |
| I2C sau init | **0x29** | **0x2A** |

Bật trong `config.h`:
```c
#define TOF_FRONT_XSHUT_GPIO    GPIO_NUM_1
#define TOF_REAR_XSHUT_GPIO     GPIO_NUM_2
#define TOF_REAR_SENSOR_ENABLE  1
```

**Thứ tự init firmware (2 sensor):** cả hai XSHUT LOW → bật front → addr 0x29 → bật rear → addr 0x2A.

**MCP / driver:** `TofController` — `self.tof.calibrate`, `self.tof.get_distance`. Cal lưu NVS namespace `blue_tof`.

#### An toàn khi đi thẳng (`tof_motor_guard`)

1. Calibrate trên **sàn trống phía trước** (~400 mm): `self.tof.calibrate` / `tof:cal:400`
2. Khi đã cal, dừng nếu lệch so với `cal_ref`:

| Case | Điều kiện |
|------|-----------|
| Vật cản | dist < cal×75% hoặc < cal−30 mm, hoặc đóng nhanh ≥ 20 mm/bước |
| Hố / mép bàn | dist > cal×125% hoặc > cal+50 mm |
| Hố sâu (sensor sau) | floor > 120 mm — cần sensor #2 |

Chưa calibrate → fallback: obstacle ≤ 120 mm, void ≥ 450 mm.

### 7. WS2812 RGB — đèn trạng thái

| Chân WS2812 | Nối tới |
|-------------|---------|
| DIN | GPIO **48** |
| VCC | **3.3 V** |
| GND | **GND** |

1 pixel, định dạng **GRB** (`LED_MODEL_WS2812`). Không có decor LED riêng trên GPIO 38 — MCP `self.lamp.*` tắt (`DECOR_LED_GPIO = NC`).

### 8. BOOT — nút factory reset

| Chân | GPIO | Hành vi |
|------|------|---------|
| BOOT (trên devkit) | **0** | Giữ **≥ 5 s** khi đang chạy → factory reset |

---

## V1 → V2 thay đổi chân

| Chức năng | Blue V1 | **Blue V2** |
|-----------|---------|-------------|
| I2S mic WS/SCK/SD | 4 / 5 / 6 | **4 / 5 / 6** |
| I2S spk BCLK/LRCK/DOUT | 15 / 16 / 7 | **5 / 4 / 7** *(duplex)* |
| ST7789 RST | **18** | **18** *(breadboard)* hoặc **3.3 V** *(PCB)* |
| TTP223 | **21** | **16** |
| Decor LED | **38** | *(bỏ — dùng RGB 48)* |
| ToF XSHUT #1 / #2 | 2 / 47 | **1 / 2** |
| Motor IN1–IN4 | 11–14 | **11–14** |
| ST7789 SPI + BL | 8–10, 17 | **8–10, 17** |
| ToF I2C | 41 / 42 | **41 / 42** |
| BOOT / RGB | 0 / 48 | **0 / 48** |

---

## Chân KHÔNG được dùng (N16R8)

| GPIO | Lý do |
|------|--------|
| **35, 36, 37** | Octal PSRAM nội bộ — **cấm** |
| **26–34** | Flash / PSRAM trong module |
| **3** | Strapping — không gán peripheral |
| **45, 46** | Strapping — tránh mức sai lúc reset |
| **19, 20** | USB-JTAG — tránh khi debug qua USB |
| **0** | Strapping — **đã dùng cho BOOT** |

Blue V2 **không** dùng GPIO trong bảng trên (trừ **0** = BOOT).

---

## GPIO map nhanh

| GPIO | Thiết bị / chức năng |
|------|----------------------|
| **0** | BOOT (factory reset) |
| **4** | I2S WS — mic + loa |
| **5** | I2S BCLK — mic + loa |
| **6** | I2S mic SD (IN) |
| **7** | I2S loa DIN (OUT) |
| **8** | LCD DC |
| **9** | LCD SCK |
| **10** | LCD MOSI |
| **11–14** | MX1508 IN4–IN1 *(11=IN4, 12=IN3, 13=IN2, 14=IN1)* |
| **16** | TTP223 touch |
| **17** | LCD backlight BLK |
| **18** | LCD RST *(breadboard; PCB → 3.3 V, GPIO NC)* |
| **41** | ToF I2C SDA |
| **42** | ToF I2C SCL |
| **48** | WS2812 RGB DIN |

**Tùy chọn (chưa lắp):** GPIO **1**, **2** = XSHUT ToF khi có 2× VL53L0X.

---

## GPIO rảnh (mở rộng)

| GPIO | Gợi ý |
|------|-------|
| **1, 2** | Rảnh nếu chỉ 1 ToF (XSHUT → 3.3 V) |
| **15** | UART / GPIO *(V1 từng dùng cho loa LRCK)* |
| **21** | I2C / GPIO *(V1 từng dùng cho touch)* |
| **38** | GPIO / decor *(V1 decor LED)* |
| **39–40** | UART (HC-05, GNSS, …) |
| **43–44** | GPIO / SPI dự phòng |
| **47** | Battery CHRG / ADC *(V1 từng ToF XSHUT)* |

```
TRÁNH:  0*, 3, 19–20†, 26–34, 35–37, 45, 46
ĐANG DÙNG: 0, 4–14, 16–18, 41–42, 48
RẢNH:   1, 2‡, 15, 21, 38, 39–40, 43–44, 47

*  GPIO 0 = BOOT
†  Tránh 19–20 khi debug USB
‡  Rảnh khi 1× ToF (XSHUT → 3.3 V)
```

Tắt peripheral không dùng: đặt pin tương ứng `GPIO_NUM_NC` trong `config.h` (`I2C_SENSOR_*`, `TOF_*`, `DISPLAY_RST_PIN`, …).

---

## Sơ đồ nối dây (breadboard hiện tại)

```
                    ESP32-S3 (Blue V2)
                    ┌─────────────────┐
    INMP441 WS ─────┤ 4               │
    MAX98357 LRC ───┤                 │
    INMP441 SCK ────┤ 5               │
    MAX98357 BCLK ──┤                 │
    INMP441 SD ─────┤ 6               │
    MAX98357 DIN ───┤ 7               │
    LCD DC ─────────┤ 8               │
    LCD SCK ────────┤ 9               │
    LCD MOSI ───────┤ 10              │
    MX1508 IN1 ─────┤ 14              │
    MX1508 IN2 ─────┤ 13              │
    MX1508 IN3 ─────┤ 12              │
    MX1508 IN4 ─────┤ 11              │
    TTP223 SIG ─────┤ 16              │
    LCD BLK ────────┤ 17              │
    LCD RES ────────┤ 18              │
    VL53L0X SDA ────┤ 41              │
    VL53L0X SCL ────┤ 42              │
    WS2812 DIN ─────┤ 48              │
    BOOT ───────────┤ 0               │
                    └─────────────────┘

    VL53L0X XSHUT ── 3.3 V
    LCD CS ───────── GND
    INMP441 L/R ──── GND
    MAX98357 SD ──── 3.3 V
```

---

## Nguồn điện (USB OK · pin 4.2 V hay restart)

### Triệu chứng thường gặp

| Nguồn | Mic / loa | Motor | Khi nói (WiFi + TTS) |
|-------|-----------|-------|----------------------|
| **USB** | Tốt | OK | Ổn — nguồn PC ~5 V, dòng lớn |
| **Pin 4.2 V trực tiếp** | Kém | Có thể OK lúc đầu | **Restart** — sụt áp (brownout) |

USB ổn định vì nguồn 5 V mạnh + tụ trên devkit. Pin 4.2 V qua dây mỏng / không đủ tụ → điện áp 3.3 V **tụt** khi:
- WiFi phát (peak **~300–500 mA**)
- MAX98357 loa
- LCD backlight (GPIO 17)
- MX1508 motor (cùng lúc càng tệ)

Firmware bật **brownout reset** — log boot: `Reset: BROWNOUT`.

### Sơ đồ nguồn khuyến nghị

```
Pin 1S LiPo 3.7–4.2 V
        │
        ├──► (tuỳ chọn) Sắc pin / BMS 1S
        │
        ├──► Buck 5 V hoặc 3.3 V (≥2 A) ──► ESP32 + INMP441 + MAX98357 + LCD + ToF
        │         │  + tụ 470–1000 µF gần ESP32
        │         └── GND chung
        │
        └──► MX1508 VMOT (cùng pin hoặc riêng 4.2 V)
                  + tụ 100 µF gần driver
                  GND ──── chung với ESP32 (dây to)
```

### Quy tắc dây

1. **GND chung** ESP32 ↔ motor ↔ pin — dây **dày, ngắn**
2. **Tụ bulk** 470–1000 µF trên rail **3.3 V** (càng gần ESP32 càng tốt)
3. **Tụ 100 µF** trên **VMOT** MX1508
4. **Không** chỉ cấp 4.2 V vào chân 3.3 V MCU — dùng **LDO/buck 3.3 V ≥ 1 A** (AMS1117 yếu khi WiFi; nên buck MP1584 / TLV62569)
5. Motor **nên** cùng pin nhưng **tách đường dây** V+ (star ground), tránh motor chạy ngay khi robot đang nói nếu pin yếu
6. Pin **≥ 1200 mAh**, C-rate đủ; pin c�ng / dây jumper mỏng → sag lớn

### Test nhanh

1. Chạy pin, mở serial — nếi restart khi nói → xem `Reset: BROWNOUT`
2. Thêm tụ 1000 µF vào 3.3 V — nếu hết restart → đúng là nguồn
3. Tạm **rút motor VMOT** — nếu hết restart → motor + pin chung là nguyên nhân chính
4. Giảm backlight (MCP `self.screen.set_brightness`) khi chạy pin

---

## Test mic only (robot nghe — không cần loa)

Chỉ cần **INMP441** + WiFi. Không cần MAX98357, motor, ToF.

### Dây tối thiểu

| INMP441 | ESP32 |
|---------|-------|
| WS | **4** |
| SCK | **5** |
| SD | **6** |
| L/R | **GND** |
| VDD / GND | **3.3 V / GND** |

`config.h`: `AUDIO_MIC_INPUT_GAIN` (mặc định **1.0**), `AUDIO_MIC_SHIFT_BITS` **13**.

ToF vẫn **41/42** nếu chưa cắm sẽ log lỗi I2C (bỏ qua được); tắt ToF: `I2C_SENSOR_* = GPIO_NUM_NC`.

### Các bước test

1. Flash firmware → serial monitor
2. Đợi WiFi + `State: ... -> idle`
3. **Chạm touch (GPIO 16)** hoặc **BOOT** → bắt đầu chat  
   *(Wake word 「你好小智」 là tiếng Trung — không dùng khi test tiếng Việt)*
4. Đợi log: `State: connecting -> listening`
5. Nói rõ: *"xin chào"* (2–3 giây)
6. **Thành công** nếu thấy:
   ```
   Application: >> xin chào
   ```
   → mic + server STT OK (không có loa vẫn OK)

### Nếu không thấy `Application: >>`

| Kiểm tra | |
|----------|--|
| L/R nối GND? | Bắt buộc |
| WS/SCK đúng 4/5? | Không đảo |
| Log có `listening`? | Nếu không → touch lại / xem server |
| Gain | Tăng `AUDIO_MIC_INPUT_GAIN` lên 10–12 |
| Boot log | `INMP441 input gain=8.0 (duplex WS=4 BCLK=5 SD=6)` |

### Log server (192.168.68.71)

Trên máy server, xem log `xiaozhi-server` khi robot đang `listening` — phải có audio/ STT activity.

---

## Pin budget vs Blue V1

| | V1 | V2 (tối thiểu) | V2 + 1 ToF | V2 + 2 ToF |
|--|----|--------------:|----------:|----------:|
| GPIO dùng | ~18 | ~15 | ~17 | ~19 |
| GPIO rảnh (an toàn) | ít hơn | 15, 21, 38, 39–40, 43–44, 47 | +1, 2 | — |
