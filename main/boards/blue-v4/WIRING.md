# Blue V4 — Pinout & Wiring (quadruped)

Firmware board: **`blue-v4`** · MCU: **ESP32-S3-WROOM-1-N16R8** (16 MB flash, 8 MB octal PSRAM)

Same peripheral stack as Blue V2 (INMP441, MAX98357, ST7789 1.54", TTP223, VL53L0X,
status LED) with the MX1508 wheel motors replaced by **8 × MG90S servos** driven by a
**PCA9685** I2C PWM expander (4 legs × hip + knee).

Source of truth: `main/boards/blue-v4/config.h`.

---

## Master pinout

| Function | Module pin | ESP32-S3 GPIO | Notes |
|---|---|---|---|
| **INMP441** (I2S mic) | WS / SCK / SD | **4 / 5 / 6** | L/R → GND (left channel) |
| **MAX98357** (I2S amp) | LRC / BCLK / DIN | **4 / 5 / 7** | Shares WS+BCLK with the mic (duplex) |
| | SD | **3.3 V** | Always enabled |
| | VIN | **5 V** | VIN accepts 2.5–5.5 V; 5 V = more output power |
| **ST7789** 1.54" 240×240 | SDA / SCL / DC / RST / BLK / CS | **10 / 9 / 8 / 18 / 17 / GND** | SDA=MOSI, SCL=SCK, CS tied low |
| **TTP223** touch | SIG | **16** | Active HIGH, 350 ms debounce |
| **VL53L0X** ToF | SDA / SCL / XSHUT | **41 / 42 / 3.3 V** | I2C 0x29, front-mounted |
| **PCA9685** servo driver | SDA / SCL | **41 / 42** | I2C **0x40**, shares the bus with the ToF |
| | VCC | **3.3 V** | Logic supply — never 5 V |
| | V+ | **5 V servo rail** | Servo power, 2200 µF across V+/GND |
| | OE# | **GPIO 1** | Active LOW; **10 kΩ pull-up to 3.3 V** |
| **Status LED** | anode via 220 Ω | **48** | WS2812B DIN or a plain LED |
| **BOOT** | — | **0** | Hold ≥5 s → factory reset |

**I2C pull-ups:** 4.7 kΩ from `IO41` and `IO42` to 3.3 V (one pair for the whole bus).
Most PCA9685 and VL53L0X breakouts include pull-ups; add only what is missing.

**Do not use:** GPIO **26–34** (module flash/PSRAM), **35–37** (octal PSRAM), **3**, **45**, **46**
(strapping), **19–20** (USB-JTAG).
**Still free:** `2`, `15`, `21`, `38`, `39`, `40`, `43`, `44` — plus 8 unused PCA9685 channels.

---

## PCA9685 → servo channel map

| PCA9685 channel | Joint | Joint index (MCP) | Leg |
|---|---|---|---|
| CH0 | front-left hip | 0 | FL |
| CH1 | front-left knee | 1 | FL |
| CH2 | front-right hip | 2 | FR |
| CH3 | front-right knee | 3 | FR |
| CH4 | rear-left hip | 4 | RL |
| CH5 | rear-left knee | 5 | RL |
| CH6 | rear-right hip | 6 | RR |
| CH7 | rear-right knee | 7 | RR |

CH8–CH15 are free (head pan/tilt, gripper, arm…).

### Cách cắm 8 servo — leg nào dùng chân nào, đâu là hip

Quy tắc: **mỗi leg dùng 2 kênh liền nhau, kênh CHẴN = HIP, kênh LẺ = KNEE**
(hip = servo gắn vào thân, xoay cả chân tới/lui; knee = servo nằm giữa femur và tibia, gập chân).

| Cặp kênh PCA9685 | Leg | HIP (chẵn) | KNEE (lẻ) |
|---|---|---|---|
| **CH0 + CH1** | Front-Left (leg 0) | CH0 = **hip** | CH1 = **knee** |
| **CH2 + CH3** | Front-Right (leg 1) | CH2 = **hip** | CH3 = **knee** |
| **CH4 + CH5** | Rear-Left (leg 2) | CH4 = **hip** | CH5 = **knee** |
| **CH6 + CH7** | Rear-Right (leg 3) | CH6 = **hip** | CH7 = **knee** |

- Cắm **4 cặp liền nhau** (0-1, 2-3, 4-5, 6-7) — mỗi cặp là 1 leg, **CH0..CH7** dùng hết, **CH8..CH15 để trống**.
- Firmware luôn dùng `joint = leg*2` (hip) và `joint = leg*2+1` (knee) → nếu đảo 2 dây trong 1 cặp thì hip/knee bị hoán, chân sẽ cử động sai kiểu.
- Nhìn silkscreen module: 16 ô 3 chân xếp 3 nhóm (6 + 4 + 6) tương ứng CH0–CH5, CH6–CH9, CH10–CH15. **Kiểm tra số in trên board** trước khi cắm vì một số loại clone đánh số khác.
- Thứ tự chân trong mỗi ô 3 chân: **PWM (signal) – V+ – GND**; dây servo: cam/vàng = signal, đỏ = V+, nâu/đen = GND.
- Đánh dấu (bút màu/giấy) 8 dây theo leg: `FL-h`, `FL-k`, `FR-h`, `FR-k`, `RL-h`, `RL-k`, `RR-h`, `RR-k` — sẽ tiết kiệm rất nhiều thời gian khi calibrate.
- Header địa chỉ A0–A5: **để hở hết** = địa chỉ `0x40` (đúng như firmware).
- Sau khi cắm: `self.gait.stand` để cả 8 về neutral (90°). Chân nào quay ngược → `self.servo.invert joint=<0..7> inverted=1`; lệch góc chuẩn → `self.servo.trim joint=<0..7> trim=<độ>`.

### Lắp servo HIP và servo KNEE cho mỗi chân

Nguyên tắc chung (thiết kế **nhện** của bản build này):

- Servo **HIP có trục THẲNG ĐỨNG, chĩa XUỐNG** (đít servo hướng lên) ⇒ hip quay quanh trục **yaw**.
- **Body là hình vuông, 4 trục hip nằm ở 4 ĐỈNH.**
- **ARM của servo hip chĩa VÀO trung tâm body** (thanh arm nằm *phía dưới thân*, nên nhìn từ trên xuống gần như không thấy).
- **CẲNG CHÂN (tibia) thì chĩa RA NGOÀI** — ở **neutral (90°) cẳng chân nằm đúng trên ĐƯỜNG CHÉO của hình vuông**, 4 chân tỏa ra 4 góc như con nhện (KHÔNG bao giờ chĩa vào thân). Bàn chân cách trục hip **R = 70 mm**.
- Servo **KNEE có trục NGANG**, **gập cẳng lên/xuống** để nhấc/hạ bàn chân. Tibia (trục knee → mũi chân) = **60 mm**.

Nhìn **từ trên xuống** (x = hướng robot đi; mũi tên = cẳng chân chĩa RA NGOÀI theo đường chéo):

```
        x ^ (robot đi)
          |
      ↖   |   ↗              ↖ ↗ ↙ ↘ = cẳng chân (RA NGOÀI, trên đường chéo body)
   ┌──┴───┴───┴──┐           ● = trục hip yaw ở 4 đỉnh                 
   │             │              (arm của servo chĩa VÀO trong, nằm dưới thân)
   │ BODY VUÔNG  │           R = 70 mm: trục hip → mũi bàn chân ở neutral
   │             │
   └──┬───┬───┬──┘
      ↙   |   ↘
          |
```

Nhìn **từ bên (hoặc trước)** — chỉ servo KNEE (trục ngang, gập cẳng lên/xuống):

```
   [HIP] ●── arm (chĩa vào tâm body)
          │
          │  thanh nối / femur
   [KNEE] ●────── trục NGANG → gập cẳng LÊN (nhấc chân) hoặc XUỐNG
           ╲
            ╲ tibia (60 mm)
             ▼ FOOT  (cách trục hip R = 70 mm khi neutral)
```

Nhìn **từ bên (hoặc trước)** — servo KNEE trục ngang, gập cẳng lên/xuống:

```
   [HIP] ●── arm (chĩa vào tâm body)
          │
          │  femur / thanh nối
   [KNEE] ●────── trục NGANG → gập cẳng LÊN (nhấc chân) hoặc XUỐNG
           ╲
            ╲ tibia
             ▼ FOOT  (R = 70 mm từ trục hip khi neutral)
```

| Leg | Vị trí | Servo HIP | Servo KNEE | Kênh | Chiều |
|---|---|---|---|---|---|
| **FL** trước-trái | đỉnh trước-trái của body | trục **đứng, chĩa xuống**, arm chĩa vào tâm body | trục **ngang**, gập cẳng LÊN khi nhấc chân | **CH0 hip / CH1 knee** | mốc chuẩn |
| **FR** trước-phải | đỉnh trước-phải | như FL (arm cũng vào tâm body) | như FL | **CH2 hip / CH3 knee** | **phải `invert`** (2, và 3 nếu gập sai chiều) |
| **RL** sau-trái | đỉnh sau-trái | như FL | như FL | **CH4 hip / CH5 knee** | mốc chuẩn |
| **RR** sau-phải | đỉnh sau-phải | như FR | như FR | **CH6 hip / CH7 knee** | **phải `invert`** (6, và 7 nếu gập sai chiều) |

Quy tắc lắp:

1. **Hip = yaw**: quay servo hip làm **cả chân quét ngang** theo cung tròn bán kính R; vì neutral chân nằm trên đường chéo nên cung đó có **thành phần ra trước/sau** (nhờ đó robot đi được) và một thành phần **vào/ra** (bề ngang stance thay đổi — bình thường).
2. **Horn lắp ở vị trí 90°**: cấp servo về 90° (`self.servo.set joint=0 angle=90`) rồi mới bắt horn, sao cho ở neutral **cẳng chân nằm đúng trên đường chéo body** và 4 bàn chân cách trục hip đúng **R = 70 mm**. Sau đó chỉnh `self.servo.trim` cho 4 chân đều nhau.
3. **Dây (cable) của servo**: vì cổ dây nằm cùng phía với trục quay trên MG90S, hãy để **dây chạy về phía thân/femur** (không hướng ra ngoài) để khi khớp gập hết cỡ dây không bị căng/kẹt — và cố định dây vào femur/thân bằng keo tụt hoặc dây rút.
4. **Bên phải là bản GƯƠNG của bên trái**: cùng một lệnh servo sẽ làm hip bên phải quét **ngược chiều** với bên trái (và knee bên phải gập ngược lên/xuống). Vì cả 4 trục hip đều chĩa xuống (cùng chiều trong không gian), **hip bên phải phải đảo chiều bằng phần mềm — đừng tháo cơ khí**:
   ```
   self.servo.invert joint=2 inverted=1     # hip chân 2 (FR)
   self.servo.invert joint=6 inverted=1     # hip chân 4 (RR)
   self.servo.invert joint=3 inverted=1     # knee chân 2 — chỉ khi nó gập XUỐNG thay vì LÊN
   self.servo.invert joint=7 inverted=1     # knee chân 4 — chỉ khi nó gập XUỐNG thay vì LÊN
   ```
   Với bản build hiện tại, macro đã đặt sẵn `#define SERVO_INVERT_DEFAULT_MASK 0x44` (hip 2 và 6). Nếu knee phải cũng sai chiều thì đổi thành `0xCC`.
5. Kiểm tra sau khi lắp: `self.gait.leg_sweep leg=0 hip_deg=40 knee_deg=40 duration_ms=2000` — riêng chân 1 quét ngang 40° và knee nhấc lên, 3 chân kia đứng yên. So 4 hip với nhau (`self.servo.raw_pulse joint=0|2|4|6 pulse_us=1200`) — **cả 4 phải quét cùng một hướng trong không gian**; so 4 knee (`joint=1|3|5|7 pulse_us=1800`) — **cả 4 phải nhấc bàn chân LÊN**.

> **Lưu ý hình học (bản build nhện, hip = yaw):** bàn chân chạy trên cung tròn bán kính **R = 70 mm** quanh trục hip, và vì neutral nằm trên đường chéo nên **thành phần đi tới/lùi mỗi bước ≈ √2 · R · sin(Δ/2)**:
>
> | Δ hip | Bàn chân đi tới/lùi mỗi bước |
> |---|---|
> | 20° | 17 mm |
> | 30° | 26 mm |
> | 40° | 34 mm (mặc định đi trên sàn) |
> | 60° | 50 mm |
> | 120° | 86 mm (chỉ test trên kệ) |
>
> **Độ nhấc chân theo góc gập knee** (tibia **60 mm**, α ≈ góc cẳng chân dưới phương ngang ở neutral): nhấc ≈ `60 · (sin(α+Δ) − sin(α))` ⇒ khoảng **20° ≈ 7 mm, 30° ≈ 15 mm, 40° ≈ 17 mm**. Mặc định firmware: `GAIT_JOINT_KNEE_TRAVEL_DEG 30` (~15 mm, đủ nhấc chân mà không cần gập sâu).
>
> Chiều cao body trong gait joint-space do **độ gập knee** quyết định (`self.gait.stand/sit/body` dùng `GAIT_JOINT_CROUCH_DEG_PER_MM ≈ 1.3 °/mm`), còn hip luôn giữ trên đường chéo. Đường IK (`RunWalk`, `LEG_FEMUR_MM`/`LEG_TIBIA_MM`, `BODY_STAND_HEIGHT_MM`) giả định kiểu Otto (hip/knee cùng gập trong mặt phẳng dọc) nên **không dùng cho robot này**. Muốn test biên độ lớn trên kệ: `self.gait.walk steps=1 step_ms=1400 hip_deg=120 knee_deg=60`.

---

## VL53L0X front ToF (shared I2C bus)

| VL53L0X | ESP32-S3 |
|---|---|
| VIN | **3.3 V** |
| GND | **GND** |
| SDA | **IO41** (shared with PCA9685) |
| SCL | **IO42** |
| XSHUT | **3.3 V** (single sensor, always enabled) |
| GPIO1 | not connected |
| I2C address | **0x29** |

Calibration and guard behaviour mirror blue-v2:

1. Put the robot on **open floor** and call `self.tof.calibrate` (auto, `distance_mm=0`) —
   the median reading is saved to NVS namespace `blue_v4_tof`.
2. While walking, the gait **stops** when:
   - obstacle: dist < cal×75% **or** < cal−30 mm, or a fast close (≥20 mm in one poll),
   - drop-off: dist > cal×125% **or** > cal+50 mm, or the signal stays lost for 3 polls.
3. Without calibration it uses fixed fallback limits: stop ≤ **190 mm**, void ≥ **200 mm**.

**Note:** the ToF calibration namespace is new (`blue_v4_tof`), so calibrate once after
flashing blue-v4 — it does not reuse blue-v2's `blue_tof` values.

Wiring:

```
PCA9685                 ESP32-S3 / power
  VCC  ───────────────── 3.3 V (logic)
  GND  ───────────────── GND
  SDA  ───────────────── IO41   (shared with VL53L0X)
  SCL  ───────────────── IO42
  OE#  ───────────────── IO1  + 10k pull-up to 3.3 V
  V+   ───────────────── 5 V servo rail (+ 2200 uF to GND)
  CH0..CH7 signal ────── servo signal wires (orange/yellow)
  servo +  ──────────── V+   (all eight)
  servo −  ──────────── GND  (all eight, thick wire back to the module)
```

**Servo pulse range:** 500–2500 µs at 50 Hz (12-bit). Out-of-the-box MG90S travel is
≈ 500–2400 µs; the trim/limit values live in NVS (namespace `blue_v4_servo`) and are
tuned with `self.servo.trim` / `self.servo.invert`.

---

## Power

```
1S 18650 (high-drain, 2500–3000 mAh)
   │
   ├── Type-C charge/discharge module (B+/B− = cell, VCC 5 V / 2 A)
   │        │
   │        ├── ESP32 devkit 5V ──► 3V3 rail (MCU, mic, LCD, ToF, PCA9685 logic, LED)
   │        ├── MAX98357 VIN (5 V)
   │        └── PCA9685 V+ (servo rail) ── 2200 uF ── GND
   │
   └── (optional upgrade) cell + ──► servo rail directly, or through a 5 V/6 A UBEC
```

**Rules**

1. **One common GND** for module, devkit, PCA9685, servos and amp — star-grounded at the module.
2. The module supplies **2 A total**. The crawl gait moves **one leg (2 joints) at a time**,
   and the firmware caps `SERVO_MAX_ACTIVE_MOVING`, slews every move and **relaxes the servos
   after 8 s idle**, which keeps peaks well inside that budget.
3. **Do not** exceed 6 V on PCA9685 `V+`, and never feed 5 V into `VCC` or any GPIO.
4. If you see `Reset: BROWNOUT` in the log, feed the servo rail from the cell instead of the
   module's 5 V output (see the upgrade path above) or lower `step_ms` / stride.
5. Keep the servo `V+`/GND wires short and thick; the signal wires are thin and can be long.

---

## Safety behaviour (firmware)

| Mechanism | Behaviour |
|---|---|
| `OE#` early HIGH | Servos stay limp through boot, reset and brownout — no twitch |
| OE# released after init | Only after the PCA9685 is configured and the first pose is applied |
| `self.gait.stop` / `self.servo.stop` | Immediate PWM off + OE# high (limp) |
| Touch tap while walking | Stops the gait instead of toggling chat |
| Sleep / factory reset | Servos relaxed before dimming / rebooting |
| Idle relax | Servos de-energised after 8 s without a new target while not holding a pose |

---

## Bring-up checklist

1. Power the devkit over USB **with the servo rail unpowered** and confirm the face UI +
   serial log (`I2C bus on SDA 41 / SCL 42`, `PCA9685 0x40 ready`).
2. Power the servo rail. On boot the servos must **not** move; they energise only after
   `self.gait.stand`.
3. Calibrate trims: `self.servo.trim` per joint until each leg's neutral (90°) is square.
   Use `self.servo.invert` for mirrored legs that move the wrong way.
4. Verify direction: `self.gait.walk` with 1 step and a small stride (10 mm) on blocks so
   the feet are off the ground, then flip the hip/knee signs if a leg swings backwards.
5. Set the real leg geometry (`LEG_FEMUR_MM`, `LEG_TIBIA_MM`, `LEG_HIP_OFFSET_*`,
   `BODY_STAND_HEIGHT_MM`) after measuring the printed frame.
6. Current check: measure the 5 V rail during a crawl step; keep peaks under ~1.5 A.
