#ifndef _BLUE_V4_CONFIG_H_
#define _BLUE_V4_CONFIG_H_

#include <driver/gpio.h>

// Blue V4 — ESP32-S3-WROOM-1-N16R8 quadruped robot.
//
// Same peripheral stack as Blue V2 (ST7789 1.54" 240x240, INMP441 I2S mic,
// MAX98357 I2S amp, TTP223 touch, VL53L0X ToF, status LED) but the MX1508 DC
// motors are replaced by 8x MG90S servos driven by a PCA9685 I2C PWM expander:
// 4 legs x 2 degrees of freedom (hip + knee).
//
// This board directory is SELF-CONTAINED: all board support files are clones
// living in main/boards/blue-v4/ (see README.md), independent of blue-v2.
// Wiring reference: WIRING.md.

#define AUDIO_INPUT_SAMPLE_RATE 16000
// Duplex I2S uses one BCLK/WS clock pair (see no_audio_codec.cc) — matches mic + Opus 16 kHz.
#define AUDIO_OUTPUT_SAMPLE_RATE 16000

// Shared I2S bus: INMP441 + MAX98357 (duplex)
#define AUDIO_I2S_GPIO_WS GPIO_NUM_4
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_5
#define AUDIO_I2S_GPIO_DIN GPIO_NUM_6   // mic SD
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_7  // speaker DIN

// INMP441 (L/R->GND): AGC + soft compressor in no_audio_codec (see AUDIO_MIC_AGC_* in CMakeLists).
// AudioCodec::input_gain_ defaults to 0.0, so GetAudioCodec() must call SetInputGain() with
// this value or the uplink stays silent (blue-v2 does exactly the same).
#define AUDIO_MIC_SHIFT_BITS 12
#define AUDIO_MIC_INPUT_GAIN 1.0f
#define AUDIO_MIC_SOFT_LIMIT 24000

// Mic level debug on serial (~1 Hz) while I2S input is active (bring-up).
#define AUDIO_MIC_DEBUG_LOG 1

#define BUILTIN_LED_GPIO GPIO_NUM_48
#define BOOT_BUTTON_GPIO GPIO_NUM_0
#define TOUCH_BUTTON_GPIO GPIO_NUM_16
// TTP223 SIG goes HIGH when touched (most modules). Set 0 if yours is active-low.
#define TOUCH_BUTTON_ACTIVE_HIGH 1
#define TOUCH_DEBOUNCE_MS 350

#define RESET_NVS_BUTTON_GPIO GPIO_NUM_NC
#define RESET_FACTORY_BUTTON_GPIO GPIO_NUM_NC
#define FACTORY_RESET_LONG_PRESS_MS 5000

// -1 = no auto sleep while the quadruped is on the bench; set 60 for battery use.
#define POWER_SAVE_SLEEP_SECS (-1)
// Minimum backlight when entering sleep via MCP (not 1% — nearly invisible on 1.54" LCD).
#define POWER_SAVE_DIM_BRIGHTNESS 25

// LVGL draw buffer height (lines). Internal DMA SRAM — avoid full-frame PSRAM flush
// (SPI DMA OOM + WDT). See main/display/lcd_display.cc.
#define DISPLAY_LVGL_BUFFER_LINES 80

#define DISPLAY_MOSI_PIN GPIO_NUM_10
#define DISPLAY_CLK_PIN GPIO_NUM_9
#define DISPLAY_DC_PIN GPIO_NUM_8
// Breadboard: module RES → GPIO 18. PCB with RES tied to 3.3 V: use GPIO_NUM_NC.
#define DISPLAY_RST_PIN GPIO_NUM_18
#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_17
#define DISPLAY_CS_PIN GPIO_NUM_NC

// No separate decor LED on Blue V4.
#define DECOR_LED_GPIO GPIO_NUM_NC

// ---------------------------------------------------------------------------
// I2C bus — shared by VL53L0X (0x29) and PCA9685 (0x40).
// The board creates this bus and injects the handle into both drivers.
// ---------------------------------------------------------------------------
#define I2C_SENSOR_SDA_PIN GPIO_NUM_41
#define I2C_SENSOR_SCL_PIN GPIO_NUM_42
#define I2C_SENSOR_SPEED_HZ 400000
#define I2C_SENSOR_PORT I2C_NUM_0

// ---------------------------------------------------------------------------
// PCA9685 servo driver — 4 legs x 2 joints (hip + knee) = 8 of 16 channels.
// ---------------------------------------------------------------------------
#define PCA9685_I2C_ADDR 0x40
// OE# (active LOW = outputs enabled). Pulled up in hardware so the servos stay
// limp through any reset/brownout and only energise after a clean init.
#define PCA9685_OE_GPIO GPIO_NUM_1

// Logical joint -> PCA9685 channel. Joint order: 0/1 = leg 1 FL hip/knee, 2/3 = leg 2 FR,
// 4/5 = leg 3 RL, 6/7 = leg 4 RR. Only change an entry when a driver channel is damaged:
// re-plug that servo into a free channel (8..15) and point the joint there.
#define SERVO_CHANNEL_MAP {0, 1, 2, 3, 4, 5, 6, 7}

// Default direction inversion per joint (bit i = joint i): the FACTORY DEFAULT only — an
// explicit value in NVS (servo.invert / tag `srv:invert=J:0|1`) overrides it absolutely.
//
// QUY LUẬT LẮP RÁP (xác nhận trên bàn 22-09-2026): 4 cặp servo được lắp NGƯỢC nhau —
//   (j0, j2) hip trước · (j1, j3) knee trước · (j4, j6) hip sau · (j5, j7) knee sau.
// Vì 2 con trong mỗi cặp lắp ngược chiều, firmware phải ĐẢO ĐÚNG MỘT CON trong mỗi cặp thì
// cả 2 chân mới quay cùng chiều trong hệ thân. Mask dưới đây đảo: j2, j1, j6, j7 ✓ (4/4 cặp).
//
// Chọn con nào trong cặp để đảo là tuỳ hướng lắp thực tế — sai 1 con ⇒ chân đó chạy ngược:
//   • ngồi xuống: 2 chân gập đúng, 2 chân kia co/duỗi NGƯỢC lại
//   • đi: robot bị kéo lệch/vặn thay vì đi thẳng
// Cách sửa nhanh, KHÔNG cần nạp lại: `srv:leg=N` (test riêng từng chân) rồi
// `srv:invert=N:0|1` (NVS thắng tuyệt đối), sau đó bake lại mask ở đây.
#define SERVO_INVERT_DEFAULT_MASK 0xC6

#define SERVO_COUNT 8
#define SERVO_PWM_FREQ_HZ 50
#define SERVO_PWM_RESOLUTION 4096
// Dải xung map thẳng vào 0..180 deg: pulse = min + (angle/180) * (max - min) ⇒ 0..180° = CẢ
// dải xung. Vì vậy DẢI XUNG chính là GAIN (µs/độ) của toàn bộ mô hình góc.
// MG90S (datasheet): 500 us (0°) / 1500 us (90°) / 2500 us (180°) ⇒ 11.1 µs/độ. Đây là dải THẬT
// của servo nên 0..180° lệnh = 0..180° VẬT LÝ, và mọi hằng số *_DEG trong file này (cùng mô
// hình hình học mm: bảng hạ thân, alpha=43.3°, "ngồi ~38°"…) mới đúng nghĩa.
// ⚠️ 1000..2000 (~5.6 µs/độ) KHÔNG phải servo yếu — nó chỉ bóp mọi động tác còn NỬA hành trình
// vật lý (servo không dùng hết dải). Các hằng số bên dưới đã được chỉnh lại theo dải 500..2500
// (xem ghi chú ở từng hằng số) để giữ đúng hành vi đã hiệu chỉnh trên bàn.
// Đổi lúc chạy: `srv:range=min-max` (lưu NVS pmin/pmax — NVS THẮNG giá trị ở đây).
// Kiểm chứng servo có chạy hết dải không: `srv:travel=N` (0→180→0, 2 vòng, xong về neutral).
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO_UPDATE_PERIOD_MS 10  // 100 Hz interpolation tick (dày hơn = đường đi mượt hơn)
// 1 = log the servo task rate/state once per second (bring-up diagnostics).
#define SERVO_TICK_DEBUG_LOG 1
#define SERVO_CMD_QUEUE_DEPTH 8

// --- Giãn nhịp PWM (kiểu "motorCurrentDelay" của Sesame) ---
// 8 servo cùng khởi động trong một nhịp là lúc rail 5 V sụt mạnh nhất (nhìn log mic sẽ thấy
// agc tụt). Khi một lệnh làm NHIỀU servo nhảy một bước lớn cùng lúc (đứng lên, đổi tư thế,
// enable) thì các servo được "thả" lệch nhau SERVO_STAGGER_MS thay vì cùng một nhịp.
// Dòng nội suy của gait (delta nhỏ, 100 Hz) KHÔNG bị giãn — nó không phải bước nhảy.
#define SERVO_STAGGER_START_ENABLE 1
#define SERVO_STAGGER_MIN_STEP_DEG 12.0f  // >= mức này mới coi là "bước nhảy"
#define SERVO_STAGGER_MS 40               // khoảng cách giữa 2 servo liên tiếp (Sesame: 20 ms)
// Trần số kênh ghi PWM trong MỘT tick, xoay vòng điểm bắt đầu ⇒ 8 kênh không dồn vào cùng
// một thời điểm (vừa đủ cho gait: 1 cú vung = 2 kênh, pha push = 4 hip).
#define SERVO_PWM_MAX_WRITES_PER_TICK 4

// Servo calibration / motion defaults (per-servo trim lives in NVS, see servo_controller).
#define SERVO_DEFAULT_MIN_DEG 0.0f
#define SERVO_DEFAULT_MAX_DEG 180.0f
#define SERVO_DEFAULT_NEUTRAL_DEG 90.0f
// Motion profile: slow and soft. Speed is ramped with an acceleration limit, so moves
// start/stop gently and arrive without overshoot (smooth, no jitter, clean stop).
#define SERVO_SLEW_DEG_PER_SEC 90.0f
#define SERVO_ACCEL_DEG_PER_SEC2 240.0f
#define SERVO_STOW_SLEW_DEG_PER_SEC 90.0f

// Boot pose: the joints stay limp after power-up, then (after the delay) every joint travels
// to the neutral point and holds there, so the legs settle instead of snapping at reset.
// Any pose published before the deadline cancels this.
#define SERVO_BOOT_NEUTRAL_ENABLE 1
#define SERVO_BOOT_NEUTRAL_DELAY_MS 1500
#define SERVO_BOOT_NEUTRAL_SLEW_DEG_PER_SEC 45.0f
// Keep torque only this long after the boot pose, then let the idle timeout release the servos
// (8 stalled joints draw a lot of current and can sag the servo rail).
#define SERVO_BOOT_NEUTRAL_HOLD_MS 3000

// Relax (PWM off) after this long with no new target WHILE NOT HOLDING A POSE.
// Holding a stand pose keeps torque; only idle/relaxed states time out.
#define SERVO_IDLE_RELAX_MS 8000
// 1 = drop mic frames while any servo is slewing (same idea as blue-v2's motor uplink
// pause: keeps servo whine/EMI out of the uplink). 0 = always keep the mic live.
#define BLUE_V4_PAUSE_UPLINK_WHILE_MOVING 1

// Motion command queue depth — the server sends mv:* sequences, so a few commands may
// arrive while one is still running.
#define BLUE_V4_GAIT_QUEUE_DEPTH 8

// 1 = also expose blue-v2's self.motor.* tool names so the existing server drives the
// servos with no server-side change (mv:* tags keep working). 0 = only self.gait.*/self.servo.*.
#define BLUE_V4_MOTOR_COMPAT_TOOLS 1
// Current budget guard: the Type-C 1S module supplies 5 V/2 A for logic + servos.
// The crawl gait moves one leg at a time, so at most 2 joints are loaded at once.
#define SERVO_MAX_ACTIVE_MOVING 2

// --------------------------------------------------------------------------
// Legacy blue-v2 wheel driver (MX1508) on IO11-IO14.
// blue-v4 never drives these pins, but if the old wiring is still connected the
// floating H-bridge inputs can chatter (wheels twitch, current spikes, EMI on the
// shared I2C/servo rail). 1 = drive them LOW at boot.
// Set 0 only if you repurpose those GPIOs for something else on blue-v4.
// --------------------------------------------------------------------------
#define BLUE_V4_LEGACY_MX1508_PINS_LOW 1
#define LEGACY_MX1508_IN1 GPIO_NUM_14
#define LEGACY_MX1508_IN2 GPIO_NUM_13
#define LEGACY_MX1508_IN3 GPIO_NUM_12
#define LEGACY_MX1508_IN4 GPIO_NUM_11

// ---------------------------------------------------------------------------
// VL53L0X front ToF (same behaviour/tuning as blue-v2).
// Shares the board-owned I2C bus with the PCA9685 (0x29 vs 0x40).
// ---------------------------------------------------------------------------
#define TOF_FRONT_XSHUT_GPIO GPIO_NUM_NC  // single sensor: tie XSHUT -> 3.3 V
#define TOF_FRONT_I2C_ADDR 0x29

// Safety guard while the gait runs (requires self.tof.calibrate on open floor first).
// Set BOTH to 0 to disable the stop logic for bench testing — the ToF sensor keeps
// working and self.tof.get_distance still reports.
#define TOF_OBSTACLE_GUARD_ENABLE 0
#define TOF_CLIFF_GUARD_ENABLE 0
#define TOF_GUARD_POLL_MS 50
#define TOF_MAX_VALID_MM 2000

// When calibrated: stop if dist deviates from the saved cal distance (% + absolute, stricter wins).
#define TOF_CAL_NEAR_MARGIN_PCT 25   // obstacle: dist < cal * 75%
#define TOF_CAL_FAR_MARGIN_PCT 25    // cliff: dist > cal * 125%
#define TOF_CAL_NEAR_MARGIN_MM 30    // also dist < cal - 30 mm (whichever stops sooner)
#define TOF_CAL_FAR_MARGIN_MM 50     // also dist > cal + 50 mm
#define TOF_CAL_APPROACH_STEP_MM 20  // closing fast: dist dropped this much in one poll
#define TOF_CAL_JUMP_MARGIN_MM 80    // sudden extra deviation from cal in one step

// Fallback cliff: jump from nearest reading this move leg (mm).
#define TOF_CLIFF_JUMP_MM 80
#define TOF_CLIFF_NEAR_MAX_MM 160  // jump/lost-signal cliff only if was this close first

// Ignore the obstacle guard for this long after the gait starts (I2C/vibration settle).
#define TOF_MOVE_GRACE_MS 150

// Fallback fixed thresholds when NOT calibrated yet (sensor ~128 mm to open floor).
#define TOF_OBSTACLE_STOP_MM 190
#define TOF_CLIFF_VOID_MM 200
#define TOF_CALIBRATION_DISTANCE_MM 128

// Verbose ToF / guard logs on serial (set 0 to reduce spam).
#define TOF_DEBUG_LOG 1

// ---------------------------------------------------------------------------
// Leg geometry (mm) — used by the gait engine inverse kinematics.
// ĐO THẬT bằng thước kẹp 22-09-2026 (khớp cả 3 số đo độc lập — xem LEG_FOOT_RADIUS_MM):
#define LEG_FEMUR_MM 30.0f         // trục yaw -> trục knee (khoảng ngang, "cánh tay" của hip)
#define LEG_TIBIA_MM 55.0f         // trục knee -> mặt bàn chân (càng)
#define LEG_HIP_OFFSET_X_MM 45.0f  // body centre -> hip axis, fore/aft
#define LEG_HIP_OFFSET_Y_MM 55.0f  // body centre -> hip axis, left/right
// ⚠️ Chiều cao thân Ở ĐÂY LÀ ĐƠN VỊ ẢO, không phải mm thật: nó chỉ dùng để quy đổi
// "hạ thân X mm" -> độ gập knee trong chế độ joint-space (GAIT_JOINT_CROUCH_DEG_PER_MM).
// Chiều cao THẬT là hơn 37.7 mm (xem LEG_TIBIA_LEN_MM) — đừng lấy 95/66/120 so với thước kẹp.
#define BODY_STAND_HEIGHT_MM 95.0f  // default hip->foot vertical distance
// Ngồi (pst:sit) = hạ thân xuống mức này; self.gait.body cũng hạ được tới đây.
// 66 mm ⇒ knee gập (95-66)*1.30 ≈ 38° (+16% so với bản 70 mm/32.5°).
// ĐÃ THỬ 62 mm (42.9°): sát giới hạn cơ khí, 4 knee gập cùng lúc làm 2 servo sau stall.
#define BODY_MIN_HEIGHT_MM 66.0f
#define BODY_MAX_HEIGHT_MM 120.0f
#define STRIDE_LENGTH_MM 40.0f  // default crawl stride

// ---------------------------------------------------------------------------
// Joint-space crawl gait (hip/knee driven directly, no IK reach clamping).
// Gives a large, predictable travel per step for bring-up and for a simpler walk.
// 0 = use the inverse-kinematics crawl instead.
// ---------------------------------------------------------------------------
#define BLUE_V4_JOINT_SPACE_GAIT 1
#define GAIT_JOINT_HIP_NEUTRAL_DEG 90.0f  // centre of the hip sweep (mount neutral)
// Hip servo = VERTICAL yaw axis (shaft pointing down, arm pointing to the body centre, leg
// pointing outward on the body diagonal): the hip swings the foot sideways/fore-aft, so the
// FORWARD travel per step is sqrt(2) * R * sin(travel/2). Với R = 70 mm: 22.5° ≈ 19 mm,
// 45° ≈ 34 mm, 60° ≈ 43 mm, 90° ≈ 70 mm mỗi bước.
// Lấy 45° vì đó ĐÚNG bằng hành trình đã test trên bàn (trước đây 90° trên thang nửa dải chỉ là
// 45° vật lý). Muốn stride dài hơn: tăng dần 45 → 60 (≈43 mm/bước) rồi kiểm tra tải + độ ổn
// định. Override từng lệnh bằng hip_deg=.
#define GAIT_JOINT_HIP_TRAVEL_DEG 45.0f  // walking default (override per call with hip_deg=)
// Biên độ nhấc của knee: knee đi từ 90° (chân chạm nền) lên 90+36 = 126° vật lý.
// 36° = đúng biên độ nhấc đã test trên bàn (trước đây 72° trên thang nửa dải cũng chỉ là 36°
// vật lý). Nhấc cao hơn (60-72°, tức 150-162°) thì chân vung đưa bàn chân lên quá cao và dễ
// đụng thân sau khi gắn robot — chỉ dùng khi test trên bàn. Override bằng knee_deg=.
#define GAIT_JOINT_KNEE_TRAVEL_DEG 36.0f  // knee fold, lifts the foot (override with knee_deg=)
#define GAIT_JOINT_STEP_MS \
    1400  // default step_ms khi lệnh không nêu (xem GAIT_JOINT_SEQUENTIAL_CRAWL)

// --- Phối hợp 4 chân ---
// 0 = CONTINUOUS (mặc định) — duty factor 3/4: cả 4 chân chạy trên MỘT đồng hồ, lệch pha 25%.
//     1 chân VUNG (25% chu kỳ) + 3 chân TRỤ quét liên tục (75%). Vì 3 chân trụ quét về sau cùng
//     một tốc độ nên bàn chân của chúng đứng yên so với nền ⇒ thân dịch đều, KHÔNG trượt lết
//     (khác bản cũ: chỉ 1 hip đẩy nên 3 chân trụ bị kéo lết). Cả 4 chân luôn chuyển động.
//     1 bước = 1 chu kỳ = 4 x swing_ms (350 ms ⇒ 1400 ms/bước, nhanh ~2.8x bản cũ).
// 1 = SEQUENTIAL — từng chân một (vung → chờ chân chạm nền → push), bản cũ (~3950 ms/bước).
// Đổi được lúc chạy: tool self.gait.crawl hoặc tag srv:crawl=continuous|sequential.
#define GAIT_JOINT_SEQUENTIAL_CRAWL 0
// Thời gian "vào nhịp": đưa 4 chân từ tư thế đứng vào đúng pha xuất phát của gait (chân sắp
// vung ở hip_back, 3 chân trụ trải đều trên hành trình). Các độ lệch cộng lại bằng 0 nên thân
// KHÔNG trôi về sau — khác pha "park" cũ (kéo cả 4 hip về sau rồi mới đi).
#define GAIT_JOINT_CRAWL_ENTRY_MS 400

// Chiều đi tới: +1 = hip quét theo chiều servo tăng, -1 = đảo lại. Đổi dấu ở đây nếu robot
// đi lùi khi được lệnh đi tới (1 bước vẫn là 4 chân/8 servo, chỉ đảo chiều quét hip).
#define GAIT_JOINT_FORWARD_SIGN (-1.0f)

// --- Phase timing (per leg, in the crawl order) ---
// One cú vung chân = MỘT đường cong duy nhất (kiểu "mix" trong RC): cùng một tham số chạy
// 0→1, kênh hip quét đơn điệu (smoothstep) còn kênh knee nhấc rồi hạ chân theo đường bao
// tam giác. Hai servo dùng chung một tham số nên không có điểm dừng giữa chân — đây là lý do
// RC mượt. Bàn giao xong, phải chờ chân THẬT SỰ chạm nền rồi mới cho hip quay về (plant settle).
//   0.0 = tam giác thuần (knee đi chậm nhất → servo bám được)
//   0.2 = giữ chân ở đỉnh 20% hành trình (chân cao lâu hơn, nhưng knee phải đi nhanh hơn ~25%)
// Cách thực hiện CÚ VUNG của một chân:
//   0 = ARC    : một đường cong "mix" hip+knee trên cùng một tham số (mượt, nhanh) — mặc định.
//   1 = DIRECT : kiểu Sesame — giao ĐÍCH từng bước (knee nhấc lên → hip quét → knee hạ xuống)
//                rồi chờ servo TỰ ĐI tới, không nội suy đường cong nào.
// Đổi được lúc chạy bằng tool self.gait.swing hoặc tag srv:swing=arc|direct, không cần nạp lại.
#define GAIT_JOINT_SWING_DIRECT 0

// Bước đầu tiên khi bắt đầu đi:
//   1 = vung chân THẲNG về phía trước từ tư thế đứng (neutral) — tự nhiên, robot không lùi lại
//       trước khi đi tới. Đổi lại bước đầu chỉ được nửa hành trình (hip: neutral → hip_forward).
//   0 = "park" cả 4 hip về hip_back trước rồi mới vung — bước đầu đủ hành trình nhưng robot
//       trông như lùi lại một đoạn trước khi đi tới.
#define GAIT_JOINT_STEP_FROM_NEUTRAL 1

#define GAIT_JOINT_LIFT_HOLD_FRAC 0.0f
// Cho phép servo bám đường cong trễ hơn bao nhiêu độ. Slew/accel của limiter được đặt thành
// 1.3x tốc độ đỉnh và rate^2/(2*lag) để sai số bám không vượt giá trị này — nhờ vậy chân không
// "trễ dần" qua từng bước (trước đây knee đọng lại giữa đường nhấc rồi mỗi bước cao thêm một
// ít cho tới khi bàn chân không bao giờ chạm nền nữa).
#define GAIT_JOINT_TRACK_LAG_DEG 4.0f
#define GAIT_JOINT_PLANT_DWELL_MS 150  // sàn tối thiểu của thời gian chờ chạm nền
// The limiter reports "arrived" when the COMMAND reaches the target, but a loaded servo (it is
// lifting the whole body here) lands later. Wait for the real travel at this rate before letting
// any hip rotate back — otherwise the yaw returns while the foot is still in the air.
#define GAIT_JOINT_SETTLE_DEG_PER_SEC 250.0f
#define GAIT_JOINT_SETTLE_TIMEOUT_MS 700  // hard cap on a settle wait (safety)
// Gập knee -> hạ thân / nhấc chân. Càng (knee -> bàn chân) = 55 mm và ở tư thế đứng nó nghiêng
// alpha = 43.3° so với mặt nền (suy ra từ R = 70 mm: cos(alpha) = (70-30)/55), nên:
//   hạ thân = 55 * (sin(alpha + delta) - sin(alpha)); chân nhấc lên cũng đúng công thức này.
//   gập 10° -> 6.4 mm · 19° (nghiêng mặc định) -> 11.0 mm · 38° (ngồi) -> 16.6 mm
//   CỰC ĐẠI ở gập 47° (càng thẳng đứng) -> 17.3 mm ⇒ gập thêm nữa KHÔNG hạ thêm được.
#define LEG_TIBIA_LEN_MM 55.0f
// Joint-space posture (spider geometry): extra knee fold per mm of body-height reduction.
// Hình học thật: ở tư thế đứng 1 mm = 1/(55*cos43.3°*pi/180) ≈ 1.43° gập, giảm dần khi càng
// tiến về thẳng đứng nên lấy trung bình ~0.65°/mm cho cả dải dùng được.
// Vì sao 0.65 chứ không 1.43: đây là góc VẬT LÝ. `BODY_MIN_HEIGHT_MM 66` ⇒ hạ 29 mm ảo ⇒
// gập ~18.9° ⇒ hạ thân THẬT ~10.9 mm (đúng bảng hạ thân ở LEG_TIBIA_LEN_MM) — y như độ sâu
// đã được duyệt "mượt, nhanh, đẹp" trên bàn. Muốn ngồi sâu hơn: tăng hệ số này (trần an toàn
// cho 4 knee là ~0.9°/mm ⇒ 26° gập) rồi kiểm tra lại dòng/tiếng kêu.
#define GAIT_JOINT_CROUCH_DEG_PER_MM 0.65f
// --- Tốc độ đổi tư thế (ngồi / đứng / nghiêng) ---
// Thời gian một LƯỢT = hành trình / tốc độ, kẹp trong [min, max]. Tốc độ là tốc độ GÓC VẬT LÝ
// của servo (deg/s) và KHÔNG chia theo số servo cùng chạy ⇒ mọi tư thế có cùng "cảm giác
// nhanh chậm": hành trình dài đi lâu hơn, hành trình ngắn xong sớm hơn.
// 70°/s = ĐÚNG tốc độ vật lý bạn đã khen ở "ngồi xuống" (trước đây 140°/s trên thang nửa dải
// cũng chỉ là 70°/s vật lý). Giữ 70 để vừa đúng cảm giác cũ, vừa giữ dòng đỉnh trong ngân sách
// rail 5 V/2 A.
#define POSTURE_RATE_DEG_PER_SEC 70.0f
// SÀN thời gian một LƯỢT: để một nhích rất nhỏ không đi quá nhanh. Nghiêng nhẹ vẫn xong trong
// 150 ms (nghiêng 12° = 2 lượt x 274 ms).
#define POSTURE_MIN_MS 150
#define POSTURE_MAX_MS 900
// Hành trình knee ≥ mức này thì mới phải chia LƯỢT. Dưới mức này làm một lượt (dòng không đáng
// kể) nên các điều chỉnh nhỏ vẫn xong trong đúng 300 ms.
// Lý do phải chia: ngồi sâu = 4 knee gập cùng lúc, mỗi con chịu ~1/4 khối lượng thân ⇒ dòng đỉnh
// lớn. Đo trên bàn: test riêng từng chân thì gập đủ 43° được, nhưng khi ngồi (4 chân cùng lúc)
// thì 2 chân sau stall.
#define POSTURE_SEQUENTIAL_DEG 12.0f
// Số servo được phép chạy CÙNG LÚC trong một lượt khi đã phải chia lượt:
//   4 knee CÙNG hướng (ngồi / đứng) ⇒ 1 — chạy từng chân một (đang rất mượt, giữ nguyên).
//   2 knee gập + 2 knee duỗi (nghiêng / chúi) ⇒ 2 — bắt cặp, chỉ 2 lượt x ~150 ms.
// Chia lượt chỉ để giới hạn DÒNG đỉnh, không để làm chậm: mỗi lượt vẫn chạy ở
// POSTURE_RATE_DEG_PER_SEC. Nếu sau này gắn thân robot vào mà thấy brownout khi nghiêng thì
// hạ số này xuống 1 (4 lượt, vẫn nhanh) hoặc hạ POSTURE_RATE_DEG_PER_SEC.
#define POSTURE_SERVOS_PER_PHASE 2
// Chênh lệch nhỏ hơn mức này (độ) coi như khớp đã ở đúng chỗ, khỏi đưa vào lượt.
#define POSTURE_MOVE_EPS_DEG 0.5f
// Độ gập thêm cho MỖI độ pitch/roll. 1.6 ⇒ `pst:lt` (mặc định 12°) = ±19.2° VẬT LÝ chênh giữa
// 2 bên (với dải 500..2500; trước đây trên thang nửa dải chỉ còn ±9.6° nên nghiêng nhìn rất ít).
// Bên "thấp" gập xuống như ngồi, bên kia duỗi ra đứng thẳng. Muốn nghiêng nhẹ lại: 1.6 → 0.8.
#define GAIT_JOINT_TILT_DEG_PER_DEG 1.6f
// Kẹp riêng 2 phía: cơ cấu gập được sâu hơn nhiều so với khi duỗi ngược ra ngoài.
#define GAIT_JOINT_TILT_MAX_FOLD_DEG \
    35.0f                                     // trần phía gập (ngồi hiện tại ~19°; trần cơ khí
                                              // đo trên bàn khi test riêng 1 chân là ~43°)
#define GAIT_JOINT_TILT_MAX_EXTEND_DEG 20.0f  // trần phía duỗi (knee mở ra ngoài ~20°)
// Khoảng cách NGANG từ trục yaw tới bàn chân ở tư thế đứng. Không phải số đo trực tiếp mà suy
// ra từ 2 số đo kia: R = LEG_FEMUR_MM + LEG_TIBIA_MM*cos(alpha) = 30 + 55*cos(43.3°) = 69.9 mm ✓
// (khớp với R ≈ 70 mm đo được trước đó ⇒ 3 số đo độc lập ăn khớp nhau).
#define LEG_FOOT_RADIUS_MM 70.0f

// ---------------------------------------------------------------------------
// Board behaviour
// ---------------------------------------------------------------------------
// 1 = OTA URL must come from the WiFi portal; block tenclass.net / xiaozhi.me fallback.
#define BLUE_V4_BLOCK_CLOUD_SERVERS 1
#define BLUE_V4_DEFAULT_EMOTION "neutral"

// N16R8 — DO NOT USE (module / strapping / USB-JTAG):
//   3, 19–20, 26–34, 35–37, 45–46
// GPIO 0 is BOOT (strapping, used). GPIO 1 = PCA9685 OE#. See WIRING.md.

#if defined(CONFIG_LCD_ST7789_240X240)
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 240
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR true
#define DISPLAY_RGB_ORDER LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X 0
#define DISPLAY_OFFSET_Y 0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 3
#else
#error "Blue V4 requires CONFIG_LCD_ST7789_240X240 (see main/Kconfig.projbuild -> DISPLAY_LCD_TYPE)"
#endif

#endif  // _BLUE_V4_CONFIG_H_
