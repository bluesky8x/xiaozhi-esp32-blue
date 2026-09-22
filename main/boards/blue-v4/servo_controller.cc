#include "servo_controller.h"

#include "mcp_server.h"
#include "settings.h"

#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#define TAG "ServoController"

namespace {
constexpr const char* kNvsNamespace = "blue_v4_servo";
constexpr int kTaskStackWords = 4096;
constexpr UBaseType_t kTaskPriority = 4;
// Ignore sub-tick jitter when deciding whether a channel needs an I2C write.
constexpr int kMinTickDeltaToWrite = 1;
// Logical joint -> PCA9685 channel (see SERVO_CHANNEL_MAP in config.h).
constexpr uint8_t kJointChannel[SERVO_COUNT] = SERVO_CHANNEL_MAP;
}  // namespace

ServoController* ServoController::instance_ = nullptr;

ServoController* ServoController::Instance() { return instance_; }

bool ServoController::Init(i2c_master_bus_handle_t bus) {
    if (instance_ != nullptr && instance_ != this) {
        ESP_LOGW(TAG, "Init called twice — reusing existing controller");
        return ready_;
    }
    instance_ = this;

    RegisterJointLimits();
    LoadTrims();

    // Initial command state: everything at neutral, servos limp.
    for (int i = 0; i < SERVO_COUNT; i++) {
        target_deg_[i] = (min_deg_[i] + max_deg_[i]) * 0.5f;
        current_deg_[i] = target_deg_[i];
        last_ticks_[i] = 0xFFFF;
    }

    hardware_ = pca_.Init(bus, PCA9685_I2C_ADDR, SERVO_PWM_FREQ_HZ, PCA9685_OE_GPIO);
    if (!hardware_) {
        ESP_LOGE(TAG, "PCA9685 not available — servo tools will report failure");
    }

    cmd_queue_ = xQueueCreate(SERVO_CMD_QUEUE_DEPTH, sizeof(Cmd));
    if (cmd_queue_ == nullptr) {
        ESP_LOGE(TAG, "command queue allocation failed");
        return false;
    }
    if (xTaskCreate(TaskEntry, "servo_ctrl", kTaskStackWords, this, kTaskPriority, &task_) !=
        pdPASS) {
        ESP_LOGE(TAG, "worker task creation failed");
        return false;
    }

    last_target_ms_ = esp_timer_get_time() / 1000;
    relaxed_ = true;
    ready_ = true;

#if SERVO_BOOT_NEUTRAL_ENABLE
    StartBootNeutral(SERVO_BOOT_NEUTRAL_DELAY_MS);
#endif

    RegisterMcpTools();
    ESP_LOGI(TAG, "ready (%d joints, %d Hz, %d us update, slew %.0f deg/s, hardware=%d)",
             SERVO_COUNT, SERVO_PWM_FREQ_HZ, SERVO_UPDATE_PERIOD_MS,
             static_cast<double>(slew_deg_per_sec_), hardware_ ? 1 : 0);
    return true;
}

void ServoController::RegisterJointLimits() {
    for (int i = 0; i < SERVO_COUNT; i++) {
        min_deg_[i] = SERVO_DEFAULT_MIN_DEG;
        max_deg_[i] = SERVO_DEFAULT_MAX_DEG;
        trim_deg_[i] = 0.0f;
        inverted_[i] = false;
    }
}

void ServoController::LoadTrims() {
    Settings settings(kNvsNamespace, false);
    min_pulse_us_ = static_cast<uint16_t>(
        std::clamp<int32_t>(settings.GetInt("pmin", SERVO_MIN_PULSE_US), 200, 2500));
    max_pulse_us_ = static_cast<uint16_t>(
        std::clamp<int32_t>(settings.GetInt("pmax", SERVO_MAX_PULSE_US), 200, 2500));
    if (max_pulse_us_ <= min_pulse_us_) {
        min_pulse_us_ = SERVO_MIN_PULSE_US;
        max_pulse_us_ = SERVO_MAX_PULSE_US;
    }
    for (int i = 0; i < SERVO_COUNT; i++) {
        char key[12];
        snprintf(key, sizeof(key), "trim%d", i);
        trim_deg_[i] = static_cast<float>(settings.GetInt(key, 0));
        snprintf(key, sizeof(key), "inv%d", i);
        // NVS là override TUYỆT ĐỐI; mask trong config.h chỉ là giá trị mặc định nhà máy.
        // (Trước đây hai thứ XOR với nhau nên `srv:invert=J:1` là no-op im lặng với mọi joint mà
        // bit mask đã bằng 1 — đó là cách robot này bị đảo ngược knee bên trái.)
        const bool mount_inverted = ((SERVO_INVERT_DEFAULT_MASK >> i) & 0x01) != 0;
        inverted_[i] = settings.GetBool(key, mount_inverted);
    }
    ESP_LOGI(TAG, "loaded trims from NVS namespace %s (pulse band %u..%u us)", kNvsNamespace,
             static_cast<unsigned>(min_pulse_us_), static_cast<unsigned>(max_pulse_us_));
}

bool ServoController::PersistTrims() {
    Settings settings(kNvsNamespace, true);
    for (int i = 0; i < SERVO_COUNT; i++) {
        char key[12];
        snprintf(key, sizeof(key), "trim%d", i);
        settings.SetInt(key, static_cast<int32_t>(lroundf(trim_deg_[i])));
        snprintf(key, sizeof(key), "inv%d", i);
        settings.SetBool(key, inverted_[i]);
    }
    ESP_LOGI(TAG, "trims saved");
    return true;
}

float ServoController::ClampAngle(uint8_t joint, float angle_deg) const {
    if (joint >= SERVO_COUNT) {
        return SERVO_DEFAULT_NEUTRAL_DEG;
    }
    const float with_trim = angle_deg + trim_deg_[joint];
    return std::clamp(with_trim, min_deg_[joint], max_deg_[joint]);
}

uint16_t ServoController::AngleToPulseUs(uint8_t joint, float angle_deg) const {
    const float span = 180.0f;
    float angle = std::clamp(angle_deg, 0.0f, span);
    if (joint < SERVO_COUNT && inverted_[joint]) {
        angle = span - angle;
    }
    const uint16_t lo = std::min(min_pulse_us_, max_pulse_us_);
    const uint16_t hi = std::max(min_pulse_us_, max_pulse_us_);
    const float pulse = static_cast<float>(lo) + (angle / span) * static_cast<float>(hi - lo);
    return static_cast<uint16_t>(lroundf(pulse));
}

bool ServoController::Enqueue(CmdType type, uint8_t joint, int32_t value, int32_t value2,
                              int32_t value3) {
    if (cmd_queue_ == nullptr) {
        return false;
    }
    const Cmd cmd = {type, joint, value, value2, value3};
    return xQueueSend(cmd_queue_, &cmd, 0) == pdTRUE;
}

bool ServoController::SweepJoint(uint8_t joint, float to_deg, uint32_t duration_ms) {
    if (joint >= SERVO_COUNT) {
        return false;
    }
    return Enqueue(CmdType::kSweep, joint, static_cast<int32_t>(lroundf(to_deg * 10.0f)), 0,
                   static_cast<int32_t>(duration_ms));
}

bool ServoController::RawChannel(uint8_t channel, uint16_t pulse_us) {
    if (!hardware_ || channel > 15) {
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        boot_neutral_at_ms_ = 0;
        boot_release_at_ms_ = 0;
        hold_ = true;
        relaxed_ = false;
    }
    pca_.SetOutputsEnabled(true);
    const uint16_t pulse = static_cast<uint16_t>(std::clamp<int32_t>(pulse_us, 200, 2500));
    const bool ok = pca_.SetChannelPulseUs(channel, pulse);
    ESP_LOGI(TAG, "raw channel %u = %u us (%s)", channel, pulse, ok ? "ok" : "failed");
    return ok;
}

void ServoController::SetPulseRange(uint16_t min_us, uint16_t max_us) {
    const uint16_t lo = std::clamp<uint16_t>(min_us, 200, 2500);
    const uint16_t hi = std::clamp<uint16_t>(max_us, 200, 2500);
    if (hi <= lo) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        min_pulse_us_ = lo;
        max_pulse_us_ = hi;
        for (int i = 0; i < SERVO_COUNT; i++) {
            last_ticks_[i] = 0xFFFF;  // force a rewrite with the new mapping
        }
    }
    Settings settings(kNvsNamespace, true);
    settings.SetInt("pmin", lo);
    settings.SetInt("pmax", hi);
    ESP_LOGI(TAG, "pulse range = %u..%u us (saved)", lo, hi);
}

void ServoController::GetPulseRange(uint16_t* min_us, uint16_t* max_us) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (min_us != nullptr) {
        *min_us = min_pulse_us_;
    }
    if (max_us != nullptr) {
        *max_us = max_pulse_us_;
    }
}

bool ServoController::RawPulse(uint8_t joint, uint16_t pulse_us) {
    if (joint >= SERVO_COUNT) {
        return false;
    }
    const float span = 180.0f;
    const float lo = static_cast<float>(std::min(min_pulse_us_, max_pulse_us_));
    const float hi = static_cast<float>(std::max(min_pulse_us_, max_pulse_us_));
    const float p = std::clamp(static_cast<float>(pulse_us), lo, hi);
    float angle = (p - lo) * span / (hi - lo);
    if (inverted_[joint]) {
        angle = span - angle;
    }
    angle -= trim_deg_[joint];  // ClampAngle() adds the trim back
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        boot_neutral_at_ms_ = 0;
        boot_release_at_ms_ = 0;
        sweep_end_ms_ = 0;
        slew_deg_per_sec_ = 400.0f;
        accel_deg_per_sec2_ = 2000.0f;
        hold_ = true;
        relaxed_ = false;
    }
    if (hardware_) {
        pca_.SetOutputsEnabled(true);
    }
    SetTarget(joint, angle);
    ESP_LOGI(TAG, "raw pulse: joint %d = %u us (%.1f deg)", joint, pulse_us,
             static_cast<double>(angle));
    return true;
}

void ServoController::SetTargets(const float angles_deg[SERVO_COUNT]) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    boot_neutral_at_ms_ = 0;  // an explicit pose wins over the boot pose
    const int64_t now_ms = esp_timer_get_time() / 1000;

    float clamped[SERVO_COUNT];
    bool stepping[SERVO_COUNT] = {};
    int stepped = 0;
    for (int i = 0; i < SERVO_COUNT; i++) {
        clamped[i] = ClampAngle(static_cast<uint8_t>(i), angles_deg[i]);
        stepping[i] = fabsf(clamped[i] - target_deg_[i]) >= SERVO_STAGGER_MIN_STEP_DEG;
        if (stepping[i]) {
            stepped++;
        }
    }
    // Giãn nhịp khởi động (xem SERVO_STAGGER_* trong config.h): chỉ áp dụng cho lệnh làm nhiều
    // servo nhảy cùng lúc. Một lệnh khác (kể cả dòng nội suy của gait) luôn xoá hold ⇒ không có
    // servo nào bị kẹt chờ.
    const bool stagger = SERVO_STAGGER_START_ENABLE && stepped >= 2;
    int slot = 0;
    for (int i = 0; i < SERVO_COUNT; i++) {
        target_deg_[i] = clamped[i];
        hold_until_ms_[i] =
            (stagger && stepping[i]) ? now_ms + static_cast<int64_t>(slot++) * SERVO_STAGGER_MS : 0;
    }
    last_target_ms_ = now_ms;
}

void ServoController::SetTarget(uint8_t joint, float angle_deg) {
    if (joint >= SERVO_COUNT) {
        return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    boot_neutral_at_ms_ = 0;  // an explicit target wins over the boot pose
    target_deg_[joint] = ClampAngle(joint, angle_deg);
    hold_until_ms_[joint] = 0;
    last_target_ms_ = esp_timer_get_time() / 1000;
}

void ServoController::StartBootNeutral(uint32_t delay_ms) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    boot_neutral_at_ms_ = esp_timer_get_time() / 1000 + static_cast<int64_t>(delay_ms);
}

void ServoController::SetSlewDegPerSec(float deg_per_sec) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    slew_deg_per_sec_ = std::max(5.0f, deg_per_sec);
}

void ServoController::SetAccelDegPerSec2(float deg_per_sec2) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    accel_deg_per_sec2_ = std::max(10.0f, deg_per_sec2);
}

void ServoController::SetHold(bool hold) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    hold_ = hold;
    if (hold) {
        last_target_ms_ = esp_timer_get_time() / 1000;
    }
}

bool ServoController::Relax() { return Enqueue(CmdType::kRelax); }

bool ServoController::EnableOutputs() { return Enqueue(CmdType::kEnable); }

bool ServoController::SaveTrims() { return Enqueue(CmdType::kSaveTrims); }

bool ServoController::IsRelaxed() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return relaxed_;
}

bool ServoController::OutputsEnabled() const { return pca_.OutputsEnabled(); }

bool ServoController::HasHardware() const { return hardware_; }

bool ServoController::ShouldPauseUplink() {
#if BLUE_V4_PAUSE_UPLINK_WHILE_MOVING
    ServoController* self = instance_;
    return self != nullptr && self->IsMoving();
#else
    return false;
#endif
}

bool ServoController::IsMoving() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (fabsf(target_deg_[i] - current_deg_[i]) > 0.75f) {
            return true;
        }
    }
    return false;
}

void ServoController::GetCommandedAngles(float out[SERVO_COUNT]) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (int i = 0; i < SERVO_COUNT; i++) {
        out[i] = current_deg_[i];
    }
}

void ServoController::GetTrimAngles(float out[SERVO_COUNT]) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (int i = 0; i < SERVO_COUNT; i++) {
        out[i] = trim_deg_[i];
    }
}

void ServoController::GetCommandedPulseUs(uint16_t out[SERVO_COUNT]) const {
    float angles[SERVO_COUNT];
    GetCommandedAngles(angles);
    for (int i = 0; i < SERVO_COUNT; i++) {
        out[i] = AngleToPulseUs(static_cast<uint8_t>(i), angles[i]);
    }
}

std::string ServoController::SetTrim(uint8_t joint, int trim_deg) {
    if (joint >= SERVO_COUNT) {
        return "error: joint must be 0-" + std::to_string(SERVO_COUNT - 1);
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        trim_deg_[joint] = static_cast<float>(trim_deg);
    }
    Enqueue(CmdType::kSetTrim, joint, trim_deg);
    return "joint " + std::to_string(joint) + " trim = " + std::to_string(trim_deg) +
           " deg (persisted)";
}

std::string ServoController::GetTrimsJson() const {
    float angles[SERVO_COUNT];
    GetCommandedAngles(angles);

    std::string json = "{\"trims\":[";
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (i > 0) {
            json += ",";
        }
        json += std::to_string(static_cast<int>(lroundf(trim_deg_[i])));
    }
    json += "],\"inverted\":[";
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (i > 0) {
            json += ",";
        }
        json += inverted_[i] ? "true" : "false";
    }
    json += "],\"commanded_deg\":[";
    for (int i = 0; i < SERVO_COUNT; i++) {
        if (i > 0) {
            json += ",";
        }
        char buf[16];
        snprintf(buf, sizeof(buf), "%.1f", static_cast<double>(angles[i]));
        json += buf;
    }
    json += "]}";
    return json;
}

void ServoController::SetInverted(uint8_t joint, bool inverted) {
    if (joint >= SERVO_COUNT) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        inverted_[joint] = inverted;
        last_ticks_[joint] = 0xFFFF;
    }
    Enqueue(CmdType::kSetInverted, joint, inverted ? 1 : 0);
}

bool ServoController::GetInverted(uint8_t joint) const {
    if (joint >= SERVO_COUNT) {
        return false;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    return inverted_[joint];
}

void ServoController::ApplyEnabledLocked() {
    // Force a full rewrite after OE# was released.
    for (int i = 0; i < SERVO_COUNT; i++) {
        last_ticks_[i] = 0xFFFF;
    }
    relaxed_ = false;
}

void ServoController::ApplyRelaxedLocked() {
    relaxed_ = true;
    for (int i = 0; i < SERVO_COUNT; i++) {
        hold_until_ms_[i] = 0;  // bỏ mọi lượt chờ giãn nhịp còn treo
        vel_deg_s_[i] = 0.0f;
    }
}

void ServoController::RunCommand(const Cmd& cmd) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    switch (cmd.type) {
        case CmdType::kRelax:
            if (hardware_) {
                pca_.SetAllOff();
                pca_.SetOutputsEnabled(false);
            }
            ApplyRelaxedLocked();
            ESP_LOGI(TAG, "relaxed (PWM off, OE# high)");
            break;
        case CmdType::kEnable:
            if (hardware_) {
                pca_.SetOutputsEnabled(true);
            }
            ApplyEnabledLocked();
            ESP_LOGI(TAG, "outputs enabled");
            break;
        case CmdType::kSaveTrims:
            PersistTrims();
            break;
        case CmdType::kSetTrim:
            if (cmd.joint < SERVO_COUNT) {
                last_ticks_[cmd.joint] = 0xFFFF;
            }
            PersistTrims();
            break;
        case CmdType::kSetInverted:
            PersistTrims();
            break;
        case CmdType::kStop:
            if (hardware_) {
                pca_.SetAllOff();
                pca_.SetOutputsEnabled(false);
            }
            ApplyRelaxedLocked();
            hold_ = false;
            ESP_LOGI(TAG, "stopped (relaxed)");
            break;
        case CmdType::kSweep: {
            if (cmd.joint >= SERVO_COUNT) {
                break;
            }
            if (hardware_) {
                pca_.SetOutputsEnabled(true);
            }
            ApplyEnabledLocked();
            hold_ = true;
            boot_release_at_ms_ = 0;
            boot_neutral_at_ms_ = 0;
            sweep_joint_ = cmd.joint;
            sweep_from_deg_ = current_deg_[cmd.joint];
            sweep_to_deg_ = static_cast<float>(cmd.value) * 0.1f;
            sweep_start_ms_ = esp_timer_get_time() / 1000;
            const int64_t duration = std::max<int64_t>(cmd.value3, 200);
            sweep_end_ms_ = sweep_start_ms_ + duration;
            slew_deg_per_sec_ = std::max(30.0f, fabsf(sweep_to_deg_ - sweep_from_deg_) * 2000.0f /
                                                    static_cast<float>(duration));
            accel_deg_per_sec2_ = std::max(400.0f, slew_deg_per_sec_ * 4.0f);
            ESP_LOGI(TAG, "sweep joint %d: %.1f -> %.1f deg over %lld ms", cmd.joint,
                     static_cast<double>(sweep_from_deg_), static_cast<double>(sweep_to_deg_),
                     duration);
            break;
        }
    }
}

void ServoController::Tick(float dt_s) {
    const int64_t now_ms = esp_timer_get_time() / 1000;

    if (boot_release_at_ms_ != 0 && now_ms >= boot_release_at_ms_) {
        // The boot pose has been held long enough: drop the hold so the idle timeout can relax the
        // servos (stalled servos draw a lot of current).
        boot_release_at_ms_ = 0;
        hold_ = false;
        last_target_ms_ = now_ms;
        ESP_LOGI(TAG, "boot pose released — idle relax may now switch the PWM off");
    }

    if (sweep_end_ms_ != 0) {
        const float span = static_cast<float>(sweep_end_ms_ - sweep_start_ms_);
        float progress = span > 1.0f ? static_cast<float>(now_ms - sweep_start_ms_) / span : 1.0f;
        progress = std::clamp(progress, 0.0f, 1.0f);
        const float k = progress * progress * (3.0f - 2.0f * progress);
        target_deg_[sweep_joint_] =
            ClampAngle(sweep_joint_, sweep_from_deg_ + (sweep_to_deg_ - sweep_from_deg_) * k);
        if (progress >= 1.0f) {
            target_deg_[sweep_joint_] = ClampAngle(sweep_joint_, sweep_to_deg_);
            sweep_end_ms_ = 0;
            ESP_LOGI(TAG, "sweep done: joint %d at %.1f deg", sweep_joint_,
                     static_cast<double>(current_deg_[sweep_joint_]));
        }
    }

    if (boot_neutral_at_ms_ != 0 && now_ms >= boot_neutral_at_ms_) {
        // Boot pose: nothing was published yet, so energise the joints at the neutral point
        // and hold them there. The pulse is written once; the servos then travel to neutral
        // at their own speed (MG90S has no position feedback).
        boot_neutral_at_ms_ = 0;
        for (int i = 0; i < SERVO_COUNT; i++) {
            target_deg_[i] = ClampAngle(static_cast<uint8_t>(i), SERVO_DEFAULT_NEUTRAL_DEG);
            current_deg_[i] = target_deg_[i];
            vel_deg_s_[i] = 0.0f;
            last_ticks_[i] = 0xFFFF;
        }
        if (hardware_) {
            pca_.SetOutputsEnabled(true);
            for (int i = 0; i < SERVO_COUNT; i++) {
                const uint16_t pulse = AngleToPulseUs(static_cast<uint8_t>(i), current_deg_[i]);
                if (pca_.SetChannelPulseUs(kJointChannel[i], pulse)) {
                    last_ticks_[i] = pca_.PulseUsToTicks(pulse);
                }
            }
        }
        relaxed_ = false;
        hold_ = true;
        slew_deg_per_sec_ = SERVO_BOOT_NEUTRAL_SLEW_DEG_PER_SEC;
        last_target_ms_ = now_ms;
        boot_release_at_ms_ = now_ms + SERVO_BOOT_NEUTRAL_HOLD_MS;
        ESP_LOGI(TAG, "boot pose: %d joints held at neutral %.0f deg", SERVO_COUNT,
                 static_cast<double>(SERVO_DEFAULT_NEUTRAL_DEG));
    }

    bool any_write = false;
    bool moved = false;

    // Giãn nhịp ghi PWM: bắt đầu từ một kênh xoay vòng và chỉ ghi tối đa
    // SERVO_PWM_MAX_WRITES_PER_TICK kênh mỗi tick, để 8 kênh không dồn vào cùng một thời điểm.
    int write_budget = SERVO_PWM_MAX_WRITES_PER_TICK;
    const int write_start = pwm_write_cursor_;

    for (int n = 0; n < SERVO_COUNT; n++) {
        const int i = (write_start + n) % SERVO_COUNT;
        const float delta = target_deg_[i] - current_deg_[i];
        if (delta == 0.0f) {
            vel_deg_s_[i] = 0.0f;
            continue;
        }
        // Giãn nhịp khởi động: servo chưa tới lượt thì còn đứng yên (không khởi động cùng lúc).
        if (hold_until_ms_[i] != 0) {
            if (now_ms < hold_until_ms_[i]) {
                vel_deg_s_[i] = 0.0f;
                continue;
            }
            hold_until_ms_[i] = 0;
        }

        // Trapezoid speed profile: ramp up at accel_deg_per_sec2_, then ramp down so we
        // arrive at the target with ~zero speed — soft start/stop, no overshoot/jitter.
        const float dist = fabsf(delta);
        const float dir = delta > 0.0f ? 1.0f : -1.0f;
        if (vel_deg_s_[i] * dir < 0.0f) {
            vel_deg_s_[i] = 0.0f;  // reversed direction: start from rest
        }
        float limit = sqrtf(2.0f * accel_deg_per_sec2_ * dist);
        if (limit > slew_deg_per_sec_) {
            limit = slew_deg_per_sec_;
        }
        float vel = vel_deg_s_[i] + accel_deg_per_sec2_ * dt_s;
        if (vel > limit) {
            vel = limit;
        }
        if (vel < 1.0f) {
            vel = std::min(limit, 1.0f);  // never crawl slower than 1 deg/s
        }
        vel_deg_s_[i] = vel;

        const float step = vel * dt_s;
        if (step >= dist) {
            current_deg_[i] = target_deg_[i];
            vel_deg_s_[i] = 0.0f;
        } else {
            current_deg_[i] += dir * step;
        }
        moved = true;

        const uint16_t pulse = AngleToPulseUs(static_cast<uint8_t>(i), current_deg_[i]);
        const uint16_t ticks = pca_.PulseUsToTicks(pulse);
        const int diff =
            last_ticks_[i] == 0xFFFF ? 1000 : abs(static_cast<int>(ticks) - last_ticks_[i]);
        if (hardware_ && diff >= kMinTickDeltaToWrite && write_budget > 0) {
            write_budget--;  // kênh chưa được ghi sẽ vẫn "cần ghi" ở tick sau
            if (pca_.SetChannelPulseUs(kJointChannel[i], pulse)) {
                last_ticks_[i] = ticks;
                any_write = true;
                tick_writes_++;
            } else {
                tick_write_fails_++;
            }
        }
    }
    pwm_write_cursor_ = static_cast<uint8_t>((write_start + 1) % SERVO_COUNT);

    if (!moved && !any_write && !hold_ && !relaxed_ &&
        (now_ms - last_target_ms_) > SERVO_IDLE_RELAX_MS) {
        if (hardware_) {
            pca_.SetAllOff();
            pca_.SetOutputsEnabled(false);
        }
        relaxed_ = true;
        ESP_LOGI(TAG, "idle %lld ms — servos relaxed (PWM off)", now_ms - last_target_ms_);
    }
}

void ServoController::TaskEntry(void* arg) { static_cast<ServoController*>(arg)->TaskLoop(); }

void ServoController::TaskLoop() {
    const TickType_t period = pdMS_TO_TICKS(SERVO_UPDATE_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_us = esp_timer_get_time();
    int64_t log_us = last_us;
    uint32_t loop_count = 0;
    uint32_t writes = 0;
    uint32_t fails = 0;
    float last_dt_ms = 0.0f;

    while (true) {
        Cmd cmd;
        if (xQueueReceive(cmd_queue_, &cmd, 0) == pdTRUE) {
            RunCommand(cmd);
            // Drain any other pending discrete commands before the next tick.
            while (xQueueReceive(cmd_queue_, &cmd, 0) == pdTRUE) {
                RunCommand(cmd);
            }
        }

        // Interpolate with the REAL elapsed time, not the nominal period: if a blocking I2C
        // write or a busy bus slows this task down, the profile must still advance at the
        // configured deg/s instead of crawling (each loop only moves vel * dt).
        const int64_t now_us = esp_timer_get_time();
        float dt = static_cast<float>(now_us - last_us) / 1000000.0f;
        last_us = now_us;
        if (dt < 0.005f) {
            dt = 0.005f;
        } else if (dt > 0.25f) {
            dt = 0.25f;  // cap catch-up so a long stall cannot cause a violent jump
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            Tick(dt);
            writes += tick_writes_;
            fails += tick_write_fails_;
            tick_writes_ = 0;
            tick_write_fails_ = 0;
        }
        last_dt_ms = dt * 1000.0f;
        loop_count++;

#if SERVO_TICK_DEBUG_LOG
        if (now_us - log_us >= 1000000) {
            const float span_ms = static_cast<float>(now_us - log_us) / 1000.0f;
            float cur[2];
            float tgt[2];
            bool moving = false;
            bool hold = false;
            bool relaxed = true;
            float slew = 0.0f;
            float accel = 0.0f;
            float vel0 = 0.0f;
            uint16_t pulse0 = 0;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                for (int i = 0; i < 2; i++) {
                    cur[i] = current_deg_[i];
                    tgt[i] = target_deg_[i];
                }
                moving = false;
                for (int i = 0; i < SERVO_COUNT; i++) {
                    if (fabsf(target_deg_[i] - current_deg_[i]) > 0.75f) {
                        moving = true;
                        break;
                    }
                }
                hold = hold_;
                relaxed = relaxed_;
                slew = slew_deg_per_sec_;
                accel = accel_deg_per_sec2_;
                vel0 = vel_deg_s_[0];
                pulse0 = AngleToPulseUs(0, current_deg_[0]);
            }
            ESP_LOGI(TAG,
                     "tick %.0f Hz (%.1f ms) writes %u fails %u hw %d hd %d rlx %d mv %d | "
                     "sl %.0f ac %.0f v0 %.1f dt %.1f | j0 %.1f->%.1f (%u us) j1 %.1f->%.1f",
                     static_cast<double>(static_cast<float>(loop_count) * 1000.0f / span_ms),
                     static_cast<double>(span_ms / static_cast<float>(loop_count)), writes, fails,
                     hardware_ ? 1 : 0, hold ? 1 : 0, relaxed ? 1 : 0, moving ? 1 : 0,
                     static_cast<double>(slew), static_cast<double>(accel),
                     static_cast<double>(vel0), static_cast<double>(last_dt_ms),
                     static_cast<double>(cur[0]), static_cast<double>(tgt[0]), pulse0,
                     static_cast<double>(cur[1]), static_cast<double>(tgt[1]));
            loop_count = 0;
            writes = 0;
            fails = 0;
            log_us = now_us;
        }
#endif

        vTaskDelayUntil(&last_wake, period);
    }
}

void ServoController::RegisterMcpTools() {
    auto& mcp = McpServer::GetInstance();

    mcp.AddTool("self.servo.set",
                "Move one servo joint to an angle in degrees (0-180). Joints: 0/1 front-left "
                "hip/knee, 2/3 front-right hip/knee, 4/5 rear-left hip/knee, 6/7 rear-right "
                "hip/knee. The leg gait engine normally owns these — use this for small "
                "corrections or testing.",
                PropertyList({Property("joint", kPropertyTypeInteger, 0, SERVO_COUNT - 1),
                              Property("angle", kPropertyTypeInteger, 0, 180)}),
                [this](const PropertyList& properties) -> ReturnValue {
                    const int joint = properties["joint"].value<int>();
                    const int angle = properties["angle"].value<int>();
                    if (!hardware_) {
                        return std::string("error: PCA9685 not detected");
                    }
                    if (IsRelaxed()) {
                        EnableOutputs();
                    }
                    SetTarget(static_cast<uint8_t>(joint), static_cast<float>(angle));
                    return std::string("joint " + std::to_string(joint) + " -> " +
                                       std::to_string(angle) + " deg");
                });

    mcp.AddTool("self.servo.sweep",
                "Bring-up test: sweep ONE joint from its current angle to to_deg over duration_ms "
                "using a timed smoothstep curve (no gait engine involved). Use it to prove the "
                "PWM/servo path works, e.g. joint=0 to_deg=120 duration_ms=3000 then "
                "to_deg=60 duration_ms=3000. Joints 0..7 = FL hip/knee, FR hip/knee, RL hip/knee, "
                "RR hip/knee.",
                PropertyList({Property("joint", kPropertyTypeInteger, 0, 0, SERVO_COUNT - 1),
                              Property("to_deg", kPropertyTypeInteger, 120, 0, 180),
                              Property("duration_ms", kPropertyTypeInteger, 3000, 200, 15000)}),
                [this](const PropertyList& properties) -> ReturnValue {
                    const int joint = properties["joint"].value<int>();
                    const int to_deg = properties["to_deg"].value<int>();
                    const int duration_ms = properties["duration_ms"].value<int>();
                    if (!hardware_) {
                        return std::string("error: PCA9685 not detected");
                    }
                    if (!SweepJoint(static_cast<uint8_t>(joint), static_cast<float>(to_deg),
                                    static_cast<uint32_t>(duration_ms))) {
                        return std::string("error: queue full");
                    }
                    char buf[128];
                    snprintf(buf, sizeof(buf), "{\"ok\":true,\"joint\":%d,\"to_deg\":%d,\"ms\":%d}",
                             joint, to_deg, duration_ms);
                    return std::string(buf);
                });

    mcp.AddTool(
        "self.servo.pulse_range",
        "Get/set the raw pulse band (microseconds) that maps onto 0..180 deg, persisted in "
        "NVS. Most MG90S-style servos accept about 1000..2000 us; the firmware default is "
        "500..2500 us, which can push a servo past its mechanical stop (the servo then just "
        "ticks and does not turn). Pass min_us and max_us to change it; pass 0/0 to just "
        "read the current values.",
        PropertyList({Property("min_us", kPropertyTypeInteger, 0, 0, 2500),
                      Property("max_us", kPropertyTypeInteger, 0, 0, 2500)}),
        [this](const PropertyList& properties) -> ReturnValue {
            const int min_us = properties["min_us"].value<int>();
            const int max_us = properties["max_us"].value<int>();
            if (min_us > 0 && max_us > min_us) {
                SetPulseRange(static_cast<uint16_t>(min_us), static_cast<uint16_t>(max_us));
            }
            uint16_t lo = 0;
            uint16_t hi = 0;
            GetPulseRange(&lo, &hi);
            char buf[128];
            snprintf(buf, sizeof(buf),
                     "{\"ok\":true,\"min_us\":%u,\"max_us\":%u,\"joint0_deg\":%.1f}", lo, hi,
                     static_cast<double>(current_deg_[0]));
            return std::string(buf);
        });

    mcp.AddTool(
        "self.servo.raw_pulse",
        "Hardware test: drive ONE joint with an exact pulse width in microseconds, bypassing "
        "the angle mapping. Use 1500 us (centre), 1200 us and 1800 us to check the servo "
        "actually turns; e.g. raw_pulse joint=0 pulse_us=1200 then 1800.",
        PropertyList({Property("joint", kPropertyTypeInteger, 0, 0, SERVO_COUNT - 1),
                      Property("pulse_us", kPropertyTypeInteger, 1500, 200, 2500)}),
        [this](const PropertyList& properties) -> ReturnValue {
            const int joint = properties["joint"].value<int>();
            const int pulse_us = properties["pulse_us"].value<int>();
            if (!hardware_) {
                return std::string("error: PCA9685 not detected");
            }
            if (!RawPulse(static_cast<uint8_t>(joint), static_cast<uint16_t>(pulse_us))) {
                return std::string("error: bad joint");
            }
            char buf[128];
            snprintf(buf, sizeof(buf), "{\"ok\":true,\"joint\":%d,\"pulse_us\":%d}", joint,
                     pulse_us);
            return std::string(buf);
        });

    mcp.AddTool(
        "self.servo.raw_channel",
        "Hardware test: drive ANY PCA9685 channel 0..15 with an exact pulse width, ignoring the "
        "joint map. Use it to test a spare channel (8..15) with a known-good servo: if the servo "
        "runs on channel 8 but not on its own channel, that driver channel is damaged — re-plug "
        "the servo into the spare channel and update SERVO_CHANNEL_MAP in config.h.",
        PropertyList({Property("channel", kPropertyTypeInteger, 0, 0, 15),
                      Property("pulse_us", kPropertyTypeInteger, 1500, 200, 2500)}),
        [this](const PropertyList& properties) -> ReturnValue {
            const int channel = properties["channel"].value<int>();
            const int pulse_us = properties["pulse_us"].value<int>();
            if (!hardware_) {
                return std::string("error: PCA9685 not detected");
            }
            if (!RawChannel(static_cast<uint8_t>(channel), static_cast<uint16_t>(pulse_us))) {
                return std::string("error: channel write failed");
            }
            char buf[128];
            snprintf(buf, sizeof(buf), "{\"ok\":true,\"channel\":%d,\"pulse_us\":%d}", channel,
                     pulse_us);
            return std::string(buf);
        });

    mcp.AddTool("self.servo.set_all",
                "Set all 8 joint angles at once. Pass a comma-separated list of 8 values in "
                "joint order 0..7 (front-left hip/knee, front-right hip/knee, rear-left "
                "hip/knee, rear-right hip/knee), e.g. \"90,90,90,90,90,90,90,90\".",
                PropertyList({Property("angles", kPropertyTypeString)}),
                [this](const PropertyList& properties) -> ReturnValue {
                    const std::string csv = properties["angles"].value<std::string>();
                    if (!hardware_) {
                        return std::string("error: PCA9685 not detected");
                    }
                    float angles[SERVO_COUNT] = {};
                    int parsed = 0;
                    size_t start = 0;
                    while (parsed < SERVO_COUNT && start <= csv.size()) {
                        const size_t comma = csv.find(',', start);
                        const std::string token = csv.substr(
                            start, comma == std::string::npos ? std::string::npos : comma - start);
                        if (!token.empty()) {
                            angles[parsed++] = strtof(token.c_str(), nullptr);
                        }
                        if (comma == std::string::npos) {
                            break;
                        }
                        start = comma + 1;
                    }
                    if (parsed != SERVO_COUNT) {
                        return std::string("error: expected 8 comma-separated angles, got " +
                                           std::to_string(parsed));
                    }
                    if (IsRelaxed()) {
                        EnableOutputs();
                    }
                    SetTargets(angles);
                    return std::string("8 joints updated");
                });

    mcp.AddTool("self.servo.get_positions",
                "Read the current commanded joint angles and trims as JSON (debug).",
                PropertyList(),
                [this](const PropertyList&) -> ReturnValue { return GetTrimsJson(); });

    mcp.AddTool("self.servo.trim",
                "Set a servo's mechanical trim offset in whole degrees (range -30..30) and "
                "persist it. Use after mounting a horn so the joint's neutral matches 90 deg.",
                PropertyList({Property("joint", kPropertyTypeInteger, 0, SERVO_COUNT - 1),
                              Property("trim", kPropertyTypeInteger, 0, -30, 30)}),
                [this](const PropertyList& properties) -> ReturnValue {
                    return SetTrim(static_cast<uint8_t>(properties["joint"].value<int>()),
                                   properties["trim"].value<int>());
                });

    mcp.AddTool("self.servo.invert",
                "Invert one joint's direction (use when a mirrored leg moves the wrong way). "
                "Persisted in NVS.",
                PropertyList({Property("joint", kPropertyTypeInteger, 0, SERVO_COUNT - 1),
                              Property("inverted", kPropertyTypeInteger, 0, 0, 1)}),
                [this](const PropertyList& properties) -> ReturnValue {
                    const uint8_t joint = static_cast<uint8_t>(properties["joint"].value<int>());
                    const bool inverted = properties["inverted"].value<int>() != 0;
                    SetInverted(joint, inverted);
                    return std::string("joint " + std::to_string(joint) +
                                       (inverted ? " inverted" : " normal") + " (persisted)");
                });

    mcp.AddTool("self.servo.relax",
                "Cut servo torque (PWM off, OE# high) so the robot goes limp. Use when the "
                "user asks to relax, stop holding a pose, or before powering down. Use for: "
                "nghỉ, thả lỏng, tắt servo, relax.",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    if (!hardware_) {
                        return std::string("error: PCA9685 not detected");
                    }
                    return Relax() ? std::string("servos relaxed") : std::string("error: busy");
                });

    mcp.AddTool("self.servo.enable",
                "Energise the servo outputs again after a relax (PWM on, OE# low).", PropertyList(),
                [this](const PropertyList&) -> ReturnValue {
                    if (!hardware_) {
                        return std::string("error: PCA9685 not detected");
                    }
                    return EnableOutputs() ? std::string("servo outputs enabled")
                                           : std::string("error: busy");
                });

    mcp.AddTool("self.servo.stop",
                "Immediate safety stop: ramp nothing, cut PWM and release the servos, and "
                "clear any held pose. Use for: dừng lại, đứng im, stop.",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    if (!hardware_) {
                        return std::string("error: PCA9685 not detected");
                    }
                    return Enqueue(CmdType::kStop) ? std::string("servos stopped (relaxed)")
                                                   : std::string("error: busy");
                });
}
