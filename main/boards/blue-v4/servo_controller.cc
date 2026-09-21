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
    for (int i = 0; i < SERVO_COUNT; i++) {
        char key[12];
        snprintf(key, sizeof(key), "trim%d", i);
        trim_deg_[i] = static_cast<float>(settings.GetInt(key, 0));
        snprintf(key, sizeof(key), "inv%d", i);
        inverted_[i] = settings.GetBool(key, false);
    }
    ESP_LOGI(TAG, "loaded trims from NVS namespace %s", kNvsNamespace);
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
    const float pulse =
        SERVO_MIN_PULSE_US +
        (angle / span) * static_cast<float>(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
    return static_cast<uint16_t>(lroundf(pulse));
}

bool ServoController::Enqueue(CmdType type, uint8_t joint, int32_t value) {
    if (cmd_queue_ == nullptr) {
        return false;
    }
    const Cmd cmd = {type, joint, value};
    return xQueueSend(cmd_queue_, &cmd, 0) == pdTRUE;
}

void ServoController::SetTargets(const float angles_deg[SERVO_COUNT]) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (int i = 0; i < SERVO_COUNT; i++) {
        target_deg_[i] = ClampAngle(static_cast<uint8_t>(i), angles_deg[i]);
    }
    last_target_ms_ = esp_timer_get_time() / 1000;
}

void ServoController::SetTarget(uint8_t joint, float angle_deg) {
    if (joint >= SERVO_COUNT) {
        return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    target_deg_[joint] = ClampAngle(joint, angle_deg);
    last_target_ms_ = esp_timer_get_time() / 1000;
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

void ServoController::ApplyRelaxedLocked() { relaxed_ = true; }

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
    }
}

void ServoController::Tick(float dt_s) {
    const int64_t now_ms = esp_timer_get_time() / 1000;

    bool any_write = false;
    bool moved = false;

    for (int i = 0; i < SERVO_COUNT; i++) {
        const float delta = target_deg_[i] - current_deg_[i];
        if (delta == 0.0f) {
            vel_deg_s_[i] = 0.0f;
            continue;
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
        if (hardware_ && diff >= kMinTickDeltaToWrite) {
            if (pca_.SetChannelPulseUs(static_cast<uint8_t>(i), pulse)) {
                last_ticks_[i] = ticks;
                any_write = true;
            }
        }
    }

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
    const float dt_s = static_cast<float>(SERVO_UPDATE_PERIOD_MS) / 1000.0f;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        Cmd cmd;
        if (xQueueReceive(cmd_queue_, &cmd, 0) == pdTRUE) {
            RunCommand(cmd);
            // Drain any other pending discrete commands before the next tick.
            while (xQueueReceive(cmd_queue_, &cmd, 0) == pdTRUE) {
                RunCommand(cmd);
            }
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            Tick(dt_s);
        }

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
