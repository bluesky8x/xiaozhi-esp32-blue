#include "motor_controller.h"

#include "application.h"
#include "config.h"
#include "mcp_server.h"
#include "tof_controller.h"
#include "tof_safety_check.h"

#if CONFIG_BOARD_TYPE_BLUE_V2
#include "assets/lang_config.h"
#include "motor_dance.h"
#endif

#include <algorithm>
#include <esp_timer.h>
#include <string_view>

MotorController* MotorController::instance_ = nullptr;

MotorController::MotorController(gpio_num_t left_in1, gpio_num_t left_in2, gpio_num_t right_in1,
                                 gpio_num_t right_in2)
    : left_in1_(left_in1),
      left_in2_(left_in2),
      right_in1_(right_in1),
      right_in2_(right_in2) {
    instance_ = this;
    InitMotorGpio();

    cmd_queue_ = xQueueCreate(MOTOR_CMD_QUEUE_DEPTH, sizeof(MotorCommand));
    ESP_ERROR_CHECK(cmd_queue_ != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

    esp_timer_create_args_t timer_args = {
        .callback = StopTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "motor_stop",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &stop_timer_));

    constexpr uint32_t kStackWords = 3072;
    constexpr UBaseType_t kPriority = 3;
    BaseType_t created = xTaskCreatePinnedToCore(WorkerTaskEntry, "motor_worker", kStackWords, this,
                                                 kPriority, &worker_task_, 0);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_FAIL);

    RegisterMcpTools();

    ESP_LOGI(MOTOR_TAG,
             "Motor worker started (queue=%d, GPIO %d/%d/%d/%d, brake=%d %dms, max_duty=%d%%)",
             MOTOR_CMD_QUEUE_DEPTH, left_in1_, left_in2_, right_in1_, right_in2_, MOTOR_BRAKE_ENABLE,
             MOTOR_BRAKE_MS, MOTOR_MAX_DUTY_PCT);
}

MotorController::~MotorController() {
    if (stop_timer_ != nullptr) {
        esp_timer_stop(stop_timer_);
        esp_timer_delete(stop_timer_);
        stop_timer_ = nullptr;
    }
    if (worker_task_ != nullptr) {
        vTaskDelete(worker_task_);
        worker_task_ = nullptr;
    }
    ExecuteStop();
    if (cmd_queue_ != nullptr) {
        vQueueDelete(cmd_queue_);
        cmd_queue_ = nullptr;
    }
    if (instance_ == this) {
        instance_ = nullptr;
    }
}

bool MotorController::ShouldPauseUplink() {
    return instance_ != nullptr &&
           (instance_->IsMoving() || instance_->IsDancing());
}

void MotorController::WorkerTaskEntry(void* arg) {
    static_cast<MotorController*>(arg)->WorkerLoop();
}

void MotorController::WorkerLoop() {
    MotorCommand cmd{};
    while (true) {
        if (xQueueReceive(cmd_queue_, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (cmd.type == CmdType::kStop) {
            ExecuteStop();
        } else if (cmd.type == CmdType::kDance) {
            TofSafetyStopReason reason = TofSafetyStopReason::None;
            if (!TofPreMoveCheck(-70, 70, &reason)) {
                ESP_LOGW(MOTOR_TAG, "Dance skipped — pre-move ToF (%s)",
                         TofSafetyStopReasonName(reason));
                continue;
            }
            RunDanceRoutine(cmd.dance_track);
        } else {
            // First move in queue (motors idle): fresh ToF check before GPIO drive.
            if (!IsMoving()) {
                TofSafetyStopReason reason = TofSafetyStopReason::None;
                if (!TofPreMoveCheck(cmd.left, cmd.right, &reason)) {
                    ESP_LOGW(MOTOR_TAG, "Move skipped — pre-move ToF (%s)", TofSafetyStopReasonName(reason));
                    continue;
                }
            }
            ExecuteMove(cmd.left, cmd.right, cmd.duration_ms);
        }
    }
}

void MotorController::StopTimerCallback(void* arg) {
    static_cast<MotorController*>(arg)->EnqueueStopFromTimer();
}

void MotorController::EnqueueStopFromTimer() {
    MotorCommand cmd = {
        .type = CmdType::kStop,
        .left = 0,
        .right = 0,
        .duration_ms = 0,
    };
    PushCommand(cmd, true);
}

bool MotorController::PushCommand(const MotorCommand& cmd, bool clear_pending) {
    if (cmd_queue_ == nullptr) {
        return false;
    }
    if (clear_pending) {
        xQueueReset(cmd_queue_);
    }
    if (xQueueSend(cmd_queue_, &cmd, 0) != pdTRUE) {
        ESP_LOGW(MOTOR_TAG, "Motor command queue full (type=%d)", static_cast<int>(cmd.type));
        return false;
    }
    return true;
}

bool MotorController::EnqueueMove(int left_speed, int right_speed, int duration_ms) {
    left_speed = std::clamp(left_speed, -100, 100);
    right_speed = std::clamp(right_speed, -100, 100);
    duration_ms = std::clamp(duration_ms, 0, 10000);
    MotorCommand cmd = {
        .type = CmdType::kMove,
        .left = static_cast<int8_t>(left_speed),
        .right = static_cast<int8_t>(right_speed),
        .duration_ms = static_cast<int16_t>(duration_ms),
    };
    return PushCommand(cmd, false);
}

bool MotorController::EnqueueStop() {
    MotorCommand cmd = {
        .type = CmdType::kStop,
        .left = 0,
        .right = 0,
        .duration_ms = 0,
    };
    return PushCommand(cmd, true);
}

bool MotorController::EnqueueDance(int track) {
    uint8_t normalized = 1;
    if (track == 3) {
        normalized = 3;
    } else if (track == 2) {
        normalized = 2;
    }
    MotorCommand cmd = {
        .type = CmdType::kDance,
        .left = 0,
        .right = 0,
        .duration_ms = 0,
        .dance_track = normalized,
    };
    return PushCommand(cmd, true);
}

void MotorController::RequestDanceStop() {
    if (!dancing_.load(std::memory_order_acquire)) {
        return;
    }
    dance_cancel_.store(true, std::memory_order_release);
    EnqueueStop();
}

int* MotorController::LevelCacheFor(gpio_num_t pin) {
    if (pin == left_in1_) {
        return &cached_level_left_in1_;
    }
    if (pin == left_in2_) {
        return &cached_level_left_in2_;
    }
    if (pin == right_in1_) {
        return &cached_level_right_in1_;
    }
    if (pin == right_in2_) {
        return &cached_level_right_in2_;
    }
    return nullptr;
}

void MotorController::ResetLevelCache() {
    cached_level_left_in1_ = -1;
    cached_level_left_in2_ = -1;
    cached_level_right_in1_ = -1;
    cached_level_right_in2_ = -1;
}

void MotorController::SetMotorPin(gpio_num_t pin, int level) {
    int* cache = LevelCacheFor(pin);
    if (cache != nullptr && *cache == level) {
        return;
    }
    gpio_set_level(pin, level);
    if (cache != nullptr) {
        *cache = level;
    }
}

void MotorController::InitMotorGpio() {
    if (gpio_initialized_) {
        return;
    }
    const gpio_num_t pins[] = {left_in1_, left_in2_, right_in1_, right_in2_};
    for (gpio_num_t pin : pins) {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, 0);
    }
    ResetLevelCache();
    cached_level_left_in1_ = 0;
    cached_level_left_in2_ = 0;
    cached_level_right_in1_ = 0;
    cached_level_right_in2_ = 0;
    gpio_initialized_ = true;
    ESP_LOGI(MOTOR_TAG, "Motor GPIO outputs ready on %d/%d/%d/%d (no LEDC)", left_in1_, left_in2_,
             right_in1_, right_in2_);
}

void MotorController::CoastSide(gpio_num_t in1, gpio_num_t in2) {
    SetMotorPin(in1, 0);
    SetMotorPin(in2, 0);
}

void MotorController::BrakeSide(gpio_num_t in1, gpio_num_t in2) {
    SetMotorPin(in1, 1);
    SetMotorPin(in2, 1);
}

void MotorController::DriveSide(gpio_num_t in1, gpio_num_t in2, int speed) {
    speed = std::clamp(speed, -100, 100);
    if (speed == 0) {
        CoastSide(in1, in2);
        return;
    }
    if (speed > 0) {
        SetMotorPin(in1, 1);
        SetMotorPin(in2, 0);
    } else {
        SetMotorPin(in1, 0);
        SetMotorPin(in2, 1);
    }
}

void MotorController::ScheduleAutoStop(int duration_ms) {
    if (stop_timer_ == nullptr || duration_ms <= 0) {
        return;
    }
    esp_timer_stop(stop_timer_);
    esp_timer_start_once(stop_timer_, static_cast<uint64_t>(duration_ms) * 1000);
}

void MotorController::DriveForMs(int left_speed, int right_speed, int duration_ms) {
    left_speed = std::clamp(left_speed, -100, 100);
    right_speed = std::clamp(right_speed, -100, 100);
    duration_ms = std::clamp(duration_ms, 0, 10000);
    ExecuteMove(left_speed, right_speed, 0);
    if (duration_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
    }
    ExecuteStop();
}

bool MotorController::DriveForMsWithCancel(int left_speed, int right_speed, int duration_ms,
                                           std::atomic<bool>& cancel) {
    left_speed = std::clamp(left_speed, -100, 100);
    right_speed = std::clamp(right_speed, -100, 100);
    duration_ms = std::clamp(duration_ms, 0, 10000);
    if (cancel.load(std::memory_order_acquire)) {
        return false;
    }
    ExecuteMove(left_speed, right_speed, 0);
    constexpr int kPollMs = 50;
    int elapsed = 0;
    while (elapsed < duration_ms) {
        if (cancel.load(std::memory_order_acquire)) {
            ExecuteStop();
            return false;
        }
        const int slice = std::min(kPollMs, duration_ms - elapsed);
        vTaskDelay(pdMS_TO_TICKS(slice));
        elapsed += slice;
    }
    ExecuteStop();
    return !cancel.load(std::memory_order_acquire);
}

namespace {

#if CONFIG_BOARD_TYPE_BLUE_V2
void StartDanceMusic(uint8_t track) {
    std::string_view sound = Lang::Sounds::OGG_DANCE1;
    if (track == 3) {
        sound = Lang::Sounds::OGG_DANCE3;
    } else if (track == 2) {
        sound = Lang::Sounds::OGG_DANCE2;
    }
    Application::GetInstance().Schedule([sound]() {
        Application::GetInstance().PlaySound(sound);
    });
}
#endif

}  // namespace

void MotorController::RunDanceRoutine(uint8_t track) {
    uint8_t normalized = 1;
    if (track == 3) {
        normalized = 3;
    } else if (track == 2) {
        normalized = 2;
    }
    dance_cancel_.store(false, std::memory_order_release);
    dancing_.store(true, std::memory_order_release);

#if CONFIG_BOARD_TYPE_BLUE_V2
    StartDanceMusic(normalized);
    ESP_LOGI(MOTOR_TAG, "Dance track %u start", static_cast<unsigned>(normalized));
    if (normalized == 3) {
        MotorDance::RunTrack3(*this, dance_cancel_);
    } else if (normalized == 2) {
        MotorDance::RunTrack2(*this, dance_cancel_);
    } else {
        MotorDance::RunTrack1(*this, dance_cancel_);
    }
#else
    ESP_LOGW(MOTOR_TAG, "Dance track %u skipped — no choreography on this board",
             static_cast<unsigned>(normalized));
#endif

    ExecuteStop();
    const bool cancelled = dance_cancel_.load(std::memory_order_acquire);
    dancing_.store(false, std::memory_order_release);
    dance_cancel_.store(false, std::memory_order_release);
    if (cancelled) {
        ESP_LOGW(MOTOR_TAG, "Dance track %u cancelled", static_cast<unsigned>(normalized));
    } else {
        ESP_LOGI(MOTOR_TAG, "Dance track %u done", static_cast<unsigned>(normalized));
    }
}

void MotorController::ExecuteMove(int left_speed, int right_speed, int duration_ms) {
    InitMotorGpio();
    left_speed_.store(left_speed, std::memory_order_release);
    right_speed_.store(right_speed, std::memory_order_release);
    moving_.store(left_speed != 0 || right_speed != 0, std::memory_order_release);
    DriveSide(left_in1_, left_in2_, left_speed);
    DriveSide(right_in1_, right_in2_, right_speed);
    ESP_LOGI(MOTOR_TAG, "Move left=%d right=%d duration=%dms", left_speed, right_speed, duration_ms);
    if (left_speed != 0 || right_speed != 0) {
        TofController::Instance().RequestFastSample();
    }
    if (left_speed == 0 && right_speed == 0) {
        esp_timer_stop(stop_timer_);
    } else {
        ScheduleAutoStop(duration_ms);
    }
}

void MotorController::NotifyStoppedOnMain() {
#if CONFIG_BOARD_TYPE_BLUE_V2
    Application::GetInstance().ScheduleListeningResyncAfterMotorStop();
#endif
}

void MotorController::ExecuteStop() {
    esp_timer_stop(stop_timer_);
    const bool was_moving = moving_.exchange(false, std::memory_order_acq_rel);
    left_speed_.store(0, std::memory_order_release);
    right_speed_.store(0, std::memory_order_release);
    if (gpio_initialized_) {
#if MOTOR_BRAKE_ENABLE
        if (was_moving) {
            BrakeSide(left_in1_, left_in2_);
            BrakeSide(right_in1_, right_in2_);
            vTaskDelay(pdMS_TO_TICKS(MOTOR_BRAKE_MS));
            ESP_LOGI(MOTOR_TAG, "Motors brake %dms then coast", MOTOR_BRAKE_MS);
        }
#endif
        CoastSide(left_in1_, left_in2_);
        CoastSide(right_in1_, right_in2_);
    }
    if (!was_moving) {
        ESP_LOGI(MOTOR_TAG, "Motors stopped");
        return;
    }
    NotifyStoppedOnMain();
}

void MotorController::RegisterMcpTools() {
    auto& mcp_server = McpServer::GetInstance();

    mcp_server.AddTool("self.motor.stop", "Stop all motors. Use for: dừng, dừng lại, stop.", PropertyList(),
                       [this](const PropertyList&) -> ReturnValue { return EnqueueStop(); });

    mcp_server.AddTool("self.motor.forward",
                       "Move robot forward. Use for: đi tới, tiến, đi thẳng, go forward.",
                       PropertyList(),
                       [this](const PropertyList&) -> ReturnValue {
                           return EnqueueMove(100, 100, MOTOR_AUTO_STOP_MS);
                       });

    mcp_server.AddTool("self.motor.backward",
                       "Move robot backward. Use for: lùi, đi lùi, go back.",
                       PropertyList(),
                       [this](const PropertyList&) -> ReturnValue {
                           return EnqueueMove(-100, -100, MOTOR_AUTO_STOP_MS);
                       });

    mcp_server.AddTool("self.motor.turn_left",
                       "Turn robot left in place. Use for: quay sang trái, rẽ trái, turn left.",
                       PropertyList(),
                       [this](const PropertyList&) -> ReturnValue {
                           return EnqueueMove(-100, 100, MOTOR_AUTO_STOP_MS);
                       });

    mcp_server.AddTool("self.motor.turn_right",
                       "Turn robot right in place. Use for: quay sang phải, rẽ phải, turn right.",
                       PropertyList(),
                       [this](const PropertyList&) -> ReturnValue {
                           return EnqueueMove(100, -100, MOTOR_AUTO_STOP_MS);
                       });

    mcp_server.AddTool(
        "self.motor.move",
        "Drive straight or turn. left/right sign = direction (-100..100), full speed when non-zero. "
        "For circle use self.motor.circle. duration_ms auto-stop (100-10000).",
        PropertyList({Property("left", kPropertyTypeInteger, 0, -100, 100),
                      Property("right", kPropertyTypeInteger, 0, -100, 100),
                      Property("duration_ms", kPropertyTypeInteger, MOTOR_AUTO_STOP_MS, 100, 10000)}),
        [this](const PropertyList& properties) -> ReturnValue {
            const int left = properties["left"].value<int>();
            const int right = properties["right"].value<int>();
            const int duration_ms = properties["duration_ms"].value<int>();
            return EnqueueMove(left, right, duration_ms);
        });

    mcp_server.AddTool(
        "self.motor.circle",
        "Drive in a circle (left wheel reverse). duration_ms auto-stop (1000-30000).",
        PropertyList({Property("duration_ms", kPropertyTypeInteger, MOTOR_AUTO_STOP_MS, 1000, 30000)}),
        [this](const PropertyList& properties) -> ReturnValue {
            const int duration_ms = properties["duration_ms"].value<int>();
            return EnqueueMove(-100, 100, duration_ms);
        });

    mcp_server.AddTool(
        "self.motor.dance",
        "Dance with embedded music. track=1 slap-house ~23 s; track=2 hip-hop ~99 s; track=3 drill ~25 s. "
        "Use for: nhảy, múa, dance, dance 1/2/3.",
        PropertyList({Property("track", kPropertyTypeInteger, 1, 1, 3)}),
        [this](const PropertyList& properties) -> ReturnValue {
            const int track = properties["track"].value<int>();
            return EnqueueDance(track);
        });

    mcp_server.AddTool("self.chassis.go_forward", "Move forward", PropertyList(),
                       [this](const PropertyList&) -> ReturnValue {
                           return EnqueueMove(100, 100, MOTOR_AUTO_STOP_MS);
                       });

    mcp_server.AddTool("self.chassis.go_back", "Move backward", PropertyList(),
                       [this](const PropertyList&) -> ReturnValue {
                           return EnqueueMove(-100, -100, MOTOR_AUTO_STOP_MS);
                       });

    mcp_server.AddTool("self.chassis.turn_left", "Turn left", PropertyList(),
                       [this](const PropertyList&) -> ReturnValue {
                           return EnqueueMove(-100, 100, MOTOR_AUTO_STOP_MS);
                       });

    mcp_server.AddTool("self.chassis.turn_right", "Turn right", PropertyList(),
                       [this](const PropertyList&) -> ReturnValue {
                           return EnqueueMove(100, -100, MOTOR_AUTO_STOP_MS);
                       });

    mcp_server.AddTool("self.chassis.dance", "Dance routine track 1 (wiggle + spin)", PropertyList(),
                       [this](const PropertyList&) -> ReturnValue { return EnqueueDance(1); });
}
