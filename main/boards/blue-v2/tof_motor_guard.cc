#include "tof_motor_guard.h"

#include "config.h"
#include "tof_controller.h"

#include <cstdlib>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" {
#include <vl53l0x.h>
}

#define TAG "TofGuard"

namespace {

enum class StopReason {
    None,
    Obstacle,
    CliffFloor,
    CliffFar,
    CliffJump,
    CliffLostSignal,
};

const char* StopReasonName(StopReason reason) {
    switch (reason) {
        case StopReason::Obstacle:
            return "obstacle";
        case StopReason::CliffFloor:
            return "cliff_floor";
        case StopReason::CliffFar:
            return "cliff_far";
        case StopReason::CliffJump:
            return "cliff_jump";
        case StopReason::CliffLostSignal:
            return "cliff_lost_signal";
        default:
            return "none";
    }
}

bool SampleUsable(const vl53l0x_data_t& sample) {
    return sample.valid && sample.distance_mm > 0 && sample.distance_mm <= TOF_MAX_VALID_MM;
}

int AbsDelta(int a, int b) {
    return a >= b ? a - b : b - a;
}

int NearLimit(int cal_mm) {
    const int pct = cal_mm * (100 - TOF_CAL_NEAR_MARGIN_PCT) / 100;
    const int abs = cal_mm - TOF_CAL_NEAR_MARGIN_MM;
    return pct > abs ? pct : abs;
}

int FarLimit(int cal_mm) {
    const int pct = cal_mm * (100 + TOF_CAL_FAR_MARGIN_PCT) / 100;
    const int abs = cal_mm + TOF_CAL_FAR_MARGIN_MM;
    return pct < abs ? pct : abs;
}

void LogFrontSample(const char* prefix, const vl53l0x_data_t& sample, int cal_mm, bool use_cal) {
    if (use_cal && cal_mm > 0) {
        const int near_limit = NearLimit(cal_mm);
        const int far_limit = FarLimit(cal_mm);
        ESP_LOGI(TAG,
                 "%s front dist=%u mm valid=%d status=%u (%s) signal=%.2f cal_ref=%d near<%d far>%d",
                 prefix, sample.distance_mm, sample.valid, sample.range_status,
                 vl53l0x_range_status_str(sample.range_status), sample.signal_rate_mcps, cal_mm,
                 near_limit, far_limit);
    } else {
        ESP_LOGI(TAG,
                 "%s front dist=%u mm valid=%d status=%u (%s) signal=%.2f "
                 "fallback stop<=%d void>=%d cal=0",
                 prefix, sample.distance_mm, sample.valid, sample.range_status,
                 vl53l0x_range_status_str(sample.range_status), sample.signal_rate_mcps,
                 TOF_OBSTACLE_STOP_MM, TOF_CLIFF_VOID_MM);
    }
}

void LogRearSample(const char* prefix, const vl53l0x_data_t& sample) {
    ESP_LOGI(TAG,
             "%s rear floor=%u mm valid=%d status=%u (%s) cliff>%d mm",
             prefix, sample.distance_mm, sample.valid, sample.range_status,
             vl53l0x_range_status_str(sample.range_status), TOF_CLIFF_FLOOR_MAX_MM);
}

StopReason CheckFrontCalibrated(const vl53l0x_data_t& sample, int cal_mm, uint16_t prev_dist,
                                bool prev_valid) {
#if !TOF_OBSTACLE_GUARD_ENABLE && !TOF_CLIFF_GUARD_ENABLE
    (void)sample;
    (void)cal_mm;
    (void)prev_dist;
    (void)prev_valid;
    return StopReason::None;
#endif

    const int near_limit = NearLimit(cal_mm);
    const int far_limit = FarLimit(cal_mm);

    if (SampleUsable(sample)) {
        const int dist = static_cast<int>(sample.distance_mm);
#if TOF_OBSTACLE_GUARD_ENABLE
        if (dist < near_limit) {
            return StopReason::Obstacle;
        }
        if (prev_valid && static_cast<int>(prev_dist) >= near_limit &&
            dist <= static_cast<int>(prev_dist) - TOF_CAL_APPROACH_STEP_MM) {
            return StopReason::Obstacle;
        }
#endif
#if TOF_CLIFF_GUARD_ENABLE
        if (dist > far_limit) {
            return StopReason::CliffFar;
        }
        if (prev_valid) {
            const int prev_dev = AbsDelta(static_cast<int>(prev_dist), cal_mm);
            const int cur_dev = AbsDelta(dist, cal_mm);
            if (cur_dev >= prev_dev + TOF_CAL_JUMP_MARGIN_MM) {
                return StopReason::CliffJump;
            }
        }
#endif
        return StopReason::None;
    }

#if TOF_CLIFF_GUARD_ENABLE
    if (prev_valid && AbsDelta(static_cast<int>(prev_dist), cal_mm) <= FarLimit(cal_mm)) {
        return StopReason::CliffLostSignal;
    }
    if (sample.distance_mm >= TOF_MAX_VALID_MM || sample.distance_mm >= 8000) {
        return StopReason::CliffFar;
    }
#else
    (void)prev_dist;
    (void)prev_valid;
#endif
    return StopReason::None;
}

StopReason CheckFrontFallback(const vl53l0x_data_t& sample) {
#if !TOF_OBSTACLE_GUARD_ENABLE && !TOF_CLIFF_GUARD_ENABLE
    (void)sample;
    return StopReason::None;
#endif

    if (SampleUsable(sample)) {
#if TOF_OBSTACLE_GUARD_ENABLE
        if (sample.distance_mm <= TOF_OBSTACLE_STOP_MM) {
            return StopReason::Obstacle;
        }
#endif
#if TOF_CLIFF_GUARD_ENABLE
        if (sample.distance_mm >= TOF_CLIFF_VOID_MM) {
            return StopReason::CliffFar;
        }
#endif
        return StopReason::None;
    }

#if TOF_CLIFF_GUARD_ENABLE
    if (sample.distance_mm >= TOF_MAX_VALID_MM || sample.distance_mm >= 8000) {
        return StopReason::CliffFar;
    }
#endif
    return StopReason::None;
}

StopReason CheckRearWhileMoving(const vl53l0x_data_t& sample) {
#if !TOF_CLIFF_GUARD_ENABLE
    (void)sample;
    return StopReason::None;
#else
    if (!SampleUsable(sample)) {
        return StopReason::CliffFloor;
    }
    if (sample.distance_mm > TOF_CLIFF_FLOOR_MAX_MM) {
        return StopReason::CliffFloor;
    }
    return StopReason::None;
#endif
}

}  // namespace

TofMotorGuard::TofMotorGuard(MotorController* motor) : motor_(motor) {}

bool TofMotorGuard::Start() {
#if !TOF_OBSTACLE_GUARD_ENABLE && !TOF_CLIFF_GUARD_ENABLE
    ESP_LOGI(TAG, "ToF safety guard disabled in config");
    return false;
#endif
    auto& tof = TofController::Instance();
    if (!tof.IsReady()) {
        ESP_LOGW(TAG, "ToF not ready — safety guard skipped");
        return false;
    }

    active_ = true;
    if (tof.IsCalibrated()) {
        ESP_LOGI(TAG,
                 "Safety guard: cal_ref=%d mm near<%d far>%d approach=%d rear=%d",
                 tof.CalibratedDistanceMm(), NearLimit(tof.CalibratedDistanceMm()),
                 FarLimit(tof.CalibratedDistanceMm()), TOF_CAL_APPROACH_STEP_MM,
                 tof.HasRearSensor());
    } else {
        ESP_LOGW(TAG,
                 "Safety guard: NOT calibrated — fallback obstacle<=%d void>=%d (run calibrate "
                 "on open floor first)",
                 TOF_OBSTACLE_STOP_MM, TOF_CLIFF_VOID_MM);
    }

    if (xTaskCreate(TaskEntry, "tof_guard", 4096, this, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start guard task");
        active_ = false;
        return false;
    }
    return true;
}

void TofMotorGuard::TaskEntry(void* arg) {
    static_cast<TofMotorGuard*>(arg)->PollOnce();
}

void TofMotorGuard::PollOnce() {
    auto& tof = TofController::Instance();
    bool was_moving_forward = false;
    int64_t last_debug_log_ms = 0;
    int64_t last_signal_fail_warn_ms = 0;
    uint16_t prev_front_dist = 0;
    bool prev_front_valid = false;

    while (true) {
        const bool moving_forward = active_ && motor_ != nullptr && motor_->IsMovingForward();
        const bool use_cal = tof.IsCalibrated();
        const int cal_mm = use_cal ? tof.CalibratedDistanceMm() : 0;

        if (moving_forward != was_moving_forward) {
            ESP_LOGI(TAG, "Motor forward=%d (left=%d right=%d) cal_ref=%d", moving_forward,
                     motor_ ? motor_->LeftSpeed() : 0, motor_ ? motor_->RightSpeed() : 0, cal_mm);
            was_moving_forward = moving_forward;
            last_debug_log_ms = 0;
            prev_front_dist = 0;
            prev_front_valid = false;
        }

        const int64_t now_ms = esp_timer_get_time() / 1000;
        const int debug_interval_ms =
            moving_forward ? TOF_DEBUG_MOVE_LOG_MS : TOF_DEBUG_IDLE_LOG_MS;
        const bool should_measure =
            moving_forward || (TOF_DEBUG_LOG && (now_ms - last_debug_log_ms >= debug_interval_ms));

        if (active_ && should_measure) {
            vl53l0x_data_t front = {};
            const bool front_ok = tof.MeasureFront(&front);

            vl53l0x_data_t rear = {};
            bool rear_ok = false;
            if (tof.HasRearSensor()) {
                rear_ok = tof.MeasureRear(&rear);
            }

            if (!front_ok) {
                ESP_LOGW(TAG, "Front measure failed (forward=%d)", moving_forward);
            } else if (TOF_DEBUG_LOG && (now_ms - last_debug_log_ms >= debug_interval_ms)) {
                if (front.valid || (now_ms - last_signal_fail_warn_ms >= 5000)) {
                    LogFrontSample(moving_forward ? "Move" : "Idle", front, cal_mm, use_cal);
                    if (rear_ok && tof.HasRearSensor()) {
                        LogRearSample(moving_forward ? "Move" : "Idle", rear);
                    }
                    last_debug_log_ms = now_ms;
                    if (!front.valid) {
                        last_signal_fail_warn_ms = now_ms;
                    }
                }
            }

            if (moving_forward) {
                StopReason reason = StopReason::None;
                if (front_ok) {
                    if (use_cal && cal_mm > 0) {
                        reason = CheckFrontCalibrated(front, cal_mm, prev_front_dist, prev_front_valid);
                    } else {
                        reason = CheckFrontFallback(front);
                    }
                    if (SampleUsable(front)) {
                        prev_front_dist = front.distance_mm;
                        prev_front_valid = true;
                    } else if (reason == StopReason::None) {
                        prev_front_valid = false;
                    }
                }
                if (reason == StopReason::None && tof.HasRearSensor() && rear_ok) {
                    reason = CheckRearWhileMoving(rear);
                }
                if (reason != StopReason::None) {
                    ESP_LOGW(TAG, ">>> STOP %s front=%u mm cal_ref=%d valid=%d rear=%u mm",
                             StopReasonName(reason), front.distance_mm, cal_mm, front.valid,
                             rear_ok ? rear.distance_mm : 0U);
                    motor_->Stop();
                    was_moving_forward = false;
                    prev_front_dist = 0;
                    prev_front_valid = false;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TOF_GUARD_POLL_MS));
    }
}
