#include "tof_motor_guard.h"

#include "config.h"
#include "tof_controller.h"

#include <cstdlib>
#include <climits>
#include <cstdint>
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

bool NeedsFrontCliffCheck(int left, int right) {
    return !(left < 0 && right < 0);
}

StopReason CheckFrontCliffInvalid(const vl53l0x_data_t& sample, int far_limit_mm, int near_limit_mm) {
#if !TOF_CLIFF_GUARD_ENABLE
    (void)sample;
    (void)far_limit_mm;
    (void)near_limit_mm;
    return StopReason::None;
#else
    if (sample.distance_mm > 0 && sample.distance_mm < static_cast<uint16_t>(near_limit_mm)) {
        return StopReason::None;
    }
    if (sample.valid && sample.distance_mm >= TOF_MAX_VALID_MM) {
        return StopReason::CliffFar;
    }
    if (!sample.valid || sample.distance_mm == 0) {
        return StopReason::CliffLostSignal;
    }
    if (static_cast<int>(sample.distance_mm) > far_limit_mm) {
        return StopReason::CliffFar;
    }
    return StopReason::None;
#endif
}

StopReason CheckFrontCalibrated(const vl53l0x_data_t& sample, int cal_mm, uint16_t prev_dist,
                                bool prev_valid, uint16_t move_min_dist) {
#if !TOF_OBSTACLE_GUARD_ENABLE && !TOF_CLIFF_GUARD_ENABLE
    (void)sample;
    (void)cal_mm;
    (void)prev_dist;
    (void)prev_valid;
    (void)move_min_dist;
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
        if (move_min_dist != UINT16_MAX &&
            dist >= static_cast<int>(move_min_dist) + TOF_CLIFF_JUMP_MM) {
            return StopReason::CliffJump;
        }
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

#if TOF_OBSTACLE_GUARD_ENABLE
    // Very close but invalid (min range / signal fail) — treat as obstacle while moving.
    if (sample.distance_mm > 0 && sample.distance_mm < near_limit) {
        return StopReason::Obstacle;
    }
#endif

#if TOF_CLIFF_GUARD_ENABLE
    if (prev_valid && AbsDelta(static_cast<int>(prev_dist), cal_mm) <= FarLimit(cal_mm)) {
        return StopReason::CliffLostSignal;
    }
    const StopReason invalid_cliff =
        CheckFrontCliffInvalid(sample, far_limit, near_limit);
    if (invalid_cliff != StopReason::None) {
        return invalid_cliff;
    }
#else
    (void)prev_dist;
    (void)prev_valid;
#endif
    return StopReason::None;
}

StopReason CheckFrontFallback(const vl53l0x_data_t& sample, uint16_t prev_dist, bool prev_valid,
                              uint16_t move_min_dist) {
#if !TOF_OBSTACLE_GUARD_ENABLE && !TOF_CLIFF_GUARD_ENABLE
    (void)sample;
    (void)prev_dist;
    (void)prev_valid;
    (void)move_min_dist;
    return StopReason::None;
#endif

    if (SampleUsable(sample)) {
#if TOF_OBSTACLE_GUARD_ENABLE
        if (sample.distance_mm <= TOF_OBSTACLE_STOP_MM) {
            return StopReason::Obstacle;
        }
#endif
#if TOF_CLIFF_GUARD_ENABLE
        if (move_min_dist != UINT16_MAX &&
            sample.distance_mm >= move_min_dist + TOF_CLIFF_JUMP_MM) {
            return StopReason::CliffJump;
        }
        if (sample.distance_mm >= TOF_CLIFF_VOID_MM) {
            return StopReason::CliffFar;
        }
#endif
        return StopReason::None;
    }

#if TOF_CLIFF_GUARD_ENABLE
    if (prev_valid && prev_dist <= TOF_CLIFF_NEAR_MAX_MM &&
        prev_dist > static_cast<uint16_t>(TOF_OBSTACLE_STOP_MM)) {
        return StopReason::CliffLostSignal;
    }
    const StopReason invalid_cliff = CheckFrontCliffInvalid(
        sample, TOF_CLIFF_VOID_MM, TOF_OBSTACLE_STOP_MM);
    if (invalid_cliff != StopReason::None) {
        return invalid_cliff;
    }
#else
    (void)prev_dist;
    (void)prev_valid;
    (void)move_min_dist;
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
    bool was_moving = false;
    int64_t motor_move_started_ms = 0;
    int64_t last_debug_log_ms = 0;
    int64_t last_signal_fail_warn_ms = 0;
    uint16_t prev_front_dist = 0;
    bool prev_front_valid = false;
    uint16_t move_min_front_dist = UINT16_MAX;
    int move_measure_fail_streak = 0;

    while (true) {
        const bool motor_moving = active_ && motor_ != nullptr && motor_->IsMoving();
        const int left_speed = motor_ ? motor_->LeftSpeed() : 0;
        const int right_speed = motor_ ? motor_->RightSpeed() : 0;
        const bool check_front = NeedsFrontCliffCheck(left_speed, right_speed);
        const bool use_cal = tof.IsCalibrated();
        const int cal_mm = use_cal ? tof.CalibratedDistanceMm() : 0;

        const int64_t now_ms = esp_timer_get_time() / 1000;

        if (motor_moving != was_moving) {
            ESP_LOGI(TAG, "Motor moving=%d (left=%d right=%d) cal_ref=%d", motor_moving,
                     motor_ ? motor_->LeftSpeed() : 0, motor_ ? motor_->RightSpeed() : 0, cal_mm);
            if (motor_moving) {
                motor_move_started_ms = now_ms;
                move_measure_fail_streak = 0;
                prev_front_dist = 0;
                prev_front_valid = false;
                move_min_front_dist = UINT16_MAX;
                TofSnapshot seed{};
                if (tof.GetLatestSnapshot(&seed) && seed.front_ok && SampleUsable(seed.front)) {
                    prev_front_dist = seed.front.distance_mm;
                    prev_front_valid = true;
                    move_min_front_dist = seed.front.distance_mm;
                }
            } else {
                prev_front_dist = 0;
                prev_front_valid = false;
                move_min_front_dist = UINT16_MAX;
            }
            was_moving = motor_moving;
            last_debug_log_ms = 0;
        }

        const int debug_interval_ms = TOF_DEBUG_MOVE_LOG_MS;
        const bool should_check = motor_moving;

        if (active_ && should_check) {
            TofSnapshot snap{};
            const bool have_snap = tof.GetLatestSnapshot(&snap);
            const int64_t snap_age_ms =
                have_snap ? (now_ms - snap.timestamp_ms) : INT64_MAX;
            const int64_t stale_limit_ms = static_cast<int64_t>(TOF_GUARD_POLL_MS * 4);
            const bool in_move_grace =
                motor_moving && motor_move_started_ms > 0 &&
                (now_ms - motor_move_started_ms) < TOF_MOVE_GRACE_MS;

            const bool snap_fresh = have_snap && snap_age_ms <= stale_limit_ms;
            const bool front_ok = snap_fresh && snap.front_ok;
            const bool rear_ok = snap_fresh && snap.rear_ok;
            const vl53l0x_data_t& front = snap.front;
            const vl53l0x_data_t& rear = snap.rear;

            if (!front_ok) {
                if (!in_move_grace) {
                    move_measure_fail_streak++;
                }
                if (!in_move_grace && now_ms - last_debug_log_ms >= debug_interval_ms) {
                    ESP_LOGW(TAG, "Front snapshot stale/missing (moving=1 age=%lld streak=%d)",
                             static_cast<long long>(snap_age_ms), move_measure_fail_streak);
                    last_debug_log_ms = now_ms;
                }
            } else {
                if (SampleUsable(front)) {
                    move_measure_fail_streak = 0;
                }
                if (TOF_DEBUG_LOG && (now_ms - last_debug_log_ms >= debug_interval_ms)) {
                    if (front.valid || (now_ms - last_signal_fail_warn_ms >= 5000)) {
                        LogFrontSample("Move", front, cal_mm, use_cal);
                        if (rear_ok && tof.HasRearSensor()) {
                            LogRearSample("Move", rear);
                        }
                        last_debug_log_ms = now_ms;
                        if (!front.valid) {
                            last_signal_fail_warn_ms = now_ms;
                        }
                    }
                }
            }

            if (front_ok && SampleUsable(front) && front.distance_mm < move_min_front_dist) {
                move_min_front_dist = front.distance_mm;
            }

            const bool check_cliff = check_front;
            const bool check_obstacle = check_front;

            if (!in_move_grace && check_obstacle) {
                StopReason reason = StopReason::None;
                if (front_ok) {
                    if (check_cliff) {
                        if (use_cal && cal_mm > 0) {
                            reason = CheckFrontCalibrated(front, cal_mm, prev_front_dist,
                                                           prev_front_valid, move_min_front_dist);
                        } else {
                            reason = CheckFrontFallback(front, prev_front_dist, prev_front_valid,
                                                        move_min_front_dist);
                        }
                    }
                    if (SampleUsable(front)) {
                        prev_front_dist = front.distance_mm;
                        prev_front_valid = true;
                    } else if (reason == StopReason::None && check_cliff) {
                        move_measure_fail_streak++;
                    }
                } else if (check_cliff && move_measure_fail_streak >= 4) {
                    reason = StopReason::CliffLostSignal;
                } else if (move_measure_fail_streak >= 8) {
                    reason = StopReason::Obstacle;
                } else {
                    move_measure_fail_streak++;
                }
                if (reason == StopReason::None && tof.HasRearSensor() && rear_ok) {
                    reason = CheckRearWhileMoving(rear);
                }
                if (reason != StopReason::None) {
                    ESP_LOGW(TAG, ">>> STOP %s front=%u mm cal_ref=%d valid=%d rear=%u mm age=%lld",
                             StopReasonName(reason), front.distance_mm, cal_mm, front.valid,
                             rear_ok ? rear.distance_mm : 0U, static_cast<long long>(snap_age_ms));
                    motor_->EnqueueStop();
                    was_moving = false;
                    motor_move_started_ms = 0;
                    prev_front_dist = 0;
                    prev_front_valid = false;
                    move_min_front_dist = UINT16_MAX;
                    move_measure_fail_streak = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(TOF_GUARD_POLL_MS));
    }
}
