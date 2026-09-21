#include "blue_v4_gait_guard.h"

#include "blue_v4_tof_controller.h"
#include "config.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "BlueV4GaitGuard"

namespace {
constexpr int kConsecutiveInvalidToStop = 3;  // sensor loss only counts if sustained
constexpr UBaseType_t kTaskPriority = 3;
constexpr int kTaskStackWords = 3072;

// Obstacle: dist < cal * (100 - pct)/100  OR  dist < cal - abs   (stricter wins).
int NearLimit(int cal_mm) {
    const int pct = cal_mm * (100 - TOF_CAL_NEAR_MARGIN_PCT) / 100;
    const int abs = cal_mm - TOF_CAL_NEAR_MARGIN_MM;
    return pct > abs ? pct : abs;
}

// Cliff: dist > cal * (100 + pct)/100  OR  dist > cal + abs   (stricter wins).
int FarLimit(int cal_mm) {
    const int pct = cal_mm * (100 + TOF_CAL_FAR_MARGIN_PCT) / 100;
    const int abs = cal_mm + TOF_CAL_FAR_MARGIN_MM;
    return pct < abs ? pct : abs;
}
}  // namespace

BlueV4GaitGuard::BlueV4GaitGuard(GaitEngine* gait) : gait_(gait) {}

bool BlueV4GaitGuard::Start() {
#if !TOF_OBSTACLE_GUARD_ENABLE && !TOF_CLIFF_GUARD_ENABLE
    ESP_LOGW(TAG, "guard DISABLED by config (TOF_OBSTACLE_GUARD_ENABLE / TOF_CLIFF_GUARD_ENABLE = 0)");
    ESP_LOGW(TAG, "bench mode: the gait will NOT stop on obstacles or drop-offs");
    active_ = false;
    return true;
#else
    if (gait_ == nullptr) {
        ESP_LOGW(TAG, "no gait engine — guard disabled");
        return false;
    }
    if (!BlueV4TofController::Instance().IsReady()) {
        ESP_LOGW(TAG, "ToF not ready — guard disabled");
        return false;
    }
    active_ = true;
    if (xTaskCreate(TaskEntry, "blue_v4_guard", kTaskStackWords, this, kTaskPriority, nullptr) !=
        pdPASS) {
        active_ = false;
        ESP_LOGE(TAG, "task creation failed");
        return false;
    }
    ESP_LOGI(TAG, "gait guard active (poll %d ms, grace %d ms, cal_ref=%d mm)", TOF_GUARD_POLL_MS,
             TOF_MOVE_GRACE_MS, BlueV4TofController::Instance().CalibratedDistanceMm());
    return true;
#endif
}

void BlueV4GaitGuard::TaskEntry(void* arg) { static_cast<BlueV4GaitGuard*>(arg)->GuardLoop(); }

void BlueV4GaitGuard::TriggerStop(const char* reason, int dist_mm, int limit_mm) {
    ESP_LOGW(TAG, "STOP (%s): dist=%d mm limit=%d mm — stopping the gait", reason, dist_mm,
             limit_mm);
    gait_->EnqueueStop();
    have_last_ = false;
    invalid_readings_ = 0;
}

void BlueV4GaitGuard::GuardLoop() {
    const TickType_t period = pdMS_TO_TICKS(TOF_GUARD_POLL_MS);
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        const int64_t now_ms = esp_timer_get_time() / 1000;
        const bool busy = gait_->IsBusy();

        if (busy && !was_busy_) {
            motion_started_ms_ = now_ms;
            have_last_ = false;
            invalid_readings_ = 0;
            ESP_LOGI(TAG, "gait started — guard watching");
        } else if (!busy) {
            was_busy_ = false;
            have_last_ = false;
            invalid_readings_ = 0;
            vTaskDelayUntil(&last_wake, period);
            continue;
        }
        was_busy_ = true;

        // Keep the sampler fast while the legs move; the guard itself never touches I2C.
        BlueV4TofController::Instance().RequestFastSample();

        const bool in_grace = (now_ms - motion_started_ms_) < TOF_MOVE_GRACE_MS;
        BlueV4TofSnapshot snap{};
        if (!BlueV4TofController::Instance().GetLatestSnapshot(&snap)) {
            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        const int cal_mm = BlueV4TofController::Instance().CalibratedDistanceMm();
        const bool use_cal = BlueV4TofController::Instance().IsCalibrated() && cal_mm > 0;
        const int near_limit = use_cal ? NearLimit(cal_mm) : TOF_OBSTACLE_STOP_MM;
        const int far_limit = use_cal ? FarLimit(cal_mm) : TOF_CLIFF_VOID_MM;
        const int dist = static_cast<int>(snap.front.distance_mm);
        const bool usable = snap.front_ok && dist > 0;

#if TOF_DEBUG_LOG
        const bool log_due = (dist != last_log_dist_mm_) || (now_ms - last_log_ms_ >= 500);
        if (log_due) {
            last_log_dist_mm_ = dist;
            last_log_ms_ = now_ms;
            ESP_LOGI(TAG, "front dist=%d mm valid=%d status=%u signal=%.2f %s near<%d far>%d%s",
                     dist, snap.front.valid, snap.front.range_status, snap.front.signal_rate_mcps,
                     use_cal ? "cal" : "fallback", near_limit, far_limit,
                     in_grace ? " (grace)" : "");
        }
#endif

        if (usable) {
            invalid_readings_ = 0;
#if TOF_OBSTACLE_GUARD_ENABLE
            // Fast approach: closing this much in one poll is an obstacle even before the limit.
            if (!in_grace && have_last_ && (last_dist_mm_ - dist) >= TOF_CAL_APPROACH_STEP_MM) {
                TriggerStop("approach", dist, last_dist_mm_ - TOF_CAL_APPROACH_STEP_MM);
                vTaskDelayUntil(&last_wake, period);
                continue;
            }
            if (!in_grace && dist <= near_limit) {
                TriggerStop("obstacle", dist, near_limit);
                vTaskDelayUntil(&last_wake, period);
                continue;
            }
#endif
#if TOF_CLIFF_GUARD_ENABLE
            if (dist >= far_limit) {
                TriggerStop("cliff_far", dist, far_limit);
                vTaskDelayUntil(&last_wake, period);
                continue;
            }
#endif
            last_dist_mm_ = dist;
            have_last_ = true;
        } else if (!in_grace && use_cal) {
#if TOF_CLIFF_GUARD_ENABLE
            // Lost signal / out of range is treated as a possible drop-off, but only if
            // it persists — single bad reads happen on reflective or dark surfaces.
            if (++invalid_readings_ >= kConsecutiveInvalidToStop) {
                TriggerStop("cliff_lost_signal", dist, far_limit);
                vTaskDelayUntil(&last_wake, period);
                continue;
            }
#endif
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
