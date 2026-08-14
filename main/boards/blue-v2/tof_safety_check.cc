#include "tof_safety_check.h"

#include "config.h"
#include "tof_controller.h"

#include <atomic>
#include <cstdlib>
#include <esp_log.h>

extern "C" {
#include <vl53l0x.h>
}

#define TAG "TofSafety"

namespace {

std::atomic<bool> g_cliff_edge_blocked{false};

bool IsCliffReason(TofSafetyStopReason reason) {
    return reason == TofSafetyStopReason::CliffFloor ||
           reason == TofSafetyStopReason::CliffFar ||
           reason == TofSafetyStopReason::CliffJump ||
           reason == TofSafetyStopReason::CliffLostSignal;
}

bool SampleUsable(const vl53l0x_data_t& sample) {
    return sample.valid && sample.distance_mm > 0 && sample.distance_mm <= TOF_MAX_VALID_MM;
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

/** Fallback mm when not calibrated; else derived from cal + TOF_CAL_FAR margins. */
int EdgePreMoveBufferMm(int cal_mm) {
    if (cal_mm <= 0) {
        return TOF_EDGE_PREMOVE_BUFFER_MM;
    }
    const int pct_buf = cal_mm * TOF_EDGE_PREMOVE_BUFFER_PCT / 100;
    const int half_far = TOF_CAL_FAR_MARGIN_MM / 2;
    return pct_buf > half_far ? pct_buf : half_far;
}

/** Block forward pre-move when dist >= this (cal-based or fallback void threshold). */
int EdgeProximityLimitMm(int cal_mm) {
    if (cal_mm <= 0) {
        return TOF_CLIFF_VOID_MM - TOF_EDGE_PREMOVE_BUFFER_MM;
    }
    return FarLimit(cal_mm) - EdgePreMoveBufferMm(cal_mm);
}

/** Latch clear: forward allowed again when dist <= this (back in safe zone near cal). */
int SafeForwardDistanceMm(int cal_mm) {
    if (cal_mm <= 0) {
        return TOF_CLIFF_VOID_MM - (2 * TOF_EDGE_PREMOVE_BUFFER_MM);
    }
    return cal_mm + EdgePreMoveBufferMm(cal_mm);
}

TofSafetyStopReason CheckFrontCliffInvalid(const vl53l0x_data_t& sample, int far_limit_mm,
                                           int near_limit_mm) {
#if !TOF_CLIFF_GUARD_ENABLE
    (void)sample;
    (void)far_limit_mm;
    (void)near_limit_mm;
    return TofSafetyStopReason::None;
#else
    if (sample.distance_mm > 0 && sample.distance_mm < static_cast<uint16_t>(near_limit_mm)) {
        return TofSafetyStopReason::None;
    }
    if (sample.valid && sample.distance_mm >= TOF_MAX_VALID_MM) {
        return TofSafetyStopReason::CliffFar;
    }
    if (!sample.valid || sample.distance_mm == 0) {
        return TofSafetyStopReason::CliffLostSignal;
    }
    if (static_cast<int>(sample.distance_mm) > far_limit_mm) {
        return TofSafetyStopReason::CliffFar;
    }
    return TofSafetyStopReason::None;
#endif
}

TofSafetyStopReason CheckFrontCalibrated(const vl53l0x_data_t& sample, int cal_mm,
                                         bool edge_proximity) {
#if !TOF_OBSTACLE_GUARD_ENABLE && !TOF_CLIFF_GUARD_ENABLE
    (void)sample;
    (void)cal_mm;
    (void)edge_proximity;
    return TofSafetyStopReason::None;
#endif

    const int near_limit = NearLimit(cal_mm);
    const int far_limit = FarLimit(cal_mm);
    const int edge_block_mm = EdgeProximityLimitMm(cal_mm);

    if (SampleUsable(sample)) {
        const int dist = static_cast<int>(sample.distance_mm);
#if TOF_OBSTACLE_GUARD_ENABLE
        if (dist < near_limit) {
            return TofSafetyStopReason::Obstacle;
        }
#endif
#if TOF_CLIFF_GUARD_ENABLE
        if (dist > far_limit) {
            return TofSafetyStopReason::CliffFar;
        }
        if (edge_proximity && dist >= edge_block_mm) {
            return TofSafetyStopReason::CliffFar;
        }
#endif
        return TofSafetyStopReason::None;
    }

#if TOF_OBSTACLE_GUARD_ENABLE
    if (sample.distance_mm > 0 && sample.distance_mm < near_limit) {
        return TofSafetyStopReason::Obstacle;
    }
#endif

#if TOF_CLIFF_GUARD_ENABLE
    return CheckFrontCliffInvalid(sample, far_limit, near_limit);
#else
    return TofSafetyStopReason::None;
#endif
}

TofSafetyStopReason CheckFrontFallback(const vl53l0x_data_t& sample, bool edge_proximity) {
#if !TOF_OBSTACLE_GUARD_ENABLE && !TOF_CLIFF_GUARD_ENABLE
    (void)sample;
    (void)edge_proximity;
    return TofSafetyStopReason::None;
#endif

    const int edge_block_mm = EdgeProximityLimitMm(0);

    if (SampleUsable(sample)) {
#if TOF_OBSTACLE_GUARD_ENABLE
        if (sample.distance_mm <= TOF_OBSTACLE_STOP_MM) {
            return TofSafetyStopReason::Obstacle;
        }
#endif
#if TOF_CLIFF_GUARD_ENABLE
        if (sample.distance_mm >= TOF_CLIFF_VOID_MM) {
            return TofSafetyStopReason::CliffFar;
        }
        if (edge_proximity &&
            static_cast<int>(sample.distance_mm) >= edge_block_mm) {
            return TofSafetyStopReason::CliffFar;
        }
#endif
        return TofSafetyStopReason::None;
    }

#if TOF_CLIFF_GUARD_ENABLE
    return CheckFrontCliffInvalid(sample, TOF_CLIFF_VOID_MM, TOF_OBSTACLE_STOP_MM);
#else
    return TofSafetyStopReason::None;
#endif
}

TofSafetyStopReason CheckRear(const vl53l0x_data_t& sample) {
#if !TOF_CLIFF_GUARD_ENABLE
    (void)sample;
    return TofSafetyStopReason::None;
#else
    if (!SampleUsable(sample)) {
        return TofSafetyStopReason::CliffFloor;
    }
    if (sample.distance_mm > TOF_CLIFF_FLOOR_MAX_MM) {
        return TofSafetyStopReason::CliffFloor;
    }
    return TofSafetyStopReason::None;
#endif
}

bool ForwardDistanceLooksSafe(const TofSnapshot& snap) {
    if (!snap.front_ok) {
        return false;
    }

    auto& tof = TofController::Instance();
    const bool use_cal = tof.IsCalibrated();
    const int cal_mm = use_cal ? tof.CalibratedDistanceMm() : 0;

    if (SampleUsable(snap.front)) {
        const int dist = static_cast<int>(snap.front.distance_mm);
        const int safe_mm = SafeForwardDistanceMm(use_cal && cal_mm > 0 ? cal_mm : 0);
        return dist <= safe_mm;
    }

    return false;
}

bool NeedsFrontCheck(int left, int right) {
    return !(left < 0 && right < 0);
}

bool NeedsRearCheck(int left, int right) {
    return left < 0 && right < 0;
}

}  // namespace

void TofNotifyCliffStop(TofSafetyStopReason reason) {
    if (IsCliffReason(reason)) {
        g_cliff_edge_blocked.store(true, std::memory_order_release);
        ESP_LOGW(TAG, "Cliff edge latch set (%s)", TofSafetyStopReasonName(reason));
    }
}

bool TofIsCliffEdgeBlocked() {
    return g_cliff_edge_blocked.load(std::memory_order_acquire);
}

const char* TofSafetyStopReasonName(TofSafetyStopReason reason) {
    switch (reason) {
        case TofSafetyStopReason::Obstacle:
            return "obstacle";
        case TofSafetyStopReason::CliffFloor:
            return "cliff_floor";
        case TofSafetyStopReason::CliffFar:
            return "cliff_far";
        case TofSafetyStopReason::CliffJump:
            return "cliff_jump";
        case TofSafetyStopReason::CliffLostSignal:
            return "cliff_lost_signal";
        case TofSafetyStopReason::SampleFailed:
            return "sample_failed";
        default:
            return "none";
    }
}

TofSafetyStopReason TofEvaluateMoveSnapshot(int left_speed, int right_speed,
                                            const TofSnapshot& snap) {
#if !TOF_OBSTACLE_GUARD_ENABLE && !TOF_CLIFF_GUARD_ENABLE
    (void)left_speed;
    (void)right_speed;
    (void)snap;
    return TofSafetyStopReason::None;
#endif

    auto& tof = TofController::Instance();
    const bool use_cal = tof.IsCalibrated();
    const int cal_mm = use_cal ? tof.CalibratedDistanceMm() : 0;
    const bool forward_check = NeedsFrontCheck(left_speed, right_speed);

    TofSafetyStopReason reason = TofSafetyStopReason::None;

    if (forward_check && snap.front_ok) {
        if (use_cal && cal_mm > 0) {
            reason = CheckFrontCalibrated(snap.front, cal_mm, true);
        } else {
            reason = CheckFrontFallback(snap.front, true);
        }
        if (reason != TofSafetyStopReason::None) {
            return reason;
        }
    } else if (forward_check && !snap.front_ok) {
        return TofSafetyStopReason::SampleFailed;
    }

    if (NeedsRearCheck(left_speed, right_speed) && tof.HasRearSensor()) {
        if (snap.rear_ok) {
            reason = CheckRear(snap.rear);
        } else {
            return TofSafetyStopReason::SampleFailed;
        }
    }

    return reason;
}

bool TofPreMoveCheck(int left_speed, int right_speed, TofSafetyStopReason* reason_out) {
#if !TOF_OBSTACLE_GUARD_ENABLE && !TOF_CLIFF_GUARD_ENABLE
    if (reason_out) {
        *reason_out = TofSafetyStopReason::None;
    }
    return true;
#endif

    auto& tof = TofController::Instance();
    if (!tof.IsReady()) {
        ESP_LOGW(TAG, "Pre-move: ToF not ready — allow move");
        if (reason_out) {
            *reason_out = TofSafetyStopReason::None;
        }
        return true;
    }

    const bool forward_move = NeedsFrontCheck(left_speed, right_speed);
    const bool backward_move = left_speed < 0 && right_speed < 0;

    TofSnapshot snap{};
    if (!tof.SampleOnDemand(&snap)) {
        if (forward_move && TofIsCliffEdgeBlocked()) {
            ESP_LOGW(TAG, "Pre-move: sample failed at cliff edge — block forward L=%d R=%d",
                     left_speed, right_speed);
            if (reason_out) {
                *reason_out = TofSafetyStopReason::CliffFar;
            }
            return false;
        }
        ESP_LOGW(TAG, "Pre-move: sample failed — block move L=%d R=%d", left_speed, right_speed);
        if (reason_out) {
            *reason_out = TofSafetyStopReason::SampleFailed;
        }
        return false;
    }

    if (forward_move && TofIsCliffEdgeBlocked()) {
        if (ForwardDistanceLooksSafe(snap)) {
            g_cliff_edge_blocked.store(false, std::memory_order_release);
            ESP_LOGI(TAG, "Cliff edge latch cleared — safe distance restored");
        } else {
            ESP_LOGW(TAG, "Pre-move BLOCKED cliff_edge_latch front=%u mm L=%d R=%d",
                     snap.front.distance_mm, left_speed, right_speed);
            if (reason_out) {
                *reason_out = TofSafetyStopReason::CliffFar;
            }
            return false;
        }
    }

    const TofSafetyStopReason reason = TofEvaluateMoveSnapshot(left_speed, right_speed, snap);
    if (reason_out) {
        *reason_out = reason;
    }

    if (reason == TofSafetyStopReason::None) {
        if (backward_move) {
            g_cliff_edge_blocked.store(false, std::memory_order_release);
        }
        ESP_LOGI(TAG, "Pre-move OK front=%u mm rear=%u mm L=%d R=%d", snap.front.distance_mm,
                 snap.rear_ok ? snap.rear.distance_mm : 0U, left_speed, right_speed);
        return true;
    }

    ESP_LOGW(TAG, "Pre-move BLOCKED %s front=%u mm valid=%d rear=%u mm L=%d R=%d",
             TofSafetyStopReasonName(reason), snap.front.distance_mm, snap.front.valid,
             snap.rear_ok ? snap.rear.distance_mm : 0U, left_speed, right_speed);
    return false;
}
