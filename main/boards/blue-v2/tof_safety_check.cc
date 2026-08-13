#include "tof_safety_check.h"

#include "config.h"
#include "tof_controller.h"

#include <cstdlib>
#include <esp_log.h>

extern "C" {
#include <vl53l0x.h>
}

#define TAG "TofSafety"

namespace {

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

TofSafetyStopReason CheckFrontCalibrated(const vl53l0x_data_t& sample, int cal_mm) {
#if !TOF_OBSTACLE_GUARD_ENABLE && !TOF_CLIFF_GUARD_ENABLE
    (void)sample;
    (void)cal_mm;
    return TofSafetyStopReason::None;
#endif

    const int near_limit = NearLimit(cal_mm);

    if (SampleUsable(sample)) {
        const int dist = static_cast<int>(sample.distance_mm);
#if TOF_OBSTACLE_GUARD_ENABLE
        if (dist < near_limit) {
            return TofSafetyStopReason::Obstacle;
        }
#endif
#if TOF_CLIFF_GUARD_ENABLE
        if (dist > FarLimit(cal_mm)) {
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
    if (sample.valid && sample.distance_mm >= TOF_MAX_VALID_MM) {
        return TofSafetyStopReason::CliffFar;
    }
#endif
    return TofSafetyStopReason::None;
}

TofSafetyStopReason CheckFrontFallback(const vl53l0x_data_t& sample) {
#if !TOF_OBSTACLE_GUARD_ENABLE && !TOF_CLIFF_GUARD_ENABLE
    (void)sample;
    return TofSafetyStopReason::None;
#endif

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
#endif
        return TofSafetyStopReason::None;
    }

#if TOF_CLIFF_GUARD_ENABLE
    if (sample.valid && sample.distance_mm >= TOF_MAX_VALID_MM) {
        return TofSafetyStopReason::CliffFar;
    }
#endif
    return TofSafetyStopReason::None;
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

bool NeedsFrontCheck(int left, int right) {
    return !(left < 0 && right < 0);
}

bool NeedsRearCheck(int left, int right) {
    return left < 0 && right < 0;
}

}  // namespace

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

    TofSafetyStopReason reason = TofSafetyStopReason::None;

    if (NeedsFrontCheck(left_speed, right_speed) && snap.front_ok) {
        if (use_cal && cal_mm > 0) {
            reason = CheckFrontCalibrated(snap.front, cal_mm);
        } else {
            reason = CheckFrontFallback(snap.front);
        }
        if (reason != TofSafetyStopReason::None) {
            return reason;
        }
    } else if (NeedsFrontCheck(left_speed, right_speed) && !snap.front_ok) {
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

    TofSnapshot snap{};
    if (!tof.SampleOnDemand(&snap)) {
        ESP_LOGW(TAG, "Pre-move: sample failed — block move L=%d R=%d", left_speed, right_speed);
        if (reason_out) {
            *reason_out = TofSafetyStopReason::SampleFailed;
        }
        return false;
    }

    const TofSafetyStopReason reason = TofEvaluateMoveSnapshot(left_speed, right_speed, snap);
    if (reason_out) {
        *reason_out = reason;
    }

    if (reason == TofSafetyStopReason::None) {
        ESP_LOGI(TAG, "Pre-move OK front=%u mm rear=%u mm L=%d R=%d", snap.front.distance_mm,
                 snap.rear_ok ? snap.rear.distance_mm : 0U, left_speed, right_speed);
        return true;
    }

    ESP_LOGW(TAG, "Pre-move BLOCKED %s front=%u mm valid=%d rear=%u mm L=%d R=%d",
             TofSafetyStopReasonName(reason), snap.front.distance_mm, snap.front.valid,
             snap.rear_ok ? snap.rear.distance_mm : 0U, left_speed, right_speed);
    return false;
}
