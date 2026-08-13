#ifndef BLUE_V2_TOF_SAFETY_CHECK_H_
#define BLUE_V2_TOF_SAFETY_CHECK_H_

#include "tof_controller.h"

enum class TofSafetyStopReason {
    None,
    Obstacle,
    CliffFloor,
    CliffFar,
    CliffJump,
    CliffLostSignal,
    SampleFailed,
};

const char* TofSafetyStopReasonName(TofSafetyStopReason reason);

/** Evaluate a snapshot for the given wheel command (pre-move, no prior sample). */
TofSafetyStopReason TofEvaluateMoveSnapshot(int left_speed, int right_speed,
                                            const TofSnapshot& snap);

/**
 * Fresh ToF read + safety check before the first move in the motor queue.
 * Returns true when safe to ExecuteMove (or guards disabled / ToF unavailable).
 */
bool TofPreMoveCheck(int left_speed, int right_speed, TofSafetyStopReason* reason_out);

#endif  // BLUE_V2_TOF_SAFETY_CHECK_H_
