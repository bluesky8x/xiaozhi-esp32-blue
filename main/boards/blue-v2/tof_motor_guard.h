#ifndef BLUE_V2_TOF_MOTOR_GUARD_H_
#define BLUE_V2_TOF_MOTOR_GUARD_H_

#include "motor_controller.h"

class TofMotorGuard {
public:
    explicit TofMotorGuard(MotorController* motor);

    // Requires TofController::Init() + sampler first. Reads latest snapshot only (no I2C).
    bool Start();

private:
    static void TaskEntry(void* arg);
    void PollOnce();

    MotorController* motor_;
    bool active_ = false;
};

#endif  // BLUE_V2_TOF_MOTOR_GUARD_H_
