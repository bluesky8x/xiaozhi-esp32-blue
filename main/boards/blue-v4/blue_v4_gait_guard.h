#ifndef _BLUE_V4_GAIT_GUARD_H_
#define _BLUE_V4_GAIT_GUARD_H_

#include "gait_engine.h"

#include <cstdint>

// Stops the crawl gait when the front VL53L0X sees an obstacle or a drop-off.
// Same intent as blue-v2's TofMotorGuard: it only stops (never steers), so it can
// never walk the robot off a table on its own. Requires self.tof.calibrate first;
// without calibration it falls back to the fixed thresholds in config.h.
class BlueV4GaitGuard {
public:
    explicit BlueV4GaitGuard(GaitEngine* gait);

    // Requires the ToF controller to be initialised first. Reads the sampler's
    // latest snapshot only (no I2C from this task).
    bool Start();

    static void TaskEntry(void* arg);

private:
    void GuardLoop();
    void TriggerStop(const char* reason, int dist_mm, int limit_mm);

    GaitEngine* gait_ = nullptr;
    bool active_ = false;
    bool was_busy_ = false;
    bool have_last_ = false;
    int last_dist_mm_ = 0;
    int invalid_readings_ = 0;
    int64_t motion_started_ms_ = 0;
    // Log throttle (walking polls at 50 ms — do not flood the UART/monitor).
    int last_log_dist_mm_ = -1;
    int64_t last_log_ms_ = 0;
};

#endif  // _BLUE_V4_GAIT_GUARD_H_
