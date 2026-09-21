#ifndef _BLUE_V4_SERVO_CONTROLLER_H_
#define _BLUE_V4_SERVO_CONTROLLER_H_

#include "config.h"
#include "pca9685.h"

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <cstdint>
#include <mutex>
#include <string>

// Joint index -> PCA9685 channel (Blue V4 default wiring).
enum BlueV4Joint : uint8_t {
    kJointFrontLeftHip = 0,
    kJointFrontLeftKnee = 1,
    kJointFrontRightHip = 2,
    kJointFrontRightKnee = 3,
    kJointRearLeftHip = 4,
    kJointRearLeftKnee = 5,
    kJointRearRightHip = 6,
    kJointRearRightKnee = 7,
};

// ServoController owns the PCA9685 and is the ONLY task that talks to it.
//
// Producers (MCP tools, GaitEngine) publish targets through SetTargets(); the
// worker task interpolates toward them at SERVO_UPDATE_PERIOD_MS and writes the
// changed channels. Discrete commands (relax/enable/trim) go through a small
// queue. This mirrors the blue-v2 MotorController producer/consumer split.
class ServoController {
public:
    ServoController() = default;
    ~ServoController() = default;

    static ServoController* Instance();

    // Creates the PCA9685 on the board-owned bus, loads NVS trims, starts the
    // worker task and registers the self.servo.* MCP tools.
    bool Init(i2c_master_bus_handle_t bus);

    bool IsReady() const { return ready_; }
    bool IsRelaxed() const;
    bool IsMoving() const;
    bool OutputsEnabled() const;
    bool HasHardware() const;

    // True while any joint is slewing — the audio service uses this to drop mic
    // frames during motion (mirrors blue-v2's MotorController uplink pause).
    static bool ShouldPauseUplink();

    // ---- producer API (thread safe, no I2C in the caller) ----
    void SetTargets(const float angles_deg[SERVO_COUNT]);
    void SetTarget(uint8_t joint, float angle_deg);
    void SetSlewDegPerSec(float deg_per_sec);
    void SetAccelDegPerSec2(float deg_per_sec2);

    // Pose hold: while true the idle-relax timeout does not relax the servos.
    void SetHold(bool hold);

    // Bring every joint to the neutral point after delay_ms (soft boot pose).
    // Cancelled automatically as soon as any pose/target is published.
    void StartBootNeutral(uint32_t delay_ms);

    // Move ONE joint from its current angle to `to_deg` over duration_ms (smoothstep). Runs in the
    // worker task, independent of the gait engine: use it to prove the PWM path works.
    bool SweepJoint(uint8_t joint, float to_deg, uint32_t duration_ms);

    // ---- pulse calibration ----
    // Raw pulse band (us) that maps onto 0..180 deg. Defaults come from config.h, overridable at
    // runtime (persisted in NVS) so a servo whose usable band is e.g. 1000..2000 us can be matched
    // without reflashing.
    void SetPulseRange(uint16_t min_us, uint16_t max_us);
    void GetPulseRange(uint16_t* min_us, uint16_t* max_us) const;
    // Hold one joint at an exact pulse width, bypassing the angle mapping entirely (hardware test).
    bool RawPulse(uint8_t joint, uint16_t pulse_us);
    // Drive one raw PCA9685 channel (0..15) at an exact pulse width — use it to test a spare
    // channel (8..15) with a known-good servo, or to prove a channel still works.
    bool RawChannel(uint8_t channel, uint16_t pulse_us);

    // Queue discrete commands (non-blocking).
    bool Relax();
    bool EnableOutputs();
    bool SaveTrims();

    // ---- feedback (no I2C) ----
    void GetCommandedAngles(float out[SERVO_COUNT]) const;
    void GetCommandedPulseUs(uint16_t out[SERVO_COUNT]) const;
    void GetTrimAngles(float out[SERVO_COUNT]) const;

    // ---- calibration ----
    std::string SetTrim(uint8_t joint, int trim_deg);  // applies + persists
    std::string GetTrimsJson() const;
    void SetInverted(uint8_t joint, bool inverted);
    bool GetInverted(uint8_t joint) const;

    void RegisterMcpTools();

private:
    enum class CmdType : uint8_t {
        kRelax,
        kEnable,
        kSaveTrims,
        kSetTrim,
        kSetInverted,
        kStop,
        kSweep,
    };

    struct Cmd {
        CmdType type;
        uint8_t joint;
        int32_t value;
        int32_t value2;
        int32_t value3;
    };

    void LoadTrims();
    bool PersistTrims();
    void RegisterJointLimits();
    float ClampAngle(uint8_t joint, float angle_deg) const;
    uint16_t AngleToPulseUs(uint8_t joint, float angle_deg) const;
    void ApplyRelaxedLocked();
    void ApplyEnabledLocked();
    void Tick(float dt_s);
    void RunCommand(const Cmd& cmd);
    static void TaskEntry(void* arg);
    void TaskLoop();
    bool Enqueue(CmdType type, uint8_t joint = 0, int32_t value = 0, int32_t value2 = 0,
                 int32_t value3 = 0);

    Pca9685 pca_;
    bool ready_ = false;
    bool hardware_ = false;

    QueueHandle_t cmd_queue_ = nullptr;
    TaskHandle_t task_ = nullptr;

    mutable std::mutex state_mutex_;
    float target_deg_[SERVO_COUNT] = {};
    float current_deg_[SERVO_COUNT] = {};
    float trim_deg_[SERVO_COUNT] = {};
    float min_deg_[SERVO_COUNT] = {};
    float max_deg_[SERVO_COUNT] = {};
    bool inverted_[SERVO_COUNT] = {};
    uint16_t last_ticks_[SERVO_COUNT] = {};
    bool hold_ = false;
    bool relaxed_ = true;
    float slew_deg_per_sec_ = SERVO_SLEW_DEG_PER_SEC;
    float accel_deg_per_sec2_ = SERVO_ACCEL_DEG_PER_SEC2;
    float vel_deg_s_[SERVO_COUNT] = {};        // per-joint speed for the trapezoid profile
    int64_t hold_until_ms_[SERVO_COUNT] = {};  // giãn nhịp: chưa tới giờ thì chưa khởi động
    uint8_t pwm_write_cursor_ = 0;             // xoay vòng kênh ghi PWM mỗi tick
    int64_t last_target_ms_ = 0;
    int64_t boot_neutral_at_ms_ = 0;  // 0 = inactive; set by StartBootNeutral()
    uint32_t tick_writes_ = 0;        // PCA9685 writes in the last Tick()
    uint32_t tick_write_fails_ = 0;   // PCA9685 writes that failed in the last Tick()
    int64_t boot_release_at_ms_ = 0;  // 0 = inactive; release pose hold at this time
    int64_t sweep_end_ms_ = 0;        // 0 = inactive; smoothstep sweep deadline
    int64_t sweep_start_ms_ = 0;
    uint8_t sweep_joint_ = 0;
    float sweep_from_deg_ = SERVO_DEFAULT_NEUTRAL_DEG;
    float sweep_to_deg_ = SERVO_DEFAULT_NEUTRAL_DEG;
    uint16_t min_pulse_us_ = SERVO_MIN_PULSE_US;
    uint16_t max_pulse_us_ = SERVO_MAX_PULSE_US;

    static ServoController* instance_;
};

#endif  // _BLUE_V4_SERVO_CONTROLLER_H_
