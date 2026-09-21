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
    };

    struct Cmd {
        CmdType type;
        uint8_t joint;
        int32_t value;
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
    bool Enqueue(CmdType type, uint8_t joint = 0, int32_t value = 0);

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
    float vel_deg_s_[SERVO_COUNT] = {};  // per-joint speed for the trapezoid profile
    int64_t last_target_ms_ = 0;

    static ServoController* instance_;
};

#endif  // _BLUE_V4_SERVO_CONTROLLER_H_
