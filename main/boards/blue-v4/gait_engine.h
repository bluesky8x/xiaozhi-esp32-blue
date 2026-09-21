#ifndef _BLUE_V4_GAIT_ENGINE_H_
#define _BLUE_V4_GAIT_ENGINE_H_

#include "config.h"
#include "servo_controller.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdint>
#include <string>

// Legs in the order used everywhere in Blue V4.
enum class BlueV4Leg : uint8_t {
    kFrontLeft = 0,
    kFrontRight = 1,
    kRearLeft = 2,
    kRearRight = 3,
};

// GaitEngine turns body commands (stand / sit / walk / turn / posture) into joint
// targets and publishes them to ServoController. It owns the inverse kinematics
// and the crawl gait sequencing; ServoController owns the hardware and the slew
// limiting.
//
// Motion is intentionally conservative: the crawl gait moves ONE leg at a time so
// at most two joints are loaded at once, which keeps the 5 V/2 A logic+servo rail
// inside budget.
class GaitEngine {
public:
    GaitEngine() = default;
    ~GaitEngine() = default;

    static GaitEngine* Instance();

    bool Init(ServoController* servos);

    bool IsBusy() const { return busy_; }
    std::string StatusJson() const;

    // Command queue (non-blocking; returns false when busy/queue full).
    bool EnqueueStand(int height_mm = 0);
    bool EnqueueSit();
    bool EnqueueRelax();
    bool EnqueueStop();
    bool EnqueueWalk(const std::string& direction, int steps, int stride_mm, int step_ms);
    bool EnqueueTurn(const std::string& direction, int steps, int step_ms);
    bool EnqueueBody(int height_mm, int pitch_deg, int roll_deg);
    // Servo dance driven by a timeline of gesture letters (D/g/v/c), one per segment.
    bool EnqueueDance(int segment_ms, const std::string& timeline);
    // Bring-up: place ONE leg's foot by IK (leg 0-3) while the others stand.
    bool EnqueueLegTest(int leg, int foot_x_mm, int foot_z_mm);
    // Bring-up: slowly sweep ONE leg's hip/knee in joint space (no IK), the others hold.
    // hip_deg = total hip travel (e.g. 120), knee_deg = knee fold (e.g. 60).
    bool EnqueueLegSweep(int leg, int hip_deg, int knee_deg, int duration_ms);

    // Registers self.gait.* MCP tools.
    void RegisterMcpTools();

private:
    enum class CmdType : uint8_t {
        kStand,
        kSit,
        kRelax,
        kStop,
        kWalk,
        kTurn,
        kBody,
        kDance,
        kLegTest,
        kLegSweep,
    };

    struct Cmd {
        CmdType type = CmdType::kStand;
        int32_t a = 0;    // height_mm / steps / segment_ms
        int32_t b = 0;    // stride_mm / step_ms
        int32_t c = 0;    // step_ms / pitch
        int32_t d = 0;    // roll
        int8_t sign = 1;  // walk/turn direction (+1 = forward / left)
        bool is_turn = false;
        char timeline[65] = {};  // dance gesture letters (fixed size: queue stays POD)
    };

    struct LegState {
        float foot_x_mm = 0.0f;  // forward offset from the hip axis
        float foot_z_mm = 0.0f;  // vertical drop below the hip axis (positive down)
        float lift_mm = 0.0f;    // current foot lift while swinging
    };

    static void TaskEntry(void* arg);
    void TaskLoop();

    bool Enqueue(const Cmd& cmd);
    void RunCommand(const Cmd& cmd);

    void ApplyPose(float height_mm, float pitch_deg, float roll_deg);
    void ApplyRelax();

    // Inverse kinematics for one leg in the sagittal plane.
    bool SolveLeg(float foot_x_mm, float foot_z_mm, float* hip_deg, float* knee_deg) const;
    // Raw IK: beta = femur angle from vertical (hip), interior = angle between femur and
    // tibia (180 = straight leg).
    bool SolveLegRaw(float foot_x_mm, float foot_z_mm, float* beta_deg,
                     float* knee_interior_deg) const;
    // Maps raw IK angles to servo degrees, referenced to the stand pose (=> 90 deg).
    void MapToServo(float beta_deg, float knee_interior_deg, float* hip_deg, float* knee_deg) const;

    void PublishLegs(const LegState legs[4], float body_height_mm, float pitch_deg, float roll_deg);

    std::string RunWalk(int steps, int stride_mm, int step_ms, int8_t sign);
    // Joint-space crawl: hip 0..GAIT_JOINT_HIP_TRAVEL_DEG, knee 0..GAIT_JOINT_KNEE_TRAVEL_DEG.
    std::string RunWalkJoint(int steps, int step_ms, int8_t sign);
    std::string RunLegSweep(int leg, int hip_deg, int knee_deg, int duration_ms);
    std::string RunTurn(int steps, int step_ms, int8_t sign);
    std::string RunDance(int segment_ms, const char* timeline);

    ServoController* servos_ = nullptr;
    QueueHandle_t queue_ = nullptr;
    TaskHandle_t task_ = nullptr;
    bool ready_ = false;

    std::atomic<bool> busy_{false};
    std::atomic<bool> cancel_{false};

    float body_height_mm_ = BODY_STAND_HEIGHT_MM;
    float pitch_deg_ = 0.0f;
    float roll_deg_ = 0.0f;
    // Mechanical neutral (stand pose) joint angles — both servos read 90 deg there.
    float ref_hip_beta_deg_ = 0.0f;
    float ref_knee_interior_deg_ = 180.0f;

    LegState legs_[4] = {};

    static GaitEngine* instance_;
};

#endif  // _BLUE_V4_GAIT_ENGINE_H_
