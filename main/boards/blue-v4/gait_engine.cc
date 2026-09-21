#include "gait_engine.h"

#include "mcp_server.h"

#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#define TAG "GaitEngine"

namespace {
constexpr const char* kStateIdle = "idle";
constexpr const char* kStateHolding = "holding";
constexpr const char* kStateWalking = "walking";
constexpr const char* kStateTurning = "turning";
constexpr const char* kStateRelaxed = "relaxed";

constexpr int kTaskStackWords = 4096;
constexpr UBaseType_t kTaskPriority = 4;

constexpr float kRadToDeg = 57.29578f;
constexpr float kDegToRad = 0.017453292f;

// Crawl gait order: the leg diagonally opposite the previous one, so the robot
// always has three feet planted. This is the low-current, statically stable gait
// (one leg = 2 loaded joints at a time) that suits the 5 V/2 A supply.
constexpr BlueV4Leg kCrawlOrder[4] = {BlueV4Leg::kFrontRight, BlueV4Leg::kRearLeft,
                                      BlueV4Leg::kFrontLeft, BlueV4Leg::kRearRight};

// Foot lift while swinging (keeps the foot clear of the ground).
constexpr float kSwingLiftMm = 14.0f;

// Servo mapping. The stand pose (foot straight below the hip at BODY_STAND_HEIGHT_MM) is
// the mechanical neutral: both servos sit at 90 deg there, so the walking swing stays well
// inside the 0-180 deg servo range and trim/invert calibration happens around 90 deg.
// Flip the signs (or use self.servo.invert per joint) if a leg moves the wrong way.
constexpr float kHipServoCenterDeg = 90.0f;
constexpr float kHipServoSign = 1.0f;  // +1 => swinging the foot forward increases the angle
constexpr float kKneeServoCenterDeg = 90.0f;
constexpr float kKneeServoSign = 1.0f;  // +1 => folding the knee increases the angle

// Minimum reach margin so the IK never asks for a fully straight/over-folded leg.
constexpr float kReachMarginMm = 2.0f;
}  // namespace

GaitEngine* GaitEngine::instance_ = nullptr;

GaitEngine* GaitEngine::Instance() { return instance_; }

bool GaitEngine::Init(ServoController* servos) {
    if (servos == nullptr) {
        ESP_LOGE(TAG, "Init without a ServoController");
        return false;
    }
    instance_ = this;
    servos_ = servos;

    body_height_mm_ = BODY_STAND_HEIGHT_MM;
    for (auto& leg : legs_) {
        leg.foot_x_mm = 0.0f;
        leg.foot_z_mm = BODY_STAND_HEIGHT_MM;
        leg.lift_mm = 0.0f;
    }

    // Stand pose = mechanical neutral; both servos read 90 deg there.
    SolveLegRaw(0.0f, BODY_STAND_HEIGHT_MM, &ref_hip_beta_deg_, &ref_knee_interior_deg_);
    ESP_LOGI(TAG, "neutral: hip beta %.1f deg, knee interior %.1f deg (servo 90/90)",
             static_cast<double>(ref_hip_beta_deg_), static_cast<double>(ref_knee_interior_deg_));

    queue_ = xQueueCreate(BLUE_V4_GAIT_QUEUE_DEPTH, sizeof(Cmd));
    if (queue_ == nullptr) {
        ESP_LOGE(TAG, "command queue allocation failed");
        return false;
    }
    if (xTaskCreate(TaskEntry, "gait", kTaskStackWords, this, kTaskPriority, &task_) != pdPASS) {
        ESP_LOGE(TAG, "task creation failed");
        return false;
    }

    ready_ = true;
    RegisterMcpTools();
    ESP_LOGI(TAG, "ready (femur %.0f mm, tibia %.0f mm, stand %.0f mm, crawl gait)",
             static_cast<double>(LEG_FEMUR_MM), static_cast<double>(LEG_TIBIA_MM),
             static_cast<double>(body_height_mm_));
    return true;
}

bool GaitEngine::Enqueue(const Cmd& cmd) {
    if (queue_ == nullptr) {
        return false;
    }
    // Motion commands queue up and run one after another: the server sends mv:* sequences
    // (e.g. "mv:f mv:p mv:s") and each step arrives while the previous one is still running.
    // Stop/relax additionally set cancel_ so a running walk aborts at the next leg boundary.
    return xQueueSend(queue_, &cmd, 0) == pdTRUE;
}

bool GaitEngine::EnqueueStand(int height_mm) {
    Cmd cmd;
    cmd.type = CmdType::kStand;
    cmd.a = height_mm;
    return Enqueue(cmd);
}

bool GaitEngine::EnqueueSit() {
    Cmd cmd;
    cmd.type = CmdType::kSit;
    return Enqueue(cmd);
}

bool GaitEngine::EnqueueRelax() {
    cancel_.store(true);
    Cmd cmd;
    cmd.type = CmdType::kRelax;
    return Enqueue(cmd);
}

bool GaitEngine::EnqueueStop() {
    // Cancel first: a run in progress must abort at its next leg boundary, not after
    // the whole walk finishes (the ToF guard depends on this).
    cancel_.store(true);
    Cmd cmd;
    cmd.type = CmdType::kStop;
    return Enqueue(cmd);
}

bool GaitEngine::EnqueueWalk(const std::string& direction, int steps, int stride_mm, int step_ms) {
    Cmd cmd;
    cmd.type = CmdType::kWalk;
    cmd.a = steps;
    cmd.b = stride_mm;
    cmd.c = step_ms;
    cmd.sign = direction == "-1" ? -1 : 1;
    return Enqueue(cmd);
}

bool GaitEngine::EnqueueTurn(const std::string& direction, int steps, int step_ms) {
    Cmd cmd;
    cmd.type = CmdType::kTurn;
    cmd.a = steps;
    cmd.c = step_ms;
    cmd.sign = direction == "-1" ? -1 : 1;
    return Enqueue(cmd);
}

bool GaitEngine::EnqueueBody(int height_mm, int pitch_deg, int roll_deg) {
    Cmd cmd;
    cmd.type = CmdType::kBody;
    cmd.a = height_mm;
    cmd.b = pitch_deg;
    cmd.c = roll_deg;
    return Enqueue(cmd);
}

bool GaitEngine::EnqueueDance(int segment_ms, const std::string& timeline) {
    Cmd cmd;
    cmd.type = CmdType::kDance;
    cmd.a = segment_ms;
    snprintf(cmd.timeline, sizeof(cmd.timeline), "%s", timeline.c_str());
    return Enqueue(cmd);
}

bool GaitEngine::EnqueueLegTest(int leg, int foot_x_mm, int foot_z_mm) {
    Cmd cmd;
    cmd.type = CmdType::kLegTest;
    cmd.a = leg;
    cmd.b = foot_x_mm;
    cmd.c = foot_z_mm;
    return Enqueue(cmd);
}

bool GaitEngine::EnqueueLegSweep(int leg, int hip_deg, int knee_deg, int duration_ms) {
    Cmd cmd;
    cmd.type = CmdType::kLegSweep;
    cmd.a = leg;
    cmd.b = hip_deg;
    cmd.c = knee_deg;
    cmd.d = duration_ms;
    return Enqueue(cmd);
}

bool GaitEngine::SolveLegRaw(float foot_x_mm, float foot_z_mm, float* beta_deg,
                             float* knee_interior_deg) const {
    const float femur = LEG_FEMUR_MM;
    const float tibia = LEG_TIBIA_MM;

    float reach = sqrtf(foot_x_mm * foot_x_mm + foot_z_mm * foot_z_mm);
    const float reach_min = fabsf(femur - tibia) + kReachMarginMm;
    const float reach_max = femur + tibia - kReachMarginMm;
    reach = std::clamp(reach, reach_min, reach_max);
    // Re-derive the working position after clamping so the angles stay consistent.
    const float angle_to_foot = atan2f(foot_x_mm, std::max(foot_z_mm, 0.001f));
    const float clamped_x = reach * sinf(angle_to_foot);
    const float clamped_z = reach * cosf(angle_to_foot);

    float cos_knee = (femur * femur + tibia * tibia - reach * reach) / (2.0f * femur * tibia);
    cos_knee = std::clamp(cos_knee, -1.0f, 1.0f);
    *knee_interior_deg = acosf(cos_knee) * kRadToDeg;  // 180 = straight leg

    float cos_hip = (femur * femur + reach * reach - tibia * tibia) / (2.0f * femur * reach);
    cos_hip = std::clamp(cos_hip, -1.0f, 1.0f);
    const float hip_offset_deg = acosf(cos_hip) * kRadToDeg;
    *beta_deg = atan2f(clamped_x, clamped_z) * kRadToDeg + hip_offset_deg;
    return true;
}

void GaitEngine::MapToServo(float beta_deg, float knee_interior_deg, float* hip_deg,
                            float* knee_deg) const {
    // Both joints read 90 deg in the stand pose, so the walking swing stays centered and
    // inside 0-180 deg even for very folded leg geometries.
    *hip_deg = kHipServoCenterDeg + kHipServoSign * (beta_deg - ref_hip_beta_deg_);
    *knee_deg = kKneeServoCenterDeg + kKneeServoSign * (ref_knee_interior_deg_ - knee_interior_deg);
}

bool GaitEngine::SolveLeg(float foot_x_mm, float foot_z_mm, float* hip_deg, float* knee_deg) const {
    float beta_deg = 0.0f;
    float knee_interior_deg = 0.0f;
    SolveLegRaw(foot_x_mm, foot_z_mm, &beta_deg, &knee_interior_deg);
    MapToServo(beta_deg, knee_interior_deg, hip_deg, knee_deg);
    return true;
}

void GaitEngine::PublishLegs(const LegState legs[4], float body_height_mm, float pitch_deg,
                             float roll_deg) {
    float angles[SERVO_COUNT] = {};
    const float pitch_rad = pitch_deg * kDegToRad;
    const float roll_rad = roll_deg * kDegToRad;

    for (int i = 0; i < 4; i++) {
        const bool is_front = (i == static_cast<int>(BlueV4Leg::kFrontLeft) ||
                               i == static_cast<int>(BlueV4Leg::kFrontRight));
        const bool is_left = (i == static_cast<int>(BlueV4Leg::kFrontLeft) ||
                              i == static_cast<int>(BlueV4Leg::kRearLeft));
        const float hip_x = (is_front ? LEG_HIP_OFFSET_X_MM : -LEG_HIP_OFFSET_X_MM);
        const float hip_y = (is_left ? LEG_HIP_OFFSET_Y_MM : -LEG_HIP_OFFSET_Y_MM);

        // Small-angle body tilt: fore/aft tilt changes the height of the front vs
        // rear hips, roll changes left vs right.
        const float height = body_height_mm - pitch_rad * hip_x + roll_rad * hip_y;
        const float foot_drop = std::max(height - legs[i].lift_mm, 10.0f);
        const float foot_x = legs[i].foot_x_mm;

        float hip_deg = 90.0f;
        float knee_deg = 90.0f;
        SolveLeg(foot_x, foot_drop, &hip_deg, &knee_deg);

        const int hip_joint = i * 2;
        angles[hip_joint] = hip_deg;
        angles[hip_joint + 1] = knee_deg;
    }

    servos_->SetTargets(angles);
}

void GaitEngine::ApplyPose(float height_mm, float pitch_deg, float roll_deg) {
    body_height_mm_ = std::clamp(height_mm, BODY_MIN_HEIGHT_MM, BODY_MAX_HEIGHT_MM);
    pitch_deg_ = std::clamp(pitch_deg, -20.0f, 20.0f);
    roll_deg_ = std::clamp(roll_deg, -20.0f, 20.0f);

    for (auto& leg : legs_) {
        leg.foot_x_mm = 0.0f;
        leg.lift_mm = 0.0f;
        leg.foot_z_mm = body_height_mm_;
    }
    PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
}

void GaitEngine::ApplyRelax() {
    for (auto& leg : legs_) {
        leg.lift_mm = 0.0f;
    }
    servos_->SetHold(false);
    servos_->Relax();
}

std::string GaitEngine::RunWalkJoint(int steps, int step_ms, int8_t sign) {
    // Joint-space crawl. Hip sweeps hip_center +/- HIP_TRAVEL/2 (=> a full HIP_TRAVEL of
    // travel per step) and the knee folds by KNEE_TRAVEL while the leg swings. All four
    // hips then push back a quarter travel, so each leg returns to its start angle and the
    // robot advances without drifting.
    steps = std::clamp(steps, 1, 12);
    step_ms = std::clamp(step_ms, 200, 6000);

    const float hip_center = GAIT_JOINT_HIP_NEUTRAL_DEG;
    const float hip_half = GAIT_JOINT_HIP_TRAVEL_DEG * 0.5f;
    const float knee_fold = GAIT_JOINT_KNEE_TRAVEL_DEG;
    const float push = GAIT_JOINT_HIP_TRAVEL_DEG / 4.0f;

    // Local source of truth for the joint targets (mount neutral = 90 deg).
    float angles[SERVO_COUNT];
    for (int i = 0; i < SERVO_COUNT; i++) {
        angles[i] = 90.0f;
    }
    for (int leg = 0; leg < 4; leg++) {
        angles[leg * 2] = hip_center;
    }

    servos_->EnableOutputs();
    servos_->SetHold(true);
    servos_->SetTargets(angles);
    vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(step_ms)));

    const int quarter = std::max(step_ms / 4, 100);
    for (int step = 0; step < steps; step++) {
        for (int index = 0; index < 4; index++) {
            if (cancel_.load()) {
                return "cancelled";
            }
            const int leg = static_cast<int>(kCrawlOrder[index]);
            const int hip = leg * 2;
            const int knee = hip + 1;

            // 1) Fold the knee (lift the foot clear).
            angles[knee] = 90.0f + knee_fold;
            servos_->SetTargets(angles);
            vTaskDelay(pdMS_TO_TICKS(quarter));

            // 2) Swing the hip across the full travel.
            angles[hip] = hip_center + hip_half * static_cast<float>(sign);
            servos_->SetTargets(angles);
            vTaskDelay(pdMS_TO_TICKS(quarter * 2));

            // 3) Plant.
            angles[knee] = 90.0f;
            servos_->SetTargets(angles);
            vTaskDelay(pdMS_TO_TICKS(quarter));

            // 4) Body advance: every hip pushes back a quarter travel.
            if (!cancel_.load()) {
                for (int l = 0; l < 4; l++) {
                    angles[l * 2] -= push * static_cast<float>(sign);
                }
                servos_->SetTargets(angles);
            }
        }
    }

    // Finish standing even with every joint back at its mount neutral.
    for (int i = 0; i < SERVO_COUNT; i++) {
        angles[i] = 90.0f;
    }
    servos_->SetTargets(angles);
    return cancel_.load() ? "cancelled" : "walk complete";
}

std::string GaitEngine::RunLegSweep(int leg, int hip_deg, int knee_deg, int duration_ms) {
    const int hip_joint = std::clamp(leg, 0, 3) * 2;
    const int knee_joint = hip_joint + 1;
    const int total_ms = std::clamp(duration_ms, 1000, 15000);
    const float hip_travel = std::clamp(static_cast<float>(hip_deg), 10.0f, 170.0f);
    const float knee_travel = std::clamp(static_cast<float>(knee_deg), 0.0f, 90.0f);

    // Shape the speed so the largest move takes (about) the requested duration.
    const float travel = std::max(hip_travel, knee_travel);
    const float slew = std::max(5.0f, travel / (static_cast<float>(total_ms) / 1000.0f));
    servos_->SetSlewDegPerSec(slew);
    servos_->SetAccelDegPerSec2(std::max(30.0f, slew * 2.0f));

    float angles[SERVO_COUNT];
    servos_->GetCommandedAngles(angles);
    const float hip_start = 90.0f;
    const float knee_start = 90.0f;
    const int segment = std::max(total_ms / 3, 200);

    servos_->EnableOutputs();
    servos_->SetHold(true);

    // 1) Fold the knee + move the hip half travel forward.
    angles[hip_joint] = hip_start + hip_travel * 0.5f;
    angles[knee_joint] = knee_start + knee_travel;
    servos_->SetTargets(angles);
    vTaskDelay(pdMS_TO_TICKS(segment));
    if (cancel_.load()) {
        return "cancelled";
    }

    // 2) Sweep the hip to the other end of the travel.
    angles[hip_joint] = hip_start - hip_travel * 0.5f;
    servos_->SetTargets(angles);
    vTaskDelay(pdMS_TO_TICKS(segment));
    if (cancel_.load()) {
        return "cancelled";
    }

    // 3) Return to neutral.
    angles[hip_joint] = hip_start;
    angles[knee_joint] = knee_start;
    servos_->SetTargets(angles);
    vTaskDelay(pdMS_TO_TICKS(segment));

    // Restore the default motion profile.
    servos_->SetSlewDegPerSec(SERVO_SLEW_DEG_PER_SEC);
    servos_->SetAccelDegPerSec2(SERVO_ACCEL_DEG_PER_SEC2);
    return cancel_.load() ? "cancelled" : "sweep complete";
}
const float stride = std::clamp(static_cast<float>(stride_mm), 10.0f, 60.0f);
steps = std::clamp(steps, 1, 12);
step_ms = std::clamp(step_ms, 200, 1500);

servos_->EnableOutputs();
servos_->SetHold(true);
ApplyPose(body_height_mm_, pitch_deg_, roll_deg_);
vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(step_ms)));

for (int step = 0; step < steps; step++) {
    for (int leg_index = 0; leg_index < 4; leg_index++) {
        if (cancel_.load()) {
            return "cancelled";
        }
        const int leg = static_cast<int>(kCrawlOrder[leg_index]);
        const int half = std::max(step_ms / 4, 40);

        // 1) Lift
        legs_[leg].lift_mm = kSwingLiftMm;
        PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
        vTaskDelay(pdMS_TO_TICKS(half));

        // 2) Swing forward (or backward)
        legs_[leg].foot_x_mm += stride * static_cast<float>(sign);
        PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
        vTaskDelay(pdMS_TO_TICKS(half * 2));

        // 3) Plant
        legs_[leg].lift_mm = 0.0f;
        PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
        vTaskDelay(pdMS_TO_TICKS(half));

        // 4) The three planted legs push back by stride/4: net body advance is
        //    one stride after all four legs have cycled.
        if (!cancel_.load()) {
            for (auto& planted : legs_) {
                planted.foot_x_mm -= stride * 0.25f * static_cast<float>(sign);
            }
            PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
        }
    }
}

// Re-centre the feet for the next command.
for (auto& leg : legs_) {
    leg.foot_x_mm = 0.0f;
    leg.lift_mm = 0.0f;
}
PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
return cancel_.load() ? "cancelled" : "walk complete";
}

std::string GaitEngine::RunTurn(int steps, int step_ms, int8_t sign) {
    const float stride = STRIDE_LENGTH_MM * 0.6f;

    servos_->EnableOutputs();
    servos_->SetHold(true);
    ApplyPose(body_height_mm_, pitch_deg_, roll_deg_);
    vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(step_ms)));

    for (int step = 0; step < steps; step++) {
        for (int leg_index = 0; leg_index < 4; leg_index++) {
            if (cancel_.load()) {
                return "cancelled";
            }
            const int leg = static_cast<int>(kCrawlOrder[leg_index]);
            const bool is_left = (leg == static_cast<int>(BlueV4Leg::kFrontLeft) ||
                                  leg == static_cast<int>(BlueV4Leg::kRearLeft));
            // Left and right legs move in opposite directions: differential stride.
            const float leg_sign = (is_left ? 1.0f : -1.0f) * static_cast<float>(sign);
            const int half = std::max(step_ms / 4, 40);

            legs_[leg].lift_mm = kSwingLiftMm;
            PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
            vTaskDelay(pdMS_TO_TICKS(half));

            legs_[leg].foot_x_mm += stride * leg_sign;
            PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
            vTaskDelay(pdMS_TO_TICKS(half * 2));

            legs_[leg].lift_mm = 0.0f;
            PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
            vTaskDelay(pdMS_TO_TICKS(half));

            if (!cancel_.load()) {
                for (int i = 0; i < 4; i++) {
                    const bool planted_left = (i == static_cast<int>(BlueV4Leg::kFrontLeft) ||
                                               i == static_cast<int>(BlueV4Leg::kRearLeft));
                    const float planted_sign =
                        (planted_left ? -1.0f : 1.0f) * static_cast<float>(sign);
                    legs_[i].foot_x_mm -= stride * 0.25f * planted_sign;
                }
                PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
            }
        }
    }

    for (auto& leg : legs_) {
        leg.foot_x_mm = 0.0f;
        leg.lift_mm = 0.0f;
    }
    PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
    return cancel_.load() ? "cancelled" : "turn complete";
}

std::string GaitEngine::RunDance(int segment_ms, const char* timeline) {
    // Gesture letters (server sends a timeline for wheel dances; here they drive
    // servo gestures): D = bounce, g = sway (roll), v = diagonal leg wave,
    // c = crouch. Unknown letters just hold the pose for one segment.
    constexpr int kMaxDanceMs = 30000;
    constexpr int kMinSegmentMs = 150;

    if (timeline == nullptr || timeline[0] == '\0') {
        return "no timeline";
    }
    const size_t segments = strnlen(timeline, sizeof(Cmd::timeline));
    int seg_ms = std::clamp(segment_ms, 200, 3000);
    if (static_cast<int64_t>(segments) * seg_ms > kMaxDanceMs) {
        seg_ms = std::max(kMinSegmentMs, kMaxDanceMs / static_cast<int>(segments));
    }

    servos_->EnableOutputs();
    servos_->SetHold(true);
    ApplyPose(BODY_STAND_HEIGHT_MM, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(300));

    for (size_t i = 0; i < segments && !cancel_.load(); i++) {
        const int third = std::max(seg_ms / 3, 60);
        switch (timeline[i]) {
            case 'D':
            case 'd':
                ApplyPose(BODY_STAND_HEIGHT_MM - 18.0f, 0.0f, 0.0f);
                vTaskDelay(pdMS_TO_TICKS(third));
                ApplyPose(BODY_STAND_HEIGHT_MM + 6.0f, 0.0f, 0.0f);
                vTaskDelay(pdMS_TO_TICKS(third));
                ApplyPose(BODY_STAND_HEIGHT_MM, 0.0f, 0.0f);
                vTaskDelay(pdMS_TO_TICKS(third));
                break;
            case 'g':
                ApplyPose(BODY_STAND_HEIGHT_MM, 0.0f, 14.0f);
                vTaskDelay(pdMS_TO_TICKS(third));
                ApplyPose(BODY_STAND_HEIGHT_MM, 0.0f, -14.0f);
                vTaskDelay(pdMS_TO_TICKS(third));
                ApplyPose(BODY_STAND_HEIGHT_MM, 0.0f, 0.0f);
                vTaskDelay(pdMS_TO_TICKS(third));
                break;
            case 'v': {
                for (auto& leg : legs_) {
                    leg.foot_x_mm = 0.0f;
                    leg.lift_mm = 0.0f;
                }
                legs_[static_cast<int>(BlueV4Leg::kFrontLeft)].lift_mm = kSwingLiftMm + 6.0f;
                legs_[static_cast<int>(BlueV4Leg::kRearRight)].lift_mm = kSwingLiftMm + 6.0f;
                PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
                vTaskDelay(pdMS_TO_TICKS(third));
                legs_[static_cast<int>(BlueV4Leg::kFrontLeft)].lift_mm = 0.0f;
                legs_[static_cast<int>(BlueV4Leg::kRearRight)].lift_mm = 0.0f;
                legs_[static_cast<int>(BlueV4Leg::kFrontRight)].lift_mm = kSwingLiftMm + 6.0f;
                legs_[static_cast<int>(BlueV4Leg::kRearLeft)].lift_mm = kSwingLiftMm + 6.0f;
                PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
                vTaskDelay(pdMS_TO_TICKS(third));
                for (auto& leg : legs_) {
                    leg.lift_mm = 0.0f;
                }
                PublishLegs(legs_, body_height_mm_, pitch_deg_, roll_deg_);
                vTaskDelay(pdMS_TO_TICKS(third));
                break;
            }
            case 'c':
                ApplyPose(BODY_MIN_HEIGHT_MM, 0.0f, 0.0f);
                vTaskDelay(pdMS_TO_TICKS(third));
                ApplyPose(BODY_STAND_HEIGHT_MM, 0.0f, 0.0f);
                vTaskDelay(pdMS_TO_TICKS(seg_ms - third));
                break;
            default:
                vTaskDelay(pdMS_TO_TICKS(seg_ms));
                break;
        }
    }

    for (auto& leg : legs_) {
        leg.foot_x_mm = 0.0f;
        leg.lift_mm = 0.0f;
    }
    ApplyPose(BODY_STAND_HEIGHT_MM, 0.0f, 0.0f);
    return cancel_.load() ? "cancelled" : "dance complete";
}

void GaitEngine::RunCommand(const Cmd& cmd) {
    switch (cmd.type) {
        case CmdType::kStand: {
            cancel_.store(false);
            const float height = cmd.a > 0 ? static_cast<float>(cmd.a) : BODY_STAND_HEIGHT_MM;
            servos_->EnableOutputs();
            servos_->SetHold(true);
            ApplyPose(height, 0.0f, 0.0f);
            ESP_LOGI(TAG, "stand at %.0f mm", static_cast<double>(body_height_mm_));
            break;
        }
        case CmdType::kSit: {
            cancel_.store(false);
            servos_->EnableOutputs();
            servos_->SetHold(true);
            // Fold down and tilt forward — the classic relaxed sit for 2-DoF legs.
            ApplyPose(BODY_MIN_HEIGHT_MM, 12.0f, 0.0f);
            ESP_LOGI(TAG, "sit (height %.0f mm)", static_cast<double>(body_height_mm_));
            break;
        }
        case CmdType::kRelax:
            cancel_.store(true);
            ApplyRelax();
            ESP_LOGI(TAG, "relax");
            break;
        case CmdType::kStop:
            cancel_.store(true);
            ApplyRelax();
            ESP_LOGI(TAG, "stop");
            break;
        case CmdType::kBody: {
            cancel_.store(false);
            servos_->EnableOutputs();
            servos_->SetHold(true);
            const float height = cmd.a > 0 ? static_cast<float>(cmd.a) : BODY_STAND_HEIGHT_MM;
            ApplyPose(height, static_cast<float>(cmd.b), static_cast<float>(cmd.c));
            ESP_LOGI(TAG, "body %.0f mm pitch %.0f roll %.0f", static_cast<double>(body_height_mm_),
                     static_cast<double>(pitch_deg_), static_cast<double>(roll_deg_));
            break;
        }
        case CmdType::kWalk: {
            cancel_.store(false);
            busy_.store(true);
#if BLUE_V4_JOINT_SPACE_GAIT
            const int joint_step_ms = cmd.c > 0 ? cmd.c : GAIT_JOINT_STEP_MS;
            const std::string result = RunWalkJoint(cmd.a, joint_step_ms, cmd.sign);
#else
            const std::string result = RunWalk(cmd.a, cmd.b, cmd.c, cmd.sign);
#endif
            busy_.store(false);
            ESP_LOGI(TAG, "walk %d steps: %s", cmd.a, result.c_str());
            break;
        }
        case CmdType::kLegSweep: {
            cancel_.store(false);
            const int leg = std::clamp(static_cast<int>(cmd.a), 0, 3);
            const std::string result = RunLegSweep(leg, cmd.b, cmd.c, cmd.d);
            ESP_LOGI(TAG, "leg %d sweep (hip %d deg, knee %d deg): %s", leg, cmd.b, cmd.c,
                     result.c_str());
            break;
        }
        case CmdType::kTurn: {
            cancel_.store(false);
            busy_.store(true);
            const std::string result = RunTurn(cmd.a, cmd.c, cmd.sign);
            busy_.store(false);
            ESP_LOGI(TAG, "turn %d steps: %s", cmd.a, result.c_str());
            break;
        }
        case CmdType::kDance: {
            cancel_.store(false);
            busy_.store(true);
            const std::string result = RunDance(cmd.a, cmd.timeline);
            busy_.store(false);
            ESP_LOGI(TAG, "dance: %s", result.c_str());
            break;
        }
        case CmdType::kLegTest: {
            cancel_.store(false);
            const int leg = std::clamp(static_cast<int>(cmd.a), 0, 3);
            const float foot_x = std::clamp(static_cast<float>(cmd.b), -60.0f, 60.0f);
            const float foot_z = std::clamp(static_cast<float>(cmd.c), 40.0f, 150.0f);
            servos_->EnableOutputs();
            servos_->SetHold(true);
            for (int i = 0; i < 4; i++) {
                legs_[i].foot_x_mm = (i == leg) ? foot_x : 0.0f;
                // PublishLegs uses body_height - lift, so this keeps the other three legs
                // at the stand height while the tested leg drops to foot_z.
                legs_[i].lift_mm = (i == leg) ? (BODY_STAND_HEIGHT_MM - foot_z) : 0.0f;
            }
            PublishLegs(legs_, BODY_STAND_HEIGHT_MM, 0.0f, 0.0f);
            ESP_LOGI(TAG,
                     "leg %d test: foot_x=%.0f mm foot_z=%.0f mm (hip joint %d, knee joint %d)",
                     leg, static_cast<double>(foot_x), static_cast<double>(foot_z), leg * 2,
                     leg * 2 + 1);
            break;
        }
    }
}

void GaitEngine::TaskEntry(void* arg) { static_cast<GaitEngine*>(arg)->TaskLoop(); }

void GaitEngine::TaskLoop() {
    while (true) {
        Cmd cmd;
        if (xQueueReceive(queue_, &cmd, portMAX_DELAY) == pdTRUE) {
            RunCommand(cmd);
        }
    }
}

std::string GaitEngine::StatusJson() const {
    const char* state = kStateIdle;
    if (servos_ != nullptr && servos_->IsRelaxed()) {
        state = kStateRelaxed;
    } else if (busy_.load()) {
        state = kStateWalking;
    } else if (servos_ != nullptr && servos_->IsMoving()) {
        state = kStateHolding;
    } else {
        state = kStateHolding;
    }

    char buffer[192];
    snprintf(buffer, sizeof(buffer),
             "{\"state\":\"%s\",\"height_mm\":%.0f,\"pitch_deg\":%.0f,\"roll_deg\":%.0f,"
             "\"busy\":%s,\"servo_ready\":%s,\"hardware\":%s}",
             state, static_cast<double>(body_height_mm_), static_cast<double>(pitch_deg_),
             static_cast<double>(roll_deg_), busy_.load() ? "true" : "false",
             (servos_ != nullptr && servos_->IsReady()) ? "true" : "false",
             (servos_ != nullptr && servos_->HasHardware()) ? "true" : "false");
    return std::string(buffer);
}

void GaitEngine::RegisterMcpTools() {
    auto& mcp = McpServer::GetInstance();

    mcp.AddTool("self.gait.stand",
                "Stand up and hold the default posture so the robot can walk. Call before "
                "any walk/turn command. Optional height_mm (0 = default).",
                PropertyList({Property("height_mm", kPropertyTypeInteger, 0, 0,
                                       static_cast<int>(BODY_MAX_HEIGHT_MM))}),
                [this](const PropertyList& properties) -> ReturnValue {
                    const int height = properties["height_mm"].value<int>();
                    return EnqueueStand(height) ? std::string("standing")
                                                : std::string("error: busy");
                });

    mcp.AddTool("self.gait.sit",
                "Fold the legs and lower the body into a resting sit posture (still "
                "holding torque). Use when the user asks the robot to sit down.",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    return EnqueueSit() ? std::string("sitting") : std::string("error: busy");
                });

    mcp.AddTool("self.gait.walk",
                "Crawl-gait walk. direction: 1 = forward, -1 = backward. steps 1-8, "
                "stride_mm 10-60, step_ms is the time per leg cycle (larger = slower and "
                "gentler on the power rail). One leg moves at a time, so the robot stays "
                "statically stable.",
                PropertyList({Property("direction", kPropertyTypeInteger, 1, -1, 1),
                              Property("steps", kPropertyTypeInteger, 1, 1, 8),
                              Property("stride_mm", kPropertyTypeInteger, 40, 10, 60),
                              Property("step_ms", kPropertyTypeInteger, 420, 200, 1500)}),
                [this](const PropertyList& properties) -> ReturnValue {
                    const int dir = properties["direction"].value<int>();
                    const int steps = properties["steps"].value<int>();
                    const int stride = properties["stride_mm"].value<int>();
                    const int step_ms = properties["step_ms"].value<int>();
                    return EnqueueWalk(std::to_string(dir), steps, stride, step_ms)
                               ? std::string("walk started")
                               : std::string("error: busy with another motion");
                });

    mcp.AddTool("self.gait.turn",
                "Turn in place with a differential crawl stride. direction: 1 = one way, "
                "-1 = the other. steps 1-8, step_ms per leg cycle.",
                PropertyList({Property("direction", kPropertyTypeInteger, 1, -1, 1),
                              Property("steps", kPropertyTypeInteger, 1, 1, 8),
                              Property("step_ms", kPropertyTypeInteger, 420, 200, 1500)}),
                [this](const PropertyList& properties) -> ReturnValue {
                    const int dir = properties["direction"].value<int>();
                    const int steps = properties["steps"].value<int>();
                    const int step_ms = properties["step_ms"].value<int>();
                    return EnqueueTurn(std::to_string(dir), steps, step_ms)
                               ? std::string("turn started")
                               : std::string("error: busy with another motion");
                });

    mcp.AddTool(
        "self.gait.body",
        "Adjust the body posture while standing: height_mm (70-120), pitch_deg "
        "(-20..20, nose up/down), roll_deg (-20..20, lean left/right). Use for "
        "gestures, looking up/down, or leaning.",
        PropertyList(
            {Property("height_mm", kPropertyTypeInteger, static_cast<int>(BODY_STAND_HEIGHT_MM),
                      static_cast<int>(BODY_MIN_HEIGHT_MM), static_cast<int>(BODY_MAX_HEIGHT_MM)),
             Property("pitch_deg", kPropertyTypeInteger, 0, -20, 20),
             Property("roll_deg", kPropertyTypeInteger, 0, -20, 20)}),
        [this](const PropertyList& properties) -> ReturnValue {
            return EnqueueBody(properties["height_mm"].value<int>(),
                               properties["pitch_deg"].value<int>(),
                               properties["roll_deg"].value<int>())
                       ? std::string("posture updated")
                       : std::string("error: busy");
        });

    mcp.AddTool("self.gait.stop",
                "Stop any gait immediately and release servo torque (safety stop). Use "
                "for: dừng lại, đứng im, stop moving.",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    EnqueueStop();
                    return std::string("gait stopped, servos relaxed");
                });

    mcp.AddTool("self.gait.status",
                "Report the gait state, body posture, and whether the servo driver and "
                "PCA9685 hardware are ready (debug).",
                PropertyList(),
                [this](const PropertyList&) -> ReturnValue { return StatusJson(); });

    mcp.AddTool("self.gait.leg_test",
                "Bring-up/calibration: move ONE leg by inverse kinematics while the other three "
                "stand. Both servos of that leg (hip = joint leg*2, knee = leg*2+1) are solved "
                "together. leg 0=front-left, 1=front-right, 2=rear-left, 3=rear-right; "
                "foot_x_mm is fore/aft from the hip, foot_z_mm is the vertical reach (40-150).",
                PropertyList({Property("leg", kPropertyTypeInteger, 0, 0, 3),
                              Property("foot_x_mm", kPropertyTypeInteger, 0, -60, 60),
                              Property("foot_z_mm", kPropertyTypeInteger,
                                       static_cast<int>(BODY_STAND_HEIGHT_MM), 40, 150)}),
                [this](const PropertyList& properties) -> ReturnValue {
                    const int leg = properties["leg"].value<int>();
                    const int foot_x = properties["foot_x_mm"].value<int>();
                    const int foot_z = properties["foot_z_mm"].value<int>();
                    if (!EnqueueLegTest(leg, foot_x, foot_z)) {
                        return std::string("error: queue full");
                    }
                    char buf[160];
                    snprintf(buf, sizeof(buf),
                             "{\"ok\":true,\"leg\":%d,\"hip_joint\":%d,\"knee_joint\":%d,"
                             "\"foot_x_mm\":%d,\"foot_z_mm\":%d}",
                             leg, leg * 2, leg * 2 + 1, foot_x, foot_z);
                    return std::string(buf);
                });

    mcp.AddTool("self.gait.leg_sweep",
                "Bring-up/calibration: slowly sweep ONE leg's servos in JOINT space (no inverse "
                "kinematics, so the full travel is used) while the other three hold. The hip "
                "sweeps +/- hip_deg/2 around neutral and the knee folds by knee_deg, then both "
                "return to neutral. leg 0=front-left, 1=front-right, 2=rear-left, 3=rear-right. "
                "Example: leg=0 hip_deg=120 knee_deg=60 duration_ms=3000.",
                PropertyList({Property("leg", kPropertyTypeInteger, 0, 0, 3),
                              Property("hip_deg", kPropertyTypeInteger, 120, 10, 170),
                              Property("knee_deg", kPropertyTypeInteger, 60, 0, 90),
                              Property("duration_ms", kPropertyTypeInteger, 3000, 1000, 15000)}),
                [this](const PropertyList& properties) -> ReturnValue {
                    const int leg = properties["leg"].value<int>();
                    const int hip_deg = properties["hip_deg"].value<int>();
                    const int knee_deg = properties["knee_deg"].value<int>();
                    const int duration_ms = properties["duration_ms"].value<int>();
                    if (!EnqueueLegSweep(leg, hip_deg, knee_deg, duration_ms)) {
                        return std::string("error: queue full");
                    }
                    char buf[160];
                    snprintf(buf, sizeof(buf),
                             "{\"ok\":true,\"leg\":%d,\"hip_joint\":%d,\"knee_joint\":%d,"
                             "\"hip_deg\":%d,\"knee_deg\":%d,\"duration_ms\":%d}",
                             leg, leg * 2, leg * 2 + 1, hip_deg, knee_deg, duration_ms);
                    return std::string(buf);
                });
}
