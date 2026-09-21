#include "blue_v4_motor_compat.h"

#include "config.h"
#include "mcp_server.h"

#include <esp_log.h>
#include <algorithm>
#include <cstdlib>

#define TAG "BlueV4MotorCompat"

namespace {
// Matches blue-v2's defaults so the server-side mv:* mapping is untouched.
constexpr int kDefaultDurationMs = 5000;
constexpr int kDefaultStrideMm = 40;
// Crawl step time from wheel-speed magnitude: 100% -> 350 ms, 30% -> 1120 ms.
// Faster phases look smoother: an analogue MG90S has a ~1 deg deadband, so a slow sweep
// moves in visible "notches", while a fast one keeps the servo continuously in motion.
constexpr int kStepMsAtFullSpeed = 1450;
constexpr int kStepMsPerPercent = 11;

// Default gesture timelines when the server sends none (per dance track).
const char* kDefaultTimelines[3] = {"DgvD", "gDgvgDg", "vDggcD"};

int StepMsForSpeed(int magnitude) {
    return std::clamp(kStepMsAtFullSpeed - magnitude * kStepMsPerPercent, 300, 1500);
}

int StepsForDuration(int duration_ms, int step_ms) {
    // One crawl step = four legs, and every leg runs its phases sequentially (see
    // GaitEngine::JointCrawlCycleMs). Using `step_ms * 4` underestimated the real time by ~4x, so
    // a "5 s" move actually walked for ~12 s and the requested duration was never honoured.
    const int cycle_ms = std::max(GaitEngine::JointCrawlCycleMs(step_ms), 400);
    const int steps = (duration_ms + cycle_ms / 2) / cycle_ms;  // round to the nearest step
    return std::clamp(steps, 1, 12);
}
}  // namespace

bool BlueV4MotorCompat::StartMove(int left, int right, int duration_ms) {
    if (gait_ == nullptr) {
        return false;
    }
    if (left == 0 && right == 0) {
        return gait_->EnqueueStop();
    }

    const int magnitude = std::max(std::abs(left), std::abs(right));
    const int step_ms = StepMsForSpeed(magnitude);
    const int steps = StepsForDuration(duration_ms, step_ms);

    // left forward + right backward = counter-clockwise = left turn (and vice versa).
    if (left > 0 && right < 0) {
        return gait_->EnqueueTurn("-1", steps, step_ms);
    }
    if (left < 0 && right > 0) {
        return gait_->EnqueueTurn("1", steps, step_ms);
    }
    return gait_->EnqueueWalk(left > 0 ? "1" : "-1", steps, kDefaultStrideMm, step_ms);
}

bool BlueV4MotorCompat::StartDance(int track, const std::string& mood, const std::string& timeline,
                                   int segment_ms) {
    if (gait_ == nullptr) {
        return false;
    }
    const int clamped_track = std::clamp(track, 1, 3);
    const char* gestures =
        timeline.empty() ? kDefaultTimelines[clamped_track - 1] : timeline.c_str();
    ESP_LOGI(TAG, "dance track=%d mood=%s segment_ms=%d gestures=%s", clamped_track, mood.c_str(),
             segment_ms, gestures);
    return gait_->EnqueueDance(segment_ms, gestures);
}

void BlueV4MotorCompat::RegisterMcpTools() {
    auto& mcp = McpServer::GetInstance();

    mcp.AddTool("self.motor.stop", "Stop moving and relax the legs. Use for: dừng, dừng lại, stop.",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    return gait_ != nullptr && gait_->EnqueueStop() ? std::string("true")
                                                                    : std::string("false");
                });

    mcp.AddTool("self.motor.forward",
                "Walk forward at full speed (one short crawl burst). Use for: đi tới, tiến lên, "
                "forward.",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    return StartMove(100, 100, kDefaultDurationMs) ? std::string("true")
                                                                   : std::string("false");
                });

    mcp.AddTool("self.motor.backward",
                "Walk backward (one short crawl burst). Use for: đi lùi, lùi lại, backward.",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    return StartMove(-100, -100, kDefaultDurationMs) ? std::string("true")
                                                                     : std::string("false");
                });

    mcp.AddTool("self.motor.turn_left",
                "Turn in place to the left. Use for: quay trái, rẽ trái, turn left.",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    return StartMove(70, -70, kDefaultDurationMs) ? std::string("true")
                                                                  : std::string("false");
                });

    mcp.AddTool("self.motor.turn_right",
                "Turn in place to the right. Use for: quay phải, rẽ phải, turn right.",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    return StartMove(-70, 70, kDefaultDurationMs) ? std::string("true")
                                                                  : std::string("false");
                });

    mcp.AddTool(
        "self.motor.circle",
        "Walk in a circle/arc (forward with a steady turn). Optional duration_ms.",
        PropertyList(
            {Property("duration_ms", kPropertyTypeInteger, kDefaultDurationMs, 1000, 30000)}),
        [this](const PropertyList& properties) -> ReturnValue {
            const int duration_ms = properties["duration_ms"].value<int>();
            return StartMove(50, 100, duration_ms) ? std::string("true") : std::string("false");
        });

    mcp.AddTool(
        "self.motor.move",
        "Drive with independent left/right speeds (-100..100). Positive = forward. "
        "left=right>0 forward, left=right<0 backward, left>0/right<0 turn left, "
        "left<0/right>0 turn right, both 0 = stop. duration_ms is the run time.",
        PropertyList(
            {Property("left", kPropertyTypeInteger, 0, -100, 100),
             Property("right", kPropertyTypeInteger, 0, -100, 100),
             Property("duration_ms", kPropertyTypeInteger, kDefaultDurationMs, 100, 30000)}),
        [this](const PropertyList& properties) -> ReturnValue {
            const int left = properties["left"].value<int>();
            const int right = properties["right"].value<int>();
            const int duration_ms = properties["duration_ms"].value<int>();
            return StartMove(left, right, duration_ms) ? std::string("true") : std::string("false");
        });

    mcp.AddTool("self.motor.dance",
                "Perform a dance with the legs (bounce, sway, leg wave, crouch). track 1-3 "
                "selects the gesture routine when the server sends no timeline; timeline is a "
                "string of gesture letters (D/g/v/c), one per segment of segment_ms.",
                PropertyList({Property("track", kPropertyTypeInteger, 1, 1, 3),
                              Property("mood", kPropertyTypeString, "groove"),
                              Property("states", kPropertyTypeString, ""),
                              Property("timeline", kPropertyTypeString, ""),
                              Property("segment_ms", kPropertyTypeInteger, 6000, 4000, 8000)}),
                [this](const PropertyList& properties) -> ReturnValue {
                    const int track = properties["track"].value<int>();
                    const std::string mood = properties["mood"].value<std::string>();
                    const std::string timeline = properties["timeline"].value<std::string>();
                    const int segment_ms = properties["segment_ms"].value<int>();
                    return StartDance(track, mood, timeline, segment_ms) ? std::string("true")
                                                                         : std::string("false");
                });

    ESP_LOGI(TAG, "blue-v2 compatible motor tools registered (8) — server needs no change");
}
