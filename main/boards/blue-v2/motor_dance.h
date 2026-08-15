#ifndef BLUE_V2_MOTOR_DANCE_H_
#define BLUE_V2_MOTOR_DANCE_H_

#include <atomic>
#include <cstdint>
#include <functional>

class MotorController;

namespace MotorDance {

enum class ActionId : uint8_t {
    Sway = 0,
    Surge,
    SpinBurst,
    Glide,
    AltTurn,
    ForwardPulse,
    Count,
};

enum class MusicState : uint8_t {
    Chill = 0,
    Groove,
    Drive,
    Drop,
    Flow,
    Count,
};

using MusicActiveFn = std::function<bool()>;
using MusicStateMask = uint8_t;

/** Compact EQ timeline: one char per segment (c/g/v/D/f), max 64 segments. */
struct DanceTimeline {
    static constexpr int kMaxLen = 64;
    char chars[kMaxLen + 1] = {};
    int len = 0;
    uint16_t segment_ms = 6000;
};

MusicState ParseMusicState(const char* name);
MusicState ParseTimelineChar(char ch);
MusicStateMask ParseMusicStateMask(const char* csv, MusicState primary);
MusicStateMask MaskForPrimary(MusicState primary);

bool ParseDanceTimeline(const char* compact, uint16_t segment_ms, DanceTimeline* out);
void SegmentAtElapsed(const DanceTimeline& timeline, int64_t elapsed_ms, MusicState* primary,
                      MusicStateMask* mask);

ActionId PickRandomAction(ActionId previous);
ActionId PickActionForMusicState(MusicStateMask mask, MusicState primary, ActionId previous);

bool RunAction(MotorController& motor, std::atomic<bool>& cancel, ActionId id);

void RunDanceSession(MotorController& motor, std::atomic<bool>& cancel, MusicActiveFn music_active,
                     MusicStateMask mood_mask = 0, MusicState primary = MusicState::Groove,
                     const DanceTimeline* timeline = nullptr);

}  // namespace MotorDance

#endif  // BLUE_V2_MOTOR_DANCE_H_
