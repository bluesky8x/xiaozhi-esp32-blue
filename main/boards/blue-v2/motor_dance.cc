#include "motor_dance.h"

#include "application.h"
#include "motor_controller.h"

#include <cctype>
#include <cstring>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

constexpr int8_t kFwd = 92;
constexpr int8_t kBack = -88;
constexpr int8_t kSpin = 94;
constexpr int8_t kSway = 90;
constexpr int8_t kGlideFast = 95;
constexpr int8_t kGlideSlow = -60;

constexpr int kMusicStartWaitMs = 15000;
constexpr int kPollMs = 80;

static const char* kTag = "MotorDance";

const char* LogMusicStateName(MotorDance::MusicState state) {
    switch (state) {
        case MotorDance::MusicState::Chill:
            return "chill";
        case MotorDance::MusicState::Groove:
            return "groove";
        case MotorDance::MusicState::Drive:
            return "drive";
        case MotorDance::MusicState::Drop:
            return "drop";
        case MotorDance::MusicState::Flow:
            return "flow";
        default:
            return "groove";
    }
}

const char* LogActionName(MotorDance::ActionId id) {
    switch (id) {
        case MotorDance::ActionId::Sway:
            return "sway";
        case MotorDance::ActionId::Surge:
            return "surge";
        case MotorDance::ActionId::SpinBurst:
            return "spin_burst";
        case MotorDance::ActionId::Glide:
            return "glide";
        case MotorDance::ActionId::AltTurn:
            return "alt_turn";
        case MotorDance::ActionId::ForwardPulse:
            return "forward_pulse";
        default:
            return "?";
    }
}

int RandRange(int lo, int hi) {
    if (hi <= lo) {
        return lo;
    }
    return lo + static_cast<int>(esp_random() % static_cast<uint32_t>(hi - lo + 1));
}

bool Step(MotorController& motor, std::atomic<bool>& cancel, int8_t left, int8_t right,
          int16_t duration_ms, int16_t pause_ms = 70, bool tof_guard = false) {
    if (cancel.load(std::memory_order_acquire)) {
        return false;
    }
    if (!motor.DriveForMsWithCancel(left, right, duration_ms, cancel, tof_guard)) {
        return false;
    }
    if (pause_ms > 0 && !cancel.load(std::memory_order_acquire)) {
        vTaskDelay(pdMS_TO_TICKS(pause_ms));
    }
    return !cancel.load(std::memory_order_acquire);
}

bool StepGuarded(MotorController& motor, std::atomic<bool>& cancel, int8_t left, int8_t right,
                 int16_t duration_ms, int16_t pause_ms = 70) {
    const bool tof = !(left < 0 && right < 0);
    return Step(motor, cancel, left, right, duration_ms, pause_ms, tof);
}

void Alt(MotorController& motor, std::atomic<bool>& cancel, int pairs, int8_t left, int8_t right,
         int16_t duration_ms, int16_t pause_ms = 70) {
    const bool tof = !(left < 0 && right < 0);
    for (int i = 0; i < pairs && !cancel.load(std::memory_order_acquire); ++i) {
        if (!Step(motor, cancel, left, right, duration_ms, pause_ms, tof)) {
            return;
        }
        if (!Step(motor, cancel, right, left, duration_ms, pause_ms, tof)) {
            return;
        }
    }
}

void Sway(MotorController& motor, std::atomic<bool>& cancel, int pairs, int16_t duration_ms,
          int16_t pause_ms = 55) {
    Alt(motor, cancel, pairs, -kSway, kSway, duration_ms, pause_ms);
}

void Surge(MotorController& motor, std::atomic<bool>& cancel, int reps, int16_t fwd_ms,
           int16_t back_ms, int16_t pause_ms = 65) {
    for (int i = 0; i < reps && !cancel.load(std::memory_order_acquire); ++i) {
        if (!StepGuarded(motor, cancel, kFwd, kFwd, fwd_ms, pause_ms)) {
            return;
        }
        if (!Step(motor, cancel, kBack, kBack, back_ms, pause_ms, false)) {
            return;
        }
    }
}

void SpinBurst(MotorController& motor, std::atomic<bool>& cancel, int count, int16_t duration_ms,
               int16_t pause_ms = 100) {
    for (int i = 0; i < count && !cancel.load(std::memory_order_acquire); ++i) {
        if (!StepGuarded(motor, cancel, -kSpin, kSpin, duration_ms, pause_ms)) {
            return;
        }
    }
}

void Glide(MotorController& motor, std::atomic<bool>& cancel, int pairs, int16_t duration_ms,
           int16_t pause_ms = 75) {
    for (int i = 0; i < pairs && !cancel.load(std::memory_order_acquire); ++i) {
        if (!StepGuarded(motor, cancel, kGlideFast, kGlideSlow, duration_ms, pause_ms)) {
            return;
        }
        if (!StepGuarded(motor, cancel, kGlideSlow, kGlideFast, duration_ms, pause_ms)) {
            return;
        }
    }
}

}  // namespace

namespace MotorDance {

namespace {

MusicStateMask DefaultMoodMask() {
    return (1u << static_cast<uint8_t>(MusicState::Groove)) |
           (1u << static_cast<uint8_t>(MusicState::Flow));
}

bool MaskHas(MusicStateMask mask, MusicState state) {
    return (mask & (1u << static_cast<uint8_t>(state))) != 0;
}

MusicStateMask MaskOrDefault(MusicStateMask mask) {
    return mask != 0 ? mask : DefaultMoodMask();
}

void AppendUniqueAction(ActionId id, ActionId* out, int& count, int cap) {
    for (int i = 0; i < count; ++i) {
        if (out[i] == id) {
            return;
        }
    }
    if (count < cap) {
        out[count++] = id;
    }
}

int ActionsForState(MusicState state, ActionId* out, int cap) {
    int count = 0;
    switch (state) {
        case MusicState::Chill:
            AppendUniqueAction(ActionId::Sway, out, count, cap);
            AppendUniqueAction(ActionId::Glide, out, count, cap);
            break;
        case MusicState::Groove:
            AppendUniqueAction(ActionId::AltTurn, out, count, cap);
            AppendUniqueAction(ActionId::Sway, out, count, cap);
            AppendUniqueAction(ActionId::Glide, out, count, cap);
            break;
        case MusicState::Drive:
            AppendUniqueAction(ActionId::ForwardPulse, out, count, cap);
            AppendUniqueAction(ActionId::Surge, out, count, cap);
            break;
        case MusicState::Drop:
            AppendUniqueAction(ActionId::SpinBurst, out, count, cap);
            AppendUniqueAction(ActionId::Surge, out, count, cap);
            AppendUniqueAction(ActionId::ForwardPulse, out, count, cap);
            break;
        case MusicState::Flow:
            AppendUniqueAction(ActionId::Glide, out, count, cap);
            AppendUniqueAction(ActionId::Sway, out, count, cap);
            break;
        default:
            break;
    }
    return count;
}

bool TokenEq(const char* token, const char* word) {
    if (token == nullptr || word == nullptr) {
        return false;
    }
    while (*token && *word) {
        if (std::tolower(static_cast<unsigned char>(*token)) !=
            std::tolower(static_cast<unsigned char>(*word))) {
            return false;
        }
        ++token;
        ++word;
    }
    return *token == '\0' && *word == '\0';
}

}  // namespace

MusicState ParseTimelineChar(char ch) {
    switch (ch) {
        case 'c':
            return MusicState::Chill;
        case 'g':
            return MusicState::Groove;
        case 'v':
            return MusicState::Drive;
        case 'D':
            return MusicState::Drop;
        case 'f':
            return MusicState::Flow;
        default:
            return MusicState::Groove;
    }
}

MusicStateMask MaskForPrimary(MusicState primary) {
    switch (primary) {
        case MusicState::Chill:
            return (1u << static_cast<uint8_t>(MusicState::Chill)) |
                   (1u << static_cast<uint8_t>(MusicState::Flow));
        case MusicState::Groove:
            return (1u << static_cast<uint8_t>(MusicState::Groove)) |
                   (1u << static_cast<uint8_t>(MusicState::Flow));
        case MusicState::Drive:
            return (1u << static_cast<uint8_t>(MusicState::Drive)) |
                   (1u << static_cast<uint8_t>(MusicState::Groove));
        case MusicState::Drop:
            return (1u << static_cast<uint8_t>(MusicState::Drop)) |
                   (1u << static_cast<uint8_t>(MusicState::Drive));
        case MusicState::Flow:
            return (1u << static_cast<uint8_t>(MusicState::Flow)) |
                   (1u << static_cast<uint8_t>(MusicState::Groove));
        default:
            return DefaultMoodMask();
    }
}

bool ParseDanceTimeline(const char* compact, uint16_t segment_ms, DanceTimeline* out) {
    if (out == nullptr) {
        return false;
    }
    *out = DanceTimeline{};
    if (compact == nullptr || compact[0] == '\0') {
        return false;
    }
    int n = 0;
    for (const char* p = compact; *p != '\0' && n < DanceTimeline::kMaxLen; ++p) {
        if (*p == '|' || *p == ',' || *p == ' ') {
            continue;
        }
        out->chars[n++] = *p;
    }
    out->chars[n] = '\0';
    out->len = n;
    out->segment_ms = segment_ms > 0 ? segment_ms : 6000;
    return n > 0;
}

void SegmentAtElapsed(const DanceTimeline& timeline, int64_t elapsed_ms, MusicState* primary,
                      MusicStateMask* mask) {
    if (primary == nullptr || mask == nullptr) {
        return;
    }
    if (timeline.len <= 0 || timeline.segment_ms == 0) {
        *primary = MusicState::Groove;
        *mask = DefaultMoodMask();
        return;
    }
    const int64_t seg_ms = timeline.segment_ms;
    int idx = static_cast<int>(elapsed_ms / seg_ms);
    if (idx < 0) {
        idx = 0;
    }
    if (idx >= timeline.len) {
        idx = timeline.len - 1;
    }
    *primary = ParseTimelineChar(timeline.chars[idx]);
    *mask = MaskForPrimary(*primary);
}

MusicState ParseMusicState(const char* name) {
    if (name == nullptr || name[0] == '\0') {
        return MusicState::Groove;
    }
    if (TokenEq(name, "chill")) {
        return MusicState::Chill;
    }
    if (TokenEq(name, "groove")) {
        return MusicState::Groove;
    }
    if (TokenEq(name, "drive")) {
        return MusicState::Drive;
    }
    if (TokenEq(name, "drop")) {
        return MusicState::Drop;
    }
    if (TokenEq(name, "flow")) {
        return MusicState::Flow;
    }
    return MusicState::Groove;
}

MusicStateMask ParseMusicStateMask(const char* csv, MusicState primary) {
    MusicStateMask mask = 0;
    if (csv == nullptr || csv[0] == '\0') {
        mask = 1u << static_cast<uint8_t>(primary);
        return MaskOrDefault(mask);
    }
    const char* p = csv;
    while (*p != '\0') {
        while (*p == ' ' || *p == ',') {
            ++p;
        }
        if (*p == '\0') {
            break;
        }
        const char* start = p;
        while (*p != '\0' && *p != ',') {
            ++p;
        }
        char token[16] = {};
        const int len = static_cast<int>(p - start);
        if (len > 0 && len < static_cast<int>(sizeof(token))) {
            std::memcpy(token, start, len);
            const MusicState state = ParseMusicState(token);
            mask |= 1u << static_cast<uint8_t>(state);
        }
    }
    if (mask == 0) {
        mask = 1u << static_cast<uint8_t>(primary);
    }
    return mask;
}

ActionId PickActionForMusicState(MusicStateMask mask, MusicState primary, ActionId previous) {
    mask = MaskOrDefault(mask);
    ActionId pool[8];
    int count = 0;

    // Primary state actions get double weight in the pool.
    if (MaskHas(mask, primary)) {
        ActionId primary_pool[4];
        const int n = ActionsForState(primary, primary_pool, 4);
        for (int i = 0; i < n; ++i) {
            AppendUniqueAction(primary_pool[i], pool, count, 8);
            AppendUniqueAction(primary_pool[i], pool, count, 8);
        }
    }
    for (uint8_t i = 0; i < static_cast<uint8_t>(MusicState::Count); ++i) {
        const auto state = static_cast<MusicState>(i);
        if (!MaskHas(mask, state) || state == primary) {
            continue;
        }
        ActionId local[4];
        const int n = ActionsForState(state, local, 4);
        for (int j = 0; j < n; ++j) {
            AppendUniqueAction(local[j], pool, count, 8);
        }
    }
    if (count <= 0) {
        return PickRandomAction(previous);
    }
    ActionId pick = pool[esp_random() % static_cast<uint32_t>(count)];
    if (pick == previous && count > 1) {
        pick = pool[(esp_random() % static_cast<uint32_t>(count - 1) + 1) % count];
    }
    return pick;
}

ActionId PickRandomAction(ActionId previous) {
    const auto count = static_cast<uint8_t>(ActionId::Count);
    if (count <= 1) {
        return ActionId::Sway;
    }
    ActionId next = static_cast<ActionId>(esp_random() % count);
    if (next == previous) {
        next = static_cast<ActionId>((static_cast<uint8_t>(next) + 1 + esp_random() % (count - 1)) %
                                       count);
    }
    return next;
}

bool RunAction(MotorController& motor, std::atomic<bool>& cancel, ActionId id) {
    switch (id) {
        case ActionId::Sway:
            Sway(motor, cancel, RandRange(2, 5), RandRange(280, 420), RandRange(45, 70));
            break;
        case ActionId::Surge:
            Surge(motor, cancel, RandRange(2, 4), RandRange(520, 760), RandRange(480, 680));
            break;
        case ActionId::SpinBurst:
            SpinBurst(motor, cancel, RandRange(1, 3), RandRange(850, 1200), RandRange(80, 130));
            break;
        case ActionId::Glide:
            Glide(motor, cancel, RandRange(2, 4), RandRange(420, 560));
            break;
        case ActionId::AltTurn:
            Alt(motor, cancel, RandRange(3, 6), -88, 88, RandRange(220, 300), RandRange(40, 65));
            break;
        case ActionId::ForwardPulse:
            for (int i = 0; i < RandRange(2, 4) && !cancel.load(std::memory_order_acquire); ++i) {
                if (!StepGuarded(motor, cancel, 86, 86, RandRange(320, 480), RandRange(50, 70))) {
                    return false;
                }
            }
            break;
        default:
            break;
    }
    return !cancel.load(std::memory_order_acquire);
}

void RunDanceSession(MotorController& motor, std::atomic<bool>& cancel, MusicActiveFn music_active,
                     MusicStateMask mood_mask, MusicState primary, const DanceTimeline* timeline) {
    if (!music_active) {
        return;
    }

    int waited_ms = 0;
    int64_t dance_start_us = 0;
    while (!cancel.load(std::memory_order_acquire) && waited_ms < kMusicStartWaitMs) {
        if (music_active()) {
            dance_start_us = esp_timer_get_time();
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(kPollMs));
        waited_ms += kPollMs;
    }

    const bool use_timeline = timeline != nullptr && timeline->len > 0;
    ActionId last = ActionId::Count;
    int last_seg_idx = -1;

    if (use_timeline) {
        ESP_LOGI(kTag, "EQ timeline: %s (%d x %ums)", timeline->chars, timeline->len,
                 static_cast<unsigned>(timeline->segment_ms));
    } else {
        ESP_LOGI(kTag, "EQ static mood=%s (no timeline)", LogMusicStateName(primary));
        Application::GetInstance().UpdateDanceMusicStateDisplay(LogMusicStateName(primary));
    }

    while (!cancel.load(std::memory_order_acquire) && music_active()) {
        if (dance_start_us == 0) {
            dance_start_us = esp_timer_get_time();
        }
        MusicStateMask active_mask = mood_mask;
        MusicState active_primary = primary;
        int seg_idx = 0;
        const int64_t elapsed_ms = (esp_timer_get_time() - dance_start_us) / 1000;
        if (use_timeline) {
            seg_idx = static_cast<int>(elapsed_ms / timeline->segment_ms);
            if (seg_idx >= timeline->len) {
                seg_idx = timeline->len - 1;
            }
            SegmentAtElapsed(*timeline, elapsed_ms, &active_primary, &active_mask);
            if (seg_idx != last_seg_idx) {
                ESP_LOGI(kTag,
                         "music state seg %d/%d @ %lldms -> %s ('%c') mask=0x%02x",
                         seg_idx + 1, timeline->len, static_cast<long long>(elapsed_ms),
                         LogMusicStateName(active_primary), timeline->chars[seg_idx],
                         static_cast<unsigned>(active_mask));
                Application::GetInstance().UpdateDanceMusicStateDisplay(
                    LogMusicStateName(active_primary));
                last_seg_idx = seg_idx;
            }
        }
        const ActionId next = PickActionForMusicState(active_mask, active_primary, last);
        ESP_LOGI(kTag, "dance action -> %s (mood=%s elapsed=%lldms)", LogActionName(next),
                 LogMusicStateName(active_primary), static_cast<long long>(elapsed_ms));
        last = next;
        if (!RunAction(motor, cancel, next)) {
            break;
        }
    }
}

}  // namespace MotorDance
