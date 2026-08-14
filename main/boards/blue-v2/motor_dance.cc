#include "motor_dance.h"

#include "motor_controller.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

constexpr int8_t kFwd = 92;
constexpr int8_t kBack = -88;
constexpr int8_t kSpin = 94;
constexpr int8_t kSway = 90;
constexpr int8_t kGlideFast = 95;
constexpr int8_t kGlideSlow = -60;

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

void RunTrack1(MotorController& motor, std::atomic<bool>& cancel) {
    // Slap-house ~23 s — big bounce party.

    Sway(motor, cancel, 8, 340, 50);
    Surge(motor, cancel, 6, 720, 640);
    Alt(motor, cancel, 7, -88, 88, 250, 45);
    SpinBurst(motor, cancel, 4, 1100, 110);
    for (int i = 0; i < 4 && !cancel.load(std::memory_order_acquire); ++i) {
        if (!StepGuarded(motor, cancel, 88, 88, 380, 60)) {
            return;
        }
    }
    StepGuarded(motor, cancel, -kSpin, kSpin, 1350, 0);
}

void RunTrack2(MotorController& motor, std::atomic<bool>& cancel) {
    // Hip-hop ~99 s — long travel, dynamic sections.

    Sway(motor, cancel, 6, 520, 90);
    Surge(motor, cancel, 7, 780, 700);
    Glide(motor, cancel, 6, 540);
    Alt(motor, cancel, 10, -88, 88, 290, 65);
    SpinBurst(motor, cancel, 5, 1200, 130);
    Alt(motor, cancel, 12, -85, 85, 230, 50);
    for (int i = 0; i < 8 && !cancel.load(std::memory_order_acquire); ++i) {
        if (!StepGuarded(motor, cancel, 86, 86, 500, 55)) {
            return;
        }
    }
    Surge(motor, cancel, 6, 680, 620);
    SpinBurst(motor, cancel, 4, 1250, 140);
    Sway(motor, cancel, 5, 460, 100);
    StepGuarded(motor, cancel, -kSpin, kSpin, 1500, 0);
}

void RunTrack3(MotorController& motor, std::atomic<bool>& cancel) {
    // Orchestral drill ~25 s — sharp hits, long charge.

    Alt(motor, cancel, 8, -92, 92, 230, 40);
    Surge(motor, cancel, 6, 620, 560);
    Glide(motor, cancel, 5, 460, 70);
    Alt(motor, cancel, 9, -90, 90, 195, 30);
    SpinBurst(motor, cancel, 4, 900, 90);
    for (int i = 0; i < 3 && !cancel.load(std::memory_order_acquire); ++i) {
        if (!StepGuarded(motor, cancel, 90, 90, 450, 50)) {
            return;
        }
    }
    StepGuarded(motor, cancel, -kSpin, kSpin, 1300, 0);
}

}  // namespace MotorDance
