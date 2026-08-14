#include "motor_dance.h"

#include "motor_controller.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

bool Step(MotorController& motor, std::atomic<bool>& cancel, int8_t left, int8_t right,
          int16_t duration_ms, int16_t pause_ms = 100) {
    if (cancel.load(std::memory_order_acquire)) {
        return false;
    }
    if (!motor.DriveForMsWithCancel(left, right, duration_ms, cancel)) {
        return false;
    }
    if (pause_ms > 0 && !cancel.load(std::memory_order_acquire)) {
        vTaskDelay(pdMS_TO_TICKS(pause_ms));
    }
    return !cancel.load(std::memory_order_acquire);
}

void Alt(MotorController& motor, std::atomic<bool>& cancel, int pairs, int8_t left, int8_t right,
         int16_t duration_ms, int16_t pause_ms = 100) {
    for (int i = 0; i < pairs && !cancel.load(std::memory_order_acquire); ++i) {
        if (!Step(motor, cancel, left, right, duration_ms, pause_ms)) {
            return;
        }
        if (!Step(motor, cancel, right, left, duration_ms, pause_ms)) {
            return;
        }
    }
}

}  // namespace

namespace MotorDance {

void RunTrack1(MotorController& motor, std::atomic<bool>& cancel) {
    struct StepDef {
        int8_t left;
        int8_t right;
        int16_t duration_ms;
        int16_t pause_ms;
    };

    static constexpr StepDef kRoutine[] = {
        {-72, 72, 380, 120}, {72, -72, 380, 120}, {-72, 72, 380, 120}, {72, -72, 380, 120},
        {-72, 72, 380, 120}, {72, -72, 380, 120}, {-72, 72, 380, 120}, {72, -72, 380, 120},
        {-72, 72, 380, 120}, {72, -72, 380, 120},
        {68, 68, 420, 80},   {-68, -68, 420, 80},  {68, 68, 420, 80},   {-68, -68, 420, 80},
        {68, 68, 420, 80},   {-68, -68, 420, 80},  {68, 68, 420, 80},   {-68, -68, 420, 80},
        {68, 68, 420, 80},   {-68, -68, 420, 80},
        {-78, 78, 280, 70},  {78, -78, 280, 70},   {-78, 78, 280, 70},  {78, -78, 280, 70},
        {-78, 78, 280, 70},  {78, -78, 280, 70},
        {-85, 85, 850, 150}, {-85, 85, 850, 150}, {-85, 85, 850, 150}, {-85, 85, 850, 150},
        {-85, 85, 850, 150},
        {75, 75, 350, 100},  {75, 75, 350, 100},   {75, 75, 350, 100},  {75, 75, 350, 100},
        {-90, 90, 1200, 0},
    };

    for (const StepDef& s : kRoutine) {
        if (!Step(motor, cancel, s.left, s.right, s.duration_ms, s.pause_ms)) {
            return;
        }
    }
}

void RunTrack2(MotorController& motor, std::atomic<bool>& cancel) {
    // Hip-hop ~99 s — assets/common/dance2.ogg (Ice Heart instrumental).

    // Intro — slow head-nod sway (~16 s)
    Alt(motor, cancel, 7, -68, 68, 480, 130);

    // Verse — stomp forward / back (~18 s)
    for (int i = 0; i < 8 && !cancel.load(std::memory_order_acquire); ++i) {
        if (!Step(motor, cancel, 72, 72, 420, 90)) {
            return;
        }
        if (!Step(motor, cancel, -72, -72, 420, 90)) {
            return;
        }
    }

    // Side glide — asymmetric wheels (~14 s)
    for (int i = 0; i < 7 && !cancel.load(std::memory_order_acquire); ++i) {
        if (!Step(motor, cancel, 80, -55, 400, 100)) {
            return;
        }
        if (!Step(motor, cancel, -55, 80, 400, 100)) {
            return;
        }
    }

    // Hook — quick bounce (~16 s)
    Alt(motor, cancel, 8, -80, 80, 320, 80);

    // Chorus — power spins (~18 s)
    for (int i = 0; i < 6 && !cancel.load(std::memory_order_acquire); ++i) {
        if (!Step(motor, cancel, -88, 88, 950, 140)) {
            return;
        }
    }

    // Bridge — double-time wiggle (~14 s)
    Alt(motor, cancel, 10, -75, 75, 260, 60);

    // Break — pulse forward (~12 s)
    for (int i = 0; i < 10 && !cancel.load(std::memory_order_acquire); ++i) {
        if (!Step(motor, cancel, 70, 70, 340, 70)) {
            return;
        }
    }

    // Build — back-it-up (~10 s)
    for (int i = 0; i < 5 && !cancel.load(std::memory_order_acquire); ++i) {
        if (!Step(motor, cancel, -70, -70, 500, 100)) {
            return;
        }
        if (!Step(motor, cancel, 65, 65, 380, 100)) {
            return;
        }
    }

    // Final drops — spins + sway outro (~21 s)
    for (int i = 0; i < 4 && !cancel.load(std::memory_order_acquire); ++i) {
        if (!Step(motor, cancel, -90, 90, 1100, 160)) {
            return;
        }
    }
    Alt(motor, cancel, 4, -65, 65, 450, 120);
    Step(motor, cancel, -85, 85, 1400, 0);
}

void RunTrack3(MotorController& motor, std::atomic<bool>& cancel) {
    // Orchestral drill ~25 s — assets/common/dance3.ogg (Old Pirate drill).

    // Intro — sharp drill pivots (~5 s)
    Alt(motor, cancel, 7, -88, 88, 260, 45);

    // March stomp — forward / retreat (~7 s)
    for (int i = 0; i < 7 && !cancel.load(std::memory_order_acquire); ++i) {
        if (!Step(motor, cancel, 78, 78, 360, 70)) {
            return;
        }
        if (!Step(motor, cancel, -78, -78, 360, 70)) {
            return;
        }
    }

    // Pirate swagger — asymmetric glide (~5 s)
    for (int i = 0; i < 5 && !cancel.load(std::memory_order_acquire); ++i) {
        if (!Step(motor, cancel, 92, -58, 400, 85)) {
            return;
        }
        if (!Step(motor, cancel, -58, 92, 400, 85)) {
            return;
        }
    }

    // Build — rapid drill fire (~4 s)
    Alt(motor, cancel, 8, -82, 82, 210, 35);

    // Drop — power spins (~4 s)
    for (int i = 0; i < 4 && !cancel.load(std::memory_order_acquire); ++i) {
        if (!Step(motor, cancel, -92, 92, 780, 110)) {
            return;
        }
    }

    // Finale — charge + victory spin (~3 s)
    for (int i = 0; i < 3 && !cancel.load(std::memory_order_acquire); ++i) {
        if (!Step(motor, cancel, 82, 82, 300, 60)) {
            return;
        }
    }
    Step(motor, cancel, -95, 95, 1200, 0);
}

}  // namespace MotorDance
