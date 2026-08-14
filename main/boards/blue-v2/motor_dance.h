#ifndef BLUE_V2_MOTOR_DANCE_H_
#define BLUE_V2_MOTOR_DANCE_H_

#include <atomic>

class MotorController;

namespace MotorDance {

/** Slap-house routine (~22 s) for dance1.ogg */
void RunTrack1(MotorController& motor, std::atomic<bool>& cancel);

/** Hip-hop routine (~99 s) for dance2.ogg */
void RunTrack2(MotorController& motor, std::atomic<bool>& cancel);

/** Orchestral drill routine (~25 s) for dance3.ogg */
void RunTrack3(MotorController& motor, std::atomic<bool>& cancel);

}  // namespace MotorDance

#endif  // BLUE_V2_MOTOR_DANCE_H_
