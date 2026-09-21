#ifndef _BLUE_V4_MOTOR_COMPAT_H_
#define _BLUE_V4_MOTOR_COMPAT_H_

#include "gait_engine.h"

#include <string>

// blue-v2 compatible MCP tool surface for the quadruped.
//
// The xiaozhi server maps its `mv:*` tags to device tools by NAME
// (core/utils/robot_move_codec.py: MOVE_CODE_TO_MCP). It tries `self.motor.*`
// first, so exposing the same tool names as blue-v2 lets the server drive the
// servos with ZERO server changes — the gait engine just interprets the same
// wheel-speed commands as leg motion.
//
// Wheel semantics translated here:
//   left>0 & right>0  -> walk forward        left<0 & right<0 -> walk backward
//   left>0 & right<0  -> turn left           left<0 & right>0 -> turn right
//   left==0 & right==0 -> stop
// Magnitude maps to gait speed (step_ms); duration_ms maps to step count.
class BlueV4MotorCompat {
public:
    explicit BlueV4MotorCompat(GaitEngine* gait) : gait_(gait) {}

    void RegisterMcpTools();

private:
    bool StartMove(int left, int right, int duration_ms);
    bool StartDance(int track, const std::string& mood, const std::string& timeline,
                    int segment_ms);

    GaitEngine* gait_ = nullptr;
};

#endif  // _BLUE_V4_MOTOR_COMPAT_H_
