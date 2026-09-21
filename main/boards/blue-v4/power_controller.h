#ifndef _BLUE_V4_POWER_CONTROLLER_H_
#define _BLUE_V4_POWER_CONTROLLER_H_

#include "mcp_server.h"
#include "power_save_timer.h"

// Sleep entry tool — Blue V4 clone of the blue-v2 power controller. The board's
// sleep hook also relaxes the servos so the robot does not hold torque while idle.
class PowerController {
public:
    explicit PowerController(PowerSaveTimer* timer) : timer_(timer) {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool("self.power.enter_sleep",
                    "Enter sleep mode (sleepy face, dim screen, servos relaxed). "
                    "Use when the user says goodbye or asks the robot to sleep.",
                    PropertyList(), [this](const PropertyList&) -> ReturnValue {
                        if (timer_ != nullptr) {
                            timer_->EnterSleepNow();
                        }
                        return true;
                    });
    }

private:
    PowerSaveTimer* timer_;
};

#endif  // _BLUE_V4_POWER_CONTROLLER_H_
