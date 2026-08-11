#ifndef __POWER_CONTROLLER_H__
#define __POWER_CONTROLLER_H__

#include "mcp_server.h"
#include "power_save_timer.h"

class PowerController {
public:
    explicit PowerController(PowerSaveTimer* timer) : timer_(timer) {
        auto& mcp = McpServer::GetInstance();
        mcp.AddTool(
            "self.power.enter_sleep",
            "Enter sleep mode (sleepy face, dim screen). "
            "Use when user says goodbye, go to sleep, tạm biệt, ngủ đi, good night.",
            PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                if (timer_ != nullptr) {
                    timer_->EnterSleepNow();
                }
                return true;
            });
    }

private:
    PowerSaveTimer* timer_;
};

#endif
