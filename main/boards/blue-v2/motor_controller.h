#ifndef __MOTOR_CONTROLLER_H__
#define __MOTOR_CONTROLLER_H__

#include "config.h"
#include "mcp_server.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>

#define MOTOR_TAG "MotorController"

#ifndef MOTOR_USE_GPIO_ONLY
#define MOTOR_USE_GPIO_ONLY 1
#endif

// MX1508 / L9110S on IN1–IN4. GPIO mode: sign = direction, magnitude ignored (full ON/OFF).
class MotorController {
private:
    gpio_num_t left_in1_;
    gpio_num_t left_in2_;
    gpio_num_t right_in1_;
    gpio_num_t right_in2_;
    esp_timer_handle_t stop_timer_ = nullptr;
    bool gpio_initialized_ = false;
    int left_speed_ = 0;
    int right_speed_ = 0;

    static void StopTimerCallback(void* arg) {
        static_cast<MotorController*>(arg)->Stop();
    }

    void InitMotorGpio() {
        if (gpio_initialized_) {
            return;
        }
        const gpio_num_t pins[] = {left_in1_, left_in2_, right_in1_, right_in2_};
        for (gpio_num_t pin : pins) {
            gpio_reset_pin(pin);
            gpio_set_direction(pin, GPIO_MODE_OUTPUT);
            gpio_set_level(pin, 0);
        }
        gpio_initialized_ = true;
        ESP_LOGI(MOTOR_TAG, "Motor GPIO outputs ready on %d/%d/%d/%d (no LEDC)",
                 left_in1_, left_in2_, right_in1_, right_in2_);
    }

    // MX1508: forward = IN1 HIGH + IN2 LOW; reverse = IN2 HIGH + IN1 LOW.
    void DriveSide(gpio_num_t in1, gpio_num_t in2, int speed) {
        speed = std::clamp(speed, -100, 100);
        if (speed == 0) {
            gpio_set_level(in1, 0);
            gpio_set_level(in2, 0);
            return;
        }
        if (speed > 0) {
            gpio_set_level(in1, 1);
            gpio_set_level(in2, 0);
        } else {
            gpio_set_level(in1, 0);
            gpio_set_level(in2, 1);
        }
    }

    void ScheduleAutoStop(int duration_ms) {
        if (stop_timer_ == nullptr || duration_ms <= 0) {
            return;
        }
        esp_timer_stop(stop_timer_);
        esp_timer_start_once(stop_timer_, static_cast<uint64_t>(duration_ms) * 1000);
    }

public:
    MotorController(gpio_num_t left_in1, gpio_num_t left_in2, gpio_num_t right_in1, gpio_num_t right_in2)
        : left_in1_(left_in1),
          left_in2_(left_in2),
          right_in1_(right_in1),
          right_in2_(right_in2) {
        InitMotorGpio();

        esp_timer_create_args_t timer_args = {
            .callback = StopTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "motor_stop",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &stop_timer_));

        auto& mcp_server = McpServer::GetInstance();

        mcp_server.AddTool("self.motor.stop", "Stop all motors. Use for: dừng, dừng lại, stop.", PropertyList(),
                           [this](const PropertyList&) -> ReturnValue {
                               Stop();
                               return true;
                           });

        mcp_server.AddTool("self.motor.forward",
                           "Move robot forward. Use for: đi tới, tiến, đi thẳng, go forward.",
                           PropertyList(), [this](const PropertyList&) -> ReturnValue {
                               Move(100, 100, MOTOR_AUTO_STOP_MS);
                               return true;
                           });

        mcp_server.AddTool("self.motor.backward",
                           "Move robot backward. Use for: lùi, đi lùi, go back.",
                           PropertyList(), [this](const PropertyList&) -> ReturnValue {
                               Move(-100, -100, MOTOR_AUTO_STOP_MS);
                               return true;
                           });

        mcp_server.AddTool("self.motor.turn_left",
                           "Turn robot left in place. Use for: quay sang trái, rẽ trái, turn left.",
                           PropertyList(),
                           [this](const PropertyList&) -> ReturnValue {
                               Move(-100, 100, MOTOR_AUTO_STOP_MS);
                               return true;
                           });

        mcp_server.AddTool("self.motor.turn_right",
                           "Turn robot right in place. Use for: quay sang phải, rẽ phải, turn right.",
                           PropertyList(),
                           [this](const PropertyList&) -> ReturnValue {
                               Move(100, -100, MOTOR_AUTO_STOP_MS);
                               return true;
                           });

        mcp_server.AddTool(
            "self.motor.move",
            "Drive straight or turn. left/right sign = direction (-100..100), full speed when non-zero. "
            "For circle use self.motor.circle. duration_ms auto-stop (100-10000).",
            PropertyList({Property("left", kPropertyTypeInteger, 0, -100, 100),
                          Property("right", kPropertyTypeInteger, 0, -100, 100),
                          Property("duration_ms", kPropertyTypeInteger, MOTOR_AUTO_STOP_MS, 100, 10000)}),
            [this](const PropertyList& properties) -> ReturnValue {
                const int left = properties["left"].value<int>();
                const int right = properties["right"].value<int>();
                const int duration_ms = properties["duration_ms"].value<int>();
                Move(left, right, duration_ms);
                return true;
            });

        mcp_server.AddTool(
            "self.motor.circle",
            "Drive in a circle (left wheel reverse). duration_ms auto-stop (1000-30000).",
            PropertyList({Property("duration_ms", kPropertyTypeInteger, MOTOR_AUTO_STOP_MS, 1000, 30000)}),
            [this](const PropertyList& properties) -> ReturnValue {
                const int duration_ms = properties["duration_ms"].value<int>();
                Move(-100, 100, duration_ms);
                return true;
            });

        mcp_server.AddTool("self.chassis.go_forward", "Move forward", PropertyList(), [this](const PropertyList&) -> ReturnValue {
            Move(100, 100, MOTOR_AUTO_STOP_MS);
            return true;
        });

        mcp_server.AddTool("self.chassis.go_back", "Move backward", PropertyList(), [this](const PropertyList&) -> ReturnValue {
            Move(-100, -100, MOTOR_AUTO_STOP_MS);
            return true;
        });

        mcp_server.AddTool("self.chassis.turn_left", "Turn left", PropertyList(), [this](const PropertyList&) -> ReturnValue {
            Move(-100, 100, MOTOR_AUTO_STOP_MS);
            return true;
        });

        mcp_server.AddTool("self.chassis.turn_right", "Turn right", PropertyList(), [this](const PropertyList&) -> ReturnValue {
            Move(100, -100, MOTOR_AUTO_STOP_MS);
            return true;
        });

        ESP_LOGI(MOTOR_TAG, "Motor MCP tools registered (GPIO on/off, MX1508 %d/%d/%d/%d)",
                 left_in1_, left_in2_, right_in1_, right_in2_);
    }

    ~MotorController() {
        if (stop_timer_ != nullptr) {
            esp_timer_stop(stop_timer_);
            esp_timer_delete(stop_timer_);
            stop_timer_ = nullptr;
        }
        Stop();
    }

    void Move(int left_speed, int right_speed, int duration_ms = MOTOR_AUTO_STOP_MS) {
        InitMotorGpio();
        left_speed_ = left_speed;
        right_speed_ = right_speed;
        DriveSide(left_in1_, left_in2_, left_speed);
        DriveSide(right_in1_, right_in2_, right_speed);
        ESP_LOGI(MOTOR_TAG, "Move left=%d right=%d duration=%dms", left_speed, right_speed, duration_ms);
        if (left_speed == 0 && right_speed == 0) {
            esp_timer_stop(stop_timer_);
        } else {
            ScheduleAutoStop(duration_ms);
        }
    }

    void PreparePwm() {}

    void Stop() {
        esp_timer_stop(stop_timer_);
        left_speed_ = 0;
        right_speed_ = 0;
        if (gpio_initialized_) {
            DriveSide(left_in1_, left_in2_, 0);
            DriveSide(right_in1_, right_in2_, 0);
        }
        ESP_LOGI(MOTOR_TAG, "Motors stopped");
    }

    bool IsMoving() const { return left_speed_ != 0 || right_speed_ != 0; }

    bool IsMovingForward() const { return left_speed_ > 0 && right_speed_ > 0; }

    int LeftSpeed() const { return left_speed_; }

    int RightSpeed() const { return right_speed_; }
};

#endif  // __MOTOR_CONTROLLER_H__
