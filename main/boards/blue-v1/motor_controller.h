#ifndef __MOTOR_CONTROLLER_H__
#define __MOTOR_CONTROLLER_H__

#include "config.h"
#include "mcp_server.h"

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>
#include <cmath>

#define MOTOR_TAG "MotorController"

// MX1508 / L9110S: PWM on IN1–IN4 (ESP-IDF LEDC, same role as Arduino ledcAttach on GPIO 11–14).
#ifndef MOTOR_PWM_FREQ_HZ
#define MOTOR_PWM_FREQ_HZ 20000
#endif

class MotorController {
private:
    gpio_num_t left_in1_;
    gpio_num_t left_in2_;
    gpio_num_t right_in1_;
    gpio_num_t right_in2_;
    ledc_channel_t left_in1_ch_;
    ledc_channel_t left_in2_ch_;
    ledc_channel_t right_in1_ch_;
    ledc_channel_t right_in2_ch_;
    esp_timer_handle_t stop_timer_ = nullptr;
    static constexpr int kPwmResolutionBits = 10;
    static constexpr int kDutyMax = (1 << kPwmResolutionBits) - 1;

    static void StopTimerCallback(void* arg) {
        static_cast<MotorController*>(arg)->Stop();
    }

    void InitPwmTimer() {
        ledc_timer_config_t timer_cfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_10_BIT,
            .timer_num = LEDC_TIMER_1,
            .freq_hz = MOTOR_PWM_FREQ_HZ,
            .clk_cfg = LEDC_AUTO_CLK,
            .deconfigure = false,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    }

    void AttachPwmPin(gpio_num_t gpio, ledc_channel_t channel) {
        ledc_channel_config_t channel_cfg = {
            .gpio_num = gpio,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = channel,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_1,
            .duty = 0,
            .hpoint = 0,
            .flags = {
                .output_invert = 0,
            },
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    }

    void SetPwmDuty(ledc_channel_t channel, int speed_percent) {
        const int duty = std::clamp(speed_percent, 0, 100) * kDutyMax / 100;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
    }

    // MX1508: forward = PWM on IN1 + IN2 LOW; reverse = PWM on IN2 + IN1 LOW.
    void DriveSide(ledc_channel_t in1_ch, ledc_channel_t in2_ch, int speed) {
        speed = std::clamp(speed, -100, 100);
        if (speed == 0) {
            SetPwmDuty(in1_ch, 0);
            SetPwmDuty(in2_ch, 0);
            return;
        }

        const int magnitude = std::abs(speed);
        if (speed > 0) {
            SetPwmDuty(in1_ch, magnitude);
            SetPwmDuty(in2_ch, 0);
        } else {
            SetPwmDuty(in1_ch, 0);
            SetPwmDuty(in2_ch, magnitude);
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
          right_in2_(right_in2),
          left_in1_ch_(LEDC_CHANNEL_1),
          left_in2_ch_(LEDC_CHANNEL_2),
          right_in1_ch_(LEDC_CHANNEL_3),
          right_in2_ch_(LEDC_CHANNEL_4) {
        // One timer, four channels — analogous to Arduino ledcAttach() on each of 11–14.
        InitPwmTimer();
        AttachPwmPin(left_in1_, left_in1_ch_);
        AttachPwmPin(left_in2_, left_in2_ch_);
        AttachPwmPin(right_in1_, right_in1_ch_);
        AttachPwmPin(right_in2_, right_in2_ch_);

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
                               Move(-70, 70, MOTOR_AUTO_STOP_MS);
                               return true;
                           });

        mcp_server.AddTool("self.motor.turn_right",
                           "Turn robot right in place. Use for: quay sang phải, rẽ phải, turn right.",
                           PropertyList(),
                           [this](const PropertyList&) -> ReturnValue {
                               Move(70, -70, MOTOR_AUTO_STOP_MS);
                               return true;
                           });

        mcp_server.AddTool(
            "self.motor.move",
            "Drive with per-wheel speed. left/right range -100 (full reverse) to 100 (full forward). "
            "duration_ms: auto-stop timeout (100-10000).",
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

        mcp_server.AddTool("self.chassis.go_forward", "Move forward", PropertyList(), [this](const PropertyList&) -> ReturnValue {
            Move(100, 100, MOTOR_AUTO_STOP_MS);
            return true;
        });

        mcp_server.AddTool("self.chassis.go_back", "Move backward", PropertyList(), [this](const PropertyList&) -> ReturnValue {
            Move(-100, -100, MOTOR_AUTO_STOP_MS);
            return true;
        });

        mcp_server.AddTool("self.chassis.turn_left", "Turn left", PropertyList(), [this](const PropertyList&) -> ReturnValue {
            Move(-70, 70, MOTOR_AUTO_STOP_MS);
            return true;
        });

        mcp_server.AddTool("self.chassis.turn_right", "Turn right", PropertyList(), [this](const PropertyList&) -> ReturnValue {
            Move(70, -70, MOTOR_AUTO_STOP_MS);
            return true;
        });

        ESP_LOGI(MOTOR_TAG, "Motor MCP tools registered (MX1508 PWM on GPIO %d/%d/%d/%d, %d Hz)",
                 left_in1_, left_in2_, right_in1_, right_in2_, MOTOR_PWM_FREQ_HZ);
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
        DriveSide(left_in1_ch_, left_in2_ch_, left_speed);
        DriveSide(right_in1_ch_, right_in2_ch_, right_speed);
        ESP_LOGI(MOTOR_TAG, "Move left=%d right=%d duration=%dms", left_speed, right_speed, duration_ms);
        if (left_speed == 0 && right_speed == 0) {
            esp_timer_stop(stop_timer_);
        } else {
            ScheduleAutoStop(duration_ms);
        }
    }

    void Stop() {
        esp_timer_stop(stop_timer_);
        Move(0, 0, 0);
        ESP_LOGI(MOTOR_TAG, "Motors stopped");
    }
};

#endif  // __MOTOR_CONTROLLER_H__
