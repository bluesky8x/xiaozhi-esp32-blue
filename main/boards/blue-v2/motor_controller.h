#ifndef __MOTOR_CONTROLLER_H__
#define __MOTOR_CONTROLLER_H__

#include "config.h"

#include <atomic>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#define MOTOR_TAG "MotorController"

#ifndef MOTOR_USE_GPIO_ONLY
#define MOTOR_USE_GPIO_ONLY 1
#endif

#ifndef MOTOR_BRAKE_ENABLE
#define MOTOR_BRAKE_ENABLE 1
#endif

#ifndef MOTOR_BRAKE_MS
#define MOTOR_BRAKE_MS 50
#endif

#ifndef MOTOR_MAX_DUTY_PCT
#define MOTOR_MAX_DUTY_PCT 100
#endif

#ifndef MOTOR_CMD_QUEUE_DEPTH
#define MOTOR_CMD_QUEUE_DEPTH 8
#endif

// Call from board constructor — MotorController init is deferred until activation (~10s).
inline void MotorGpioBrakeEarly(gpio_num_t left_in1, gpio_num_t left_in2, gpio_num_t right_in1,
                               gpio_num_t right_in2) {
    const gpio_num_t pins[] = {left_in1, left_in2, right_in1, right_in2};
    for (gpio_num_t pin : pins) {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, 0);
    }
    ESP_LOGI(MOTOR_TAG, "Motor GPIO brake early on %d/%d/%d/%d (LOW until MCP init)",
             left_in1, left_in2, right_in1, right_in2);
}

// MX1508 / L9110S on IN1–IN4. GPIO mode: sign = direction, magnitude ignored (full ON/OFF).
// MCP / ToF producers enqueue commands; a dedicated FreeRTOS task is the sole GPIO consumer.
class MotorController {
public:
    MotorController(gpio_num_t left_in1, gpio_num_t left_in2, gpio_num_t right_in1,
                    gpio_num_t right_in2);
    ~MotorController();

    MotorController(const MotorController&) = delete;
    MotorController& operator=(const MotorController&) = delete;

    /** Enqueue move (non-blocking). Returns false if queue full. */
    bool EnqueueMove(int left_speed, int right_speed, int duration_ms = MOTOR_AUTO_STOP_MS);

    /** Enqueue stop — clears pending moves. Returns false if queue full. */
    bool EnqueueStop();

    bool IsMoving() const { return moving_.load(std::memory_order_acquire); }

    bool IsMovingForward() const {
        return left_speed_.load(std::memory_order_acquire) > 0 &&
               right_speed_.load(std::memory_order_acquire) > 0;
    }

    int LeftSpeed() const { return left_speed_.load(std::memory_order_acquire); }

    int RightSpeed() const { return right_speed_.load(std::memory_order_acquire); }

    void PreparePwm() {}

    /** True while motors run — audio uplink should drop mic frames (EMI). */
    static bool ShouldPauseUplink();

private:
    enum class CmdType : uint8_t { kStop, kMove };

    struct MotorCommand {
        CmdType type;
        int8_t left;
        int8_t right;
        int16_t duration_ms;
    };

    gpio_num_t left_in1_;
    gpio_num_t left_in2_;
    gpio_num_t right_in1_;
    gpio_num_t right_in2_;
    QueueHandle_t cmd_queue_ = nullptr;
    TaskHandle_t worker_task_ = nullptr;
    esp_timer_handle_t stop_timer_ = nullptr;
    bool gpio_initialized_ = false;
    std::atomic<bool> moving_{false};
    std::atomic<int> left_speed_{0};
    std::atomic<int> right_speed_{0};
    int cached_level_left_in1_ = -1;
    int cached_level_left_in2_ = -1;
    int cached_level_right_in1_ = -1;
    int cached_level_right_in2_ = -1;

    static MotorController* instance_;

    static void WorkerTaskEntry(void* arg);
    static void StopTimerCallback(void* arg);

    void WorkerLoop();
    void EnqueueStopFromTimer();
    bool PushCommand(const MotorCommand& cmd, bool clear_pending);
    void ExecuteMove(int left_speed, int right_speed, int duration_ms);
    void ExecuteStop();
    void NotifyStoppedOnMain();

    int* LevelCacheFor(gpio_num_t pin);
    void ResetLevelCache();
    void SetMotorPin(gpio_num_t pin, int level);
    void InitMotorGpio();
    void CoastSide(gpio_num_t in1, gpio_num_t in2);
    void BrakeSide(gpio_num_t in1, gpio_num_t in2);
    void DriveSide(gpio_num_t in1, gpio_num_t in2, int speed);
    void ScheduleAutoStop(int duration_ms);
    void RegisterMcpTools();
};

#endif  // __MOTOR_CONTROLLER_H__
