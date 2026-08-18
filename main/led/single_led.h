#ifndef _SINGLE_LED_H_
#define _SINGLE_LED_H_

#include "led.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <led_strip.h>
#include <esp_timer.h>
#include <atomic>
#include <mutex>

class SingleLed : public Led {
public:
    SingleLed(gpio_num_t gpio);
    virtual ~SingleLed();

    void OnStateChanged() override;

    /** Solid color; blocks OnStateChanged until ClearManualColor() (e.g. dance EQ). */
    void SetManualColor(uint8_t r, uint8_t g, uint8_t b);
    /** Pulse between two colors around the EQ hue (1-pixel "gradient"). */
    void SetManualColorGradient(uint8_t r0, uint8_t g0, uint8_t b0, uint8_t r1, uint8_t g1, uint8_t b1,
                                int period_ms);
    void ClearManualColor();

private:
    enum class AnimMode : uint8_t { None, Blink, Gradient };

    std::mutex mutex_;
    bool manual_control_ = false;
    AnimMode anim_mode_ = AnimMode::None;
    TaskHandle_t blink_task_ = nullptr;
    led_strip_handle_t led_strip_ = nullptr;
    uint8_t r_ = 0, g_ = 0, b_ = 0;
    int blink_counter_ = 0;
    int blink_interval_ms_ = 0;
    esp_timer_handle_t blink_timer_ = nullptr;

    uint8_t grad_r0_ = 0, grad_g0_ = 0, grad_b0_ = 0;
    uint8_t grad_r1_ = 0, grad_g1_ = 0, grad_b1_ = 0;
    int grad_period_ms_ = 800;
    int grad_step_ms_ = 50;
    int grad_tick_ = 0;

    void StartBlinkTask(int times, int interval_ms);
    void OnBlinkTimer();
    void OnGradientTimer();
    void OnAnimTimer();
    void StopAnimTimerLocked();

    static void AnimTimerTrampoline(void* arg);

    void BlinkOnce();
    void Blink(int times, int interval_ms);
    void StartContinuousBlink(int interval_ms);
    void TurnOn();
    void TurnOff();
    void SetColor(uint8_t r, uint8_t g, uint8_t b);
};

#endif // _SINGLE_LED_H_
