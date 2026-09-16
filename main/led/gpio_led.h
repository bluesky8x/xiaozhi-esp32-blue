#ifndef _GPIO_LED_H_
#define _GPIO_LED_H_

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "led.h"
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <esp_timer.h>
#include <atomic>
#include <mutex>

class GpioLed : public Led {
 public:
    GpioLed(gpio_num_t gpio);
    GpioLed(gpio_num_t gpio, int output_invert);
    GpioLed(gpio_num_t gpio, int output_invert, ledc_timer_t timer_num, ledc_channel_t channel);
    virtual ~GpioLed();

    void OnStateChanged() override;
    void TurnOn();
    void TurnOff();
    void SetBrightness(uint8_t brightness);
    uint8_t GetBrightness() const;
    void SetManualBrightness(uint8_t brightness);
    void ClearManualControl();
    void BlinkOnce();
    void Blink(int times, int interval_ms);
    void BlinkFor(int duration_ms, int interval_ms = 150);
    void StartContinuousBlink(int interval_ms);

 private:
    std::mutex mutex_;
    TaskHandle_t blink_task_ = nullptr;
    ledc_channel_config_t ledc_channel_ = {0};
    bool ledc_initialized_ = false;
    uint8_t brightness_ = 50;
    uint32_t duty_ = 0;
    bool is_on_ = false;
    bool manual_control_ = false;
    int blink_counter_ = 0;
    int blink_interval_ms_ = 0;
    esp_timer_handle_t blink_timer_ = nullptr;
    bool fade_up_ = true;
    TaskHandle_t event_task_handle_ = nullptr;
    
    static void EventTask(void* arg);
    void StartBlinkTask(int times, int interval_ms);
    void OnBlinkTimer();

    void StartFadeTask();
    void OnFadeEnd();
    static bool IRAM_ATTR FadeCallback(const ledc_cb_param_t *param, void *user_arg);
};

#endif  // _GPIO_LED_H_
