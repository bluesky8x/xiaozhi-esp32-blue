#include "gpio_led.h"
#include "application.h"
#include "device_state.h"
#include <esp_log.h>

#define TAG "GpioLed"

#define DEFAULT_BRIGHTNESS 40
#define HIGH_BRIGHTNESS 80
#define LOW_BRIGHTNESS 10

#define IDLE_BRIGHTNESS 5
#define SPEAKING_BRIGHTNESS 75
#define UPGRADING_BRIGHTNESS 25
#define ACTIVATING_BRIGHTNESS 35

#define BLINK_INFINITE -1

// GPIO_LED
#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0

#define LEDC_DUTY              (8191)
#define LEDC_FADE_TIME    (1000)
// GPIO_LED

GpioLed::GpioLed(gpio_num_t gpio)
        : GpioLed(gpio, 0, LEDC_LS_TIMER, LEDC_LS_CH0_CHANNEL) {
}

GpioLed::GpioLed(gpio_num_t gpio, int output_invert)
        : GpioLed(gpio, output_invert, LEDC_LS_TIMER, LEDC_LS_CH0_CHANNEL) {
}

GpioLed::GpioLed(gpio_num_t gpio, int output_invert, ledc_timer_t timer_num, ledc_channel_t channel) {
    // If the gpio is not connected, you should use NoLed class
    assert(gpio != GPIO_NUM_NC);

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {};
    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;  // resolution of PWM duty
    ledc_timer.freq_hz = 4000;                      // frequency of PWM signal
    ledc_timer.speed_mode = LEDC_LS_MODE;           // timer mode
    ledc_timer.timer_num = timer_num;               // timer index
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;              // Auto select the source clock

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_.channel    = channel,
    ledc_channel_.duty       = 0,
    ledc_channel_.gpio_num   = gpio,
    ledc_channel_.speed_mode = LEDC_LS_MODE,
    ledc_channel_.hpoint     = 0,
    ledc_channel_.timer_sel  = timer_num,
    ledc_channel_.flags.output_invert = output_invert & 0x01,

    // Set LED Controller with previously prepared configuration
    ledc_channel_config(&ledc_channel_);

    // Initialize fade service.
    ledc_fade_func_install(0);

    // When the callback registered by ledc_cb_degister is called, run led ->OnFadeEnd()
    ledc_cbs_t ledc_callbacks = {
        .fade_cb = FadeCallback
    };
    ledc_cb_register(ledc_channel_.speed_mode, ledc_channel_.channel, &ledc_callbacks, this);

    esp_timer_create_args_t blink_timer_args = {
        .callback = [](void *arg) {
            auto led = static_cast<GpioLed*>(arg);
            led->OnBlinkTimer();
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "Blink Timer",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&blink_timer_args, &blink_timer_));

    xTaskCreate(EventTask, "LedEvent", 2048, this, 
            tskIDLE_PRIORITY + 2, &event_task_handle_);

    ledc_initialized_ = true;
}

GpioLed::~GpioLed() {
    esp_timer_stop(blink_timer_);
    if (ledc_initialized_) {
        ledc_fade_stop(ledc_channel_.speed_mode, ledc_channel_.channel);
        ledc_fade_func_uninstall();
    }
}


void GpioLed::SetBrightness(uint8_t brightness) {
    if (brightness > 80) {
        brightness = 80;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    brightness_ = brightness;
    duty_ = (uint32_t)brightness * LEDC_DUTY / 100;

    if (ledc_initialized_ && is_on_) {
        ledc_set_duty(ledc_channel_.speed_mode, ledc_channel_.channel, duty_);
        ledc_update_duty(ledc_channel_.speed_mode, ledc_channel_.channel);
    }
}

uint8_t GpioLed::GetBrightness() const {
    return brightness_;
}

void GpioLed::SetManualBrightness(uint8_t brightness) {
    if (brightness > 80) {
        brightness = 80;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    manual_control_ = true;
    esp_timer_stop(blink_timer_);
    if (ledc_initialized_) {
        ledc_fade_stop(ledc_channel_.speed_mode, ledc_channel_.channel);
    }

    brightness_ = brightness;
    duty_ = (uint32_t)brightness * LEDC_DUTY / 100;
    if (brightness > 0) {
        is_on_ = true;
        ledc_set_duty(ledc_channel_.speed_mode, ledc_channel_.channel, duty_);
    } else {
        is_on_ = false;
        ledc_set_duty(ledc_channel_.speed_mode, ledc_channel_.channel, 0);
    }
    ledc_update_duty(ledc_channel_.speed_mode, ledc_channel_.channel);
}

void GpioLed::ClearManualControl() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        manual_control_ = false;
    }
    OnStateChanged();
}

void GpioLed::TurnOn() {
    if (!ledc_initialized_) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    esp_timer_stop(blink_timer_);
    ledc_fade_stop(ledc_channel_.speed_mode, ledc_channel_.channel);
    is_on_ = true;
    ledc_set_duty(ledc_channel_.speed_mode, ledc_channel_.channel, duty_);
    ledc_update_duty(ledc_channel_.speed_mode, ledc_channel_.channel);
}

void GpioLed::TurnOff() {
    if (!ledc_initialized_) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    esp_timer_stop(blink_timer_);
    ledc_fade_stop(ledc_channel_.speed_mode, ledc_channel_.channel);
    is_on_ = false;
    ledc_set_duty(ledc_channel_.speed_mode, ledc_channel_.channel, 0);
    ledc_update_duty(ledc_channel_.speed_mode, ledc_channel_.channel);
}

void GpioLed::BlinkOnce() {
    Blink(1, 100);
}

void GpioLed::Blink(int times, int interval_ms) {
    StartBlinkTask(times, interval_ms);
}

void GpioLed::BlinkFor(int duration_ms, int interval_ms) {
    if (interval_ms <= 0) {
        interval_ms = 150;
    }
    int times = duration_ms / (interval_ms * 2);
    if (times <= 0) {
        times = 1;
    }
    Blink(times, interval_ms);
}

void GpioLed::StartContinuousBlink(int interval_ms) {
    StartBlinkTask(BLINK_INFINITE, interval_ms);
}

void GpioLed::StartBlinkTask(int times, int interval_ms) {
    if (!ledc_initialized_) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    esp_timer_stop(blink_timer_);
    ledc_fade_stop(ledc_channel_.speed_mode, ledc_channel_.channel);

    blink_counter_ = times * 2;
    blink_interval_ms_ = interval_ms;
    if (times > 0) {
        manual_control_ = true;
        is_on_ = true;
    }
    esp_timer_start_periodic(blink_timer_, interval_ms * 1000);
}

void GpioLed::OnBlinkTimer() {
    std::lock_guard<std::mutex> lock(mutex_);
    blink_counter_--;
    if (blink_counter_ & 1) {
        ledc_set_duty(ledc_channel_.speed_mode, ledc_channel_.channel, duty_);
    } else {
        if (blink_counter_ == 0) {
            esp_timer_stop(blink_timer_);
            manual_control_ = false;

            auto& app = Application::GetInstance();
            auto device_state = app.GetDeviceState();
            if (device_state == kDeviceStateSpeaking) {
                brightness_ = 40;
                duty_ = (uint32_t)40 * LEDC_DUTY / 100;
                is_on_ = true;
                ledc_set_duty(ledc_channel_.speed_mode, ledc_channel_.channel, duty_);
            } else if (device_state == kDeviceStateListening || device_state == kDeviceStateAudioTesting) {
                brightness_ = 80;
                duty_ = (uint32_t)80 * LEDC_DUTY / 100;
                is_on_ = true;
                ledc_set_duty(ledc_channel_.speed_mode, ledc_channel_.channel, duty_);
            } else if (device_state == kDeviceStateIdle) {
                is_on_ = false;
                ledc_set_duty(ledc_channel_.speed_mode, ledc_channel_.channel, 0);
            } else {
                is_on_ = true;
                ledc_set_duty(ledc_channel_.speed_mode, ledc_channel_.channel, duty_);
            }
        } else {
            ledc_set_duty(ledc_channel_.speed_mode, ledc_channel_.channel, 0);
        }
    }
    ledc_update_duty(ledc_channel_.speed_mode, ledc_channel_.channel);
}

void GpioLed::StartFadeTask() {
    if (!ledc_initialized_) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    esp_timer_stop(blink_timer_);
    ledc_fade_stop(ledc_channel_.speed_mode, ledc_channel_.channel);
    fade_up_ = true;
    ledc_set_fade_with_time(ledc_channel_.speed_mode,
                            ledc_channel_.channel, LEDC_DUTY, LEDC_FADE_TIME);
    ledc_fade_start(ledc_channel_.speed_mode,
                    ledc_channel_.channel, LEDC_FADE_NO_WAIT);
}

void GpioLed::OnFadeEnd() {
    std::lock_guard<std::mutex> lock(mutex_);
    fade_up_ = !fade_up_;
    ledc_set_fade_with_time(ledc_channel_.speed_mode,
                            ledc_channel_.channel, fade_up_ ? LEDC_DUTY : 0, LEDC_FADE_TIME);
    ledc_fade_start(ledc_channel_.speed_mode,
                    ledc_channel_.channel, LEDC_FADE_NO_WAIT);
}

bool IRAM_ATTR GpioLed::FadeCallback(const ledc_cb_param_t *param, void *user_arg) {
    if (param->event == LEDC_FADE_END_EVT) {
        auto led = static_cast<GpioLed*>(user_arg);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTaskNotifyFromISR(led->event_task_handle_, 0x01, eSetValueWithOverwrite,
                           &xHigherPriorityTaskWoken);
    }
    return true;
}

void GpioLed::OnStateChanged() {
    if (manual_control_) {
        return;
    }
    auto& app = Application::GetInstance();
    auto device_state = app.GetDeviceState();
    switch (device_state) {
        case kDeviceStateStarting:
            SetBrightness(DEFAULT_BRIGHTNESS);
            TurnOn();
            break;
        case kDeviceStateWifiConfiguring:
            SetBrightness(DEFAULT_BRIGHTNESS);
            StartContinuousBlink(500);
            break;
        case kDeviceStateIdle:
            TurnOff();
            break;
        case kDeviceStateConnecting:
            SetBrightness(DEFAULT_BRIGHTNESS);
            TurnOn();
            break;
        case kDeviceStateListening:
        case kDeviceStateAudioTesting:
            // Mic chuyển sang lắng nghe -> bật đèn sáng (80% max)
            SetBrightness(HIGH_BRIGHTNESS);
            TurnOn();
            break;
        case kDeviceStateSpeaking:
            // Khi nói -> giảm brightness xuống 40
            SetBrightness(40);
            TurnOn();
            break;
        case kDeviceStateUpgrading:
            SetBrightness(25);
            StartContinuousBlink(100);
            break;
        case kDeviceStateActivating:
            SetBrightness(35);
            StartContinuousBlink(500);
            break;
        default:
            ESP_LOGE(TAG, "Unknown gpio led event: %d", device_state);
            return;
    }
}

void GpioLed::EventTask(void* arg) {
    GpioLed* led = static_cast<GpioLed*>(arg);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        led->OnFadeEnd();
    }
}