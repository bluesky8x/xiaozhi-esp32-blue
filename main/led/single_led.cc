#include "single_led.h"
#include "application.h"
#include <esp_log.h> 

#define TAG "SingleLed"

#define DEFAULT_BRIGHTNESS 4
#define HIGH_BRIGHTNESS 16
#define LOW_BRIGHTNESS 2

#define BLINK_INFINITE -1


SingleLed::SingleLed(gpio_num_t gpio) {
    if (gpio == GPIO_NUM_NC) {
        ESP_LOGW(TAG, "SingleLed initialized with GPIO_NUM_NC, LED will not function");
        return;
    }

    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = gpio;
    strip_config.max_leds = 1;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_config.led_model = LED_MODEL_WS2812;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000; // 10MHz

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_));
    led_strip_clear(led_strip_);

    esp_timer_create_args_t blink_timer_args = {
        .callback = AnimTimerTrampoline,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "led_anim",
        .skip_unhandled_events = true,
    };
    ESP_ERROR_CHECK(esp_timer_create(&blink_timer_args, &blink_timer_));
}

SingleLed::~SingleLed() {
    if (blink_timer_ != nullptr) {
        esp_timer_stop(blink_timer_);
    }
    if (led_strip_ != nullptr) {
        led_strip_del(led_strip_);
    }
}


void SingleLed::SetColor(uint8_t r, uint8_t g, uint8_t b) {
    r_ = r;
    g_ = g;
    b_ = b;
}

void SingleLed::AnimTimerTrampoline(void* arg) {
    static_cast<SingleLed*>(arg)->OnAnimTimer();
}

void SingleLed::OnAnimTimer() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (anim_mode_ == AnimMode::Gradient) {
        OnGradientTimer();
    } else if (anim_mode_ == AnimMode::Blink) {
        OnBlinkTimer();
    }
}

void SingleLed::StopAnimTimerLocked() {
    esp_timer_stop(blink_timer_);
    anim_mode_ = AnimMode::None;
    grad_tick_ = 0;
}

static uint8_t LerpChannel(uint8_t a, uint8_t b, uint8_t t) {
    return static_cast<uint8_t>((static_cast<uint16_t>(a) * (255 - t) + static_cast<uint16_t>(b) * t) / 255);
}

void SingleLed::SetManualColor(uint8_t r, uint8_t g, uint8_t b) {
    if (led_strip_ == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    manual_control_ = true;
    StopAnimTimerLocked();
    r_ = r;
    g_ = g;
    b_ = b;
    led_strip_set_pixel(led_strip_, 0, r_, g_, b_);
    led_strip_refresh(led_strip_);
}

void SingleLed::SetManualColorGradient(uint8_t r0, uint8_t g0, uint8_t b0, uint8_t r1, uint8_t g1,
                                       uint8_t b1, int period_ms) {
    if (led_strip_ == nullptr) {
        return;
    }
    if (period_ms < 200) {
        period_ms = 200;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    manual_control_ = true;
    StopAnimTimerLocked();
    grad_r0_ = r0;
    grad_g0_ = g0;
    grad_b0_ = b0;
    grad_r1_ = r1;
    grad_g1_ = g1;
    grad_b1_ = b1;
    grad_period_ms_ = period_ms;
    grad_tick_ = 0;
    r_ = r0;
    g_ = g0;
    b_ = b0;
    anim_mode_ = AnimMode::Gradient;
    led_strip_set_pixel(led_strip_, 0, r_, g_, b_);
    led_strip_refresh(led_strip_);
    esp_timer_start_periodic(blink_timer_, static_cast<uint64_t>(grad_step_ms_) * 1000);
}

void SingleLed::OnGradientTimer() {
    if (led_strip_ == nullptr) {
        return;
    }
    const int ticks_per_period = grad_period_ms_ / grad_step_ms_;
    if (ticks_per_period < 4) {
        return;
    }
    grad_tick_ = (grad_tick_ + 1) % ticks_per_period;
    const int half = ticks_per_period / 2;
    uint8_t mix;
    if (grad_tick_ < half) {
        mix = static_cast<uint8_t>(grad_tick_ * 255 / half);
    } else {
        mix = static_cast<uint8_t>((ticks_per_period - grad_tick_) * 255 / half);
    }
    const uint8_t r = LerpChannel(grad_r0_, grad_r1_, mix);
    const uint8_t g = LerpChannel(grad_g0_, grad_g1_, mix);
    const uint8_t b = LerpChannel(grad_b0_, grad_b1_, mix);
    r_ = r;
    g_ = g;
    b_ = b;
    led_strip_set_pixel(led_strip_, 0, r_, g_, b_);
    led_strip_refresh(led_strip_);
}

void SingleLed::ClearManualColor() {
    if (led_strip_ == nullptr) {
        manual_control_ = false;
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        manual_control_ = false;
        StopAnimTimerLocked();
    }
    OnStateChanged();
}

void SingleLed::TurnOn() {
    if (led_strip_ == nullptr) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    StopAnimTimerLocked();
    led_strip_set_pixel(led_strip_, 0, r_, g_, b_);
    led_strip_refresh(led_strip_);
}

void SingleLed::TurnOff() {
    if (led_strip_ == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    StopAnimTimerLocked();
    led_strip_clear(led_strip_);
}

void SingleLed::BlinkOnce() {
    Blink(1, 100);
}

void SingleLed::Blink(int times, int interval_ms) {
    StartBlinkTask(times, interval_ms);
}

void SingleLed::StartContinuousBlink(int interval_ms) {
    StartBlinkTask(BLINK_INFINITE, interval_ms);
}

void SingleLed::StartBlinkTask(int times, int interval_ms) {
    if (led_strip_ == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    StopAnimTimerLocked();

    blink_counter_ = times * 2;
    blink_interval_ms_ = interval_ms;
    anim_mode_ = AnimMode::Blink;
    esp_timer_start_periodic(blink_timer_, interval_ms * 1000);
}

void SingleLed::OnBlinkTimer() {
    blink_counter_--;
    if (blink_counter_ & 1) {
        led_strip_set_pixel(led_strip_, 0, r_, g_, b_);
        led_strip_refresh(led_strip_);
    } else {
        led_strip_clear(led_strip_);

        if (blink_counter_ == 0) {
            StopAnimTimerLocked();
        }
    }
}


void SingleLed::OnStateChanged() {
    if (manual_control_) {
        return;
    }
    auto& app = Application::GetInstance();
    auto device_state = app.GetDeviceState();
    switch (device_state) {
        case kDeviceStateStarting:
            SetColor(0, 0, DEFAULT_BRIGHTNESS);
            StartContinuousBlink(100);
            break;
        case kDeviceStateWifiConfiguring:
            SetColor(0, 0, DEFAULT_BRIGHTNESS);
            StartContinuousBlink(500);
            break;
        case kDeviceStateIdle:
            TurnOff();
            break;
        case kDeviceStateConnecting:
            SetColor(0, 0, DEFAULT_BRIGHTNESS);
            TurnOn();
            break;
        case kDeviceStateListening:
        case kDeviceStateAudioTesting:
            if (app.IsVoiceDetected()) {
                SetColor(HIGH_BRIGHTNESS, 0, 0);
            } else {
                SetColor(LOW_BRIGHTNESS, 0, 0);
            }
            TurnOn();
            break;
        case kDeviceStateSpeaking:
            SetColor(0, DEFAULT_BRIGHTNESS, 0);
            TurnOn();
            break;
        case kDeviceStateUpgrading:
            SetColor(0, DEFAULT_BRIGHTNESS, 0);
            StartContinuousBlink(100);
            break;
        case kDeviceStateActivating:
            SetColor(0, DEFAULT_BRIGHTNESS, 0);
            StartContinuousBlink(500);
            break;
        default:
            ESP_LOGW(TAG, "Unknown led strip event: %d", device_state);
            return;
    }
}
