#include "pca9685.h"

#include "config.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cmath>
#include <cstring>

#define TAG "Pca9685"

namespace {
// Internal 25 MHz oscillator; PRE_SCALE = round(osc / (4096 * freq)) - 1.
constexpr float kOscillatorHz = 25000000.0f;
constexpr int kI2cTimeoutMs = 50;

// MODE1 bits
constexpr uint8_t kMode1AllCall = 0x01;
constexpr uint8_t kMode1Ai = 0x20;  // register auto-increment
constexpr uint8_t kMode1Sleep = 0x10;
constexpr uint8_t kMode1Restart = 0x80;

// LEDn_OFF_H bit 4 = full off; LEDn_ON_H bit 4 = full on.
constexpr uint8_t kFullOffBit = 0x10;
constexpr uint8_t kFullOnBit = 0x10;
}  // namespace

Pca9685::~Pca9685() {
    if (dev_ != nullptr) {
        i2c_master_bus_rm_device(dev_);
        dev_ = nullptr;
    }
}

bool Pca9685::Init(i2c_master_bus_handle_t bus, uint8_t address, int freq_hz, gpio_num_t oe_gpio) {
    if (bus == nullptr) {
        ESP_LOGE(TAG, "Init without an I2C bus handle");
        return false;
    }

    // OE# first, so the servos cannot twitch while the driver is being configured.
    oe_gpio_ = oe_gpio;
    if (oe_gpio_ != GPIO_NUM_NC) {
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << oe_gpio_;
        cfg.mode = GPIO_MODE_OUTPUT;
        cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&cfg);
        gpio_set_level(oe_gpio_, 1);  // active LOW -> 1 = outputs disabled
    }
    outputs_enabled_ = false;

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = address;
    dev_cfg.scl_speed_hz = I2C_SENSOR_SPEED_HZ;
    if (i2c_master_bus_add_device(bus, &dev_cfg, &dev_) != ESP_OK || dev_ == nullptr) {
        ESP_LOGE(TAG, "PCA9685 0x%02X not on bus", address);
        return false;
    }

    if (!SetPwmFrequency(freq_hz)) {
        return false;
    }

    // All channels off until the servo controller writes real targets.
    if (!SetAllOff()) {
        return false;
    }

    ready_ = true;
    ESP_LOGI(TAG, "PCA9685 0x%02X ready — %d Hz, %u channels, OE# GPIO %d (outputs OFF)", address,
             freq_hz_, kChannelCount, static_cast<int>(oe_gpio_));
    return true;
}

bool Pca9685::WriteReg(uint8_t reg, uint8_t value) { return WriteRegs(reg, &value, 1); }

bool Pca9685::WriteRegs(uint8_t reg, const uint8_t* data, size_t length) {
    if (dev_ == nullptr || data == nullptr || length > 8) {
        return false;
    }
    uint8_t buffer[9] = {};
    buffer[0] = reg;
    memcpy(&buffer[1], data, length);
    const esp_err_t err = i2c_master_transmit(dev_, buffer, length + 1, kI2cTimeoutMs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write reg 0x%02X failed: %s", reg, esp_err_to_name(err));
        return false;
    }
    return true;
}

bool Pca9685::SetPwmFrequency(int freq_hz) {
    if (freq_hz < 24 || freq_hz > 1526) {
        ESP_LOGE(TAG, "unsupported PWM frequency %d Hz (24-1526)", freq_hz);
        return false;
    }

    int prescale = static_cast<int>(lroundf(kOscillatorHz / (kTickCount * freq_hz))) - 1;
    if (prescale < 3) {
        prescale = 3;
    } else if (prescale > 255) {
        prescale = 255;
    }

    // Sleep is required before touching PRE_SCALE.
    if (!WriteReg(kRegMode1, kMode1Sleep)) {
        return false;
    }
    if (!WriteReg(kRegPreScale, static_cast<uint8_t>(prescale))) {
        return false;
    }
    // Wake up with auto-increment + ALLCALL, then RESTART to clear stale state.
    if (!WriteReg(kRegMode1, static_cast<uint8_t>(kMode1Ai | kMode1AllCall))) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    if (!WriteReg(kRegMode1, static_cast<uint8_t>(kMode1Ai | kMode1AllCall | kMode1Restart))) {
        return false;
    }

    freq_hz_ = freq_hz;
    ESP_LOGI(TAG, "PWM %d Hz (PRE_SCALE=%d, period %.0f us)", freq_hz, prescale,
             1000000.0f / static_cast<float>(freq_hz));
    return true;
}

uint16_t Pca9685::PulseUsToTicks(uint16_t pulse_us) const {
    const float period_us = 1000000.0f / static_cast<float>(freq_hz_);
    int ticks = static_cast<int>(lroundf((static_cast<float>(pulse_us) / period_us) * kTickCount));
    if (ticks < 0) {
        ticks = 0;
    } else if (ticks > kTickCount) {
        ticks = kTickCount;
    }
    return static_cast<uint16_t>(ticks);
}

bool Pca9685::SetChannelPulseUs(uint8_t channel, uint16_t pulse_us) {
    if (!ready_ || channel >= kChannelCount) {
        return false;
    }

    const uint8_t base = static_cast<uint8_t>(kRegLed0OnL + 4 * channel);
    uint8_t regs[4] = {0, 0, 0, 0};

    if (pulse_us == 0) {
        regs[3] = kFullOffBit;
    } else {
        const uint16_t ticks = PulseUsToTicks(pulse_us);
        if (ticks >= kTickCount) {
            regs[1] = kFullOnBit;  // full on
        } else {
            regs[2] = static_cast<uint8_t>(ticks & 0xFF);
            regs[3] = static_cast<uint8_t>((ticks >> 8) & 0x0F);
        }
    }
    return WriteRegs(base, regs, sizeof(regs));
}

bool Pca9685::SetChannelOff(uint8_t channel) { return SetChannelPulseUs(channel, 0); }

bool Pca9685::SetAllOff() {
    if (!ready_ && dev_ == nullptr) {
        return false;
    }
    // ALL_LED_ON_L/H = 0, ALL_LED_OFF_H = full off.
    const uint8_t regs[4] = {0x00, 0x00, 0x00, kFullOffBit};
    return WriteRegs(kRegAllLedOnL, regs, sizeof(regs));
}

void Pca9685::SetOutputsEnabled(bool enabled) {
    if (oe_gpio_ != GPIO_NUM_NC) {
        gpio_set_level(oe_gpio_, enabled ? 0 : 1);  // OE# is active LOW
    }
    outputs_enabled_ = enabled;
}
