#ifndef _BLUE_V4_PCA9685_H_
#define _BLUE_V4_PCA9685_H_

#include <driver/gpio.h>
#include <driver/i2c_master.h>

#include <cstddef>
#include <cstdint>

// PCA9685 — 16-channel, 12-bit I2C PWM driver used for the Blue V4 quadruped's
// 8 servos (4 legs x hip + knee).
//
// The I2C master bus is OWNED BY THE BOARD and injected here; the PCA9685 shares
// that bus with the VL53L0X (0x29 vs 0x40). All register access happens from the
// caller's task (ServoController worker) — this class is not internally locked.
class Pca9685 {
public:
    static constexpr uint8_t kChannelCount = 16;
    static constexpr uint16_t kTickCount = 4096;  // 12-bit

    Pca9685() = default;
    ~Pca9685();

    Pca9685(const Pca9685&) = delete;
    Pca9685& operator=(const Pca9685&) = delete;

    // Adds the device to an existing bus and configures 50 Hz / 12-bit.
    // oe_gpio is driven (and left) disabled until SetOutputsEnabled(true).
    bool Init(i2c_master_bus_handle_t bus, uint8_t address, int freq_hz, gpio_num_t oe_gpio);

    bool IsReady() const { return ready_; }
    bool OutputsEnabled() const { return outputs_enabled_; }
    int FrequencyHz() const { return freq_hz_; }

    // Pulse width in microseconds for one channel (0 disables the channel).
    bool SetChannelPulseUs(uint8_t channel, uint16_t pulse_us);

    // Full-off for one channel / all channels.
    bool SetChannelOff(uint8_t channel);
    bool SetAllOff();

    // OE# (active LOW). Disabling is the emergency stop: servos go limp immediately.
    void SetOutputsEnabled(bool enabled);

    // Converts a pulse width to PCA9685 ticks for the configured frequency.
    uint16_t PulseUsToTicks(uint16_t pulse_us) const;

private:
    static constexpr uint8_t kRegMode1 = 0x00;
    static constexpr uint8_t kRegMode2 = 0x01;
    static constexpr uint8_t kRegLed0OnL = 0x06;
    static constexpr uint8_t kRegAllLedOnL = 0xFA;
    static constexpr uint8_t kRegPreScale = 0xFE;

    bool WriteReg(uint8_t reg, uint8_t value);
    bool WriteRegs(uint8_t reg, const uint8_t* data, size_t length);
    bool SetPwmFrequency(int freq_hz);

    i2c_master_dev_handle_t dev_ = nullptr;
    gpio_num_t oe_gpio_ = GPIO_NUM_NC;
    int freq_hz_ = 50;
    bool ready_ = false;
    bool outputs_enabled_ = false;
};

#endif  // _BLUE_V4_PCA9685_H_
