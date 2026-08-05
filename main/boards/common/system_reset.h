#ifndef _SYSTEM_RESET_H
#define _SYSTEM_RESET_H

#include <driver/gpio.h>

class SystemReset {
public:
    SystemReset(gpio_num_t reset_nvs_pin, gpio_num_t reset_factory_pin);
    void CheckButtons();

    /** Erase NVS (WiFi, settings) and reboot. */
    static void ResetNvsAndReboot(int delay_seconds = 3);
    /** Erase NVS + OTA otadata and reboot. */
    static void FactoryResetAndReboot(int delay_seconds = 3);

private:
    gpio_num_t reset_nvs_pin_;
    gpio_num_t reset_factory_pin_;

    static void ResetNvsFlash();
    static void ResetOtaData();
    static void RestartInSeconds(int seconds);
};

#endif
