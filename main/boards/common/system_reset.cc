#include "system_reset.h"

#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <esp_partition.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>

#define TAG "SystemReset"

SystemReset::SystemReset(gpio_num_t reset_nvs_pin, gpio_num_t reset_factory_pin)
    : reset_nvs_pin_(reset_nvs_pin), reset_factory_pin_(reset_factory_pin) {
    uint64_t pin_mask = 0;
    if (reset_nvs_pin_ != GPIO_NUM_NC) {
        pin_mask |= (1ULL << reset_nvs_pin_);
    }
    if (reset_factory_pin_ != GPIO_NUM_NC) {
        pin_mask |= (1ULL << reset_factory_pin_);
    }
    if (pin_mask == 0) {
        return;
    }

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = pin_mask;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}

void SystemReset::CheckButtons() {
    if (reset_factory_pin_ != GPIO_NUM_NC && gpio_get_level(reset_factory_pin_) == 0) {
        ESP_LOGI(TAG, "Factory reset button held at boot");
        FactoryResetAndReboot();
        return;
    }

    if (reset_nvs_pin_ != GPIO_NUM_NC && gpio_get_level(reset_nvs_pin_) == 0) {
        ESP_LOGI(TAG, "NVS reset button held at boot");
        ResetNvsAndReboot();
    }
}

void SystemReset::ResetNvsFlash() {
    ESP_LOGI(TAG, "Resetting NVS flash");
    esp_err_t ret = nvs_flash_erase();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase NVS flash");
    }
    ret = nvs_flash_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash");
    }
}

void SystemReset::ResetOtaData() {
    ESP_LOGI(TAG, "Erasing OTA data partition");
    const esp_partition_t* partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, NULL);
    if (partition == NULL) {
        ESP_LOGE(TAG, "Failed to find otadata partition");
        return;
    }
    esp_partition_erase_range(partition, 0, partition->size);
}

void SystemReset::RestartInSeconds(int seconds) {
    for (int i = seconds; i > 0; i--) {
        ESP_LOGI(TAG, "Rebooting in %d seconds", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    esp_restart();
}

void SystemReset::ResetNvsAndReboot(int delay_seconds) {
    ResetNvsFlash();
    RestartInSeconds(delay_seconds);
}

void SystemReset::FactoryResetAndReboot(int delay_seconds) {
    ResetNvsFlash();
    ResetOtaData();
    RestartInSeconds(delay_seconds);
}
