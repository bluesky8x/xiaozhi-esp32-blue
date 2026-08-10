#include "blue_v2_otto_display.h"

#include "config.h"
#include "lvgl_theme.h"

#include <esp_log.h>

#define TAG "BlueV2Otto"

void BlueV2OttoDisplay::SetupUI() {
    OttoEmojiDisplay::SetupUI();

    auto* light = LvglThemeManager::GetInstance().GetTheme("light");
    if (light != nullptr) {
        SetTheme(light);
    }
    SetEmotion(BLUE_V2_DEFAULT_EMOTION);
    ESP_LOGI(TAG, "Otto GIF bench mode (light theme, invert-safe)");
}

void BlueV2OttoDisplay::UpdateStatusBar(bool update_all) {
    (void)update_all;
    // No GetAudioCodec / I2S — display-only bench.
}

void BlueV2OttoDisplay::RestoreFace() {
    SetEmotion(BLUE_V2_DEFAULT_EMOTION);
    RefreshNow();
}

void BlueV2OttoDisplay::HardRestoreFace() {
    RecoverPanel();
    SetEmotion(BLUE_V2_DEFAULT_EMOTION);
    RefreshNow();
    ESP_LOGI(TAG, "HardRestoreFace after SPI glitch");
}
