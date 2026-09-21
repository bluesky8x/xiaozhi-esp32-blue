#include "blue_v4_otto_display.h"

#include "config.h"
#include "lvgl_theme.h"

#include <esp_log.h>

#define TAG "BlueV4Otto"

void BlueV4OttoDisplay::SetupUI() {
    OttoEmojiDisplay::SetupUI();

    auto* light = LvglThemeManager::GetInstance().GetTheme("light");
    if (light != nullptr) {
        SetTheme(light);
    }
    SetEmotion(BLUE_V4_DEFAULT_EMOTION);
    ESP_LOGI(TAG, "Otto GIF face ready (light theme, invert-safe)");
}

void BlueV4OttoDisplay::UpdateStatusBar(bool update_all) {
    (void)update_all;
    // Status bar (battery/network text) is not drawn — same as blue-v2's robot UI.
}

void BlueV4OttoDisplay::RestoreFace() {
    SetEmotion(BLUE_V4_DEFAULT_EMOTION);
    RefreshNow();
}

void BlueV4OttoDisplay::HardRestoreFace() {
    RecoverPanel();
    SetEmotion(BLUE_V4_DEFAULT_EMOTION);
    RefreshNow();
    ESP_LOGI(TAG, "HardRestoreFace after SPI glitch");
}
