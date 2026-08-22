#include "display.h"
#include <esp_err.h>
#include <esp_log.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include "application.h"
#include "assets/lang_config.h"
#include "audio_codec.h"
#include "board.h"
#include "settings.h"

#define TAG "Display"

Display::Display() {}

Display::~Display() {}

void Display::SetStatus(const char* status) { ESP_LOGW(TAG, "SetStatus: %s", status); }

void Display::ShowNotification(const std::string& notification, int duration_ms) {
    ShowNotification(notification.c_str(), duration_ms);
}

void Display::ShowNotification(const char* notification, int duration_ms) {
    ESP_LOGW(TAG, "ShowNotification: %s", notification);
}

void Display::UpdateStatusBar(bool update_all) {}

#include "led/gpio_led.h"

void Display::SetEmotion(const char* emotion) {
    ESP_LOGW(TAG, "SetEmotion: %s", emotion);
    if (emotion == nullptr) {
        return;
    }

    auto* led = dynamic_cast<GpioLed*>(Board::GetInstance().GetLed());
    if (led == nullptr) {
        return;
    }

    if (strstr(emotion, "happy") != nullptr) {
        led->SetBrightness(80);
        led->BlinkFor(3000);
        return;
    }

    uint8_t brightness = 40;
    if (strcmp(emotion, "crying") == 0) {
        brightness = 8;    // Rất buồn (khóc) -> 8%
    } else if (strcmp(emotion, "shocked") == 0) {
        brightness = 12;   // Sốc -> 12%
    } else if (strcmp(emotion, "sleepy") == 0) {
        brightness = 15;   // Buồn ngủ -> 15%
    } else if (strcmp(emotion, "angry") == 0) {
        brightness = 20;   // Tức giận -> 20%
    } else if (strcmp(emotion, "sad") == 0) {
        brightness = 25;   // Buồn -> 25%
    } else if (strcmp(emotion, "neutral") == 0) {
        brightness = 40;   // Bình thường -> 40%
    } else if (strcmp(emotion, "thinking") == 0 || strcmp(emotion, "confused") == 0 || 
               strcmp(emotion, "surprised") == 0 || strcmp(emotion, "embarrassed") == 0) {
        brightness = 50;   // Suy nghĩ / thắc mắc -> 50%
    } else if (strcmp(emotion, "relaxed") == 0 || strcmp(emotion, "friendly") == 0 || 
               strcmp(emotion, "confident") == 0 || strcmp(emotion, "winking") == 0 || 
               strcmp(emotion, "kissy") == 0) {
        brightness = 65;   // Thư giãn / thân thiện -> 65%
    } else if (strcmp(emotion, "laughing") == 0 || strcmp(emotion, "funny") == 0 || 
               strcmp(emotion, "loving") == 0 || strcmp(emotion, "cool") == 0 || 
               strcmp(emotion, "excited") == 0) {
        brightness = 80;   // Rất vui sướng -> max 80%
    } else {
        brightness = 40;
    }

    led->SetBrightness(brightness);
    led->TurnOn();
}

void Display::SetChatMessage(const char* role, const char* content) {
    ESP_LOGW(TAG, "Role:%s", role);
    ESP_LOGW(TAG, "     %s", content);
}

void Display::ClearChatMessages() {
    // Default empty implementation, override in subclasses if needed
}

void Display::SetTheme(Theme* theme) {
    current_theme_ = theme;
    Settings settings("display", true);
    settings.SetString("theme", theme->name());
}

void Display::SetPowerSaveMode(bool on) { ESP_LOGW(TAG, "SetPowerSaveMode: %d", on); }
