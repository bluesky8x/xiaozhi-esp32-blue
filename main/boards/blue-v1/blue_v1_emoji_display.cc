#include "blue_v1_emoji_display.h"

#include <esp_log.h>

#include <cstring>

#include "assets/lang_config.h"
#include "display/lvgl_display/lvgl_theme.h"

#define TAG "BlueV1Emoji"

BlueV1EmojiDisplay::BlueV1EmojiDisplay(esp_lcd_panel_io_handle_t panel_io,
                                         esp_lcd_panel_handle_t panel, int width, int height,
                                         int offset_x, int offset_y, bool mirror_x, bool mirror_y,
                                         bool swap_xy)
    : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y,
                    swap_xy) {}

void BlueV1EmojiDisplay::SetupUI() {
    if (setup_ui_called_) {
        ESP_LOGW(TAG, "SetupUI() called multiple times, skipping");
        return;
    }

    SpiLcdDisplay::SetupUI();

    auto* dark_theme = LvglThemeManager::GetInstance().GetTheme("dark");
    if (dark_theme != nullptr) {
        SetTheme(dark_theme);
    }

    current_status_ = Lang::Strings::INITIALIZING;
    llm_emotion_active_ = false;
    ApplyStateEmotion();
}

const char* BlueV1EmojiDisplay::EmotionForStatus(const char* status) {
    if (status == nullptr || status[0] == '\0') {
        return "neutral";
    }
    if (strcmp(status, Lang::Strings::LISTENING) == 0) {
        return "thinking";
    }
    if (strcmp(status, Lang::Strings::SPEAKING) == 0) {
        return "happy";
    }
    if (strcmp(status, Lang::Strings::CONNECTING) == 0) {
        return "thinking";
    }
    if (strcmp(status, Lang::Strings::STANDBY) == 0) {
        return "neutral";
    }
    if (strcmp(status, Lang::Strings::INITIALIZING) == 0) {
        return "neutral";
    }
    return "neutral";
}

void BlueV1EmojiDisplay::ApplyStateEmotion() {
    if (llm_emotion_active_) {
        return;
    }
    SpiLcdDisplay::SetEmotion(EmotionForStatus(current_status_.c_str()));
}

void BlueV1EmojiDisplay::SetEmotion(const char* emotion) {
    if (emotion == nullptr) {
        return;
    }

    // Server / alert emotions — show until next neutral reset from application state machine.
    if (strcmp(emotion, "neutral") != 0) {
        llm_emotion_active_ = true;
        SpiLcdDisplay::SetEmotion(emotion);
        return;
    }

    llm_emotion_active_ = false;
    ApplyStateEmotion();
}

LV_FONT_DECLARE(OTTO_ICON_FONT);

void BlueV1EmojiDisplay::SetStatus(const char* status) {
    if (status == nullptr) {
        ESP_LOGE(TAG, "SetStatus: status is nullptr");
        return;
    }

    current_status_ = status;

    {
        auto lvgl_theme = static_cast<LvglTheme*>(current_theme_);
        auto text_font = lvgl_theme->text_font()->font();
        DisplayLockGuard lock(this);

        if (strcmp(status, Lang::Strings::LISTENING) == 0) {
            lv_obj_set_style_text_font(status_label_, &OTTO_ICON_FONT, 0);
            lv_label_set_text(status_label_, "\xEF\x84\xB0");
            lv_obj_clear_flag(status_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(network_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(battery_label_, LV_OBJ_FLAG_HIDDEN);
        } else if (strcmp(status, Lang::Strings::SPEAKING) == 0) {
            lv_obj_set_style_text_font(status_label_, &OTTO_ICON_FONT, 0);
            lv_label_set_text(status_label_, "\xEF\x80\xA8");
            lv_obj_clear_flag(status_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(network_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(battery_label_, LV_OBJ_FLAG_HIDDEN);
        } else if (strcmp(status, Lang::Strings::CONNECTING) == 0) {
            lv_obj_set_style_text_font(status_label_, &OTTO_ICON_FONT, 0);
            lv_label_set_text(status_label_, "\xEF\x83\x81");
            lv_obj_clear_flag(status_label_, LV_OBJ_FLAG_HIDDEN);
        } else if (strcmp(status, Lang::Strings::STANDBY) == 0) {
            lv_obj_set_style_text_font(status_label_, text_font, 0);
            lv_label_set_text(status_label_, "");
            lv_obj_clear_flag(status_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(network_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(battery_label_, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_set_style_text_font(status_label_, text_font, 0);
            lv_label_set_text(status_label_, status);
            lv_obj_clear_flag(status_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(network_label_, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(battery_label_, LV_OBJ_FLAG_HIDDEN);
        }
    }

    ApplyStateEmotion();
}
