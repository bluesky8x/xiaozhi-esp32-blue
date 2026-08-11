#include "blue_v1_emoji_display.h"

#include <esp_log.h>

#include <cstring>

#include "assets/lang_config.h"
#include "display/lvgl_display/lvgl_theme.h"
#include "material_symbols.h"
#include "settings.h"
#include <noto_emoji.h>

#include <sdkconfig.h>
#if CONFIG_BOARD_TYPE_BLUE_V2
#include "boards/blue-v2/config.h"
#endif

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

#if CONFIG_BOARD_TYPE_BLUE_V2
    {
        Settings settings("display", true);
        settings.SetString("theme", "light");
    }
    if (auto* ui_theme = LvglThemeManager::GetInstance().GetTheme("light")) {
        current_theme_ = ui_theme;
    }
#endif

    SpiLcdDisplay::SetupUI();

#if CONFIG_BOARD_TYPE_BLUE_V2
    ApplyInvertSafeChrome();
#endif

#if !CONFIG_BOARD_TYPE_BLUE_V2
    auto* ui_theme = LvglThemeManager::GetInstance().GetTheme("dark");
    if (ui_theme != nullptr) {
        SetTheme(ui_theme);
    }
#endif

    current_status_ = Lang::Strings::INITIALIZING;
    llm_emotion_active_ = false;
    ApplyStateEmotion();
#if CONFIG_BOARD_TYPE_BLUE_V2
    EnsureEmotionVisible();
    RefreshNow();
#if BLUE_V2_ICON_EMOJI
    ESP_LOGI(TAG, "Blue V2 emoji UI ready (icon mode, invert-safe)");
#else
    ESP_LOGI(TAG, "Blue V2 emoji UI ready (Otto GIF, light theme)");
#endif
#endif
}

#if CONFIG_BOARD_TYPE_BLUE_V2
void BlueV1EmojiDisplay::ApplyInvertSafeChrome() {
    DisplayLockGuard lock(this);
    lv_obj_t* screen = lv_screen_active();
    if (screen != nullptr) {
        lv_obj_set_style_bg_color(screen, lv_color_hex(0xFFFFFF), 0);
    }
    if (container_ != nullptr) {
        lv_obj_set_style_bg_image_src(container_, nullptr, 0);
        lv_obj_set_style_bg_color(container_, lv_color_hex(0xFFFFFF), 0);
    }
    if (emoji_box_ != nullptr) {
        lv_obj_set_style_bg_color(emoji_box_, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_opa(emoji_box_, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(emoji_box_, 20, 0);
        lv_obj_set_style_pad_all(emoji_box_, 10, 0);
        lv_obj_remove_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
    }
}

void BlueV1EmojiDisplay::ShowIconEmotion(const char* emotion) {
    if (emotion == nullptr || emoji_label_ == nullptr || emoji_image_ == nullptr) {
        return;
    }

    DisplayLockGuard lock(this);
    ApplyInvertSafeChrome();

    if (gif_controller_) {
        gif_controller_->Stop();
        gif_controller_.reset();
    }

    auto* lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    const char* utf8 = noto_emoji_get_utf8(emotion);
    const lv_font_t* emotion_font = lvgl_theme->emoji_font()->font();
    if (utf8 == nullptr) {
        utf8 = material_symbols_get_utf8(emotion);
        emotion_font = lvgl_theme->large_icon_font()->font();
    }
    if (utf8 == nullptr) {
        utf8 = MATERIAL_SYMBOLS_ROBOT_2;
        emotion_font = lvgl_theme->large_icon_font()->font();
    }

    lv_obj_set_style_text_font(emoji_label_, emotion_font, 0);
    lv_obj_set_style_text_color(emoji_label_, lv_color_hex(0x000000), 0);
    lv_label_set_text(emoji_label_, utf8);
    lv_obj_add_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_remove_flag(emoji_label_, LV_OBJ_FLAG_HIDDEN);
    RefreshNow();
}

void BlueV1EmojiDisplay::EnsureEmotionVisible() {
    DisplayLockGuard lock(this);
    if (emoji_image_ == nullptr || emoji_label_ == nullptr) {
        return;
    }
    const bool image_hidden = lv_obj_has_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);
    const bool label_hidden = lv_obj_has_flag(emoji_label_, LV_OBJ_FLAG_HIDDEN);
    if (!image_hidden || !label_hidden) {
        return;
    }
    ESP_LOGW(TAG, "Emoji not visible — showing robot icon fallback");
    ShowIconEmotion("neutral");
}

void BlueV1EmojiDisplay::SetTheme(Theme* theme) {
    (void)theme;
    auto* light = LvglThemeManager::GetInstance().GetTheme("light");
    if (light == nullptr) {
        return;
    }
    light->set_background_color(lv_color_hex(0xFFFFFF));
    light->set_text_color(lv_color_hex(0x000000));
    light->set_chat_background_color(lv_color_hex(0xE0E0E0));
    light->set_background_image(nullptr);
    current_theme_ = light;
    SpiLcdDisplay::SetTheme(light);
    ApplyInvertSafeChrome();
    ApplyStateEmotion();
    EnsureEmotionVisible();
    RefreshNow();
}

void BlueV1EmojiDisplay::SetChatMessage(const char* role, const char* content) {
    (void)role;
    DisplayLockGuard lock(this);
    if (chat_message_label_ == nullptr || bottom_bar_ == nullptr) {
        return;
    }
    if (content == nullptr || content[0] == '\0') {
        lv_label_set_text(chat_message_label_, "");
        lv_obj_add_flag(bottom_bar_, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_label_set_text(chat_message_label_, content);
    if (!hide_subtitle_) {
        lv_obj_remove_flag(bottom_bar_, LV_OBJ_FLAG_HIDDEN);
    }
    ApplyInvertSafeChrome();
    RefreshNow();
}
#endif

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
#if CONFIG_BOARD_TYPE_BLUE_V2 && BLUE_V2_ICON_EMOJI
    ShowIconEmotion(EmotionForStatus(current_status_.c_str()));
#else
    SpiLcdDisplay::SetEmotion(EmotionForStatus(current_status_.c_str()));
#endif
}

void BlueV1EmojiDisplay::SetEmotion(const char* emotion) {
    if (emotion == nullptr) {
        return;
    }

    if (strcmp(emotion, "neutral") != 0) {
        llm_emotion_active_ = true;
#if CONFIG_BOARD_TYPE_BLUE_V2 && BLUE_V2_ICON_EMOJI
        ShowIconEmotion(emotion);
#else
        SpiLcdDisplay::SetEmotion(emotion);
#if CONFIG_BOARD_TYPE_BLUE_V2
        EnsureEmotionVisible();
        RefreshNow();
#endif
#endif
        return;
    }

    llm_emotion_active_ = false;
    ApplyStateEmotion();
#if CONFIG_BOARD_TYPE_BLUE_V2 && !BLUE_V2_ICON_EMOJI
    EnsureEmotionVisible();
    RefreshNow();
#endif
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
#if CONFIG_BOARD_TYPE_BLUE_V2
    ApplyInvertSafeChrome();
    RefreshNow();
#endif
}
