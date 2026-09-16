#include "blue_v2_face_display.h"

#include "application.h"
#include "assets/lang_config.h"
#include "lvgl_theme.h"
#include "material_symbols.h"
#include <noto_emoji.h>

#include <esp_log.h>
#include <cstring>

#define TAG "BlueV2Face"

LV_FONT_DECLARE(OTTO_ICON_FONT);

BlueV2FaceDisplay::BlueV2FaceDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                                       int width, int height, int offset_x, int offset_y,
                                       bool mirror_x, bool mirror_y, bool swap_xy)
    : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y,
                    swap_xy) {
    auto* dark = LvglThemeManager::GetInstance().GetTheme("dark");
    if (dark != nullptr) {
        current_theme_ = dark;
    }
}

BlueV2FaceDisplay::~BlueV2FaceDisplay() {
    if (face_plate_ != nullptr) {
        lv_anim_delete(face_plate_, AnimBorderOpa);
    }
}

void BlueV2FaceDisplay::AnimBorderOpa(void* obj, int32_t opa) {
    lv_obj_set_style_border_opa(static_cast<lv_obj_t*>(obj), static_cast<lv_opa_t>(opa), 0);
}

void BlueV2FaceDisplay::StartKeepAliveAnim() {
    if (face_plate_ == nullptr) {
        return;
    }
    lv_anim_t anim;
    lv_anim_init(&anim);
    lv_anim_set_var(&anim, face_plate_);
    lv_anim_set_values(&anim, LV_OPA_40, LV_OPA_COVER);
    lv_anim_set_duration(&anim, 900);
    lv_anim_set_playback_duration(&anim, 900);
    lv_anim_set_repeat_count(&anim, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_path_cb(&anim, lv_anim_path_ease_in_out);
    lv_anim_set_exec_cb(&anim, AnimBorderOpa);
    lv_anim_start(&anim);
}

void BlueV2FaceDisplay::SetupUI() {
    if (setup_ui_called_) {
        ESP_LOGW(TAG, "SetupUI() already called");
        return;
    }

    Display::SetupUI();
    DisplayLockGuard lock(this);

    auto* lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    const lv_font_t* text_font = lvgl_theme->text_font()->font();
    const lv_font_t* large_icon_font = lvgl_theme->large_icon_font()->font();

    lv_obj_t* screen = lv_screen_active();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);
    lv_obj_set_style_text_font(screen, text_font, 0);
    lv_obj_set_style_text_color(screen, lv_color_hex(0xFFFFFF), 0);

    container_ = lv_obj_create(screen);
    lv_obj_set_size(container_, LV_HOR_RES, LV_VER_RES);
    lv_obj_set_style_radius(container_, 0, 0);
    lv_obj_set_style_pad_all(container_, 0, 0);
    lv_obj_set_style_border_width(container_, 0, 0);
    lv_obj_set_style_bg_color(container_, lv_color_hex(0x000000), 0);

    face_plate_ = lv_obj_create(container_);
    lv_obj_set_size(face_plate_, 160, 160);
    lv_obj_set_style_radius(face_plate_, 80, 0);
    lv_obj_set_style_bg_color(face_plate_, lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_bg_opa(face_plate_, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(face_plate_, 4, 0);
    lv_obj_set_style_border_color(face_plate_, lv_color_hex(0xFFFF00), 0);
    lv_obj_set_style_border_opa(face_plate_, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(face_plate_, 0, 0);
    lv_obj_align(face_plate_, LV_ALIGN_CENTER, 0, -8);

    face_label_ = lv_label_create(face_plate_);
    lv_obj_set_style_text_font(face_label_, large_icon_font, 0);
    lv_obj_set_style_text_color(face_label_, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(face_label_, MATERIAL_SYMBOLS_ROBOT_2);
    lv_obj_center(face_label_);

    status_bar_ = lv_obj_create(screen);
    lv_obj_set_size(status_bar_, LV_HOR_RES, LV_SIZE_CONTENT);
    lv_obj_set_style_bg_opa(status_bar_, LV_OPA_60, 0);
    lv_obj_set_style_bg_color(status_bar_, lv_color_hex(0x000000), 0);
    lv_obj_set_style_border_width(status_bar_, 0, 0);
    lv_obj_set_style_pad_all(status_bar_, 4, 0);
    lv_obj_align(status_bar_, LV_ALIGN_TOP_MID, 0, 0);

    notification_label_ = lv_label_create(status_bar_);
    lv_obj_set_width(notification_label_, LV_HOR_RES * 0.9);
    lv_obj_set_style_text_align(notification_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(notification_label_, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(notification_label_, "");
    lv_obj_add_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(notification_label_, LV_ALIGN_CENTER, 0, 0);

    status_label_ = lv_label_create(status_bar_);
    lv_obj_set_width(status_label_, LV_HOR_RES * 0.9);
    lv_label_set_long_mode(status_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_align(status_label_, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(status_label_, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(status_label_, Lang::Strings::INITIALIZING);
    lv_obj_align(status_label_, LV_ALIGN_CENTER, 0, 0);

    ShowFaceLocked("neutral");
    StartKeepAliveAnim();
    RefreshNow();
    ESP_LOGI(TAG, "Face UI ready (V3-style, lv_anim keepalive)");
}

void BlueV2FaceDisplay::ShowFaceLocked(const char* emotion) {
    if (face_label_ == nullptr || emotion == nullptr) {
        return;
    }

    auto* lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    const char* utf8 = noto_emoji_get_utf8(emotion);
    const lv_font_t* font = lvgl_theme->emoji_font()->font();
    if (utf8 == nullptr) {
        utf8 = material_symbols_get_utf8(emotion);
        font = lvgl_theme->large_icon_font()->font();
    }
    if (utf8 == nullptr) {
        utf8 = MATERIAL_SYMBOLS_ROBOT_2;
        font = lvgl_theme->large_icon_font()->font();
    }

    lv_obj_set_style_text_font(face_label_, font, 0);
    lv_obj_set_style_text_color(face_label_, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(face_label_, utf8);
}

void BlueV2FaceDisplay::ShowFace(const char* emotion) {
    DisplayLockGuard lock(this);
    ShowFaceLocked(emotion);
    RefreshNow();
}

void BlueV2FaceDisplay::SetEmotion(const char* emotion) {
    Display::SetEmotion(emotion);
    ShowFace(emotion);
}

void BlueV2FaceDisplay::SetStatus(const char* status) {
    if (status == nullptr || status_label_ == nullptr) {
        return;
    }

    DisplayLockGuard lock(this);
    auto* lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    const lv_font_t* text_font = lvgl_theme->text_font()->font();

    if (strcmp(status, Lang::Strings::LISTENING) == 0) {
        lv_obj_set_style_text_font(status_label_, &OTTO_ICON_FONT, 0);
        lv_label_set_text(status_label_, "\xEF\x84\xB0");
        ShowFaceLocked("thinking");
    } else if (strcmp(status, Lang::Strings::SPEAKING) == 0) {
        lv_obj_set_style_text_font(status_label_, &OTTO_ICON_FONT, 0);
        lv_label_set_text(status_label_, "\xEF\x80\xA8");
        ShowFaceLocked("happy");
    } else if (strcmp(status, Lang::Strings::CONNECTING) == 0) {
        lv_obj_set_style_text_font(status_label_, &OTTO_ICON_FONT, 0);
        lv_label_set_text(status_label_, "\xEF\x83\x81");
        ShowFaceLocked("thinking");
    } else if (strcmp(status, Lang::Strings::STANDBY) == 0) {
        lv_obj_set_style_text_font(status_label_, text_font, 0);
        lv_label_set_text(status_label_, Lang::Strings::STANDBY);
        ShowFaceLocked("neutral");
    } else {
        lv_obj_set_style_text_font(status_label_, text_font, 0);
        lv_label_set_text(status_label_, status);
    }
    RefreshNow();
}

void BlueV2FaceDisplay::SetTheme(Theme* theme) {
    (void)theme;
    DisplayLockGuard lock(this);
    auto* lvgl_theme = static_cast<LvglTheme*>(current_theme_);
    if (lvgl_theme == nullptr || lvgl_theme->text_font() == nullptr) {
        return;
    }
    const lv_font_t* text_font = lvgl_theme->text_font()->font();
    lv_obj_t* screen = lv_screen_active();
    if (screen != nullptr) {
        lv_obj_set_style_text_font(screen, text_font, 0);
        lv_obj_set_style_text_color(screen, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);
    }
    if (container_ != nullptr) {
        lv_obj_set_style_bg_color(container_, lv_color_hex(0x000000), 0);
    }
    if (status_label_ != nullptr) {
        lv_obj_set_style_text_font(status_label_, text_font, 0);
        lv_obj_set_style_text_color(status_label_, lv_color_hex(0xFFFFFF), 0);
    }
    RefreshNow();
}

void BlueV2FaceDisplay::SetChatMessage(const char* role, const char* content) {
    (void)role;
    if (Application::GetInstance().IsDanceSessionActive()) {
        return;
    }
    DisplayLockGuard lock(this);
    if (status_label_ == nullptr) {
        return;
    }
    if (content == nullptr || content[0] == '\0') {
        lv_label_set_text(status_label_, Lang::Strings::STANDBY);
    } else {
        lv_label_set_text(status_label_, content);
    }
    RefreshNow();
}
