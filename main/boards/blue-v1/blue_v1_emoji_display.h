#pragma once

#include <string>

#include "display/lcd_display.h"

/**
 * Blue V1 — Otto GIF face on ST7789 240×240.
 * Emotions follow device state; LLM emotions (non-neutral) override until next state change.
 */
class BlueV1EmojiDisplay : public SpiLcdDisplay {
public:
    BlueV1EmojiDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, int width,
                         int height, int offset_x, int offset_y, bool mirror_x, bool mirror_y,
                         bool swap_xy);

    ~BlueV1EmojiDisplay() override = default;

    void SetupUI() override;
    void SetStatus(const char* status) override;
    void SetEmotion(const char* emotion) override;

private:
    std::string current_status_;
    bool llm_emotion_active_ = false;

    void ApplyStateEmotion();
    static const char* EmotionForStatus(const char* status);
};
