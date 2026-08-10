#ifndef BLUE_V2_OTTO_DISPLAY_H
#define BLUE_V2_OTTO_DISPLAY_H

#include "../otto-robot/otto_emoji_display.h"

// Otto GIF on inverted ST7789 — light LVGL theme (same approach as blue-v4).
class BlueV2OttoDisplay : public OttoEmojiDisplay {
public:
    using OttoEmojiDisplay::OttoEmojiDisplay;
    void SetupUI() override;
    void UpdateStatusBar(bool update_all = false) override;
    void RestoreFace();
};

#endif  // BLUE_V2_OTTO_DISPLAY_H
