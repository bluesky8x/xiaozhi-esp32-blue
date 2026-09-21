#ifndef _BLUE_V4_OTTO_DISPLAY_H_
#define _BLUE_V4_OTTO_DISPLAY_H_

#include "../otto-robot/otto_emoji_display.h"

// Otto GIF face on the inverted ST7789 — light LVGL theme.
// Clone of blue-v2's BlueV2OttoDisplay, self-contained in the blue-v4 board dir
// (only the shared otto-robot emoji display/font assets come from outside).
class BlueV4OttoDisplay : public OttoEmojiDisplay {
public:
    using OttoEmojiDisplay::OttoEmojiDisplay;
    void SetupUI() override;
    void UpdateStatusBar(bool update_all = false) override;
    void RestoreFace();
    /** Panel reset + emotion reload after SPI is disturbed (servo I2C/PWM glitch). */
    void HardRestoreFace();
};

#endif  // _BLUE_V4_OTTO_DISPLAY_H_
