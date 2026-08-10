#ifndef BLUE_V2_FACE_DISPLAY_H
#define BLUE_V2_FACE_DISPLAY_H

#include "lcd_display.h"

/** Minimal invert-safe face UI — same drawing model as BlueV3TestDisplay. */
class BlueV2FaceDisplay : public SpiLcdDisplay {
public:
    BlueV2FaceDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, int width,
                      int height, int offset_x, int offset_y, bool mirror_x, bool mirror_y,
                      bool swap_xy);
    ~BlueV2FaceDisplay() override;

    void SetupUI() override;
    void SetEmotion(const char* emotion) override;
    void SetStatus(const char* status) override;
    void SetTheme(Theme* theme) override;
    void SetChatMessage(const char* role, const char* content) override;

private:
    void ShowFaceLocked(const char* emotion);
    void ShowFace(const char* emotion);
    void StartKeepAliveAnim();
    static void AnimBorderOpa(void* obj, int32_t opa);

    lv_obj_t* face_plate_ = nullptr;
    lv_obj_t* face_label_ = nullptr;
};

#endif  // BLUE_V2_FACE_DISPLAY_H
