#include "wifi_board.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "led/single_led.h"
#include "system_reset.h"
#include "power_save_timer.h"
#include "assets/lang_config.h"

#if BLUE_V2_LCD_TEST_SCREEN || BLUE_V2_USE_V3_DISPLAY
#include "../blue-v3/blue_v3_test_display.h"
#endif
#if BLUE_V2_LCD_TEST_SCREEN || (BLUE_V2_OTTO_LCD_ONLY && !BLUE_V2_OTTO_AUDIO_ENABLE)
#include "audio_codec.h"
#endif
#if BLUE_V2_OTTO_LCD_ONLY && BLUE_V2_OTTO_AUDIO_ENABLE
#include "codecs/no_audio_codec.h"
#endif
#if BLUE_V2_OTTO_LCD_ONLY && BLUE_V2_OTTO_MOTOR_ENABLE
#include "motor_controller.h"
#include "../blue-v1/power_controller.h"
#endif
#if !BLUE_V2_LCD_TEST_SCREEN && !BLUE_V2_OTTO_LCD_ONLY
#include "codecs/no_audio_codec.h"
#if !BLUE_V2_USE_V3_DISPLAY
#include "blue_v2_face_display.h"
#endif
#include "motor_controller.h"
#include "tof_controller.h"
#include "tof_motor_guard.h"
#include "lamp_controller.h"
#include "../blue-v1/power_controller.h"
#endif
#if BLUE_V2_OTTO_LCD_ONLY
#include "blue_v2_otto_display.h"
#endif
#include "lcd_display.h"

#include <esp_log.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <vector>

#define TAG "BlueV2Board"

namespace {

void LogResetReason() {
    switch (esp_reset_reason()) {
        case ESP_RST_BROWNOUT:
            ESP_LOGW(TAG, "Reset: BROWNOUT — pin sụt áp khi tải cao (WiFi + loa + motor). Xem WIRING.md § Nguồn.");
            break;
        case ESP_RST_POWERON:
            ESP_LOGI(TAG, "Reset: power-on");
            break;
        case ESP_RST_SW:
            ESP_LOGI(TAG, "Reset: software");
            break;
        case ESP_RST_PANIC:
            ESP_LOGW(TAG, "Reset: panic/assert");
            break;
        case ESP_RST_WDT:
            ESP_LOGW(TAG, "Reset: watchdog");
            break;
        default:
            ESP_LOGI(TAG, "Reset reason: %d", static_cast<int>(esp_reset_reason()));
            break;
    }
}

}  // namespace

#if BLUE_V2_LCD_TEST_SCREEN || (BLUE_V2_OTTO_LCD_ONLY && !BLUE_V2_OTTO_AUDIO_ENABLE)

namespace {

class DisplayTestAudioCodec : public AudioCodec {
public:
    DisplayTestAudioCodec() {
        input_sample_rate_ = 16000;
        output_sample_rate_ = 16000;
    }

protected:
    int Read(int16_t* dest, int samples) override {
        (void)dest;
        (void)samples;
        return 0;
    }

    int Write(const int16_t* data, int samples) override {
        (void)data;
        (void)samples;
        return 0;
    }
};

}  // namespace

#endif

// TTP223 long-press was removed — it conflicted with click and only toggled mute.

class BlueV2Board : public WifiBoard {
private:
    Button boot_button_;
    Button touch_button_;
    LcdDisplay* display_ = nullptr;
    PowerSaveTimer* power_save_timer_ = nullptr;
    bool mic_muted_by_user_ = false;

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    static void HardwareResetDisplay() {
#if DISPLAY_RST_PIN != GPIO_NUM_NC
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = 1ULL << DISPLAY_RST_PIN;
        cfg.mode = GPIO_MODE_OUTPUT;
        gpio_config(&cfg);
        // Double pulse: ESP resets on USB reconnect but LCD module may stay powered (ST7789 stale).
        for (int pulse = 0; pulse < 2; pulse++) {
            gpio_set_level(DISPLAY_RST_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(DISPLAY_RST_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(150));
        }
#endif
    }

    void InitializeLcdDisplay() {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->SetBrightness(0);
        }

        vTaskDelay(pdMS_TO_TICKS(120));
        HardwareResetDisplay();

        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 10 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

        {
            // INVERT_COLOR=true: RGB565 0x0000 → white on panel; 0xFFFF → black (smoke test must match).
            const uint16_t fill = DISPLAY_INVERT_COLOR ? 0x0000 : 0xFFFF;
            std::vector<uint16_t> line(DISPLAY_WIDTH, fill);
            for (int y = 0; y < DISPLAY_HEIGHT; y++) {
                ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, y, DISPLAY_WIDTH, y + 1, line.data()));
            }
            if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
                GetBacklight()->SetBrightness(100);
                vTaskDelay(pdMS_TO_TICKS(200));
                GetBacklight()->SetBrightness(0);
            }
            ESP_LOGI(TAG, "Panel SPI smoke test OK (fill=0x%04X, invert=%d)", fill,
                     DISPLAY_INVERT_COLOR ? 1 : 0);
        }

#if BLUE_V2_OTTO_LCD_ONLY
        display_ = new BlueV2OttoDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X,
                                           DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
                                           DISPLAY_SWAP_XY);
#if BLUE_V2_OTTO_MOTOR_ENABLE
        ESP_LOGI(TAG, "ST7789 + Otto bench (LCD + I2S + motor, no wake word)");
#elif BLUE_V2_OTTO_AUDIO_ENABLE
        ESP_LOGI(TAG, "ST7789 + Otto bench (LCD + I2S, no motor/wake word)");
#else
        ESP_LOGI(TAG, "ST7789 + Otto GIF bench (LCD only, no I2S/motor/wake word)");
#endif
#elif BLUE_V2_LCD_TEST_SCREEN || BLUE_V2_USE_V3_DISPLAY
        display_ = new BlueV3TestDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X,
                                         DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
                                         DISPLAY_SWAP_XY);
#if BLUE_V2_LCD_TEST_SCREEN
        ESP_LOGI(TAG, "ST7789 init OK — LCD TEST SCREEN mode (BLUE_V2_LCD_TEST_SCREEN=1)");
#else
        ESP_LOGI(TAG, "ST7789 init OK — BlueV3TestDisplay (robot, invert-safe)");
#endif
#else
        display_ = new BlueV2FaceDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X,
                                         DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
                                         DISPLAY_SWAP_XY);
        ESP_LOGI(TAG, "ST7789 init OK (RES=GPIO%d, SPI mode %d, 9/10/8, BL=17)", DISPLAY_RST_PIN,
                 DISPLAY_SPI_MODE);
#endif
    }

    void InitializeSystemReset() {
#if RESET_NVS_BUTTON_GPIO != GPIO_NUM_NC || RESET_FACTORY_BUTTON_GPIO != GPIO_NUM_NC
        static SystemReset system_reset(RESET_NVS_BUTTON_GPIO, RESET_FACTORY_BUTTON_GPIO);
        system_reset.CheckButtons();
#endif
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, POWER_SAVE_SLEEP_SECS, -1);
        power_save_timer_->OnEnterSleepMode([this]() {
            ESP_LOGI(TAG, "Display sleep: sleepy face, BL=%d", POWER_SAVE_DIM_BRIGHTNESS);
            GetDisplay()->SetPowerSaveMode(true);
            if (auto* backlight = GetBacklight()) {
                backlight->SetBrightness(POWER_SAVE_DIM_BRIGHTNESS);
            }
        });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            if (auto* backlight = GetBacklight()) {
                backlight->RestoreBrightness();
            }
        });
        power_save_timer_->SetEnabled(true);
    }

    void ResumeMicAfterUserMute() {
        auto& app = Application::GetInstance();
        auto& audio_service = app.GetAudioService();
        auto* codec = GetAudioCodec();
        const auto state = app.GetDeviceState();

        codec->EnableInput(true);
        // Duplex I2S shares BCLK/WS — RX needs TX enabled after a full mute/power-down.
        if (codec->duplex()) {
            codec->EnableOutput(true);
        }

        if (state == kDeviceStateListening || state == kDeviceStateSpeaking ||
            state == kDeviceStateConnecting) {
            audio_service.EnableVoiceProcessing(true);
        } else {
            audio_service.EnableWakeWordDetection(true);
        }
    }

    void RestoreMicIfMuted() {
        if (!mic_muted_by_user_) {
            return;
        }
        mic_muted_by_user_ = false;
        ResumeMicAfterUserMute();
        if (display_ != nullptr) {
            display_->ShowNotification("Mic on");
        }
        ESP_LOGI(TAG, "Mic restored for chat");
    }

    void ToggleMicMute() {
        mic_muted_by_user_ = !mic_muted_by_user_;
        auto& app = Application::GetInstance();
        auto& audio_service = app.GetAudioService();
        auto* codec = GetAudioCodec();

        if (mic_muted_by_user_) {
            audio_service.EnableWakeWordDetection(false);
            audio_service.EnableVoiceProcessing(false);
            // Duplex boards: keep I2S clock running so unmute does not stall RX.
            if (!codec->duplex()) {
                codec->EnableInput(false);
            }
            if (display_ != nullptr) {
                display_->ShowNotification(Lang::Strings::MUTED);
            }
            ESP_LOGI(TAG, "Mic muted by user");
        } else {
            ResumeMicAfterUserMute();
            if (display_ != nullptr) {
                display_->ShowNotification("Mic on");
            }
            ESP_LOGI(TAG, "Mic unmuted by user");
        }
    }

    void HandleTouchClick() {
        if (power_save_timer_ != nullptr) {
            const bool was_sleeping = power_save_timer_->IsInSleepMode();
            power_save_timer_->WakeUp();
            if (was_sleeping) {
                Application::GetInstance().WakeWordInvoke("touch");
                return;
            }
        }

        RestoreMicIfMuted();
        ESP_LOGI(TAG, "Touch: toggle chat");
        Application::GetInstance().ToggleChatState();
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            if (power_save_timer_ != nullptr) {
                power_save_timer_->WakeUp();
            }
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

        boot_button_.OnLongPress([this]() {
            if (display_ != nullptr) {
                display_->ShowNotification("Factory reset...");
            }
            SystemReset::FactoryResetAndReboot(3);
        });

        if (TOUCH_BUTTON_GPIO != GPIO_NUM_NC &&
            (!BLUE_V2_OTTO_LCD_ONLY || BLUE_V2_OTTO_TOUCH_ENABLE)) {
            touch_button_.OnClick([this]() { HandleTouchClick(); });
            ESP_LOGI(TAG, "Touch button ready on GPIO %d", TOUCH_BUTTON_GPIO);
        }
    }

#if BLUE_V2_OTTO_LCD_ONLY && BLUE_V2_OTTO_MOTOR_ENABLE
    void InitializeOttoBenchTools() {
        static MotorController motor(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2);
        static PowerController power_ctrl(power_save_timer_);
        bench_motor_ = &motor;
        (void)motor;
        ESP_LOGI(TAG, "Motor MCP tools registered (MX1508 GPIO 11-14, bench mode)");
    }

    MotorController* bench_motor_ = nullptr;
#endif

#if !BLUE_V2_LCD_TEST_SCREEN && !BLUE_V2_OTTO_LCD_ONLY
    void InitializeTools() {
        static MotorController motor(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2);
        if (TofController::Instance().Init()) {
            static TofMotorGuard tof_guard(&motor);
            if (!tof_guard.Start()) {
                ESP_LOGW(TAG, "ToF obstacle guard not active");
            }
        } else {
            ESP_LOGW(TAG, "ToF sensor not available (optional)");
        }
#if DECOR_LED_GPIO != GPIO_NUM_NC
        static LampController decor_lamp(DECOR_LED_GPIO);
#endif
        static PowerController power_ctrl(power_save_timer_);
    }
#endif

public:
    BlueV2Board()
        : boot_button_(BOOT_BUTTON_GPIO, false, FACTORY_RESET_LONG_PRESS_MS),
          touch_button_(TOUCH_BUTTON_GPIO) {
        LogResetReason();
        InitializeSystemReset();
        InitializeSpi();
        InitializeLcdDisplay();
        InitializePowerSaveTimer();
        InitializeButtons();
        // I2S deferred until activation — motor GPIO is safe at boot.
    }

    void OnApplicationDisplayReady() override {
#if !BLUE_V2_LCD_TEST_SCREEN && !BLUE_V2_OTTO_LCD_ONLY
        InitializeTools();
        ESP_LOGI(TAG, "Robot tools initialized (deferred until activation)");
#elif BLUE_V2_OTTO_LCD_ONLY
#if BLUE_V2_OTTO_MOTOR_ENABLE
        InitializeOttoBenchTools();
#endif
        ESP_LOGI(TAG, "Otto bench init done (no wake word)");
#endif
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#if BLUE_V2_LCD_TEST_SCREEN || (BLUE_V2_OTTO_LCD_ONLY && !BLUE_V2_OTTO_AUDIO_ENABLE)
        static DisplayTestAudioCodec codec;
        return &codec;
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                              AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
                                              AUDIO_I2S_GPIO_DIN);
        static bool configured = false;
        if (!configured) {
            audio_codec.SetInputGain(AUDIO_MIC_INPUT_GAIN);
            ESP_LOGI(TAG, "INMP441 input gain=%.1f (duplex WS=%d BCLK=%d SD=%d)",
                     AUDIO_MIC_INPUT_GAIN, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_DIN);
            configured = true;
        }
        return &audio_codec;
#endif
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (power_save_timer_ != nullptr && level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(BlueV2Board);
