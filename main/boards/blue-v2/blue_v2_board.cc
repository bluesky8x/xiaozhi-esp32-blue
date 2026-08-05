#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "../blue-v1/blue_v1_emoji_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "motor_controller.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "system_reset.h"
#include "power_save_timer.h"
#include "assets/lang_config.h"

#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#define TAG "BlueV2Board"

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

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

        if (DISPLAY_RST_PIN != GPIO_NUM_NC) {
            esp_lcd_panel_reset(panel);
        }
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);

        display_ = new BlueV1EmojiDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X,
                                          DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
                                          DISPLAY_SWAP_XY);
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
            GetDisplay()->SetPowerSaveMode(true);
            if (auto* backlight = GetBacklight()) {
                backlight->SetBrightness(1);
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

    void ToggleMicMute() {
        mic_muted_by_user_ = !mic_muted_by_user_;
        auto& app = Application::GetInstance();
        auto& audio_service = app.GetAudioService();
        auto codec = GetAudioCodec();

        if (mic_muted_by_user_) {
            audio_service.EnableWakeWordDetection(false);
            audio_service.EnableVoiceProcessing(false);
            codec->EnableInput(false);
            if (display_ != nullptr) {
                display_->ShowNotification(Lang::Strings::MUTED);
            }
            ESP_LOGI(TAG, "Mic muted by user");
        } else {
            codec->EnableInput(true);
            if (app.GetDeviceState() == kDeviceStateIdle) {
                audio_service.EnableWakeWordDetection(true);
            }
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
        ToggleMicMute();
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

        if (TOUCH_BUTTON_GPIO != GPIO_NUM_NC) {
            touch_button_.OnClick([this]() { HandleTouchClick(); });
        }
    }

    void InitializeTools() {
        static MotorController motor(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2);
#if DECOR_LED_GPIO != GPIO_NUM_NC
        static LampController decor_lamp(DECOR_LED_GPIO);
#endif
    }

public:
    BlueV2Board()
        : boot_button_(BOOT_BUTTON_GPIO, false, FACTORY_RESET_LONG_PRESS_MS),
          touch_button_(TOUCH_BUTTON_GPIO) {
        InitializeSystemReset();
        InitializeSpi();
        InitializeLcdDisplay();
        InitializePowerSaveTimer();
        InitializeButtons();
        InitializeTools();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                              AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT,
                                              AUDIO_I2S_GPIO_DIN);
        return &audio_codec;
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
