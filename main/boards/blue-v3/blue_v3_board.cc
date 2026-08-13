#include "wifi_board.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "codecs/no_audio_codec.h"
#include "led/single_led.h"
#include "lcd_display.h"
#include "power_save_timer.h"
#include "system_reset.h"
#include "assets/lang_config.h"
#include "../blue-v2/blue_cloud_guard.h"
#include "../blue-v2/blue_v2_otto_display.h"

#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <vector>

#define TAG "BlueV3Board"

class BlueV3Board : public WifiBoard {
private:
    Button boot_button_;
    Button touch_button_;
    LcdDisplay* display_ = nullptr;
    PowerSaveTimer* power_save_timer_ = nullptr;
    bool mic_muted_by_user_ = false;
    int64_t last_touch_us_ = 0;

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
        }

        display_ = new BlueV2OttoDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X,
                                         DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
                                         DISPLAY_SWAP_XY);
        ESP_LOGI(TAG, "ST7789 + Otto (LCD + I2S, no motor/ToF — chat stability)");
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, POWER_SAVE_SLEEP_SECS, -1);
        power_save_timer_->OnEnterSleepMode([this]() {
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
    }

    void HandleTouchClick() {
        const int64_t now_us = esp_timer_get_time();
        if (now_us - last_touch_us_ < static_cast<int64_t>(TOUCH_DEBOUNCE_MS) * 1000) {
            return;
        }
        last_touch_us_ = now_us;

        if (power_save_timer_ != nullptr) {
            const bool was_sleeping = power_save_timer_->IsInSleepMode();
            power_save_timer_->WakeUp();
            if (was_sleeping) {
                Application::GetInstance().WakeWordInvoke("touch");
                return;
            }
        }

        RestoreMicIfMuted();
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

        if (TOUCH_BUTTON_GPIO != GPIO_NUM_NC) {
            touch_button_.OnPressDown([this]() { HandleTouchClick(); });
        }
    }

public:
    BlueV3Board()
        : boot_button_(BOOT_BUTTON_GPIO, false, FACTORY_RESET_LONG_PRESS_MS),
          touch_button_(TOUCH_BUTTON_GPIO, TOUCH_BUTTON_ACTIVE_HIGH) {
#if BLUE_V3_BLOCK_CLOUD_SERVERS
        BlueSanitizeStoredServerSettings();
#endif
        InitializeSpi();
        InitializeLcdDisplay();
        InitializePowerSaveTimer();
        InitializeButtons();
        ESP_LOGI(TAG, "Blue V3 ready — LCD + audio (no motor/ToF)");
    }

    void OnApplicationDisplayReady() override {
        ESP_LOGI(TAG, "Chat profile init done (no wake word, no motor MCP)");
    }

    Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    AudioCodec* GetAudioCodec() override {
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                              AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS,
                                              AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
        static bool configured = false;
        if (!configured) {
            audio_codec.SetInputGain(AUDIO_MIC_INPUT_GAIN);
            configured = true;
        }
        return &audio_codec;
    }

    Display* GetDisplay() override { return display_; }

    Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (power_save_timer_ != nullptr && level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(BlueV3Board);
