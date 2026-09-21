#include "application.h"
#include "assets/lang_config.h"
#include "backlight.h"
#include "button.h"
#include "config.h"
#include "led/gpio_led.h"
#include "power_save_timer.h"
#include "system_reset.h"
#include "wifi_board.h"

#include "blue_v4_cloud_guard.h"
#include "blue_v4_gait_guard.h"
#include "blue_v4_motor_compat.h"
#include "blue_v4_otto_display.h"
#include "blue_v4_tof_controller.h"
#include "codecs/no_audio_codec.h"
#include "gait_engine.h"
#include "lcd_display.h"
#include "power_controller.h"
#include "servo_controller.h"

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <vector>

#define TAG "BlueV4Board"

namespace {

void LogResetReason() {
    switch (esp_reset_reason()) {
        case ESP_RST_BROWNOUT:
            ESP_LOGW(TAG,
                     "Reset: BROWNOUT — rail sagged under load. Servos + amp + WiFi share "
                     "the 5 V/2 A module: slow the gait, relax servos when idle, or feed the "
                     "servo rail from the cell. See WIRING.md § Power.");
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

// OE# is pulled up in hardware; drive it high as early as possible so the servos
// cannot twitch during boot, SPI bring-up or a brownout reset.
void DisableServoOutputsEarly() {
#if (PCA9685_OE_GPIO != GPIO_NUM_NC)
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << PCA9685_OE_GPIO;
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
    gpio_set_level(PCA9685_OE_GPIO, 1);
    ESP_LOGI(TAG, "Servo outputs disabled early (OE# GPIO %d high)", PCA9685_OE_GPIO);
#endif
}

// The blue-v2 wheel motors are gone on blue-v4, but the old MX1508 wiring may still be
// connected. Floating H-bridge inputs let the outputs chatter (wheels twitch, current
// spikes, EMI on the shared I2C/servo rail), so hold IN1-IN4 LOW.
void HoldLegacyMx1508InputsLow() {
#if BLUE_V4_LEGACY_MX1508_PINS_LOW
    const gpio_num_t pins[] = {LEGACY_MX1508_IN1, LEGACY_MX1508_IN2, LEGACY_MX1508_IN3,
                               LEGACY_MX1508_IN4};
    for (gpio_num_t pin : pins) {
        gpio_reset_pin(pin);
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
        gpio_set_level(pin, 0);
    }
    ESP_LOGI(TAG, "Legacy MX1508 inputs held LOW (%d/%d/%d/%d) — disconnect its VMOT if unused",
             LEGACY_MX1508_IN1, LEGACY_MX1508_IN2, LEGACY_MX1508_IN3, LEGACY_MX1508_IN4);
#endif
}

}  // namespace

class BlueV4Board : public WifiBoard {
public:
    BlueV4Board()
        : boot_button_(BOOT_BUTTON_GPIO, false, FACTORY_RESET_LONG_PRESS_MS),
          touch_button_(TOUCH_BUTTON_GPIO, TOUCH_BUTTON_ACTIVE_HIGH) {
#if BLUE_V4_BLOCK_CLOUD_SERVERS
        BlueSanitizeStoredServerSettings();
#endif
        LogResetReason();
        DisableServoOutputsEarly();
        HoldLegacyMx1508InputsLow();

        InitializeSystemReset();
        InitializeI2cBus();
        InitializeSpi();
        InitializeLcdDisplay();
        InitializePowerSaveTimer();
        InitializeButtons();
        // Servo controller + gait engine + MCP tools are deferred until activation
        // (~10 s) so the I2C traffic cannot disturb the LCD/audio bring-up.
    }

    void OnApplicationDisplayReady() override { InitializeRobotTools(); }

    Led* GetLed() override {
        // LEDC_TIMER_1 / CHANNEL_1 — CHANNEL_0 is taken by PwmBacklight.
        static GpioLed led(BUILTIN_LED_GPIO, 0, LEDC_TIMER_1, LEDC_CHANNEL_1);
        return &led;
    }

    AudioCodec* GetAudioCodec() override {
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                                              AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS,
                                              AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
        // AudioCodec::input_gain_ defaults to 0.0 — without this the uplink is silence.
        static bool configured = false;
        if (!configured) {
            audio_codec.SetInputGain(AUDIO_MIC_INPUT_GAIN);
            ESP_LOGI(TAG, "INMP441 input gain=%.1f (duplex WS=%d BCLK=%d SD=%d)",
                     static_cast<double>(AUDIO_MIC_INPUT_GAIN), AUDIO_I2S_GPIO_WS,
                     AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_DIN);
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

private:
    Button boot_button_;
    Button touch_button_;
    LcdDisplay* display_ = nullptr;
    PowerSaveTimer* power_save_timer_ = nullptr;
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    int64_t last_touch_us_ = 0;

    void InitializeSystemReset() {
#if RESET_NVS_BUTTON_GPIO != GPIO_NUM_NC || RESET_FACTORY_BUTTON_GPIO != GPIO_NUM_NC
        static SystemReset system_reset(RESET_NVS_BUTTON_GPIO, RESET_FACTORY_BUTTON_GPIO);
        system_reset.CheckButtons();
#endif
    }

    // The board owns the I2C bus; the PCA9685 (0x40) and the ToF sensor (0x29)
    // are injected with this same handle.
    void InitializeI2cBus() {
        if (I2C_SENSOR_SDA_PIN == GPIO_NUM_NC || I2C_SENSOR_SCL_PIN == GPIO_NUM_NC) {
            ESP_LOGW(TAG, "I2C pins not configured — servo driver will be unavailable");
            return;
        }

        i2c_master_bus_config_t bus_cfg = {};
        bus_cfg.i2c_port = I2C_SENSOR_PORT;
        bus_cfg.sda_io_num = I2C_SENSOR_SDA_PIN;
        bus_cfg.scl_io_num = I2C_SENSOR_SCL_PIN;
        bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_cfg.glitch_ignore_cnt = 7;
        bus_cfg.flags.enable_internal_pullup = true;

        const esp_err_t err = i2c_new_master_bus(&bus_cfg, &i2c_bus_);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
            i2c_bus_ = nullptr;
            return;
        }
        ESP_LOGI(TAG, "I2C bus on SDA %d / SCL %d (%d Hz)", I2C_SENSOR_SDA_PIN, I2C_SENSOR_SCL_PIN,
                 I2C_SENSOR_SPEED_HZ);
    }

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
        // Double pulse: the ESP resets on USB reconnect but the LCD module may stay
        // powered and keep stale ST7789 state.
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

        display_ = new BlueV4OttoDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                         DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                         DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        ESP_LOGI(TAG, "ST7789 ready (MOSI %d, CLK %d, DC %d, RST %d, BL %d, CS tied to GND)",
                 DISPLAY_MOSI_PIN, DISPLAY_CLK_PIN, DISPLAY_DC_PIN, DISPLAY_RST_PIN,
                 DISPLAY_BACKLIGHT_PIN);
    }

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, POWER_SAVE_SLEEP_SECS, -1);
        power_save_timer_->OnEnterSleepMode([this]() {
            ESP_LOGI(TAG, "Sleep: dim backlight to %d, relax servos", POWER_SAVE_DIM_BRIGHTNESS);
            GetDisplay()->SetPowerSaveMode(true);
            if (auto* backlight = GetBacklight()) {
                backlight->SetBrightness(POWER_SAVE_DIM_BRIGHTNESS);
            }
            // Never hold servo torque while asleep — protects the gearboxes and the rail.
            if (auto* servos = ServoController::Instance()) {
                servos->Relax();
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

        // While the gait is running, a tap is a stop request rather than a chat toggle.
        if (auto* gait = GaitEngine::Instance()) {
            if (gait->IsBusy()) {
                gait->EnqueueStop();
                ESP_LOGI(TAG, "Touch: gait stop");
                return;
            }
        }

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
            // Cut torque before rebooting so the robot does not collapse under power.
            if (auto* servos = ServoController::Instance()) {
                servos->Relax();
            }
            SystemReset::FactoryResetAndReboot(3);
        });

        if (TOUCH_BUTTON_GPIO != GPIO_NUM_NC) {
            touch_button_.OnPressDown([this]() { HandleTouchClick(); });
            ESP_LOGI(TAG, "Touch button ready on GPIO %d (active_%s, press-down)",
                     TOUCH_BUTTON_GPIO, TOUCH_BUTTON_ACTIVE_HIGH ? "high" : "low");
        }
    }

    void InitializeRobotTools() {
        static ServoController servos;
        static GaitEngine gait;
        static PowerController power_ctrl(power_save_timer_);

        if (!servos.Init(i2c_bus_)) {
            ESP_LOGE(TAG, "Servo controller init failed — servo/gait tools unavailable");
            return;
        }
        if (!servos.HasHardware()) {
            ESP_LOGW(TAG, "PCA9685 not detected on 0x%02X — tools registered but inert",
                     PCA9685_I2C_ADDR);
        }
        gait.Init(&servos);
#if BLUE_V4_MOTOR_COMPAT_TOOLS
        // blue-v2 compatible self.motor.* tools: the existing server drives the gait.
        static BlueV4MotorCompat motor_compat(&gait);
        motor_compat.RegisterMcpTools();
#endif

        // Front range sensor on the same board-owned bus (0x29), then the gait guard
        // that stops walking on an obstacle or a drop-off.
        if (BlueV4TofController::Instance().Init(i2c_bus_)) {
            static BlueV4GaitGuard tof_guard(&gait);
            if (!tof_guard.Start()) {
                ESP_LOGW(TAG, "ToF gait guard not active");
            }
        } else {
            ESP_LOGW(TAG, "ToF sensor not available (optional)");
        }

        ESP_LOGI(TAG, "Robot tools initialized (servo + gait + tof + power)");
    }
};

DECLARE_BOARD(BlueV4Board);
