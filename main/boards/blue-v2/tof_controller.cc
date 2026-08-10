#include "tof_controller.h"

#include "config.h"
#include "mcp_server.h"
#include "settings.h"

#include <cstdio>
#include <cstdlib>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" {
#include <vl53l0x.h>
}

#define TAG "TofCtrl"
#define NVS_NS "blue_tof"

namespace {

void HoldXShutLow(gpio_num_t pin) {
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = 1ULL << pin;
    cfg.mode = GPIO_MODE_OUTPUT;
    gpio_config(&cfg);
    gpio_set_level(pin, 0);
}

void ReleaseXShut(gpio_num_t pin) {
    gpio_set_level(pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
}

void InitFrontXShut() {
    if (TOF_FRONT_XSHUT_GPIO == GPIO_NUM_NC) {
        return;
    }
    HoldXShutLow(TOF_FRONT_XSHUT_GPIO);
    if (TOF_REAR_XSHUT_GPIO != GPIO_NUM_NC) {
        HoldXShutLow(TOF_REAR_XSHUT_GPIO);
    }
    ReleaseXShut(TOF_FRONT_XSHUT_GPIO);
}

void InitDualXShutHoldAllLow() {
    if (TOF_FRONT_XSHUT_GPIO != GPIO_NUM_NC) {
        HoldXShutLow(TOF_FRONT_XSHUT_GPIO);
    }
    if (TOF_REAR_SENSOR_ENABLE && TOF_REAR_XSHUT_GPIO != GPIO_NUM_NC) {
        HoldXShutLow(TOF_REAR_XSHUT_GPIO);
    }
}

}  // namespace

bool TofController::InitRearSensor(i2c_master_bus_handle_t bus) {
#if !TOF_REAR_SENSOR_ENABLE
    (void)bus;
    return false;
#else
    if (TOF_REAR_XSHUT_GPIO == GPIO_NUM_NC) {
        return false;
    }

    if (TOF_FRONT_XSHUT_GPIO != GPIO_NUM_NC) {
        HoldXShutLow(TOF_FRONT_XSHUT_GPIO);
    }
    ReleaseXShut(TOF_REAR_XSHUT_GPIO);

    auto* rear = reinterpret_cast<vl53l0x_handle_t*>(&rear_sensor_);
    if (vl53l0x_create(rear, bus) != ESP_OK) {
        ESP_LOGE(TAG, "rear vl53l0x_create failed");
        return false;
    }
    if (vl53l0x_set_address(*rear, bus, TOF_REAR_I2C_ADDR) != ESP_OK) {
        ESP_LOGE(TAG, "rear set_address 0x%02X failed", TOF_REAR_I2C_ADDR);
        return false;
    }
    if (vl53l0x_init(*rear) != ESP_OK) {
        ESP_LOGE(TAG, "rear vl53l0x_init failed");
        return false;
    }
    vl53l0x_ref_calibration_t ref = {};
    if (vl53l0x_perform_ref_calibration(*rear, &ref) != ESP_OK) {
        ESP_LOGW(TAG, "rear ref_calibration failed");
    }
    if (vl53l0x_set_profile(*rear, VL53L0X_PROFILE_DEFAULT) != ESP_OK) {
        ESP_LOGW(TAG, "rear set_profile failed");
    }
    has_rear_ = true;
    ESP_LOGI(TAG, "Rear VL53L0X OK @ 0x%02X (floor/cliff)", TOF_REAR_I2C_ADDR);
    return true;
#endif
}

namespace {

std::string JsonCalResult(bool ok, int distance_mm, int32_t offset_um, const char* detail) {
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"ok\":%s,\"distance_mm\":%d,\"offset_um\":%ld,\"detail\":\"%s\"}", ok ? "true" : "false",
             distance_mm, static_cast<long>(offset_um), detail ? detail : "");
    return std::string(buf);
}

}  // namespace

TofController& TofController::Instance() {
    static TofController instance;
    return instance;
}

bool TofController::LoadCalibration() {
    Settings settings(NVS_NS, false);
    calibrated_ = settings.GetBool("calibrated", false);
    if (!calibrated_) {
        ESP_LOGI(TAG, "No saved ToF calibration in NVS");
        return false;
    }
    offset_um_ = settings.GetInt("offset_um", 0);
    cal_distance_mm_ = settings.GetInt("cal_dist_mm", TOF_CALIBRATION_DISTANCE_MM);
    ref_vhv_ = static_cast<uint8_t>(settings.GetInt("ref_vhv", 0));
    ref_phase_ = static_cast<uint8_t>(settings.GetInt("ref_phase", 0));
    std::string xtalk_str = settings.GetString("xtalk_mcps", "0");
    xtalk_mcps_ = strtof(xtalk_str.c_str(), nullptr);
    ESP_LOGI(TAG, "Loaded NVS cal: dist=%ld mm offset=%ld um ref_vhv=%u ref_phase=%u xtalk=%.3f",
             static_cast<long>(cal_distance_mm_), static_cast<long>(offset_um_), ref_vhv_, ref_phase_,
             xtalk_mcps_);
    return true;
}

bool TofController::SaveCalibration() {
    Settings settings(NVS_NS, true);
    settings.SetBool("calibrated", true);
    settings.SetInt("offset_um", offset_um_);
    settings.SetInt("cal_dist_mm", cal_distance_mm_);
    settings.SetInt("ref_vhv", ref_vhv_);
    settings.SetInt("ref_phase", ref_phase_);
    char xtalk_buf[32];
    snprintf(xtalk_buf, sizeof(xtalk_buf), "%.6f", xtalk_mcps_);
    settings.SetString("xtalk_mcps", xtalk_buf);
    calibrated_ = true;
    ESP_LOGI(TAG, "Saved ToF calibration to NVS");
    return true;
}

bool TofController::EnsureRefCalibration() {
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
    if (!sensor) {
        return false;
    }

    if (calibrated_ && ref_vhv_ != 0) {
        vl53l0x_ref_calibration_t ref = {.vhv_settings = ref_vhv_, .phase_cal = ref_phase_};
        esp_err_t err = vl53l0x_set_ref_calibration(sensor, &ref);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "set_ref_calibration failed: %s — re-running ref cal", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Applied NVS ref cal vhv=%u phase=%u", ref_vhv_, ref_phase_);
            return true;
        }
    }

    vl53l0x_ref_calibration_t ref = {};
    esp_err_t err = vl53l0x_perform_ref_calibration(sensor, &ref);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ref_calibration failed: %s", esp_err_to_name(err));
        return false;
    }
    ref_vhv_ = ref.vhv_settings;
    ref_phase_ = ref.phase_cal;
    ESP_LOGI(TAG, "ref_calibration OK vhv=%u phase=%u", ref_vhv_, ref_phase_);
    return true;
}

bool TofController::ApplyStoredCalibration() {
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
    if (!sensor) {
        return false;
    }
    if (!calibrated_) {
        return true;
    }

    vl53l0x_ref_calibration_t ref = {.vhv_settings = ref_vhv_, .phase_cal = ref_phase_};
    esp_err_t err = vl53l0x_set_ref_calibration(sensor, &ref);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set_ref_calibration failed: %s", esp_err_to_name(err));
    }
    err = vl53l0x_set_offset_calibration(sensor, offset_um_);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set_offset_calibration failed: %s", esp_err_to_name(err));
        return false;
    }
    if (xtalk_mcps_ > 0.0f) {
        err = vl53l0x_set_xtalk_calibration(sensor, xtalk_mcps_);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "set_xtalk_calibration failed: %s", esp_err_to_name(err));
        }
        vl53l0x_set_xtalk_compensation_enable(sensor, true);
    }
    ESP_LOGI(TAG, "Applied stored calibration (offset=%ld um @ %ld mm)",
             static_cast<long>(offset_um_), static_cast<long>(cal_distance_mm_));
    return true;
}

bool TofController::InitHardware() {
    if (I2C_SENSOR_SDA_PIN == GPIO_NUM_NC || I2C_SENSOR_SCL_PIN == GPIO_NUM_NC) {
        ESP_LOGI(TAG, "ToF I2C not configured (SDA=%d SCL=%d)", I2C_SENSOR_SDA_PIN,
                 I2C_SENSOR_SCL_PIN);
        return false;
    }

    ESP_LOGI(TAG, "Init I2C SDA=GPIO%d SCL=GPIO%d front_XSHUT=GPIO%d rear=%d", I2C_SENSOR_SDA_PIN,
             I2C_SENSOR_SCL_PIN, TOF_FRONT_XSHUT_GPIO, TOF_REAR_SENSOR_ENABLE);

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = I2C_SENSOR_SDA_PIN;
    bus_cfg.scl_io_num = I2C_SENSOR_SCL_PIN;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.flags.enable_internal_pullup = true;

    auto* bus = reinterpret_cast<i2c_master_bus_handle_t*>(&i2c_bus_);
    if (i2c_new_master_bus(&bus_cfg, bus) != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed");
        return false;
    }

    if (TOF_FRONT_XSHUT_GPIO != GPIO_NUM_NC || TOF_REAR_SENSOR_ENABLE) {
        InitDualXShutHoldAllLow();
        if (TOF_FRONT_XSHUT_GPIO != GPIO_NUM_NC) {
            InitFrontXShut();
        }
    }

    auto* sensor = reinterpret_cast<vl53l0x_handle_t*>(&sensor_);
    if (vl53l0x_create(sensor, *bus) != ESP_OK) {
        ESP_LOGE(TAG, "vl53l0x_create failed");
        return false;
    }
    if (vl53l0x_init(*sensor) != ESP_OK) {
        ESP_LOGE(TAG, "vl53l0x_init failed");
        return false;
    }
    if (vl53l0x_set_profile(*sensor, VL53L0X_PROFILE_DEFAULT) != ESP_OK) {
        ESP_LOGW(TAG, "set_profile failed — using defaults");
    }

    InitRearSensor(*bus);

    vl53l0x_data_t probe = {};
    if (vl53l0x_single_measure(*sensor, &probe) == ESP_OK) {
        ESP_LOGI(TAG, "Probe (pre-ref-cal): dist=%u mm valid=%d status=%u (%s)", probe.distance_mm,
                 probe.valid, probe.range_status, vl53l0x_range_status_str(probe.range_status));
    }
    return true;
}

void TofController::RegisterMcpTools() {
    auto& mcp = McpServer::GetInstance();

    mcp.AddTool(
        "self.tof.calibrate",
        "Calibrate VL53L0X on open floor at normal travel distance (distance_mm, default 400). "
        "Hold robot still facing the floor ahead, then call. Saved distance becomes the safe "
        "reference: stop if closer (obstacle) or farther (cliff) than calibrated range.",
        PropertyList({Property("distance_mm", kPropertyTypeInteger, TOF_CALIBRATION_DISTANCE_MM, 50, 800)}),
        [this](const PropertyList& properties) -> ReturnValue {
            const int distance_mm = properties["distance_mm"].value<int>();
            return Calibrate(distance_mm);
        });

    mcp.AddTool(
        "self.tof.get_distance",
        "Read front VL53L0X distance in millimeters (debug).",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue {
            vl53l0x_data_t sample = {};
            if (!Measure(&sample)) {
                return std::string("{\"ok\":false,\"error\":\"measure_failed\"}");
            }
            char buf[192];
            snprintf(buf, sizeof(buf),
                     "{\"ok\":true,\"distance_mm\":%u,\"valid\":%s,\"range_status\":%u,"
                     "\"calibrated\":%s,\"cal_distance_mm\":%ld}",
                     sample.distance_mm, sample.valid ? "true" : "false", sample.range_status,
                     calibrated_ ? "true" : "false", static_cast<long>(cal_distance_mm_));
            return std::string(buf);
        });
}

bool TofController::Init() {
    if (ready_) {
        return true;
    }
    if (!InitHardware()) {
        return false;
    }
    LoadCalibration();
    if (!EnsureRefCalibration()) {
        ESP_LOGW(TAG, "Ref calibration failed — measurements may be invalid");
    }
    ApplyStoredCalibration();

    RegisterMcpTools();
    ready_ = true;

    vl53l0x_data_t probe = {};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
        if (sensor && vl53l0x_single_measure(sensor, &probe) == ESP_OK) {
            ESP_LOGI(TAG, "Probe: dist=%u mm valid=%d status=%u (%s) signal=%.2f mcps",
                     probe.distance_mm, probe.valid, probe.range_status,
                     vl53l0x_range_status_str(probe.range_status), probe.signal_rate_mcps);
            if (!probe.valid || probe.range_status == 2) {
                ESP_LOGW(TAG, "Signal Fail — check: (1) remove sticker on lens (2) white target "
                              "10–30 cm in front (3) VCC 3.3V SDA/SCL 41/42 (4) then calibrate");
            }
        }
    }

    ESP_LOGI(TAG, "ToF ready (calibrated=%d)", calibrated_);
    return true;
}

bool TofController::Measure(vl53l0x_data_t* out) {
    if (!out || !ready_) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
    return vl53l0x_single_measure(sensor, out) == ESP_OK;
}

bool TofController::MeasureRear(vl53l0x_data_t* out) {
    if (!out || !ready_ || !has_rear_) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(rear_sensor_);
    return vl53l0x_single_measure(sensor, out) == ESP_OK;
}

std::string TofController::Calibrate(int distance_mm) {
    if (!ready_) {
        return JsonCalResult(false, distance_mm, 0, "sensor_not_ready");
    }
    if (distance_mm < 50 || distance_mm > 800) {
        return JsonCalResult(false, distance_mm, 0, "distance_out_of_range");
    }

    std::lock_guard<std::mutex> lock(mutex_);
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);

    ESP_LOGI(TAG, "=== CALIBRATE: place flat target at %d mm, hold still ===", distance_mm);

    vl53l0x_ref_calibration_t ref = {};
    esp_err_t err = vl53l0x_perform_ref_calibration(sensor, &ref);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ref_calibration failed: %s", esp_err_to_name(err));
        return JsonCalResult(false, distance_mm, 0, "ref_calibration_failed");
    }
    ref_vhv_ = ref.vhv_settings;
    ref_phase_ = ref.phase_cal;
    ESP_LOGI(TAG, "ref_calibration OK vhv=%u phase=%u", ref_vhv_, ref_phase_);

    int32_t offset_um = 0;
    err = vl53l0x_perform_offset_calibration(sensor, static_cast<float>(distance_mm), &offset_um);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "offset_calibration failed: %s", esp_err_to_name(err));
        return JsonCalResult(false, distance_mm, 0, "offset_calibration_failed");
    }
    offset_um_ = offset_um;
    cal_distance_mm_ = distance_mm;
    ESP_LOGI(TAG, "offset_calibration OK offset=%ld um @ %d mm", static_cast<long>(offset_um_), distance_mm);

    float xtalk = 0.0f;
    err = vl53l0x_perform_xtalk_calibration(sensor, static_cast<float>(distance_mm), &xtalk);
    if (err == ESP_OK) {
        xtalk_mcps_ = xtalk;
        vl53l0x_set_xtalk_compensation_enable(sensor, true);
        ESP_LOGI(TAG, "xtalk_calibration OK xtalk=%.3f mcps", xtalk_mcps_);
    } else {
        ESP_LOGW(TAG, "xtalk_calibration skipped: %s", esp_err_to_name(err));
        xtalk_mcps_ = 0.0f;
    }

    SaveCalibration();

    vl53l0x_data_t verify = {};
    if (vl53l0x_single_measure(sensor, &verify) == ESP_OK) {
        ESP_LOGI(TAG, "Verify after cal: dist=%u mm valid=%d status=%u (%s)", verify.distance_mm,
                 verify.valid, verify.range_status, vl53l0x_range_status_str(verify.range_status));
    }

    ESP_LOGI(TAG, "=== CALIBRATE DONE saved to NVS ===");
    return JsonCalResult(true, distance_mm, offset_um_, "saved");
}
