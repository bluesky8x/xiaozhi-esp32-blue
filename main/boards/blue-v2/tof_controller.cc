#include "tof_controller.h"

#include "config.h"
#include "mcp_server.h"
#include "settings.h"

#include <cstdio>
#include <cstdlib>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_log.h>
#include <esp_timer.h>
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

int AbsDeltaInt(int a, int b) {
    return a >= b ? a - b : b - a;
}

void TryAbortMeasurement(vl53l0x_handle_t sensor) {
    if (sensor == nullptr) {
        return;
    }
    (void)vl53l0x_stop_measurement(sensor, 500);
    (void)vl53l0x_clear_interrupt_mask(sensor);
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

bool TofController::EnsureRefCalibration(bool force_fresh) {
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
    if (!sensor) {
        return false;
    }

    if (!force_fresh && calibrated_ && ref_vhv_ != 0) {
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

    mcp.AddTool("self.tof.clear_calibration",
                "Clear saved ToF calibration from NVS. Obstacle guard uses fallback thresholds until "
                "self.tof.calibrate succeeds.",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    return ClearCalibration() ? std::string("{\"ok\":true}") :
                                                std::string("{\"ok\":false}");
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

bool TofController::ReinitSensor(bool force_ref_cal) {
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
    if (sensor == nullptr) {
        return false;
    }
    TryAbortMeasurement(sensor);
    if (vl53l0x_reset(sensor) == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (vl53l0x_init(sensor) != ESP_OK) {
        ESP_LOGE(TAG, "vl53l0x_init failed");
        return false;
    }
    if (vl53l0x_set_profile(sensor, VL53L0X_PROFILE_DEFAULT) != ESP_OK) {
        ESP_LOGW(TAG, "set_profile failed after reinit");
    }
    if (!EnsureRefCalibration(force_ref_cal)) {
        ESP_LOGE(TAG, "ref_calibration failed after reinit");
        return false;
    }
    if (calibrated_ && !ApplyStoredCalibration()) {
        ESP_LOGW(TAG, "stored calibration not applied after reinit");
    }
    return true;
}

bool TofController::RecreateFrontSensor() {
    auto* bus = reinterpret_cast<i2c_master_bus_handle_t>(i2c_bus_);
    if (bus == nullptr) {
        return false;
    }
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
    if (sensor != nullptr) {
        TryAbortMeasurement(sensor);
        if (vl53l0x_destroy(sensor) != ESP_OK) {
            ESP_LOGE(TAG, "vl53l0x_destroy failed — press RESET on ESP");
            return false;
        }
        sensor_ = nullptr;
    }
    if (i2c_master_bus_reset(bus) != ESP_OK) {
        ESP_LOGE(TAG, "bus reset before recreate failed");
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    auto* created = reinterpret_cast<vl53l0x_handle_t*>(&sensor_);
    if (vl53l0x_create(created, bus) != ESP_OK) {
        ESP_LOGE(TAG, "vl53l0x_create failed after recreate");
        return false;
    }
    sensor = *created;
    if (vl53l0x_init(sensor) != ESP_OK) {
        ESP_LOGE(TAG, "vl53l0x_init failed after recreate");
        return false;
    }
    if (vl53l0x_set_profile(sensor, VL53L0X_PROFILE_DEFAULT) != ESP_OK) {
        ESP_LOGW(TAG, "set_profile failed after recreate");
    }
    if (!EnsureRefCalibration(true)) {
        ESP_LOGE(TAG, "ref_calibration failed after recreate");
        return false;
    }
    if (calibrated_ && !ApplyStoredCalibration()) {
        ESP_LOGW(TAG, "stored calibration not applied after recreate");
    }
    ESP_LOGI(TAG, "Front VL53L0X recreated on I2C bus");
    return true;
}

bool TofController::HardRecoverSensor() {
    auto* bus = reinterpret_cast<i2c_master_bus_handle_t>(i2c_bus_);
    if (bus == nullptr || sensor_ == nullptr) {
        return false;
    }
    ESP_LOGW(TAG, "Hard recover VL53L0X...");
    if (i2c_master_bus_reset(bus) != ESP_OK) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (ReinitSensor(true)) {
        return true;
    }
    ESP_LOGW(TAG, "Soft reinit failed — recreating sensor handle");
    return RecreateFrontSensor();
}

bool TofController::RecoverBus(const char* reason) {
    if (i2c_bus_ == nullptr || sensor_ == nullptr) {
        return false;
    }
    const int64_t now_ms = esp_timer_get_time() / 1000;
    if (now_ms - last_bus_recover_ms_ < 3000) {
        return false;
    }
    last_bus_recover_ms_ = now_ms;
    io_busy_ = true;
    ESP_LOGW(TAG, "Recovering I2C bus: %s", reason ? reason : "measure_failed");
    const bool ok = HardRecoverSensor();
    io_busy_ = false;
    if (!ok) {
        ESP_LOGE(TAG, "Hard recover failed");
        return false;
    }
    consecutive_measure_failures_ = 0;
    vl53l0x_data_t probe = {};
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
    if (vl53l0x_single_measure(sensor, &probe) == ESP_OK) {
        ESP_LOGI(TAG, "Recovered: dist=%u mm valid=%d status=%u", probe.distance_mm, probe.valid,
                 probe.range_status);
    } else {
        ESP_LOGW(TAG, "Recovered init OK but probe measure failed");
    }
    return true;
}

bool TofController::Measure(vl53l0x_data_t* out) {
    if (!out || !ready_ || io_busy_) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
    if (vl53l0x_single_measure(sensor, out) == ESP_OK) {
        consecutive_measure_failures_ = 0;
        return true;
    }
    consecutive_measure_failures_++;
    if (consecutive_measure_failures_ >= 3) {
        RecoverBus("consecutive_measure_failures");
    }
    return false;
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
    io_busy_ = true;
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);

    ESP_LOGI(TAG, "=== CALIBRATE: target %d mm — robot still, NO TTS/speaker ===", distance_mm);

    vl53l0x_data_t probe = {};
    if (vl53l0x_single_measure(sensor, &probe) == ESP_OK && probe.valid) {
        const int reading = static_cast<int>(probe.distance_mm);
        const int tol_mm = (distance_mm * 35 / 100) > 50 ? (distance_mm * 35 / 100) : 50;
        if (AbsDeltaInt(reading, distance_mm) > tol_mm) {
            ESP_LOGW(TAG, "Calibrate rejected: reading=%d mm, distance_mm=%d (tol ±%d)", reading,
                     distance_mm, tol_mm);
            io_busy_ = false;
            char detail[96];
            snprintf(detail, sizeof(detail), "distance_mismatch:reading_%d", reading);
            return JsonCalResult(false, distance_mm, 0, detail);
        }
        ESP_LOGI(TAG, "Precheck OK: reading=%d mm matches target %d mm", reading, distance_mm);
    }

    auto fail = [&](const char* detail) -> std::string {
        ESP_LOGE(TAG, "Calibrate failed: %s — hard recover sensor", detail);
        if (!HardRecoverSensor()) {
            ESP_LOGE(TAG, "Hard recover after failed calibrate also failed — reboot ESP");
        }
        io_busy_ = false;
        return JsonCalResult(false, distance_mm, 0, detail);
    };

    TryAbortMeasurement(sensor);
    vTaskDelay(pdMS_TO_TICKS(30));

    vl53l0x_ref_calibration_t ref = {};
    esp_err_t err = vl53l0x_perform_ref_calibration(sensor, &ref);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ref_calibration failed: %s", esp_err_to_name(err));
        return fail("ref_calibration_failed");
    }
    ref_vhv_ = ref.vhv_settings;
    ref_phase_ = ref.phase_cal;
    ESP_LOGI(TAG, "ref_calibration OK vhv=%u phase=%u", ref_vhv_, ref_phase_);

    int32_t offset_um = 0;
    err = vl53l0x_perform_offset_calibration(sensor, static_cast<float>(distance_mm), &offset_um);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "offset_calibration failed: %s", esp_err_to_name(err));
        return fail("offset_calibration_failed");
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

    io_busy_ = false;
    ESP_LOGI(TAG, "=== CALIBRATE DONE saved to NVS ===");
    return JsonCalResult(true, distance_mm, offset_um_, "saved");
}

bool TofController::ClearCalibration() {
    std::lock_guard<std::mutex> lock(mutex_);
    Settings settings(NVS_NS, true);
    settings.SetBool("calibrated", false);
    calibrated_ = false;
    offset_um_ = 0;
    cal_distance_mm_ = 0;
    ref_vhv_ = 0;
    ref_phase_ = 0;
    xtalk_mcps_ = 0.0f;
    ESP_LOGI(TAG, "Cleared ToF calibration from NVS — guard uses fallback thresholds");
    return true;
}
