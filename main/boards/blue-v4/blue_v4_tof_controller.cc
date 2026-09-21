#include "blue_v4_tof_controller.h"

#include "config.h"
#include "mcp_server.h"
#include "settings.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

extern "C" {
#include <vl53l0x.h>
}

#define TAG "BlueV4Tof"
#define NVS_NS "blue_v4_tof"

namespace {
constexpr int kIdleSamplePeriodMs = 200;  // background refresh while idle
constexpr int kCalibrationSamples = 9;    // median of up to 9 readings
constexpr int kCalibrationSettleMs = 40;
constexpr int64_t kBusRecoverCooldownMs = 2000;

bool SampleUsable(const vl53l0x_data_t& sample) {
    return sample.valid && sample.distance_mm >= 50 && sample.distance_mm <= TOF_MAX_VALID_MM;
}

int MedianReading(vl53l0x_handle_t sensor) {
    if (sensor == nullptr) {
        return 0;
    }
    std::vector<int> readings;
    readings.reserve(kCalibrationSamples);
    for (int i = 0; i < kCalibrationSamples; i++) {
        vl53l0x_data_t sample = {};
        if (vl53l0x_single_measure(sensor, &sample) == ESP_OK && SampleUsable(sample)) {
            readings.push_back(static_cast<int>(sample.distance_mm));
        }
        vTaskDelay(pdMS_TO_TICKS(kCalibrationSettleMs));
    }
    if (readings.size() < 3) {
        return 0;
    }
    std::sort(readings.begin(), readings.end());
    return readings[readings.size() / 2];
}

std::string JsonCalResult(bool ok, int distance_mm, int32_t offset_um, const char* detail) {
    char buf[256];
    snprintf(buf, sizeof(buf), "{\"ok\":%s,\"distance_mm\":%d,\"offset_um\":%ld,\"detail\":\"%s\"}",
             ok ? "true" : "false", distance_mm, static_cast<long>(offset_um),
             detail != nullptr ? detail : "");
    return std::string(buf);
}
}  // namespace

BlueV4TofController& BlueV4TofController::Instance() {
    static BlueV4TofController instance;
    return instance;
}

bool BlueV4TofController::Init(i2c_master_bus_handle_t bus) {
    if (ready_) {
        return true;
    }
    if (bus == nullptr) {
        ESP_LOGE(TAG, "Init without an I2C bus handle (board must create the bus)");
        return false;
    }
    bus_ = bus;

    if (!InitSensor()) {
        return false;
    }
    LoadCalibration();
    EnsureRefCalibration();
    ApplyStoredCalibration();

    RegisterMcpTools();
    ready_ = true;

    vl53l0x_data_t probe = {};
    if (MeasureSummary(&probe)) {
        ESP_LOGI(TAG, "Probe: dist=%u mm valid=%d status=%u (%s) signal=%.2f mcps",
                 probe.distance_mm, probe.valid, probe.range_status,
                 vl53l0x_range_status_str(probe.range_status), probe.signal_rate_mcps);
        if (!probe.valid || probe.range_status == 2) {
            ESP_LOGW(TAG,
                     "Signal Fail — check: (1) remove sticker on lens (2) white target 10-30 cm "
                     "(3) VCC 3.3 V / SDA 41 / SCL 42 (4) then run self.tof.calibrate");
        }
    } else {
        ESP_LOGW(TAG, "Probe measurement failed");
    }

    sampler_running_.store(true);
    if (xTaskCreate(SamplerTaskEntry, "blue_v4_tof", 4096, this, 3, &sampler_task_) != pdPASS) {
        sampler_running_.store(false);
        ESP_LOGE(TAG, "sampler task creation failed");
        return false;
    }

    ESP_LOGI(TAG, "ToF ready (calibrated=%d, cal_ref=%ld mm)", calibrated_ ? 1 : 0,
             static_cast<long>(cal_distance_mm_));
    return true;
}

bool BlueV4TofController::InitSensor() {
    auto* sensor = reinterpret_cast<vl53l0x_handle_t*>(&sensor_);
    if (vl53l0x_create(sensor, bus_) != ESP_OK) {
        ESP_LOGE(TAG, "vl53l0x_create failed — sensor missing on the shared I2C bus?");
        sensor_ = nullptr;
        return false;
    }
    if (vl53l0x_init(*sensor) != ESP_OK) {
        ESP_LOGE(TAG, "vl53l0x_init failed");
        return false;
    }
    if (vl53l0x_set_profile(*sensor, VL53L0X_PROFILE_DEFAULT) != ESP_OK) {
        ESP_LOGW(TAG, "set_profile failed — using defaults");
    }
    ESP_LOGI(TAG, "VL53L0X ready @ 0x%02X", TOF_FRONT_I2C_ADDR);
    return true;
}

bool BlueV4TofController::LoadCalibration() {
    Settings settings(NVS_NS, false);
    calibrated_ = settings.GetBool("calibrated", false);
    if (!calibrated_) {
        ESP_LOGI(TAG, "No saved ToF calibration in NVS (%s)", NVS_NS);
        return false;
    }
    offset_um_ = settings.GetInt("offset_um", 0);
    cal_distance_mm_ = settings.GetInt("cal_dist_mm", TOF_CALIBRATION_DISTANCE_MM);
    ref_vhv_ = static_cast<uint8_t>(settings.GetInt("ref_vhv", 0));
    ref_phase_ = static_cast<uint8_t>(settings.GetInt("ref_phase", 0));
    const std::string xtalk = settings.GetString("xtalk_mcps", "0");
    xtalk_mcps_ = strtof(xtalk.c_str(), nullptr);
    ESP_LOGI(TAG, "Loaded NVS cal: dist=%ld mm offset=%ld um ref_vhv=%u ref_phase=%u xtalk=%.3f",
             static_cast<long>(cal_distance_mm_), static_cast<long>(offset_um_), ref_vhv_,
             ref_phase_, static_cast<double>(xtalk_mcps_));
    return true;
}

bool BlueV4TofController::SaveCalibration() {
    Settings settings(NVS_NS, true);
    settings.SetBool("calibrated", true);
    settings.SetInt("offset_um", offset_um_);
    settings.SetInt("cal_dist_mm", cal_distance_mm_);
    settings.SetInt("ref_vhv", ref_vhv_);
    settings.SetInt("ref_phase", ref_phase_);
    char xtalk_buf[32];
    snprintf(xtalk_buf, sizeof(xtalk_buf), "%.6f", static_cast<double>(xtalk_mcps_));
    settings.SetString("xtalk_mcps", xtalk_buf);
    calibrated_ = true;
    ESP_LOGI(TAG, "Saved ToF calibration to NVS (%s)", NVS_NS);
    return true;
}

bool BlueV4TofController::EnsureRefCalibration() {
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
    if (sensor == nullptr) {
        return false;
    }
    if (calibrated_ && ref_vhv_ != 0) {
        vl53l0x_ref_calibration_t ref = {.vhv_settings = ref_vhv_, .phase_cal = ref_phase_};
        if (vl53l0x_set_ref_calibration(sensor, &ref) == ESP_OK) {
            ESP_LOGI(TAG, "Applied NVS ref cal vhv=%u phase=%u", ref_vhv_, ref_phase_);
            return true;
        }
        ESP_LOGW(TAG, "set_ref_calibration failed — re-running ref calibration");
    }

    vl53l0x_ref_calibration_t ref = {};
    if (vl53l0x_perform_ref_calibration(sensor, &ref) != ESP_OK) {
        ESP_LOGW(TAG, "ref calibration failed — measurements may be invalid");
        return false;
    }
    ref_vhv_ = ref.vhv_settings;
    ref_phase_ = ref.phase_cal;
    ESP_LOGI(TAG, "ref calibration OK vhv=%u phase=%u", ref_vhv_, ref_phase_);
    return true;
}

bool BlueV4TofController::ApplyStoredCalibration() {
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
    if (sensor == nullptr || !calibrated_) {
        return calibrated_;
    }
    vl53l0x_ref_calibration_t ref = {.vhv_settings = ref_vhv_, .phase_cal = ref_phase_};
    (void)vl53l0x_set_ref_calibration(sensor, &ref);
    if (vl53l0x_set_offset_calibration(sensor, offset_um_) != ESP_OK) {
        ESP_LOGW(TAG, "set_offset_calibration failed");
        return false;
    }
    if (xtalk_mcps_ > 0.0f) {
        (void)vl53l0x_set_xtalk_calibration(sensor, xtalk_mcps_);
        vl53l0x_set_xtalk_compensation_enable(sensor, true);
    }
    ESP_LOGI(TAG, "Applied stored calibration (offset=%ld um @ %ld mm)",
             static_cast<long>(offset_um_), static_cast<long>(cal_distance_mm_));
    return true;
}

bool BlueV4TofController::RecoverBus(const char* reason) {
    const int64_t now_ms = esp_timer_get_time() / 1000;
    if (now_ms - last_bus_recover_ms_ < kBusRecoverCooldownMs) {
        return false;
    }
    last_bus_recover_ms_ = now_ms;
    ESP_LOGW(TAG, "Recovering I2C bus (%s)", reason != nullptr ? reason : "");
    const esp_err_t err = i2c_master_bus_reset(bus_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_reset failed: %s", esp_err_to_name(err));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    return true;
}

bool BlueV4TofController::MeasureSummary(vl53l0x_data_t* out) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
    if (sensor == nullptr || io_busy_) {
        return false;
    }
    io_busy_ = true;
    vl53l0x_data_t sample = {};
    esp_err_t err = vl53l0x_single_measure(sensor, &sample);
    io_busy_ = false;

    if (err != ESP_OK) {
        RecoverBus("single_measure failed");
        return false;
    }
    Publish(sample, SampleUsable(sample));
    if (out != nullptr) {
        *out = sample;
    }
    return true;
}

void BlueV4TofController::Publish(const vl53l0x_data_t& sample, bool ok) {
    BlueV4TofSnapshot snap{};
    snap.front = sample;
    snap.front_ok = ok;
    snap.timestamp_ms = esp_timer_get_time() / 1000;
    snap.sequence = snapshot_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    latest_ = snap;
    snapshot_valid_ = true;
}

bool BlueV4TofController::GetLatestSnapshot(BlueV4TofSnapshot* out) const {
    if (out == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    if (!snapshot_valid_) {
        return false;
    }
    *out = latest_;
    return true;
}

bool BlueV4TofController::SampleOnDemand(BlueV4TofSnapshot* out) {
    if (!MeasureSummary(nullptr)) {
        return false;
    }
    if (out != nullptr) {
        return GetLatestSnapshot(out);
    }
    return true;
}

void BlueV4TofController::RequestFastSample() { fast_sample_requested_.store(true); }

std::string BlueV4TofController::Calibrate(int distance_mm) {
    if (!ready_) {
        return JsonCalResult(false, 0, offset_um_, "tof_not_ready");
    }
    auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
    if (sensor == nullptr) {
        return JsonCalResult(false, 0, offset_um_, "no_sensor");
    }

    int median = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        median = MedianReading(sensor);
        if (median > 0) {
            const int reference = distance_mm > 0 ? distance_mm : median;
            // Trim the reported distance toward the known reference distance.
            offset_um_ += (reference - median) * 1000;
            cal_distance_mm_ = reference;
            if (vl53l0x_set_offset_calibration(sensor, offset_um_) != ESP_OK) {
                ESP_LOGW(TAG, "set_offset_calibration failed during calibrate");
            }
            SaveCalibration();
            ESP_LOGI(TAG, "Calibrated: median=%d mm reference=%d mm offset=%ld um", median,
                     reference, static_cast<long>(offset_um_));
        }
    }
    if (median <= 0) {
        return JsonCalResult(false, 0, offset_um_, "no_valid_reading");
    }

    MeasureSummary(nullptr);
    return JsonCalResult(true, cal_distance_mm_, offset_um_,
                         distance_mm > 0 ? "offset_from_reference" : "median_auto");
}

bool BlueV4TofController::ClearCalibration() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        offset_um_ = 0;
        cal_distance_mm_ = 0;
        xtalk_mcps_ = 0.0f;
        calibrated_ = false;
        auto* sensor = reinterpret_cast<vl53l0x_handle_t>(sensor_);
        if (sensor != nullptr) {
            (void)vl53l0x_set_offset_calibration(sensor, 0);
            vl53l0x_set_xtalk_compensation_enable(sensor, false);
        }
    }
    Settings settings(NVS_NS, true);
    settings.EraseAll();
    ESP_LOGI(TAG, "ToF calibration cleared — guard uses fallback thresholds");
    return true;
}

void BlueV4TofController::SamplerTaskEntry(void* arg) {
    static_cast<BlueV4TofController*>(arg)->SamplerLoop();
}

void BlueV4TofController::SamplerLoop() {
    ESP_LOGI(TAG, "ToF sampler running (%d ms idle, immediate on request)", kIdleSamplePeriodMs);
    while (sampler_running_.load(std::memory_order_relaxed)) {
        const bool fast = fast_sample_requested_.exchange(false);
        MeasureSummary(nullptr);
        vTaskDelay(pdMS_TO_TICKS(fast ? TOF_GUARD_POLL_MS : kIdleSamplePeriodMs));
    }
    ESP_LOGW(TAG, "ToF sampler stopped");
}

void BlueV4TofController::RegisterMcpTools() {
    auto& mcp = McpServer::GetInstance();

    mcp.AddTool(
        "self.tof.calibrate",
        "Calibrate the front VL53L0X on open floor. distance_mm=0 auto-uses the median reading; "
        "128 mm is a normal low-mount reading to the floor. The saved distance is the safe "
        "reference for the obstacle/cliff guard used while walking.",
        PropertyList({Property("distance_mm", kPropertyTypeInteger, 0, 0, 800)}),
        [this](const PropertyList& properties) -> ReturnValue {
            return Calibrate(properties["distance_mm"].value<int>());
        });

    mcp.AddTool("self.tof.get_distance", "Read the front VL53L0X distance in millimeters (debug).",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    BlueV4TofSnapshot snap{};
                    if (!SampleOnDemand(&snap)) {
                        return std::string("{\"ok\":false,\"error\":\"measure_failed\"}");
                    }
                    char buf[256];
                    snprintf(
                        buf, sizeof(buf),
                        "{\"ok\":true,\"distance_mm\":%u,\"valid\":%s,\"range_status\":%u,"
                        "\"calibrated\":%s,\"cal_distance_mm\":%ld,\"age_ms\":%lld,\"seq\":%lu}",
                        snap.front.distance_mm, snap.front.valid ? "true" : "false",
                        snap.front.range_status, calibrated_ ? "true" : "false",
                        static_cast<long>(cal_distance_mm_),
                        static_cast<long long>(esp_timer_get_time() / 1000 - snap.timestamp_ms),
                        static_cast<unsigned long>(snap.sequence));
                    return std::string(buf);
                });

    mcp.AddTool("self.tof.clear_calibration",
                "Clear the saved ToF calibration from NVS. The obstacle guard falls back to "
                "fixed thresholds until self.tof.calibrate succeeds again.",
                PropertyList(), [this](const PropertyList&) -> ReturnValue {
                    return ClearCalibration() ? std::string("{\"ok\":true}")
                                              : std::string("{\"ok\":false}");
                });
}
