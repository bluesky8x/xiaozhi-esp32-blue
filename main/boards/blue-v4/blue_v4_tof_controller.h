#ifndef _BLUE_V4_TOF_CONTROLLER_H_
#define _BLUE_V4_TOF_CONTROLLER_H_

#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

extern "C" {
#include <vl53l0x.h>
}

// Latest reading from the dedicated ToF sampler task (mailbox, not FIFO).
struct BlueV4TofSnapshot {
    vl53l0x_data_t front{};
    bool front_ok = false;
    int64_t timestamp_ms = 0;
    uint32_t sequence = 0;
};

// Blue V4 front VL53L0X controller.
//
// Unlike blue-v2 this controller does NOT own the I2C bus: the board creates it
// once and injects the handle (the PCA9685 shares the same bus at 0x40).
// Same MCP tool names and calibration semantics as blue-v2, own NVS namespace
// (blue_v4_tof) so a fresh calibration is required.
class BlueV4TofController {
public:
    static BlueV4TofController& Instance();

    bool Init(i2c_master_bus_handle_t bus);
    bool IsReady() const { return ready_; }
    bool IsCalibrated() const { return calibrated_; }
    int CalibratedDistanceMm() const { return cal_distance_mm_; }

    bool GetLatestSnapshot(BlueV4TofSnapshot* out) const;
    bool SampleOnDemand(BlueV4TofSnapshot* out = nullptr);
    void RequestFastSample();

    // Run offset calibration at a known target distance (mm). 0 = auto (median reading).
    std::string Calibrate(int distance_mm);
    bool ClearCalibration();

    void RegisterMcpTools();

private:
    BlueV4TofController() = default;

    bool InitSensor();
    bool LoadCalibration();
    bool SaveCalibration();
    bool ApplyStoredCalibration();
    bool EnsureRefCalibration();
    bool MeasureSummary(vl53l0x_data_t* out);
    void Publish(const vl53l0x_data_t& sample, bool ok);
    bool RecoverBus(const char* reason);

    static void SamplerTaskEntry(void* arg);
    void SamplerLoop();

    i2c_master_bus_handle_t bus_ = nullptr;
    void* sensor_ = nullptr;
    bool ready_ = false;
    bool calibrated_ = false;
    bool io_busy_ = false;
    std::mutex mutex_;
    int32_t offset_um_ = 0;
    int32_t cal_distance_mm_ = 0;
    uint8_t ref_vhv_ = 0;
    uint8_t ref_phase_ = 0;
    float xtalk_mcps_ = 0.0f;
    int64_t last_bus_recover_ms_ = 0;

    mutable std::mutex snapshot_mutex_;
    BlueV4TofSnapshot latest_{};
    bool snapshot_valid_ = false;
    std::atomic<uint32_t> snapshot_sequence_{0};
    std::atomic<bool> fast_sample_requested_{false};
    std::atomic<bool> sampler_running_{false};
    TaskHandle_t sampler_task_ = nullptr;
};

#endif  // _BLUE_V4_TOF_CONTROLLER_H_
