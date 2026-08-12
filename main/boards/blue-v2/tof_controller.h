#ifndef BLUE_V2_TOF_CONTROLLER_H_
#define BLUE_V2_TOF_CONTROLLER_H_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

extern "C" {
#include <vl53l0x.h>
}

/** Latest front/rear samples from the dedicated ToF sampler task (mailbox, not FIFO). */
struct TofSnapshot {
    vl53l0x_data_t front{};
    vl53l0x_data_t rear{};
    bool front_ok = false;
    bool rear_ok = false;
    bool has_rear = false;
    int64_t timestamp_ms = 0;
    uint32_t sequence = 0;
};

class TofController {
public:
    static TofController& Instance();

    // Init I2C + VL53L0X, load saved calibration, register MCP tools, start sampler.
    bool Init();

    bool IsReady() const {
        return ready_;
    }

    bool IsCalibrated() const {
        return calibrated_;
    }

    // Safe travel distance saved at last self.tof.calibrate (mm).
    int CalibratedDistanceMm() const {
        return cal_distance_mm_;
    }

    bool HasRearSensor() const {
        return has_rear_;
    }

    // Direct I2C measure — calibration/recovery only; prefer GetLatestSnapshot().
    bool Measure(vl53l0x_data_t* out);

    bool IsIoBusy() const {
        return io_busy_;
    }

    bool MeasureFront(vl53l0x_data_t* out) {
        return Measure(out);
    }

    bool MeasureRear(vl53l0x_data_t* out);

    /** Thread-safe copy of the newest sampler reading. Returns false if never sampled. */
    bool GetLatestSnapshot(TofSnapshot* out) const;

    /** Ask sampler to measure again soon (motor start). */
    void RequestFastSample();

    /** One-shot I2C measure (MCP debug while idle). Updates latest snapshot. */
    bool SampleOnDemand(TofSnapshot* out = nullptr);

    // Run offset calibration at known target distance (mm). Saves result to NVS.
    std::string Calibrate(int distance_mm);

    // Clear saved NVS calibration (guard uses fallback thresholds until recal).
    bool ClearCalibration();

private:
    TofController() = default;

    bool InitHardware();
    bool InitRearSensor(i2c_master_bus_handle_t bus);
    bool LoadCalibration();
    bool SaveCalibration();
    bool ApplyStoredCalibration();
    bool EnsureRefCalibration(bool force_fresh = false);
    void RegisterMcpTools();
    bool ReinitSensor(bool force_ref_cal);
    bool RecreateFrontSensor();
    bool HardRecoverSensor();
    bool RecoverBus(const char* reason);
    bool StartSampler();
    static void SamplerTaskEntry(void* arg);
    void SamplerLoop();
    void PublishSnapshot(bool front_ok, const vl53l0x_data_t& front, bool rear_ok,
                         const vl53l0x_data_t& rear);

    mutable std::mutex snapshot_mutex_;
    TofSnapshot latest_snapshot_;
    bool snapshot_valid_ = false;
    std::atomic<uint32_t> snapshot_sequence_{0};
    TaskHandle_t sampler_task_ = nullptr;
    std::atomic<bool> sampler_running_{false};
    std::atomic<bool> fast_sample_requested_{false};

    std::mutex mutex_;
    void* i2c_bus_ = nullptr;
    void* sensor_ = nullptr;
    void* rear_sensor_ = nullptr;
    bool has_rear_ = false;
    bool ready_ = false;
    bool calibrated_ = false;
    bool io_busy_ = false;
    int consecutive_measure_failures_ = 0;
    int64_t last_bus_recover_ms_ = 0;
    int32_t offset_um_ = 0;
    int32_t cal_distance_mm_ = 0;
    uint8_t ref_vhv_ = 0;
    uint8_t ref_phase_ = 0;
    float xtalk_mcps_ = 0.0f;
};

#endif  // BLUE_V2_TOF_CONTROLLER_H_
