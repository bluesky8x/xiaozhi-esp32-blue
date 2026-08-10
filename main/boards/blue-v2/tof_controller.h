#ifndef BLUE_V2_TOF_CONTROLLER_H_
#define BLUE_V2_TOF_CONTROLLER_H_

#include <mutex>
#include <string>

extern "C" {
#include <vl53l0x.h>
}

class TofController {
public:
    static TofController& Instance();

    // Init I2C + VL53L0X, load saved calibration, register MCP tools.
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

    // Thread-safe measure — front sensor (default).
    bool Measure(vl53l0x_data_t* out);

    bool MeasureFront(vl53l0x_data_t* out) {
        return Measure(out);
    }

    bool MeasureRear(vl53l0x_data_t* out);

    // Run offset calibration at known target distance (mm). Saves result to NVS.
    std::string Calibrate(int distance_mm);

private:
    TofController() = default;

    bool InitHardware();
    bool InitRearSensor(i2c_master_bus_handle_t bus);
    bool LoadCalibration();
    bool SaveCalibration();
    bool ApplyStoredCalibration();
    bool EnsureRefCalibration();
    void RegisterMcpTools();

    std::mutex mutex_;
    void* i2c_bus_ = nullptr;
    void* sensor_ = nullptr;
    void* rear_sensor_ = nullptr;
    bool has_rear_ = false;
    bool ready_ = false;
    bool calibrated_ = false;
    int32_t offset_um_ = 0;
    int32_t cal_distance_mm_ = 0;
    uint8_t ref_vhv_ = 0;
    uint8_t ref_phase_ = 0;
    float xtalk_mcps_ = 0.0f;
};

#endif  // BLUE_V2_TOF_CONTROLLER_H_
