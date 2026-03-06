#ifndef VL53L1X_DS28E18_H
#define VL53L1X_DS28E18_H

#include <Arduino.h>
#include "DS28E18.h"
#include "DS28E18_Sequencer.h"
#include "OneWireBus.h"

// VL53L1X default I2C address (7-bit)
#define VL53L1X_ADDR_DEFAULT 0x29

// Key register addresses (16-bit) from the VL53L1X register map
// -- Identification --
#define VL53L1X_IDENTIFICATION_MODEL_ID             0x010F

// -- Configuration --
#define VL53L1X_SOFT_RESET                          0x0000
#define VL53L1X_I2C_SLAVE_DEVICE_ADDRESS            0x0001
#define VL53L1X_PAD_I2C_HV_EXTSUP_CONFIG            0x002E
#define VL53L1X_GPIO_HV_MUX_CTRL                    0x0030
#define VL53L1X_GPIO_TIO_HV_STATUS                  0x0031
#define VL53L1X_OSC_MEASURED_FAST_OSC_FREQUENCY      0x0006
#define VL53L1X_RESULT_OSC_CALIBRATE_VAL             0x00DE
#define VL53L1X_FIRMWARE_SYSTEM_STATUS               0x00E5
#define VL53L1X_SYSTEM_INTERRUPT_CLEAR               0x0086
#define VL53L1X_SYSTEM_MODE_START                    0x0087
#define VL53L1X_RESULT_RANGE_STATUS                  0x0089
#define VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD       0x006C
#define VL53L1X_SYSTEM_SEQUENCE_CONFIG               0x0081

// -- Static Config --
#define VL53L1X_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS        0x0024
#define VL53L1X_SIGMA_ESTIMATOR_EFFECTIVE_PULSE_WIDTH    0x0036
#define VL53L1X_SIGMA_ESTIMATOR_EFFECTIVE_AMBIENT_WIDTH  0x0037
#define VL53L1X_ALGO_CROSSTALK_COMPENSATION_VALID_HEIGHT 0x0039
#define VL53L1X_ALGO_RANGE_IGNORE_VALID_HEIGHT_MM        0x003E
#define VL53L1X_ALGO_RANGE_MIN_CLIP                      0x003F
#define VL53L1X_ALGO_CONSISTENCY_CHECK_TOLERANCE          0x0040
#define VL53L1X_SYSTEM_THRESH_RATE_HIGH                  0x0050
#define VL53L1X_SYSTEM_THRESH_RATE_LOW                   0x0052
#define VL53L1X_DSS_CONFIG_APERTURE_ATTENUATION          0x0057
#define VL53L1X_RANGE_CONFIG_SIGMA_THRESH                0x0064
#define VL53L1X_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT    0x0066

// -- Timing Config --
#define VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_A              0x0060
#define VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_B              0x0063
#define VL53L1X_RANGE_CONFIG_VALID_PHASE_HIGH            0x0069
#define VL53L1X_PHASECAL_CONFIG_TIMEOUT_MACROP           0x004B
#define VL53L1X_MM_CONFIG_TIMEOUT_MACROP_A               0x005A
#define VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_A            0x005E
#define VL53L1X_MM_CONFIG_TIMEOUT_MACROP_B               0x005C
#define VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_B            0x0061

// -- Dynamic Config --
#define VL53L1X_SYSTEM_GROUPED_PARAMETER_HOLD_0          0x0071
#define VL53L1X_SYSTEM_GROUPED_PARAMETER_HOLD_1          0x007C
#define VL53L1X_SD_CONFIG_QUANTIFIER                     0x007E
#define VL53L1X_SYSTEM_GROUPED_PARAMETER_HOLD            0x0082
#define VL53L1X_SYSTEM_SEED_CONFIG                       0x0077
#define VL53L1X_SD_CONFIG_WOI_SD0                        0x0078
#define VL53L1X_SD_CONFIG_WOI_SD1                        0x0079
#define VL53L1X_SD_CONFIG_INITIAL_PHASE_SD0              0x007A
#define VL53L1X_SD_CONFIG_INITIAL_PHASE_SD1              0x007B
#define VL53L1X_DSS_CONFIG_MANUAL_EFF_SPADS_SELECT       0x0054
#define VL53L1X_DSS_CONFIG_ROI_MODE_CONTROL              0x004F

// -- Calibration --
#define VL53L1X_VHV_CONFIG_INIT                          0x000B
#define VL53L1X_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND     0x0008
#define VL53L1X_PHASECAL_CONFIG_OVERRIDE                 0x004D
#define VL53L1X_CAL_CONFIG_VCSEL_START                   0x0047
#define VL53L1X_PHASECAL_RESULT_VCSEL_START              0x00D8

// -- Offset --
#define VL53L1X_ALGO_PART_TO_PART_RANGE_OFFSET_MM        0x001E
#define VL53L1X_MM_CONFIG_INNER_OFFSET_MM                0x0020
#define VL53L1X_MM_CONFIG_OUTER_OFFSET_MM                0x0022

// -- ROI --
#define VL53L1X_ROI_CONFIG_USER_ROI_CENTRE_SPAD          0x007F
#define VL53L1X_ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY  0x0080


class VL53L1X_DS28E18 {
public:
    enum DistanceMode { Short, Medium, Long, Unknown };

    enum RangeStatus : uint8_t {
        RangeValid                =  0,
        SigmaFail                 =  1,
        SignalFail                =  2,
        RangeValidMinRangeClipped =  3,
        OutOfBoundsFail           =  4,
        HardwareFail              =  5,
        RangeValidNoWrapCheckFail =  6,
        WrapTargetFail            =  7,
        XtalkSignalFail           =  9,
        SynchronizationInt        = 10,
        MinRangeFail              = 13,
        None                      = 255,
    };

    struct RangingData {
        uint16_t range_mm;
        RangeStatus range_status;
    };

    RangingData ranging_data;

    // Constructor for single DS28E18
    VL53L1X_DS28E18(DS28E18 &ds28e18_instance, uint8_t i2cAddr = VL53L1X_ADDR_DEFAULT);

    // Constructor for OneWireBus with multiple devices
    VL53L1X_DS28E18(OneWireBus &busInstance, uint8_t deviceIndex = 0, uint8_t i2cAddr = VL53L1X_ADDR_DEFAULT);

    // Initialize: soft reset, verify model ID (0xEACC), configure defaults
    bool begin(bool io_2v8 = true);

    // Check if sensor responds
    bool isConnected();

    // Get model ID (should return 0xEACC)
    uint16_t getModelID();

    // Distance mode
    bool setDistanceMode(DistanceMode mode);
    DistanceMode getDistanceMode() { return distance_mode; }

    // Timing budget in microseconds
    bool setMeasurementTimingBudget(uint32_t budget_us);
    uint32_t getMeasurementTimingBudget();

    // Continuous ranging
    void startContinuous(uint32_t period_ms);
    void stopContinuous();

    // Read distance (blocking waits for data ready)
    uint16_t read(bool blocking = true);

    // Single-shot measurement
    uint16_t readSingle(bool blocking = true);

    // Check if new data is available
    bool dataReady();

    // Timeout
    void setTimeout(uint16_t timeout) { io_timeout = timeout; }
    uint16_t getTimeout() { return io_timeout; }
    bool timeoutOccurred();

    // Device index (for bus mode)
    void setDeviceIndex(uint8_t index);
    
    // Diagnostics
    uint8_t getRangeStatus() { return ranging_data.range_status; }
    uint16_t getSignalRate() { return results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0; }
    uint16_t getAmbientRate() { return results.ambient_count_rate_mcps_sd0; }
    uint16_t getSpadCount() { return results.dss_actual_effective_spads_sd0; }

    uint8_t getDeviceIndex() const { return deviceIndex; }

    // ---- Debug helpers (raw register/result dumps) ----
    // These bypass range-status interpretation and are meant for comparing
    // DS28E18 reads against direct-I2C (e.g. Pololu) reads.
    uint8_t debugReadReg8(uint16_t reg) { return readReg(reg); }
    uint16_t debugReadReg16(uint16_t reg) { return readReg16Bit(reg); }
    bool debugReadResultsRaw(uint8_t out17[17]);

private:
    DS28E18 *dsPtr = nullptr;
    OneWireBus *busPtr = nullptr;
    uint8_t deviceIndex = 0;
    uint8_t address;

    DS28E18_Sequencer seq;

    uint16_t io_timeout;
    bool did_timeout;
    uint16_t timeout_start_ms;

    uint16_t fast_osc_frequency;
    uint16_t osc_calibrate_val;

    bool calibrated;
    uint8_t saved_vhv_init;
    uint8_t saved_vhv_timeout;
    DistanceMode distance_mode;

    static const uint32_t TimingGuard = 4528;
    static const uint16_t TargetRate = 0x0A00;

    struct ResultBuffer {
        uint8_t range_status;
        uint8_t stream_count;
        uint16_t dss_actual_effective_spads_sd0;
        uint16_t ambient_count_rate_mcps_sd0;
        uint16_t final_crosstalk_corrected_range_mm_sd0;
        uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
    };
    ResultBuffer results;

    DS28E18 &activeDS();

    // Low-level I2C via DS28E18 sequencer (16-bit register addressing)
    void writeReg(uint16_t reg, uint8_t value);
    void writeReg16Bit(uint16_t reg, uint16_t value);
    void writeReg32Bit(uint16_t reg, uint32_t value);
    uint8_t readReg(uint16_t reg);
    uint16_t readReg16Bit(uint16_t reg);
    uint32_t readReg32Bit(uint16_t reg);

    // Bulk read for results (17 bytes from RESULT__RANGE_STATUS)
    bool readResults();

    void setupManualCalibration();
    void updateDSS();
    void getRangingData();

    void startTimeout() { timeout_start_ms = millis(); }
    bool checkTimeoutExpired() { return (io_timeout > 0) && ((uint16_t)(millis() - timeout_start_ms) > io_timeout); }

    static uint32_t decodeTimeout(uint16_t reg_val);
    static uint16_t encodeTimeout(uint32_t timeout_mclks);
    static uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us);
    static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us);
    uint32_t calcMacroPeriod(uint8_t vcsel_period);
};

#endif
