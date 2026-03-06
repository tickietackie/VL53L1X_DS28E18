#include "VL53L1X_DS28E18.h"

namespace {
// DS28E18 "Read Sequencer" response payload seems to be capped (commonly 18 bytes payload),
// which makes it unreliable to read back the entire sequencer program and parse it.
//
// Instead, we read only the bytes that the DS28E18 overwrote (the dummy 0xFF bytes that
// follow SEQ_CMD_READ/SEQ_CMD_READ_NACK) since those are exactly the I2C read data bytes.
static bool readSequencerBytesChunked(DS28E18 &ds, uint16_t addr, uint8_t *out, uint16_t len)
{
    if (!out || len == 0) return false;

    // Conservative payload chunk size. From real logs, DS28E18 often returns 0x13 (19) total bytes,
    // which is 1 result byte + 18 payload bytes.
    static constexpr uint16_t MaxPayload = 18;

    uint16_t written = 0;
    while (written < len)
    {
        uint16_t chunk = len - written;
        if (chunk > MaxPayload) chunk = MaxPayload;

        uint8_t raw[1 + MaxPayload];
        uint16_t rawLen = 0;
        if (!ds.readSequencer(addr + written, raw, chunk, rawLen)) return false;
        if (rawLen < 1) return false;

        // raw[0] is 0xAA (result). Following bytes are payload (up to chunk bytes).
        uint16_t payload = rawLen - 1;
        if (payload == 0) return false;

        // If device returned fewer bytes than requested, still copy what we got.
        if (payload > chunk) payload = chunk;
        memcpy(out + written, raw + 1, payload);
        written += payload;

        if (payload < chunk) return false; // truncated unexpectedly
    }
    return true;
}

static bool readI2CReadDataFromExecutedSequencer(
    DS28E18 &ds,
    const uint8_t *seqBuf,
    uint16_t seqLen,
    uint8_t *out,
    uint16_t outLen)
{
    if (!seqBuf || seqLen == 0 || !out || outLen == 0) return false;

    uint16_t outIdx = 0;
    for (uint16_t i = 0; i + 1 < seqLen && outIdx < outLen;)
    {
        uint8_t op = seqBuf[i];
        if (op != SEQ_CMD_READ && op != SEQ_CMD_READ_NACK)
        {
            i++;
            continue;
        }

        uint8_t len = seqBuf[i + 1];
        uint16_t dummyStart = i + 2; // DS28E18 overwrites these dummy bytes with read data
        if (dummyStart + len > seqLen) return false;

        // Read back this dummy segment directly from sequencer SRAM.
        uint16_t remaining = outLen - outIdx;
        uint16_t take = len;
        if (take > remaining) take = remaining;

        if (!readSequencerBytesChunked(ds, dummyStart, out + outIdx, take)) return false;
        outIdx += take;

        i = dummyStart + len; // advance past this read block
    }

    return outIdx == outLen;
}
} // namespace

// ---- Constructors ----

VL53L1X_DS28E18::VL53L1X_DS28E18(DS28E18 &ds28e18_instance, uint8_t i2cAddr)
    : dsPtr(&ds28e18_instance), busPtr(nullptr), deviceIndex(0),
      address(i2cAddr), io_timeout(0), did_timeout(false),
      fast_osc_frequency(0), osc_calibrate_val(0),
      calibrated(false), saved_vhv_init(0), saved_vhv_timeout(0), 
      distance_mode(Unknown) {}

VL53L1X_DS28E18::VL53L1X_DS28E18(OneWireBus &busInstance, uint8_t index, uint8_t i2cAddr)
    : dsPtr(nullptr), busPtr(&busInstance), deviceIndex(index),
      address(i2cAddr), io_timeout(0), did_timeout(false),
      fast_osc_frequency(0), osc_calibrate_val(0),
      calibrated(false), saved_vhv_init(0), saved_vhv_timeout(0), 
      distance_mode(Unknown) {}

DS28E18 &VL53L1X_DS28E18::activeDS() {
    if (busPtr) return busPtr->device(deviceIndex);
    return *dsPtr;
}

void VL53L1X_DS28E18::setDeviceIndex(uint8_t index) {
    deviceIndex = index;
}

// ======================================================================
// Low-level I2C via DS28E18 sequencer
// VL53L1X uses 16-bit register addresses: [regHI, regLO]
// Write: START + ADDR(W) + regHI + regLO + data... + STOP
// Read:  START + ADDR(W) + regHI + regLO + STOP + START + ADDR(R) + read N + STOP
// ======================================================================

void VL53L1X_DS28E18::writeReg(uint16_t reg, uint8_t value) {
    DS28E18 &ds = activeDS();
    seq.clear();
    seq.addStart();
    uint8_t data[3] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), value};
    seq.addWrite(address, data, 3);
    seq.addStop();

    if (!ds.writeSequencer(0, seq.getBuffer(), seq.getLength())) return;
    uint8_t result;
    ds.runSequencer(0, seq.getLength(), result);
}

void VL53L1X_DS28E18::writeReg16Bit(uint16_t reg, uint16_t value) {
    DS28E18 &ds = activeDS();
    seq.clear();
    seq.addStart();
    uint8_t data[4] = {
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)
    };
    seq.addWrite(address, data, 4);
    seq.addStop();

    if (!ds.writeSequencer(0, seq.getBuffer(), seq.getLength())) return;
    uint8_t result;
    ds.runSequencer(0, seq.getLength(), result);
}

void VL53L1X_DS28E18::writeReg32Bit(uint16_t reg, uint32_t value) {
    DS28E18 &ds = activeDS();
    seq.clear();
    seq.addStart();
    uint8_t data[6] = {
        (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF),
        (uint8_t)(value >> 24), (uint8_t)(value >> 16),
        (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)
    };
    seq.addWrite(address, data, 6);
    seq.addStop();

    if (!ds.writeSequencer(0, seq.getBuffer(), seq.getLength())) return;
    uint8_t result;
    ds.runSequencer(0, seq.getLength(), result);
}

uint8_t VL53L1X_DS28E18::readReg(uint16_t reg) {
    DS28E18 &ds = activeDS();
    seq.clear();

    // Write register address
    seq.addStart();
    uint8_t regData[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    seq.addWrite(address, regData, 2);
    seq.addStop();

    // Read 1 byte
    seq.addStart();
    seq.addRead(address, 1);
    seq.addStop();

    if (!ds.writeSequencer(0, seq.getBuffer(), seq.getLength())) return 0;
    uint8_t result;
    if (!ds.runSequencer(0, seq.getLength(), result)) return 0;

    uint8_t b = 0;
    if (!readI2CReadDataFromExecutedSequencer(ds, seq.getBuffer(), seq.getLength(), &b, 1)) return 0;
    return b;
}

uint16_t VL53L1X_DS28E18::readReg16Bit(uint16_t reg) {
    DS28E18 &ds = activeDS();
    seq.clear();

    seq.addStart();
    uint8_t regData[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    seq.addWrite(address, regData, 2);
    seq.addStop();

    seq.addStart();
    seq.addRead(address, 2);
    seq.addStop();

    if (!ds.writeSequencer(0, seq.getBuffer(), seq.getLength())) return 0;
    uint8_t result;
    if (!ds.runSequencer(0, seq.getLength(), result)) return 0;

    uint8_t b[2] = {0, 0};
    if (!readI2CReadDataFromExecutedSequencer(ds, seq.getBuffer(), seq.getLength(), b, 2)) return 0;
    return ((uint16_t)b[0] << 8) | b[1];
}

uint32_t VL53L1X_DS28E18::readReg32Bit(uint16_t reg) {
    DS28E18 &ds = activeDS();
    seq.clear();

    seq.addStart();
    uint8_t regData[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    seq.addWrite(address, regData, 2);
    seq.addStop();

    seq.addStart();
    seq.addRead(address, 4);
    seq.addStop();

    if (!ds.writeSequencer(0, seq.getBuffer(), seq.getLength())) return 0;
    uint8_t result;
    if (!ds.runSequencer(0, seq.getLength(), result)) return 0;

    uint8_t b[4] = {0, 0, 0, 0};
    if (!readI2CReadDataFromExecutedSequencer(ds, seq.getBuffer(), seq.getLength(), b, 4)) return 0;
    return ((uint32_t)b[0] << 24) |
           ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8)  |
           b[3];
}

// ======================================================================
// Read 17 bytes from RESULT__RANGE_STATUS for ranging data
// ======================================================================
bool VL53L1X_DS28E18::readResults() {
    DS28E18 &ds = activeDS();
    seq.clear();

    uint16_t reg = VL53L1X_RESULT_RANGE_STATUS;
    seq.addStart();
    uint8_t regData[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    seq.addWrite(address, regData, 2);
    seq.addStop();

    seq.addStart();
    seq.addRead(address, 17);
    seq.addStop();

    if (!ds.writeSequencer(0, seq.getBuffer(), seq.getLength())) return false;
    uint8_t result;
    if (!ds.runSequencer(0, seq.getLength(), result)) return false;
    uint8_t d[17];
    if (!readI2CReadDataFromExecutedSequencer(ds, seq.getBuffer(), seq.getLength(), d, 17)) return false;

    results.range_status = d[0];
    // d[1] = report_status (not used)
    results.stream_count = d[2];
    results.dss_actual_effective_spads_sd0 = ((uint16_t)d[3] << 8) | d[4];
    // d[5], d[6] = peak_signal_count_rate_mcps_sd0 (not used here)
    results.ambient_count_rate_mcps_sd0 = ((uint16_t)d[7] << 8) | d[8];
    // d[9], d[10] = sigma_sd0 (not used)
    // d[11], d[12] = phase_sd0 (not used)
    results.final_crosstalk_corrected_range_mm_sd0 = ((uint16_t)d[13] << 8) | d[14];
    results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 = ((uint16_t)d[15] << 8) | d[16];

    return true;
}

bool VL53L1X_DS28E18::debugReadResultsRaw(uint8_t out17[17])
{
    if (!out17) return false;

    DS28E18 &ds = activeDS();
    seq.clear();

    uint16_t reg = VL53L1X_RESULT_RANGE_STATUS;
    seq.addStart();
    uint8_t regData[2] = {(uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF)};
    seq.addWrite(address, regData, 2);
    seq.addStop();

    seq.addStart();
    seq.addRead(address, 17);
    seq.addStop();

    if (!ds.writeSequencer(0, seq.getBuffer(), seq.getLength())) return false;
    uint8_t result;
    if (!ds.runSequencer(0, seq.getLength(), result)) return false;

    return readI2CReadDataFromExecutedSequencer(ds, seq.getBuffer(), seq.getLength(), out17, 17);
}

// ======================================================================
// Public API
// ======================================================================

bool VL53L1X_DS28E18::isConnected() {
    uint16_t id = readReg16Bit(VL53L1X_IDENTIFICATION_MODEL_ID);
    return (id == 0xEACC);
}

uint16_t VL53L1X_DS28E18::getModelID() {
    return readReg16Bit(VL53L1X_IDENTIFICATION_MODEL_ID);
}

bool VL53L1X_DS28E18::begin(bool io_2v8) {
    // Check model ID
    if (readReg16Bit(VL53L1X_IDENTIFICATION_MODEL_ID) != 0xEACC) {
        return false;
    }

    // Software reset
    writeReg(VL53L1X_SOFT_RESET, 0x00);
    delay(1);
    writeReg(VL53L1X_SOFT_RESET, 0x01);
    delay(1);

    // Wait for boot
    startTimeout();
    while ((readReg(VL53L1X_FIRMWARE_SYSTEM_STATUS) & 0x01) == 0) {
        if (checkTimeoutExpired()) {
            did_timeout = true;
            return false;
        }
        delay(1);
    }

    // 2V8 mode
    if (io_2v8) {
        writeReg(VL53L1X_PAD_I2C_HV_EXTSUP_CONFIG,
                 readReg(VL53L1X_PAD_I2C_HV_EXTSUP_CONFIG) | 0x01);
    }

    // Store oscillator info
    fast_osc_frequency = readReg16Bit(VL53L1X_OSC_MEASURED_FAST_OSC_FREQUENCY);
    osc_calibrate_val = readReg16Bit(VL53L1X_RESULT_OSC_CALIBRATE_VAL);

    // Static config
    writeReg16Bit(VL53L1X_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS, TargetRate);
    writeReg(VL53L1X_GPIO_TIO_HV_STATUS, 0x02);
    writeReg(VL53L1X_SIGMA_ESTIMATOR_EFFECTIVE_PULSE_WIDTH, 8);
    writeReg(VL53L1X_SIGMA_ESTIMATOR_EFFECTIVE_AMBIENT_WIDTH, 16);
    writeReg(VL53L1X_ALGO_CROSSTALK_COMPENSATION_VALID_HEIGHT, 0x01);
    writeReg(VL53L1X_ALGO_RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
    writeReg(VL53L1X_ALGO_RANGE_MIN_CLIP, 0);
    writeReg(VL53L1X_ALGO_CONSISTENCY_CHECK_TOLERANCE, 2);

    // General config
    writeReg16Bit(VL53L1X_SYSTEM_THRESH_RATE_HIGH, 0x0000);
    writeReg16Bit(VL53L1X_SYSTEM_THRESH_RATE_LOW, 0x0000);
    writeReg(VL53L1X_DSS_CONFIG_APERTURE_ATTENUATION, 0x38);

    // Timing config
    writeReg16Bit(VL53L1X_RANGE_CONFIG_SIGMA_THRESH, 360);
    writeReg16Bit(VL53L1X_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 192);

    // Dynamic config
    writeReg(VL53L1X_SYSTEM_GROUPED_PARAMETER_HOLD_0, 0x01);
    writeReg(VL53L1X_SYSTEM_GROUPED_PARAMETER_HOLD_1, 0x01);
    writeReg(VL53L1X_SD_CONFIG_QUANTIFIER, 2);
    writeReg(VL53L1X_SYSTEM_GROUPED_PARAMETER_HOLD, 0x00);
    writeReg(VL53L1X_SYSTEM_SEED_CONFIG, 1);

    // Sequence config (VHV, PHASECAL, DSS1, RANGE)
    writeReg(VL53L1X_SYSTEM_SEQUENCE_CONFIG, 0x8B);

    // DSS config
    writeReg16Bit(VL53L1X_DSS_CONFIG_MANUAL_EFF_SPADS_SELECT, 200 << 8);
    writeReg(VL53L1X_DSS_CONFIG_ROI_MODE_CONTROL, 2);

    // Default to long range, 50ms timing budget
    setDistanceMode(Long);
    setMeasurementTimingBudget(50000);

    // Apply offset
    writeReg16Bit(VL53L1X_ALGO_PART_TO_PART_RANGE_OFFSET_MM,
                  readReg16Bit(VL53L1X_MM_CONFIG_OUTER_OFFSET_MM) * 4);

    return true;
}

// ======================================================================
// Distance Mode
// ======================================================================
bool VL53L1X_DS28E18::setDistanceMode(DistanceMode mode) {
    uint32_t budget_us = getMeasurementTimingBudget();

    switch (mode) {
        case Short:
            writeReg(VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_A, 0x07);
            writeReg(VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_B, 0x05);
            writeReg(VL53L1X_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
            writeReg(VL53L1X_SD_CONFIG_WOI_SD0, 0x07);
            writeReg(VL53L1X_SD_CONFIG_WOI_SD1, 0x05);
            writeReg(VL53L1X_SD_CONFIG_INITIAL_PHASE_SD0, 6);
            writeReg(VL53L1X_SD_CONFIG_INITIAL_PHASE_SD1, 6);
            break;
        case Medium:
            writeReg(VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_A, 0x0B);
            writeReg(VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_B, 0x09);
            writeReg(VL53L1X_RANGE_CONFIG_VALID_PHASE_HIGH, 0x78);
            writeReg(VL53L1X_SD_CONFIG_WOI_SD0, 0x0B);
            writeReg(VL53L1X_SD_CONFIG_WOI_SD1, 0x09);
            writeReg(VL53L1X_SD_CONFIG_INITIAL_PHASE_SD0, 10);
            writeReg(VL53L1X_SD_CONFIG_INITIAL_PHASE_SD1, 10);
            break;
        case Long:
            writeReg(VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_A, 0x0F);
            writeReg(VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_B, 0x0D);
            writeReg(VL53L1X_RANGE_CONFIG_VALID_PHASE_HIGH, 0xB8);
            writeReg(VL53L1X_SD_CONFIG_WOI_SD0, 0x0F);
            writeReg(VL53L1X_SD_CONFIG_WOI_SD1, 0x0D);
            writeReg(VL53L1X_SD_CONFIG_INITIAL_PHASE_SD0, 14);
            writeReg(VL53L1X_SD_CONFIG_INITIAL_PHASE_SD1, 14);
            break;
        default:
            return false;
    }

    setMeasurementTimingBudget(budget_us);
    distance_mode = mode;
    return true;
}

// ======================================================================
// Timing Budget
// ======================================================================
bool VL53L1X_DS28E18::setMeasurementTimingBudget(uint32_t budget_us) {
    if (budget_us <= TimingGuard) return false;

    uint32_t range_config_timeout_us = budget_us - TimingGuard;
    if (range_config_timeout_us > 1100000) return false;
    range_config_timeout_us /= 2;

    uint32_t macro_period_us = calcMacroPeriod(readReg(VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_A));

    uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
    if (phasecal_timeout_mclks > 0xFF) phasecal_timeout_mclks = 0xFF;
    writeReg(VL53L1X_PHASECAL_CONFIG_TIMEOUT_MACROP, phasecal_timeout_mclks);

    writeReg16Bit(VL53L1X_MM_CONFIG_TIMEOUT_MACROP_A,
                  encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)));
    writeReg16Bit(VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_A,
                  encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

    macro_period_us = calcMacroPeriod(readReg(VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_B));

    writeReg16Bit(VL53L1X_MM_CONFIG_TIMEOUT_MACROP_B,
                  encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)));
    writeReg16Bit(VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_B,
                  encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

    return true;
}

uint32_t VL53L1X_DS28E18::getMeasurementTimingBudget() {
    uint32_t macro_period_us = calcMacroPeriod(readReg(VL53L1X_RANGE_CONFIG_VCSEL_PERIOD_A));
    uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(
        decodeTimeout(readReg16Bit(VL53L1X_RANGE_CONFIG_TIMEOUT_MACROP_A)), macro_period_us);
    return 2 * range_config_timeout_us + TimingGuard;
}

// ======================================================================
// Continuous / Single-shot
// ======================================================================
void VL53L1X_DS28E18::startContinuous(uint32_t period_ms) {
    writeReg32Bit(VL53L1X_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val);
    writeReg(VL53L1X_SYSTEM_INTERRUPT_CLEAR, 0x01);
    writeReg(VL53L1X_SYSTEM_MODE_START, 0x40);
}

void VL53L1X_DS28E18::stopContinuous() {
    writeReg(VL53L1X_SYSTEM_MODE_START, 0x80);
    calibrated = false;
    if (saved_vhv_init != 0)
        writeReg(VL53L1X_VHV_CONFIG_INIT, saved_vhv_init);
    if (saved_vhv_timeout != 0)
        writeReg(VL53L1X_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout);
    writeReg(VL53L1X_PHASECAL_CONFIG_OVERRIDE, 0x00);
}

bool VL53L1X_DS28E18::dataReady() {
    return (readReg(VL53L1X_GPIO_TIO_HV_STATUS) & 0x01) == 0;
}

uint16_t VL53L1X_DS28E18::read(bool blocking) {
    if (blocking) {
        startTimeout();
        while (!dataReady()) {
            if (checkTimeoutExpired()) {
                did_timeout = true;
                return 0;
            }
            delay(1);
        }
    }

    readResults();

    if (!calibrated) {
        setupManualCalibration();
        calibrated = true;
    }

    updateDSS();
    getRangingData();

    writeReg(VL53L1X_SYSTEM_INTERRUPT_CLEAR, 0x01);
    return ranging_data.range_mm;
}

uint16_t VL53L1X_DS28E18::readSingle(bool blocking) {
    writeReg(VL53L1X_SYSTEM_INTERRUPT_CLEAR, 0x01);
    writeReg(VL53L1X_SYSTEM_MODE_START, 0x10);

    if (blocking) return read(true);
    return 0;
}

bool VL53L1X_DS28E18::timeoutOccurred() {
    bool tmp = did_timeout;
    did_timeout = false;
    return tmp;
}

// ======================================================================
// Private helpers (ported from Pololu library)
// ======================================================================
void VL53L1X_DS28E18::setupManualCalibration() {
    saved_vhv_init = readReg(VL53L1X_VHV_CONFIG_INIT);
    saved_vhv_timeout = readReg(VL53L1X_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND);
    writeReg(VL53L1X_VHV_CONFIG_INIT, saved_vhv_init & 0x7F);
    writeReg(VL53L1X_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND,
             (saved_vhv_timeout & 0x03) + (3 << 2));
    writeReg(VL53L1X_PHASECAL_CONFIG_OVERRIDE, 0x01);
    writeReg(VL53L1X_CAL_CONFIG_VCSEL_START, readReg(VL53L1X_PHASECAL_RESULT_VCSEL_START));
}

void VL53L1X_DS28E18::updateDSS() {
    uint16_t spadCount = results.dss_actual_effective_spads_sd0;
    if (spadCount != 0) {
        uint32_t totalRatePerSpad =
            (uint32_t)results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
            results.ambient_count_rate_mcps_sd0;
        if (totalRatePerSpad > 0xFFFF) totalRatePerSpad = 0xFFFF;
        totalRatePerSpad <<= 16;
        totalRatePerSpad /= spadCount;
        if (totalRatePerSpad != 0) {
            uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;
            if (requiredSpads > 0xFFFF) requiredSpads = 0xFFFF;
            writeReg16Bit(VL53L1X_DSS_CONFIG_MANUAL_EFF_SPADS_SELECT, requiredSpads);
            return;
        }
    }
    writeReg16Bit(VL53L1X_DSS_CONFIG_MANUAL_EFF_SPADS_SELECT, 0x8000);
}

void VL53L1X_DS28E18::getRangingData() {
    uint16_t range = results.final_crosstalk_corrected_range_mm_sd0;
    ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

    switch (results.range_status) {
        case 17: case 2: case 1: case 3:
            ranging_data.range_status = HardwareFail; break;
        case 13:
            ranging_data.range_status = MinRangeFail; break;
        case 18:
            ranging_data.range_status = SynchronizationInt; break;
        case 5:
            ranging_data.range_status = OutOfBoundsFail; break;
        case 4:
            ranging_data.range_status = SignalFail; break;
        case 6:
            ranging_data.range_status = SigmaFail; break;
        case 7:
            ranging_data.range_status = WrapTargetFail; break;
        case 12:
            ranging_data.range_status = XtalkSignalFail; break;
        case 8:
            ranging_data.range_status = RangeValidMinRangeClipped; break;
        case 9:
            ranging_data.range_status = (results.stream_count == 0)
                ? RangeValidNoWrapCheckFail : RangeValid;
            break;
        default:
            ranging_data.range_status = None;
    }
}

uint32_t VL53L1X_DS28E18::decodeTimeout(uint16_t reg_val) {
    return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

uint16_t VL53L1X_DS28E18::encodeTimeout(uint32_t timeout_mclks) {
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;
    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }
        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    return 0;
}

uint32_t VL53L1X_DS28E18::timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us) {
    return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

uint32_t VL53L1X_DS28E18::timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us) {
    return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

uint32_t VL53L1X_DS28E18::calcMacroPeriod(uint8_t vcsel_period) {
    uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;
    uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;
    uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;
    return macro_period_us;
}
