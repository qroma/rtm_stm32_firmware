#include <unity.h>
#include <cmath>

#include "dsp_pipeline.h"
#include "allan_tracker.h"
#include "telemetry.h"

HardwareSerial Serial;

namespace {
void resetAll(DSPPipeline& dsp, AllanTracker& allan) {
    dsp = DSPPipeline();
    allan = AllanTracker();
}
}

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

static void test_trimmed_mean_rejects_extremes() {
    uint16_t samples[8] = {100, 101, 102, 103, 104, 105, 106, 300};
    float mv = DSPPipeline::computeTrimmedMeanMv(samples, 8);

    TEST_ASSERT_FLOAT_WITHIN(1.0f, 83.0f, mv);
}

static void test_detector_linearization_reference() {
    float value = DSPPipeline::linearizeDetectorMv(DETECTOR_V_REF_MV);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1000.0f, value);
}

static void test_asymmetry_calibration_and_interpolation() {
    DSPPipeline dsp;

    dsp.beginAsymmetryCalibration();
    bool calibration_complete = false;
    for (size_t i = 0; i < CAL_ASYM_SAMPLES; ++i) {
        calibration_complete = dsp.accumulateCalibrationSample(1550.0f, 1500.0f);
    }
    TEST_ASSERT_TRUE(calibration_complete);
    TEST_ASSERT_TRUE(dsp.finalizeAsymmetryCalibration(1000));

    float first = dsp.getKappa();
    TEST_ASSERT_TRUE(std::isfinite(first));
    TEST_ASSERT_TRUE(first > 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, first, dsp.getInterpolatedKappa(1000));

    dsp.beginAsymmetryCalibration();
    for (size_t i = 0; i < CAL_ASYM_SAMPLES; ++i) {
        dsp.accumulateCalibrationSample(1600.0f, 1500.0f);
    }
    TEST_ASSERT_TRUE(dsp.finalizeAsymmetryCalibration(3000));

    TEST_ASSERT_FLOAT_WITHIN(0.0001f, first, dsp.getInterpolatedKappa(1000));
    float second = DSPPipeline::linearizeDetectorMv(1600.0f) /
                   DSPPipeline::linearizeDetectorMv(1500.0f);

    float interpolated = dsp.getInterpolatedKappa(2000);
    TEST_ASSERT_TRUE(std::isfinite(interpolated));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, (first + second) * 0.5f, interpolated);
}

static void test_decimation_and_history_wrap() {
    DSPPipeline dsp;

    for (size_t i = 0; i < INTERP_BUFFER_SIZE + 5; ++i) {
        DecimatedSample sample{};
        bool ready = false;
        for (uint16_t j = 0; j < DECIMATION_FACTOR; ++j) {
            ready = dsp.pushCycleAndDecimate(1500.0f, 1500.0f,
                                             100U * static_cast<uint32_t>(i),
                                             true, sample);
        }
        TEST_ASSERT_TRUE(ready);
    }

    size_t flushed = dsp.flushInterpolatedHistory(
        25.0f,
        [](uint32_t, float, float, float) {});
    TEST_ASSERT_EQUAL(INTERP_BUFFER_SIZE, flushed);
}

static void test_crc16_known_vector() {
    const uint8_t data[] = "123456789";
    TEST_ASSERT_EQUAL_HEX16(0x29B1,
                            TelemetryManager::computeCRC16(data, 9));
}

static void test_serial_commands() {
    DSPPipeline dsp;
    AllanTracker allan;
    SystemMode mode = SystemMode::Idle;

    Serial.setInput("CMD:KCAL=2.5\nCMD:DTDOP=0.25\n");
    TelemetryManager::processIncomingCommands(mode, dsp, allan);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.5f, dsp.getKCal());
    TEST_ASSERT_FALSE(allan.isSystemReady());

    Serial.setInput("CMD:START\n");
    TelemetryManager::processIncomingCommands(mode, dsp, allan);
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(SystemMode::Measuring),
                      static_cast<uint8_t>(mode));

    Serial.setInput("CMD:STOP\n");
    TelemetryManager::processIncomingCommands(mode, dsp, allan);
    TEST_ASSERT_EQUAL(static_cast<uint8_t>(SystemMode::Idle),
                      static_cast<uint8_t>(mode));
}

static void test_invalid_numeric_commands_are_rejected() {
    DSPPipeline dsp;
    AllanTracker allan;
    SystemMode mode = SystemMode::Measuring;

    dsp.setKCal(2.33f);
    Serial.setInput("CMD:KCAL=nan\nCMD:DTDOP=999\n");
    TelemetryManager::processIncomingCommands(mode, dsp, allan);

    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.33f, dsp.getKCal());
}

static void test_allan_constant_data_is_ready_and_finite() {
    AllanTracker allan;
    allan.setAllowedError(0.20f);

    for (size_t i = 0; i < 512; ++i) {
        allan.pushSample(100.0f, 2.33f);
    }

    TEST_ASSERT_TRUE(std::isfinite(allan.getTauOptSec()));
    TEST_ASSERT_TRUE(std::isfinite(allan.getSigmaTOpt()));
    TEST_ASSERT_TRUE(std::isfinite(allan.getAdaptivelyFilteredValue()));
    TEST_ASSERT_TRUE(allan.getTauOptSec() >= 0.1f);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_trimmed_mean_rejects_extremes);
    RUN_TEST(test_detector_linearization_reference);
    RUN_TEST(test_asymmetry_calibration_and_interpolation);
    RUN_TEST(test_decimation_and_history_wrap);
    RUN_TEST(test_crc16_known_vector);
    RUN_TEST(test_serial_commands);
    RUN_TEST(test_invalid_numeric_commands_are_rejected);
    RUN_TEST(test_allan_constant_data_is_ready_and_finite);
    UNITY_END();
}

int main() {
    setup();
    return 0;
}
