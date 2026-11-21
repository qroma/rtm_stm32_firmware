#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "rf_hal.h"
#include "dsp_pipeline.h"
#include "allan_tracker.h"
#include "telemetry.h"

RFHardware   rf_hw;
DSPPipeline  dsp;
AllanTracker allan;

SystemMode current_mode  = SystemMode::Measuring;
uint32_t   frame_counter = 0;

// Робочі буфери для копії готового ping-pong банку DMA
uint16_t raw_ant_buf[ADC_SAMPLES_PER_HALF];
uint16_t raw_et_buf[ADC_SAMPLES_PER_HALF];

void setup() {
    Serial.begin(UART_BAUD_RATE);
    rf_hw.init();

    if (current_mode != SystemMode::Idle) {
        rf_hw.startAcquisition();
    }
}

void loop() {
    // 1. Неблокуюча обробка команд від ПК по UART
    TelemetryManager::processIncomingCommands(current_mode, dsp, allan);

    if (current_mode == SystemMode::Idle) {
        if (rf_hw.isRunning()) {
            rf_hw.stopAcquisition();
        }
        return;
    }

    if (!rf_hw.isRunning()) {
        rf_hw.startAcquisition();
    }

    // 2. Асинхронне опитування термодатчика еталона (SHT3x) — виконується паралельно з DMA
    uint32_t now_ms = millis();
    rf_hw.updateReferenceTemperatureAsync(now_ms);

    // 3. Очікуємо готову пару фаз (V_ant, V_et) від апаратного конвеєра TIM2 + TIM6 + ADC1/DMA
    if (!rf_hw.fetchReadyCycle(raw_ant_buf, raw_et_buf)) {
        return;
    }

    // 4. Первинна фільтрація (усічене середнє по 256 вибірках — відсікання імпульсних завад)
    float v_ant_clean_mv = DSPPipeline::computeTrimmedMeanMv(raw_ant_buf, ADC_SAMPLES_PER_HALF);
    float v_et_clean_mv  = DSPPipeline::computeTrimmedMeanMv(raw_et_buf,  ADC_SAMPLES_PER_HALF);

    // 5. Попередня децимація (кожні 20 циклів = 100 мс формуємо відлік та оновлюємо буфер)
    DecimatedSample dec_sample{};
    if (dsp.pushCycleAndDecimate(v_ant_clean_mv, v_et_clean_mv, now_ms, dec_sample)) {

        // У режимі калібрування асиметрії обчислюємо k у лінеаризованій шкалі ADL5902 (2.23)
        if (current_mode == SystemMode::CalibratingAsym && dec_sample.v_et_mv > 1.0f) {
            float measured_kappa = DSPPipeline::computeAsymmetryKappa(dec_sample.v_ant_mv, dec_sample.v_et_mv);
            dsp.updateCalibrationKappa(measured_kappa, now_ms);
        }

        // Компенсація асиметрії в лінеаризованій шкалі потужності (2.24)
        float dv_raw_cor = dsp.getCompensatedDiff(dec_sample.v_ant_mv, dec_sample.v_et_mv);

        // Оновлення кумулятивних сум Аллана, пошук tau_opt та адаптивна фільтрація
        allan.pushSample(dv_raw_cor, dsp.getKCal());
        float dv_filtered = allan.getAdaptivelyFilteredValue();

        // Первинна оцінка температури антени T_A (2.10)
        float t_et = rf_hw.getReferenceTemperature();
        float t_a  = dsp.getPrimaryAntennaTemp(dv_filtered, t_et);

        // 6. Формування та відправлення кадру з CRC-16 на ПК (10 кадрів/с, не впливає на цикл Дікке)
        TelemetryFrame frame{};
        frame.sync_word     = 0xAA55;
        frame.frame_counter = ++frame_counter;
        frame.timestamp_ms  = now_ms;
        frame.mode          = static_cast<uint8_t>(current_mode);
        frame.is_ready      = allan.isSystemReady() ? 1 : 0;
        frame.v_ant_mv      = dec_sample.v_ant_mv;
        frame.v_et_mv       = dec_sample.v_et_mv;
        frame.dv_cor_mv     = dv_filtered;
        frame.t_a_celsius   = t_a;
        frame.t_et_celsius  = t_et;
        frame.kappa_curr    = dsp.getKappa();
        frame.tau_opt_sec   = allan.getTauOptSec();
        frame.sigma_t_opt   = allan.getSigmaTOpt();

        TelemetryManager::sendFrameCSV(frame);
    }
}