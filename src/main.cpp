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

SystemMode   current_mode  = SystemMode::Measuring;
uint32_t     frame_counter = 0;

// Сирі буфери АЦП для напівперіодів модуляції
uint16_t raw_ant_buf[ADC_SAMPLES_PER_HALF];
uint16_t raw_et_buf[ADC_SAMPLES_PER_HALF];

void setup() {
  // setup code here, to run once:
    Serial.begin(115200);
    rf_hw.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  // 1. Перевірка вхідних команд від ПК по UART
    TelemetryManager::processIncomingCommands(current_mode, dsp, allan);

    if (current_mode == SystemMode::Idle) {
        delay(10);
        return;
    }

    // 2. Фаза 1 комутації: RFSource::Antenna (RF1 -> RFC), захисна пауза 150 мкс, оцифрування V_ant
    rf_hw.setSwitchState(RFSource::Antenna);
    rf_hw.acquireHalfPeriodSamples(raw_ant_buf, ADC_SAMPLES_PER_HALF);

    // 3. Фаза 2 комутації: RFSource::Reference (RF2 -> RFC), захисна пауза 150 мкс, оцифрування V_et
    rf_hw.setSwitchState(RFSource::Reference);
    rf_hw.acquireHalfPeriodSamples(raw_et_buf, ADC_SAMPLES_PER_HALF);

    // 4. Первинна фільтрація (усічене середнє — відсікання імпульсних завад Wi-Fi/GSM)
    float v_ant_clean_mv = DSPPipeline::computeTrimmedMeanMv(raw_ant_buf, ADC_SAMPLES_PER_HALF);
    float v_et_clean_mv  = DSPPipeline::computeTrimmedMeanMv(raw_et_buf,  ADC_SAMPLES_PER_HALF);

    // 5. Асинхронне опитування датчика температури еталона по I2C
    uint32_t now_ms = millis();
    rf_hw.updateReferenceTemperatureAsync(now_ms);

    // 6. Попередня децимація (кожні 20 циклів = 100 мс формуємо вихідний відлік та оновлюємо буфер)
    DecimatedSample dec_sample;
    if (dsp.pushCycleAndDecimate(v_ant_clean_mv, v_et_clean_mv, now_ms, dec_sample)) {
        
        // Якщо активний режим калібрування асиметрії (обидва входи підключені до еталона):
        if (current_mode == SystemMode::CalibratingAsym && dec_sample.v_et_mv > 1.0f) {
            float measured_kappa = dec_sample.v_ant_mv / dec_sample.v_et_mv; // Формула (2.23)
            dsp.updateCalibrationKappa(measured_kappa, now_ms);
        }

        // Компенсація асиметрії та обчислення різницевого сигналу
        float dv_raw_cor = dsp.getCompensatedDiff(dec_sample.v_ant_mv, dec_sample.v_et_mv);

        // Оновлення масиву кумулятивних сум девіації Аллана, пошук tau_opt та адаптивна фільтрація
        allan.pushSample(dv_raw_cor, dsp.getKCal());
        float dv_filtered_mv = allan.getAdaptivelyFilteredValue();

        // Обчислення первинної температури антени T_A за формулою двоточкового калібрування (2.10)
        float t_et = rf_hw.getReferenceTemperature();
        float t_a  = dsp.getPrimaryAntennaTemp(dv_filtered_mv, t_et);

        // 7. Формування та відправлення телеметричного кадру з CRC-16 на ПК (10 кадрів/с)
        TelemetryFrame frame{};
        frame.sync_word     = 0xAA55;
        frame.frame_counter = ++frame_counter;
        frame.timestamp_ms  = now_ms;
        frame.mode          = static_cast<uint8_t>(current_mode);
        frame.is_ready      = allan.isSystemReady() ? 1 : 0;
        frame.v_ant_mv      = dec_sample.v_ant_mv;
        frame.v_et_mv       = dec_sample.v_et_mv;
        frame.dv_cor_mv     = dv_filtered_mv;
        frame.t_a_celsius   = t_a;
        frame.t_et_celsius  = t_et;
        frame.kappa_curr    = dsp.getKappa();
        frame.tau_opt_sec   = allan.getTauOptSec();
        frame.sigma_t_opt   = allan.getSigmaTOpt();

        TelemetryManager::sendFrameCSV(frame);
    }
}
