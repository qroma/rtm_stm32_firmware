#include "dsp_pipeline.h"
#include <algorithm>

float DSPPipeline::computeTrimmedMeanMv(uint16_t* samples, size_t count) {
    // 1. Швидке сортування масиву вибірок напівфази (std::sort на 64 елементах займає ~15 мкс на Cortex-M4)
    std::sort(samples, samples + count);

    // 2. Симетричне відсікання екстремальних викидів (12.5% найменших та 12.5% найбільших)
    size_t trim_count = static_cast<size_t>(count * TRIM_PERCENTAGE);
    size_t valid_start = trim_count;
    size_t valid_end   = count - trim_count;

    uint32_t sum = 0;
    for (size_t i = valid_start; i < valid_end; ++i) {
        sum += samples[i];
    }

    // 3. Формування середнього значення з підвищеною ефективною розрядністю (oversampling)
    float avg_counts = static_cast<float>(sum) / static_cast<float>(valid_end - valid_start);
    return (avg_counts / ADC_MAX_COUNTS) * ADC_VREF_MV;
}

bool DSPPipeline::pushCycleAndDecimate(float v_ant_mv, float v_et_mv, uint32_t now_ms, DecimatedSample& out_sample) {
    accum_ant_mv_ += v_ant_mv;
    accum_et_mv_  += v_et_mv;
    decim_count_++;

    if (decim_count_ >= DECIMATION_FACTOR) {
        out_sample.timestamp_ms = now_ms;
        out_sample.v_ant_mv     = accum_ant_mv_ / static_cast<float>(DECIMATION_FACTOR);
        out_sample.v_et_mv      = accum_et_mv_  / static_cast<float>(DECIMATION_FACTOR);

        // Зберігаємо у кільцевий буфер для лінійної інтерполяції між сеансами калібрування
        history_buf_[buf_head_] = out_sample;
        buf_head_ = (buf_head_ + 1) % INTERP_BUFFER_SIZE;

        accum_ant_mv_ = 0.0f;
        accum_et_mv_  = 0.0f;
        decim_count_  = 0;
        return true;
    }
    return false;
}

void DSPPipeline::updateCalibrationKappa(float new_kappa, uint32_t cal_timestamp_ms) {
    kappa_prev_ = kappa_curr_;
    t_cal_prev_ = t_cal_curr_;
    kappa_curr_ = new_kappa;
    t_cal_curr_ = cal_timestamp_ms;
}

float DSPPipeline::getCompensatedDiff(float v_ant_mv, float v_et_mv) const {
    // Формула (2.24): dV_cor = (V_ant / k) - V_et
    return (v_ant_mv / kappa_curr_) - v_et_mv;
}

float DSPPipeline::getPrimaryAntennaTemp(float dv_cor_mv, float t_et_celsius) const {
    // Формула (2.10): T_A = T_et + dV_cor / k_cal
    return t_et_celsius + (dv_cor_mv / k_cal_);
}