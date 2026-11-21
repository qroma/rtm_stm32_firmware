#include "dsp_pipeline.h"
#include <algorithm>
#include <cmath>

float DSPPipeline::computeTrimmedMeanMv(uint16_t* samples, size_t count) {
    // 1. Швидке сортування масиву вибірок напівфази (std::sort на 64 елементах займає ~15 мкс на Cortex-M4)
    std::sort(samples, samples + count);

    // 2. Симетричне відсікання екстремальних викидів (12.5% найменших та 12.5% найбільших)
    size_t trim_count  = static_cast<size_t>(count * TRIM_PERCENTAGE);
    size_t valid_start = trim_count;
    size_t valid_end   = count - trim_count;

    uint32_t sum = 0;
    for (size_t i = valid_start; i < valid_end; ++i) {
        sum += samples[i];
    }

    // 3. Усереднення очищеного масиву (разом із децимацією 20 циклів дає 960 вибірок -> ENOB > 16 біт)
    float avg_counts = static_cast<float>(sum) / static_cast<float>(valid_end - valid_start);
    return (avg_counts / ADC_MAX_COUNTS) * ADC_VREF_MV;
}

float DSPPipeline::linearizeDetectorMv(float v_mv) {
    // Програмна лінеаризація лог-лінійної характеристики ADL5902:
    // V_lin = V_0 * 10^(V_det / (10 * S_dB)), де нормування на DETECTOR_V_REF_MV
    // запобігає втраті точності  32-бітного float
    return 1000.0f * powf(10.0f, (v_mv - DETECTOR_V_REF_MV) / (10.0f * DETECTOR_SLOPE_MV_DB));
}

float DSPPipeline::computeAsymmetryKappa(float v_ant_mv, float v_et_mv) {
    // Формула (2.23) у лінеаризованій шкалі потужності: k = V_et1_lin / V_et2_lin
    float v_ant_lin = linearizeDetectorMv(v_ant_mv);
    float v_et_lin  = linearizeDetectorMv(v_et_mv);
    return (v_et_lin > 1e-6f) ? (v_ant_lin / v_et_lin) : 1.0f;
}

bool DSPPipeline::pushCycleAndDecimate(float v_ant_mv, float v_et_mv, uint32_t now_ms, DecimatedSample& out_sample) {
    accum_ant_mv_ += v_ant_mv;
    accum_et_mv_  += v_et_mv;
    decim_count_++;

    if (decim_count_ >= DECIMATION_FACTOR) {
        out_sample.timestamp_ms = now_ms;
        out_sample.v_ant_mv     = accum_ant_mv_ / static_cast<float>(DECIMATION_FACTOR);
        out_sample.v_et_mv      = accum_et_mv_  / static_cast<float>(DECIMATION_FACTOR);

        // Зберігаємо у кільцевий буфер для кусково-лінійної інтерполяції між калібруваннями
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

float DSPPipeline::getInterpolatedKappa(uint32_t timestamp_ms) const {
    // Формула (2.25): кусково-лінійна інтерполяція "плаваючого" коефіцієнта асиметрії
    if (t_cal_curr_ <= t_cal_prev_) {
        return kappa_curr_;
    }
    if (timestamp_ms <= t_cal_prev_) {
        return kappa_prev_;
    }
    if (timestamp_ms >= t_cal_curr_) {
        return kappa_curr_;
    }
    float alpha = static_cast<float>(timestamp_ms - t_cal_prev_) /
                  static_cast<float>(t_cal_curr_ - t_cal_prev_);
    return kappa_prev_ + (kappa_curr_ - kappa_prev_) * alpha;
}

float DSPPipeline::getCompensatedDiff(float v_ant_mv, float v_et_mv) const {
    // Формула (2.24) у лінеаризованій шкалі: dV_cor = (V_ant_lin / k) - V_et_lin
    float v_ant_lin = linearizeDetectorMv(v_ant_mv);
    float v_et_lin  = linearizeDetectorMv(v_et_mv);
    return (v_ant_lin / kappa_curr_) - v_et_lin;
}

float DSPPipeline::getCompensatedDiffInterpolated(const DecimatedSample& sample) const {
    // Формула (2.26): скоригований різницевий сигнал із лінійною інтерполяцією k_interp(t)
    float v_ant_lin = linearizeDetectorMv(sample.v_ant_mv);
    float v_et_lin  = linearizeDetectorMv(sample.v_et_mv);
    float k_interp  = getInterpolatedKappa(sample.timestamp_ms);
    return (v_ant_lin / k_interp) - v_et_lin;
}

float DSPPipeline::getPrimaryAntennaTemp(float dv_cor_lin, float t_et_celsius) const {
    // Формула (2.10): T_A = T_et + dV_cor / k_cal (k_cal визначається у лінеаризованих одиницях/К)
    return t_et_celsius + (dv_cor_lin / k_cal_);
}