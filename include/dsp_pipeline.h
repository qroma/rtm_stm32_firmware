#pragma once
#include <cstddef>
#include <cstdint>
#include "config.h"
#include "types.h"

class DSPPipeline {
public:
    // Callback для передавання ретроспективно перерахованих відліків після завершення калібрування
    using InterpolatedSampleCallback = void (*)(uint32_t timestamp_ms,
                                                float dv_interp_lin,
                                                float t_a_interp_c,
                                                float k_interp);

    // 1. Рангова фільтрація імпульсних завад (усічене середнє) + перехід у мВ
    static float computeTrimmedMeanMv(uint16_t* samples, size_t count);

    // 2. Програмна лінеаризація логарифмічної характеристики детектора ADL5902 (мВ -> лінійна шкала потужності)
    static float linearizeDetectorMv(float v_mv);

    // 3. Обчислення точкової оцінки коефіцієнта асиметрії k у лінеаризованій шкалі (формула 2.23)
    static float computeAsymmetryKappa(float v_ant_mv, float v_et_mv);

    // 4. Попередня децимація (20 циклів = 100 мс) та опціональне збереження у кільцевий буфер історії
    bool pushCycleAndDecimate(float v_ant_mv,
                              float v_et_mv,
                              uint32_t now_ms,
                              bool store_in_history,
                              DecimatedSample& out_sample);

    // 5. Усереднене калібрування асиметрії за весь калібрувальний інтервал (CAL_ASYM_SAMPLES = 1.0 с)
    void beginAsymmetryCalibration();
    bool accumulateCalibrationSample(float v_ant_mv, float v_et_mv);
    bool finalizeAsymmetryCalibration(uint32_t cal_timestamp_ms);

    // 6. Пряме оновлення коефіцієнта асиметрії та параметрів інтерполяції
    void updateCalibrationKappa(float new_kappa, uint32_t cal_timestamp_ms);

    // 7. Кусково-лінійна інтерполяція "плаваючого" k_interp(t) (формула 2.25)
    float getInterpolatedKappa(uint32_t timestamp_ms) const;

    // 8. Оперативна компенсація асиметрії за поточним k_m для миттєвої індикації (формула 2.24)
    float getCompensatedDiff(float v_ant_mv, float v_et_mv) const;

    // 9. Ретроспективна компенсація відліку за інтерпольованим k_interp(t) (формула 2.26)
    float getCompensatedDiffInterpolated(const DecimatedSample& sample) const;

    // 10. Перерахунок та видача накопиченого буфера історії після чергового калібрування
    size_t flushInterpolatedHistory(float t_et_celsius, InterpolatedSampleCallback cb);

    // 11. Обчислення первинної температури антени T_A (формула 2.10)
    float getPrimaryAntennaTemp(float dv_cor_lin, float t_et_celsius) const;

    float getKappa() const { return kappa_curr_; }
    float getKCal() const  { return k_cal_; }
    void  setKCal(float k_cal) {
        if (k_cal > 1e-6f) {
            k_cal_ = k_cal;
        }
    }

private:
    // Акумулятори децимації
    float    accum_ant_mv_ = 0.0f;
    float    accum_et_mv_  = 0.0f;
    uint16_t decim_count_  = 0;

    // Кільцевий буфер децимованих відліків за інтервал T_cal (300 елементів = 30 с)
    DecimatedSample history_buf_[INTERP_BUFFER_SIZE]{};
    size_t          buf_head_  = 0;
    size_t          buf_count_ = 0;

    // Параметри кусково-лінійної інтерполяції коефіцієнта асиметрії
    float    kappa_prev_   = 1.0f;
    float    kappa_curr_   = 1.0f;
    uint32_t t_cal_prev_   = 0;
    uint32_t t_cal_curr_   = 0;
    bool     has_curr_cal_ = false;
    bool     has_prev_cal_ = false;

    // Акумулятори усередненого калібрування асиметрії
    float    cal_ant_lin_sum_ = 0.0f;
    float    cal_et_lin_sum_  = 0.0f;
    size_t   cal_sample_cnt_  = 0;

    // Крутизна двоточкового калібрування у лінеаризованій шкалі
    float    k_cal_ = DEFAULT_K_CAL;
};