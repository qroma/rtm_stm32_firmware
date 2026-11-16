#pragma once
#include "config.h"
#include "types.h"

class DSPPipeline {
public:
    // Обчислення усіченого середнього з перетворенням у мілівольти (oversampling float)
    static float computeTrimmedMeanMv(uint16_t* samples, size_t count);

    // Додає пару (V_ant, V_et) одного періоду комутації (5 мс); повертає true, коли пачка децимації готова (кожні 100 мс)
    bool pushCycleAndDecimate(float v_ant_mv, float v_et_mv, uint32_t now_ms, DecimatedSample& out_sample);

    // Керування коефіцієнтом асиметрії k та ретроспективною лінійною інтерполяцією
    void updateCalibrationKappa(float new_kappa, uint32_t cal_timestamp_ms);
    float getCompensatedDiff(float v_ant_mv, float v_et_mv) const;
    float getPrimaryAntennaTemp(float dv_cor_mv, float t_et_celsius) const;

    float getKappa() const { return kappa_curr_; }
    void  setKCal(float k_cal) { k_cal_ = k_cal; }
    float getKCal() const { return k_cal_; }

private:
    // Акумулятори децимації
    float    accum_ant_mv_ = 0.0f;
    float    accum_et_mv_  = 0.0f;
    uint16_t decim_count_  = 0;

    // Параметри асиметрії та калібрування
    float    kappa_prev_   = 1.047f; // Попередня оцінка k_m
    float    kappa_curr_   = 1.047f; // Актуальна оцінка k_{m+1}
    uint32_t t_cal_prev_   = 0;
    uint32_t t_cal_curr_   = 0;
    float    k_cal_        = DEFAULT_K_CAL;

    // Кільцевий буфер на 300 елементів для кусково-лінійної інтерполяції (2.3.4)
    DecimatedSample history_buf_[INTERP_BUFFER_SIZE];
    size_t          buf_head_ = 0;
};