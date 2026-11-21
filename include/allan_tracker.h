#pragma once
#include "config.h"

class AllanTracker {
public:
    // Додає черговий скомпенсований лінеаризований відлік (крок tau_0 = 0.1 с після децимації)
    void pushSample(float dv_cor_lin, float k_cal);

    float getTauOptSec() const { return tau_opt_sec_; }
    float getSigmaTOpt() const { return sigma_t_opt_; }
    bool  isSystemReady() const { return is_ready_; }
    void  setAllowedError(float dt_dop) { dt_dop_ = dt_dop; }

    // Повертає адаптивно усереднене значення сигналу у вікні шириною tau_opt
    float getAdaptivelyFilteredValue() const;

private:
    void   evaluateAllanDeviation(float k_cal);
    size_t pushAndGetMedianM(size_t new_m);

    // Базовий період після децимації = 0.1 с (DECIMATION_FACTOR / DICKE_FREQ_HZ)
    static constexpr float BASE_TAU0_SEC =
        static_cast<float>(DECIMATION_FACTOR) / static_cast<float>(DICKE_FREQ_HZ);

    int32_t ring_samples_scaled_[ALLAN_WINDOW_SIZE]{};   // Фіксована крапка (x1000) із захистом від переповнення
    int64_t cum_sum_scaled_[ALLAN_WINDOW_SIZE + 1]{};    // Кумулятивні суми C_j (int64_t)
    size_t  sample_count_ = 0;
    size_t  head_         = 0;

    // Буфер медіанної стабілізації tau_opt (крок 3 алгоритму в підрозділі 2.3.5)
    size_t  m_history_[ALLAN_MEDIAN_LEN] = {8, 8, 8, 8, 8};
    size_t  m_hist_head_                 = 0;
    size_t  m_hist_count_                = 0;

    float   tau_opt_sec_ = 0.8f;
    float   sigma_t_opt_ = 0.50f;
    float   dt_dop_      = DEFAULT_DT_DOP;
    bool    is_ready_    = false;
};