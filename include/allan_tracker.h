#pragma once
#include "config.h"

class AllanTracker {
public:
    // Додає черговий скомпенсований відлік (крок tau_0 = 0.1 с після децимації)
    void pushSample(float dv_cor_mv, float k_cal);

    float getTauOptSec() const { return tau_opt_sec_; }
    float getSigmaTOpt() const { return sigma_t_opt_; }
    bool  isSystemReady() const { return is_ready_; }
    void  setAllowedError(float dt_dop) { dt_dop_ = dt_dop; }

    // Повертає адаптивно усереднене значення сигналу у вікні шириною tau_opt
    float getAdaptivelyFilteredValue() const;

private:
    void evaluateAllanDeviation(float k_cal);

    int32_t  ring_samples_uv_[ALLAN_WINDOW_SIZE] = {0}; // Відліки у мікровольтах (цілочисельні)
    int64_t  cum_sum_uv_[ALLAN_WINDOW_SIZE + 1]  = {0}; // Масив кумулятивних сум C_j (int64_t)
    size_t   sample_count_ = 0;
    size_t   head_         = 0;

    float    tau_opt_sec_  = 1.0f;
    float    sigma_t_opt_  = 0.50f;
    float    dt_dop_       = DEFAULT_DT_DOP;
    bool     is_ready_     = false;
};