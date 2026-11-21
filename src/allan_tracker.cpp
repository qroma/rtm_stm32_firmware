#include "allan_tracker.h"
#include <algorithm>
#include <cmath>

void AllanTracker::pushSample(float dv_cor_lin, float k_cal) {
    // Захист від переповнення int32_t при насиченні детектора або обриві антени
    float scaled = dv_cor_lin * 1000.0f;
    if (scaled > 2.0e9f)  scaled =  2.0e9f;
    if (scaled < -2.0e9f) scaled = -2.0e9f;

    ring_samples_scaled_[head_] = static_cast<int32_t>(lroundf(scaled));
    head_ = (head_ + 1) % ALLAN_WINDOW_SIZE;
    if (sample_count_ < ALLAN_WINDOW_SIZE) {
        sample_count_++;
    }

    // Перераховуємо девіацію Аллана кожні 16 нових відліків (1.6 с) після заповнення вікна
    if (sample_count_ == ALLAN_WINDOW_SIZE && (head_ % 16 == 0)) {
        evaluateAllanDeviation(k_cal);
    }
}

size_t AllanTracker::pushAndGetMedianM(size_t new_m) {
    m_history_[m_hist_head_] = new_m;
    m_hist_head_ = (m_hist_head_ + 1) % ALLAN_MEDIAN_LEN;
    if (m_hist_count_ < ALLAN_MEDIAN_LEN) {
        m_hist_count_++;
    }

    size_t sorted[ALLAN_MEDIAN_LEN];
    for (size_t i = 0; i < m_hist_count_; ++i) {
        sorted[i] = m_history_[i];
    }
    std::sort(sorted, sorted + m_hist_count_);
    return sorted[m_hist_count_ / 2];
}

void AllanTracker::evaluateAllanDeviation(float k_cal) {
    if (k_cal <= 1e-6f) {
        is_ready_ = false;
        return;
    }

    // 1. Масив кумулятивних сум C_j = sum(y_k) у 64-бітній цілочисельній арифметиці (2.3.5)
    cum_sum_scaled_[0] = 0;
    for (size_t i = 0; i < ALLAN_WINDOW_SIZE; ++i) {
        size_t idx = (head_ + i) % ALLAN_WINDOW_SIZE;
        cum_sum_scaled_[i + 1] = cum_sum_scaled_[i] + ring_samples_scaled_[idx];
    }

    // 2. Октавний набір m = 1, 2, 4, 8, 16, 32 (m_max = 32 = N/8 -> tau до 3.2 с)
    static constexpr size_t m_values[] = {1, 2, 4, 8, 16, 32};
    float  min_sigma_t = 999.0f;
    size_t best_m      = 8;

    for (size_t m : m_values) {
        if (2 * m >= ALLAN_WINDOW_SIZE) break;
        size_t num_terms   = ALLAN_WINDOW_SIZE - 2 * m + 1;
        double sum_sq_diff = 0.0;

        for (size_t i = 0; i < num_terms; ++i) {
            int64_t sum_first  = cum_sum_scaled_[i + m]     - cum_sum_scaled_[i];
            int64_t sum_second = cum_sum_scaled_[i + 2 * m] - cum_sum_scaled_[i + m];
            double  diff       = static_cast<double>(sum_second - sum_first) /
                                 (static_cast<double>(m) * 1000.0);
            sum_sq_diff += diff * diff;
        }

        // Формула (2.28): Варіація та Девіація Аллана
        float sigma_a = sqrtf(static_cast<float>(sum_sq_diff / (2.0 * static_cast<double>(num_terms))));
        float sigma_t = sigma_a / k_cal;

        if (sigma_t < min_sigma_t) {
            min_sigma_t = sigma_t;
            best_m      = m;
        }
    }

    // 3. Медіанна стабілізація оптимального вікна tau_opt за останніми 5 оцінками
    size_t stable_m = pushAndGetMedianM(best_m);
    tau_opt_sec_    = static_cast<float>(stable_m) * BASE_TAU0_SEC;
    sigma_t_opt_    = min_sigma_t;

    // 4. Критерій метрологічної готовності системи (крок 5, підрозділ 2.3.5)
    is_ready_ = (sigma_t_opt_ <= dt_dop_);
}

float AllanTracker::getAdaptivelyFilteredValue() const {
    if (sample_count_ == 0) return 0.0f;

    // Коректне округлення без похибки відкидання дробової частини float
    long raw_len = lroundf(tau_opt_sec_ / BASE_TAU0_SEC);
    size_t window_len = (raw_len < 1) ? 1U : static_cast<size_t>(raw_len);
    if (window_len > sample_count_) {
        window_len = sample_count_;
    }

    int64_t sum_scaled = 0;
    for (size_t i = 0; i < window_len; ++i) {
        size_t idx = (head_ + ALLAN_WINDOW_SIZE - 1 - i) % ALLAN_WINDOW_SIZE;
        sum_scaled += ring_samples_scaled_[idx];
    }
    return static_cast<float>(sum_scaled) / (static_cast<float>(window_len) * 1000.0f);
}