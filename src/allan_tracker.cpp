#include "allan_tracker.h"
#include <math.h>

void AllanTracker::pushSample(float dv_cor_mv, float k_cal) {
    // Переводимо мілівольти у мікровольти (int32_t) для точного цілочисельного підсумовування
    int32_t val_uv = static_cast<int32_t>(dv_cor_mv * 1000.0f);
    ring_samples_uv_[head_] = val_uv;
    head_ = (head_ + 1) % ALLAN_WINDOW_SIZE;
    if (sample_count_ < ALLAN_WINDOW_SIZE) {
        sample_count_++;
    }

    // Перераховуємо девіацію Аллана, коли вікно заповнене (кожні 16 нових відліків = 1.6 с)
    if (sample_count_ == ALLAN_WINDOW_SIZE && (head_ % 16 == 0)) {
        evaluateAllanDeviation(k_cal);
    }
}

void AllanTracker::evaluateAllanDeviation(float k_cal) {
    // 1. Будуємо масив кумулятивних сум C_j = sum(y_k) у 64-бітній цілочисельній арифметиці (підрозділ 2.3.5)
    cum_sum_uv_[0] = 0;
    for (size_t i = 0; i < ALLAN_WINDOW_SIZE; ++i) {
        size_t idx = (head_ + i) % ALLAN_WINDOW_SIZE;
        cum_sum_uv_[i + 1] = cum_sum_uv_[i] + ring_samples_uv_[idx];
    }

    // 2. Перебираємо октавні коефіцієнти усереднення m = 1, 2, 4, 8, 16 (m_max <= N / 8)
    const size_t m_values[] = {1, 2, 4, 8, 16};
    float min_sigma_t = 999.0f;
    size_t best_m = 4;

    for (size_t m : m_values) {
        size_t num_terms = ALLAN_WINDOW_SIZE - 2 * m + 1;
        double sum_sq_diff = 0.0;

        for (size_t i = 0; i < num_terms; ++i) {
            // Ковзні середні за одну операцію віднімання кумулятивних сум:
            int64_t sum_first  = cum_sum_uv_[i + m]     - cum_sum_uv_[i];
            int64_t sum_second = cum_sum_uv_[i + 2 * m] - cum_sum_uv_[i + m];
            double diff_mv = static_cast<double>(sum_second - sum_first) / (static_cast<double>(m) * 1000.0);
            sum_sq_diff += diff_mv * diff_mv;
        }

        // Формула (2.28): Варіація та Девіація Аллана
        float sigma_a_mv = sqrtf(static_cast<float>(sum_sq_diff / (2.0 * num_terms)));
        float sigma_t    = sigma_a_mv / k_cal; // Перехід у градуси Кельвіна/Цельсія

        if (sigma_t < min_sigma_t) {
            min_sigma_t = sigma_t;
            best_m = m;
        }
    }

    // Базовий період після децимації = 0.1 с (DECIMATION_FACTOR * 5 мс)
    constexpr float BASE_TAU0_SEC = (DECIMATION_FACTOR * 5.0f) / 1000.0f;
    tau_opt_sec_ = static_cast<float>(best_m) * BASE_TAU0_SEC;
    sigma_t_opt_ = min_sigma_t;

    // Крок 5 алгоритму (2.3.5): Перевірка критерію готовності системи
    is_ready_ = (sigma_t_opt_ <= dt_dop_);
}

float AllanTracker::getAdaptivelyFilteredValue() const {
    if (sample_count_ == 0) return 0.0f;
    
    // Ширина вікна ФНЧ у відліках відповідає знайденому tau_opt
    size_t window_len = static_cast<size_t>(tau_opt_sec_ / 0.1f);
    if (window_len < 1) window_len = 1;
    if (window_len > sample_count_) window_len = sample_count_;

    int64_t sum_uv = 0;
    for (size_t i = 0; i < window_len; ++i) {
        size_t idx = (head_ + ALLAN_WINDOW_SIZE - 1 - i) % ALLAN_WINDOW_SIZE;
        sum_uv += ring_samples_uv_[idx];
    }
    return static_cast<float>(sum_uv) / (static_cast<float>(window_len) * 1000.0f);
}