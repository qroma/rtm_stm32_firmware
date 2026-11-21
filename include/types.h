#pragma once
#include <cstdint>

// Поточна фаза комутації вхідного SPDT-комутатора (HMC849)
enum class RFSource : uint8_t {
    Antenna   = 0,  // Канал RF1 -> RFC (контактна патч-антена)
    Reference = 1   // Канал RF2 -> RFC (еталонне навантаження 50 Ом)
};

// Режими роботи скінченного автомата радіотермометра
enum class SystemMode : uint8_t {
    Idle            = 0,  // Очікування (апаратний збір зупинено)
    Measuring       = 1,  // Штатний режим вимірювання (схема Дікке)
    CalibratingAsym = 2   // Режим калібрування асиметрії комутаційних плечей k
};

// Децимований відлік (усереднення за DECIMATION_FACTOR циклів = 100 мс)
struct DecimatedSample {
    uint32_t timestamp_ms = 0;     // Мітка часу (мс)
    float    v_ant_mv     = 0.0f;  // Очищена напруга детектора у фазі антени (мВ)
    float    v_et_mv      = 0.0f;  // Очищена напруга детектора у фазі еталона (мВ)
};

// Внутрішня структура передавання телеметрії (DTO).
// Серіалізується у текстовий ASCII CSV-кадр "$RTM,...*CRC16" у TelemetryManager::sendFrameCSV().
struct TelemetryFrame {
    uint32_t frame_counter = 0;     // Наскрізний лічильник кадрів
    uint32_t timestamp_ms  = 0;     // Мітка часу формування кадру (мс)
    uint8_t  mode          = 0;     // Поточний режим (SystemMode)
    uint8_t  is_ready      = 0;     // Критерій готовності (1 — sigma_T <= dT_dop, 0 — не готово)
    float    v_ant_mv      = 0.0f;  // Напруга детектора фази антени (мВ)
    float    v_et_mv       = 0.0f;  // Напруга детектора фази еталона (мВ)
    float    dv_cor_mv     = 0.0f;  // Скомпенсований різницевий сигнал (лінеаризована шкала потужності)
    float    t_a_celsius   = 0.0f;  // Первинна оцінка ефективної температури антени T_A (°C)
    float    t_et_celsius  = 25.0f; // Термодинамічна температура еталона T_et (°C)
    float    kappa_curr    = 1.0f;  // Поточний коефіцієнт асиметрії комутаційних плечей k
    float    tau_opt_sec   = 0.0f;  // Оптимальний час інтегрування tau_opt за Алланом (с)
    float    sigma_t_opt   = 0.0f;  // Оцінка флуктуаційної чутливості sigma_T(tau_opt) (°C)
};