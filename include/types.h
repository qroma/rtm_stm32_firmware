#pragma once
#include <stdint.h>

// Стан вхідного НВЧ-комутатора
enum class RFSource : uint8_t {
    Antenna   = 0, // Канал RF1 (Антена)
    Reference = 1  // Канал RF2 (Еталон 50 Ом)
};

// Режим роботи радіотермометра
enum class SystemMode : uint8_t {
    Idle               = 0, // Очікування
    Measuring          = 1, // Штатне вимірювання (модуляція Антена/Еталон)
    CalibratingAsym    = 2  // Визначення коефіцієнта асиметрії k (еталон на обох входах)
};

// Елемент буфера для ретроспективної лінійної інтерполяції k_interp(t)
struct DecimatedSample {
    uint32_t timestamp_ms;
    float    v_ant_mv;
    float    v_et_mv;
};

// Структура телеметричного кадру для передачі на ПК (вирівняна по байтах)
#pragma pack(push, 1)
struct TelemetryFrame {
    uint16_t sync_word;     // Маркер початку кадру (0xAA55)
    uint32_t frame_counter; // Наскрізний лічильник кадрів
    uint32_t timestamp_ms;  // Мітка часу МК (мс)
    uint8_t  mode;          // Поточний режим (SystemMode)
    uint8_t  is_ready;      // Ознака готовності (1: sigma_T <= dT_dop, 0: прогрів/нестабільність)
    float    v_ant_mv;      // Очищена напруга антени (мВ)
    float    v_et_mv;       // Очищена напруга еталона (мВ)
    float    dv_cor_mv;     // Скомпенсований різницевий сигнал dV_cor (мВ)
    float    t_a_celsius;   // Первинна ефективна температура антени T_A (°C)
    float    t_et_celsius;  // Температура еталона T_et з датчика I2C (°C)
    float    kappa_curr;    // Діючий коефіцієнт асиметрії k
    float    tau_opt_sec;   // Поточний оптимальний час інтегрування tau_opt (с)
    float    sigma_t_opt;   // Поточна девіація Аллана в точці мінімуму (°C)
    uint16_t crc16;         // Контрольна сума CRC-16-CCITT
};
#pragma pack(pop)