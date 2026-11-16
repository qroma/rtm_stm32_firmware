#pragma once
#include <Arduino.h>

// --- Піни керування НВЧ-комутатором HMC849 ---
constexpr uint8_t PIN_SW_V1       = D8;  // Керування станом SPDT (Антена / Еталон)
constexpr uint8_t PIN_SW_EN       = D9;  // Enable комутатора (якщо використовується)
constexpr uint8_t PIN_ADC_DET     = A0;  // Вхід АЦП від детектора (PA0 / ADC1_IN1)

// --- Часові параметри модуляції (Схема Дайка) ---
constexpr uint32_t DICKIE_FREQ_HZ       = 200;    // Частота комутації f_sw = 200 Гц
constexpr uint32_t HALF_PERIOD_US       = 2500;   // Напівперіод = 2.5 мс (2500 мкс)
constexpr uint32_t BLANKING_TIME_US     = 150;    // Захисна пауза після перемикання (150 мкс)

// --- Параметри оцифрування та рангової фільтрації ---
constexpr size_t   ADC_SAMPLES_PER_HALF = 64;     // Кількість вибірок за одну напівфазу (DMA)
constexpr float    TRIM_PERCENTAGE      = 0.125f; // Відсікання 12.5% зверху і знизу (по 8 вибірок із 64)
constexpr float    ADC_VREF_MV          = 3300.0f;// Опорна напруга АЦП у мВ (3.3 В)
constexpr float    ADC_MAX_COUNTS       = 4095.0f;// 12-бітний АЦП

// --- Децимація та компенсація асиметрії k(t) ---
constexpr uint16_t DECIMATION_FACTOR    = 20;     // Усереднення пачками по 20 циклів (20 * 5 мс = 100 мс)
constexpr size_t   INTERP_BUFFER_SIZE   = 300;    // Буфер на T_cal = 30 с при кроці 100 мс (300 елементів)
constexpr uint32_t I2C_POLL_INTERVAL_MS = 1000;   // Період опитування термодатчика еталона (1 с)

// --- Параметри алгоритму девіації Аллана ---
constexpr size_t   ALLAN_WINDOW_SIZE    = 128;    // Оперативне вікно відліків (після децимації: 12.8 с)
constexpr float    DEFAULT_K_CAL        = 15.0f;  // Крутизна k_cal за замовчуванням (мВ/К)
constexpr float    DEFAULT_DT_DOP       = 0.20f;  // Допустима похибка готовності системи (0.2 °C)

// --- Адреса I2C датчика температури еталона (SHT3x) ---
constexpr uint8_t  SHT3X_I2C_ADDR       = 0x44;