#pragma once
#include <Arduino.h>

// --- Піни керування НВЧ-комутатором HMC849 ---
constexpr uint8_t PIN_SW_V1       = D8;  // Керування станом SPDT (Антена / Еталон)
constexpr uint8_t PIN_SW_EN       = D9;  // Enable комутатора (LOW = активний для HMC849ALP4CE)
constexpr uint8_t PIN_ADC_DET     = A0;  // Вхід АЦП від детектора (PA0 / ADC1_IN1)

// --- Часові параметри модуляції (Схема Дікке) ---
constexpr uint32_t DICKE_FREQ_HZ        = 200;     // Частота комутації f_sw = 200 Гц (період 5 мс)
constexpr uint32_t HALF_PERIOD_US       = 2500;    // Напівперіод = 2.5 мс (2500 мкс)
constexpr uint32_t BLANKING_TIME_US     = 150;     // Захисна пауза після перемикання (150 мкс)

// --- Параметри апаратного оцифрування (TIM6 -> ADC1 -> DMA1_Channel1) ---
// 256 вибірок на 125 кГц тривають 2048 мкс; 150 + 2048 = 2198 мкс < 2500 мкс (запас 302 мкс)
constexpr uint32_t ADC_SAMPLE_RATE_HZ   = 125000;  // Точний дільник 72 МГц (72 000 000 / 125 000 = 576)
constexpr size_t   ADC_SAMPLES_PER_HALF = 256;     // Кількість вибірок DMA за одну напівфазу
constexpr float    TRIM_PERCENTAGE      = 0.125f;  // Відсікання 12.5% знизу і зверху (по 32 із 256 -> 192 валідні)
constexpr float    ADC_VREF_MV          = 3300.0f; // Опорна напруга АЦП у мВ (3.3 В)
constexpr float    ADC_MAX_COUNTS       = 4095.0f; // 12-бітний АЦП

// --- Децимація та компенсація асиметрії k(t) ---
constexpr uint16_t DECIMATION_FACTOR    = 20;      // Усереднення пачками по 20 циклів (20 * 5 мс = 100 мс)
constexpr size_t   INTERP_BUFFER_SIZE   = 300;     // Буфер на T_cal = 30 с при кроці 100 мс (300 елементів)
constexpr uint32_t I2C_POLL_INTERVAL_MS = 1000;    // Період опитування термодатчика еталона (1 с)

// --- Параметри алгоритму девіації Аллана ---
constexpr size_t   ALLAN_WINDOW_SIZE    = 256;     // Оперативне вікно відліків (25.6 с; m до 32 -> tau до 3.2 с)
constexpr size_t   ALLAN_MEDIAN_LEN     = 5;       // Довжина медіанного фільтра стабілізації tau_opt (2.3.5)
constexpr float    DEFAULT_K_CAL        = 2.33f;   // Крутизна k_cal у лінеаризованих одиницях/К (при T_sys ≈ 430 К)
constexpr float    DEFAULT_DT_DOP       = 0.20f;   // Допустима похибка готовності системи (0.2 °C)

// --- Параметри логарифмічного детектора ADL5902 для програмної лінеаризації ---
constexpr float    DETECTOR_SLOPE_MV_DB = 53.0f;   // Типова крутизна ADL5902 (~53 мВ/дБ)
constexpr float    DETECTOR_V_REF_MV    = 1500.0f; // Опорна точка нормування (~ -30 дБм) для точності float

// --- Інтерфейси ---
constexpr uint32_t UART_BAUD_RATE       = 460800;  // Висока швидкість VCP ST-LINK (~2 мс на кадр CSV)
constexpr uint8_t  SHT3X_I2C_ADDR       = 0x44;    // Адреса I2C датчика температури еталона (SHT3x)
constexpr size_t   CAL_ASYM_SAMPLES     = 10;      // 10 децимованих відліків = 1.0 с усереднення калібрування k