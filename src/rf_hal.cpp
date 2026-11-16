#include "rf_hal.h"
#include <Wire.h>

void RFHardware::init() {
    pinMode(PIN_SW_V1, OUTPUT);
    pinMode(PIN_SW_EN, OUTPUT);
    digitalWrite(PIN_SW_EN, HIGH); // Вмикаємо комутатор
    
    analogReadResolution(12);      // Налаштовуємо 12-бітний режим АЦП STM32F303RE
    
    Wire.begin();
    Wire.setClock(400000);         // Fast Mode I2C (400 кГц), щоб транзакція тривала < 200 мкс
}

void RFHardware::setSwitchState(RFSource source) {
    // Керування лінією V1 комутатора HMC849
    digitalWrite(PIN_SW_V1, (source == RFSource::Antenna) ? LOW : HIGH);
}

void RFHardware::acquireHalfPeriodSamples(uint16_t* buffer, size_t count) {
    // 1. Витримуємо захисну паузу (blanking time = 150 мкс) після перемикання комутатора
    delayMicroseconds(BLANKING_TIME_US);

    // 2. Високошвидкісний збір вибірок у межах стабілізованого плато напівфази
    uint32_t sample_interval_us = (HALF_PERIOD_US - BLANKING_TIME_US - 250) / count;
    for (size_t i = 0; i < count; ++i) {
        uint32_t t0 = micros();
        buffer[i] = static_cast<uint16_t>(analogRead(PIN_ADC_DET));
        while ((micros() - t0) < sample_interval_us) {
            __NOP();
        }
    }
}

void RFHardware::updateReferenceTemperatureAsync(uint32_t now_ms) {
    // Розділення запиту вимірювання і зчитування SHT3x у часі, щоб не блокувати цикл Дайка
    if (!i2c_conversion_requested_ && (now_ms - last_i2c_Millis_ >= I2C_POLL_INTERVAL_MS)) {
        Wire.beginTransmission(SHT3X_I2C_ADDR);
        Wire.write(0x24); 
        Wire.write(0x00);
        Wire.endTransmission();
        i2c_conversion_requested_ = true;
        last_i2c_Millis_ = now_ms;
    } 
    else if (i2c_conversion_requested_ && (now_ms - last_i2c_Millis_ >= 20)) {
        // Через 20 мс дані в SHT3x готові — зчитуємо 3 байти температури (триває ~100 мкс)
        if (Wire.requestFrom(SHT3X_I2C_ADDR, (uint8_t)3) == 3) {
            uint16_t raw_temp = (Wire.read() << 8) | Wire.read();
            Wire.read(); // Пропускаємо CRC датчика
            current_t_et_celsius_ = -45.0f + 175.0f * (static_cast<float>(raw_temp) / 65535.0f);
        }
        i2c_conversion_requested_ = false;
        last_i2c_Millis_ = now_ms;
    }
}