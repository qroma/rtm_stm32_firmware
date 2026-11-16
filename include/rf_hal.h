#pragma once
#include "config.h"
#include "types.h"

class RFHardware {
public:
    void init();
    void setSwitchState(RFSource source);
    
    // Виконує захисну паузу та оцифровує масив вибірок через АЦП (DMA / швидкий буфер)
    void acquireHalfPeriodSamples(uint16_t* buffer, size_t count);

    // Неблокуючий автомат опитування датчика SHT3x по I2C
    void updateReferenceTemperatureAsync(uint32_t now_ms);
    float getReferenceTemperature() const { return current_t_et_celsius_; }

private:
    float    current_t_et_celsius_ = 25.0f;
    uint32_t last_i2c_Millis_      = 0;
    bool     i2c_conversion_requested_ = false;
};