#pragma once
#include <Arduino.h>
#include "config.h"
#include "types.h"

class RFHardware {
public:
    void init();

    // Запуск та зупинка апаратного циклу комутації TIM2 + TIM6 + ADC1/DMA
    void startAcquisition();
    void stopAcquisition();
    bool isRunning() const { return running_; }

    // Неблокуюче отримання готової пари буферів (антена + еталон) із ping-pong банку
    bool fetchReadyCycle(uint16_t* ant_dst, uint16_t* et_dst);

    // Діагностика пропусків (якщо loop() не встиг забрати буфер за 5 мс)
    uint32_t getDroppedCycles() const { return dropped_cycles_; }

    // Неблокуюче опитування датчика температури еталона по I2C
    void  updateReferenceTemperatureAsync(uint32_t now_ms);
    float getReferenceTemperature() const { return current_t_et_celsius_; }

    // Обробники переривань (викликаються з ISR-обгорток)
    void handleHalfPeriodISR();
    void handleBlankingEndISR();
    void handleDmaCompleteISR();

    ADC_HandleTypeDef* getAdcHandle() { return &hadc1_; }
    DMA_HandleTypeDef* getDmaHandle() { return &hdma_adc1_; }

private:
    void initAdcAndDma();
    void initSampleTriggerTim6();
    void initDickeTimerTim2();
    void setSwitchStateFast(RFSource source);

    ADC_HandleTypeDef hadc1_{};
    DMA_HandleTypeDef hdma_adc1_{};
    TIM_HandleTypeDef htim6_{};
    HardwareTimer*    dicke_timer_ = nullptr;

    // Подвійний (ping-pong) буфер: [2 банки][ADC_SAMPLES_PER_HALF]
    uint16_t ant_banks_[2][ADC_SAMPLES_PER_HALF]{};
    uint16_t et_banks_[2][ADC_SAMPLES_PER_HALF]{};

    volatile RFSource current_phase_  = RFSource::Antenna;
    volatile uint8_t  active_bank_    = 0;
    volatile uint8_t  ready_bank_     = 0;
    volatile bool     cycle_ready_    = false;
    volatile bool     running_        = false;
    volatile uint32_t dropped_cycles_ = 0;

    // Стан асинхронного опитування I2C SHT3x
    bool     i2c_conversion_requested_ = false;
    uint32_t last_i2c_millis_          = 0;
    float    current_t_et_celsius_     = 25.0f;
};