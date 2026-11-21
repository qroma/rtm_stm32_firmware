#include "rf_hal.h"
#include <Wire.h>
#include <cstring>

namespace {
RFHardware* s_rf_instance = nullptr;

uint32_t getApb1TimerClockHz() {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
        return pclk1 * 2U;
    }
    return pclk1;
}

void onTim2HalfPeriod() {
    if (s_rf_instance != nullptr) {
        s_rf_instance->handleHalfPeriodISR();
    }
}

void onTim2BlankingEnd() {
    if (s_rf_instance != nullptr) {
        s_rf_instance->handleBlankingEndISR();
    }
}
} // namespace

// Апаратне переривання завершення DMA1_Channel1 (ADC1)
extern "C" void DMA1_Channel1_IRQHandler(void) {
    if (s_rf_instance != nullptr) {
        HAL_DMA_IRQHandler(s_rf_instance->getDmaHandle());
    }
}

// Callback завершення пакету вибірок АЦП через DMA
extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (s_rf_instance != nullptr && hadc == s_rf_instance->getAdcHandle()) {
        s_rf_instance->handleDmaCompleteISR();
    }
}

void RFHardware::init() {
    s_rf_instance = this;

    // 1. Ініціалізація GPIO керування комутатором HMC849
    pinMode(PIN_SW_V1, OUTPUT);
    pinMode(PIN_SW_EN, OUTPUT);
    // HMC849ALP4CE вивід EN активний за низьким рівнем (LOW = увімкнено, HIGH = ізоляція).
    digitalWrite(PIN_SW_EN, LOW);
    setSwitchStateFast(RFSource::Antenna);

    // 2. Ініціалізація I2C (Fast Mode 400 кГц для мінімального часу транзакції < 200 мкс)
    Wire.begin();
    Wire.setClock(400000);
    Wire.setTimeout(5);

    // 3. Ініціалізація апаратного конвеєра: TIM6 (TRGO) -> ADC1 -> DMA1_Channel1 та TIM2 (200 Гц)
    initAdcAndDma();
    initSampleTriggerTim6();
    initDickeTimerTim2();
}

void RFHardware::initAdcAndDma() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_ADC12_CLK_ENABLE();

    // PA0 (Arduino A0) -> аналоговий вхід ADC1_IN1
    GPIO_InitTypeDef gpio{};
    gpio.Pin  = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    // DMA1_Channel1: ADC1->DR -> RAM (16-бітні слова, режим Normal для фіксованої пачки 256 вибірок)
    hdma_adc1_.Instance                 = DMA1_Channel1;
    hdma_adc1_.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1_.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1_.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1_.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1_.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1_.Init.Mode                = DMA_NORMAL;
    hdma_adc1_.Init.Priority            = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_adc1_);

    __HAL_LINKDMA(&hadc1_, DMA_Handle, hdma_adc1_);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    // ADC1: синхронне тактування HCLK/2 (36 МГц при 72 МГц SYSCLK), зовнішній запуск від TIM6_TRGO
    hadc1_.Instance                   = ADC1;
    hadc1_.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1_.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1_.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1_.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1_.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1_.Init.LowPowerAutoWait      = DISABLE;
    hadc1_.Init.ContinuousConvMode    = DISABLE;
    hadc1_.Init.NbrOfConversion       = 1;
    hadc1_.Init.DiscontinuousConvMode = DISABLE;
    hadc1_.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T6_TRGO;
    hadc1_.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1_.Init.DMAContinuousRequests = DISABLE;
    hadc1_.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    HAL_ADC_Init(&hadc1_);

    // Заводське внутрішнє автокалібрування зміщення АЦП STM32F303
    HAL_ADCEx_Calibration_Start(&hadc1_, ADC_SINGLE_ENDED);

    // Канал ADC1_IN1 (PA0); 61.5 + 12.5 = 74 такти ≈ 2.05 мкс (< 8 мкс при 125 кГц)
    ADC_ChannelConfTypeDef ch{};
    ch.Channel      = ADC_CHANNEL_1;
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SingleDiff   = ADC_SINGLE_ENDED;
    ch.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset       = 0;
    HAL_ADC_ConfigChannel(&hadc1_, &ch);
}

void RFHardware::initSampleTriggerTim6() {
    __HAL_RCC_TIM6_CLK_ENABLE();

    uint32_t tim_clk = getApb1TimerClockHz();
    uint32_t period_ticks = tim_clk / ADC_SAMPLE_RATE_HZ; // 72 000 000 / 125 000 = 576

    htim6_.Instance               = TIM6;
    htim6_.Init.Prescaler         = 0;
    htim6_.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6_.Init.Period            = (period_ticks > 0U) ? (period_ticks - 1U) : 575U;
    htim6_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim6_);

    // Генеруємо апаратний імпульс TRGO на кожне оновлення TIM6 (без CPU-переривань)
    TIM_MasterConfigTypeDef master_cfg{};
    master_cfg.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master_cfg.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim6_, &master_cfg);
}

void RFHardware::initDickeTimerTim2() {
    // Створюємо HardwareTimer після ініціалізації тактування ядра (уникнення конфлікту з TIM2_IRQHandler)
    static HardwareTimer s_tim2(TIM2);
    dicke_timer_ = &s_tim2;

    uint32_t tim_clk = getApb1TimerClockHz();
    dicke_timer_->pause();
    dicke_timer_->setPrescaleFactor(tim_clk / 1000000U);                  // 1 тік = 1 мкс
    dicke_timer_->setOverflow(HALF_PERIOD_US, TICK_FORMAT);               // 2500 мкс (400 Гц -> 200 Гц Дікке)
    dicke_timer_->setMode(1, TIMER_OUTPUT_DISABLED, NC);                  // Чистий compare без впливу на PA0
    dicke_timer_->setCaptureCompare(1, BLANKING_TIME_US, TICK_COMPARE_FORMAT); // 150 мкс захисна пауза
    dicke_timer_->refresh();

    dicke_timer_->attachInterrupt(onTim2HalfPeriod);
    dicke_timer_->attachInterrupt(1, onTim2BlankingEnd);
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
}

void RFHardware::setSwitchStateFast(RFSource source) {
    digitalWrite(PIN_SW_V1, (source == RFSource::Antenna) ? LOW : HIGH);
}

void RFHardware::startAcquisition() {
    if (running_ || dicke_timer_ == nullptr) return;

    HAL_TIM_Base_Stop(&htim6_);
    HAL_ADC_Stop_DMA(&hadc1_);

    noInterrupts();
    current_phase_ = RFSource::Antenna;
    active_bank_   = 0;
    ready_bank_    = 0;
    cycle_ready_   = false;
    running_       = true;
    interrupts();

    // Встановлюємо першу фазу (Антена) і запускаємо таймер з CNT = 0
    setSwitchStateFast(RFSource::Antenna);
    dicke_timer_->setCount(0);
    __HAL_TIM_CLEAR_FLAG(dicke_timer_->getHandle(), TIM_FLAG_UPDATE | TIM_FLAG_CC1);
    dicke_timer_->resume();
}

void RFHardware::stopAcquisition() {
    if (!running_ || dicke_timer_ == nullptr) return;

    running_ = false;
    dicke_timer_->pause();
    HAL_TIM_Base_Stop(&htim6_);
    HAL_ADC_Stop_DMA(&hadc1_);
}

void RFHardware::handleHalfPeriodISR() {
    if (!running_) return;

    // Гарантовано зупиняємо попередню пачку, якщо вона ще не завершилась
    HAL_TIM_Base_Stop(&htim6_);
    if ((HAL_ADC_GetState(&hadc1_) & HAL_ADC_STATE_REG_BUSY) != 0U) {
        HAL_ADC_Stop_DMA(&hadc1_);
    }

    // Перемикаємо фазу комутації на точній межі 2500 мкс
    current_phase_ = (current_phase_ == RFSource::Antenna) ? RFSource::Reference : RFSource::Antenna;
    setSwitchStateFast(current_phase_);
}

void RFHardware::handleBlankingEndISR() {
    if (!running_) return;

    // Рівно через 150 мкс після фронту перемикання запускаємо DMA-пачку на 256 вибірок
    uint16_t* target_buf = (current_phase_ == RFSource::Antenna)
                               ? ant_banks_[active_bank_]
                               : et_banks_[active_bank_];

    __HAL_TIM_SET_COUNTER(&htim6_, 0);
    HAL_ADC_Start_DMA(&hadc1_, reinterpret_cast<uint32_t*>(target_buf), ADC_SAMPLES_PER_HALF);
    HAL_TIM_Base_Start(&htim6_);
}

void RFHardware::handleDmaCompleteISR() {
    HAL_TIM_Base_Stop(&htim6_);
    HAL_ADC_Stop_DMA(&hadc1_);

    if (!running_) return;

    // Коли завершено другу напівфазу (Еталон), повна пара (V_ant, V_et) готова
    if (current_phase_ == RFSource::Reference) {
        if (cycle_ready_) {
            dropped_cycles_++;
        }
        ready_bank_  = active_bank_;
        active_bank_ ^= 1U;
        cycle_ready_ = true;
    }
}

bool RFHardware::fetchReadyCycle(uint16_t* ant_dst, uint16_t* et_dst) {
    uint8_t bank_to_read = 0;

    noInterrupts();
    if (!cycle_ready_) {
        interrupts();
        return false;
    }
    bank_to_read = ready_bank_;
    cycle_ready_ = false;
    interrupts();

    // Копіюємо неактивний банк (ISR не торкається його щонайменше 2.5 мс; memcpy 512 байт триває ~3 мкс)
    std::memcpy(ant_dst, ant_banks_[bank_to_read], ADC_SAMPLES_PER_HALF * sizeof(uint16_t));
    std::memcpy(et_dst,  et_banks_[bank_to_read],  ADC_SAMPLES_PER_HALF * sizeof(uint16_t));
    return true;
}

void RFHardware::updateReferenceTemperatureAsync(uint32_t now_ms) {
    // Неблокуюче розділення запиту та зчитування SHT3x (не впливає на апаратний цикл Дікке)
    if (!i2c_conversion_requested_ && (now_ms - last_i2c_millis_ >= I2C_POLL_INTERVAL_MS)) {
        Wire.beginTransmission(SHT3X_I2C_ADDR);
        Wire.write(0x24);
        Wire.write(0x00);
        if (Wire.endTransmission() == 0) {
            i2c_conversion_requested_ = true;
        }
        last_i2c_millis_ = now_ms;
    } else if (i2c_conversion_requested_ && (now_ms - last_i2c_millis_ >= 20)) {
        if (Wire.requestFrom(SHT3X_I2C_ADDR, static_cast<uint8_t>(3)) == 3) {
            uint16_t raw_temp = (static_cast<uint16_t>(Wire.read()) << 8) | Wire.read();
            Wire.read(); // CRC датчика
            current_t_et_celsius_ = -45.0f + 175.0f * (static_cast<float>(raw_temp) / 65535.0f);
        }
        i2c_conversion_requested_ = false;
        last_i2c_millis_ = now_ms;
    }
}