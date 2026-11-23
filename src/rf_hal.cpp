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

// Прямий неблокуючий обробник DMA1_Channel1 без викликів HAL_ADC_Stop_DMA() в ISR
extern "C" void DMA1_Channel1_IRQHandler(void) {
    uint32_t isr = DMA1->ISR;

    if ((isr & (DMA_ISR_TCIF1 | DMA_ISR_TEIF1)) != 0U) {
        // Негайно зупиняємо TIM6 і DMA на рівні регістрів (2 такти CPU, нульовий ризик deadlock)
        TIM6->CR1 &= ~TIM_CR1_CEN;
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        DMA1->IFCR = DMA_IFCR_CGIF1;

        if (((isr & DMA_ISR_TCIF1) != 0U) && (s_rf_instance != nullptr)) {
            s_rf_instance->handleDmaCompleteISR();
        }
    } else {
        DMA1->IFCR = DMA_IFCR_CHTIF1;
    }
}

void RFHardware::init() {
    s_rf_instance = this;

    pinMode(PIN_SW_V1, OUTPUT);
    pinMode(PIN_SW_EN, OUTPUT);
    // HMC849ALP4CE: LOW = увімкнено, HIGH = ізоляція (змініть лише за наявності інвертора на платі)
    digitalWrite(PIN_SW_EN, LOW);
    setSwitchStateFast(RFSource::Antenna);

    Wire.begin();
    Wire.setClock(400000);
    Wire.setTimeout(5);

    initAdcAndDma();
    initSampleTriggerTim6();
    initDickeTimerTim2();
}

void RFHardware::initAdcAndDma() {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_ADC12_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin  = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    // DMA_NORMAL: канал зупиняється апаратно рівно після ADC_SAMPLES_PER_HALF вибірок
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

    // DMAContinuousRequests = ENABLE на стороні ADC1 дозволяє перезапускати DMA1_Channel1
    // без зупинки/перезапуску самого АЦП; перетворення жорстко гейтуються через TIM6_TRGO
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
    hadc1_.Init.DMAContinuousRequests = ENABLE;
    hadc1_.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    HAL_ADC_Init(&hadc1_);

    HAL_ADCEx_Calibration_Start(&hadc1_, ADC_SINGLE_ENDED);

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

    uint32_t tim_clk      = getApb1TimerClockHz();
    uint32_t period_ticks = tim_clk / ADC_SAMPLE_RATE_HZ;

    htim6_.Instance               = TIM6;
    htim6_.Init.Prescaler         = 0;
    htim6_.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6_.Init.Period            = (period_ticks > 0U) ? (period_ticks - 1U) : 575U;
    htim6_.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim6_);

    TIM_MasterConfigTypeDef master_cfg{};
    master_cfg.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master_cfg.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim6_, &master_cfg);
}

void RFHardware::initDickeTimerTim2() {
    static HardwareTimer s_tim2(TIM2);
    dicke_timer_ = &s_tim2;

    uint32_t tim_clk = getApb1TimerClockHz();
    dicke_timer_->pause();
    dicke_timer_->setPrescaleFactor(tim_clk / 1000000U);
    dicke_timer_->setOverflow(HALF_PERIOD_US, TICK_FORMAT);
    dicke_timer_->setMode(1, TIMER_OUTPUT_DISABLED, NC);
    dicke_timer_->setCaptureCompare(1, BLANKING_TIME_US, TICK_COMPARE_FORMAT);
    dicke_timer_->refresh();

    dicke_timer_->attachInterrupt(onTim2HalfPeriod);
    dicke_timer_->attachInterrupt(1, onTim2BlankingEnd);
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
}

void RFHardware::setSwitchStateFast(RFSource source) {
    // Таблиця істинності HMC849 (при EN = LOW):
    // Vctl = HIGH -> RFC-RF1 ON (Антена), Vctl = LOW -> RFC-RF2 ON (Еталон)
    digitalWrite(PIN_SW_V1, (source == RFSource::Antenna) ? HIGH : LOW);
}

void RFHardware::startAcquisition() {
    if (running_ || dicke_timer_ == nullptr) return;

    TIM6->CR1 &= ~TIM_CR1_CEN;
    HAL_ADC_Stop_DMA(&hadc1_); // Безпечно: викликається з контексту loop(), не з ISR

    // Одноразово вмикаємо ADC1 + DMA1_Channel1 (TIM6 зупинено, тож перетворення ще не йдуть)
    HAL_ADC_Start_DMA(&hadc1_, reinterpret_cast<uint32_t*>(ant_banks_[0]), ADC_SAMPLES_PER_HALF);
    // Вимикаємо HTIE та тимчасово деактивуємо канал до завершення першої захисної паузи
    DMA1_Channel1->CCR &= ~(DMA_CCR_EN | DMA_CCR_HTIE);
    DMA1->IFCR = DMA_IFCR_CGIF1;

    noInterrupts();
    current_phase_ = RFSource::Antenna;
    active_bank_   = 0;
    ready_bank_    = 0;
    cycle_ready_   = false;
    running_       = true;
    interrupts();

    setSwitchStateFast(RFSource::Antenna);
    dicke_timer_->setCount(0);
    __HAL_TIM_CLEAR_FLAG(dicke_timer_->getHandle(), TIM_FLAG_UPDATE | TIM_FLAG_CC1);
    dicke_timer_->resume();
}

void RFHardware::stopAcquisition() {
    if (!running_ || dicke_timer_ == nullptr) return;

    running_ = false;
    dicke_timer_->pause();
    TIM6->CR1 &= ~TIM_CR1_CEN;
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    HAL_ADC_Stop_DMA(&hadc1_); // Безпечно: викликається з контексту loop()
}

void RFHardware::rearmDmaTransferISR(uint16_t* target_buf) {
    // Повністю неблокуюче переналаштування DMA1_Channel1 (< 0.5 мкс)
    TIM6->CR1 &= ~TIM_CR1_CEN;
    DMA1_Channel1->CCR &= ~(DMA_CCR_EN | DMA_CCR_HTIE);
    DMA1->IFCR = DMA_IFCR_CGIF1;
    ADC1->ISR  = ADC_ISR_EOC | ADC_ISR_EOS | ADC_ISR_OVR;

    DMA1_Channel1->CMAR  = reinterpret_cast<uint32_t>(target_buf);
    DMA1_Channel1->CNDTR = ADC_SAMPLES_PER_HALF;
    DMA1_Channel1->CCR  |= (DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_TEIE);

    TIM6->CNT  = 0;
    TIM6->CR1 |= TIM_CR1_CEN;
}

void RFHardware::handleHalfPeriodISR() {
    if (!running_) return;

    // Неблокуюча зупинка TIM6/DMA на межі 2500 мкс перед перемиканням комутатора
    TIM6->CR1 &= ~TIM_CR1_CEN;
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    current_phase_ = (current_phase_ == RFSource::Antenna) ? RFSource::Reference : RFSource::Antenna;
    setSwitchStateFast(current_phase_);
}

void RFHardware::handleBlankingEndISR() {
    if (!running_) return;

    uint16_t* target_buf = (current_phase_ == RFSource::Antenna)
                               ? ant_banks_[active_bank_]
                               : et_banks_[active_bank_];
    rearmDmaTransferISR(target_buf);
}

void RFHardware::handleDmaCompleteISR() {
    if (!running_) return;

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

    std::memcpy(ant_dst, ant_banks_[bank_to_read], ADC_SAMPLES_PER_HALF * sizeof(uint16_t));
    std::memcpy(et_dst,  et_banks_[bank_to_read],  ADC_SAMPLES_PER_HALF * sizeof(uint16_t));
    return true;
}

// Стандартний CRC-8 для Sensirion SHT3x (поліном 0x31, початкове значення 0xFF)
uint8_t RFHardware::computeSht3xCrc8(uint8_t msb, uint8_t lsb) {
    uint8_t crc = 0xFF;
    const uint8_t bytes[2] = {msb, lsb};
    for (uint8_t b : bytes) {
        crc ^= b;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1) ^ 0x31U)
                                : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

void RFHardware::updateReferenceTemperatureAsync(uint32_t now_ms) {
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
            uint8_t msb      = Wire.read();
            uint8_t lsb      = Wire.read();
            uint8_t crc_recv = Wire.read();

            // Оновлюємо T_et лише після проходження перевірки CRC-8
            if (computeSht3xCrc8(msb, lsb) == crc_recv) {
                uint16_t raw_temp = (static_cast<uint16_t>(msb) << 8) | lsb;
                current_t_et_celsius_ = -45.0f + 175.0f * (static_cast<float>(raw_temp) / 65535.0f);
            }
        }
        i2c_conversion_requested_ = false;
        last_i2c_millis_ = now_ms;
    }
}