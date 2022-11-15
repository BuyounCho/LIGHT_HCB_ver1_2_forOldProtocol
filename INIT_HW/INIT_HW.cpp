#include "mbed.h"
#include "FastPWM.h"
#include "setting.h"

extern ADC_HandleTypeDef        hadc1;
extern ADC_HandleTypeDef        hadc2;
extern ADC_HandleTypeDef        hadc3;

void Init_ADC1(void)
{
    /*
        // ADC Setup
    //     RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;                        // clock for ADC3
    //     RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;                        // clock for ADC2
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;                        // clock for ADC1
         RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;                        // Enable clock for GPIOC
    //    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;                        // Enable clock for GPIOB
        ADC1->CR2 |= ADC_CR2_ADON;//0x00000001;                    // ADC1 ON
    //    ADC->CCR = 0x00000016;                                     // Regular simultaneous mode only
        ADC1->CR1 |= 0x800;                                         // Discontinuous mode
        ADC1->CR1 |= 0x100;                                         // Scan mode
        ADC1->CR2 |= 0x400;                                         // EOC
        ADC1->SQR1 |= 0x100000;                                     // 2 conversions
    //    ADC1->SQR3 = 0x00000008;                                   // use PB_0 as input - ADC1_IN8
         ADC1->SQR3 |= 0x0000000E;                    //channel      // use PC_4 as input - ADC1_IN14
         ADC1->SQR3 |= 0x000001E0;                                   // use PC_5 as input - ADC1_IN15  0b0000 0000 0000 0000 0000 0001 1110 0000
    //     ADC2->CR2 |= ADC_CR2_ADON;//0x00000001;                    // ADC2 ON
    //     ADC2->SQR3 = 0x00000008;                                   // use PB_0 as input - ADC2_IN8
    //     ADC3->CR2 |= ADC_CR2_ADON;                                 // ADC3 ON00000000000
    //     ADC3->SQR3 = 0x0000000B;                                   // use PC_1, - ADC3_IN11
         GPIOC->MODER |= 0b111100000000;             //each channel   // PC_4, PC_5 are analog inputs
    //    GPIOB->MODER |= 0x3;                                       // PB_0 as analog input
    //    ADC1->SMPR2 |= 0x01000000;                                     // 15 cycles on CH_8,  0b0000000100000000<<16
    //     ADC1->SMPR1 |= 0x00001000;                                     // 15 cycles on CH_14
    //     ADC1->SMPR1 |= 0x00008000;                                     // 15 cycles on CH_15
    //     ADC2->SMPR2 |= 0x01000000;                                     // 15 cycles on CH_8,  0b0000000100000000<<16
    //     ADC3->SMPR1 |= 0x00000008;                                     // 15 cycles on CH_11, 0b0000000000001000

    */

    //////////////////////////////////////////////////ADC1////////////////////////////////////////////////
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = ENABLE;
    hadc1.Init.NbrOfDiscConversion = 1;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
//    Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_14;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
//    Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.Rank = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
//    Error_Handler();
    }
    hadc1.Instance->CR2 |=  ADC_CR2_ADON;
}

void Init_ADC2(void)
{

    //////////////////////////////////////////////////ADC2////////////////////////////////////////////////
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc2.Instance = ADC2;
    hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode = ENABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = ENABLE;
    hadc2.Init.NbrOfDiscConversion = 1;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 2;
    hadc2.Init.DMAContinuousRequests = DISABLE;
    hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
//    Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
//    Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = 2;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
//    Error_Handler();
    }
    hadc2.Instance->CR2 |=  ADC_CR2_ADON;
}

void Init_ADC3(void)
{
    //////////////////////////////////////////////////ADC3////////////////////////////////////////////////
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc3.Instance = ADC3;
    hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc3.Init.Resolution = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode = DISABLE;
    hadc3.Init.ContinuousConvMode = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion = 1;
    hadc3.Init.DMAContinuousRequests = DISABLE;
    hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) {
//    Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
//    Error_Handler();
    }
    hadc3.Instance->CR2 |=  ADC_CR2_ADON;
}


void Init_TMR3()
{
    ///////////////Trigger by TIMER 1/////////////////////////////////
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                         // enable TIM3 clock
    NVIC_EnableIRQ(TIM3_IRQn);                                  //Enable TIM3 IRQ

    TIM3->DIER |= TIM_DIER_UIE;                                 // enable update interrupt
    TIM3->CR1 = 0x10;
    TIM3->CR1 |= TIM_CR1_UDIS;
    TIM3->EGR |= TIM_EGR_UG;

    TIM3->PSC = 1-1;                                           // 10kHz
    TIM3->ARR = 1;
    TIM3->CNT = 0;
    TIM3->SR = 0;
    TIM3->SMCR = 0x07;                                         //External clock mode
    TIM3->CR1 |= TIM_CR1_CEN;                                   // enable TIM3
    
//    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                         // enable TIM3 clock
//    NVIC_EnableIRQ(TIM3_IRQn);                                  //Enable TIM3 IRQ
//
//    TIM3->DIER |= TIM_DIER_UIE;                                 // enable update interrupt
//    TIM3->CR1 = 0x40;                                           // CMS = 10, interrupt only when counting up // Center-aligned mode
//    TIM3->CR1 |= TIM_CR1_UDIS;
//    TIM3->RCR |= 0x001;                                         // update event once per up/down count of TIM3
//    TIM3->EGR |= TIM_EGR_UG;
//
//    TIM3->PSC = 0x00;                                            // no prescaler, timer counts up in sync with the peripheral clock
//    TIM3->ARR = TMR3_COUNT-1;                                          // set auto reload, 5 khz
//    TIM3->CNT = 0;
//    TIM3->SR = 0;
//    TIM3->CCER |= ~(TIM_CCER_CC1NP);                            // Interupt when low side is on.
//    TIM3->CR1 |= TIM_CR1_CEN;                                   // enable TIM4
}

void Init_TMR2()
{
    ///////////////Trigger by TIMER 1/////////////////////////////////
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;                         // enable TIM2 clock
    NVIC_EnableIRQ(TIM2_IRQn);                                  //Enable TIM2 IRQ

    TIM2->DIER |= TIM_DIER_UIE;                                 // enable update interrupt
    TIM2->CR1 = 0x10;
    TIM2->CR1 |= TIM_CR1_UDIS;
    TIM2->EGR |= TIM_EGR_UG;

    TIM2->PSC = 2-1;                                           // 20 prescaler, 5kHz
    TIM2->ARR = 1;
    TIM2->CNT = 0;
    TIM2->SR = 0;
    TIM2->SMCR = 0x07;                                         //External clock mode
    TIM2->CR1 |= TIM_CR1_CEN;                                   // enable TIM2


    ////////////////Trigger by Internal Clock/////////////////////////////////
//    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;                         // enable TIM2 clock
//    //ISR Setup
//    NVIC_EnableIRQ(TIM2_IRQn);                                  //Enable TIM2 IRQ
//    TIM2->DIER |= TIM_DIER_UIE;                                 // enable update interrupt
//    TIM2->CR1 = 0x40;                                           // CMS = 10, interrupt only when counting up // Center-aligned mode
//    TIM2->CR1 |= TIM_CR1_UDIS;
//    TIM2->RCR |= 0x001;                                         // update event once per up/down count of TIM2
//    TIM2->EGR |= TIM_EGR_UG;
//    TIM2->PSC = 0x00;                                            // no prescaler, timer counts up in sync with the peripheral clock
//    TIM2->ARR = TMR2_COUNT-1;                                          // set auto reload, 5 khz
//    TIM2->CNT = 0;
//    TIM2->SR = 0;
//    TIM2->CR1 |= TIM_CR1_CEN;                                   // enable TIM2
}


void Init_TMR1()
{
    ///////////////////PWM output by L6205D/////////////////////////////////
    
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;                         // enable TIM1 clock
    FastPWM pwm_v(PIN_H1);
    FastPWM pwm_w(PIN_L1);

    TIM1->CR2 |= 0x20;                                          //MMS = 010
    TIM1->DIER |= TIM_DIER_UIE;                                 // enable update interrupt
    TIM1->CR1 = 0x40;                                           // CMS = 10, interrupt only when counting up // Center-aligned mode
    TIM1->CR1 |= TIM_CR1_UDIS;
    TIM1->CR1 |= TIM_CR1_ARPE;                                  // autoreload on,
    TIM1->RCR |= 0x001;                                         // update event once per up/down count of TIM1
    TIM1->EGR |= TIM_EGR_UG;
    TIM1->CCMR1 |= 0x60;                                        //CH1 - PWM mode 1
    TIM1->PSC = 0x0;                                           // no prescaler, timer counts up in sync with the peripheral clock
    TIM1->ARR = TMR1_COUNT-1;                                   // set auto reload, 20 khz
    TIM1->CNT = 0;
    TIM1->SR = 0;
    TIM1->CR1 |= TIM_CR1_CEN;                                   // enable TIM1


    ///////////////////PWM output by IRSM045MA/////////////////////////////////
//    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;                         // enable TIM1 clock
//
//    FastPWM pwm_H1(PIN_H1);
//    FastPWM pwm_L1(PIN_L1);
//    FastPWM pwm_H2(PIN_H2);
//    FastPWM pwm_L2(PIN_L2);
//
//
//    TIM1->CR2 |= 0x20;                                          //MMS = 010
//
//
//    TIM1->DIER |= TIM_DIER_UIE;                                 // enable update interrupt
//    TIM1->CR1 = 0x40;                                           // CMS = 10, interrupt only when counting up // Center-aligned mode
////    TIM1->CR1 = 0x10;                                           // CMS = 10, interrupt only when counting up // Center-aligned mode
//    TIM1->CR1 |= TIM_CR1_UDIS;
//    TIM1->CR1 |= TIM_CR1_ARPE;                                  // autoreload on,
//    TIM1->RCR |= 0x001;                                         // update event once per up/down count of TIM1
//    TIM1->EGR |= TIM_EGR_UG;
//
//    TIM1->CCMR1 |= 0x60;                                        //CH1 - PWM mode 1
//    TIM1->CCMR1 |= 0x6000;                                      //CH2 - PWM mode 1
//
//    TIM1->PSC = 0x0;                                           // no prescaler, timer counts up in sync with the peripheral clock
//    TIM1->ARR = TMR1_COUNT-1;                                   // set auto reload, 20 khz
//    TIM1->CNT = 0;
//    TIM1->SR = 0;
//    TIM1->CCER |= 0x05;                                         // CC1E = 1, CC1P = 0, CC1NE = 1, CC1NP = 0
//    TIM1->CCER |= 0x50;                                         // CC2E = 1, CC2P = 0, CC2NE = 1, CC2NP = 0
//
//    TIM1->BDTR |= 0x8000;                                       // MOE = 1;
////    TIM1->BDTR |= 0x7F;                                         // Dead-time
//    TIM1->BDTR |= 0x05;
//    TIM1->CR1 |= TIM_CR1_CEN;                                   // enable TIM1
}

void Init_TIM4()
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;                         // enable TIM4 clock

    NVIC_EnableIRQ(TIM4_IRQn);                                  //Enable TIM4 IRQ

    TIM4->DIER |= TIM_DIER_UIE;                                 // enable update interrupt
    TIM4->CR1 = 0x10;
    TIM4->CR1 |= TIM_CR1_UDIS;
    TIM4->EGR |= TIM_EGR_UG;

//    TIM4->PSC = 10-1;                                           // 10 prescaler, 1kHz
    TIM4->PSC = 20-1;                                           // 20 prescaler, 500Hz
    TIM4->ARR = 1;
    TIM4->CNT = 0;
    TIM4->SR = 0;
    TIM4->SMCR = 0x07;                                         //External clock mode
    TIM4->CR1 |= TIM_CR1_CEN;                                   // enable TIM4
}