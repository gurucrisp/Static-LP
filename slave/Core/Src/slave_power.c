/**
  ******************************************************************************
  * @file    slave_power.c
  * @brief   Power-stage hardware layer (see slave_power.h for the topology).
  *
  * The timer and break configuration below is the one verified on the board
  * during bring-up (BKIN active-low grounded -> gate inputs low, both timers
  * independently). It was moved here unchanged so that bring-up and control
  * firmware share ONE copy.
  ******************************************************************************
  */
#include "slave_power.h"
#include "slave_irq_prio.h"
#include "slave_config.h"
#include "slave_features.h"          /* SLAVE_FLT_KEEP_LOWS (v0.9.65) */
#ifndef SLAVE_FLT_KEEP_LOWS
#error "SLAVE_FLT_KEEP_LOWS undefined: slave_features.h was not included"
#endif

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim15;

extern void Error_Handler(void);

/* FLT input (PD2): level that means "fault". The comparator outputs idle high,
 * so a fault pulls low. If the real board idles LOW this reads as an immediate
 * fault and the controller refuses to start - a safe way to fail. */
#ifndef POWER_FLT_ASSERTED_LEVEL
#define POWER_FLT_ASSERTED_LEVEL  0U
#endif

/* ------------------------------------------------------------------------- */
/* INVERTER PIN POLICY (control firmware)                                    */
/*                                                                           */
/* The four inverter pins (PB14 1H, PB13 1L, PA8 2H, PA7 2L) are PLAIN GPIO   */
/* OUTPUTS DRIVEN LOW until the inverter is deliberately enabled, and go back */
/* to that state the instant anything switches it off. They are handed to    */
/* the timers only inside SlavePower_InverterEnable().                       */
/*                                                                           */
/* WHY: bench result - with the timers configured but NOT enabled, handing    */
/* the pins to the timers made the complementary (low-side) pins 1L and 2L   */
/* go HIGH. On the real board that turns the low-side IGBTs on at power-up,  */
/* before the relays and every check. The idle level of a disabled timer     */
/* output must therefore never decide the state of a gate-driver input.      */
/*                                                                           */
/* The BKIN pins (PB12, PA9) stay alternate-function inputs at all times.    */
/* ------------------------------------------------------------------------- */
static uint8_t s_control_mode;     /* 1 = SlavePower_Init() was used: pin policy above applies */

#define INV_A_LOW_MASK   ((uint32_t)(GPIO_PIN_7 | GPIO_PIN_8))      /* GPIOA: PA7 (2L), PA8 (2H)   */
#define INV_B_LOW_MASK   ((uint32_t)(GPIO_PIN_13 | GPIO_PIN_14))    /* GPIOB: PB13 (1L), PB14 (1H) */

/* ISR-SAFE. Pins low FIRST, then plain output mode. */
static inline void Power_OutputsToGpio(void)
{
    GPIOA->BSRR  = INV_A_LOW_MASK << 16U;
    GPIOB->BSRR  = INV_B_LOW_MASK << 16U;
    GPIOA->MODER = (GPIOA->MODER & ~((3UL << 14) | (3UL << 16))) | ((1UL << 14) | (1UL << 16));
    GPIOB->MODER = (GPIOB->MODER & ~((3UL << 26) | (3UL << 28))) | ((1UL << 26) | (1UL << 28));
}

/* Hands the four pins to the timers. Main loop, interrupts disabled by the caller. */
static inline void Power_OutputsToAf(void)
{
    GPIOA->MODER = (GPIOA->MODER & ~((3UL << 14) | (3UL << 16))) | ((2UL << 14) | (2UL << 16));
    GPIOB->MODER = (GPIOB->MODER & ~((3UL << 26) | (3UL << 28))) | ((2UL << 26) | (2UL << 28));
}

/* Writes the alternate-function numbers and the speed NOW, while the pins are
 * still plain low outputs, so that Power_OutputsToAf() only has to flip MODER. */
static void Power_PrepareOutputPins(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    Power_OutputsToGpio();

    GPIOA->OSPEEDR |= (3UL << 14) | (3UL << 16);
    GPIOB->OSPEEDR |= (3UL << 26) | (3UL << 28);

    /* PA7 = TIM1_CH1N AF2 (AFRL pin 7), PA8 = TIM1_CH1 AF2 (AFRH pin 8) */
    GPIOA->AFR[0] = (GPIOA->AFR[0] & ~(0xFUL << 28)) | (2UL << 28);
    GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFUL << 0))  | (2UL << 0);
    /* PB13 = TIM15_CH1N AF5, PB14 = TIM15_CH1 AF5 (AFRH pins 13, 14) - same AF numbers as the verified setup */
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~((0xFUL << 20) | (0xFUL << 24))) | (5UL << 20) | (5UL << 24);
}

/* ------------------------------------------------------------------------- */
/* Pin alternate functions for PWM + BKIN (identical to the verified setup)  */
/* ------------------------------------------------------------------------- */
static void Power_PinsAF(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (htim->Instance == TIM1)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        if (s_control_mode == 0U)      /* bring-up PWM test: pins go straight to the timer */
        {
            /* PA8 = TIM1_CH1 (AF2), PA7 = TIM1_CH1N (AF2) */
            GPIO_InitStruct.Pin       = GPIO_PIN_8;
            GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull      = GPIO_NOPULL;
            GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
            GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

            GPIO_InitStruct.Pin = GPIO_PIN_7;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        }

        /* PB12 = TIM1_BKIN (AF2) */
        GPIO_InitStruct.Pin       = GPIO_PIN_12;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else if (htim->Instance == TIM15)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        if (s_control_mode == 0U)      /* bring-up PWM test: pins go straight to the timer */
        {
            /* PB14 = TIM15_CH1 (AF5), PB13 = TIM15_CH1N (AF5) */
            GPIO_InitStruct.Pin       = GPIO_PIN_14;
            GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull      = GPIO_NOPULL;
            GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
            GPIO_InitStruct.Alternate = GPIO_AF5_TIM15;
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

            GPIO_InitStruct.Pin = GPIO_PIN_13;
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        }

        /* PA9 = TIM15_BKIN (AF5) */
        GPIO_InitStruct.Pin       = GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Alternate = GPIO_AF5_TIM15;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/* ------------------------------------------------------------------------- */
/* TIM1 (leg 2) and TIM15 (leg 1)                                            */
/* ------------------------------------------------------------------------- */
static void Power_Tim1Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = POWER_PERIOD_TICKS - 1U;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) Error_Handler();

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    /* Hardware break (PB12), ACTIVE LOW - verified on the board. BreakState
     * ENABLE: a break forces MOE off in silicon, no CPU involved. */
    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = POWER_DEADTIME_DTG;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_ENABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.BreakFilter      = POWER_BREAK_FILTER;
    sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_LOW;
    sBreakDeadTimeConfig.Break2Filter     = 0;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

    Power_PinsAF(&htim1);
}

static void Power_Tim15Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    __HAL_RCC_TIM15_CLK_ENABLE();

    htim15.Instance = TIM15;
    htim15.Init.Prescaler         = 0;
    htim15.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim15.Init.Period            = POWER_PERIOD_TICKS - 1U;
    htim15.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim15.Init.RepetitionCounter = 0;
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim15) != HAL_OK) Error_Handler();

    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = POWER_DEADTIME_DTG;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_ENABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.BreakFilter      = POWER_BREAK_FILTER;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

    Power_PinsAF(&htim15);
}

void SlavePower_TimersInit(void)
{
    Power_Tim1Init();
    Power_Tim15Init();
}

void SlavePower_Init(void)
{
    /* Converter pins: low first, then outputs (MX_GPIO_Init already did this;
     * repeated so this module is safe on its own). */
    SlavePower_ConverterOffNow();

    s_control_mode = 1U;                 /* the four inverter pins stay plain LOW outputs for now */
    Power_Tim1Init();
    Power_Tim15Init();
    Power_PrepareOutputPins();           /* AF numbers written; MODER stays "output, low" */

    /* Both inverter timers: zero duty, outputs disabled (MOE off, idle level
     * low through OSSI) until SlavePower_InverterEnable(). */
    TIM1->CCR1  = 0U;
    TIM15->CCR1 = 0U;
    TIM1->BDTR  &= ~TIM_BDTR_MOE;
    TIM15->BDTR &= ~TIM_BDTR_MOE;

    /* Break interrupts: the CPU must also kill the converter pins, which have
     * no hardware break. Highest priority. */
    TIM1->SR  = ~TIM_SR_BIF;
    TIM15->SR = ~TIM_SR_BIF;
    TIM1->DIER  |= TIM_DIER_BIE;
    TIM15->DIER |= TIM_DIER_BIE;
    HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, IRQ_PRIO_TIMING, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    HAL_NVIC_SetPriority(TIM15_IRQn, IRQ_PRIO_TIMING, 0);
    HAL_NVIC_EnableIRQ(TIM15_IRQn);

    SlavePower_FltInit();
}

void SlavePower_FltInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* FLT (PD2): software fault input -> interrupt on the asserting edge. */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitStruct.Pin  = GPIO_PIN_2;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
#if (POWER_FLT_ASSERTED_LEVEL == 0U)
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
#else
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
#endif
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI2_3_IRQn, IRQ_PRIO_FLT, 0);
    HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
}

/* ------------------------------------------------------------------------- */
/* Converter                                                                 */
/* ------------------------------------------------------------------------- */
void SlavePower_ConverterOffNow(void)
{
    GPIOB->BSRR = (uint32_t)(GPIO_PIN_3 | GPIO_PIN_5) << 16U;     /* 3H, 4L off */
    GPIOC->BSRR = (uint32_t)(GPIO_PIN_4 | GPIO_PIN_5) << 16U;     /* 3L, 4H off */
}

void SlavePower_ConverterPositive(void)
{
    GPIOC->BSRR = (uint32_t)(GPIO_PIN_4 | GPIO_PIN_5) << 16U;     /* negative diagonal OFF first */
    GPIOB->BSRR = (uint32_t)(GPIO_PIN_3 | GPIO_PIN_5);            /* 3H + 4L ON */
}

void SlavePower_ConverterNegative(void)
{
    GPIOB->BSRR = (uint32_t)(GPIO_PIN_3 | GPIO_PIN_5) << 16U;     /* positive diagonal OFF first */
    GPIOC->BSRR = (uint32_t)(GPIO_PIN_4 | GPIO_PIN_5);            /* 3L + 4H ON */
}

/* ------------------------------------------------------------------------- */
/* Inverter                                                                  */
/* ------------------------------------------------------------------------- */
uint8_t SlavePower_InverterEnable(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t  ok;

    __disable_irq();                              /* a break / fault interrupt must not interleave with the pin hand-over */

    TIM1->CCR1  = 0U;
    TIM15->CCR1 = 0U;
    TIM1->CCMR1  = (TIM1->CCMR1  & ~TIM_CCMR1_OC1M) | (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);     /* PWM mode 1 again (v0.9.65: SlavePower_FltOffNow forces it inactive) */
    TIM15->CCMR1 = (TIM15->CCMR1 & ~TIM_CCMR1_OC1M) | (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);
    TIM1->EGR   = TIM_EGR_UG;                     /* load shadow registers, counter -> 0 */
    TIM15->EGR  = TIM_EGR_UG;
    TIM1->SR    = ~(TIM_SR_BIF | TIM_SR_UIF);
    TIM15->SR   = ~(TIM_SR_BIF | TIM_SR_UIF);

    TIM1->CCER  |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    TIM15->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    TIM1->CR1   |= TIM_CR1_CEN;
    TIM15->CR1  |= TIM_CR1_CEN;
    TIM1->BDTR  |= TIM_BDTR_MOE;                  /* stays 0 while a BKIN is asserted */
    TIM15->BDTR |= TIM_BDTR_MOE;

    ok = SlavePower_InverterIsEnabled();
    if (ok != 0U)
    {
        Power_OutputsToAf();                      /* only NOW do the gate-driver inputs follow the timers:
                                                     zero duty = high sides low, LOW sides on */
    }
    else
    {
        TIM1->BDTR  &= ~TIM_BDTR_MOE;             /* a break input is still asserted: the pins stay low */
        TIM15->BDTR &= ~TIM_BDTR_MOE;
    }

    if (primask == 0U) __enable_irq();
    return ok;
}

void SlavePower_InverterDisable(void)
{
    TIM1->CCR1  = 0U;
    TIM15->CCR1 = 0U;
    TIM1->BDTR  &= ~TIM_BDTR_MOE;
    TIM15->BDTR &= ~TIM_BDTR_MOE;
    if (s_control_mode != 0U) Power_OutputsToGpio();   /* all four gate-driver inputs: plain LOW outputs */
}

void SlavePower_SetLegDuty(uint16_t leg1_tim15, uint16_t leg2_tim1)
{
    const uint16_t arr = (uint16_t)(POWER_PERIOD_TICKS - 1U);
    if (leg1_tim15 > arr) leg1_tim15 = arr;
    if (leg2_tim1  > arr) leg2_tim1  = arr;
    TIM15->CCR1 = leg1_tim15;
    TIM1->CCR1  = leg2_tim1;
}

void SlavePower_SyncCounters(void)
{
    TIM1->CNT  = 0U;
    TIM15->CNT = 0U;
}

uint8_t SlavePower_InverterIsEnabled(void)
{
    return (uint8_t)(((TIM1->BDTR & TIM_BDTR_MOE) != 0U) && ((TIM15->BDTR & TIM_BDTR_MOE) != 0U));
}

void SlavePower_AllOffNow(void)
{
    SlavePower_ConverterOffNow();
    SlavePower_InverterDisable();
}

void SlavePower_FltOffNow(void)
{
#if (SLAVE_FLT_KEEP_LOWS != 0)
    TIM1->CCMR1  = (TIM1->CCMR1  & ~TIM_CCMR1_OC1M) | TIM_CCMR1_OC1M_2;     /* OC1M = 0100, forced inactive: OC1 (high side) low NOW, OC1N (low side) high after the dead time */
    TIM15->CCMR1 = (TIM15->CCMR1 & ~TIM_CCMR1_OC1M) | TIM_CCMR1_OC1M_2;
    SlavePower_ConverterOffNow();
#else
    SlavePower_AllOffNow();
#endif
}

uint32_t SlavePower_TakeBreakFlags(void)
{
    uint32_t f = 0U;
    if ((TIM1->SR  & TIM_SR_BIF) != 0U)
    {
        TIM1->DIER &= ~TIM_DIER_BIE;             /* mask first: see the note in the header */
        TIM1->SR    = ~TIM_SR_BIF;
        f |= POWER_BREAK_TIM1;
    }
    if ((TIM15->SR & TIM_SR_BIF) != 0U)
    {
        TIM15->DIER &= ~TIM_DIER_BIE;
        TIM15->SR    = ~TIM_SR_BIF;
        f |= POWER_BREAK_TIM15;
    }
    return f;
}

uint8_t SlavePower_BreakInputActive(void)
{
    /* The pins are in alternate-function mode, but the input data register
     * still shows the pad level. Both break inputs are active LOW. */
    return (uint8_t)(((GPIOB->IDR & GPIO_PIN_12) == 0U) || ((GPIOA->IDR & GPIO_PIN_9) == 0U));
}

void SlavePower_BreakIrqRearm(void)
{
    if (SlavePower_BreakInputActive() != 0U) return;
    if ((TIM1->DIER & TIM_DIER_BIE) == 0U)  { TIM1->SR  = ~TIM_SR_BIF; TIM1->DIER  |= TIM_DIER_BIE; }
    if ((TIM15->DIER & TIM_DIER_BIE) == 0U) { TIM15->SR = ~TIM_SR_BIF; TIM15->DIER |= TIM_DIER_BIE; }
}

void SlavePower_RelaySh(uint8_t on)
{
    GPIOC->BSRR = (on != 0U) ? (uint32_t)GPIO_PIN_9 : ((uint32_t)GPIO_PIN_9 << 16U);
}

void SlavePower_RelayW(uint8_t on)
{
    GPIOC->BSRR = (on != 0U) ? (uint32_t)GPIO_PIN_8 : ((uint32_t)GPIO_PIN_8 << 16U);
}

uint8_t SlavePower_FltAsserted(void)
{
    GPIO_PinState s = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);
    return (uint8_t)(((uint32_t)s == POWER_FLT_ASSERTED_LEVEL) ? 1U : 0U);
}
