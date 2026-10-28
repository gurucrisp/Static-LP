/**
  ******************************************************************************
  * @file    slave_zc.c
  * @brief   Zero-cross (ZCD, PB0) capture + characterisation - slave.
  *          See slave_zc.h for what this is for and how to read g_zc.
  ******************************************************************************
  */

#include "main.h"
#include "slave_zc.h"
#include "slave_irq_prio.h"
#include "slave_config.h"
#include "slave_features.h"
#if (SLAVE_POWER_STAGE >= 1)
#include "slave_ctrl.h"
#endif
#include "slave_flt_filter.h"
#if (SLAVE_FLT_FILTER_US > 0U) && (SLAVE_POWER_STAGE >= 1)
#include "slave_power.h"
#endif
#if (SLAVE_SEQ_SYNC != 0)
#include "slave_phase.h"           /* the phase sequence measurement (v0.9.51) */
#endif

TIM_HandleTypeDef htim2;
SlaveZC_t         g_zc;

/* One-second window bookkeeping for rise_per_s / fall_per_s. */
static uint32_t s_win_start_us;
static uint32_t s_win_rise;
static uint32_t s_win_fall;

void SlaveZC_ResetStats(void)
{
    g_zc.rise_count      = 0U;
    g_zc.fall_count      = 0U;
    g_zc.t_rise_us       = 0U;
    g_zc.t_fall_us       = 0U;
    g_zc.t_last_us       = 0U;
    g_zc.level           = 0U;
    g_zc.period_rise_us  = 0U;
    g_zc.period_fall_us  = 0U;
    g_zc.width_hi_us     = 0U;
    g_zc.width_lo_us     = 0U;
    g_zc.width_hi_min_us = 0xFFFFFFFFU;
    g_zc.width_hi_max_us = 0U;
    g_zc.width_lo_min_us = 0xFFFFFFFFU;
    g_zc.width_lo_max_us = 0U;
    g_zc.rise_per_s      = 0U;
    g_zc.fall_per_s      = 0U;
    s_win_rise           = 0U;
    s_win_fall           = 0U;
}

/* Hardware-independent core. All time differences use unsigned 32-bit
 * arithmetic, so they stay correct across the timer's wrap-around. */
void SlaveZC_OnEdge(uint32_t now_us, uint8_t rising)
{
    uint32_t w;

    /* Roll the one-second window over (lazily, on the first edge after it). */
    if ((uint32_t)(now_us - s_win_start_us) >= 1000000U)
    {
        g_zc.rise_per_s = s_win_rise;
        g_zc.fall_per_s = s_win_fall;
        s_win_rise      = 0U;
        s_win_fall      = 0U;
        s_win_start_us  = now_us;
    }

    if (rising)
    {
        if (g_zc.rise_count > 0U)
        {
            g_zc.period_rise_us = (uint32_t)(now_us - g_zc.t_rise_us);
        }
        if (g_zc.fall_count > 0U)
        {
            w = (uint32_t)(now_us - g_zc.t_fall_us);       /* time the line was LOW */
            g_zc.width_lo_us = w;
            if (w < g_zc.width_lo_min_us) g_zc.width_lo_min_us = w;
            if (w > g_zc.width_lo_max_us) g_zc.width_lo_max_us = w;
        }
        g_zc.t_rise_us = now_us;
        g_zc.rise_count++;
        s_win_rise++;
    }
    else
    {
        if (g_zc.fall_count > 0U)
        {
            g_zc.period_fall_us = (uint32_t)(now_us - g_zc.t_fall_us);
        }
        if (g_zc.rise_count > 0U)
        {
            w = (uint32_t)(now_us - g_zc.t_rise_us);       /* time the line was HIGH */
            g_zc.width_hi_us = w;
            if (w < g_zc.width_hi_min_us) g_zc.width_hi_min_us = w;
            if (w > g_zc.width_hi_max_us) g_zc.width_hi_max_us = w;
        }
        g_zc.t_fall_us = now_us;
        g_zc.fall_count++;
        s_win_fall++;
    }

    g_zc.t_last_us = now_us;
}

uint32_t SlaveZC_NowUs(void)
{
    return TIM2->CNT;
}

uint32_t SlaveZC_AgeUs(void)
{
    if ((g_zc.rise_count == 0U) && (g_zc.fall_count == 0U))
    {
        return 0xFFFFFFFFU;
    }
    return (uint32_t)(TIM2->CNT - g_zc.t_last_us);
}

uint32_t SlaveZC_RiseRateHzX100(void)
{
    uint32_t p = g_zc.period_rise_us;
    if (p == 0U)
    {
        return 0U;
    }
    return 100000000UL / p;      /* 1e8 / period_us  =  Hz x 100 */
}

void SlaveZC_Init(void)
{
    /* TIM2: 32-bit, free-running, 1 MHz (1 count = 1 us). */
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = (SLAVEZC_TIMER_CLK_HZ / 1000000UL) - 1UL;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0xFFFFFFFFU;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)  Error_Handler();
    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) Error_Handler();

    SlaveZC_ResetStats();
    s_win_start_us = TIM2->CNT;

    /* Discard any edge that was latched before we were ready, then enable
     * the interrupt. Highest priority: the converter commutation that will
     * be built on this edge must not be delayed by anything else. */
    __HAL_GPIO_EXTI_CLEAR_RISING_IT(ZCD_Pin);
    __HAL_GPIO_EXTI_CLEAR_FALLING_IT(ZCD_Pin);
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, IRQ_PRIO_TIMING, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

#if (SLAVE_FLT_FILTER_US > 0U) && (SLAVE_POWER_STAGE >= 1)
/* v0.9.62, the FLT glitch filter (slave_flt_filter.h), called by EXTI2_3_IRQHandler INSTEAD of the plain shutdown when SLAVE_FLT_FILTER_US > 0.
 * Wait the window on the CPU clock, clear the edge flags, THEN read the pin (an edge after the clear raises a new interrupt; one before it is seen by the read).
 * Still asserted, or too many spikes in this millisecond: the outputs off at once and the normal FLT handling. Released: ignored. */
static FltFilter_t s_fltf;
static uint32_t flt_systick_val(void) { return SysTick->VAL; }
uint32_t SlaveZC_FltIgnored(void) { return s_fltf.ignored; }
void SlaveZC_FltIrqFiltered(void)
{
    const uint32_t cyc = (uint32_t)SLAVE_FLT_FILTER_US * (SystemCoreClock / 1000000U);
    uint8_t asserted;
    FltFilter_Wait(flt_systick_val, SysTick->LOAD + 1U, cyc, cyc + 16U);
    __HAL_GPIO_EXTI_CLEAR_FALLING_IT(GPIO_PIN_2);
    __HAL_GPIO_EXTI_CLEAR_RISING_IT(GPIO_PIN_2);
    asserted = (uint8_t)(SlavePower_FltAsserted() | SlavePower_FltAsserted());
    if (FltFilter_Judge(&s_fltf, asserted, HAL_GetTick()) == 0U) return;
    SlavePower_AllOffNow();
    SlaveCtrl_OnFltIrq();
}
#else
uint32_t SlaveZC_FltIgnored(void) { return 0U; }
#endif

/* EXTI callbacks (called from HAL_GPIO_EXTI_IRQHandler, which
 * EXTI0_1_IRQHandler in stm32g0xx_it.c invokes). Keep these tiny. */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
    uint32_t now_us;
#if (SLAVE_POWER_STAGE >= 1)
    if (GPIO_Pin == GPIO_PIN_2) { SlaveCtrl_OnFltIrq(); return; }   /* FLT, if it asserts HIGH */
#endif
    if (GPIO_Pin != ZCD_Pin) return;
    now_us = TIM2->CNT;                      /* timestamp first */
#if (SLAVE_POWER_STAGE >= 1)
    SlaveCtrl_OnZcEdge(now_us, 1U);          /* BEFORE any bookkeeping: drop the converter diagonal */
#endif
#if (SLAVE_SEQ_SYNC != 0)
    SlavePhase_OnZc(now_us, 1U);             /* the reference edge of the phase sequence measurement */
#endif
    g_zc.level = ((ZCD_GPIO_Port->IDR & ZCD_Pin) != 0U) ? 1U : 0U;
    SlaveZC_OnEdge(now_us, 1U);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    uint32_t now_us;
#if (SLAVE_POWER_STAGE >= 1)
    if (GPIO_Pin == GPIO_PIN_2) { SlaveCtrl_OnFltIrq(); return; }   /* FLT asserts LOW */
#endif
    if (GPIO_Pin != ZCD_Pin) return;
    now_us = TIM2->CNT;
#if (SLAVE_POWER_STAGE >= 1)
    SlaveCtrl_OnZcEdge(now_us, 0U);
#endif
#if (SLAVE_SEQ_SYNC != 0)
    SlavePhase_OnZc(now_us, 0U);
#endif
    g_zc.level = ((ZCD_GPIO_Port->IDR & ZCD_Pin) != 0U) ? 1U : 0U;
    SlaveZC_OnEdge(now_us, 0U);
}
