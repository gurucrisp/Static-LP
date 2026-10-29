/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Exception/interrupt handlers for the slave (STM32G0B1RET6).
  *
  * WHY THIS FILE EXISTS: without a C definition of SysTick_Handler, the
  * startup file aliases it to Default_Handler (an infinite loop). HAL_Init()
  * enables the 1 ms SysTick interrupt, so the CPU froze ~1 ms after startup
  * and HAL_Delay()/HAL_GetTick() could never work. SysTick_Handler below
  * calls HAL_IncTick(), which is what the HAL timebase needs.
  *
  * EXTI0_1_IRQHandler below serves the zero-cross input (PB0, see slave_zc.c).
  * Other peripheral IRQ handlers (TIM1 break, TIM15, UART, ADC/DMA) are NOT
  * here yet - those interrupts are not enabled in this bring-up skeleton.
  * They get added when the code that uses them goes in.
  ******************************************************************************
  */

#include "main.h"
#include "stm32g0xx_it.h"
#include "slave_config.h"
#include "slave_features.h"
#include "slave_link.h"
#if (SLAVE_POWER_STAGE >= 1)
#include "slave_ctrl.h"
#endif
#if (SLAVE_POWER_STAGE >= 1) || (BRINGUP_FLT_TEST != 0)
#include "slave_zc.h"
#include "slave_flt_filter.h"
#include "slave_flt_cbc.h"
#include "slave_power.h"          /* the FLT handler calls SlavePower_AllOffNow(); the stage-0 FLT bench build needs it too */
#endif

/* On any fatal exception, make the power stage safe BEFORE parking the CPU:
 * the timers keep running in hardware even if the CPU is stuck, so without
 * this a crash would leave the inverter bridge switching with nothing
 * controlling it. Direct register writes only - no HAL calls, since the HAL
 * itself may be what faulted. */
static void Force_Outputs_Safe(void)
{
    /* Inverter: disable main outputs (MOE) of both PWM timers. */
    TIM1->BDTR  &= ~TIM_BDTR_MOE;
    TIM15->BDTR &= ~TIM_BDTR_MOE;
#if (SLAVE_POWER_STAGE >= 1)
    /* ...and take the four gate-driver inputs away from the timers: plain LOW outputs. */
    GPIOA->BSRR  = (uint32_t)(GPIO_PIN_7 | GPIO_PIN_8) << 16U;
    GPIOB->BSRR  = (uint32_t)(GPIO_PIN_13 | GPIO_PIN_14) << 16U;
    GPIOA->MODER = (GPIOA->MODER & ~((3UL << 14) | (3UL << 16))) | ((1UL << 14) | (1UL << 16));
    GPIOB->MODER = (GPIOB->MODER & ~((3UL << 26) | (3UL << 28))) | ((1UL << 26) | (1UL << 28));
#endif

    /* Converter GPIO pair (PB3, PB5, PC4, PC5): drive low. */
    GPIOB->BSRR = (uint32_t)(GPIO_PIN_3 | GPIO_PIN_5) << 16U;
    GPIOC->BSRR = (uint32_t)(GPIO_PIN_4 | GPIO_PIN_5) << 16U;
}

void NMI_Handler(void)
{
    Force_Outputs_Safe();
    while (1) { }
}

void HardFault_Handler(void)
{
    Force_Outputs_Safe();
    while (1) { }
}

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

/* HAL timebase: 1 ms tick. Required for HAL_Delay(), HAL_GetTick(), and
 * the timeouts inside the HAL clock/peripheral init functions. */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* Zero-cross input PB0 (EXTI line 0). The HAL dispatches to
 * HAL_GPIO_EXTI_Rising_Callback / _Falling_Callback, defined in slave_zc.c. */
void EXTI0_1_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

#if (SLAVE_POWER_STAGE >= 1)
/* ---------------------------------------------------------------------------
 * Power-stage interrupts. ALL at priority 0 (set in SlavePower_Init() and
 * CtrlHw_Init()). They only exist in the build when the power stage is on.
 * ------------------------------------------------------------------------- */

/* TIM2 compare channels: CC2 = converter dead time, CC3 = predicted-zero
 * turn-off, CC4 = zero-cross event watchdog.
 * Each is a one-shot: its interrupt enable is cleared before the callback. */
void TIM2_IRQHandler(void)
{
    uint32_t sr = TIM2->SR;

    if (((sr & TIM_SR_CC2IF) != 0U) && ((TIM2->DIER & TIM_DIER_CC2IE) != 0U))
    {
        TIM2->DIER &= ~TIM_DIER_CC2IE;
        TIM2->SR    = ~TIM_SR_CC2IF;
        SlaveCtrl_OnDeadtimeIrq();
    }
    if (((sr & TIM_SR_CC3IF) != 0U) && ((TIM2->DIER & TIM_DIER_CC3IE) != 0U))
    {
        TIM2->DIER &= ~TIM_DIER_CC3IE;
        TIM2->SR    = ~TIM_SR_CC3IF;
        SlaveCtrl_OnPredictOffIrq();
    }
    if (((sr & TIM_SR_CC4IF) != 0U) && ((TIM2->DIER & TIM_DIER_CC4IE) != 0U))
    {
        TIM2->DIER &= ~TIM_DIER_CC4IE;
        TIM2->SR    = ~TIM_SR_CC4IF;
        SlaveCtrl_OnWatchdogIrq();
    }
}

/* TIM3 update = the 20 kHz ADC trigger tick (TIM3 and TIM4 share this vector;
 * only the TIM3 update flag is enabled). Runs the polarity guard. */
void TIM3_TIM4_IRQHandler(void)
{
    if ((TIM3->SR & TIM_SR_UIF) != 0U)
    {
        TIM3->SR = ~TIM_SR_UIF;
        SlaveCtrl_OnSampleTick();
    }
}

/* Hardware break of TIM1 (PB12). The silicon has already disabled the inverter
 * outputs; the converter pins have no hardware break, so kill them first
 * (two register writes), then let the controller record the fault. The same
 * vector also carries the update/trigger/commutation events, which are not
 * enabled - only act if the break flag is really set. */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
    if ((TIM1->SR & TIM_SR_BIF) != 0U)
    {
        SlaveZC_BreakIrq();       /* v0.9.67: limited in RUN with the cycle-by-cycle limit, else the converter off and the fault (the silicon has already held the low sides on) */
    }
    SlaveZC_Tim1UpdateIrq();      /* v0.9.66: the cycle-by-cycle limit's period start (acts only while it is armed) */
}

/* Hardware break of TIM15 (PA9). */
void TIM15_IRQHandler(void)
{
    if ((TIM15->SR & TIM_SR_BIF) != 0U)
    {
        SlaveZC_BreakIrq();
    }
}

#endif /* SLAVE_POWER_STAGE >= 1 */

/* FLT input PD2 (EXTI line 2): dispatched to the callbacks in slave_zc.c. It exists from stage 1 upward, and in the stage-0 bench build with BRINGUP_FLT_TEST (where the
 * interrupt is armed by SlavePower_FltInit(): without a handler it would land in the default one, which loops forever). */
#if (SLAVE_POWER_STAGE >= 1) || (BRINGUP_FLT_TEST != 0)
#if (SLAVE_LINK_V2 != 0)
/* The link receive interrupt: the lowest priority (IRQ_PRIO_LINK), so it can never delay the FLT shutdown or the timing. It only moves bytes into a ring. */
void USART1_IRQHandler(void)
{
    SlaveLink_RxIrq();
}
#endif

void EXTI2_3_IRQHandler(void)
{
    /* FIRST, before the HAL handler and before any bookkeeping (decision D25): the CT over-current comparators arrive here, so every instruction before this line is
     * time during which the IGBTs keep carrying the fault current. Outputs off; the counting and the trip record follow in SlaveCtrl_OnFltIrq(). */
#if (SLAVE_FLT_CBC != 0) && (SLAVE_POWER_STAGE >= 1) && (BRINGUP_FLT_TEST == 0)
    SlaveZC_FltIrqCbc();          /* v0.9.66: the cycle-by-cycle limit (slave_flt_cbc.h): the PWM forced off first, then limit or trip */
#elif (SLAVE_FLT_FILTER_US > 0U) && (SLAVE_POWER_STAGE >= 1) && (BRINGUP_FLT_TEST == 0)
    SlaveZC_FltIrqFiltered();     /* v0.9.62: the glitch filter (slave_flt_filter.h): the shutdown comes SLAVE_FLT_FILTER_US later, only if FLT is still asserted */
#else
    SlavePower_FltOffNow();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
#endif
}
#endif
