/**
  ******************************************************************************
  * @file    stm32g0xx_it.h
  * @brief   Interrupt/exception handler declarations (slave, STM32G0B1RET6).
  ******************************************************************************
  */

#ifndef __STM32G0xx_IT_H
#define __STM32G0xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void EXTI0_1_IRQHandler(void);

/* Power-stage interrupts (defined only when SLAVE_POWER_STAGE >= 1) */
void TIM2_IRQHandler(void);
void TIM3_TIM4_IRQHandler(void);
void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
void TIM15_IRQHandler(void);
void EXTI2_3_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32G0xx_IT_H */
