/**
  ******************************************************************************
  * @file    main.h
  * @brief   Header for main.c - slave (STM32G0B1RET6, "PB LP V1.0") bring-up
  *          skeleton. Pin defines below match the labeled pins in slave.ioc
  *          and the confirmed pin map (spec doc sections 2.4/2.5).
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g0xx_hal.h"

/* Exported functions prototypes --------------------------------------------*/
void Error_Handler(void);
void SystemClock_Config(void);

/* Private defines - labeled pins, matching slave.ioc GPIO_Label entries ---- */
#define ZCD_Pin                 GPIO_PIN_0
#define ZCD_GPIO_Port           GPIOB

#define PWM3H_Pin               GPIO_PIN_3
#define PWM3H_GPIO_Port         GPIOB

#define PWM4L_Pin               GPIO_PIN_5
#define PWM4L_GPIO_Port         GPIOB

#define PWM3L_Pin               GPIO_PIN_4
#define PWM3L_GPIO_Port         GPIOC

#define PWM4H_Pin               GPIO_PIN_5
#define PWM4H_GPIO_Port         GPIOC

#define RELAY_W_Pin              GPIO_PIN_8
#define RELAY_W_GPIO_Port        GPIOC

#define RELAY_SH_Pin             GPIO_PIN_9
#define RELAY_SH_GPIO_Port       GPIOC

#define FLT_Pin                  GPIO_PIN_2
#define FLT_GPIO_Port            GPIOD

#define BUZZER_Pin               GPIO_PIN_6
#define BUZZER_GPIO_Port         GPIOA

#define FAN_Pin                  GPIO_PIN_9
#define FAN_GPIO_Port            GPIOB

/* PWM1/PWM2 (TIM15/TIM1 channels) and the two BKIN pins are configured as
 * timer alternate-function pins in HAL_TIM_PWM_MspInit() (see
 * slave_main_skeleton.c) rather than plain GPIO - no _Pin/_Port defines
 * needed for those here. Analog pins (PA0/PA1/PA5) and the UART pins
 * (PB6/PB7) are likewise configured directly in their respective Init/
 * MspInit functions. */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
