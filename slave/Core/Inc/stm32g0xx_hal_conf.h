/**
  ******************************************************************************
  * @file    stm32g0xx_hal_conf.h
  * @brief   HAL configuration file for the slave (STM32G0B1RET6) project.
  ******************************************************************************
  * Standard ST HAL config template, trimmed to the modules this project
  * actually uses: RCC, GPIO, ADC, TIM (+TIMEx for PWM/break-deadtime), UART,
  * DMA (driver files present in the build log), CORTEX, FLASH, PWR, EXTI.
  * If you add a peripheral later (I2C, SPI, etc.) its #define here must be
  * uncommented too, or you'll get the same "undefined reference" class of
  * error again for that peripheral's HAL functions.
  ******************************************************************************
  */

#ifndef STM32G0xx_HAL_CONF_H
#define STM32G0xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------ */
/* Module Selection                                                         */
/* ------------------------------------------------------------------------ */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* ------------------------------------------------------------------------ */
/* Oscillator Values adaptation                                             */
/* ------------------------------------------------------------------------ */
#if !defined(HSE_VALUE)
#define HSE_VALUE 8000000U   /* 8 MHz HSE crystal, per schematic (PF0/PF1) */
#endif

#if !defined(HSE_STARTUP_TIMEOUT)
#define HSE_STARTUP_TIMEOUT 100U
#endif

#if !defined(HSI_VALUE)
#define HSI_VALUE 16000000U
#endif

#if !defined(LSI_VALUE)
#define LSI_VALUE 32000U
#endif

#if !defined(LSE_VALUE)
#define LSE_VALUE 32768U
#endif

#if !defined(LSE_STARTUP_TIMEOUT)
#define LSE_STARTUP_TIMEOUT 5000U
#endif

#if !defined(EXTERNAL_CLOCK_VALUE)
#define EXTERNAL_CLOCK_VALUE 48000U
#endif

#if !defined(EXTERNAL_I2S1_CLOCK_VALUE)
#define EXTERNAL_I2S1_CLOCK_VALUE 48000U
#endif

#if !defined(EXTERNAL_I2S2_CLOCK_VALUE)
#define EXTERNAL_I2S2_CLOCK_VALUE 48000U
#endif

/* ------------------------------------------------------------------------ */
/* System Configuration                                                     */
/* ------------------------------------------------------------------------ */
#define VDD_VALUE                    3300U
#define TICK_INT_PRIORITY            0U
#define USE_RTOS                     0U
#define PREFETCH_ENABLE               1U
#define INSTRUCTION_CACHE_ENABLE      0U
#define DATA_CACHE_ENABLE             0U

/* ------------------------------------------------------------------------ */
/* Ethernet peripheral configuration (not present on this MCU - left as
   default per ST template, no functional effect here) */
/* ------------------------------------------------------------------------ */

/* ------------------------------------------------------------------------ */
/* Assert Selection                                                         */
/* ------------------------------------------------------------------------ */
/* #define USE_FULL_ASSERT    1U */

/* Includes ---------------------------------------------------------------- */
#ifdef HAL_RCC_MODULE_ENABLED
#include "stm32g0xx_hal_rcc.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
#include "stm32g0xx_hal_gpio.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
#include "stm32g0xx_hal_dma.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
#include "stm32g0xx_hal_cortex.h"
#endif

#ifdef HAL_ADC_MODULE_ENABLED
#include "stm32g0xx_hal_adc.h"
#endif

#ifdef HAL_EXTI_MODULE_ENABLED
#include "stm32g0xx_hal_exti.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
#include "stm32g0xx_hal_flash.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
#include "stm32g0xx_hal_pwr.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32g0xx_hal_tim.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
#include "stm32g0xx_hal_uart.h"
#endif

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
#define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
void assert_failed(uint8_t *file, uint32_t line);
#else
#define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32G0xx_HAL_CONF_H */
