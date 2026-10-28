/* declarations only: a stand-in for the STM32G0 HAL, to compile-check the SLAVE's hardware glue (slave_link.c, slave_zc.c) on the PC. Same idea as tests/hal_check (the master's), with what the slave uses. */
#ifndef STM32G0XX_HAL_H
#define STM32G0XX_HAL_H
#include <stdint.h>
typedef enum { HAL_OK = 0, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2], BRR; } GPIO_TypeDef;
typedef struct { volatile uint32_t CR1, CR2, CR3, BRR, GTPR, RTOR, RQR, ISR, ICR, RDR, TDR, PRESC; } USART_TypeDef;
typedef struct { volatile uint32_t CCR, CNDTR, CPAR, CMAR; } DMA_Channel_TypeDef;
typedef struct { uint32_t Direction, PeriphInc, MemInc, PeriphDataAlignment, MemDataAlignment, Mode, Priority, Request; } DMA_InitTypeDef;
typedef struct { DMA_Channel_TypeDef *Instance; DMA_InitTypeDef Init; } DMA_HandleTypeDef;
typedef struct { uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling; } UART_InitTypeDef;
typedef struct { USART_TypeDef *Instance; UART_InitTypeDef Init; DMA_HandleTypeDef *hdmatx, *hdmarx; volatile uint32_t gState, RxState; } UART_HandleTypeDef;
typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET } GPIO_PinState;
typedef struct { volatile uint32_t CTRL, LOAD, VAL, CALIB; } SysTick_Type;
typedef struct { volatile uint32_t CSR; } RCC_TypeDef;
typedef struct { volatile uint32_t DR; } DWT_Type;
#define GPIOA ((GPIO_TypeDef *)0x50000000UL)
#define GPIOB ((GPIO_TypeDef *)0x50000400UL)
#define GPIOC ((GPIO_TypeDef *)0x50000800UL)
#define GPIOD ((GPIO_TypeDef *)0x50000C00UL)
#define GPIOF ((GPIO_TypeDef *)0x50001400UL)
#define GPIO_PIN_0 0x0001U
#define GPIO_PIN_1 0x0002U
#define GPIO_PIN_2 0x0004U
#define GPIO_PIN_3 0x0008U
#define GPIO_PIN_4 0x0010U
#define GPIO_PIN_5 0x0020U
#define GPIO_PIN_6 0x0040U
#define GPIO_PIN_7 0x0080U
#define GPIO_PIN_8 0x0100U
#define GPIO_PIN_9 0x0200U
#define GPIO_PIN_10 0x0400U
#define GPIO_PIN_11 0x0800U
#define GPIO_PIN_12 0x1000U
#define GPIO_PIN_13 0x2000U
#define GPIO_PIN_14 0x4000U
#define GPIO_PIN_15 0x8000U
#define GPIO_MODE_INPUT 0U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_NOPULL 0U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_LOW 0U
#define DMA_CIRCULAR 0x20U
#define ADC_CHANNEL_0 0U
#define USART_ISR_ORE 0x08U
#define USART_ISR_FE 0x02U
#define USART_ISR_NE 0x04U
#define USART_ICR_ORECF 0x08U
#define USART_ICR_FECF 0x02U
#define USART_ICR_NECF 0x04U
#define USART_CR3_DMAR 0x40U
#define RCC_CSR_IWDGRSTF 0x20000000U
#define RCC_CSR_RMVF 0x01000000U
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT) ((REG) & (BIT))
#define RCC ((RCC_TypeDef *)0x40021000UL)
#define SysTick ((SysTick_Type *)0xE000E010UL)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
extern uint32_t SystemCoreClock;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
void Error_Handler(void);
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Start(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init);

typedef struct { volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR, CCR1, CCR2, CCR3, CCR4, BDTR; } TIM_TypeDef;
typedef struct { uint32_t Prescaler, CounterMode, Period, ClockDivision, RepetitionCounter, AutoReloadPreload; } TIM_Base_InitTypeDef;
typedef struct { TIM_TypeDef *Instance; TIM_Base_InitTypeDef Init; } TIM_HandleTypeDef;
typedef struct { void *Instance; uint32_t dummy[16]; } ADC_HandleTypeDef;
#define TIM1 ((TIM_TypeDef *)0x40012C00UL)
#define TIM2 ((TIM_TypeDef *)0x40000000UL)
#define TIM3 ((TIM_TypeDef *)0x40000400UL)
#define TIM_COUNTERMODE_UP 0U
#define TIM_CLOCKDIVISION_DIV1 0U
#define TIM_AUTORELOAD_PRELOAD_DISABLE 0U
#define TIM_AUTORELOAD_PRELOAD_ENABLE 0x80U
#define USART_ISR_TXE_TXFNF 0x80U
#define USART_ISR_RXNE_RXFNE 0x20U
#define USART_CR1_RXNEIE_RXFNEIE 0x20U
#define UART_RXFIFO_THRESHOLD_1_8 0U
#define RCC_CSR_WWDGRSTF 0x40000000U
#define RCC_CSR_PWRRSTF 0x08000000U
typedef enum { EXTI0_1_IRQn = 5, EXTI2_3_IRQn = 6, USART1_IRQn = 27, TIM1_BRK_UP_TRG_COM_IRQn = 13, DMA1_Channel1_IRQn = 9, TIM15_IRQn = 20 } IRQn_Type;
#define __HAL_RCC_TIM2_CLK_ENABLE() ((void)0)
#define __HAL_RCC_TIM1_CLK_ENABLE() ((void)0)
extern UART_HandleTypeDef huart1;
uint32_t __get_PRIMASK(void);
void __set_PRIMASK(uint32_t m);
void __disable_irq(void);
HAL_StatusTypeDef HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold);
HAL_StatusTypeDef HAL_UARTEx_EnableFifoMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *huart, uint32_t Threshold);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
#define __HAL_GPIO_EXTI_CLEAR_RISING_IT(pin) ((void)(pin))
#define __HAL_GPIO_EXTI_CLEAR_FALLING_IT(pin) ((void)(pin))
/* v0.9.65: what slave_power.c uses (the inverter timers TIM1 / TIM15), so it is compile-checked too. The CCMR1 / CCER / BDTR / SR bit values are the STM32G0's. */
#define TIM15 ((TIM_TypeDef *)0x40014000UL)
#define GPIO_MODE_AF_PP 2U
#define GPIO_MODE_IT_FALLING 0x10210000U
#define GPIO_SPEED_FREQ_VERY_HIGH 3U
#define GPIO_AF2_TIM1 2U
#define GPIO_AF5_TIM15 5U
#define TIM_CCMR1_OC1M   0x00010070U
#define TIM_CCMR1_OC1M_0 0x00000010U
#define TIM_CCMR1_OC1M_1 0x00000020U
#define TIM_CCMR1_OC1M_2 0x00000040U
#define TIM_CCMR1_OC1M_3 0x00010000U
#define TIM_CCER_CC1E  0x0001U
#define TIM_CCER_CC1NE 0x0004U
#define TIM_BDTR_MOE   0x8000U
#define TIM_CR1_CEN    0x0001U
#define TIM_DIER_BIE   0x0080U
#define TIM_EGR_UG     0x0001U
#define TIM_SR_UIF     0x0001U
#define TIM_SR_BIF     0x0080U
#define TIM_CHANNEL_1 0U
#define TIM_CLOCKSOURCE_INTERNAL 0x1000U
#define TIM_TRGO_RESET 0U
#define TIM_MASTERSLAVEMODE_DISABLE 0U
#define TIM_OCMODE_PWM1 0x60U
#define TIM_OCPOLARITY_HIGH 0U
#define TIM_OCNPOLARITY_HIGH 0U
#define TIM_OCFAST_DISABLE 0U
#define TIM_OCIDLESTATE_RESET 0U
#define TIM_OCNIDLESTATE_RESET 0U
#define TIM_OSSR_ENABLE 0x0800U
#define TIM_OSSI_ENABLE 0x0400U
#define TIM_LOCKLEVEL_OFF 0U
#define TIM_BREAK_ENABLE 0x1000U
#define TIM_BREAKPOLARITY_LOW 0U
#define TIM_BREAK2_DISABLE 0U
#define TIM_BREAK2POLARITY_LOW 0U
#define TIM_AUTOMATICOUTPUT_DISABLE 0U
typedef struct { uint32_t ClockSource, ClockPolarity, ClockPrescaler, ClockFilter; } TIM_ClockConfigTypeDef;
typedef struct { uint32_t MasterOutputTrigger, MasterOutputTrigger2, MasterSlaveMode; } TIM_MasterConfigTypeDef;
typedef struct { uint32_t OCMode, Pulse, OCPolarity, OCNPolarity, OCFastMode, OCIdleState, OCNIdleState; } TIM_OC_InitTypeDef;
typedef struct { uint32_t OffStateRunMode, OffStateIDLEMode, LockLevel, DeadTime, BreakState, BreakPolarity, BreakFilter, BreakAFMode, Break2State, Break2Polarity, Break2Filter, Break2AFMode, AutomaticOutput; } TIM_BreakDeadTimeConfigTypeDef;
#define __HAL_RCC_TIM15_CLK_ENABLE() ((void)0)
void __enable_irq(void);
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef *cfg);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim, TIM_MasterConfigTypeDef *cfg);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *cfg, uint32_t ch);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *htim, TIM_BreakDeadTimeConfigTypeDef *cfg);
#endif
