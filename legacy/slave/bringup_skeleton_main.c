/* ============================================================================
 * SLAVE PERIPHERAL INIT SKELETON - STM32G0B1RET6 ("PB LP V1.0")
 * ============================================================================
 * Rewritten from scratch per requirements/schematic, NOT based on the old
 * codebase (old code was referenced only to verify pin usage/behavior).
 *
 * SCOPE OF THIS FILE: peripheral bring-up only. Clocks, GPIO, the two
 * inverter PWM timers (with dead-time + hardware BKIN), ADC, and the master
 * UART link are initialized and left in a safe, verified state - but no
 * regulation/control-loop logic runs yet. That comes next, once this is
 * confirmed against real hardware (scope on PWM pins, multimeter on ADC
 * inputs, UART loopback).
 *
 * PIN MAP - confirmed directly against schematic + STM32G0B1 datasheet AF
 * tables (see spec doc sections 2.4/2.5 for the full derivation):
 *
 *   PWM1 H/L  = PB14 / PB13  -> TIM15_CH1/CH1N  (inverter, hw complementary + BKIN)
 *   PWM2 H/L  = PA8  / PA7   -> TIM1_CH1/CH1N   (inverter, hw complementary + BKIN)
 *   PWM3 H/L  = PB3  / PC4   -> plain GPIO      (converter, zero-cross rate only)
 *   PWM4 H/L  = PC5  / PB5   -> plain GPIO      (converter, zero-cross rate only)
 *   TIM1_BKIN  = PB12
 *   TIM15_BKIN = PA9
 *   FLT (soft fault input, not silicon BKIN) = PD2
 *   ZCD        = PB0   (EXTI both edges; captured by slave_zc.c)
 *   NTC        = PA5  (ADC)
 *   MCU_AC_IN  = PA0  (ADC, input voltage)
 *   MCU_AC_OUT = PA1  (ADC, output voltage)
 *   Relay SH (precharge) = PC9
 *   Relay W  (main)      = PC8
 *   Buzzer   = PA6
 *   Fan      = PB9
 *   Master-link UART: TX = PB6, RX = PB7 (simple RX/TX, no enable pin)
 *   8 MHz HSE crystal: PF0/PF1
 *   SWD (programming): PA13/PA14
 *
 *   NOT USED / reserved for future expansion (left unconfigured):
 *   PC11, PC13, PC14, PC15, PC0 (Contactor1), PC1 (Contactor2), PA2
 *   (current meas.), PA4 (LCD_UART1_EN), PA15, PD0, PD1, PD3, PD4, PB10, PB11
 *   (UART2, unused), CONT_2, GPIO_1-4, RLY3
 * ========================================================================== */

#include "main.h"
#include "slave_zc.h"
#include "slave_meas.h"
#include "slave_link.h"
#include "link_proto.h"
#include <stdint.h>

/* ----------------------------------------------------------------------
 * Handles
 * ---------------------------------------------------------------------- */
TIM_HandleTypeDef htim1;   /* Inverter PWM2 (PA8/PA7), CH1/CH1N, hw BKIN  */
TIM_HandleTypeDef htim15;  /* Inverter PWM1 (PB14/PB13), CH1/CH1N, hw BKIN */
UART_HandleTypeDef huart1; /* Master link: PB6=TX, PB7=RX                 */

/* ----------------------------------------------------------------------
 * PWM timing constants
 * ---------------------------------------------------------------------- */
#define PWM_TIMER_CLK_HZ     64000000UL   /* APB timer clock after PLL, see SystemClock_Config */
#define PWM_SWITCHING_HZ     20000UL      /* 20 kHz switching frequency - adjust to your design */
#define PWM_PERIOD_TICKS     (PWM_TIMER_CLK_HZ / PWM_SWITCHING_HZ)   /* = 3200 ticks */

/* Inverter hardware dead time: raw value written to TIM1/TIM15 BDTR.DTG.
 * For DTG < 128 the dead time is DTG x t_DTS, and t_DTS = 1/64 MHz =
 * 15.625 ns here (ClockDivision DIV1), so:  64 -> 1.0 us,  96 -> 1.5 us.
 * (128 = 0x80 switches to a coarser step and gives 2.0 us.)
 *
 * WHY 1.0 us: the previous placeholder (32 -> 0.5 us) is BELOW the minimum
 * implied by the FGL60N100BNTD datasheet (typ., at 600 V / 60 A / 51 ohm /
 * 15 V): td(off) 630 ns + tf 130 ns (250 ns max) = about 0.76-0.88 us for the
 * collector current to actually stop, versus td(on) 140 ns for the partner
 * device to start - a difference of about 0.6-0.7 us before any margin for
 * TLP152 delay mismatch, temperature, or this board's slower gate network
 * (47 ohm + about 2 x 10 nF gate-emitter). 1.0 us is also the value the old
 * firmware initialised these timers with (DeadTime = 64).
 * It is still only a starting value: confirm the real gap at the IGBT gates
 * (test points on the power board) with the DC bus UNPOWERED, and keep
 * mains / DC bus disconnected until that is done.
 *
 * NOTE: this is the INVERTER hardware dead time only. The converter pairs
 * (PB3/PC4, PC5/PB5) need their own, much longer SOFTWARE dead time at each
 * zero-crossing (the old firmware used 40 us); that is not implemented here. */
#define PWM_DEADTIME_DTG      64U

/* ----------------------------------------------------------------------
 * SystemClock_Config: 8 MHz HSE -> PLL -> 64 MHz SYSCLK (max for this chip)
 * ---------------------------------------------------------------------- */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    /* 8 MHz HSE / M=1 * N=16 / P,Q,R=2  -> VCO 128 MHz -> 64 MHz out */
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 16;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                 | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ----------------------------------------------------------------------
 * GPIO init - every confirmed pin from the map above.
 * ---------------------------------------------------------------------- */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* --- Converter GPIO pair (plain outputs, no timer AF) - start LOW/safe --- */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_3 | GPIO_PIN_5;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;   /* toggles only at zero-cross rate */
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* --- Inverter gate-driver inputs (PB14, PB13, PA8, PA7): plain outputs,
     * LOW from the very start, so every IGBT is off. They are only handed to
     * the timers (alternate function) by HAL_TIM_MspPostInit() when
     * BRINGUP_PWM_MODE is 1. --- */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 | GPIO_PIN_8, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* --- Relays (SH precharge = PC9, W main = PC8) - start OFF/safe --- */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* --- Buzzer (PA6), Fan (PB9) - start OFF --- */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_6;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* --- FLT input (PD2) - soft fault input, not silicon BKIN, pulled up --- */
    GPIO_InitStruct.Pin  = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* --- ZCD input (PB0): interrupt on BOTH edges. The ACOUT line of the
     * zero-cross IC is open-drain, so it idles high through a pull-up; the
     * internal pull-up here only guarantees a defined level (no phantom
     * edges) if that signal is ever disconnected. The NVIC side is enabled
     * in SlaveZC_Init() (slave_zc.c) once its timestamp timer is running. --- */
    GPIO_InitStruct.Pin  = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* --- Analog inputs: AC_IN (PA0), AC_OUT (PA1), NTC (PA5) --- */
    GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* --- TIM1_BKIN (PB12), TIM15_BKIN (PA9): configured by HAL_TIM_MspInit
     * via the timer init below (alternate function, not plain GPIO) - no
     * separate GPIO_Init call needed here. --- */

    /* --- PWM1 (PB14/PB13) and PWM2 (PA8/PA7): configured by HAL_TIM_MspInit
     * via the timer init below (AF2/AF5, per datasheet) - no separate
     * GPIO_Init call needed here either. --- */
}

/* Explicit alternate-function pin setup for the PWM + BKIN pins of a timer.
 * Defined below. Called directly at the end of MX_TIM1_Init/MX_TIM15_Init -
 * NOT left to the HAL's HAL_TIM_PWM_MspInit callback, because the HAL only
 * calls that when the timer state is still RESET, and HAL_TIM_Base_Init()
 * (used in MX_TIM1_Init) already moves it to READY, so TIM1's pins were
 * silently never configured. (This is also how CubeMX itself does it.) */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* ----------------------------------------------------------------------
 * TIM1 init: PWM2 (PA8=CH1, PA7=CH1N), hardware dead-time, TIM1_BKIN (PB12)
 * ---------------------------------------------------------------------- */
static void __attribute__((unused)) MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler         = 0;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = PWM_PERIOD_TICKS - 1U;
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

    /* Start at zero duty - never come out of init already driving the bridge. */
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();

    /* Hardware break input (TIM1_BKIN = PB12) + dead-time. BreakState is
     * ENABLE so a break event forces MOE off in silicon, no CPU involved -
     * this is the fast hardware inrush/fault protection path. Polarity is
     * ACTIVE_LOW: confirm against your actual comparator/fault-circuit
     * output polarity before relying on this - if the real signal idles
     * high and pulls low on a fault, LOW is correct; if it's the other way
     * around, this needs to be ACTIVE_HIGH instead, or the break will never
     * un-latch (or will latch immediately at idle). VERIFY ON HARDWARE. */
    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = PWM_DEADTIME_DTG;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_ENABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.Break2State      = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity   = TIM_BREAK2POLARITY_LOW;
    sBreakDeadTimeConfig.Break2Filter     = 0;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

    /* Wire PA8/PA7 (PWM) and PB12 (BKIN) to TIM1 - see note above. */
    HAL_TIM_MspPostInit(&htim1);
}

/* ----------------------------------------------------------------------
 * TIM15 init: PWM1 (PB14=CH1, PB13=CH1N), hardware dead-time, TIM15_BKIN (PA9)
 * ---------------------------------------------------------------------- */
static void __attribute__((unused)) MX_TIM15_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    __HAL_RCC_TIM15_CLK_ENABLE();

    htim15.Instance = TIM15;
    htim15.Init.Prescaler         = 0;
    htim15.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim15.Init.Period            = PWM_PERIOD_TICKS - 1U;
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

    /* Same caveat as TIM1 above: BreakPolarity assumed ACTIVE_LOW - VERIFY
     * against the real TIM15_BKIN (PA9) source signal on hardware. */
    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = PWM_DEADTIME_DTG;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_ENABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
    sBreakDeadTimeConfig.BreakFilter      = 0;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK) Error_Handler();

    /* Wire PB14/PB13 (PWM) and PA9 (BKIN) to TIM15 - same explicit path as TIM1. */
    HAL_TIM_MspPostInit(&htim15);
}

/* GPIO alternate-function wiring for the PWM + BKIN pins. Called explicitly
 * from MX_TIM1_Init()/MX_TIM15_Init() (see the note at its declaration) -
 * deliberately not named HAL_TIM_PWM_MspInit, which the HAL may skip.
 * Kept here (not in a separate msp.c) for visibility while this is still a
 * bring-up skeleton. */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (htim->Instance == TIM1)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PA8 = TIM1_CH1 (AF2), PA7 = TIM1_CH1N (AF2) */
        GPIO_InitStruct.Pin       = GPIO_PIN_8;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_7;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

        /* PB14 = TIM15_CH1 (AF5), PB13 = TIM15_CH1N (AF5) */
        GPIO_InitStruct.Pin       = GPIO_PIN_14;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_TIM15;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_13;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* PA9 = TIM15_BKIN (AF5) */
        GPIO_InitStruct.Pin       = GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Alternate = GPIO_AF5_TIM15;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/* (ADC1 + DMA + the 20 kHz trigger now live in slave_meas.c.) */

/* ----------------------------------------------------------------------
 * UART1 init: master link, PB6=TX / PB7=RX, simple RX/TX only (no enable
 * pin - confirmed not needed for this link).
 * Baud rate: LINK_BAUDRATE (link_proto.h, 500000), 8N1. It divides the 64 MHz
 * clock exactly (BRR = 128), and the same value divides a 16 MHz master clock
 * exactly (BRR = 32). The frame format is in link_proto.h; the status frame is
 * sent by slave_link.c.
 * ---------------------------------------------------------------------- */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate     = LINK_BAUDRATE;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* PB6 = USART1_TX (AF0), PB7 = USART1_RX (AF0) */
        GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/* ----------------------------------------------------------------------
 * Error_Handler: placeholder - a real one should latch a safe state
 * (PWM off, relays open) rather than just spin, once the control loop
 * exists. For this bring-up skeleton it just traps execution so a failed
 * init is obvious on a debugger.
 * ---------------------------------------------------------------------- */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

/* ----------------------------------------------------------------------
 * main(): bring-up only. Initializes everything above, starts PWM
 * channels AT ZERO DUTY (verify with a scope: pulses present, ~20 kHz,
 * correct dead-time gap between H and L, and that a BKIN event actually
 * forces the outputs off), then idles. No regulation logic yet.
 * ---------------------------------------------------------------------- */
/* ----------------------------------------------------------------------
 * TEMPORARY BRING-UP READOUT: report numbers as BEEP CODES on the buzzer,
 * for use when no debugger is available. Timed from TIM2 (1 us timebase,
 * started by SlaveZC_Init), so it does not depend on SysTick or on compiler
 * optimisation. Bench-confirmed buzzer drive: ACTIVE-HIGH (SET = sounding).
 *
 * How to read it:
 *   digit 1..9  -> that many SHORT beeps
 *   digit 0     -> ONE LONG beep
 *   a number    -> its digits one after another (short pause between digits)
 *   a report    -> TWO VERY LONG beeps (start), then the numbers one after
 *                  another, each separated by a long pause.
 * The blocking delays are not compatible with real-time regulation: remove
 * this readout (replace it with the UART protocol) when the control loop
 * goes in.
 * ---------------------------------------------------------------------- */
/* Inverter PWM during bring-up:
 *   0 = OFF. TIM1/TIM15 are not started; the four gate-driver input pins
 *       (PB14, PB13, PA8, PA7) are plain GPIO outputs held LOW from the first
 *       instant, so every IGBT is off. USE THIS for any test with mains or a
 *       DC bus connected (the voltage measurement tests).
 *   1 = 20 kHz, 50 % duty debug pulses with the 1 us hardware dead time and
 *       the break inputs (for scope checks of the PWM signals, gate-driver
 *       supply only - NO mains, NO DC bus).                                  */
#define BRINGUP_PWM_MODE     0

/* What the readout reports (see the loop in main()):
 *   0 = zero-cross statistics of PB0 (rise/s, fall/s, low width, high width)
 *   1 = measurements: Vin (V), Vout (V), mains frequency x10 (0 if unknown)
 *   2 = silent: no beeps; the values go to the master over the link only      */
#define BRINGUP_REPORT_MODE  1

#define BEEP_SHORT_US        120000U
#define BEEP_LONG_US         450000U
#define BEEP_GAP_US          200000U    /* between beeps inside a digit      */
#define BEEP_DIGIT_GAP_US    400000U    /* extra pause after each digit      */
#define BEEP_ITEM_GAP_US    2000000U    /* between numbers in a report       */
#define BEEP_START_US       1000000U    /* the two start-of-report beeps     */
#define BEEP_REPORT_GAP_US  6000000U    /* pause before the next report      */

static __attribute__((unused)) void Bringup_DelayUs(uint32_t us)
{
    uint32_t t0 = SlaveZC_NowUs();
    while ((uint32_t)(SlaveZC_NowUs() - t0) < us)
    {
        SlaveMeas_Service();     /* keep the measurement and the master link running   */
        SlaveLink_Service();     /* during the (long) beep delays                       */
    }
}

static __attribute__((unused)) void Bringup_Tone(uint32_t on_us)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);     /* sounding */
    Bringup_DelayUs(on_us);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);   /* silent   */
}

/* Splits v into decimal digits, most significant first. Returns the count
 * (1..5). Values above 99999 are clamped. */
static __attribute__((unused)) uint8_t Bringup_SplitDigits(uint32_t v, uint8_t out[5])
{
    uint8_t tmp[5];
    uint8_t n = 0U;
    if (v > 99999U) v = 99999U;
    do
    {
        tmp[n++] = (uint8_t)(v % 10U);
        v /= 10U;
    } while (v != 0U);
    for (uint8_t i = 0U; i < n; i++) out[i] = tmp[n - 1U - i];
    return n;
}

static __attribute__((unused)) void Bringup_BeepDigit(uint8_t d)
{
    if (d == 0U)
    {
        Bringup_Tone(BEEP_LONG_US);
        Bringup_DelayUs(BEEP_GAP_US);
        return;
    }
    for (uint8_t i = 0U; i < d; i++)
    {
        Bringup_Tone(BEEP_SHORT_US);
        Bringup_DelayUs(BEEP_GAP_US);
    }
}

static __attribute__((unused)) void Bringup_BeepNumber(uint32_t v)
{
    uint8_t digits[5];
    uint8_t n = Bringup_SplitDigits(v, digits);
    for (uint8_t i = 0U; i < n; i++)
    {
        Bringup_BeepDigit(digits[i]);
        Bringup_DelayUs(BEEP_DIGIT_GAP_US);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
#if (BRINGUP_PWM_MODE == 1)
    MX_TIM1_Init();
    MX_TIM15_Init();
#endif
    MX_USART1_UART_Init();
    SlaveZC_Init();      /* TIM2 1 MHz timebase + ZCD (PB0) edge capture - see slave_zc.h */
    SlaveMeas_Init();    /* TIM3 20 kHz trigger + ADC1 + circular DMA - see slave_meas.h  */
    SlaveLink_Init();    /* status frame to the master every 10 ms - see slave_link.h    */

#if (BRINGUP_PWM_MODE == 1)
    /* TEMPORARY BRING-UP DEBUG PWM: 20 kHz, 50 % duty on both inverter legs.
     * Gate-driver supply only - never with mains or a DC bus connected, and
     * never as the final behaviour (the control loop sets the duty). */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);

    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_1, PWM_PERIOD_TICKS / 2U);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, PWM_PERIOD_TICKS / 2U);
#endif

    while (1)
    {
#if (BRINGUP_REPORT_MODE == 2)
        /* Silent: only keep the measurement and the master link running. */
        SlaveMeas_Service();
        SlaveLink_Service();
#else
        /* TEMPORARY BRING-UP READOUT (no debugger): once per cycle, report
         * the selected numbers as beep codes - see the legend above
         * Bringup_DelayUs() and BRINGUP_REPORT_MODE. Values are snapshotted
         * first so that one report is self-consistent. */
#if (BRINGUP_REPORT_MODE == 0)
        /* Zero-cross statistics. Numbers, in order:
         *   1) rising edges in the last whole second
         *   2) falling edges in the last whole second
         *   3) time the line was LOW  (last measurement), in 10 us units
         *   4) time the line was HIGH (last measurement), in 10 us units
         * All four 0 (four long beeps) = no edges seen on PB0. */
        uint32_t rep_a = g_zc.rise_per_s;
        uint32_t rep_b = g_zc.fall_per_s;
        uint32_t rep_c = g_zc.width_lo_us / 10U;
        uint32_t rep_d = g_zc.width_hi_us / 10U;
        if (rep_c > 9999U) rep_c = 9999U;
        if (rep_d > 9999U) rep_d = 9999U;
#else
        /* Measurements. Numbers, in order:
         *   1) AC_IN  RMS in volts (mains-equivalent, rounded)
         *   2) AC_OUT RMS in volts
         *   3) mains frequency in Hz x10 (e.g. 500 = 50.0 Hz); 0 if no valid
         *      zero-cross rate yet (the window then assumes 50 Hz)
         * Volts use the NOMINAL schematic scaling until a calibration exists.
         * All 0 (long beeps) = no result yet. */
        uint32_t rep_a = 0U, rep_b = 0U, rep_c = 0U, rep_d = 0U;
        if (g_meas.valid)
        {
            rep_a = (uint32_t)(g_meas.vrms_in  + 0.5f);
            rep_b = (uint32_t)(g_meas.vrms_out + 0.5f);
            rep_c = g_meas.freq_valid ? (uint32_t)((g_meas.freq_hz * 10.0f) + 0.5f) : 0U;
        }
#endif

        Bringup_Tone(BEEP_START_US);                 /* start-of-report marker */
        Bringup_DelayUs(300000U);
        Bringup_Tone(BEEP_START_US);
        Bringup_DelayUs(BEEP_ITEM_GAP_US);

        Bringup_BeepNumber(rep_a);  Bringup_DelayUs(BEEP_ITEM_GAP_US);
        Bringup_BeepNumber(rep_b);  Bringup_DelayUs(BEEP_ITEM_GAP_US);
        Bringup_BeepNumber(rep_c);
#if (BRINGUP_REPORT_MODE == 0)
        Bringup_DelayUs(BEEP_ITEM_GAP_US);
        Bringup_BeepNumber(rep_d);
#else
        (void)rep_d;
#endif

        Bringup_DelayUs(BEEP_REPORT_GAP_US);
#endif

        /* Control logic (zero-cross sync, converter GPIO polarity
         * selection, regulation, protection state machine, UART protocol)
         * goes here next, once this skeleton is confirmed against real
         * hardware. */
    }
}
