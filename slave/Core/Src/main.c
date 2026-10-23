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
#include "slave_config.h"      /* SLAVE_POWER_STAGE, BRINGUP_* switches */
#include "slave_zc.h"
#include "slave_meas.h"
#include "slave_features.h"
#include "slave_cal_hw.h"
#if (SLAVE_AUTOTUNE != 0)
#include "slave_profile.h"
#include "slave_flash.h"
#endif
#include "slave_iwdg.h"
#include "slave_link.h"
#include "slave_power.h"       /* TIM1/TIM15 setup now lives here (htim1, htim15, POWER_*) */
#if (SLAVE_POWER_STAGE >= 1)
#include "slave_ctrl.h"
#endif
#include "link_proto.h"
#include <stdint.h>

/* ----------------------------------------------------------------------
 * Handles
 * ---------------------------------------------------------------------- */
/* htim1 / htim15 (inverter PWM timers) are defined in slave_power.c */
UART_HandleTypeDef huart1; /* Master link: PB6=TX, PB7=RX                 */

/* PWM timing constants and the 1.0 us hardware dead time: see slave_power.h */

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

    /* --- Output contactor control (FODM3083 opto-triac + BTA204S-800C on PHASE-OUT): CONT1 = PC0 is ACTIVE LOW, HIGH = contactor OFF; CONT2 = PC1.
     * Driven like the OLD firmware's start-up: PC0 HIGH (OFF) with pull-up, PC1 LOW. Before v0.8.0 these two pins were left UNCONFIGURED (high impedance): the
     * output stage was in whatever state the board's resistors gave it. The level is written BEFORE the pin becomes an output, so there is no glitch.
     * What CONT2 drives is NOT known to me: I reproduce the old state. Contactor control itself (closing it for a load) is NOT implemented. --- */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin   = GPIO_PIN_0;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_InitStruct.Pin   = GPIO_PIN_1;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

/* (TIM1/TIM15 setup now lives in slave_power.c; ADC1 + DMA + the 20 kHz trigger in slave_meas.c.) */

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
/* BRINGUP_PWM_MODE is defined in slave_config.h */

/* What the readout reports (see the loop in main()):
 *   0 = zero-cross statistics of PB0 (rise/s, fall/s, low width, high width)
 *   1 = measurements: Vin (V), Vout (V), mains frequency x10 (0 if unknown)
 *   2 = silent: no beeps; the values go to the master over the link only      */
/* BRINGUP_REPORT_MODE is defined in slave_config.h */

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
    SlavePower_TimersInit();   /* same verified TIM1/TIM15 + break setup the control code uses */
#endif
    MX_USART1_UART_Init();
    SlaveZC_Init();      /* TIM2 1 MHz timebase + ZCD (PB0) edge capture - see slave_zc.h */
    SlaveMeas_Init();    /* TIM3 20 kHz trigger + ADC1 + circular DMA - see slave_meas.h  */
#if (SLAVE_VOLTAGE_CAL != 0)
    SlaveCalHw_Init();   /* the stored voltage calibration (two copies, CRC) is applied to the measurement */
#endif
#if (SLAVE_AUTOTUNE != 0)
    SlaveProfile_Init(&g_slave_flash, SFLASH_PROF_PAGE_A, SFLASH_PROF_PAGE_B);   /* the transformer profile (two copies, CRC, validated): $H tuned follows it */
#endif
    SlaveLink_Init();    /* status frame to the master every 10 ms - see slave_link.h    */

#if (SLAVE_POWER_STAGE >= 1)
    /* Power stage: ALL outputs stay off until the controller has validated the
     * zero-cross pattern and the inputs (see slave_ctrl.h). */
    SlavePower_Init();   /* timers, break interrupts, converter pins low, FLT interrupt */
    SlaveCtrl_Init();    /* state OFF; run request only if SLAVE_BENCH_AUTORUN          */
#endif

#if (BRINGUP_PWM_MODE == 1)
    /* TEMPORARY BRING-UP DEBUG PWM: 20 kHz, 50 % duty on both inverter legs.
     * Gate-driver supply only - never with mains or a DC bus connected, and
     * never as the final behaviour (the control loop sets the duty). */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);

    __HAL_TIM_SET_COMPARE(&htim1,  TIM_CHANNEL_1, POWER_PERIOD_TICKS / 2U);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, POWER_PERIOD_TICKS / 2U);
#if (BRINGUP_FLT_TEST != 0)
    SlavePower_FltInit();    /* bench: a FLT edge on PD2 switches the debug PWM off (gate supply only, NO MAINS): scope the delay */
#endif
#endif

#if (SLAVE_WATCHDOG != 0) && (BRINGUP_REPORT_MODE == 2)
    (void)SlaveIwdg_Start(SLAVE_WATCHDOG_MS);   /* after ALL the slow initialisation; it cannot be stopped again */
#endif
    while (1)
    {
#if (SLAVE_WATCHDOG != 0) && (BRINGUP_REPORT_MODE == 2)
        SlaveIwdg_Kick();                       /* once per pass of the main loop, never from an interrupt */
#endif
#if (BRINGUP_REPORT_MODE == 2)
        /* Silent: only keep the measurement and the master link running. */
        SlaveMeas_Service();
        SlaveLink_Service();
#if (SLAVE_POWER_STAGE >= 1)
        SlaveCtrl_Service();
#endif
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
