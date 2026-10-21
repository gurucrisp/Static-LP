/**
  ******************************************************************************
  * @file    system_stm32g0xx.c
  * @brief   CMSIS Cortex-M0+ system source file for STM32G0B1.
  *          Standard CMSIS template content - provides SystemInit(),
  *          SystemCoreClock, SystemCoreClockUpdate(), and the AHB/APB
  *          prescaler lookup tables that stm32g0xx_hal_rcc.c links against.
  *          Actual clock tree setup (HSE/PLL/64 MHz) happens in
  *          SystemClock_Config() in main.c via HAL, not here - SystemInit()
  *          only does early, minimal core setup before main() runs.
  ******************************************************************************
  */

#include "stm32g0xx.h"

#if !defined(HSE_VALUE)
#define HSE_VALUE 8000000U
#endif

#if !defined(HSI_VALUE)
#define HSI_VALUE 16000000U
#endif

/* Vector table base offset field - relocate the vector table if needed
 * (e.g. bootloader scenarios). 0x0 = vector table stays at the start of
 * flash/SRAM, the normal case for this project. */
#if !defined(VECT_TAB_OFFSET)
#define VECT_TAB_OFFSET 0x00000000U
#endif

/* SystemCoreClock: current core clock frequency in Hz. HAL_InitTick() and
 * others read this directly. Starts at the HSI reset default (16 MHz) -
 * SystemClock_Config() in main.c (via HAL_RCC_ClockConfig) is what actually
 * switches to the 8 MHz HSE + PLL -> 64 MHz configuration; call
 * SystemCoreClockUpdate() afterward (HAL does this internally) to keep this
 * variable in sync with the real, current clock. */
uint32_t SystemCoreClock = 16000000U;

const uint32_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint32_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

/**
  * @brief  Minimal early setup, called from the reset handler before main().
  *         On STM32G0, actual clock configuration is done later via HAL in
  *         SystemClock_Config(), not here - this only does the small amount
  *         of setup that must happen before any C runtime/HAL code runs.
  */
void SystemInit(void)
{
    /* FPU settings: not applicable - Cortex-M0+ has no FPU on this device. */

    /* Reset the RCC clock configuration to the default reset state
     * (HSI ON, PLL OFF, prescalers at default) - matches CMSIS convention
     * so SystemClock_Config() in main.c always starts from a known state. */
    RCC->CR |= RCC_CR_HSION;

    /* Reset CFGR (system clock switch, prescalers, MCO) */
    RCC->CFGR = 0x00000000U;

    /* Reset HSEON, CSSON, PLLON */
    RCC->CR &= 0xFAF6FEFFU;

    /* Reset PLLCFGR */
    RCC->PLLCFGR = 0x00001000U;

    /* Reset HSEBYP */
    RCC->CR &= 0xFFFBFFFFU;

    /* Disable all interrupts */
    RCC->CIER = 0x00000000U;

#if defined(VECT_TAB_SRAM)
    SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET;
#else
    SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET;
#endif
}

/**
  * @brief  Recomputes SystemCoreClock from the actual current RCC register
  *         state. HAL's clock-config functions call this internally after
  *         switching clocks, so it's rarely called directly - included here
  *         for completeness/debugging use.
  */
void SystemCoreClockUpdate(void)
{
    uint32_t tmp, pllvco, pllr, pllsource, pllm;

    /* Get SYSCLK source */
    tmp = RCC->CFGR & RCC_CFGR_SWS;

    switch (tmp)
    {
        case RCC_CFGR_SWS_0:                 /* HSE used as system clock */
            SystemCoreClock = HSE_VALUE;
            break;

        case (RCC_CFGR_SWS_1):               /* LSI used as system clock */
            SystemCoreClock = 32000U;
            break;

        case (RCC_CFGR_SWS_1 | RCC_CFGR_SWS_0): /* LSE used as system clock */
            SystemCoreClock = 32768U;
            break;

        case (RCC_CFGR_SWS_2):               /* PLL used as system clock */
            pllsource = RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC;
            pllm = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1U;

            if (pllsource == RCC_PLLCFGR_PLLSRC_HSE)
            {
                pllvco = (HSE_VALUE / pllm);
            }
            else
            {
                pllvco = (HSI_VALUE / pllm);
            }

            pllvco = pllvco * ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos);
            pllr = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1U);
            SystemCoreClock = pllvco / pllr;
            break;

        case 0U:                              /* HSI used as system clock */
        default:
            SystemCoreClock = HSI_VALUE;
            break;
    }

    /* Apply the AHB prescaler */
    tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos)];
    SystemCoreClock >>= tmp;
}
