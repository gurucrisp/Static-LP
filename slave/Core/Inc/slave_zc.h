/**
  ******************************************************************************
  * @file    slave_zc.h
  * @brief   Zero-cross (ZCD, PB0) capture + characterisation module - slave.
  *
  * STEP 1 OF THE MEASUREMENT WORK: this module does NOT yet drive the
  * converter or compute RMS. It timestamps every edge of the ZCD signal with
  * a free-running 1 MHz timer (TIM2) and keeps statistics, so that what the
  * BM1Z012FJ zero-cross IC really outputs on THIS board is MEASURED instead
  * of assumed. The converter commutation and the RMS windows get built on
  * the result.
  *
  * Read g_zc in the debugger (Live Expressions) - see the "How to read"
  * table at the bottom of this file.
  ******************************************************************************
  */

#ifndef SLAVE_ZC_H
#define SLAVE_ZC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32g0xx_hal.h"

/* Timer clock feeding TIM2 (APB timer clock after the PLL = 64 MHz). */
#define SLAVEZC_TIMER_CLK_HZ   64000000UL

typedef struct
{
    /* Raw edge bookkeeping (timestamps are TIM2 microseconds, free-running) */
    volatile uint32_t rise_count;        /* total rising edges seen            */
    volatile uint32_t fall_count;        /* total falling edges seen           */
    volatile uint32_t t_rise_us;         /* timestamp of the latest rising edge  */
    volatile uint32_t t_fall_us;         /* timestamp of the latest falling edge */
    volatile uint32_t t_last_us;         /* timestamp of the latest edge of any kind */
    volatile uint8_t  level;             /* pin level read at the latest edge  */

    /* Periods */
    volatile uint32_t period_rise_us;    /* rising  -> next rising   */
    volatile uint32_t period_fall_us;    /* falling -> next falling  */

    /* Widths (how long the line stayed in each state) */
    volatile uint32_t width_hi_us;       /* rising  -> next falling  = time HIGH */
    volatile uint32_t width_lo_us;       /* falling -> next rising   = time LOW  */
    volatile uint32_t width_hi_min_us;   /* smallest / largest seen since reset  */
    volatile uint32_t width_hi_max_us;
    volatile uint32_t width_lo_min_us;
    volatile uint32_t width_lo_max_us;

    /* Edges counted in the last COMPLETE second (updated at the first edge
     * after each whole second, so it can go stale if edges stop - use
     * SlaveZC_AgeUs() to detect that). */
    volatile uint32_t rise_per_s;
    volatile uint32_t fall_per_s;
} SlaveZC_t;

extern SlaveZC_t g_zc;
extern TIM_HandleTypeDef htim2;          /* 1 MHz free-running timebase */

/* Starts TIM2 (1 MHz, 32-bit) and enables the PB0 EXTI interrupt.
 * Requires PB0 to be configured as GPIO_MODE_IT_RISING_FALLING in
 * MX_GPIO_Init() before this is called. */
void     SlaveZC_Init(void);

/* Clears all counters/statistics (min values to 0xFFFFFFFF, max to 0). */
void     SlaveZC_ResetStats(void);

/* Hardware-independent core: feed it one edge with its timestamp.
 * Called by the EXTI callbacks; also used directly by the host-side test. */
void     SlaveZC_OnEdge(uint32_t now_us, uint8_t rising);

/* Current TIM2 time in microseconds (wraps every ~71.6 minutes). */
uint32_t SlaveZC_NowUs(void);
void     SlaveZC_FltIrqFiltered(void);  /* v0.9.62: the FLT interrupt with the glitch filter (SLAVE_FLT_FILTER_US > 0), see slave_flt_filter.h */
uint32_t SlaveZC_FltIgnored(void);      /* FLT spikes ignored by the filter since power-up */
void     SlaveZC_FltIrqCbc(void);       /* v0.9.66: the FLT interrupt with the cycle-by-cycle limit (SLAVE_FLT_CBC), see slave_flt_cbc.h */
void     SlaveZC_Tim1UpdateIrq(void);   /* v0.9.66: TIM1 update (PWM period start) for the limit; nothing without SLAVE_FLT_CBC */

/* Microseconds since the latest edge of any kind; 0xFFFFFFFF if none yet. */
uint32_t SlaveZC_AgeUs(void);

/* Rate of RISING edges in Hz x100 (0 if no valid period yet).
 * Mains frequency = this value if the signal has ONE rising edge per mains
 * cycle, or half of it if it has one per half-cycle. Which of the two it is
 * is exactly what this characterisation step is meant to establish. */
uint32_t SlaveZC_RiseRateHzX100(void);

#ifdef __cplusplus
}
#endif

#endif /* SLAVE_ZC_H */

/* ============================================================================
 * HOW TO READ g_zc - what each ZCD signal shape looks like
 * ----------------------------------------------------------------------------
 *  Mains 50 Hz.  rise_per_s / fall_per_s are edges per second.
 *
 *  (A) One short pulse at EVERY zero-crossing (ACOUT idles high, pulses low):
 *        rise_per_s = 100, fall_per_s = 100
 *        width_lo ~ the pulse width (e.g. a few hundred us), width_hi ~ 9.5 ms
 *        period_rise = period_fall ~ 10000 us
 *
 *  (B) Square-ish wave, one HIGH half and one LOW half per mains cycle:
 *        rise_per_s = 50, fall_per_s = 50
 *        width_hi ~ width_lo ~ 10000 us,   period_rise ~ 20000 us
 *
 *  (C) One short pulse per mains CYCLE (only one polarity of crossing):
 *        rise_per_s = 50, fall_per_s = 50, one width tiny and the other ~ 20 ms
 *
 *  Anything else (very different counts, or min/max widths far apart) points
 *  at noise/glitches on the line - look at width_*_min/max.
 * ========================================================================== */
