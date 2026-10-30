/**
  ******************************************************************************
  * @file    slave_meas_due.h
  * @brief   v0.9.71: when SlaveMeas_Service computes a new measurement. HEADER ONLY (no project change), PURE C: the PC tests run it.
  *
  * THE BUG IT FIXES (F10 "measurement stale" every 71.6 minutes, v0.9.x up to v0.9.70): the 100 ms warm-up after power-up was checked on EVERY call as
  * (now - start) < MEAS_WARMUP_US on TIM2, the free-running 32-bit microsecond timer. TIM2 wraps every 2^32 us = 71.6 minutes, so that difference fell below
  * 100 ms again once per wrap: for 100 ms no measurement was computed, the controller saw none for more than CTRL_MEAS_STALE_MS (100 ms) and tripped F10 while
  * running. Bench (single phase, v0.9.70): F10 at 13:34, 14:45 and 15:57, the main loop's longest pass 4 ms all along.
  * Now the warm-up is checked only until it has passed once.
  ******************************************************************************
  */
#ifndef SLAVE_MEAS_DUE_H
#define SLAVE_MEAS_DUE_H

#include <stdint.h>

typedef struct
{
    uint32_t start_us;    /* TIM2 at the start of the measurement */
    uint32_t last_us;     /* the last call that passed the update period */
    uint8_t  warm;        /* the warm-up has passed (never checked again) */
} MeasClock_t;

static inline void MeasDue_Init(MeasClock_t *c, uint32_t now_us) { c->start_us = now_us; c->last_us = now_us; c->warm = 0U; }

/* 1 = compute a new measurement now. period_us: the update period; warmup_us: nothing for this long after MeasDue_Init. Wrap-safe for any run time. */
static inline uint8_t MeasDue(MeasClock_t *c, uint32_t now_us, uint32_t period_us, uint32_t warmup_us)
{
    if ((uint32_t)(now_us - c->last_us) < period_us) return 0U;
    c->last_us = now_us;
    if (c->warm == 0U)
    {
        if ((uint32_t)(now_us - c->start_us) < warmup_us) return 0U;
        c->warm = 1U;
    }
    return 1U;
}

#endif /* SLAVE_MEAS_DUE_H */
