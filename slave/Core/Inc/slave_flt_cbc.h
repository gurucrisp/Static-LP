/**
  ******************************************************************************
  * @file    slave_flt_cbc.h
  * @brief   v0.9.66: the FLT cycle-by-cycle limit (SLAVE_FLT_CBC), as a dsPIC's PWM fault input in current-limit mode. HEADER ONLY (no project change), PURE C.
  *
  * A FLT edge while the unit RUNS forces only the inverter PWM inactive (high sides off, low sides on: the primary stays shorted); the converter keeps running.
  * At every PWM period start (TIM1 update, 50 us) the pin is read: released = the PWM resumes; still asserted = it stays off. A REAL fault escalates to the
  * full FLT shutdown (SlaveCtrl_OnFltIrq: F02, the low-sides-on unwinding of v0.9.65):
  *   - FLT still asserted at FLT_CBC_MAX_HOLD consecutive period starts (1 ms), or
  *   - more than FLT_CBC_MAX_EVENTS limited edges within FLT_CBC_WINDOW_MS (10 ms, half a mains cycle): a fault chattering on its threshold.
  * Outside RUN a FLT edge is the full shutdown at once, as before.
  ******************************************************************************
  */
#ifndef SLAVE_FLT_CBC_H
#define SLAVE_FLT_CBC_H

#include <stdint.h>

#ifndef SLAVE_FLT_CBC
#define SLAVE_FLT_CBC 0
#endif
#define FLT_CBC_MAX_HOLD     20U     /* PWM period starts with FLT still asserted before it is a fault (20 x 50 us = 1 ms) */
#define FLT_CBC_MAX_EVENTS   40U     /* limited edges allowed within one window */
#define FLT_CBC_WINDOW_MS    10U

typedef struct
{
    uint8_t  active;      /* the PWM is held off, waiting for a period start with FLT released */
    uint8_t  hold;        /* period starts seen with FLT still asserted */
    uint8_t  win_n;       /* limited edges in this window */
    uint32_t win_ms;      /* when the window began */
    uint32_t total;       /* limited edges since power-up (diagnostics) */
} FltCbc_t;

#define FLT_CBC_LIMIT   1U        /* FltCbc_OnEdge: keep the PWM off, resume at a period start */
#define FLT_CBC_TRIP    0U        /*                 a real fault: the full FLT shutdown */
#define FLT_CBC_HOLD    0U        /* FltCbc_OnPeriod: stay off */
#define FLT_CBC_RESUME  1U        /*                   the PWM resumes */
#define FLT_CBC_FAULT   2U        /*                   held too long: the full FLT shutdown */

/* A FLT edge while running. */
static inline uint8_t FltCbc_OnEdge(FltCbc_t *c, uint32_t now_ms)
{
    if ((uint32_t)(now_ms - c->win_ms) >= FLT_CBC_WINDOW_MS) { c->win_ms = now_ms; c->win_n = 0U; }
    if (c->win_n >= FLT_CBC_MAX_EVENTS) { c->active = 0U; return FLT_CBC_TRIP; }
    c->win_n++; c->total++;
    if (c->active == 0U) c->hold = 0U;          /* a new edge while already held does not restart the hold count (chatter cannot keep it alive) */
    c->active = 1U;
    return FLT_CBC_LIMIT;
}

/* At a PWM period start while active: asserted = FLT read asserted now. */
static inline uint8_t FltCbc_OnPeriod(FltCbc_t *c, uint8_t asserted)
{
    if (c->active == 0U) return FLT_CBC_RESUME;
    if (asserted != 0U)
    {
        c->hold++;
        if (c->hold >= FLT_CBC_MAX_HOLD) { c->active = 0U; return FLT_CBC_FAULT; }
        return FLT_CBC_HOLD;
    }
    c->active = 0U;
    return FLT_CBC_RESUME;
}

static inline void FltCbc_Reset(FltCbc_t *c) { c->active = 0U; c->hold = 0U; }      /* the unit left RUN / a fault: nothing is held any more (the counters stay) */

#endif /* SLAVE_FLT_CBC_H */
