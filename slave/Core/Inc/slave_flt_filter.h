/**
  ******************************************************************************
  * @file    slave_flt_filter.h
  * @brief   v0.9.62: the FLT glitch filter (SLAVE_FLT_FILTER_US). HEADER ONLY (no project change), PURE C: the PC tests run it.
  *
  * Why: on the bench F02 (FLT, the short-circuit comparators U10) came when the contactor closed onto a load and even when a heavy load NEARBY was switched,
  * with a 1k pull-up and 1 nF on the FLT pin; the converter current cannot be 61 A then: noise reaching the comparator inputs.
  *
  * What: the FLT interrupt WAITS SLAVE_FLT_FILTER_US (measured on the CPU clock, SysTick) and reads the pin again. Still asserted = a real trip: the outputs go
  * off at once and everything follows as before. Released = a spike shorter than the window: ignored and counted. More than FLT_FILTER_BURST_MAX ignored spikes
  * within one millisecond count as REAL (a fault that chatters on its threshold must not be filtered away).
  *
  * THE COST: a real short circuit is switched off SLAVE_FLT_FILTER_US later. At most 2 us is accepted (the IGBT's short-circuit withstand time is about 8 us and
  * the comparator, the firmware and the gate driver already use about 3.5 to 4 us; NOT MEASURED on the hardware: slave_config_SCOPE_flt_latency_NO_MAINS.h).
  * 0 = no filter: the outputs go off on the edge, the old behaviour.
  *
  * The wait has an iteration cap: a SysTick that does not count can never hold the IGBTs on.
  ******************************************************************************
  */
#ifndef SLAVE_FLT_FILTER_H
#define SLAVE_FLT_FILTER_H

#include <stdint.h>

#ifndef SLAVE_FLT_FILTER_US
#define SLAVE_FLT_FILTER_US 0U
#endif
#if (SLAVE_FLT_FILTER_US > 2U)
#error "SLAVE_FLT_FILTER_US above 2 us: the IGBT short-circuit margin is gone (see slave_flt_filter.h)"
#endif
#define FLT_FILTER_BURST_MAX  4U      /* ignored spikes allowed within one millisecond; the next one is a trip */

typedef struct
{
    uint32_t burst_ms;     /* the millisecond the burst count belongs to */
    uint8_t  burst_n;      /* ignored spikes in it */
    uint32_t ignored;      /* all ignored spikes since power-up (diagnostics) */
} FltFilter_t;

/* Wait until window_cyc CPU cycles have passed on a DOWN-counting SysTick (val() reads the counter, reload = LOAD + 1). The loop runs at most cap times. */
static inline void FltFilter_Wait(uint32_t (*val)(void), uint32_t reload, uint32_t window_cyc, uint32_t cap)
{
    const uint32_t start = val();
    uint32_t n;
    for (n = 0U; n < cap; n++)
    {
        const uint32_t now = val();
        const uint32_t el = (start >= now) ? (start - now) : ((start + reload) - now);
        if (el >= window_cyc) break;
    }
}

/* After the wait: asserted = FLT read asserted again (any of the reads). 1 = REAL, switch off now; 0 = a spike, ignore it. */
static inline uint8_t FltFilter_Judge(FltFilter_t *f, uint8_t asserted, uint32_t now_ms)
{
    if (asserted != 0U) return 1U;
    if (now_ms != f->burst_ms) { f->burst_ms = now_ms; f->burst_n = 0U; }
    if (f->burst_n >= FLT_FILTER_BURST_MAX) return 1U;
    f->burst_n++;
    f->ignored++;
    return 0U;
}

#endif /* SLAVE_FLT_FILTER_H */
