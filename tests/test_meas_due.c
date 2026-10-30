/* v0.9.71: when the slave computes a measurement (slave/Core/Inc/slave_meas_due.h). The F10 bug: the warm-up re-checked at every TIM2 wrap (71.6 min) left
 * 100 ms without a measurement. A main loop calling every 1 ms (with jitter) for 4 hours, from several start values of the 32-bit microsecond timer:
 * no gap above CTRL_MEAS_STALE_MS (100 ms) after the warm-up, the first measurement after the warm-up, one per 10 ms. The OLD rule must show the gaps.
 * gcc -std=c99 -O2 -Wall -Wextra -Islave/Core/Inc tests/test_meas_due.c */
#include <stdio.h>
#include <stdint.h>
#include "slave_meas_due.h"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define PERIOD   10000U
#define WARMUP   100000U
#define STALE_US 100000U

static uint32_t old_start, old_last;
static uint8_t old_due(uint32_t now)       /* the rule of v0.9.70 and before */
{
    if ((uint32_t)(now - old_last) < PERIOD) return 0U;
    old_last = now;
    if ((uint32_t)(now - old_start) < WARMUP) return 0U;
    return 1U;
}

/* runs `hours` from start value t0; returns the number of gaps between measurements longer than STALE_US after the first one */
static unsigned run(uint32_t t0, double hours, int use_old, uint64_t *first_after, uint64_t *n_meas, uint64_t *max_gap)
{
    MeasClock_t c; uint64_t t, end = (uint64_t)(hours * 3600e6), last = 0; unsigned gaps = 0U; int have = 0; uint32_t lcg = 77U;
    MeasDue_Init(&c, t0); old_start = t0; old_last = t0;
    *first_after = 0; *n_meas = 0; *max_gap = 0;
    for (t = 0; t < end; )
    {
        uint32_t now = (uint32_t)(t0 + t);
        uint8_t d = use_old ? old_due(now) : MeasDue(&c, now, PERIOD, WARMUP);
        if (d)
        {
            if (!have) { *first_after = t; have = 1; }
            else { uint64_t g = t - last; if (g > *max_gap) *max_gap = g; if (g > STALE_US) gaps++; }
            last = t; (*n_meas)++;
        }
        lcg = lcg * 1664525U + 1013904223U;
        t += 700U + (lcg >> 22);                 /* a main-loop pass of 0.7 to 1.7 ms */
    }
    return gaps;
}

int main(void)
{
    const uint32_t starts[4] = { 0U, 12345678U, 0xFFFFFFFFU - 50000U, 0x80000000U };
    int i; uint64_t fa, nm, mg; unsigned g;
    for (i = 0; i < 4; i++)
    {
        g = run(starts[i], 4.0, 0, &fa, &nm, &mg);
        CHECK(g == 0U, "start %08X, 4 h (3 TIM2 wraps): no gap above 100 ms (%u gaps, longest %llu us)", (unsigned)starts[i], g, (unsigned long long)mg);
        CHECK(mg <= PERIOD + 1800U, "start %08X: the longest gap %llu us is one period plus one loop pass", (unsigned)starts[i], (unsigned long long)mg);
        CHECK(fa >= WARMUP && fa <= WARMUP + PERIOD + 1800U, "start %08X: the first measurement after the warm-up (%llu us)", (unsigned)starts[i], (unsigned long long)fa);
        CHECK(nm >= 1300000ULL && nm <= 1440001ULL, "start %08X: about one measurement per 10 ms plus a loop pass (%llu in 4 h)", (unsigned)starts[i], (unsigned long long)nm);
        g = run(starts[i], 4.0, 1, &fa, &nm, &mg);
        CHECK(g >= 2U, "start %08X: the OLD rule shows the bug: %u gaps above 100 ms in 4 h (longest %llu us)", (unsigned)starts[i], g, (unsigned long long)mg);
    }
    printf("  measurement timing: no gap above 100 ms over 3 TIM2 wraps from 4 start values; the old rule leaves one every 71.6 min\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
