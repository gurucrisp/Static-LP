/* v0.9.62: the FLT glitch filter (slave/Core/Inc/slave_flt_filter.h): the wait on a down-counting SysTick (wrap, a stopped counter), the judge (a real trip,
 * a spike, the burst limit, a new millisecond). gcc -std=c99 -Wall -Wextra -Islave/Core/Inc tests/test_flt_filter.c */
#include <stdio.h>
#include <stdint.h>
#define SLAVE_FLT_FILTER_US 1U
#include "slave_flt_filter.h"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* a SysTick model: LOAD = 63999 (1 ms at 64 MHz), counting DOWN by `step` cycles per read; `stopped` = it never moves */
static uint32_t st_val, st_step, st_reads; static int st_stopped;
static uint32_t st_read(void)
{
    uint32_t v = st_val; st_reads++;
    if (!st_stopped) { if (st_val >= st_step) st_val -= st_step; else st_val = 64000U - (st_step - st_val); }
    return v;
}
static uint32_t waited(uint32_t start, uint32_t step, uint32_t window, uint32_t cap)
{   /* cycles that passed, as the model counts them */
    st_val = start; st_step = step; st_reads = 0U; st_stopped = 0;
    FltFilter_Wait(st_read, 64000U, window, cap);
    return (st_reads - 1U) * step;
}

int main(void)
{
    FltFilter_t f = { 0U, 0U, 0U }; uint32_t c, k; int real = 0;
    /* ---- the wait ---- */
    c = waited(50000U, 6U, 64U, 80U);
    CHECK(c >= 64U && c < 64U + 2U * 6U, "1 us at 64 MHz: waits at least 64 cycles and stops right after (%u)", (unsigned)c);
    c = waited(30U, 6U, 64U, 80U);
    CHECK(c >= 64U && c < 64U + 2U * 6U, "across the SysTick wrap (start at 30): still 64 cycles (%u)", (unsigned)c);
    c = waited(0U, 6U, 128U, 144U);
    CHECK(c >= 128U && c < 128U + 2U * 6U, "2 us, starting at 0 (the reload moment): %u cycles", (unsigned)c);
    st_val = 1000U; st_stopped = 1; st_reads = 0U; FltFilter_Wait(st_read, 64000U, 64U, 80U);
    CHECK(st_reads == 81U, "a SysTick that does not count: the loop ends at its cap (%u reads), it never holds the outputs on", (unsigned)st_reads);
    /* ---- the judge ---- */
    CHECK(FltFilter_Judge(&f, 1U, 10U) == 1U && f.ignored == 0U, "FLT still asserted after the window: a REAL trip");
    CHECK(FltFilter_Judge(&f, 0U, 10U) == 0U && f.ignored == 1U, "released: a spike, ignored and counted");
    for (k = 0U; k < 3U; k++) (void)FltFilter_Judge(&f, 0U, 10U);
    CHECK(f.ignored == 4U && f.burst_n == FLT_FILTER_BURST_MAX, "four spikes in the same millisecond are ignored (%u)", (unsigned)f.ignored);
    CHECK(FltFilter_Judge(&f, 0U, 10U) == 1U && f.ignored == 4U, "the fifth in that millisecond is a TRIP (a chattering fault is not filtered away)");
    CHECK(FltFilter_Judge(&f, 0U, 10U) == 1U, "and so is every further one in it");
    CHECK(FltFilter_Judge(&f, 0U, 11U) == 0U && f.burst_n == 1U, "a new millisecond starts a new count");
    CHECK(FltFilter_Judge(&f, 1U, 11U) == 1U, "asserted is always a trip, whatever the count");
    /* a long run of isolated spikes, one per millisecond: none trips */
    for (k = 0U; k < 1000U; k++) if (FltFilter_Judge(&f, 0U, 100U + k) != 0U) real++;
    CHECK(real == 0, "1000 isolated spikes, one per millisecond: all ignored (%d trips)", real);
    printf("  FLT glitch filter: the wait (wrap, stopped SysTick capped), real trips, spikes, the burst limit\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
