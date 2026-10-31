/* v0.9.76: the mains period from the zero-cross rate and the RMS window, 37 to 66 Hz. The two functions are CUT OUT of the current slave_meas.c by the runner
 * (old_snippets/meas_freq.inc), so this tests the real code. A rejected period makes the measurement assume 50 Hz and use a 20 ms window: at 40 Hz (25 ms) that is
 * not a whole cycle and the voltage readings would ripple. v0.9.75 and before accepted at most 25 000 us: exactly 40 Hz. */
#include <stdio.h>
#include <stdint.h>
#define MEAS_SAMPLE_PERIOD_US 50U
#define MEAS_WINDOW_MAX       800U
#include "old_snippets/meas_freq.inc"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

int main(void)
{
    static const double hz[] = { 37.5, 40.0, 45.0, 50.0, 55.0, 60.0, 65.0 };
    unsigned i; int j;
    for (i = 0U; i < sizeof hz / sizeof hz[0]; i++)
    {
        const uint32_t per = (uint32_t)(1e6 / hz[i] + 0.5);
        for (j = -300; j <= 300; j += 150)
        {   /* one rising edge per cycle, with up to 300 us of edge jitter */
            const uint32_t p = (uint32_t)((int32_t)per + j);
            CHECK(SlaveMeas_MainsPeriodFromZc(p) == (float)p, "%.1f Hz, a rising edge per cycle, %d us jitter: period %u us accepted", hz[i], j, (unsigned)p);
            CHECK(SlaveMeas_MainsPeriodFromZc(p / 2U) == 2.0f * (float)(p / 2U), "%.1f Hz, a rising edge per HALF-cycle (%u us): the period is twice that", hz[i], (unsigned)(p / 2U));
        }
        { uint32_t n = SlaveMeas_WindowFrames((float)per);
          CHECK(n * 50U >= per - 50U && n * 50U <= per + 50U && n <= MEAS_WINDOW_MAX, "%.1f Hz: the RMS window is one whole cycle (%u frames = %u us)", hz[i], (unsigned)n, (unsigned)(n * 50U)); }
    }
    CHECK(SlaveMeas_MainsPeriodFromZc(14999U) == 0.0f && SlaveMeas_MainsPeriodFromZc(27001U) == 0.0f && SlaveMeas_MainsPeriodFromZc(7499U) == 0.0f, "outside 37 to 66 Hz: rejected");
    CHECK(SlaveMeas_MainsPeriodFromZc(13600U) == 0.0f, "13.6 ms is neither a cycle nor a half-cycle in range: rejected");
    printf("  mains period and RMS window: 37.5 to 65 Hz with 300 us of jitter, a rising edge per cycle or per half-cycle, a whole-cycle window\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
