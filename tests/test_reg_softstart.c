/* The regulator's SOFT-START ramp (slave_reg.c): the setpoint starts at the present input and slides to the target. Slopes and the slow time are per-instance settings
 * (Reg_t.ss_*), set by Reg_Init to the OLD firmware's constants and overridable by the controller (slave_config.h). This test checks, for the old values and for a faster set,
 * upward (boost) and downward (buck): the defaults ARE the old constants, the first phase runs at the slow slope, the ramp reaches the target at the time an independent
 * double-precision model predicts, the slope never exceeds the largest configured one, and it ends exactly on the target. */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "slave_reg.h"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

typedef struct { double slow, fast, fin; unsigned slow_ms; } Ramp_t;

/* independent model of the ramp */
static double model_time(double vin, double target, Ramp_t rp)
{
    double sp = vin, t = 0.0;
    while (fabs(sp - target) > 0.05 && t < 60.0)
    {
        double diff = fabs(sp - target);
        double slope = (t * 1000.0 < (double)rp.slow_ms) ? rp.slow : ((diff <= 3.0) ? rp.fin : rp.fast);
        double step = slope * 0.001;
        sp += (sp < target) ? step : -step;
        t += 0.001;
    }
    return t;
}

static void run_one(double vin, Ramp_t rp, const char *name)
{
    Reg_t r; int i; double sp_prev = vin, max_slope = 0.0, slow_slope_seen = 0.0, t_reach = -1.0, target = 230.0, expect = model_time(vin, target, rp);
    double maxcfg = fmax(rp.slow, fmax(rp.fast, rp.fin));
    Reg_Init(&r, (float)target, (float)vin, (float)vin, 1000U);
    r.ss_slow_slope_vps = (float)rp.slow; r.ss_fast_slope_vps = (float)rp.fast; r.ss_final_slope_vps = (float)rp.fin; r.ss_slow_time_ms = rp.slow_ms;
    for (i = 0; i < 12000; i++)
    {
        double slope;
        Reg_Service(&r, 1000U + (uint32_t)i, (float)vin, (float)vin, 1U);
        slope = fabs((double)r.sp_v - sp_prev) / 0.001;
        if (i > 3 && slope > max_slope) max_slope = slope;
        if (i == 400) slow_slope_seen = slope;                               /* inside the slow phase (if the slow phase is longer than 0.4 s) */
        if (t_reach < 0.0 && fabs((double)r.sp_v - target) <= 0.05) t_reach = i * 0.001;
        sp_prev = (double)r.sp_v;
    }
    CHECK(t_reach >= 0.0 && fabs(t_reach - expect) <= 0.12, "%s, %.0f V: the ramp reaches the target at %.2f s, the model says %.2f s", name, vin, t_reach, expect);
    CHECK(max_slope <= maxcfg * 1.03 + 0.2, "%s, %.0f V: the setpoint never moves faster than the largest configured slope (%.2f V/s of %.2f)", name, vin, max_slope, maxcfg);
    if (rp.slow_ms > 500U && fabs(vin - target) > rp.slow * 0.5) CHECK(fabs(slow_slope_seen - rp.slow) <= rp.slow * 0.05 + 0.2, "%s, %.0f V: the first phase runs at the slow slope (%.2f V/s, configured %.2f)", name, vin, slow_slope_seen, rp.slow);
    CHECK(fabs((double)r.sp_v - target) <= 0.06, "%s, %.0f V: the ramp ends on the target (%.3f V)", name, vin, (double)r.sp_v);
}

int main(void)
{
    const double ins[] = { 205.0, 215.0, 225.0, 245.0, 260.0 };
    Ramp_t old_rp = { 3.0, 10.0, 5.0, 2000U }, fast = { 10.0, 30.0, 10.0, 500U };
    Reg_t r; int i;
    Reg_Init(&r, 230.0f, 205.0f, 205.0f, 0U);
    CHECK(r.ss_slow_slope_vps == REG_SOFTSTART_SLOW_SLOPE_VPS && r.ss_fast_slope_vps == REG_SOFTSTART_FAST_SLOPE_VPS &&
          r.ss_final_slope_vps == REG_SOFTSTART_FINAL_SLOPE_VPS && r.ss_slow_time_ms == REG_SOFTSTART_SLOW_TIME_MS, "Reg_Init sets the OLD firmware's soft-start constants");
    CHECK(REG_SOFTSTART_SLOW_SLOPE_VPS == 3.0f && REG_SOFTSTART_FAST_SLOPE_VPS == 10.0f && REG_SOFTSTART_FINAL_SLOPE_VPS == 5.0f && REG_SOFTSTART_SLOW_TIME_MS == 2000U, "the old constants are 3 / 10 / 5 V/s and 2000 ms");
    for (i = 0; i < 5; i++) { run_one(ins[i], old_rp, "old firmware ramp"); run_one(ins[i], fast, "fast ramp"); }
    printf("  soft-start ramp: the old 3 / 10 / 5 V/s ramp takes %.2f s from 205 V, the fast 10 / 30 / 10 V/s ramp %.2f s (independent model)\n", model_time(205.0, 230.0, old_rp), model_time(205.0, 230.0, fast));
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
