/* The start-up transformer check against simulated start-ups: a real profile from the real sweep, then the regulator's ramp in boost on the same transformer, a changed one, a reversed one, with sensor offset, noise and input jumps. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "slave_tunechk.h"
static int tot = 0, bad = 0;
#define CHECK(c, ...) do { tot++; if (!(c)) { bad++; printf("      FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
static uint32_t rs = 4242U;
static double gauss(void) { double a = 0.0; int i; for (i = 0; i < 3; i++) { rs = rs * 1664525U + 1013904223U; a += (double)((rs >> 8) & 0xFFFFFFU) / 8388608.0 - 1.0; } return a / 1.732; }
static double urand(double lo, double hi) { rs = rs * 1664525U + 1013904223U; return lo + (hi - lo) * (double)((rs >> 8) & 0xFFFFFFU) / 16777216.0; }

/* a real profile: the real sweep against a transformer of gain k at 160 V in */
static TuneProfile_t make_profile(double k)
{
    TuneIn_t in; TuneOut_t out; TuneFrame_t f; TuneProfile_t p; uint32_t t = 100000U, t0 = t; double inj = 0.0, dead = 40.0; uint16_t req = 0U; const double vin = 160.0;
    memset(&in, 0, sizeof in); memset(&p, 0, sizeof p);
    (void)Tune_Start(120.0f, t);
    while ((t - t0) < 120000U)
    {
        double eff = ((double)req > dead) ? ((double)req - dead) : 0.0;
        t += 10U; inj += (k * (double)Tune_RefG((float)vin) * eff - inj) * (1.0 - exp(-0.01 / 0.3));
        in.vin = (float)(vin + 0.05 * gauss()); in.vout = (float)(vin + 0.2 + inj + 0.1 * gauss()); in.meas_valid = 1U; in.vin_in_window = 1U; in.maxc_v = 120.0f; in.hw_cap_ticks = 2752U;
        Tune_Service(t, &in, &out); req = out.request_ticks; while (Tune_TakeFrame(&f)) { }
        if (out.done) break;
    }
    (void)Tune_GetProfile(&p); return p;
}

/* a start-up in boost: the regulator ramps the duty at `slope` ticks/s to the duty that gives `target_dv` on the REAL plant (gain k * the reference curve), holds. Returns the first verdict (and when). */
typedef struct { double k, vin, target_dv, slope, offset, noise, jump_at_s, jump_v; uint8_t boost; uint32_t secs; } Run_t;
static uint8_t start_run(const TuneProfile_t *prof, const Run_t *r, double *t_verdict)
{
    TuneChk_t c; uint32_t t = 200000U, k; double d = 0.0, inj = 0.0, dead = 40.0, vin = r->vin, g = r->k * (double)Tune_RefG((float)r->vin), dtarget = dead + ((g != 0.0) ? fabs(r->target_dv / g) : 0.0);
    TuneChk_Init(&c, prof, t); *t_verdict = -1.0;
    for (k = 0U; k < r->secs * 50U; k++)
    {
        double eff, vout; uint8_t v;
        t += 20U; if (d < dtarget) { d += r->slope * 0.02; if (d > dtarget) d = dtarget; }
        if ((r->jump_at_s > 0.0) && (fabs((double)(t - 200000U) / 1000.0 - r->jump_at_s) < 0.011)) vin += r->jump_v;
        eff = (d > dead) ? (d - dead) : 0.0; inj += (g * eff - inj) * (1.0 - exp(-0.02 / 0.3));
        vout = vin + r->offset + inj + r->noise * gauss();
        v = TuneChk_Update(&c, t, d, r->boost, (float)(vin + 0.03 * gauss()), (float)vout);
        if (v != TUNECHK_CONT) { *t_verdict = (double)(t - 200000U) / 1000.0; return v; }
    }
    return TUNECHK_CONT;
}
static Run_t base(void) { Run_t r; memset(&r, 0, sizeof r); r.k = 1.0; r.vin = 190.0; r.target_dv = 40.0; r.slope = 320.0; r.noise = 0.05; r.boost = 1U; r.secs = 40U; return r; }

int main(void)
{
    const TuneProfile_t P = make_profile(1.0); double tv; Run_t r; uint8_t v; int i, fa = 0, miss = 0, n_ok = 0, n_bad = 0;
    CHECK(Tune_ProfileValid(&P) != 0U && P.cap_ticks == 2752U, "a real profile from the real sweep: cap %u, gain scale %u", (unsigned)P.cap_ticks, (unsigned)P.kib_m);
    r = base(); v = start_run(&P, &r, &tv);
    CHECK(v == TUNECHK_OK && tv < 8.0, "the SAME transformer, 190 V in, 40 V of correction: OK after %.1f s", tv);
    r = base(); r.k = 0.85; v = start_run(&P, &r, &tv); CHECK(v == TUNECHK_OK, "a gain 15 %% lower is still within the tolerance: OK (%u)", (unsigned)v);
    r = base(); r.k = 1.15; v = start_run(&P, &r, &tv); CHECK(v == TUNECHK_OK, "a gain 15 %% higher: OK (%u)", (unsigned)v);
    r = base(); r.k = 1.6; v = start_run(&P, &r, &tv); CHECK(v == TUNECHK_MISMATCH && tv < 10.0, "a transformer with 1.6 times the gain: MISMATCH after %.1f s", tv);
    r = base(); r.k = 0.55; v = start_run(&P, &r, &tv); CHECK(v == TUNECHK_MISMATCH && tv < 12.0, "a transformer with 0.55 of the gain: MISMATCH after %.1f s", tv);
    r = base(); r.k = -1.0; v = start_run(&P, &r, &tv); CHECK(v == TUNECHK_FLIP, "a reversed transformer: FLIP (%u)", (unsigned)v);
    r = base(); r.offset = 0.9; r.noise = 0.2; v = start_run(&P, &r, &tv); CHECK(v == TUNECHK_OK, "0.9 V of sensor offset and 0.2 V of noise, the same transformer: still OK (the offset is measured and taken off) (%u)", (unsigned)v);
    r = base(); r.offset = 0.9; r.noise = 0.2; r.k = 1.6; v = start_run(&P, &r, &tv); CHECK(v == TUNECHK_MISMATCH, "...and a different transformer with the same offset and noise: MISMATCH (%u)", (unsigned)v);
    r = base(); r.k = 1.6; r.boost = 0U; v = start_run(&P, &r, &tv); CHECK(v == TUNECHK_CONT, "in BUCK nothing is judged (the profile describes the boost injection): no verdict");
    r = base(); r.k = 1.6; r.target_dv = 1.5; r.secs = 30U; v = start_run(&P, &r, &tv); CHECK(v == TUNECHK_CONT, "a start that needs only 1.5 V of correction is too small to judge: no verdict, no false alarm");
    r = base(); r.jump_at_s = 3.0; r.jump_v = 5.0; v = start_run(&P, &r, &tv); CHECK(v == TUNECHK_OK, "a 5 V jump of the input in the middle of the ramp does not cause a false alarm: OK (%u)", (unsigned)v);
    r = base(); r.k = 1.0; r.slope = 1500.0; v = start_run(&P, &r, &tv); CHECK(v == TUNECHK_OK, "a fast ramp (1500 ticks/s): OK, the lagging output is allowed for (%u after %.1f s)", (unsigned)v, tv);
    r = base(); r.slope = 100.0; v = start_run(&P, &r, &tv); CHECK(v == TUNECHK_OK, "a slow ramp (100 ticks/s): OK (%u)", (unsigned)v);
    { TuneChk_t c; TuneChk_Init(&c, 0, 0U); CHECK(TuneChk_Update(&c, 20U, 1000.0f, 1U, 190.0f, 230.0f) == TUNECHK_CONT, "no profile: the check stays silent"); }
    /* many random start-ups: never a false alarm for a gain within +-20 %, always an alarm for a gain 1.5 times or more (or below 0.6) */
    for (i = 0; i < 300; i++)
    {
        r = base(); r.vin = urand(150.0, 235.0); r.target_dv = urand(8.0, 55.0); r.slope = urand(150.0, 900.0); r.offset = urand(-1.0, 1.0); r.noise = urand(0.03, 0.25);
        if ((i & 1) == 0) { r.k = urand(0.8, 1.2); v = start_run(&P, &r, &tv); n_ok++; if (v == TUNECHK_MISMATCH || v == TUNECHK_FLIP) fa++; }
        else { r.k = (i & 2) ? urand(1.5, 2.6) : urand(0.3, 0.6); v = start_run(&P, &r, &tv); n_bad++; if (v != TUNECHK_MISMATCH) miss++; }
    }
    CHECK(fa == 0, "%d random start-ups with the same transformer (gain +-20 %%, random input, correction, ramp, offset, noise): %d false alarms", n_ok, fa);
    CHECK(miss == 0, "%d random start-ups with a clearly different gain (1.5 to 2.6 times, or 0.3 to 0.6): %d not detected", n_bad, miss);
    printf("  start-up transformer check: a real profile, the same / a changed / a reversed transformer, offset, noise, jumps, ramps, 300 random start-ups\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", tot, bad, bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
