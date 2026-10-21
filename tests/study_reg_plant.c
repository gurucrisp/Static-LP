/* Regulator (slave_reg.c, the REAL module) against the plant MEASURED on the bench at stage 2 (v0.7.1, effective 64-tick = 1.0 us pulse):
 *   injected change in (Vout - Vin) at 64 effective ticks: about 0.7 V at 110 V, 1.9 V at 200 V, 2.1 V at 250 V  ->  g = 0.011 / 0.030 / 0.033 V per effective tick.
 * Plant structure: the duty is applied once per half-cycle (10 ms); the CCR is dead time + effective ticks (zero stays zero), a pulse at or below the dead time
 * outputs nothing; Vout = Vin +/- g(Vin) x effective ticks (boost adds, buck subtracts); the controller sees an RMS over 20 ms refreshed every 10 ms, plus noise.
 * The REAL gain at larger duty is NOT measured (narrow-pulse region only), so every scenario is also run with the gain scaled by 0.5, 1, 2 and 3.
 * The regulator's integral gain is scheduled for the OLD transformer (buck gain 1.54 x boost): the measured buck / boost gains are almost equal, so the buck loop is
 * about 1.7 x more aggressive than designed: this study is the check that it still settles.  Pass/fail rules are at the bottom. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "slave_reg.h"

#define DT_TICKS 128.0
static uint32_t rng = 12345u;
static double noise(double a) { rng = rng * 1664525u + 1013904223u; return a * (((double)(rng >> 8) / 8388608.0) - 1.0); }
static double g_of(double vin)                       /* measured V per effective tick */
{
    if (vin <= 110.0) return 0.011 * vin / 110.0;
    if (vin <= 200.0) return 0.011 + (0.0297 - 0.011) * (vin - 110.0) / 90.0;
    if (vin <= 250.0) return 0.0297 + (0.0328 - 0.0297) * (vin - 200.0) / 50.0;
    return 0.0328;
}

typedef struct { double settle_err, duty_pp, peak_dev, t_ok, vend; int osc; } Res_t;

/* vin_of(t) gives the input at time t; cap = effective-tick cap (the integrator clamp) */
static Res_t run(double (*vin_of)(double), double gscale, double cap, double tmax, double t_settle, double t_event, double t_last_event)
{
    Reg_t r; int i, k; double win[20]; double duty_eff = 0.0, ccr = 0.0, eff_applied = 0.0;
    double meas_out, meas_in, vout_true, dmin = 1e9, dmax = -1e9, maxerr = 0.0, peak_dev = 0.0, t_last_bad = -1.0;
    Res_t res; double vin0 = vin_of(0.0);
    memset(&res, 0, sizeof res);
    for (k = 0; k < 20; k++) win[k] = vin0;
    meas_out = meas_in = vin0;
    Reg_Init(&r, 230.0f, (float)vin0, (float)vin0, 5000U);
    r.out_max = (float)cap;
    for (i = 0; i < (int)(tmax * 1000.0); i++)            /* 1 ms steps */
    {
        double t = i * 0.001, vin = vin_of(t);
        uint32_t now = 5000U + (uint32_t)i;
        int boost = (meas_in < (double)r.sp_v);
        double inj;
        if (i % 10 == 0)                                  /* once per half-cycle: apply the duty the controller wants */
        {
            double e = (double)Reg_Duty(&r); if (e > cap) e = cap;
            duty_eff = e;
            ccr = (duty_eff >= 1.0) ? (DT_TICKS + duty_eff) : 0.0;
            eff_applied = (ccr > DT_TICKS) ? (ccr - DT_TICKS) : 0.0;           /* a pulse at or below the dead time outputs nothing */
        }
        inj = gscale * g_of(vin) * eff_applied;
        vout_true = boost ? vin + inj : vin - inj;
        for (k = 19; k > 0; k--) win[k] = win[k - 1];
        win[0] = vout_true;
        if (i % 10 == 0)
        {
            double s = 0.0; for (k = 0; k < 20; k++) s += win[k];
            meas_out = s / 20.0 + noise(0.1); meas_in = vin + noise(0.1);
        }
        Reg_Service(&r, now, (float)meas_in, (float)meas_out, 1U);
        if (t >= t_settle)
        {
            double err = fabs(vout_true - 230.0);
            if (err > maxerr) maxerr = err;
            if (ccr < dmin) dmin = ccr; if (ccr > dmax) dmax = ccr;
        }
        if (t_event > 0.0 && t >= t_event)
        {
            double err = fabs(vout_true - 230.0);
            if (err > peak_dev) peak_dev = err;                  /* the worst deviation after the event (the instantaneous steps include one 1 ms sample in the OLD direction) */
            if (err > 1.5) t_last_bad = t;                       /* the last moment the error was outside 1.5 V */
        }
        res.vend = vout_true;
    }
    res.settle_err = maxerr; res.duty_pp = dmax - dmin; res.peak_dev = peak_dev; res.t_ok = (t_event > 0.0) ? ((t_last_bad < t_last_event) ? 0.0 : (t_last_bad - t_last_event)) : 0.0;
    res.osc = (maxerr > 3.0) ? 1 : 0;
    return res;
}
static double v_215(double t)   { (void)t; return 215.0; }
static double v_250(double t)   { (void)t; return 250.0; }
static double v_232(double t)   { (void)t; return 232.0; }
static double v_ramp_up(double t)   { double v = 200.0 + (t - 12.0) * 10.0; return (t < 12.0) ? 200.0 : ((v > 245.0) ? 245.0 : v); }     /* 200 -> 245 V at 10 V/s: boost -> buck across the setpoint */
static double v_ramp_dn(double t)   { double v = 245.0 - (t - 12.0) * 10.0; return (t < 12.0) ? 245.0 : ((v < 205.0) ? 205.0 : v); }     /* 245 -> 205 V at 10 V/s */
static double v_step(double t)  { return (t < 12.0) ? 200.0 : 240.0; }                 /* INSTANTANEOUS +40 V: a stress case, not a variac */
static double v_fast(double t)  { return (t < 10.0) ? 215.0 : ((t < 14.0) ? 245.0 : 205.0); }  /* instantaneous +30 V then -40 V */
static double v_sat(double t)   { return (t < 14.0) ? 170.0 : 215.0; }                 /* saturated first (cap 640), then recovers */

int main(void)
{
    const double gs[4] = { 0.5, 1.0, 2.0, 3.0 };
    int gi, fails = 0, runs = 0, s;
    struct { const char *name; double (*f)(double); double cap, tmax, tset, tev, tlast; int kind; double min_gain; } sc[] = {
        /* kind 0: constant input   1: variac-speed ramp   2: instantaneous step (stress)   3: saturation then recovery */
        { "215 V constant (boost)",              v_215,     2400.0, 28.0, 14.0,  0.0, 0.0, 0, 0.5 },
        { "250 V constant (buck)",               v_250,     2400.0, 28.0, 14.0,  0.0, 0.0, 0, 0.5 },
        { "232 V (inside the freeze band)",      v_232,     2400.0, 20.0, 10.0,  0.0, 0.0, 0, 0.5 },
        { "ramp 200 -> 245 V at 10 V/s",         v_ramp_up, 2400.0, 30.0,  0.0, 12.0, 12.0, 1, 0.5 },
        { "ramp 245 -> 205 V at 10 V/s",         v_ramp_dn, 2400.0, 30.0,  0.0, 12.0, 12.0, 1, 0.5 },
        { "STEP 200 -> 240 V instantly",         v_step,    2400.0, 30.0,  0.0, 12.0, 12.0, 2, 0.5 },
        { "STEP 215 -> 245 -> 205 V instantly",  v_fast,    2400.0, 30.0,  0.0, 10.0, 14.0, 2, 0.5 },
        { "170 V saturated (cap 640), then 215", v_sat,      640.0, 34.0,  0.0, 14.0, 14.0, 3, 1.0 },
    };
    printf("  regulator vs the measured plant. constant: worst |Vout-230| after settling and CCR swing | ramp/step: worst deviation after the event and time until the error stays within 1.5 V\n");
    printf("  %-38s %-6s %12s %12s %12s\n", "scenario", "gain", "worst err", "CCR swing", "time to 1.5V");
    for (gi = 0; gi < 4; gi++)
        for (s = 0; s < 8; s++)
        {
            Res_t r = run(sc[s].f, gs[gi], sc[s].cap, sc[s].tmax, (sc[s].kind == 0) ? sc[s].tset : sc[s].tmax - 3.0, sc[s].tev, sc[s].tlast);
            int ok = 1;
            if (sc[s].kind == 0) ok = (r.settle_err <= 1.5) && (r.duty_pp <= 40.0);                         /* settles, no hunting */
            if (sc[s].kind == 1) ok = (r.peak_dev <= 3.0)  && (r.settle_err <= 1.5);                        /* a variac-speed ramp is followed to within 3 V */
            if (sc[s].kind == 2) ok = (r.t_ok <= 1.0)      && (r.settle_err <= 1.5);                        /* an instantaneous step is recovered within 1 s */
            if (sc[s].kind == 3) ok = (r.t_ok <= 6.0)      && (r.settle_err <= 1.5);                        /* leaves saturation without a long excursion */
            printf("  %-38s x%-5.1f %9.2f V %10.0f  %9.2f s%s%s\n", sc[s].name, gs[gi], (sc[s].kind == 0) ? r.settle_err : r.peak_dev, r.duty_pp, r.t_ok,
                   ok ? "" : "   <-- FAILS", (gs[gi] > 2.0 && !ok) ? " (x3: reported only)" : "");
            if (gs[gi] >= sc[s].min_gain && gs[gi] <= 2.0) { runs++; if (!ok) fails++; }
        }
    printf("  RULES (gain x0.5 / x1 / x2; the x1 saturation case needs the cap to be large enough, so it is checked at x1 and x2): constant inputs settle to 1.5 V with a CCR swing <= 40;\n");
    printf("        a 10 V/s ramp is followed within 3 V; an instantaneous step is recovered within 1 s; leaving saturation takes <= 6 s. x3 is reported, not required.\n");
    printf("  %d required runs, %d failures\n", runs, fails);
    printf("  RESULT: %s\n", fails ? "FAILURES" : "ALL PASSED");
    return fails ? 1 : 0;
}
