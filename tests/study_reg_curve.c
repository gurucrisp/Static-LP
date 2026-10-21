/* Regulator (slave_reg.c, the REAL module) against the plant MEASURED on the bench at stage 2 and stage 3 (v0.7.1 / v0.8.0), no load:
 *   injected change in (Vout - Vin), in the expected direction, per EFFECTIVE tick count:
 *      64 ticks: 1.9 V at 200 V, 2.1 V at 250 V    160 ticks: 2.8 V / 3.4 V    1600 ticks: about 17 V / 22 V (from "about 20 V" with the 1600 cap)    2752 ticks: extrapolated about 27 V / 38 V
 *   The curve is STEEP over the first 64 ticks and then flattens to about 0.009 to 0.014 V per tick (proportional to Vin): about 15 to 17 % of Vin per unit duty.
 *   The shape between 0 and 64 ticks is NOT measured: two assumptions are run (A: straight from the origin, B: steep start, 1 - exp(-ticks / 25)).
 * The inputs are CLOSE to the 230 V setpoint (small corrections: the steepest part of the curve, where the unit spends most of its time) and a few further away.
 * Rule: every run settles to within 1.5 V of 230 V with a CCR swing of at most 60 ticks (no hunting). */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "slave_reg.h"

#define DT_TICKS 128.0
static uint32_t rng = 12345u;
static double noise(double a) { rng = rng * 1664525u + 1013904223u; return a * (((double)(rng >> 8) / 8388608.0) - 1.0); }
static int g_shape = 0;                               /* 0 = A (straight from the origin to the 64-tick point), 1 = B (steep start) */
static double lerp(double x, double x0, double y0, double x1, double y1) { return y0 + (y1 - y0) * (x - x0) / (x1 - x0); }
static double dv_at(double eff, double vin)           /* injected change in the expected direction, volts */
{
    double w = (vin - 200.0) / 50.0, p64, p160, p1600, p2752;
    if (w < 0.0) w = 0.0; if (w > 1.2) w = 1.2;
    p64 = 1.9 + 0.2 * w; p160 = 2.8 + 0.6 * w; p1600 = 17.0 + 5.0 * w; p2752 = 27.0 + 11.0 * w;
    if (eff <= 0.0) return 0.0;
    if (eff < 64.0) return (g_shape == 0) ? p64 * eff / 64.0 : p64 * (1.0 - exp(-eff / 25.0)) / (1.0 - exp(-64.0 / 25.0));
    if (eff < 160.0)  return lerp(eff, 64.0, p64, 160.0, p160);
    if (eff < 1600.0) return lerp(eff, 160.0, p160, 1600.0, p1600);
    return lerp(eff, 1600.0, p1600, 2752.0, p2752);
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
        inj = gscale * dv_at(eff_applied, vin);
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

static double c_in = 230.0;
static double v_const(double t) { (void)t; return c_in; }
int main(void)
{
    const double ins[] = { 222.0, 226.0, 228.5, 231.5, 233.0, 235.0, 238.0, 245.0, 260.0 };
    int sh, i, bad = 0;
    printf("  the REAL regulator against the MEASURED curve (64 / 160 / 1600 / 2752 ticks), cap 2752, constant inputs near the 230 V setpoint\n");
    printf("  %-26s %-9s %12s %12s %12s\n", "plant shape between 0-64 ticks", "input", "worst error", "CCR swing", "final Vout");
    for (sh = 0; sh < 2; sh++)
    {
        g_shape = sh;
        for (i = 0; i < (int)(sizeof ins / sizeof ins[0]); i++)
        {
            Res_t r; int ok;
            c_in = ins[i];
            r = run(v_const, 1.0, 2752.0, 40.0, 24.0, 0.0, 0.0);
            ok = (r.settle_err <= 1.5) && (r.duty_pp <= 60.0);
            printf("  %-26s %6.1f V  %9.2f V %10.0f   %9.2f V%s\n", sh ? "B: steep start (25 ticks)" : "A: straight from origin", ins[i], r.settle_err, r.duty_pp, r.vend, ok ? "" : "   <-- HUNTING / NOT SETTLED");
            if (!ok) bad++;
        }
    }
    printf("  %d of %d runs hunt or do not settle\n", bad, 2 * (int)(sizeof ins / sizeof ins[0]));
    printf("  RESULT: %s\n", bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
