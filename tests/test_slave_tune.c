/* Tests for the transformer autotune (slave_tune): the sweep against a SIMULATED FAMILY of transformers (gain, dead time, saturation knee, flipped, absent, noisy, unsteady input, faults), the profile, the model check, the reach. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "slave_tune.h"
static int tot = 0, bad = 0;
#define CHECK(c, ...) do { tot++; if (!(c)) { bad++; printf("      FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint32_t rs = 4242U;
static double rnd(void) { rs = rs * 1664525U + 1013904223U; return (double)((rs >> 8) & 0xFFFFFFU) / 8388608.0 - 1.0; }       /* -1..1 */
static double gauss(void) { return (rnd() + rnd() + rnd()) / 1.732; }                                                           /* about unit variance */

typedef struct
{
    double k;            /* gain against the reference curve (1 = the bench transformer) */
    double dead;         /* dead time, ticks: nothing is injected below it */
    double knee;         /* ticks where the slope collapses (0 = no knee) */
    double knee_slope;   /* the slope above the knee, fraction */
    double sign;         /* +1, or -1 = connected backwards, 0 = no transformer */
    double noise;        /* volts, on every vout - vin sample */
    double offset;       /* sensor offset vout - vin at rest */
    double vin;          /* mains */
    double drift;        /* slow random walk of the mains, volts per second */
    double jump_t, jump_v;  /* the mains steps by jump_v at jump_t ms (0 = never) */
    double tau;          /* output lag, seconds */
    double superlin;     /* the real injection is this much larger than the linear model above 600 ticks (1 = none) */
    double sh_lo, sh_hi, sh_frac;   /* a SHOULDER in the low-duty region: between sh_lo and sh_hi effective ticks only sh_frac of the slope is injected, outside it the full slope (0 = none): what the converter's non-linear low-duty region looks like */
    double wob;          /* the mains wanders: the sum of two sines (2.3 s and 0.7 s periods) of this amplitude, volts: up to 3 x wob peak to peak (a variac with a converter on it) */
    double isat_v, isat_k;   /* the converter-side current: 1 A RMS at no load, and above isat_v volts of injection it rises by isat_k amperes per volt (a core that saturates); 0 = never */
    double wob_from;         /* the common-mode wander of the mains starts at this ms (0 = from the start) */
    double dwob, dwob_from;  /* v0.9.73: the INJECTION itself wobbles (not the mains): a 1.7 Hz sine of this amplitude, volts, from dwob_from effective ticks on (0 = never) */
    double jump_at, jump_gain;   /* above jump_at effective ticks the gain suddenly becomes jump_gain times larger (0 = never): a surprise that no step prediction can foresee */
} Plant_t;
static Plant_t nominal(void) { Plant_t p; p.k = 1.0; p.dead = 40.0; p.knee = 0.0; p.knee_slope = 0.25; p.sign = 1.0; p.noise = 0.1; p.offset = 0.2; p.vin = 230.0; p.drift = 0.0; p.jump_t = 0.0; p.jump_v = 0.0; p.tau = 0.3; p.superlin = 1.0; p.jump_at = 0.0; p.jump_gain = 1.0; p.wob = 0.0; p.sh_lo = 0.0; p.sh_hi = 0.0; p.sh_frac = 1.0; p.isat_v = 0.0; p.isat_k = 0.0; p.wob_from = 0.0; p.dwob = 0.0; p.dwob_from = 0.0; return p; }

static double true_inj(const Plant_t *p, double ticks, double vin)
{
    double eff = (ticks > p->dead) ? (ticks - p->dead) : 0.0, v;
    if (p->knee > 0.0 && eff > p->knee) eff = p->knee + p->knee_slope * (eff - p->knee);
    if (p->jump_at > 0.0 && eff > p->jump_at) eff = p->jump_at + p->jump_gain * (eff - p->jump_at);
    if (p->sh_hi > p->sh_lo && eff > p->sh_lo) { double in_sh = ((eff < p->sh_hi) ? eff : p->sh_hi) - p->sh_lo; eff = p->sh_lo + p->sh_frac * in_sh + ((eff > p->sh_hi) ? (eff - p->sh_hi) : 0.0); }
    v = p->sign * p->k * (double)Tune_RefG((float)vin) * eff;
    if (p->superlin != 1.0 && ticks > 600.0) v *= 1.0 + (p->superlin - 1.0) * (ticks - 600.0) / 2000.0;
    return v;
}

typedef struct { uint8_t result; uint32_t ms; int frames, steps; double max_inj, max_ticks_over; uint8_t profile_ok; TuneProfile_t prof; uint8_t request_after_done; TuneFrame_t last; } Run_t;

/* run one sweep: abort_at / fault_at in ms (0 = never) */
static Run_t sim(Plant_t p, double maxc, uint16_t hw_cap, uint32_t abort_at, uint32_t fault_at, uint32_t limit_ms)
{
    Run_t r; TuneIn_t in; TuneOut_t out; TuneFrame_t f; double inj = 0.0, vin = p.vin; uint32_t t = 100000U, t0 = t; uint16_t req = 0U;
    memset(&r, 0, sizeof r); memset(&in, 0, sizeof in); memset(&out, 0, sizeof out);
    if (Tune_Start((float)maxc, t) != 0U) { r.result = 255U; return r; }
    while ((t - t0) < limit_ms)
    {
        double target, a = 1.0 - exp(-0.01 / p.tau);
        t += 10U; vin += p.drift * 0.01 * rnd();
        if (p.jump_t > 0.0 && (t - t0) == (uint32_t)p.jump_t) vin += p.jump_v;
        target = true_inj(&p, (double)req, vin); inj += (target - inj) * a;
        if (fabs(inj) > r.max_inj) r.max_inj = fabs(inj);
        if (inj > r.max_inj) r.max_inj = inj;
        if ((double)req > (double)hw_cap) r.max_ticks_over = (double)req - (double)hw_cap;
        { double wob_on = ((double)(t - t0) >= p.wob_from) ? p.wob : 0.0; double vm = vin + wob_on * (sin(6.2831853 * (double)(t - t0) / 2300.0) + 0.5 * sin(6.2831853 * (double)(t - t0) / 700.0));
          in.vin = (float)(vm + 0.05 * gauss()); in.vout = (float)(vm + p.offset + inj + p.noise * gauss() + ((p.dwob > 0.0 && (double)req >= p.dwob_from) ? p.dwob * sin(6.2831853 * (double)(t - t0) / 588.0) : 0.0)); } in.meas_valid = 1U; in.vin_in_window = 1U;
        in.iout_valid = (uint8_t)(p.isat_k >= 0.0); in.iout_clip = 0U; in.iout_rms = (float)(1.0 + ((p.isat_k > 0.0 && inj > p.isat_v) ? p.isat_k * (inj - p.isat_v) : 0.0) + 0.03 * gauss()); in.iout_pk = in.iout_rms * 4.0f;       /* a no-load converter current of 1 A RMS, spiky */
        in.fault = (uint8_t)((fault_at != 0U && (t - t0) >= fault_at) ? 1U : 0U); in.maxc_v = (float)maxc; in.hw_cap_ticks = hw_cap;
        if (abort_at != 0U && (t - t0) == abort_at) Tune_Abort();
        Tune_Service(t, &in, &out); req = out.request_ticks;
        while (Tune_TakeFrame(&f)) { r.frames++; if (f.result == TUNE_R_STEP) r.steps++; r.last = f; }
        if (out.done) { r.result = r.last.result; r.ms = t - t0; r.profile_ok = Tune_GetProfile(&r.prof);
            { uint32_t k; for (k = 0U; k < 50U; k++) { Tune_Service(t + 10U * (k + 1U), &in, &out); if (out.request_ticks != 0U || out.active) r.request_after_done = 1U; } }
            return r; }
    }
    r.result = 254U; r.ms = limit_ms; return r;
}

int main(void)
{
    Plant_t p; Run_t r; double e; int i;
    /* ---- the nominal transformer (the bench one), Max Correction 120 V so that the whole list fits: finishes, about 33 s, a valid profile ---- */
    p = nominal(); r = sim(p, 120.0, 2752U, 0U, 0U, 120000U);
    CHECK(r.result == TUNE_R_FINISHED && r.profile_ok && Tune_ProfileValid(&r.prof), "the nominal transformer: FINISHED with a valid profile (result %u)", r.result);
    CHECK(r.ms >= 31000U && r.ms <= 40000U, "it takes about 33 s: 3 s baseline and 10 steps of 3 s (%u ms)", (unsigned)r.ms);
    CHECK(r.steps == 10 && r.frames == 11, "one $T frame per step and a final one (%d steps, %d frames)", r.steps, r.frames);
    CHECK(r.request_after_done == 0U, "after the end nothing is requested and it is no longer active");
    CHECK(r.prof.kib_m >= 940U && r.prof.kib_m <= 1060U && r.prof.kib_m == r.prof.kiu_m, "gain scale about 1.0 for the bench transformer (%u)", r.prof.kib_m);
    CHECK(r.prof.cap_ticks == 2752U && r.prof.knee == 0U, "no knee: the cap is at the end of the list (%u)", r.prof.cap_ticks);
    CHECK(r.max_inj <= 0.85 * 120.0, "the injection never went above 85 %% of Max Correction (%.1f V of 120 V)", r.max_inj);
    for (i = 0, e = 0.0; i < 6; i++) { double t_ = 100.0 + 450.0 * i, v = (double)Tune_PredictDv(&r.prof, (float)t_, 230.0f), tr = true_inj(&p, t_, 230.0); if (tr > 3.0 && fabs(v - tr) / tr > e) e = fabs(v - tr) / tr; }
    CHECK(e < 0.06, "the profile predicts the injection at 230 V within 6 %% over its range (worst %.1f %%)", e * 100.0);
    for (i = 0, e = 0.0; i < 4; i++) { double vv[4] = { 160.0, 190.0, 260.0, 290.0 }, tr = true_inj(&p, 1500.0, vv[i]), v = (double)Tune_PredictDv(&r.prof, 1500.0f, (float)vv[i]); if (fabs(v - tr) / tr > e) e = fabs(v - tr) / tr; }
    CHECK(e < 0.08, "and at other input voltages (160 to 290 V) within 8 %% by the shape of the bench curve (worst %.1f %%)", e * 100.0);
    /* ---- the 80 % rule with Max Correction 60 V: the sweep stops at about 48 V (about 1560 ticks), after 8 steps, the last one shortened ---- */
    p = nominal(); r = sim(p, 60.0, 2752U, 0U, 0U, 120000U);
    CHECK(r.result == TUNE_R_FINISHED && r.steps == 8 && r.prof.cap_ticks >= 1400U && r.prof.cap_ticks <= 1700U && r.max_inj <= 0.85 * 60.0 && r.ms >= 24000U && r.ms <= 30000U,
          "Max Correction 60 V: FINISHED after %d steps at a cap of %u ticks, the injection stays at or below 80 %% of 60 V (%.1f V), %u ms", r.steps, r.prof.cap_ticks, r.max_inj, (unsigned)r.ms);
    /* ---- a strong transformer: the 80 % rule shortens the last step, the cap is lower ---- */
    p = nominal(); p.k = 2.0; r = sim(p, 60.0, 2752U, 0U, 0U, 120000U);
    CHECK(r.result == TUNE_R_FINISHED && r.prof.cap_ticks < 2752U && r.max_inj <= 0.85 * 60.0, "k = 2: FINISHED, the cap is below the list end (%u) and the injection stays at or below 80 %% of 60 V (%.1f V)", r.prof.cap_ticks, r.max_inj);
    CHECK(r.prof.kib_m >= 440U && r.prof.kib_m <= 560U, "its gain scale is about 0.5 (%u)", r.prof.kib_m);
    /* ---- a weak transformer: the whole list, the scale limit 1.5 ---- */
    p = nominal(); p.k = 0.4; r = sim(p, 60.0, 2752U, 0U, 0U, 120000U);
    CHECK(r.result == TUNE_R_FINISHED && r.prof.cap_ticks == 2752U && r.prof.kib_m == 1500U, "k = 0.4: the whole list, the scale is limited to 1.5 (cap %u, scale %u)", r.prof.cap_ticks, r.prof.kib_m);
    /* ---- a saturating transformer: the knee ends the sweep, the cap is the step before it ---- */
    p = nominal(); p.knee = 800.0; p.knee_slope = 0.15; r = sim(p, 80.0, 2752U, 0U, 0U, 120000U);
    CHECK(r.result == TUNE_R_FINISHED && r.prof.knee == 1U && r.prof.cap_ticks <= 1100U && r.prof.cap_ticks >= 500U, "a knee at 800 ticks: FINISHED with the knee flag, the cap is the step before it (%u)", r.prof.cap_ticks);
    /* ---- flipped: stops early, nothing stored, little wrong-way injection ---- */
    p = nominal(); p.sign = -1.0; r = sim(p, 60.0, 2752U, 0U, 0U, 120000U);
    CHECK(r.result == TUNE_R_FLIPPED && !r.profile_ok && r.ms < 25000U && r.max_inj < 4.0, "a flipped transformer: FLIPPED, nothing stored, stopped after %u ms with at most %.1f V the wrong way", (unsigned)r.ms, r.max_inj);
    /* ---- nothing connected ---- */
    p = nominal(); p.sign = 0.0; r = sim(p, 60.0, 2752U, 0U, 0U, 120000U);
    CHECK(r.result == TUNE_R_NO_RESPONSE && !r.profile_ok, "no transformer: NO_RESPONSE, nothing stored");
    /* ---- Max Correction not set ---- */
    CHECK(Tune_Start(0.0f, 1000U) == 1U && Tune_Running() == 0U, "Max Correction 0: refused, nothing started");
    /* ---- a second start while running ---- */
    (void)Tune_Start(60.0f, 1000U); CHECK(Tune_Start(60.0f, 1010U) == 2U, "a start while running is refused"); Tune_Abort();
    /* ---- unsteady input ---- */
    p = nominal(); p.jump_t = 20000.0; p.jump_v = 4.0; r = sim(p, 60.0, 2752U, 0U, 0U, 120000U);
    CHECK(r.result == TUNE_R_FINISHED && r.profile_ok, "the mains jumps 4 V in the middle of a step: that step's window is measured AGAIN and the tune finishes (result %u)", r.result);
    p = nominal(); p.drift = 200.0; r = sim(p, 60.0, 2752U, 0U, 0U, 200000U);
    CHECK(r.result == TUNE_R_UNSTEADY && !r.profile_ok && r.last.dv_dv >= 80, "a mains that really never settles (a random walk of 200 V/s): UNSTEADY after waiting, nothing stored, the frame says the input moved %.1f V (result %u, %u ms)", (double)r.last.dv_dv / 10.0, r.result, (unsigned)r.ms);
    /* ---- THE CONVERTER-CURRENT GUARD (v0.9.43): a core that starts to saturate draws sharp current spikes. The sweep sees VOLTAGE only; the current says it ---- */
    { Plant_t q = nominal(); q.k = 1.0; q.isat_v = 12.0; q.isat_k = 0.9; r = sim(q, 60.0, 2752U, 0U, 0U, 200000U);
      CHECK(r.result == TUNE_R_ABORTED && !r.profile_ok && r.last.dv_dv >= 30 && r.max_inj < 12.0 + 8.0, "a core that saturates above 12 V (the current rises 0.9 A per volt): ABORTED, nothing stored, the frame says %.1f A, the injection reached %.1f V (the limit was 60)", (double)r.last.dv_dv / 10.0, r.max_inj);
      CHECK(r.request_after_done == 0U, "and the duty request is 0 as the sweep ends"); }
    { Plant_t q = nominal(); q.isat_v = 0.0; q.isat_k = 0.0; r = sim(q, 60.0, 2752U, 0U, 0U, 200000U); CHECK(r.result == TUNE_R_FINISHED && r.profile_ok, "a steady 1 A converter current all the way: the guard stays out of the way (result %u)", r.result); }
    { Plant_t q = nominal(); q.isat_k = -1.0; r = sim(q, 60.0, 2752U, 0U, 0U, 200000U);
      CHECK(r.result == TUNE_R_ABORTED && !r.profile_ok && r.last.dv_dv == -1 && r.max_inj < 0.5, "no converter-current reading: the sweep does NOT run (ABORTED at the end of the baseline, no duty applied, the frame says -1): result %u, frame %d", r.result, (int)r.last.dv_dv); }
    /* the source starts to swing late, at high duty (a toroid, a soft variac): the duty must not be held for long (it was 30 s) */
    { Plant_t q = nominal(); q.k = 1.0; q.wob = 6.0; q.wob_from = 15000.0; r = sim(q, 60.0, 2752U, 0U, 0U, 200000U);
      CHECK(r.result == TUNE_R_UNSTEADY && !r.profile_ok && r.ms <= 15000U + 15000U, "the input starts to swing 18 V peak to peak after 15 s: the sweep gives up within 15 s of that (it took up to 30), at %u ms", (unsigned)r.ms); }
    /* ---- YOUR PROFILE (v0.9.38 and earlier): "Cap 64 ticks, Knee, n04" at 234 V in with 36 V of headroom: readings of about 0.7, 1.9, 4.4 V and then only 1.5 V more at 128 ticks (twice the reference gain, then a slope of a third).
     *      A slope that changes at 4 V of injection is noise and the converter's low-duty region, not a saturation knee: it must not become the cap (with SLAVE_TUNE_APPLY on it held the regulator to 4 V of correction) ---- */
    { Plant_t q = nominal(); TuneProfile_t pf; q.vin = 234.0; q.dead = 0.0; q.k = 2.2; q.knee = 64.0; q.knee_slope = 0.3; q.noise = 0.25;
      r = sim(q, 36.0, 2752U, 0U, 0U, 300000U); (void)Tune_GetProfile(&pf);
      CHECK(r.result == TUNE_R_FINISHED && r.profile_ok && pf.knee == 0U && pf.n >= 6U && pf.cap_ticks >= 300U, "a slope that changes at 4 V is NOT a knee: no knee flag, %u points, cap %u ticks (the old rule: cap 64, knee, 4 points)", (unsigned)pf.n, (unsigned)pf.cap_ticks);
      CHECK(r.max_inj <= 0.8 * 36.0 + 4.0, "and the sweep still stays under 80 %% of the headroom: %.1f V of 28.8", r.max_inj); }
    { Plant_t q = nominal(); TuneProfile_t pf; q.vin = 230.0; q.dead = 0.0; q.knee = 800.0; q.knee_slope = 0.12; q.noise = 0.2;
      r = sim(q, 120.0, 2752U, 0U, 0U, 300000U); (void)Tune_GetProfile(&pf);
      CHECK(r.result == TUNE_R_FINISHED && pf.knee == 1U && pf.cap_ticks >= 512U && pf.cap_ticks <= 1024U, "a REAL knee (800 ticks, about 25 V of injection) is still found: knee flag, cap %u ticks", (unsigned)pf.cap_ticks); }
    { int nbad = 0, seeds, worst_n = 99; for (seeds = 0; seeds < 120; seeds++)
      { Plant_t q = nominal(); TuneProfile_t pf; q.vin = 234.0; q.dead = 0.0; q.k = 1.4 + 0.8 * (double)(seeds % 7) / 6.0; q.noise = 0.2 + 0.25 * (double)(seeds % 4) / 3.0; rs = 4242U + (uint32_t)seeds * 7919U;
        r = sim(q, 36.0, 2752U, 0U, 0U, 300000U); (void)Tune_GetProfile(&pf);
        if (!((r.result == TUNE_R_FINISHED) && (pf.knee == 0U) && (pf.cap_ticks >= 250U))) { nbad++; }
        if (pf.n < worst_n) { worst_n = pf.n; } }
      CHECK(nbad == 0, "120 noisy sweeps at 234 V (gain 1.4 to 2.2, noise 0.2 to 0.45 V): no false knee, a cap of at least 250 ticks (%d bad, the fewest points %d)", nbad, worst_n); }
    /* a VARIAC with the converter on it: the input wanders by 2 to 3 V peak to peak in every window. v0.9.36 stopped the sweep at the first such window (1.5 V limit) */
    p = nominal(); p.wob = 0.9; r = sim(p, 60.0, 2752U, 0U, 0U, 200000U);
    CHECK(r.result == TUNE_R_FINISHED && r.profile_ok, "a wandering mains (up to 2.7 V peak to peak): the tune finishes (result %u, %u ms)", r.result, (unsigned)r.ms);
    { TuneProfile_t pf; p = nominal(); p.wob = 0.9; r = sim(p, 60.0, 2752U, 0U, 0U, 200000U); (void)Tune_GetProfile(&pf);
      CHECK(pf.kib_m >= 940U && pf.kib_m <= 1060U && pf.cap_ticks >= 1500U, "and its profile is as good as with a steady mains (gain scale %u, cap %u)", (unsigned)pf.kib_m, (unsigned)pf.cap_ticks); }
    p = nominal(); p.wob = 0.9; p.k = 1.9; r = sim(p, 80.0, 2752U, 0U, 0U, 200000U);
    CHECK(r.result == TUNE_R_FINISHED && r.profile_ok, "a transformer with 1.9 times the gain on a wandering mains (your bench): finishes (result %u)", r.result);
    p = nominal(); p.wob = 10.0; r = sim(p, 60.0, 2752U, 0U, 0U, 400000U);
    CHECK(r.result == TUNE_R_UNSTEADY && !r.profile_ok && r.last.dv_dv >= 80, "a mains that swings 25 V peak to peak (the dial being turned) never gives a window: UNSTEADY, nothing stored, the frame says %.1f V (result %u)", (double)r.last.dv_dv / 10.0, r.result);
    /* ---- YOUR bench (v0.9.40): "Input Not Stable" at step 5 or 6, 4.4 V or 9 V. The input wanders together with the output (a variac with the converter's current on it): the DIFFERENCE is what is measured, and it is clean ---- */
    { TuneProfile_t pf; p = nominal(); p.wob = 2.5; p.k = 1.0; r = sim(p, 60.0, 2752U, 0U, 0U, 300000U); (void)Tune_GetProfile(&pf);
      CHECK(r.result == TUNE_R_FINISHED && r.profile_ok && pf.kib_m >= 940U && pf.kib_m <= 1060U, "an input that swings 7 V peak to peak together with the output: the tune finishes with a good gain scale (result %u, %u thousandths)", r.result, (unsigned)pf.kib_m);
      p = nominal(); p.wob = 3.0; p.k = 2.0; p.vin = 234.0; r = sim(p, 36.0, 2752U, 0U, 0U, 300000U); (void)Tune_GetProfile(&pf);
      CHECK(r.result == TUNE_R_FINISHED && r.profile_ok && pf.n >= 5U, "twice the gain, 9 V peak to peak of common swing, 36 V of headroom (your sweep): finishes with %u points", (unsigned)pf.n); }
    /* the difference itself: noise, and a plant that has not settled */
    p = nominal(); p.noise = 0.6; r = sim(p, 60.0, 2752U, 0U, 0U, 300000U);
    CHECK(r.result == TUNE_R_FINISHED && r.profile_ok, "0.6 V of noise on the difference: accepted (result %u)", r.result);
    p = nominal(); p.noise = 4.0; r = sim(p, 60.0, 2752U, 0U, 0U, 300000U);
    CHECK(r.result == TUNE_R_UNSTEADY && !r.profile_ok && r.last.dv_dv <= -10, "4 V of noise on the difference: UNSTEADY and the frame says the reading is noisy (%.1f V of spread), nothing stored", (double)r.last.dv_dv / 10.0);
    {   /* v0.9.73: a toroid gave up at step 5 or 7 with "Reading spread 2.3 V": a step that stays noisy or drifting (not an input jump) for the whole wait is accepted
         * with its average if the spread is within the hard limit */
        TuneProfile_t pf; uint32_t clean_ms;
        { Run_t rc = sim(nominal(), 60.0, 2752U, 0U, 0U, 300000U); clean_ms = rc.ms; }
        p = nominal(); p.dwob = 3.2; p.dwob_from = 256.0; r = sim(p, 60.0, 2752U, 0U, 0U, 600000U); (void)Tune_GetProfile(&pf);
        CHECK(r.ms >= clean_ms + (uint32_t)Tune_SpreadAccepts() * (TUNE_STEP_WAIT_MS - TUNE_STEP_MS), "each such step was accepted only after the full wait (%u ms against %u clean, %u accepted)", (unsigned)r.ms, (unsigned)clean_ms, (unsigned)Tune_SpreadAccepts());
        {   /* the accepted points themselves: within 0.6 V of the true injection (their averages are unbiased) */
            uint32_t pi; double worst = 0.0;
            for (pi = 0U; pi < pf.n; pi++) { double tr = true_inj(&p, (double)pf.ticks[pi], (double)pf.vin_ref_dv / 10.0), er = fabs((double)pf.dv_dv[pi] / 10.0 - tr); if (pf.ticks[pi] >= 256U && er > worst) worst = er; }
            CHECK(worst <= 0.6, "the points from step 5 on are within 0.6 V of the true injection (worst %.2f V)", worst);
        }
        CHECK(r.result == TUNE_R_FINISHED && r.profile_ok && pf.kib_m >= 900U && pf.kib_m <= 1100U && Tune_SpreadAccepts() > 0U, "the injection itself wobbles 3.2 V (2.3 V of spread) at 1.7 Hz from step 5 on (a toroid): finishes, the gain scale right (result %u, %u thousandths, %u accepted)", r.result, (unsigned)pf.kib_m, (unsigned)Tune_SpreadAccepts());
        {   /* the window judge itself: noisy (accepted after the wait within the hard limit), drifting (never accepted), the frame's encoding */
            float w[150]; int16_t un; uint8_t ac, c; int k;
            for (k = 0; k < 150; k++) w[k] = 6.0f + 3.2f * (float)sin(6.2831853 * (double)k * 10.0 / 588.0);                   /* 6 V of injection (step 5) wobbling 3.2 V: 2.3 V of spread */
            c = Tune_TestWindow(w, 150U, 0.0f, &un, &ac);
            CHECK(c == 2U && un < 0 && un > -500 && ac == 1U, "a noisy window (2.3 V of spread at 6 V): refused at first, 'reading noisy' (%d), accepted after the wait (%u)", (int)un, ac);
            for (k = 0; k < 150; k++) w[k] = 6.0f + 8.0f * (float)sin(6.2831853 * (double)k * 10.0 / 588.0);
            c = Tune_TestWindow(w, 150U, 0.0f, &un, &ac);
            CHECK(c == 2U && un < 0 && un > -500 && ac == 0U, "8 V of wobble (5.7 V of spread) at 6 V: beyond the hard limit (4.5 V), never accepted (%d, %u)", (int)un, ac);
            for (k = 0; k < 150; k++) w[k] = 8.0f + 3.0f * (float)k / 150.0f;                                                       /* a steady creep of 2 V/s */
            c = Tune_TestWindow(w, 150U, 0.0f, &un, &ac);
            CHECK(c == 2U && un <= -500 && ac == 0U, "a drifting window (2 V/s at 8 V): 'not settling' (%d), never accepted (%u)", (int)un, ac);
            for (k = 0; k < 150; k++) w[k] = 10.0f + 0.3f * (float)sin((double)k);
            c = Tune_TestWindow(w, 150U, 0.0f, &un, &ac);
            CHECK(c == 0U, "a clean window: good (%u)", c);
        }
        p = nominal(); r = sim(p, 60.0, 2752U, 0U, 0U, 300000U);
        CHECK(r.result == TUNE_R_FINISHED && Tune_SpreadAccepts() == 0U, "a clean plant: no step needs that (%u)", (unsigned)Tune_SpreadAccepts());
        p = nominal(); p.dwob = 12.0; p.dwob_from = 256.0; r = sim(p, 60.0, 2752U, 0U, 0U, 600000U);
        CHECK(r.result == TUNE_R_UNSTEADY && !r.profile_ok && r.last.dv_dv <= -10 && r.last.dv_dv > -500, "an injection that wobbles 12 V from step 5: beyond the hard limit, still UNSTEADY, 'reading noisy' (frame %d)", (int)r.last.dv_dv);
    }
    { TuneProfile_t pf; p = nominal(); p.tau = 1.3; r = sim(p, 60.0, 2752U, 0U, 0U, 400000U); (void)Tune_GetProfile(&pf);
      CHECK(r.result == TUNE_R_FINISHED && pf.kib_m >= 940U && pf.kib_m <= 1060U, "a slow plant (time constant 1.3 s): the windows are measured again until the difference has settled, and the gain scale is right (%u thousandths, %u ms)", (unsigned)pf.kib_m, (unsigned)r.ms); }
    { Plant_t q = nominal(); q.vin = 230.0; q.wob = 0.0; r = sim(q, 60.0, 2752U, 0U, 0U, 200000U); CHECK(r.result == TUNE_R_FINISHED, "a steady mains: unchanged, finishes"); }
    p = nominal(); p.jump_t = 1500.0; p.jump_v = 4.0; r = sim(p, 60.0, 2752U, 0U, 0U, 120000U);
    CHECK(r.result == TUNE_R_FINISHED, "a jump during the first baseline window only restarts the window: the tune still finishes (result %u)", r.result);
    /* ---- faults and the master ---- */
    p = nominal(); r = sim(p, 60.0, 2752U, 0U, 14000U, 120000U);
    CHECK(r.result == TUNE_R_ABORTED && r.request_after_done == 0U && !r.profile_ok, "a fault (FLT, break, over-voltage) at 14 s: ABORTED, duty to zero at once, nothing stored");
    p = nominal(); r = sim(p, 60.0, 2752U, 17000U, 0U, 120000U);
    CHECK(r.result == TUNE_R_BY_MASTER && r.request_after_done == 0U && !r.profile_ok, "TABORT at 17 s: BY_MASTER, nothing stored");
    p = nominal(); p.superlin = 6.0; r = sim(p, 60.0, 2752U, 0U, 0U, 120000U);
    CHECK((r.result == TUNE_R_ABORTED || r.result == TUNE_R_FINISHED) && r.max_inj <= 60.0 + 5.0 + 3.0, "a transformer that injects 6 times more than the model predicts: the over-voltage backstop holds it (%.1f V of 60 V, result %u)", r.max_inj, r.result);
    /* ---- the surprise: the gain jumps 8 times above 500 ticks. The step prediction cannot see it: the over-voltage backstop is the only thing that stops the injection ---- */
    p = nominal(); p.jump_at = 500.0; p.jump_gain = 8.0; r = sim(p, 60.0, 2752U, 0U, 0U, 120000U);
    CHECK(r.result == TUNE_R_ABORTED && !r.profile_ok && r.max_inj <= 60.0 + 5.0 + 6.0 && r.request_after_done == 0U,
          "the gain jumps 8 x: ABORTED by the over-voltage backstop, nothing stored, the injection peaked at %.1f V (Max Correction 60 V plus 5 V of margin plus one sample's rise)", r.max_inj);
    /* ---- the hardware cap is never exceeded ---- */
    p = nominal(); p.k = 0.4; r = sim(p, 60.0, 1000U, 0U, 0U, 120000U);
    CHECK(r.result == TUNE_R_FINISHED && r.prof.cap_ticks <= 1000U && r.max_ticks_over == 0.0, "a hardware cap of 1000 ticks: never exceeded, the cap is at most 1000 (%u)", r.prof.cap_ticks);
    /* ---- a family of 400 random transformers: the safety properties hold for every one ---- */
    { int ok_inj = 1, ok_valid = 1, ok_after = 1, ok_cap = 1, ok_pred = 1, n_fin = 0, n_other = 0, worst_i = -1; double worst = 0.0;
      for (i = 0; i < 400; i++)
      {
          Plant_t q = nominal(); double maxc = 20.0 + 100.0 * (rnd() * 0.5 + 0.5);
          q.k = 0.3 + 2.7 * (rnd() * 0.5 + 0.5); q.dead = 120.0 * (rnd() * 0.5 + 0.5); q.noise = 0.05 + 0.15 * (rnd() * 0.5 + 0.5); q.offset = 0.5 * rnd(); q.vin = 120.0 + 170.0 * (rnd() * 0.5 + 0.5);
          if (rnd() > 0.6) { q.knee = 300.0 + 1500.0 * (rnd() * 0.5 + 0.5); q.knee_slope = 0.05 + 0.3 * (rnd() * 0.5 + 0.5); }
          r = sim(q, maxc, 2752U, 0U, 0U, 120000U);
          if (r.max_inj > 0.85 * maxc) { ok_inj = 0; if (r.max_inj / maxc > worst) { worst = r.max_inj / maxc; worst_i = i; } }
          if (r.request_after_done != 0U) ok_after = 0;
          if (r.result == TUNE_R_FINISHED)
          {
              n_fin++; if (!r.profile_ok || !Tune_ProfileValid(&r.prof)) ok_valid = 0;
              if (r.prof.cap_ticks > 2752U) ok_cap = 0;
              { double tc = true_inj(&q, (double)r.prof.cap_ticks, q.vin), pc = (double)Tune_PredictDv(&r.prof, (float)r.prof.cap_ticks, (float)q.vin); if (tc > 3.0 && fabs(pc - tc) / tc > 0.10) ok_pred = 0; }
          }
          else n_other++;
      }
      CHECK(ok_inj, "400 random transformers: the injection never went above 85 %% of Max Correction (worst %.0f %% at #%d)", worst * 100.0, worst_i);
      CHECK(ok_after && ok_valid && ok_cap, "no duty after the end, every finished profile valid, no cap above the hardware limit");
      CHECK(ok_pred, "every finished profile predicts the injection at its own cap within 10 %%");
      CHECK(n_fin > 300, "most of the family finishes (%d of 400; the others stopped on the 80 %% / noise rules)", n_fin); (void)n_other; }
    /* ---- serialisation ---- */
    p = nominal(); r = sim(p, 120.0, 2752U, 0U, 0U, 120000U);
    { uint8_t buf[96]; TuneProfile_t q; uint32_t n = Tune_Serialize(&r.prof, buf, sizeof buf);
      CHECK(n == TUNE_BYTES(r.prof.n) && n <= 96U, "a profile serialises to %u bytes (the store takes 96)", (unsigned)n);
      CHECK(Tune_Deserialize(buf, n, &q) == 1U && q.cap_ticks == r.prof.cap_ticks && q.kib_m == r.prof.kib_m && q.n == r.prof.n && q.ticks[3] == r.prof.ticks[3] && q.dv_dv[5] == r.prof.dv_dv[5], "and comes back identical");
      CHECK(Tune_Serialize(&r.prof, buf, 20U) == 0U, "no room: nothing written");
      CHECK(Tune_Deserialize(buf, n - 1U, &q) == 0U && Tune_Deserialize(buf, 5U, &q) == 0U, "a truncated record is rejected");
      buf[1] = 13U; CHECK(Tune_Deserialize(buf, n, &q) == 0U, "a point count above 12 is rejected"); buf[1] = r.prof.n;
      { TuneProfile_t b = r.prof; b.cap_ticks = 0U; CHECK(Tune_ProfileValid(&b) == 0U, "cap 0 is invalid"); b = r.prof; b.kib_m = 50U; CHECK(Tune_ProfileValid(&b) == 0U, "a gain scale of 0.05 is invalid"); b = r.prof; b.kiu_m = 2000U; CHECK(Tune_ProfileValid(&b) == 0U, "a gain scale of 2.0 is invalid");
        b = r.prof; b.ticks[2] = b.ticks[1]; CHECK(Tune_ProfileValid(&b) == 0U, "ticks that do not increase are invalid"); b = r.prof; b.version = 9U; CHECK(Tune_ProfileValid(&b) == 0U, "an unknown version is invalid"); b = r.prof; b.n = 2U; CHECK(Tune_ProfileValid(&b) == 0U, "fewer than 3 points is invalid"); } }
    /* ---- the model check ---- */
    { TuneProfile_t pr = r.prof; double tr = true_inj(&p, 1000.0, 230.0), tr2 = true_inj(&p, 1000.0, 180.0); int i2, okc = 1;
      CHECK(Tune_ModelCheck(&pr, 1000.0f, 230.0f, (float)tr) == TUNE_CHECK_OK, "the same transformer at 230 V: OK");
      CHECK(Tune_ModelCheck(&pr, 1000.0f, 180.0f, (float)tr2) == TUNE_CHECK_OK, "and at 180 V (the shape of the bench curve carries it): OK");
      for (i2 = 0; i2 < 200; i2++) { double v = 110.0 + 180.0 * (rnd() * 0.5 + 0.5), tt = 300.0 + 1500.0 * (rnd() * 0.5 + 0.5), m = true_inj(&p, tt, v) + 0.1 * gauss(); if (Tune_ModelCheck(&pr, (float)tt, (float)v, (float)m) != TUNE_CHECK_OK) okc = 0; }
      CHECK(okc, "200 random start-ups of the SAME transformer (110 to 290 V, 300 to 1800 ticks, 0.1 V of noise): never a false alarm");
      CHECK(Tune_ModelCheck(&pr, 1000.0f, 230.0f, (float)(tr * 0.60)) == TUNE_CHECK_MISMATCH, "a transformer with 60 %% of the gain: MISMATCH");
      CHECK(Tune_ModelCheck(&pr, 1000.0f, 230.0f, (float)(tr * 1.50)) == TUNE_CHECK_MISMATCH, "one with 150 %% of the gain: MISMATCH");
      CHECK(Tune_ModelCheck(&pr, 1000.0f, 230.0f, (float)(tr * 0.85)) == TUNE_CHECK_OK && Tune_ModelCheck(&pr, 1000.0f, 230.0f, (float)(tr * 1.15)) == TUNE_CHECK_OK, "+-15 %% is inside the tolerance");
      CHECK(Tune_ModelCheck(&pr, 1000.0f, 230.0f, (float)(-tr)) == TUNE_CHECK_FLIP, "the wrong sign: FLIP");
      CHECK(Tune_ModelCheck(&pr, 20.0f, 230.0f, 0.0f) == TUNE_CHECK_SKIP, "too little injection to judge (20 ticks): SKIP"); }
    /* ---- the reach ---- */
    { double rl = (double)Tune_ReachLoDv(&r.prof, 230.0f) / 10.0, rh = (double)Tune_ReachHiDv(&r.prof, 230.0f) / 10.0; TuneProfile_t weak; Run_t rw; Plant_t pw = nominal(); pw.k = 0.5; rw = sim(pw, 60.0, 2752U, 0U, 0U, 120000U); weak = rw.prof;
      CHECK(rl > 100.0 && rl < 230.0 && 230.0 - rl >= 40.0 && rh > 230.0 && rh <= 400.0, "the reach for a 230 V set voltage: from %.1f V to %.1f V", rl, rh);
      { double chk = rl + true_inj(&p, (double)r.prof.cap_ticks, rl); CHECK(fabs(chk - 230.0) < 4.0, "at the lowest input the cap really reaches the set voltage (%.1f V)", chk); }
      CHECK((double)Tune_ReachLoDv(&weak, 230.0f) / 10.0 > rl, "a weaker transformer reaches less low (%.1f V against %.1f V)", (double)Tune_ReachLoDv(&weak, 230.0f) / 10.0, rl); }
    /* ---- the frames ---- */
    p = nominal(); r = sim(p, 120.0, 2752U, 0U, 0U, 120000U);
    CHECK(r.last.result == TUNE_R_FINISHED && r.last.ticks >= 2000U && r.last.vin_dv > 2200U && r.last.vin_dv < 2400U, "the last frame says FINISHED with the last step's ticks and the input (ticks %u, vin %u)", r.last.ticks, r.last.vin_dv);
    printf("  autotune: the sweep against 400+ simulated transformers (gain, dead time, knee, noise, flipped, absent, unsteady, faults), the profile, the model check, the reach\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", tot, bad, bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
