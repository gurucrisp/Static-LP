/* The phase sequence decision and the phase-to-phase voltage (m_phase.c) against the exact maths. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdint.h>
#include "m_phase.h"
static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
static uint32_t rs = 0x1234567U;
static uint32_t rnd(void) { rs ^= rs << 13; rs ^= rs >> 17; rs ^= rs << 5; return rs; }

/* a round as three slaves would report it: R's next rising edge eR after the SYNC, Y and B lagging by fy and fb of a period (+-jit us of jitter), each slave with its own period error */
static void make_round(MPhRound_t *r, uint32_t T, uint32_t eR, double fy, double fb, int jit, int per_err_pm)
{
    uint32_t i; double f[3]; f[0] = 0.0; f[1] = fy; f[2] = fb;
    for (i = 0U; i < 3U; i++)
    {
        double e = (double)eR + f[i] * (double)T + (double)((int)(rnd() % (uint32_t)(2 * jit + 1)) - jit);
        long k = (long)floor(e / (double)T); e -= (double)k * (double)T;                                        /* the NEXT edge after the SYNC: within one period */
        r->valid[i] = 1U; r->delay_us[i] = (uint16_t)e; r->period_us[i] = (uint16_t)((double)T * (1.0 + (double)((int)(rnd() % (uint32_t)(2 * per_err_pm + 1)) - per_err_pm) / 1000.0));
    }
}
static MPhRound_t RD;
static void feed(MPhase_t *p, uint8_t cls_wanted, int n)   /* n rounds of the ideal class */
{
    int i;
    for (i = 0; i < n; i++)
    {
        double fy = (cls_wanted == MPH_REVERSE) ? 2.0 / 3.0 : 1.0 / 3.0, fb = (cls_wanted == MPH_REVERSE) ? 1.0 / 3.0 : 2.0 / 3.0;
        if (cls_wanted == MPH_SAME) { fy = 0.0; fb = 0.0; }
        make_round(&RD, 20000U, 1000U + (uint32_t)(i * 777), fy, fb, 100, 3); MPhase_Round(p, &RD, 1000U + (uint32_t)i * 500U);
    }
}

int main(void)
{
    uint32_t d; double worst = 0.0;
    for (d = 0U; d < 3600U; d++) { double e = fabs((double)MPhase_CosQ15(d) / 32768.0 - cos((double)d * M_PI / 1800.0)); if (e > worst) worst = e; }
    CHECK(worst < 0.0002, "the cosine table with interpolation is within %.6f of cos() over all 3600 angles (worst %.6f)", 0.0002, worst);
    CHECK(MPhase_CosQ15(0U) == 32768 && MPhase_CosQ15(900U) == 0 && MPhase_CosQ15(1800U) == -32768 && MPhase_CosQ15(2700U) == 0 && MPhase_CosQ15(3600U) == 32768, "cos at 0, 90, 180, 270, 360 degrees exactly");
    CHECK(abs(MPhase_CosQ15(1200U) + 16384) <= 2 && abs(MPhase_CosQ15(2400U) + 16384) <= 2, "cos(120) and cos(240) are -0.5");
    /* classification: thousands of random conditions */
    { int bad[4] = {0, 0, 0, 0}, t; static const uint32_t PER[3] = { 16667U, 20000U, 25000U };
      for (t = 0; t < 30000; t++)
      {
          uint32_t T = PER[rnd() % 3U], eR = rnd() % T; MPhRound_t r; uint8_t c;
          make_round(&r, T, eR, 1.0 / 3.0, 2.0 / 3.0, 150, 5); c = MPhase_Classify(&r, 0, 0); if (c != MPH_OK) bad[0]++;
          make_round(&r, T, eR, 2.0 / 3.0, 1.0 / 3.0, 150, 5); c = MPhase_Classify(&r, 0, 0); if (c != MPH_REVERSE) bad[1]++;
          make_round(&r, T, eR, 0.0, 0.0, 150, 5);              c = MPhase_Classify(&r, 0, 0); if (c != MPH_SAME) bad[2]++;
          make_round(&r, T, eR, 0.0, 2.0 / 3.0, 150, 5);        c = MPhase_Classify(&r, 0, 0); if (c != MPH_SAME) bad[3]++;
      }
      CHECK(bad[0] == 0, "30000 random mains (16.7, 20, 25 ms, any SYNC time, +-150 us jitter, +-0.5%% period error): sequence R-Y-B is always OK (%d wrong)", bad[0]);
      CHECK(bad[1] == 0, "R-B-Y is always REVERSE (%d wrong)", bad[1]);
      CHECK(bad[2] == 0, "Y and B on R's phase is always SAME (%d wrong)", bad[2]);
      CHECK(bad[3] == 0, "Y on R's phase (B correct) is SAME (%d wrong)", bad[3]); }
    /* the tolerance: 100 per mille of a period (36 degrees) */
    { MPhRound_t r; uint8_t c; int k;
      for (k = 0; k < 4; k++)
      {   static const int off[4] = { 90, 99, 101, 110 }; static const int exp_ok[4] = { 1, 1, 0, 0 };
          uint32_t T = 20000U; r.valid[0] = r.valid[1] = r.valid[2] = 1U; r.period_us[0] = r.period_us[1] = r.period_us[2] = (uint16_t)T;
          r.delay_us[0] = 5000U; r.delay_us[1] = (uint16_t)((5000U + 6667U + (uint32_t)(off[k] * 20)) % T); r.delay_us[2] = (uint16_t)((5000U + 13333U) % T); c = MPhase_Classify(&r, 0, 0);
          CHECK((c == MPH_OK) == (exp_ok[k] != 0), "Y %d per mille off its 1/3: %s (class %u)", off[k], exp_ok[k] ? "still OK" : "not OK", (unsigned)c); } }
    /* missing data */
    { MPhRound_t r; make_round(&r, 20000U, 3000U, 1.0 / 3.0, 2.0 / 3.0, 0, 0);
      r.valid[2] = 0U; CHECK(MPhase_Classify(&r, 0, 0) == MPH_UNKNOWN, "a phase that did not answer: unknown"); r.valid[2] = 1U;
      r.delay_us[1] = MPH_NONE_US; CHECK(MPhase_Classify(&r, 0, 0) == MPH_UNKNOWN, "a failed round (delay 65535): unknown"); make_round(&r, 20000U, 3000U, 1.0 / 3.0, 2.0 / 3.0, 0, 0);
      r.period_us[0] = r.period_us[1] = r.period_us[2] = 0U; CHECK(MPhase_Classify(&r, 0, 0) == MPH_UNKNOWN, "no slave knew the period: unknown");
      make_round(&r, 20000U, 3000U, 1.0 / 3.0, 2.0 / 3.0, 0, 0); r.period_us[0] = 0U; r.period_us[2] = 0U; CHECK(MPhase_Classify(&r, 0, 0) == MPH_OK, "one slave's period is enough");
      r.delay_us[2] = 23000U; CHECK(MPhase_Classify(&r, 0, 0) == MPH_UNKNOWN, "a delay longer than a period (an edge was missed): unknown"); }
    /* the decision: three identical rounds */
    { MPhase_t p; MPhase_Init(&p); CHECK(p.state == MPH_UNKNOWN, "starts unknown");
      feed(&p, MPH_OK, 2); CHECK(p.state == MPH_UNKNOWN, "two rounds are not enough"); feed(&p, MPH_OK, 1); CHECK(p.state == MPH_OK, "the third decides: OK");
      feed(&p, MPH_REVERSE, 2); CHECK(p.state == MPH_OK, "two rounds of REVERSE do not change a decision"); feed(&p, MPH_REVERSE, 1); CHECK(p.state == MPH_REVERSE, "the third does: REVERSE");
      feed(&p, MPH_OK, 1); feed(&p, MPH_REVERSE, 1); feed(&p, MPH_OK, 1); feed(&p, MPH_REVERSE, 1); CHECK(p.state == MPH_REVERSE, "alternating rounds never flip it");
      feed(&p, MPH_SAME, 3); CHECK(p.state == MPH_SAME && p.have_ang == 0U, "three rounds of SAME: SAME, and no angles");
      { MPhRound_t junk; make_round(&junk, 20000U, 3000U, 0.20, 0.45, 0, 0); MPhase_Round(&p, &junk, 9000U); CHECK(p.state == MPH_SAME, "a round that says nothing leaves the decision as it is"); }
      MPhase_Init(&p); feed(&p, MPH_OK, 3); MPhase_Age(&p, 1000U + 2U * 500U + 19999U); CHECK(p.state == MPH_OK, "19.999 s after the last classifying round it still stands");
      MPhase_Age(&p, 1000U + 2U * 500U + 20000U); CHECK(p.state == MPH_UNKNOWN, "20 s: unknown again");
      feed(&p, MPH_OK, 3); MPhase_Lost(&p); CHECK(p.state == MPH_UNKNOWN && p.have_ang == 0U, "a lost slave: unknown at once"); }
    /* the angles */
    { MPhase_t p; MPhase_Init(&p); feed(&p, MPH_OK, 4);
      CHECK(p.have_ang && abs((int)p.ang_dd[1] - 1200) < 40 && abs((int)p.ang_dd[2] - 2400) < 40 && p.ang_dd[0] == 0U, "R-Y-B: Y at about 120.0 degrees and B at about 240.0 (%u, %u)", (unsigned)p.ang_dd[1], (unsigned)p.ang_dd[2]);
      feed(&p, MPH_REVERSE, 1); CHECK(p.state == MPH_OK && p.have_ang && abs((int)p.ang_dd[1] - 2400) < 40 && abs((int)p.ang_dd[2] - 1200) < 40, "the angles follow a change of class at once, even before the decision changes (Y %u, B %u)", (unsigned)p.ang_dd[1], (unsigned)p.ang_dd[2]);
      MPhase_Init(&p); feed(&p, MPH_REVERSE, 4); CHECK(p.have_ang && abs((int)p.ang_dd[1] - 2400) < 40 && abs((int)p.ang_dd[2] - 1200) < 40, "R-B-Y: Y at about 240 and B at about 120 (%u, %u)", (unsigned)p.ang_dd[1], (unsigned)p.ang_dd[2]); }
    /* the phase-to-phase voltage */
    { MPhase_t p; uint16_t v; MPhase_Init(&p); p.have_ang = 1U; p.ang_dd[0] = 0U; p.ang_dd[1] = 1200U; p.ang_dd[2] = 2400U;
      v = MPhase_LineDv(&p, 0U, 1U, 2300U, 2300U); CHECK(abs((int)v - 3984) <= 2, "230 V and 230 V at 120 degrees: 398.4 V (%u)", (unsigned)v);
      CHECK(abs((int)MPhase_LineDv(&p, 1U, 2U, 2300U, 2300U) - 3984) <= 2 && abs((int)MPhase_LineDv(&p, 2U, 0U, 2300U, 2300U) - 3984) <= 2, "Y-B and B-R the same");
      CHECK(MPhase_LineDv(&p, 0U, 1U, 2300U, 2300U) == MPhase_LineDv(&p, 1U, 0U, 2300U, 2300U), "the order of the two phases does not matter");
      CHECK(MPhase_LineDv(&p, 0U, 1U, 0U, 2300U) == 0U && MPhase_LineDv(&p, 0U, 1U, 2300U, 0U) == 0U, "a phase with no voltage: 0 (shown as ---)");
      CHECK(MPhase_LineDv(&p, 1U, 1U, 2300U, 2300U) == 0U && MPhase_LineDv(&p, 0U, 3U, 2300U, 2300U) == 0U, "the same phase twice, or a phase 3: 0");
      p.have_ang = 0U; CHECK(MPhase_LineDv(&p, 0U, 1U, 2300U, 2300U) == 0U, "no angles known: 0");
      p.have_ang = 1U; p.ang_dd[1] = 0U; v = MPhase_LineDv(&p, 0U, 1U, 2300U, 2000U); CHECK(abs((int)v - 300) <= 1, "in phase (0 degrees): the difference, 30.0 V (%u)", (unsigned)v);
      p.ang_dd[1] = 1800U; v = MPhase_LineDv(&p, 0U, 1U, 2300U, 2000U); CHECK(abs((int)v - 4300) <= 1, "opposite (180 degrees): the sum, 430.0 V (%u)", (unsigned)v);
      { int bad = 0, bad2 = 0, t; double w = 0.0, w2 = 0.0; for (t = 0; t < 100000; t++)
        {   uint16_t va = (uint16_t)(500U + rnd() % 2501U), vb = (uint16_t)(500U + rnd() % 2501U), th = (uint16_t)(200U + rnd() % 3201U); double ref, e;      /* 20 to 340 degrees: the angles that are ever used (below 36 degrees the decision is SAME and no voltage is shown) */
            p.ang_dd[1] = th; ref = sqrt((double)va * va + (double)vb * vb - 2.0 * va * vb * cos((double)th * M_PI / 1800.0)); v = MPhase_LineDv(&p, 0U, 1U, va, vb); e = fabs((double)v - ref); if (e > w) w = e; if (e > 2.0) bad++; }
        CHECK(bad == 0, "100,000 random voltages (50 to 300 V) and angles from 20 to 340 degrees: within 0.2 V of the formula (worst %.2f tenths, %d over)", w, bad);
        for (t = 0; t < 100000; t++)
        {   uint16_t va = (uint16_t)(500U + rnd() % 2501U), vb = (uint16_t)(500U + rnd() % 2501U), th = (uint16_t)(rnd() % 3600U); double ref, e;
            p.ang_dd[1] = th; ref = sqrt((double)va * va + (double)vb * vb - 2.0 * va * vb * cos((double)th * M_PI / 1800.0)); v = MPhase_LineDv(&p, 0U, 1U, va, vb); e = fabs((double)v - ref); if (e > w2) w2 = e; if (e > 30.0) bad2++; }
        CHECK(bad2 == 0, "and over ALL angles within 3 V (two nearly equal voltages at nearly the same angle subtract two large numbers: worst %.1f tenths)", w2); } }
    printf("  phase decision: classification over 120000 random rounds, the 36 degree tolerance edge, missing data, 3 rounds to decide, ageing, the angles, the line voltage\n  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
