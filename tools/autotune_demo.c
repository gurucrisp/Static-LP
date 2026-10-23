/* Prints an autotune sweep against a simulated transformer, step by step: what the unit would do.
 *   gcc -std=c99 -O2 -Islave/Core/Inc tools/autotune_demo.c slave/Core/Src/slave_tune.c -o autotune_demo -lm
 *   ./autotune_demo [gain] [dead_ticks] [knee_ticks] [sign] [max_correction_V] [mains_V]
 *   gain 1.0 = the bench transformer; knee 0 = none; sign -1 = connected backwards, 0 = no transformer. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "slave_tune.h"
static unsigned rs = 777U;
static double rnd(void) { rs = rs * 1664525U + 1013904223U; return (double)((rs >> 8) & 0xFFFFFFU) / 8388608.0 - 1.0; }
int main(int argc, char **argv)
{
    double k = (argc > 1) ? atof(argv[1]) : 1.0, dead = (argc > 2) ? atof(argv[2]) : 40.0, knee = (argc > 3) ? atof(argv[3]) : 0.0, sign = (argc > 4) ? atof(argv[4]) : 1.0, maxc = (argc > 5) ? atof(argv[5]) : 60.0, vin = (argc > 6) ? atof(argv[6]) : 230.0;
    TuneIn_t in; TuneOut_t out; TuneFrame_t f; unsigned t = 100000U, t0 = t; double inj = 0.0; uint16_t req = 0U; const char *rn[7] = { "step", "FINISHED", "FLIPPED", "NO RESPONSE", "ABORTED", "UNSTEADY", "STOPPED BY MASTER" };
    memset(&in, 0, sizeof in); memset(&out, 0, sizeof out);
    printf("simulated transformer: gain %.2f x the bench one, dead time %.0f ticks, knee %s, %s, mains %.0f V, Max Correction %.0f V (the sweep stays at or below %.0f V)\n\n", k, dead, (knee > 0.0) ? "yes" : "none", (sign > 0.0) ? "connected correctly" : ((sign < 0.0) ? "CONNECTED BACKWARDS" : "NOT CONNECTED"), vin, maxc, 0.8 * maxc);
    if (Tune_Start((float)maxc, t) != 0U) { printf("refused: Max Correction not set\n"); return 1; }
    printf(" time    step  ticks   input   injected   result\n");
    while ((t - t0) < 120000U)
    {
        double eff, target;
        t += 10U; eff = ((double)req > dead) ? ((double)req - dead) : 0.0; if (knee > 0.0 && eff > knee) eff = knee + 0.2 * (eff - knee);
        target = sign * k * (double)Tune_RefG((float)vin) * eff; inj += (target - inj) * (1.0 - exp(-0.01 / 0.3));
        in.vin = (float)(vin + 0.05 * rnd()); in.vout = (float)(vin + 0.2 + inj + 0.1 * rnd()); in.meas_valid = 1U; in.vin_in_window = 1U; in.maxc_v = (float)maxc; in.hw_cap_ticks = 2752U;
        Tune_Service(t, &in, &out); req = out.request_ticks;
        while (Tune_TakeFrame(&f)) printf("%5.1f s  %3u  %5u  %6.1f V  %+7.1f V   %s\n", (double)(t - t0) / 1000.0, (unsigned)f.step, (unsigned)f.ticks, (double)f.vin_dv / 10.0, (double)f.dv_dv / 10.0, rn[f.result > 6U ? 6U : f.result]);
        if (out.done) break;
    }
    { TuneProfile_t p; if (Tune_GetProfile(&p) != 0U)
      {
        printf("\nPROFILE STORED:\n  cap %u ticks%s\n  regulator gain scale %.3f (boost) %.3f (buck)   [1.000 = like the bench transformer]\n  measured at %.1f V, Max Correction %.1f V\n", (unsigned)p.cap_ticks, p.knee ? " (limited by a saturation knee)" : "", (double)p.kib_m / 1000.0, (double)p.kiu_m / 1000.0, (double)p.vin_ref_dv / 10.0, (double)p.maxc_dv / 10.0);
        printf("  reach for a 230 V set voltage: from %.1f V to %.1f V of input\n  what it predicts at the cap: %.1f V of injection at %.0f V\n", (double)Tune_ReachLoDv(&p, 230.0f) / 10.0, (double)Tune_ReachHiDv(&p, 230.0f) / 10.0, (double)Tune_PredictDv(&p, (float)p.cap_ticks, (float)vin), vin);
      } else printf("\nNOTHING STORED (the sweep stopped).\n"); }
    return 0;
}
