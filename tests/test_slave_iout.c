/* Tests for the I-OUT conversion: the numbers of the schematic's note, a synthetic sine through the whole chain, clipping, offset, rounding. */
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "slave_iout.h"
static int tot = 0, bad = 0;
#define CHECK(c, ...) do { tot++; if (!(c)) { bad++; printf("      FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
/* a sine of the given primary RMS current through the analogue chain and a 12-bit ADC, 400 samples over one mains cycle; returns the statistics */
static void adc_window(double irms, double offset_counts, double noise, uint32_t *seed, float *rms, float *mean, uint16_t *mn, uint16_t *mx)
{
    double s = 0.0, s2 = 0.0; int i, n = 400; uint32_t lo = 9999U, hi = 0U;
    for (i = 0; i < n; i++)
    {
        double a = 6.283185307179586 * (double)i / (double)n, ip = irms * 1.41421356237 * sin(a);
        double v = 1.65 + (double)IOUT_V_PER_A * ip, c;
        double r;
        *seed = *seed * 1664525U + 1013904223U; r = (double)((*seed >> 8) & 0xFFFFU) / 32768.0 - 1.0;
        if (v > 3.3) v = 3.3;
        if (v < 0.0) v = 0.0;                                         /* the amplifier cannot go beyond the rails */
        c = v / 3.3 * 4095.0 + offset_counts + noise * r; if (c < 0.0) c = 0.0;
        if (c > 4095.0) c = 4095.0;
        c = floor(c + 0.5);
        s += c; s2 += c * c; if ((uint32_t)c < lo) lo = (uint32_t)c; if ((uint32_t)c > hi) hi = (uint32_t)c;
    }
    *mean = (float)(s / n); *rms = (float)sqrt(s2 / n - (s / n) * (s / n)); *mn = (uint16_t)lo; *mx = (uint16_t)hi;
}
int main(void)
{
    IoutResult_t r; float rms, mean; uint16_t mn, mx; uint32_t seed = 1U; double t, apc = (double)IOUT_A_PER_COUNT, clip_rms = (double)IOUT_PK_LIMIT_A / 1.41421356237;
    int is68 = (fabs((double)IOUT_BURDEN_R_EACH_OHM - 68.0) < 0.01), is180 = (fabs((double)IOUT_BURDEN_R_EACH_OHM - 180.0) < 0.01), is120 = (fabs((double)IOUT_BURDEN_R_EACH_OHM - 120.0) < 0.01);
    printf("  burden under test: %.0f ohm x 3 in parallel = %.2f ohm: %.4f V/A, %.2f mA per count, clips at +-%.1f A peak, hardware trip about %.0f A peak\n", (double)IOUT_BURDEN_R_EACH_OHM, (double)IOUT_BURDEN_OHM, (double)IOUT_V_PER_A, apc * 1000.0, (double)IOUT_PK_LIMIT_A, (double)IOUT_TRIP_A);
    /* the constants follow from the circuit: 6.8 x R_burden / 2500 V per ampere; counts of 3.3 V / 4095 */
    CHECK(fabs((double)IOUT_V_PER_A - 6.8 * ((double)IOUT_BURDEN_R_EACH_OHM / 3.0) / 2500.0) < 1e-7, "V per ampere = 6.8 x burden / 2500 (%.5f)", (double)IOUT_V_PER_A);
    CHECK(fabs(apc - (3.3 / 4095.0) / (double)IOUT_V_PER_A) < 1e-9, "amperes per count follow from it (%.6f A)", apc);
    if (is68) CHECK(fabs((double)IOUT_V_PER_A - 0.06165) < 2e-5 && fabs(apc - 0.013072) < 5e-5 && fabs((double)IOUT_PK_LIMIT_A - 26.76) < 0.05 && fabs((double)IOUT_TRIP_A - 108.3) < 0.5, "68 ohm x 3: 0.0617 V/A, 13.1 mA per count, clips at 26.8 A peak, trips at about 108 A peak");
    if (is120) CHECK(fabs((double)IOUT_V_PER_A - 0.1088) < 1e-5 && fabs(apc - 0.007407) < 2e-5 && fabs((double)IOUT_PK_LIMIT_A - 15.17) < 0.05 && fabs((double)IOUT_TRIP_A - 61.4) < 0.3, "120 ohm x 3 (v0.9.74, production): 0.1088 V/A, 7.41 mA per count, clips at 15.2 A peak, trips at about 61 A peak");
    CHECK(is68 || is120 || is180, "the burden under test is one of the known boards (68, 120 or 180 ohm)");
    if (is180) CHECK(fabs((double)IOUT_V_PER_A - 0.1632) < 1e-5 && fabs(apc - 0.004937) < 2e-5 && fabs((double)IOUT_PK_LIMIT_A - 10.11) < 0.05 && fabs((double)IOUT_TRIP_A - 40.9) < 0.3, "180 ohm x 3: 0.1632 V/A, 4.94 mA per count, clips at 10.1 A peak, trips at about 41 A peak");
    /* a 5.89 A RMS load (the original board's full load): 8.33 A peak, inside the range for either burden */
    adc_window(5.89, 0.0, 0.0, &seed, &rms, &mean, &mn, &mx); Iout_Compute(rms, mean, mn, mx, &r);
    CHECK(fabs((double)r.irms_a - 5.89) < 3.0 * apc + 0.02 && fabs((double)r.ipk_a - 8.33) < 3.0 * apc + 0.04 && r.clipped == 0U, "5.89 A RMS: reads %.2f A RMS and %.2f A peak, not clipped", (double)r.irms_a, (double)r.ipk_a);
    CHECK(mn > 5U && mx < 4090U && (int)(mx - mn) > 100, "its ADC swing stays inside the range (%u .. %u counts of 4095)", (unsigned)mn, (unsigned)mx);
    /* the whole usable range: the error stays within the quantisation, and a DC offset of the whole chain does not matter */
    { double worst = 0.0, top = 0.7 * clip_rms; int k; for (k = 0; k < 14; k++) { double ia = 0.05 + (top - 0.05) * (double)k / 13.0; adc_window(ia, 25.0, 0.0, &seed, &rms, &mean, &mn, &mx); Iout_Compute(rms, mean, mn, mx, &r); { double e = fabs((double)r.irms_a - ia) - 0.004 * ia; if (e > worst) worst = e; } }
      CHECK(worst < 3.0 * apc, "0.05 to %.1f A RMS with a 25 count DC offset: the worst error is %.3f A, within 3 counts (%.3f A): the offset cancels", top, worst, 3.0 * apc); }
    /* noise: 3 counts of noise on a 1 A signal */
    adc_window(1.0, 0.0, 3.0, &seed, &rms, &mean, &mn, &mx); Iout_Compute(rms, mean, mn, mx, &r);
    CHECK(fabs((double)r.irms_a - 1.0) < 4.0 * apc + 0.01, "1 A with 3 counts of noise: %.3f A", (double)r.irms_a);
    /* zero current: about nothing */
    adc_window(0.0, 0.0, 0.0, &seed, &rms, &mean, &mn, &mx); Iout_Compute(rms, mean, mn, mx, &r);
    CHECK(r.irms_a < 0.001f && r.ipk_a < 0.02f && r.clipped == 0U, "no current: %.4f A", (double)r.irms_a);
    /* beyond the range the amplifier clips: the reading is flagged, never trusted */
    adc_window(1.25 * clip_rms, 0.0, 0.0, &seed, &rms, &mean, &mn, &mx); Iout_Compute(rms, mean, mn, mx, &r);
    CHECK(r.clipped == 1U && (double)r.irms_a < 1.25 * clip_rms, "%.1f A RMS (25 %% beyond the %.1f A the amplifier can show): clipped, and the reading is below the truth (%.2f A)", 1.25 * clip_rms, clip_rms, (double)r.irms_a);
    adc_window(0.97 * clip_rms, 0.0, 0.0, &seed, &rms, &mean, &mn, &mx); Iout_Compute(rms, mean, mn, mx, &r);
    CHECK(r.clipped == 0U && fabs((double)r.irms_a - 0.97 * clip_rms) < 0.02 * clip_rms, "%.1f A RMS (97 %% of the range) just fits: not clipped, %.2f A", 0.97 * clip_rms, (double)r.irms_a);
    /* an unsymmetrical waveform: the peak is the larger side */
    Iout_Compute(100.0f, 2048.0f, 1900U, 2548U, &r); CHECK(fabs((double)r.ipk_a - 500.0 * apc) < 1e-4, "an unsymmetrical window: the peak is the larger excursion from the mean (%.3f A)", (double)r.ipk_a);
    Iout_Compute(100.0f, 2048.0f, 1500U, 2100U, &r); CHECK(fabs((double)r.ipk_a - 548.0 * apc) < 1e-4, "and on the other side (%.3f A)", (double)r.ipk_a);
    Iout_Compute(-3.0f, 2048.0f, 2050U, 2040U, &r); CHECK(r.irms_a == 0.0f && r.ipk_a >= 0.0f, "a negative or inverted input never gives a negative current");
    /* the field range of the $E fields (0 .. 40.00 A): the channel's own range is smaller, so nothing is lost */
    CHECK(Iout_ToCentiAmps(5.89f) == 589U && Iout_ToCentiAmps(0.004f) == 0U && Iout_ToCentiAmps(0.006f) == 1U && Iout_ToCentiAmps(-1.0f) == 0U && Iout_ToCentiAmps(99.0f) == 4000U, "hundredths of an ampere: 5.89 A = 589, rounding, negative = 0, over 40 A = 4000");
    CHECK((double)IOUT_PK_LIMIT_A < 40.0, "the largest peak the channel can show (%.1f A) fits the 40.00 A of the $E field", (double)IOUT_PK_LIMIT_A);
    for (t = 0.0; t < 40.0; t += 0.37) { uint16_t c = Iout_ToCentiAmps((float)t); if (fabs((double)c / 100.0 - t) > 0.0051) { CHECK(0, "%.3f A became %u", t, (unsigned)c); break; } }
    printf("  I-OUT conversion: the circuit's constants for the burden under test, a synthetic sine through the chain, offset, noise, clipping, the $E field range\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", tot, bad, bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
