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
    IoutResult_t r; float rms, mean; uint16_t mn, mx; uint32_t seed = 1U; double t;
    /* the constants: 6.8 x 60 / 2500 = 0.1632 V/A, about 4.94 mA per count */
    CHECK(fabs((double)IOUT_V_PER_A - 0.1632) < 1e-6, "0.1632 V per ampere (%.5f)", (double)IOUT_V_PER_A);
    CHECK(fabs((double)IOUT_A_PER_COUNT - 0.004937) < 2e-5, "about 4.94 mA per ADC count (%.6f A)", (double)IOUT_A_PER_COUNT);
    /* the full load of the board: 5.89 A RMS, 8.33 A peak, 1.36 V of swing */
    adc_window(5.89, 0.0, 0.0, &seed, &rms, &mean, &mn, &mx); Iout_Compute(rms, mean, mn, mx, &r);
    CHECK(fabs((double)r.irms_a - 5.89) < 0.06 && fabs((double)r.ipk_a - 8.33) < 0.08 && r.clipped == 0U, "the board's full load, 5.89 A RMS: reads %.2f A RMS and %.2f A peak, not clipped", (double)r.irms_a, (double)r.ipk_a);
    CHECK(mn > 340U && mx < 3750U, "its ADC swing stays inside the range (%u .. %u counts of 4095)", (unsigned)mn, (unsigned)mx);
    /* a range of currents: the error stays within the quantisation, and a DC offset of the whole chain does not matter */
    { double worst = 0.0; int k; for (k = 0; k < 14; k++) { double ia = 0.05 + 0.5 * k; adc_window(ia, 25.0, 0.0, &seed, &rms, &mean, &mn, &mx); Iout_Compute(rms, mean, mn, mx, &r); if (fabs((double)r.irms_a - ia) > worst) worst = fabs((double)r.irms_a - ia); }
      CHECK(worst < 0.03, "0.05 to 6.5 A RMS with a 25 count DC offset: the worst error is %.3f A (the offset cancels)", worst); }
    /* noise: 3 counts of noise on a 1 A signal */
    adc_window(1.0, 0.0, 3.0, &seed, &rms, &mean, &mn, &mx); Iout_Compute(rms, mean, mn, mx, &r);
    CHECK(fabs((double)r.irms_a - 1.0) < 0.04, "1 A with 3 counts of noise: %.3f A", (double)r.irms_a);
    /* zero current: about nothing */
    adc_window(0.0, 0.0, 0.0, &seed, &rms, &mean, &mn, &mx); Iout_Compute(rms, mean, mn, mx, &r);
    CHECK(r.irms_a < 0.001f && r.ipk_a < 0.01f && r.clipped == 0U, "no current: %.4f A", (double)r.irms_a);
    /* beyond the range the amplifier clips: the reading is flagged, never trusted */
    adc_window(9.0, 0.0, 0.0, &seed, &rms, &mean, &mn, &mx); Iout_Compute(rms, mean, mn, mx, &r);
    CHECK(r.clipped == 1U && (double)r.irms_a < 9.0, "9 A RMS (12.7 A peak, beyond the 10.1 A the amplifier can show): clipped, and the reading is below the truth (%.2f A)", (double)r.irms_a);
    adc_window(7.0, 0.0, 0.0, &seed, &rms, &mean, &mn, &mx); Iout_Compute(rms, mean, mn, mx, &r);
    CHECK(r.clipped == 0U && fabs((double)r.irms_a - 7.0) < 0.1, "7 A RMS (9.9 A peak) just fits: not clipped, %.2f A", (double)r.irms_a);
    /* an unsymmetrical waveform: the peak is the larger side */
    Iout_Compute(100.0f, 2048.0f, 1900U, 2548U, &r); CHECK(fabs((double)r.ipk_a - 500.0 * (double)IOUT_A_PER_COUNT) < 1e-4, "an unsymmetrical window: the peak is the larger excursion from the mean (%.3f A)", (double)r.ipk_a);
    Iout_Compute(100.0f, 2048.0f, 1500U, 2100U, &r); CHECK(fabs((double)r.ipk_a - 548.0 * (double)IOUT_A_PER_COUNT) < 1e-4, "and on the other side (%.3f A)", (double)r.ipk_a);
    Iout_Compute(-3.0f, 2048.0f, 2050U, 2040U, &r); CHECK(r.irms_a == 0.0f && r.ipk_a >= 0.0f, "a negative or inverted input never gives a negative current");
    /* the field range of the $E fields */
    CHECK(Iout_ToCentiAmps(5.89f) == 589U && Iout_ToCentiAmps(0.004f) == 0U && Iout_ToCentiAmps(0.006f) == 1U && Iout_ToCentiAmps(-1.0f) == 0U && Iout_ToCentiAmps(99.0f) == 4000U, "hundredths of an ampere: 5.89 A = 589, rounding, negative = 0, over 40 A = 4000");
    for (t = 0.0; t < 40.0; t += 0.37) { uint16_t c = Iout_ToCentiAmps((float)t); if (fabs((double)c / 100.0 - t) > 0.0051) { CHECK(0, "%.3f A became %u", t, (unsigned)c); break; } }
    printf("  I-OUT conversion: the schematic's constants, a synthetic sine through the chain, offset, noise, clipping, the $E field range\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", tot, bad, bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
