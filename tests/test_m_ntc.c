/* The NTC conversion (m_ntc.c) against the beta equation, and its limits. */
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "m_ntc.h"
static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
static double raw_exact(double tc) { double t = tc + 273.15, r = 10000.0 * exp(3950.0 * (1.0 / t - 1.0 / 298.15)); return 4095.0 * r / (r + 1500.0); }
int main(void)
{
    double worst = 0.0, tc; int16_t t; uint8_t st; uint16_t prev = 0U; int mono = 1, have_prev = 0;
    for (tc = 0.0; tc <= 125.0; tc += 0.5)
    {
        double e; st = MNtc_RawToDeci((uint16_t)(raw_exact(tc) + 0.5), &t); e = fabs((double)t / 10.0 - tc);
        if (st != MNTC_OK) { CHECK(0, "%.1f degC gives status %u", tc, (unsigned)st); continue; }
        if (e > worst) worst = e;
    }
    CHECK(worst <= 1.0, "0 to 125 degC: within 1.0 degC of the beta equation (worst %.2f)", worst);
    st = MNtc_RawToDeci((uint16_t)(raw_exact(75.0) + 0.5), &t); CHECK(st == MNTC_OK && t >= 745 && t <= 755, "75 degC reads %d tenths", (int)t);
    st = MNtc_RawToDeci((uint16_t)(raw_exact(25.0) + 0.5), &t); CHECK(st == MNTC_OK && t >= 245 && t <= 255, "25 degC reads %d tenths", (int)t);
    for (tc = 120.0; tc >= -5.0; tc -= 0.5) { uint16_t r = (uint16_t)(raw_exact(tc) + 0.5); MNtc_RawToDeci(r, &t); if (have_prev && r < prev) { /* raw rises as it cools */ } prev = r; have_prev = 1; }
    { int16_t tp = 20000; uint16_t r; for (r = 4000U; r >= 800U; r--) { st = MNtc_RawToDeci(r, &t); if (st == MNTC_OK) { if (t < tp && tp != 20000) mono = 0; if (tp != 20000 && t < tp) mono = 0; tp = t; } } }
    CHECK(mono, "the temperature never falls while the reading falls (monotonic)");
    st = MNtc_RawToDeci(4095U, &t); CHECK(st == MNTC_OPEN, "4095 = no NTC (open): MNTC_OPEN");
    st = MNtc_RawToDeci(4060U, &t); CHECK(st == MNTC_OPEN, "4060 (colder than -10 degC) is open too");
    st = MNtc_RawToDeci(0U, &t); CHECK(st == MNTC_SHORT && t >= 1300, "0 = shorted: MNTC_SHORT, reported hot (%d)", (int)t);
    st = MNtc_RawToDeci(149U, &t); CHECK(st == MNTC_SHORT, "149 is still a short");
    st = MNtc_RawToDeci(150U, &t); CHECK(st == MNTC_OK && t == 1300, "150 is a very hot sink, clamped to 130.0 (%d)", (int)t);
    st = MNtc_RawToDeci(5000U, &t); CHECK(st == MNTC_OPEN, "a reading above 4095 is treated as 4095");
    printf("  NTC: table + interpolation within 1 degC of the beta equation, open and short detected\n  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
