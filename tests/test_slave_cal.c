/* Tests for slave_cal: the voltage calibration logic against a simulated measurement chain and a flash that can fail. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "slave_cal.h"
#include "link_v2.h"
static int tot = 0, bad = 0;
#define CHECK(c, ...) do { tot++; if (!(c)) { bad++; printf("      FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* the simulated chain: raw counts = true volts / true gain; the slave's NOMINAL gain is 0.1 V per count, the true chain differs per channel */
static double true_v[2] = { 226.8, 226.8 }, true_gain[2] = { 0.1, 0.1 }; static double noise = 0.0; static int stopped = 1, meas_valid = 1; static uint32_t counter; static int frozen = 0;
static uint16_t applied[2] = { 10000U, 10000U }; static int set_calls = 0; static uint16_t stored[2]; static int have_store = 0, save_fail = 0, saves = 0; static uint32_t rs = 7U;
static double frand(void) { rs = rs * 1664525U + 1013904223U; return (double)((rs >> 8) & 0xFFFFU) / 65535.0 * 2.0 - 1.0; }
static float h_raw(uint8_t ch, uint8_t *valid) { *valid = (uint8_t)meas_valid; return (float)(true_v[ch] * (1.0 + noise * frand()) / true_gain[ch]); }
static uint32_t h_cnt(void) { return counter; }
static uint8_t h_stopped(void) { return (uint8_t)stopped; }
static float h_nominal(uint8_t ch) { (void)ch; return 0.1f; }
static void h_set(uint8_t ch, uint16_t r) { applied[ch] = r; set_calls++; }
static int h_load(uint16_t r[2]) { if (!have_store) return -1; r[0] = stored[0]; r[1] = stored[1]; return 0; }
static int h_save(const uint16_t r[2]) { if (save_fail) return 20310080;  stored[0] = r[0]; stored[1] = r[1]; have_store = 1; saves++; return 0; }
static const SlaveCalHw_t HW = { h_raw, h_cnt, h_stopped, h_nominal, h_set, h_load, h_save };
/* run one command to its answer: the main loop polls every few ms, new results every 10 ms */
static int run(uint8_t cmd, int32_t dv, char letter, uint8_t *st, int32_t *val, uint32_t *t_ms)
{
    uint32_t t = 100000U, k;
    if (SlaveCal_Command(cmd, dv, letter, st, val, t) == 1) { *t_ms = 0U; return 1; }
    for (k = 0U; k < 400U; k++) { t += 5U; if ((t % 10U) == 0U && !frozen) counter++; if (SlaveCal_Poll(st, val, t) == 1) { *t_ms = t - 100000U; return 1; } }
    return 0;
}
static void reset_world(void) { true_v[0] = true_v[1] = 226.8; true_gain[0] = true_gain[1] = 0.1; noise = 0.0; stopped = 1; meas_valid = 1; frozen = 0; have_store = 0; save_fail = 0; saves = 0; set_calls = 0; applied[0] = applied[1] = 10000U; counter = 0U; SlaveCal_Init(&HW); }

int main(void)
{
    uint8_t st; int32_t v; uint32_t t;
    /* power-up with nothing stored: nominal, applied to both channels */
    reset_world(); CHECK(SlaveCal_Ratio(0) == 10000U && SlaveCal_Ratio(1) == 10000U && applied[0] == 10000U && applied[1] == 10000U && set_calls == 2, "no stored record: both channels nominal and applied");
    /* the example of the protocol sheet: the meter reads 230.2 V where the nominal chain reads 226.8 V: ratio 1.0150 */
    reset_world(); true_gain[0] = 0.1 * (226.8 / 230.2);                                   /* the real chain reads the true 226.8 V... */
    true_v[0] = 230.2; true_gain[0] = 0.1 * (230.2 / 226.8) * 0.0 + 0.1; true_v[0] = 226.8;            /* raw counts give 226.8 V at the nominal gain */
    CHECK(run(SCAL_CMD_CALV, 2302, 0, &st, &v, &t) == 1 && st == LINKV2_ST_OK, "CALV 230.2 V (the nominal chain reads 226.8 V): accepted after the averaging (status %u)", st);
    CHECK(v >= 10148 && v <= 10152 && SlaveCal_Ratio(0) == (uint16_t)v && applied[0] == (uint16_t)v && SlaveCal_Ratio(1) == 10000U && applied[1] == 10000U, "the total correction is +1.5 %% (%d), applied to the input only", (int)v);
    CHECK(t >= 1000U && t <= 1100U && saves == 1 && stored[0] == (uint16_t)v && stored[1] == 10000U, "it took the 1 s averaging (%u ms) and stored both ratios once", (unsigned)t);
    /* a power cycle: the stored ratios come back and are applied */
    applied[0] = applied[1] = 10000U; SlaveCal_Init(&HW); CHECK(SlaveCal_Ratio(0) == (uint16_t)v && applied[0] == (uint16_t)v && applied[1] == 10000U, "after a restart the stored calibration is applied again");
    /* the output channel is independent */
    reset_world(); CHECK(run(SCAL_CMD_CALO, 2250, 0, &st, &v, &t) == 1 && st == LINKV2_ST_OK && SlaveCal_Ratio(1) == (uint16_t)v && SlaveCal_Ratio(0) == 10000U && v >= 9915 && v <= 9930, "CALO 225.0 V against 226.8 V: -0.8 %% (%d), the output only", (int)v);
    /* repeating the same calibration gives the same ratio: it is always against NOMINAL, it does not compound */
    { int32_t v1 = v; CHECK(run(SCAL_CMD_CALO, 2250, 0, &st, &v, &t) == 1 && v == v1, "the same calibration twice: the same ratio (%d, %d)", (int)v1, (int)v); }
    /* refused: the unit is not stopped */
    reset_world(); stopped = 0; CHECK(run(SCAL_CMD_CALV, 2302, 0, &st, &v, &t) == 1 && st == LINKV2_ST_NOT_STOPPED && t == 0U && saves == 0 && SlaveCal_Ratio(0) == 10000U, "not stopped: refused at once, nothing changed");
    /* the unit gets a run command during the averaging: nothing is changed */
    reset_world(); { uint32_t k, tt = 100000U; SlaveCal_Command(SCAL_CMD_CALV, 2302, 0, &st, &v, tt); for (k = 0U; k < 60U; k++) { tt += 5U; if (tt % 10U == 0U) counter++; SlaveCal_Poll(&st, &v, tt); } stopped = 0; tt += 5U; CHECK(SlaveCal_Poll(&st, &v, tt) == 1 && st == LINKV2_ST_NOT_STOPPED && saves == 0 && SlaveCal_Ratio(0) == 10000U, "a run command during the averaging: NOT_STOPPED, nothing stored"); }
    /* the reading is too low */
    reset_world(); true_v[0] = 30.0; CHECK(run(SCAL_CMD_CALV, 500, 0, &st, &v, &t) == 1 && st == LINKV2_ST_BAD_SIGNAL && saves == 0, "the chain reads only 30 V (below 50 V): BAD_SIGNAL, whatever the meter says");
    /* the correction is out of range */
    reset_world(); CHECK(run(SCAL_CMD_CALV, 2600, 0, &st, &v, &t) == 1 && st == LINKV2_ST_OUT_OF_RANGE && v > 11000 && saves == 0 && SlaveCal_Ratio(0) == 10000U, "260.0 V against a reading of 226.8 V (+14.6 %%): OUT_OF_RANGE, nothing stored (ratio %d)", (int)v);
    reset_world(); CHECK(run(SCAL_CMD_CALV, 2000, 0, &st, &v, &t) == 1 && st == LINKV2_ST_OUT_OF_RANGE, "200.0 V against 226.8 V (-11.8 %%): OUT_OF_RANGE");
    reset_world(); CHECK(run(SCAL_CMD_CALV, 2494, 0, &st, &v, &t) == 1 && st == LINKV2_ST_OK && v >= 10995 && v <= 11000, "+9.97 %% (249.4 V against 226.8 V) is just inside and is accepted (%d)", (int)v);
    reset_world(); CHECK(run(SCAL_CMD_CALV, 2496, 0, &st, &v, &t) == 1 && st == LINKV2_ST_OUT_OF_RANGE, "+10.05 %% (249.6 V) is just outside: OUT_OF_RANGE");
    reset_world(); CHECK(run(SCAL_CMD_CALV, 2042, 0, &st, &v, &t) == 1 && st == LINKV2_ST_OK && v >= 9000 && v <= 9006, "-9.97 %% (204.2 V) is just inside and is accepted (%d)", (int)v);
    reset_world(); CHECK(run(SCAL_CMD_CALV, 2040, 0, &st, &v, &t) == 1 && st == LINKV2_ST_OUT_OF_RANGE, "-10.05 %% (204.0 V) is just outside: OUT_OF_RANGE");
    /* an unsteady input */
    reset_world(); noise = 0.05; CHECK(run(SCAL_CMD_CALV, 2268, 0, &st, &v, &t) == 1 && st == LINKV2_ST_BAD_SIGNAL && saves == 0, "an input that swings +-5 %% during the second: BAD_SIGNAL");
    reset_world(); noise = 0.004; CHECK(run(SCAL_CMD_CALV, 2268, 0, &st, &v, &t) == 1 && st == LINKV2_ST_OK, "+-0.4 %% of ordinary noise is fine (status %u)", st);
    /* no new results (the measurement stopped) */
    reset_world(); frozen = 1; CHECK(run(SCAL_CMD_CALV, 2268, 0, &st, &v, &t) == 1 && st == LINKV2_ST_BAD_SIGNAL, "no new measurement results: BAD_SIGNAL");
    reset_world(); meas_valid = 0; CHECK(run(SCAL_CMD_CALV, 2268, 0, &st, &v, &t) == 1 && st == LINKV2_ST_BAD_SIGNAL, "a measurement that is not valid: BAD_SIGNAL");
    /* bad arguments */
    reset_world(); CHECK(run(SCAL_CMD_CALV, 499, 0, &st, &v, &t) == 1 && st == LINKV2_ST_BAD_ARG && run(SCAL_CMD_CALV, 4001, 0, &st, &v, &t) == 1 && st == LINKV2_ST_BAD_ARG, "a meter value outside 50.0 to 400.0 V: BAD_ARG");
    reset_world(); CHECK(run(9, 2302, 0, &st, &v, &t) == 1 && st == LINKV2_ST_BAD_ARG, "an unknown command code: BAD_ARG");
    /* a failed store: the old value stays, FLASH_FAIL */
    reset_world(); CHECK(run(SCAL_CMD_CALV, 2302, 0, &st, &v, &t) == 1 && st == LINKV2_ST_OK, "a good calibration first"); { uint16_t good = SlaveCal_Ratio(0); save_fail = 1; CHECK(run(SCAL_CMD_CALV, 2250, 0, &st, &v, &t) == 1 && st == LINKV2_ST_FLASH_FAIL && SlaveCal_Ratio(0) == good && applied[0] == good, "the flash write fails: FLASH_FAIL and the OLD calibration stays in force (applied %u)", (unsigned)applied[0]); CHECK(v == 20310080, "the answer's value carries the flash diagnostic (%d)", (int)v); }
    /* one at a time */
    reset_world(); { uint32_t tt = 100000U, k; uint8_t st2; int32_t v2; SlaveCal_Command(SCAL_CMD_CALV, 2302, 0, &st, &v, tt); CHECK(SlaveCal_Busy() == 1U, "busy while averaging"); CHECK(SlaveCal_Command(SCAL_CMD_CALO, 2302, 0, &st2, &v2, tt) == 1 && st2 == LINKV2_ST_NOT_ALLOWED, "a second command while one is averaging: NOT_ALLOWED");
      for (k = 0U; k < 300U; k++) { tt += 5U; if (tt % 10U == 0U) counter++; if (SlaveCal_Poll(&st, &v, tt) == 1) break; } CHECK(st == LINKV2_ST_OK && SlaveCal_Busy() == 0U, "the first one still finishes"); }
    /* reset */
    reset_world(); (void)run(SCAL_CMD_CALV, 2302, 0, &st, &v, &t); (void)run(SCAL_CMD_CALO, 2250, 0, &st, &v, &t);
    CHECK(run(SCAL_CMD_CALR, 0, 'V', &st, &v, &t) == 1 && st == LINKV2_ST_OK && v == 10000 && SlaveCal_Ratio(0) == 10000U && SlaveCal_Ratio(1) != 10000U && stored[0] == 10000U, "CALR V: the input back to nominal, the output kept, stored");
    CHECK(run(SCAL_CMD_CALR, 0, 'O', &st, &v, &t) == 1 && st == LINKV2_ST_OK && SlaveCal_Ratio(1) == 10000U, "CALR O: the output back to nominal");
    CHECK(run(SCAL_CMD_CALR, 0, 'X', &st, &v, &t) == 1 && st == LINKV2_ST_BAD_ARG, "CALR with another letter: BAD_ARG");
    stopped = 0; CHECK(run(SCAL_CMD_CALR, 0, 'V', &st, &v, &t) == 1 && st == LINKV2_ST_NOT_STOPPED, "CALR while running: NOT_STOPPED");
    /* a corrupt stored record outside the accepted range is ignored */
    reset_world(); have_store = 1; stored[0] = 5000U; stored[1] = 10000U; SlaveCal_Init(&HW); CHECK(SlaveCal_Ratio(0) == 10000U && SlaveCal_Ratio(1) == 10000U, "a stored ratio of 0.50 is ignored: nominal");
    /* a long run of random calibrations: the ratio in force is always inside the accepted range */
    { int ok = 1, k; reset_world(); for (k = 0; k < 300; k++) { int meter = 1500 + (int)(frand() * 800.0 + 800.0); true_v[0] = 150.0 + 100.0 * (frand() + 1.0); (void)run(SCAL_CMD_CALV, meter, 0, &st, &v, &t); if (SlaveCal_Ratio(0) < SCAL_RATIO_MIN || SlaveCal_Ratio(0) > SCAL_RATIO_MAX) ok = 0; }
      CHECK(ok, "300 random calibrations, accepted or refused: the ratio in force is always 0.90 to 1.10"); }
    printf("  voltage calibration logic: single point against nominal, 1 s average, refused when running / too low / unsteady / out of range, flash failure keeps the old value, survives a restart\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", tot, bad, bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
