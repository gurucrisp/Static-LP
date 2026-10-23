/* Tests for the autotune over the link: $K TUNE / TABORT as BYTES, the $T progress frames, the BUSY / NOT_TUNED flags in $E, the refusals with their reasons, the profile stored before the result is reported. The real link library,
 * link layer, tune core and profile store, against a simulated transformer and a flash that can refuse. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "slave_link2.h"
#include "slave_tune_link.h"
#include "slave_profile.h"
static int tot = 0, bad = 0;
#define CHECK(c, ...) do { tot++; if (!(c)) { bad++; printf("      FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- the flash of the profile store (a RAM model that can refuse) ---- */
#define PA 0x1000UL
#define PB 0x1800UL
static uint8_t mem[4096]; static int flash_fail;
static int frd(uint32_t a, uint8_t *b, uint32_t n) { memcpy(b, &mem[a - PA], n); return 0; }
static int fer(uint32_t a) { if (flash_fail) return -1; memset(&mem[a - PA], 0xFF, 2048U); return 0; }
static int fpr(uint32_t a, const uint8_t *b, uint32_t n) { uint32_t i; if (flash_fail) return -1; for (i = 0U; i < n; i++) mem[a - PA + i] &= b[i]; return 0; }
static const SFlash_t FL = { frd, fpr, fer };

/* ---- what the controller side would say ---- */
static uint16_t maxc_dv = 1200U; static uint8_t is_stopped = 1U, can_run = 1U, other_busy = 0U;
static uint16_t hw_maxc(void) { return maxc_dv; } static uint8_t hw_stopped(void) { return is_stopped; } static uint8_t hw_can_run(void) { return can_run; } static uint8_t hw_busy(void) { return other_busy; }
static const TuneLinkHw_t HW = { hw_maxc, hw_stopped, hw_can_run, hw_busy };

/* ---- the link: captured frames ---- */
#define MAXCAP 1200
static char cap[MAXCAP][100]; static int cap_n[MAXCAP]; static int ncap, tx_ok = 1;
static uint8_t h_tx(const char *f, uint32_t n) { if (!tx_ok) return 0U; if (ncap < MAXCAP && n < 100U) { memcpy(cap[ncap], f, n); cap_n[ncap] = (int)n; ncap++; } return 1U; }
static void h_on_control(const LinkM_t *m, uint32_t now) { (void)m; (void)now; }
static void h_fill_e(LinkE_t *e) { e->setv_dv = 2300U; e->flags2 = SlaveTuneLink_Flags2(); }
static void h_fill_h(LinkH_t *h) { h->fw = 931UL; h->vin_min_dv = 800U; h->vin_max_dv = 3000U; h->tuned = SlaveProfile_Tuned(); h->vout_max_dv = 2800U; }
static int h_tune_cmd(uint8_t c, const char *n, int32_t v, uint8_t *s, int32_t *val, uint32_t now) { return SlaveTuneLink_Command(c, n, v, s, val, now); }
static int h_tune_frame(LinkT_t *f, uint32_t now) { return SlaveTuneLink_NextFrame(f, now); }
static const SL2_Hooks_t HK = { h_on_control, h_fill_e, h_fill_h, h_tx, 0, 0, h_tune_cmd, h_tune_frame };

/* ---- the simulated transformer ---- */
typedef struct { double k, dead, knee, sign, noise, vin; } Plant_t;
static Plant_t P; static double inj; static uint32_t T; static uint32_t rs = 99U; static double gauss(void) { double a = 0.0; int i; for (i = 0; i < 3; i++) { rs = rs * 1664525U + 1013904223U; a += (double)((rs >> 8) & 0xFFFFFFU) / 8388608.0 - 1.0; } return a / 1.732; }
static double true_inj(double ticks)
{
    double eff = (ticks > P.dead) ? (ticks - P.dead) : 0.0;
    if (P.knee > 0.0 && eff > P.knee) eff = P.knee + 0.25 * (eff - P.knee);
    return P.sign * P.k * (double)Tune_RefG((float)P.vin) * eff;
}
static void plant_reset(void) { P.k = 1.0; P.dead = 40.0; P.knee = 0.0; P.sign = 1.0; P.noise = 0.1; P.vin = 230.0; inj = 0.0; }
static void step_ms(uint32_t ms)
{
    uint32_t k;
    for (k = 0U; k < ms / 10U; k++)
    {
        TuneIn_t in; double a = 1.0 - exp(-0.01 / 0.3);
        T += 10U; inj += (true_inj((double)SlaveTuneLink_RequestTicks()) - inj) * a;
        in.vin = (float)(P.vin + 0.05 * gauss()); in.vout = (float)(P.vin + 0.2 + inj + P.noise * gauss()); in.meas_valid = 1U; in.fault = 0U; in.vin_in_window = 1U; in.maxc_v = (float)maxc_dv / 10.0f; in.hw_cap_ticks = 2752U;
        SlaveTuneLink_Service(T, &in); SL2_Service(T);
    }
}
static void send_kn(uint8_t id, LinkCmd_t cmd, const char *name, int32_t num)
{
    char b[LINKV2_MAX_LEN + 8]; LinkK_t k; int32_t n, i; memset(&k, 0, sizeof k); k.id = id; k.cmd = cmd; k.num = num; if (name != 0) { for (i = 0; i < 7 && name[i]; i++) k.name[i] = name[i]; }
    n = LinkV2_BuildK(b, sizeof b, &k); for (i = 0; i < n; i++) SL2_Rx((uint8_t)b[i], T);
    SL2_Service(T);
}
static void send_k(uint8_t id, LinkCmd_t cmd) { send_kn(id, cmd, 0, 0); }
/* what the master has seen */
static int find_r(uint8_t id, int *status, int32_t *value) { int i, c = 0; LinkV2Frame_t f; for (i = 0; i < ncap; i++) if (LinkV2_Parse(cap[i], (uint32_t)cap_n[i], &f) == LINK_OK && f.type == LINKV2_R && f.u.r.id == id) { c++; *status = f.u.r.status; *value = f.u.r.value; } return c; }
static int count_t(int result, int *steps_in_order) { int i, c = 0, last = 0; LinkV2Frame_t f; *steps_in_order = 1; for (i = 0; i < ncap; i++) if (LinkV2_Parse(cap[i], (uint32_t)cap_n[i], &f) == LINK_OK && f.type == LINKV2_T) { if (f.u.t.result == (uint8_t)result) c++; if (f.u.t.result == 0U) { if (f.u.t.step <= last) *steps_in_order = 0; last = f.u.t.step; } } return c; }
static int last_t_result(void) { int i, r = -1; LinkV2Frame_t f; for (i = 0; i < ncap; i++) if (LinkV2_Parse(cap[i], (uint32_t)cap_n[i], &f) == LINK_OK && f.type == LINKV2_T) r = f.u.t.result; return r; }
static uint16_t last_e_flags(void) { int i; uint16_t fl = 0xFFFFU; LinkV2Frame_t f; for (i = 0; i < ncap; i++) if (LinkV2_Parse(cap[i], (uint32_t)cap_n[i], &f) == LINK_OK && f.type == LINKV2_E) fl = f.u.e.flags2; return fl; }
static int e_flag_seen(uint16_t mask) { int i; LinkV2Frame_t f; for (i = 0; i < ncap; i++) if (LinkV2_Parse(cap[i], (uint32_t)cap_n[i], &f) == LINK_OK && f.type == LINKV2_E && (f.u.e.flags2 & mask) != 0U) return 1; return 0; }
static void fresh(void)
{
    if (Tune_Running()) { Tune_Abort(); }
    memset(mem, 0xFF, sizeof mem); flash_fail = 0; maxc_dv = 1200U; is_stopped = 1U; can_run = 1U; other_busy = 0U; tx_ok = 1; ncap = 0; T = 50000U; plant_reset();
    SlaveProfile_Init(&FL, PA, PB); SlaveTuneLink_Init(&HW); SL2_Init(&HK, T); step_ms(300U); ncap = 0;
}

int main(void)
{
    int st = -1, ordered, steps; int32_t val = 0; uint8_t id;
    /* ---- refusals, each with its reason ---- */
    fresh(); can_run = 0U; send_k(1U, LINKCMD_TUNE); CHECK(find_r(1, &st, &val) == 1 && st == (int)LINKV2_ST_NOT_ALLOWED && val == 1 && !Tune_Running(), "no tune mode in this build: NOT_ALLOWED, reason 1 (status %d, reason %d)", st, (int)val);
    fresh(); maxc_dv = 0U; send_k(1U, LINKCMD_TUNE); CHECK(find_r(1, &st, &val) == 1 && st == (int)LINKV2_ST_NOT_ALLOWED && val == 2 && !Tune_Running(), "Max Correction not set: NOT_ALLOWED, reason 2 (%d, %d)", st, (int)val);
    fresh(); is_stopped = 0U; send_k(1U, LINKCMD_TUNE); CHECK(find_r(1, &st, &val) == 1 && st == (int)LINKV2_ST_NOT_STOPPED && !Tune_Running(), "the unit is running: NOT_STOPPED (%d)", st);
    fresh(); other_busy = 1U; send_k(1U, LINKCMD_TUNE); CHECK(find_r(1, &st, &val) == 1 && st == (int)LINKV2_ST_NOT_ALLOWED && val == 3 && !Tune_Running(), "a calibration is busy: NOT_ALLOWED, reason 3 (%d, %d)", st, (int)val);
    fresh(); send_kn(1U, LINKCMD_TPARAM, "CAP", 1000); send_k(2U, LINKCMD_TSAVE); CHECK(find_r(1, &st, &val) == 1 && st == (int)LINKV2_ST_UNKNOWN_CMD && find_r(2, &st, &val) == 1 && st == (int)LINKV2_ST_UNKNOWN_CMD, "TPARAM and TSAVE are not in this version: UNKNOWN_CMD");
    /* ---- flags before any tune ---- */
    fresh(); step_ms(300U); CHECK((last_e_flags() & LINKV2_FLAG_NOT_TUNED) != 0U && (last_e_flags() & LINKV2_FLAG_BUSY) == 0U, "no profile yet: $E says NOT_TUNED, not BUSY (0x%04X)", (unsigned)last_e_flags());
    /* ---- a whole successful sweep, by bytes ---- */
    fresh(); send_k(1U, LINKCMD_TUNE);
    CHECK(find_r(1, &st, &val) == 1 && st == 0 && Tune_Running(), "K TUNE: R status 0 and the sweep runs");
    { uint32_t ticks_before = (uint32_t)SlaveTuneLink_RequestTicks(); send_k(1U, LINKCMD_TUNE); CHECK(find_r(1, &st, &val) == 2 && st == 0 && Tune_Running() && (uint32_t)SlaveTuneLink_RequestTicks() == ticks_before, "a retry of the same id gets the SAME answer again (R status 0) and the sweep is not started a second time"); }
    send_k(2U, LINKCMD_TUNE); CHECK(find_r(2, &st, &val) == 1 && st == (int)LINKV2_ST_NOT_ALLOWED && val == 4, "another TUNE while it runs: NOT_ALLOWED, reason 4 (%d, %d)", st, (int)val);
    step_ms(5000U); CHECK(e_flag_seen(LINKV2_FLAG_BUSY) && SlaveTuneLink_Active() && SlaveTuneLink_RequestTicks() > 0U, "while it runs $E says BUSY and the duty it asks for is %u ticks", (unsigned)SlaveTuneLink_RequestTicks());
    step_ms(36000U);
    steps = count_t(0, &ordered); CHECK(steps == 10 && ordered, "ten $T step frames arrived, in order (%d, ordered %d)", steps, ordered);
    CHECK(count_t(1, &ordered) >= 7, "and the FINISHED result: sent once and repeated 6 times (%d frames)", count_t(1, &ordered));
    CHECK(last_t_result() == 1 && SlaveProfile_Tuned() == 1U && !Tune_Running() && SlaveTuneLink_RequestTicks() == 0U, "the profile is stored, the duty request is back to 0");
    { TuneProfile_t p; CHECK(SlaveProfile_Get(&p) == 1U && p.cap_ticks == 2752U && p.kib_m >= 940U && p.kib_m <= 1060U, "the stored profile: cap %u, gain scale %u", (unsigned)p.cap_ticks, (unsigned)p.kib_m); }
    step_ms(500U); ncap = 0; step_ms(500U); CHECK((last_e_flags() & (LINKV2_FLAG_BUSY | LINKV2_FLAG_NOT_TUNED)) == 0U, "afterwards $E has neither BUSY nor NOT_TUNED (0x%04X)", (unsigned)last_e_flags());
    /* the same again, but the transmitter has no room for 2 s in the middle: nothing is lost or reordered */
    fresh(); send_k(1U, LINKCMD_TUNE); step_ms(8000U); tx_ok = 0; step_ms(2500U); tx_ok = 1; step_ms(40000U);
    steps = count_t(0, &ordered); CHECK(steps == 10 && ordered && last_t_result() == 1, "a transmitter without room for 2.5 s: still 10 step frames, in order, and the result (%d, ordered %d)", steps, ordered);
    /* ---- a re-tune over an existing profile: the old one stays until the new one is stored ---- */
    { TuneProfile_t a, b; fresh(); send_k(1U, LINKCMD_TUNE); step_ms(42000U); (void)SlaveProfile_Get(&a); ncap = 0;
      P.k = 0.6; send_k(2U, LINKCMD_TUNE); CHECK(find_r(2, &st, &val) == 1 && st == 0, "a re-tune is accepted");
      step_ms(9000U); { TuneProfile_t m; CHECK(SlaveProfile_Tuned() == 1U && SlaveProfile_Get(&m) == 1U && m.kib_m == a.kib_m, "during it the OLD profile is still the active one"); }
      step_ms(40000U); (void)SlaveProfile_Get(&b); CHECK(b.kib_m != a.kib_m && b.kib_m > a.kib_m, "afterwards the NEW profile (a weaker transformer: gain scale %u, was %u)", (unsigned)b.kib_m, (unsigned)a.kib_m); }
    /* ---- a new sweep started while the old result is still being repeated: the old FINISHED must not leak into the new run ---- */
    fresh(); send_k(1U, LINKCMD_TUNE); step_ms(35200U);                      /* the first sweep has just finished, its result is still repeated for about 3 s */
    CHECK(count_t(1, &ordered) >= 1, "the first sweep has finished and its result is being repeated");
    ncap = 0; send_k(2U, LINKCMD_TUNE); step_ms(12000U);
    CHECK(find_r(2, &st, &val) == 1 && st == 0 && count_t(1, &ordered) == 0 && count_t(0, &ordered) >= 3, "a new TUNE: no FINISHED frame of the OLD run arrives during the new one, only its own steps (%d finished, %d steps)", count_t(1, &ordered), count_t(0, &ordered));
    /* ---- TABORT ---- */
    fresh(); send_k(1U, LINKCMD_TUNE); step_ms(17000U); send_k(2U, LINKCMD_TABORT); step_ms(1500U);
    CHECK(find_r(2, &st, &val) == 1 && st == 0 && !Tune_Running() && last_t_result() == 6 && SlaveProfile_Tuned() == 0U && SlaveTuneLink_RequestTicks() == 0U, "TABORT at 17 s: R 0, the sweep stops, the result is 6 (BY_MASTER), nothing stored, no duty (result %d)", last_t_result());
    send_k(3U, LINKCMD_TABORT); CHECK(find_r(3, &st, &val) == 1 && st == 0, "TABORT with nothing running is answered 0 as well");
    step_ms(600U); ncap = 0; step_ms(500U); CHECK((last_e_flags() & LINKV2_FLAG_BUSY) == 0U && (last_e_flags() & LINKV2_FLAG_NOT_TUNED) != 0U, "after the abort: not BUSY, still NOT_TUNED");
    /* ---- the transformer is the wrong way round, or not there ---- */
    fresh(); P.sign = -1.0; send_k(1U, LINKCMD_TUNE); step_ms(30000U);
    CHECK(last_t_result() == 2 && SlaveProfile_Tuned() == 0U && !Tune_Running(), "a flipped transformer: result 2 (FLIPPED), nothing stored (result %d)", last_t_result());
    fresh(); P.sign = 0.0; send_k(1U, LINKCMD_TUNE); step_ms(30000U);
    CHECK(last_t_result() == 3 && SlaveProfile_Tuned() == 0U, "no transformer: result 3 (NO_RESPONSE), nothing stored (result %d)", last_t_result());
    /* ---- the flash refuses the save: the master is told ABORTED, not FINISHED ---- */
    fresh(); flash_fail = 1; send_k(1U, LINKCMD_TUNE); step_ms(45000U);
    CHECK(last_t_result() == 4 && count_t(1, &ordered) == 0 && SlaveProfile_Tuned() == 0U && !Tune_Running(), "the flash refuses the profile: the result is 4 (ABORTED), never 1 (FINISHED), nothing is claimed stored (result %d)", last_t_result());
    flash_fail = 0; ncap = 0; send_k(2U, LINKCMD_TUNE); step_ms(45000U); CHECK(last_t_result() == 1 && SlaveProfile_Tuned() == 1U, "and with the flash working again a new tune finishes");
    /* ---- Max Correction is read when the sweep starts: a low rating keeps the sweep short and the injection low ---- */
    fresh(); maxc_dv = 600U; send_k(1U, LINKCMD_TUNE); step_ms(40000U);
    { int steps6 = count_t(0, &ordered); TuneProfile_t p; CHECK(last_t_result() == 1 && steps6 == 8 && SlaveProfile_Get(&p) == 1U && p.maxc_dv == 600U && p.cap_ticks < 1700U, "Max Correction 60 V: 8 steps, a profile made with 60 V, the cap %u", (unsigned)p.cap_ticks); }
    /* ---- NOT_TUNED also says that Max Correction is missing ---- */
    maxc_dv = 0U; ncap = 0; step_ms(500U); CHECK((last_e_flags() & LINKV2_FLAG_NOT_TUNED) != 0U, "a profile is stored but Max Correction is 0 again: NOT_TUNED");
    (void)id;
    printf("  autotune over the link: TUNE / TABORT as bytes with their refusals and reasons, $T frames in order (also through a blocked transmitter), BUSY / NOT_TUNED, the profile stored before FINISHED is reported, flipped / absent / flash refusal\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", tot, bad, bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
