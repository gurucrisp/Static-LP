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
static uint8_t mem[4096]; static int flash_fail; static int flash_ops, flash_viol; static uint32_t ctrl_stop_at; static uint8_t ctrl_stuck, stuck_now; static float link_iout_rms = 1.0f; static uint8_t link_iout_valid = 1U;
static int frd(uint32_t a, uint8_t *b, uint32_t n) { memcpy(b, &mem[a - PA], n); return 0; }
static uint8_t hw_stopped(void);
static int fer(uint32_t a) { flash_ops++; if (hw_stopped() == 0U) flash_viol++; if (flash_fail) return -1; memset(&mem[a - PA], 0xFF, 2048U); return 0; }
static int fpr(uint32_t a, const uint8_t *b, uint32_t n) { uint32_t i; flash_ops++; if (hw_stopped() == 0U) flash_viol++; if (flash_fail) return -1; for (i = 0U; i < n; i++) mem[a - PA + i] &= b[i]; return 0; }
static const SFlash_t FL = { frd, fpr, fer };

/* ---- what the controller side would say: a fake controller that takes prep_ms to reach RUN and then applies the requested duty ---- */
static uint32_t T;
static uint16_t maxc_dv = 1200U; static uint8_t is_stopped = 1U, can_run = 1U, other_busy = 0U, begin_ok = 1U, ctrl_on = 0U; static uint16_t ctrl_req = 0U, ctrl_fault = 0U, ctrl_max_req = 0U; static uint32_t ctrl_ready_at = 0U, prep_ms = 6000U; static int ctrl_ends = 0, ctrl_begins = 0;
static uint16_t hw_maxc(void) { return maxc_dv; } static uint8_t hw_stopped(void) { return (uint8_t)((is_stopped != 0U) && (ctrl_on == 0U) && (stuck_now == 0U) && (T >= ctrl_stop_at)); } static uint8_t hw_can_run(void) { return can_run; } static uint8_t hw_busy(void) { return other_busy; }
      /* the fake controller needs 400 ms to be OFF after hw_end(); a stuck one never is; every flash operation is counted and judged */
static uint8_t hw_begin(void) { if (!begin_ok) return 0U; ctrl_on = 1U; ctrl_ready_at = T + prep_ms; ctrl_begins++; ctrl_req = 0U; return 1U; }
static void hw_request(uint16_t k) { ctrl_req = k; if (k > ctrl_max_req) ctrl_max_req = k; }
static void hw_end(void) { ctrl_on = 0U; ctrl_ends++; ctrl_req = 0U; ctrl_stop_at = T + 400U; if (ctrl_stuck != 0U) stuck_now = 1U; }      /* a stuck controller never reaches OFF once it has been told to stop */
static uint8_t hw_ready(void) { return (uint8_t)((ctrl_on != 0U) && (T >= ctrl_ready_at)); }
static uint8_t hw_alive(void) { return ctrl_on; }
static uint16_t hw_fault(void) { return ctrl_fault; }
static uint16_t headroom = 3000U; static uint16_t hw_headroom(void) { return headroom; }
static const TuneLinkHw_t HW = { hw_maxc, hw_stopped, hw_can_run, hw_busy, hw_begin, hw_request, hw_end, hw_ready, hw_alive, hw_fault, hw_headroom };

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
static Plant_t P; static double inj; static uint32_t rs = 99U; static double gauss(void) { double a = 0.0; int i; for (i = 0; i < 3; i++) { rs = rs * 1664525U + 1013904223U; a += (double)((rs >> 8) & 0xFFFFFFU) / 8388608.0 - 1.0; } return a / 1.732; }
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
        T += 10U; inj += (true_inj((double)((hw_ready() != 0U) ? ctrl_req : 0U)) - inj) * a;           /* the plant sees the duty the CONTROLLER applies */
        in.vin = (float)(P.vin + 0.05 * gauss()); in.vout = (float)(P.vin + 0.2 + inj + P.noise * gauss()); in.meas_valid = 1U; in.fault = 0U; in.vin_in_window = 1U; in.maxc_v = (float)maxc_dv / 10.0f; in.hw_cap_ticks = 2752U;
        in.iout_valid = (uint8_t)(link_iout_valid != 0U); in.iout_clip = 0U; in.iout_rms = link_iout_rms; in.iout_pk = link_iout_rms * 4.0f;                     /* the converter current: 1 A RMS, spiky */
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
static int count_t(int result, int *steps_in_order) { int i, c = 0, last = 0; LinkV2Frame_t f; *steps_in_order = 1; for (i = 0; i < ncap; i++) if (LinkV2_Parse(cap[i], (uint32_t)cap_n[i], &f) == LINK_OK && f.type == LINKV2_T) { if (f.u.t.result == (uint8_t)result && (result != 0 || f.u.t.step > 0U)) c++; if (f.u.t.result == 0U && f.u.t.step > 0U) { if (f.u.t.step <= last) *steps_in_order = 0; last = f.u.t.step; } } return c; }
static int last_t_dv(void) { int i, r = 0; LinkV2Frame_t f; for (i = 0; i < ncap; i++) if (LinkV2_Parse(cap[i], (uint32_t)cap_n[i], &f) == LINK_OK && f.type == LINKV2_T) r = f.u.t.dv; return r; }
static int count_heartbeats(void) { int i, c = 0; LinkV2Frame_t f; for (i = 0; i < ncap; i++) if (LinkV2_Parse(cap[i], (uint32_t)cap_n[i], &f) == LINK_OK && f.type == LINKV2_T && f.u.t.result == 0U && f.u.t.step == 0U) c++; return c; }
static int last_t_result(void) { int i, r = -1; LinkV2Frame_t f; for (i = 0; i < ncap; i++) if (LinkV2_Parse(cap[i], (uint32_t)cap_n[i], &f) == LINK_OK && f.type == LINKV2_T) r = f.u.t.result; return r; }
static uint16_t last_e_flags(void) { int i; uint16_t fl = 0xFFFFU; LinkV2Frame_t f; for (i = 0; i < ncap; i++) if (LinkV2_Parse(cap[i], (uint32_t)cap_n[i], &f) == LINK_OK && f.type == LINKV2_E) fl = f.u.e.flags2; return fl; }
static int e_flag_seen(uint16_t mask) { int i; LinkV2Frame_t f; for (i = 0; i < ncap; i++) if (LinkV2_Parse(cap[i], (uint32_t)cap_n[i], &f) == LINK_OK && f.type == LINKV2_E && (f.u.e.flags2 & mask) != 0U) return 1; return 0; }
static void fresh(void)
{
    if (Tune_Running()) { Tune_Abort(); }
    memset(mem, 0xFF, sizeof mem); flash_fail = 0; maxc_dv = 1200U; is_stopped = 1U; can_run = 1U; other_busy = 0U; begin_ok = 1U; headroom = 3000U; ctrl_stuck = 0U; stuck_now = 0U; link_iout_rms = 1.0f; link_iout_valid = 1U; ctrl_stop_at = 0U; flash_ops = 0; flash_viol = 0; ctrl_on = 0U; ctrl_req = 0U; ctrl_fault = 0U; ctrl_max_req = 0U; ctrl_ends = 0; ctrl_begins = 0; prep_ms = 6000U; tx_ok = 1; ncap = 0; T = 50000U; plant_reset();
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
    CHECK(find_r(1, &st, &val) == 1 && st == 0 && SlaveTuneLink_Active(), "K TUNE: R status 0 and the run is under way (preparing)");
    { uint32_t ticks_before = (uint32_t)SlaveTuneLink_RequestTicks(); send_k(1U, LINKCMD_TUNE); CHECK(find_r(1, &st, &val) == 2 && st == 0 && SlaveTuneLink_Active() && ctrl_begins == 1 && (uint32_t)SlaveTuneLink_RequestTicks() == ticks_before, "a retry of the same id gets the SAME answer again (R status 0) and the controller is told to begin only once"); }
    send_k(2U, LINKCMD_TUNE); CHECK(find_r(2, &st, &val) == 1 && st == (int)LINKV2_ST_NOT_ALLOWED && val == 4, "another TUNE while it runs: NOT_ALLOWED, reason 4 (%d, %d)", st, (int)val);
    step_ms(14000U); CHECK(e_flag_seen(LINKV2_FLAG_BUSY) && SlaveTuneLink_Active() && ctrl_req > 0U, "while it runs $E says BUSY and the controller is asked for %u ticks", (unsigned)ctrl_req);
    step_ms(36000U);
    steps = count_t(0, &ordered); CHECK(steps == 10 && ordered, "ten $T step frames arrived, in order (%d, ordered %d)", steps, ordered);
    CHECK(count_t(1, &ordered) >= 7, "and the FINISHED result: sent once and repeated 6 times (%d frames)", count_t(1, &ordered));
    CHECK(last_t_result() == 1 && SlaveProfile_Tuned() == 1U && !Tune_Running() && SlaveTuneLink_RequestTicks() == 0U, "the profile is stored, the duty request is back to 0");
    { TuneProfile_t p; CHECK(SlaveProfile_Get(&p) == 1U && p.cap_ticks == 2752U && p.kib_m >= 940U && p.kib_m <= 1060U, "the stored profile: cap %u, gain scale %u", (unsigned)p.cap_ticks, (unsigned)p.kib_m); }
    step_ms(500U); ncap = 0; step_ms(500U); CHECK((last_e_flags() & (LINKV2_FLAG_BUSY | LINKV2_FLAG_NOT_TUNED)) == 0U, "afterwards $E has neither BUSY nor NOT_TUNED (0x%04X)", (unsigned)last_e_flags());
    /* the same again, but the transmitter has no room for 2 s in the middle: nothing is lost or reordered */
    fresh(); send_k(1U, LINKCMD_TUNE); step_ms(14000U); tx_ok = 0; step_ms(2500U); tx_ok = 1; step_ms(40000U);
    steps = count_t(0, &ordered); CHECK(steps == 10 && ordered && last_t_result() == 1, "a transmitter without room for 2.5 s: still 10 step frames, in order, and the result (%d, ordered %d)", steps, ordered);
    /* ---- a re-tune over an existing profile: the old one stays until the new one is stored ---- */
    { TuneProfile_t a, b; fresh(); send_k(1U, LINKCMD_TUNE); step_ms(50000U); (void)SlaveProfile_Get(&a); ncap = 0;
      P.k = 0.6; send_k(2U, LINKCMD_TUNE); CHECK(find_r(2, &st, &val) == 1 && st == 0, "a re-tune is accepted");
      step_ms(16000U); { TuneProfile_t m; CHECK(SlaveProfile_Tuned() == 1U && SlaveProfile_Get(&m) == 1U && m.kib_m == a.kib_m, "during it the OLD profile is still the active one"); }
      step_ms(45000U); (void)SlaveProfile_Get(&b); CHECK(b.kib_m != a.kib_m && b.kib_m > a.kib_m, "afterwards the NEW profile (a weaker transformer: gain scale %u, was %u)", (unsigned)b.kib_m, (unsigned)a.kib_m); }
    /* ---- a new sweep started while the old result is still being repeated: the old FINISHED must not leak into the new run ---- */
    fresh(); send_k(1U, LINKCMD_TUNE); step_ms(41500U);                      /* the first sweep has just finished, its result is still repeated for about 3 s */
    CHECK(count_t(1, &ordered) >= 1, "the first sweep has finished and its result is being repeated");
    ncap = 0; send_k(2U, LINKCMD_TUNE); step_ms(20000U);
    CHECK(find_r(2, &st, &val) == 1 && st == 0 && count_t(1, &ordered) == 0 && count_t(0, &ordered) >= 3, "a new TUNE: no FINISHED frame of the OLD run arrives during the new one, only its own steps (%d finished, %d steps)", count_t(1, &ordered), count_t(0, &ordered));
    /* ---- TABORT ---- */
    fresh(); send_k(1U, LINKCMD_TUNE); step_ms(23000U); send_k(2U, LINKCMD_TABORT); step_ms(1500U);
    CHECK(find_r(2, &st, &val) == 1 && st == 0 && !Tune_Running() && last_t_result() == 6 && SlaveProfile_Tuned() == 0U && SlaveTuneLink_RequestTicks() == 0U, "TABORT at 23 s: R 0, the sweep stops, the result is 6 (BY_MASTER), nothing stored, no duty (result %d)", last_t_result());
    send_k(3U, LINKCMD_TABORT); CHECK(find_r(3, &st, &val) == 1 && st == 0, "TABORT with nothing running is answered 0 as well");
    step_ms(600U); ncap = 0; step_ms(500U); CHECK((last_e_flags() & LINKV2_FLAG_BUSY) == 0U && (last_e_flags() & LINKV2_FLAG_NOT_TUNED) != 0U, "after the abort: not BUSY, still NOT_TUNED");
    /* ---- the transformer is the wrong way round, or not there ---- */
    fresh(); P.sign = -1.0; send_k(1U, LINKCMD_TUNE); step_ms(38000U);
    CHECK(last_t_result() == 2 && SlaveProfile_Tuned() == 0U && !Tune_Running(), "a flipped transformer: result 2 (FLIPPED), nothing stored (result %d)", last_t_result());
    fresh(); P.sign = 0.0; send_k(1U, LINKCMD_TUNE); step_ms(38000U);
    CHECK(last_t_result() == 3 && SlaveProfile_Tuned() == 0U, "no transformer: result 3 (NO_RESPONSE), nothing stored (result %d)", last_t_result());
    /* ---- the flash refuses the save: the master is told ABORTED, not FINISHED ---- */
    fresh(); flash_fail = 1; send_k(1U, LINKCMD_TUNE); step_ms(52000U);
    CHECK(last_t_result() == 4 && count_t(1, &ordered) == 0 && SlaveProfile_Tuned() == 0U && !Tune_Running(), "the flash refuses the profile: the result is 4 (ABORTED), never 1 (FINISHED), nothing is claimed stored (result %d)", last_t_result());
    flash_fail = 0; ncap = 0; send_k(2U, LINKCMD_TUNE); step_ms(52000U); CHECK(last_t_result() == 1 && SlaveProfile_Tuned() == 1U, "and with the flash working again a new tune finishes");
    /* ---- Max Correction is read when the sweep starts: a low rating keeps the sweep short and the injection low ---- */
    fresh(); maxc_dv = 600U; send_k(1U, LINKCMD_TUNE); step_ms(46000U);
    { int steps6 = count_t(0, &ordered); TuneProfile_t p; CHECK(last_t_result() == 1 && steps6 == 8 && SlaveProfile_Get(&p) == 1U && p.maxc_dv == 600U && p.cap_ticks < 1700U, "Max Correction 60 V: 8 steps, a profile made with 60 V, the cap %u", (unsigned)p.cap_ticks); }
    /* ---- the preparation: the controller needs about 6 s to reach RUN; the master hears heartbeats, the sweep starts only when the controller is ready ---- */
    fresh(); send_k(1U, LINKCMD_TUNE);
    CHECK(find_r(1, &st, &val) == 1 && st == 0 && SlaveTuneLink_Active() && ctrl_begins == 1 && !Tune_Running(), "TUNE: the controller is told to begin once, the sweep does not start yet (the controller is not in RUN)");
    step_ms(5500U); CHECK(!Tune_Running() && ctrl_req == 0U && count_t(0, &ordered) == 0 && count_heartbeats() >= 2 && e_flag_seen(LINKV2_FLAG_BUSY), "5.5 s of preparation: no duty asked for, no step frame, %d heartbeats (step 0), BUSY", count_heartbeats());
    step_ms(2500U); CHECK(Tune_Running() && ctrl_on, "then the controller is in RUN and the sweep starts");
    step_ms(40000U); CHECK(ctrl_ends >= 1 && !ctrl_on && ctrl_req == 0U && ctrl_max_req <= 2752U && last_t_result() == 1, "at the end the controller is told to stop (%d), the duty is back to 0, the largest duty asked for was %u ticks (the cap is 2752)", ctrl_ends, (unsigned)ctrl_max_req);
    /* TABORT while the controller is still starting up */
    fresh(); send_k(1U, LINKCMD_TUNE); step_ms(2000U); send_k(2U, LINKCMD_TABORT); step_ms(600U);
    CHECK(find_r(2, &st, &val) == 1 && st == 0 && !SlaveTuneLink_Active() && ctrl_ends == 1 && !ctrl_on && last_t_result() == 6, "TABORT in the preparation: R 0, the controller is told to stop, the result is 6 (BY_MASTER) (result %d)", last_t_result());
    step_ms(9000U); CHECK(!Tune_Running(), "and the sweep never starts afterwards");
    /* the controller has a fault while starting */
    fresh(); send_k(1U, LINKCMD_TUNE); step_ms(2000U); ctrl_fault = 6U; step_ms(500U);
    CHECK(!SlaveTuneLink_Active() && ctrl_ends == 1 && last_t_result() == 4 && SlaveProfile_Tuned() == 0U, "a controller fault during the preparation: result 4 (ABORTED), the controller is told to stop");
    /* the controller never reaches RUN */
    fresh(); prep_ms = 100000U; send_k(1U, LINKCMD_TUNE); step_ms(24000U); CHECK(SlaveTuneLink_Active(), "still waiting at 24 s"); step_ms(2500U);
    CHECK(!SlaveTuneLink_Active() && ctrl_ends == 1 && last_t_result() == 4, "it never reaches RUN: ABORTED after 25 s (result %d)", last_t_result());
    /* the controller refuses to begin (a latched fault, not OFF) */
    fresh(); begin_ok = 0U; send_k(1U, LINKCMD_TUNE); CHECK(find_r(1, &st, &val) == 1 && st == (int)LINKV2_ST_NOT_ALLOWED && val == 5 && !SlaveTuneLink_Active() && ctrl_begins == 0, "the controller cannot begin: NOT_ALLOWED, reason 5 (%d, %d)", st, (int)val);
    /* the controller stops under the sweep (the master link was lost): the sweep is aborted */
    fresh(); send_k(1U, LINKCMD_TUNE); step_ms(14000U); CHECK(Tune_Running(), "a sweep is running"); ctrl_on = 0U; step_ms(500U);
    CHECK(!Tune_Running() && last_t_result() == 4 && SlaveProfile_Tuned() == 0U, "tune mode ends in the controller during the sweep: the sweep is aborted (result %d)", last_t_result());
    /* a controller fault in the middle of the sweep */
    fresh(); send_k(1U, LINKCMD_TUNE); step_ms(18000U); ctrl_fault = 2U; step_ms(500U);
    CHECK(!Tune_Running() && last_t_result() == 4 && ctrl_req == 0U && ctrl_ends >= 1, "a controller fault in the middle of the sweep: ABORTED, the duty request is 0, the controller is told to stop");
    /* ---- the output over-voltage headroom: the input is too high, or the sweep is limited by it ---- */
    fresh(); headroom = 100U; send_k(1U, LINKCMD_TUNE); CHECK(find_r(1, &st, &val) == 1 && st == (int)LINKV2_ST_NOT_ALLOWED && val == 6 && !SlaveTuneLink_Active() && ctrl_begins == 0, "10 V of headroom: NOT_ALLOWED, reason 6 (input too high), the controller is not even started (%d, %d)", st, (int)val);
    fresh(); headroom = 400U; send_k(1U, LINKCMD_TUNE); step_ms(46000U);
    { TuneProfile_t pf; CHECK(last_t_result() == 1 && SlaveProfile_Get(&pf) == 1U && pf.maxc_dv == 400U && ctrl_max_req < 1300U, "40 V of headroom with Max Correction 120 V: the sweep is limited to the headroom: the profile is made at %u (tenths), the largest duty asked for is %u ticks", (unsigned)pf.maxc_dv, (unsigned)ctrl_max_req); }
    /* ---- FLASH IS NEVER TOUCHED WHILE THE STAGE IS ENERGISED (v0.9.42; v0.9.32 to v0.9.41 wrote the profile at the end of the sweep with the converter and inverter still running at the last step's duty:
     *      a flash erase stalls the CPU and the commutation interrupts, and a diagonal can stay on across a polarity change) ---- */
    fresh(); send_k(1U, LINKCMD_TUNE); step_ms(52000U);
    CHECK(flash_ops > 0 && flash_viol == 0 && SlaveProfile_Tuned() == 1U && last_t_result() == 1, "the profile is written (%d flash operations) and NOT ONE of them while the controller was running or still stopping (%d violations)", flash_ops, flash_viol);
    { int saw_final_before_tuned = 0, k; fresh(); send_k(1U, LINKCMD_TUNE);
      for (k = 0; (k < 6000) && (last_t_result() != 1); k++)
      {
          step_ms(10U);
          if ((last_t_result() == 1) && (SlaveProfile_Tuned() == 0U)) { saw_final_before_tuned = 1; }
      }
      CHECK(!saw_final_before_tuned && last_t_result() == 1 && SlaveProfile_Tuned() == 1U, "FINISHED is never reported before the profile is in flash"); }
    /* a controller that never reaches OFF after the sweep: nothing is written, the master is told ABORTED */
    fresh(); ctrl_stuck = 1U; send_k(1U, LINKCMD_TUNE); step_ms(60000U);
    CHECK(flash_ops == 0 && SlaveProfile_Tuned() == 0U && last_t_result() == 4 && !SlaveTuneLink_Active(), "the controller never reaches OFF: NO flash operation at all, result 4 (ABORTED) (%d operations, result %d)", flash_ops, last_t_result());
    /* STOP pressed after the sweep, while the profile waits for the stage to stop: cancelled, nothing is written */
    { int k, waiting = 0; fresh(); send_k(1U, LINKCMD_TUNE);
      for (k = 0; (k < 9000) && !Tune_Running(); k++) { step_ms(10U); }
      for (k = 0; (k < 9000) && (Tune_Running() || !SlaveTuneLink_Active()); k++) { step_ms(10U); }
      waiting = (!Tune_Running() && SlaveTuneLink_Active() && (flash_ops == 0));
      CHECK(waiting, "the sweep is over and the profile is waiting for the controller to stop (no flash operation yet)");
      send_k(2U, LINKCMD_TABORT); step_ms(2000U);
      CHECK(flash_ops == 0 && SlaveProfile_Tuned() == 0U && last_t_result() == 6 && !SlaveTuneLink_Active(), "STOP at that moment: nothing is written, result 6 (%d operations, result %d)", flash_ops, last_t_result()); }
    /* ---- THE CONVERTER-CURRENT GUARD end to end: the current rises in the middle of the sweep ---- */
    fresh(); send_k(1U, LINKCMD_TUNE); step_ms(24000U); link_iout_rms = 8.0f; step_ms(2500U);
    CHECK(!Tune_Running() && last_t_result() == 4 && last_t_dv() >= 70 && ctrl_req == 0U && ctrl_ends >= 1 && flash_ops == 0 && SlaveProfile_Tuned() == 0U, "the converter current rises to 8 A during a step: ABORTED, the duty request is 0, the controller is told to stop, NOTHING is written (result %d, the frame says %.1f A, %d flash operations)", last_t_result(), (double)last_t_dv() / 10.0, flash_ops);
    fresh(); link_iout_valid = 0U; send_k(1U, LINKCMD_TUNE); step_ms(16000U);
    CHECK(last_t_result() == 4 && last_t_dv() == -1 && ctrl_max_req == 0U && flash_ops == 0, "no converter-current reading: the sweep never applies any duty, result 4, the frame says -1 (result %d, frame %d, largest duty %u)", last_t_result(), last_t_dv(), (unsigned)ctrl_max_req);
    /* ---- NOT_TUNED also says that Max Correction is missing ---- */
    maxc_dv = 0U; ncap = 0; step_ms(500U); CHECK((last_e_flags() & LINKV2_FLAG_NOT_TUNED) != 0U, "a profile is stored but Max Correction is 0 again: NOT_TUNED");
    (void)id;
    printf("  autotune over the link: TUNE / TABORT as bytes with their refusals and reasons, $T frames in order (also through a blocked transmitter), BUSY / NOT_TUNED, the profile stored before FINISHED is reported, flipped / absent / flash refusal\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", tot, bad, bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
