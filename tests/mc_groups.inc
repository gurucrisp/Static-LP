#ifdef TEST_MC
/* ================================================================================================================================================================
 * MASTER CONTROL (roadmap step 3; decisions D1, D5, D9, D12): built with SLAVE_MASTER_CONTROL 1, SLAVE_LINK_V2 1, no autorun, stage 3 with the legacy relays.
 * The "master" below is a few lines that call SlaveCtrl_OnControl() once per 100 ms exactly as the link would; the last group drives the controller through
 * real frame bytes and the real link logic. */
#include "slave_link2.h"
#include "link_v2.h"

static uint32_t mc_ms(void) { return (uint32_t)(sim_t / 1000ULL); }
static void mc_world(uint64_t seed, float vin) { l3_world(seed, vin); }
/* the master: one $M every 100 ms for `ms` milliseconds */
static void mc_for(uint32_t ms, uint8_t run, uint16_t setv, uint8_t clr, uint16_t maxc)
{
    uint32_t k;
    for (k = 0U; k < (ms / 100U); k++) { SlaveCtrl_OnControl(run, setv, clr, maxc, mc_ms()); run_for(100000); }
}
/* until RUN (or the timeout), sending run = 1; returns the milliseconds it took */
static uint32_t mc_until_run(uint16_t setv, uint8_t clr, uint32_t timeout_ms)
{
    uint32_t k;
    for (k = 0U; k < (timeout_ms / 100U); k++)
    {
        if (g_ctrl.state == CTRL_RUN) return k * 100U;
        SlaveCtrl_OnControl(1U, setv, clr, 300U, mc_ms());
        run_for(100000);
    }
    return timeout_ms;
}
static void mc_to_run_settled(uint64_t seed, float vin, uint16_t setv, uint8_t clr)
{
    uint32_t t;
    mc_world(seed, vin);
    t = mc_until_run(setv, clr, 40000U);
    CHECK(g_ctrl.state == CTRL_RUN, "the unit reaches RUN under master control (state %d after %u ms)", g_ctrl.state, (unsigned)t);
    mc_for(10000U, 1U, setv, clr, 300U);
}

static void t_mc_no_autorun(void)
{
    mc_world(220000000ULL, 225.0f);
    run_for(5000000);
    CHECK(g_ctrl.state == CTRL_OFF && n_energise == 0 && !is_running(), "no $M at all: the unit does not start (state %d, %d energisations)", g_ctrl.state, n_energise);
    CHECK((SlaveCtrl_Flags2() & LINKV2_FLAG_LINK_LOST) != 0U, "after the 1 s timeout LINK_LOST is flagged (flags2 0x%04X)", (unsigned)SlaveCtrl_Flags2());
    CHECK((SlaveCtrl_Flags2() & LINKV2_FLAG_RUN_ACK) == 0U, "and run is not acknowledged");
    CHECK(SlaveCtrl_SetvDv() == 0U && SlaveCtrl_ClrEcho() == 0U, "no set voltage and no clear counter accepted yet (setv %u, clr %u)", (unsigned)SlaveCtrl_SetvDv(), (unsigned)SlaveCtrl_ClrEcho());
    mc_for(10000U, 0U, 2300U, 0U, 300U);                                     /* a master that says run = 0 */
    CHECK(n_energise == 0 && g_ctrl.state == CTRL_OFF, "run = 0 for 10 s: still nothing energised (state %d)", g_ctrl.state);
    CHECK((SlaveCtrl_Flags2() & LINKV2_FLAG_LINK_LOST) == 0U, "the link is alive again as soon as $M frames arrive");
    CHECK(SlaveCtrl_SetvDv() == 2300U, "the set voltage is adopted while stopped (%u)", (unsigned)SlaveCtrl_SetvDv());
    check_invariants("master control: no autorun");
    printf("  master control: no autorun; LINK_LOST after 1 s; run = 0 keeps it off\n");
}

static void t_mc_run_in_band(void)
{
    uint16_t f2, f_entry; double v_entry; uint32_t t;
    mc_world(221000000ULL, 225.0f);
    t = mc_until_run(2300U, 5U, 40000U);
    f_entry = SlaveCtrl_Flags2(); v_entry = (double)W.vout;
    CHECK(g_ctrl.state == CTRL_RUN && v_entry < 227.5, "at the start of RUN the output is still about 225 V, 5 V from the set voltage (%.2f V, after %u ms)", v_entry, (unsigned)t);
    CHECK((f_entry & LINKV2_FLAG_IN_BAND) == 0U && (f_entry & LINKV2_FLAG_RUN_ACK) != 0U, "IN_BAND is CLEAR while the output is 5 V from the set voltage, RUN_ACK is set (flags2 0x%04X)", (unsigned)f_entry);
    mc_for(10000U, 1U, 2300U, 5U, 300U);
    f2 = SlaveCtrl_Flags2();
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "running without a fault (state %d, fault %d)", g_ctrl.state, g_ctrl.fault);
    CHECK((double)W.vout > 228.5 && (double)W.vout < 231.5, "regulating to the set voltage 230 V (%.2f V)", (double)W.vout);
    CHECK((f2 & LINKV2_FLAG_RUN_ACK) != 0U && (f2 & LINKV2_FLAG_IN_BAND) != 0U, "RUN_ACK and IN_BAND are set (flags2 0x%04X)", (unsigned)f2);
    CHECK((f2 & (LINKV2_FLAG_LINK_LOST | LINKV2_FLAG_FAULT_LATCHED)) == 0U, "no link loss and no latched fault");
    CHECK(SlaveCtrl_SetvDv() == 2300U && SlaveCtrl_ClrEcho() == 5U, "the set voltage and the clear counter the master sent are echoed (%u, %u)", (unsigned)SlaveCtrl_SetvDv(), (unsigned)SlaveCtrl_ClrEcho());
    CHECK(SlaveCtrl_MaxCorrDv() == 300U, "the Max Correction is stored (%u)", (unsigned)SlaveCtrl_MaxCorrDv());
    mc_for(1000U, 1U, 2300U, 5U, 450U);
    CHECK(SlaveCtrl_MaxCorrDv() == 450U, "and follows the master (%u)", (unsigned)SlaveCtrl_MaxCorrDv());
    check_invariants("master control: run");
    printf("  master control: run = 1 reaches RUN, regulates to the set voltage, RUN_ACK + IN_BAND, echoes\n");
}

static void t_mc_stop_restart(void)
{
    uint32_t t;
    mc_to_run_settled(222000000ULL, 225.0f, 2300U, 1U);
    mc_for(8000U, 0U, 2300U, 1U, 300U);
    CHECK(g_ctrl.state == CTRL_OFF && !inv_enabled, "run = 0 stops it and releases everything (state %d, inverter %d)", g_ctrl.state, inv_enabled);
    CHECK((SlaveCtrl_Flags2() & LINKV2_FLAG_RUN_ACK) == 0U && (SlaveCtrl_Flags2() & LINKV2_FLAG_IN_BAND) == 0U, "RUN_ACK and IN_BAND drop (flags2 0x%04X)", (unsigned)SlaveCtrl_Flags2());
    CHECK(g_ctrl.fault == CF_NONE, "a stop is not a fault");
    t = mc_until_run(2300U, 1U, 40000U);
    CHECK(g_ctrl.state == CTRL_RUN, "run = 1 starts it again (state %d after %u ms)", g_ctrl.state, (unsigned)t);
    check_invariants("master control: stop and restart");
    printf("  master control: run = 0 stops, run = 1 starts again\n");
}

static void t_mc_fault_latches(void)
{
    int energised, k;
    mc_to_run_settled(223000000ULL, 225.0f, 2300U, 5U);
    energised = n_energise;
    SlaveCtrl_OnFltIrq();
    run_for(50000);
    CHECK(g_ctrl.state == CTRL_LATCHED, "a fault latches AT ONCE (state %d): no retry timer, no escalation count", g_ctrl.state);
    CHECK(g_ctrl.fault == CF_FLT && SlaveCtrl_FaultCode() == (uint16_t)CF_FLT && SlaveCtrl_LinkState() == 3U, "the fault code is reported (fault %d, link state %u)", g_ctrl.fault, (unsigned)SlaveCtrl_LinkState());
    CHECK((SlaveCtrl_Flags2() & LINKV2_FLAG_FAULT_LATCHED) != 0U && (SlaveCtrl_Flags2() & LINKV2_FLAG_IN_BAND) == 0U, "FAULT_LATCHED is set, IN_BAND is not (flags2 0x%04X)", (unsigned)SlaveCtrl_Flags2());
    for (k = 0; k < 3; k++)
    {   /* the master keeps saying run = 1 with the SAME clear counter: nothing happens, for a minute */
        mc_for(20000U, 1U, 2300U, 5U, 300U);
        CHECK(g_ctrl.state == CTRL_LATCHED && n_energise == energised, "20 s later (x%d): still latched, nothing energised again (state %d)", k + 1, g_ctrl.state);
    }
    check_invariants("master control: fault latches");
    printf("  master control: a fault latches at once and stays latched for a minute while the master keeps saying run = 1\n");
}

static void t_mc_clear_counter(void)
{
    uint32_t t; int energised;
    mc_to_run_settled(224000000ULL, 225.0f, 2300U, 5U);
    SlaveCtrl_OnFltIrq(); run_for(50000);
    CHECK(g_ctrl.state == CTRL_LATCHED, "latched");
    mc_for(2000U, 1U, 2300U, 5U, 300U);
    CHECK(g_ctrl.state == CTRL_LATCHED, "the SAME counter again is not a clear");
    SlaveCtrl_OnControl(1U, 2300U, 6U, 300U, mc_ms());
    CHECK(g_ctrl.state != CTRL_LATCHED && g_ctrl.fault == CF_NONE && (SlaveCtrl_Flags2() & LINKV2_FLAG_FAULT_LATCHED) == 0U, "a CHANGED counter clears it (state %d, fault %d)", g_ctrl.state, g_ctrl.fault);
    CHECK(SlaveCtrl_ClrEcho() == 6U, "and is echoed (%u)", (unsigned)SlaveCtrl_ClrEcho());
    t = mc_until_run(2300U, 6U, 40000U);
    CHECK(g_ctrl.state == CTRL_RUN, "with run = 1 it starts again after the clear (state %d after %u ms)", g_ctrl.state, (unsigned)t);
    /* a second fault: repeating the new counter does not clear it, the next change does, and run = 0 keeps it off */
    mc_for(8000U, 1U, 2300U, 6U, 300U);
    SlaveCtrl_OnFltIrq(); run_for(50000);
    CHECK(g_ctrl.state == CTRL_LATCHED, "latched again");
    mc_for(3000U, 1U, 2300U, 6U, 300U);
    CHECK(g_ctrl.state == CTRL_LATCHED, "the counter the slave has already seen does not clear a NEW fault");
    energised = n_energise;
    SlaveCtrl_OnControl(0U, 2300U, 7U, 300U, mc_ms());
    run_for(2000000);
    CHECK(g_ctrl.state == CTRL_OFF && n_energise == energised, "clear with run = 0: cleared, and it stays OFF (state %d)", g_ctrl.state);
    check_invariants("master control: clear counter");
    printf("  master control: a changed clear counter clears; the same one does not; clear + run = 1 restarts, clear + run = 0 stays off\n");
}

static void t_mc_first_counter(void)
{
    mc_world(225500000ULL, 225.0f);
    g_ctrl.state = CTRL_LATCHED; g_ctrl.fault = CF_FLT;                      /* white box: a slave that is latched and has seen no $M yet */
    SlaveCtrl_OnControl(0U, 2300U, 7U, 300U, mc_ms());
    CHECK(g_ctrl.state == CTRL_LATCHED && SlaveCtrl_ClrEcho() == 7U, "the FIRST counter is adopted and is not a clear (state %d)", g_ctrl.state);
    SlaveCtrl_OnControl(0U, 2300U, 7U, 300U, mc_ms());
    CHECK(g_ctrl.state == CTRL_LATCHED, "the same counter again: still latched");
    SlaveCtrl_OnControl(0U, 2300U, 8U, 300U, mc_ms());
    CHECK(g_ctrl.state == CTRL_OFF && g_ctrl.fault == CF_NONE, "a change clears it (state %d)", g_ctrl.state);
    mc_to_run_settled(225000000ULL, 225.0f, 2300U, 7U);
    CHECK(SlaveCtrl_ClrEcho() == 7U, "the first $M after a start only adopts its counter (7)");
    SlaveCtrl_OnFltIrq(); run_for(50000);
    mc_for(3000U, 1U, 2300U, 7U, 300U);
    CHECK(g_ctrl.state == CTRL_LATCHED, "a counter equal to the adopted one is not a clear");
    SlaveCtrl_OnControl(1U, 2300U, 8U, 300U, mc_ms());
    CHECK(g_ctrl.state != CTRL_LATCHED, "8 is");
    printf("  master control: the first counter is only adopted, never a clear\n");
}

static void t_mc_link_loss(void)
{
    uint32_t t;
    mc_to_run_settled(226000000ULL, 225.0f, 2300U, 2U);
    run_for(1300000);                                                          /* the master goes silent */
    CHECK((SlaveCtrl_Flags2() & LINKV2_FLAG_LINK_LOST) != 0U, "LINK_LOST after 1.3 s of silence (flags2 0x%04X)", (unsigned)SlaveCtrl_Flags2());
    CHECK(g_ctrl.state != CTRL_RUN && g_ctrl.state != CTRL_FAULT && g_ctrl.state != CTRL_LATCHED, "the unit has begun to stop, and it is not a fault (state %d)", g_ctrl.state);
    run_for(6000000);
    CHECK(g_ctrl.state == CTRL_OFF && g_ctrl.fault == CF_NONE && !inv_enabled, "a few seconds later it is OFF with no fault (state %d)", g_ctrl.state);
    CHECK((SlaveCtrl_Flags2() & LINKV2_FLAG_RUN_ACK) == 0U, "RUN_ACK is gone");
    t = mc_until_run(2300U, 2U, 40000U);
    CHECK(g_ctrl.state == CTRL_RUN && (SlaveCtrl_Flags2() & LINKV2_FLAG_LINK_LOST) == 0U, "the frames return with run = 1: LINK_LOST clears and it runs again (state %d after %u ms)", g_ctrl.state, (unsigned)t);
    /* a silence shorter than the timeout changes nothing */
    run_for(900000);
    CHECK(g_ctrl.state == CTRL_RUN && (SlaveCtrl_Flags2() & LINKV2_FLAG_LINK_LOST) == 0U, "0.9 s of silence is within the timeout: it keeps running");
    check_invariants("master control: link loss");
    printf("  master control: 1 s without $M stops it (no fault); frames back with run = 1 start it; 0.9 s of silence is tolerated\n");
}

static void t_mc_setv(void)
{
    uint32_t t;
    mc_to_run_settled(227000000ULL, 225.0f, 2300U, 3U);
    mc_for(6000U, 1U, 2400U, 3U, 300U);                                        /* the master changes the set voltage WHILE it runs */
    CHECK(SlaveCtrl_SetvDv() == 2300U, "while energised the new set voltage is NOT adopted (echo %u)", (unsigned)SlaveCtrl_SetvDv());
    CHECK((double)W.vout > 228.5 && (double)W.vout < 231.5, "and the output stays at 230 V (%.2f V)", (double)W.vout);
    mc_for(8000U, 0U, 2400U, 3U, 300U);                                        /* stop, still sending 2400 */
    CHECK(g_ctrl.state == CTRL_OFF && SlaveCtrl_SetvDv() == 2400U, "after the stop it is adopted, because the master repeats it (echo %u)", (unsigned)SlaveCtrl_SetvDv());
    t = mc_until_run(2400U, 3U, 40000U);
    mc_for(10000U, 1U, 2400U, 3U, 300U);
    CHECK(g_ctrl.state == CTRL_RUN && (double)W.vout > 238.5 && (double)W.vout < 241.5, "the next start regulates to 240 V (%.2f V, RUN after %u ms)", (double)W.vout, (unsigned)t);
    CHECK((SlaveCtrl_Flags2() & LINKV2_FLAG_IN_BAND) != 0U, "IN_BAND follows the NEW set voltage");
    check_invariants("master control: set voltage");
    printf("  master control: a new set voltage waits for the stop, then is adopted and regulated to\n");
}

static void t_mc_condition_fault(void)
{
    mc_to_run_settled(228000000ULL, 225.0f, 2300U, 1U);
    W.vin = 70.0f; W.vout = 70.0f;                                             /* the mains leaves the slave's window (trips below 80 V): a CONDITION, not a hardware event */
    mc_for(4000U, 1U, 2300U, 1U, 300U);
    CHECK(g_ctrl.state == CTRL_LATCHED && g_ctrl.fault == CF_VIN_RANGE, "even a condition fault latches under master control (state %d, fault %d)", g_ctrl.state, g_ctrl.fault);
    W.vin = 225.0f; W.vout = 225.0f;
    mc_for(30000U, 1U, 2300U, 1U, 300U);
    CHECK(g_ctrl.state == CTRL_LATCHED, "and does NOT resume by itself when the mains returns (state %d)", g_ctrl.state);
    SlaveCtrl_OnControl(1U, 2300U, 2U, 300U, mc_ms());
    CHECK(mc_until_run(2300U, 2U, 40000U) < 40000U && g_ctrl.state == CTRL_RUN, "the master's clear restarts it (state %d)", g_ctrl.state);
    printf("  master control: a condition fault (input out of the window) latches too and waits for the master\n");
}

/* ---- end to end: real frame bytes -> the real link logic -> the real controller; the frames that come back are parsed ---- */
#define E2E_MAX 700
static char     e2e_f[E2E_MAX][100]; static int e2e_n[E2E_MAX]; static int e2e_cnt;
static uint32_t e2e_seq_m;
static void    e2e_on_control(const LinkM_t *m, uint32_t now) { SlaveCtrl_OnControl(m->run, m->setv_dv, m->clr, m->maxc_dv, now); }
static uint16_t e2e_iout_rms, e2e_iout_pk; static uint8_t e2e_iout_on, e2e_iout_clip;      /* what the simulated slave measures on I-OUT (hundredths of an ampere); off = like a slave without the channel */
static void    e2e_fill_e(LinkE_t *e)
{
    e->setv_dv = SlaveCtrl_SetvDv(); e->flags2 = SlaveCtrl_Flags2(); e->clr = SlaveCtrl_ClrEcho(); e->reach_lo_dv = 0U; e->reach_hi_dv = 0U; e->iout_rms_ca = 0U; e->iout_pk_ca = 0U;
    if (e2e_iout_on != 0U) { e->iout_rms_ca = e2e_iout_rms; e->iout_pk_ca = e2e_iout_pk; e->flags2 = (uint16_t)(e->flags2 | LINKV2_FLAG_IOUT_VALID | ((e2e_iout_clip != 0U) ? LINKV2_FLAG_IOUT_CLIP : 0U)); }
}
static void    e2e_fill_h(LinkH_t *h) { h->fw = SLAVE_FW_VERSION; h->vin_min_dv = (uint16_t)(CTRL_VIN_MIN_V * 10.0f); h->vin_max_dv = (uint16_t)(CTRL_VIN_MAX_V * 10.0f); h->rst = 0U; h->tuned = 1U; h->vout_max_dv = (uint16_t)(CTRL_VOUT_MAX_V * 10.0f); }
static uint8_t e2e_tx(const char *f, uint32_t n) { if (e2e_cnt >= E2E_MAX) return 0U; memcpy(e2e_f[e2e_cnt], f, n); e2e_n[e2e_cnt] = (int)n; e2e_cnt++; return 1U; }
static void    e2e_bytes(const char *s, uint32_t n) { uint32_t i; for (i = 0U; i < n; i++) SL2_Rx((uint8_t)s[i], mc_ms()); }
#if (SLAVE_TUNE_MODE != 0)
/* ---- the autotune chain on the REAL controller: the link commands, the sweep, the profile store (a RAM flash), the controller's own tune mode ---- */
#include "slave_tune_link.h"
#include "slave_profile.h"
#define TPA 0x1000UL
#define TPB 0x1800UL
static uint8_t tl_mem[4096]; static int tl_flash_fail; static uint32_t tl_last_ms;
static int tl_rd(uint32_t a, uint8_t *b, uint32_t n) { memcpy(b, &tl_mem[a - TPA], n); return 0; }
static int tl_er(uint32_t a) { if (tl_flash_fail) return -1; memset(&tl_mem[a - TPA], 0xFF, 2048U); return 0; }
static int tl_pr(uint32_t a, const uint8_t *b, uint32_t n) { uint32_t i; if (tl_flash_fail) return -1; for (i = 0U; i < n; i++) tl_mem[a - TPA + i] &= b[i]; return 0; }
static const SFlash_t TL_FL = { tl_rd, tl_pr, tl_er };
static uint8_t  tl_stopped(void) { return SlaveCtrl_TuneStopped(); }
static uint8_t  tl_can(void) { return 1U; }
static uint8_t  tl_busy(void) { return 0U; }
static uint16_t tl_fault(void) { return SlaveCtrl_FaultCode(); }
static uint16_t tl_headroom(void) { float h = CTRL_VOUT_MAX_V - W.vin - 10.0f; return (uint16_t)((h > 0.0f) ? (h * 10.0f) : 0.0f); }
static const TuneLinkHw_t TL_HW = { SlaveCtrl_MaxCorrDv, tl_stopped, tl_can, tl_busy, SlaveCtrl_TuneBegin, SlaveCtrl_TuneRequest, SlaveCtrl_TuneEnd, SlaveCtrl_TuneReady, SlaveCtrl_TuneRunning, tl_fault, tl_headroom };
static int e2e_tune_cmd(uint8_t c, const char *n, int32_t v, uint8_t *s, int32_t *val, uint32_t now) { return SlaveTuneLink_Command(c, n, v, s, val, now); }
static int e2e_tune_frame(LinkT_t *f, uint32_t now) { return SlaveTuneLink_NextFrame(f, now); }
static void tl_service(void)
{   /* every 10 ms, as the slave's main loop does: the measurements, the controller's fault state */
    uint32_t now = mc_ms(); TuneIn_t in;
    if ((uint32_t)(now - tl_last_ms) < 10U) return;
    tl_last_ms = now;
    in.vin = W.vin; in.vout = W.vout; in.meas_valid = 1U; in.fault = (uint8_t)(SlaveCtrl_FaultCode() != 0U); in.vin_in_window = (uint8_t)((W.vin >= CTRL_VIN_MIN_V) && (W.vin <= CTRL_VIN_MAX_V));
    in.maxc_v = (float)SlaveCtrl_MaxCorrDv() / 10.0f; in.hw_cap_ticks = (uint16_t)CTRL_STAGE3_MAX_EFF_TICKS;
    SlaveTuneLink_Service(now, &in);
}
#define E2E_TUNE_HOOKS e2e_tune_cmd, e2e_tune_frame
#else
#define E2E_TUNE_HOOKS 0, 0
static void tl_service(void) { }
#endif
static void    e2e_step(uint32_t ms) { uint32_t k; for (k = 0U; k < ms / 5U; k++) { run_for(5000); tl_service(); SL2_Service(mc_ms()); } }
static void    e2e_master(uint8_t run, uint16_t setv, uint8_t clr, uint16_t maxc, int corrupt)
{
    LinkM_t m; char b[96]; int32_t n;
    m.seq = (uint8_t)e2e_seq_m++; m.run = run; m.setv_dv = setv; m.clr = clr; m.maxc_dv = maxc;
    n = LinkV2_BuildM(b, sizeof b, &m);
    if (corrupt) b[n - 4] = (char)((b[n - 4] == '0') ? '1' : '0');
    e2e_bytes(b, (uint32_t)n);
    e2e_step(100);
}
/* a corrupted run = 0 frame: its checksum digit is wrong, so it must be dropped. It arrives IN ADDITION to the valid frames: replacing them would (rightly) trip the link timeout. */
static void    e2e_inject_bad_stop(void)
{
    LinkM_t m; char b[96]; int32_t n;
    m.seq = 0U; m.run = 0U; m.setv_dv = 2300U; m.clr = 4U; m.maxc_dv = 300U;
    n = LinkV2_BuildM(b, sizeof b, &m);
    b[n - 4] = (char)((b[n - 4] == '0') ? '1' : '0');
    e2e_bytes(b, (uint32_t)n);
}
static int e2e_last_e(LinkE_t *out) { int i; LinkV2Frame_t f; for (i = e2e_cnt - 1; i >= 0; i--) if (LinkV2_Parse(e2e_f[i], (uint32_t)e2e_n[i], &f) == LINK_OK && f.type == LINKV2_E) { *out = f.u.e; return 1; } return 0; }
static int e2e_count(LinkV2Type_t t) { int i, c = 0; LinkV2Frame_t f; for (i = 0; i < e2e_cnt; i++) if (LinkV2_Parse(e2e_f[i], (uint32_t)e2e_n[i], &f) == LINK_OK && f.type == t) c++; return c; }

static void t_mc_end_to_end(void)
{
    static const SL2_Hooks_t HK = { e2e_on_control, e2e_fill_e, e2e_fill_h, e2e_tx, 0, 0, E2E_TUNE_HOOKS };
    LinkE_t e; LinkV2Frame_t f; int k, i, ok;
    mc_world(229000000ULL, 225.0f);
    e2e_cnt = 0; e2e_seq_m = 0U;
    SL2_Init(&HK, mc_ms());
    e2e_step(1500);                                                           /* no master yet */
    CHECK(e2e_count(LINKV2_H) == 3, "three hellos at start (%d)", e2e_count(LINKV2_H));
    CHECK(e2e_last_e(&e) && (e.flags2 & LINKV2_FLAG_LINK_LOST) != 0U && e.setv_dv == 0U, "with no master, $E says LINK_LOST and no set voltage");
    for (i = 0; i < e2e_cnt; i++) if (LinkV2_Parse(e2e_f[i], (uint32_t)e2e_n[i], &f) == LINK_OK && f.type == LINKV2_H) { CHECK(f.u.h.proto == 2U && f.u.h.fw == SLAVE_FW_VERSION && f.u.h.vin_min_dv == (uint16_t)(CTRL_VIN_MIN_V * 10.0f), "the hello carries protocol 2, the firmware version and the input window"); break; }
    for (k = 0; (k < 400) && (g_ctrl.state != CTRL_RUN); k++) e2e_master(1U, 2300U, 4U, 300U, 0);
    CHECK(g_ctrl.state == CTRL_RUN, "$M frames as BYTES start the unit (state %d after %d frames)", g_ctrl.state, k);
    for (k = 0; k < 100; k++) e2e_master(1U, 2300U, 4U, 300U, 0);
    CHECK(e2e_last_e(&e) && (e.flags2 & (LINKV2_FLAG_RUN_ACK | LINKV2_FLAG_IN_BAND)) == (LINKV2_FLAG_RUN_ACK | LINKV2_FLAG_IN_BAND) && e.setv_dv == 2300U && e.clr == 4U, "$E reports RUN_ACK + IN_BAND, the set voltage and the counter (flags2 0x%04X, setv %u, clr %u)", (unsigned)e.flags2, (unsigned)e.setv_dv, (unsigned)e.clr);
    /* a corrupted $M with run = 0 is ignored: the unit keeps running */
    for (k = 0; k < 20; k++) { e2e_inject_bad_stop(); e2e_master(1U, 2300U, 4U, 300U, 0); }
    CHECK(g_ctrl.state == CTRL_RUN && SL2_Stats()->rx_bad >= 20U, "20 corrupted run = 0 frames arriving beside the valid ones are dropped and counted, the unit keeps running (state %d, rx_bad %u)", g_ctrl.state, (unsigned)SL2_Stats()->rx_bad);
    /* commands */
    {   LinkK_t kk; char b[96]; int32_t n; int h0 = e2e_count(LINKV2_H), r0 = e2e_count(LINKV2_R);
        memset(&kk, 0, sizeof kk); kk.id = 9U; kk.cmd = LINKCMD_HELLO; n = LinkV2_BuildK(b, sizeof b, &kk); e2e_bytes(b, (uint32_t)n); e2e_step(100);
        CHECK(e2e_count(LINKV2_H) == h0 + 1 && e2e_count(LINKV2_R) == r0 + 1, "HELLO is answered with a response and a hello");
        e2e_bytes(b, (uint32_t)n); e2e_step(100);                              /* the master's retry, same id */
        CHECK(e2e_count(LINKV2_H) == h0 + 1 && e2e_count(LINKV2_R) == r0 + 2, "the retry is answered again but the hello is not sent twice");
        kk.id = 10U; kk.cmd = LINKCMD_TUNE; n = LinkV2_BuildK(b, sizeof b, &kk); e2e_bytes(b, (uint32_t)n); e2e_step(100);
        ok = 0; for (i = e2e_cnt - 1; i >= 0; i--) if (LinkV2_Parse(e2e_f[i], (uint32_t)e2e_n[i], &f) == LINK_OK && f.type == LINKV2_R && f.u.r.id == 10U) { ok = (f.u.r.status == LINKV2_ST_UNKNOWN_CMD); break; }
        CHECK(ok, "TUNE is answered UNKNOWN_CMD in this firmware version"); }
    /* a fault, seen in $E, and the clear counter as bytes */
    SlaveCtrl_OnFltIrq();
    for (k = 0; k < 10; k++) e2e_master(1U, 2300U, 4U, 300U, 0);
    CHECK(e2e_last_e(&e) && (e.flags2 & LINKV2_FLAG_FAULT_LATCHED) != 0U && (e.flags2 & LINKV2_FLAG_IN_BAND) == 0U, "$E shows FAULT_LATCHED after a trip (flags2 0x%04X)", (unsigned)e.flags2);
    for (k = 0; (k < 400) && (g_ctrl.state != CTRL_RUN); k++) e2e_master(1U, 2300U, 5U, 300U, 0);
    CHECK(g_ctrl.state == CTRL_RUN && e2e_last_e(&e) && e.clr == 5U, "clear counter 5 as bytes clears the latch and the unit runs again (state %d)", g_ctrl.state);
    /* run = 0 as bytes stops it */
    for (k = 0; k < 80; k++) e2e_master(0U, 2300U, 5U, 300U, 0);
    CHECK(g_ctrl.state == CTRL_OFF && e2e_last_e(&e) && (e.flags2 & LINKV2_FLAG_RUN_ACK) == 0U, "run = 0 as bytes stops it (state %d)", g_ctrl.state);
    /* the $E seq: every number in order with no gap while the queue had room */
    {   int prev = -1, gaps = 0; for (i = 0; i < e2e_cnt; i++) if (LinkV2_Parse(e2e_f[i], (uint32_t)e2e_n[i], &f) == LINK_OK && f.type == LINKV2_E) { if (prev >= 0 && f.u.e.seq != (uint8_t)(prev + 1)) gaps++; prev = f.u.e.seq; }
        CHECK(gaps == 0 && e2e_count(LINKV2_E) > 100, "%d $E frames in order with no gap (%d gaps)", e2e_count(LINKV2_E), gaps); }
    check_invariants("master control: end to end");
    printf("  master control, end to end: bytes in, the real link logic and controller, frames out: start, in band, a corrupted stop dropped, HELLO and a retry, UNKNOWN_CMD, a latched fault, a clear, a stop\n");
}
/* ---- the REAL master logic (m_ctl) against the REAL slave controller: what the master sends in $M frames, as bytes, is all the slave gets ---- */
#include "m_ctl.h"
static MCtl_t mm_c; static MSettings_t mm_s; static uint32_t mm_seq; static int mm_clr_changes; static uint8_t mm_last_clr;
#if (SLAVE_VOLTAGE_CAL != 0)
/* the slave's REAL calibration logic on a simulated chain: the nominal chain reads exactly the plant's voltages; the flash is a pair of variables */
static uint16_t e2e_cal_applied[2], e2e_cal_stored[2]; static int e2e_cal_have_store, e2e_cal_saves, e2e_cal_fail_save;
static float    e2e_h_raw(uint8_t ch, uint8_t *valid) { *valid = 1U; return ((ch == 0U) ? W.vin : W.vout) / 0.1f; }
static uint32_t e2e_h_cnt(void) { return mc_ms() / 10U; }
static uint8_t  e2e_h_stopped(void) { return (uint8_t)(g_ctrl.state == CTRL_OFF); }
static float    e2e_h_nominal(uint8_t ch) { (void)ch; return 0.1f; }
static void     e2e_h_set(uint8_t ch, uint16_t r) { e2e_cal_applied[ch] = r; }
static int      e2e_h_load(uint16_t r[2]) { if (!e2e_cal_have_store) return -1; r[0] = e2e_cal_stored[0]; r[1] = e2e_cal_stored[1]; return 0; }
static int      e2e_h_save(const uint16_t r[2]) { if (e2e_cal_fail_save) return -1; e2e_cal_stored[0] = r[0]; e2e_cal_stored[1] = r[1]; e2e_cal_have_store = 1; e2e_cal_saves++; return 0; }
static const SlaveCalHw_t E2E_CAL_HW = { e2e_h_raw, e2e_h_cnt, e2e_h_stopped, e2e_h_nominal, e2e_h_set, e2e_h_load, e2e_h_save };
static int e2e_cal_cmd(uint8_t cmd, int32_t num, char letter, uint8_t *st, int32_t *val, uint32_t now)
{ return SlaveCal_Command((cmd == (uint8_t)LINKCMD_CALV) ? SCAL_CMD_CALV : ((cmd == (uint8_t)LINKCMD_CALO) ? SCAL_CMD_CALO : SCAL_CMD_CALR), num, letter, st, val, now); }
static int e2e_cal_poll(uint8_t *st, int32_t *val, uint32_t now) { return SlaveCal_Poll(st, val, now); }
static const SL2_Hooks_t HK2 = { e2e_on_control, e2e_fill_e, e2e_fill_h, e2e_tx, e2e_cal_cmd, e2e_cal_poll, E2E_TUNE_HOOKS };
#else
static const SL2_Hooks_t HK2 = { e2e_on_control, e2e_fill_e, e2e_fill_h, e2e_tx, 0, 0, E2E_TUNE_HOOKS };
#endif
static void mm_start(uint8_t restart_auto)
{
    mc_world(230000000ULL, 225.0f);
    e2e_cnt = 0; SL2_Init(&HK2, mc_ms());
    MS_Defaults(&mm_s); mm_s.restart_auto = restart_auto; mm_s.restart_time = 5U; MCtl_Init(&mm_c); mm_seq = 0U; mm_clr_changes = 0; mm_last_clr = 0U;
}
/* one master tick of 100 ms: decide, send the $M as bytes, let the slave run */
static MCtlOut_t mm_tick(uint8_t setmode, uint8_t start, uint8_t stop)
{
    MCtlOut_t o; LinkM_t m; char b[96]; int32_t n;
    uint8_t sf = (uint8_t)((SlaveCtrl_LinkState() == 3U) || (SlaveCtrl_FaultCode() != 0U));
    MCtl_Step(&mm_c, &mm_s, setmode, start, stop, 1U, sf, mc_ms(), &o);
    m.seq = (uint8_t)mm_seq++; m.run = o.run; m.setv_dv = o.setv_dv; m.clr = o.clr; m.maxc_dv = o.maxc_dv;
    n = LinkV2_BuildM(b, sizeof b, &m);
    if (n > 0) e2e_bytes(b, (uint32_t)n);
    e2e_step(100);
    if (o.clr != mm_last_clr) { mm_clr_changes++; mm_last_clr = o.clr; }
    return o;
}
static uint32_t mm_until_run(uint32_t timeout_ms, uint8_t setmode)
{
    uint32_t k; for (k = 0U; k < timeout_ms / 100U; k++) { if (g_ctrl.state == CTRL_RUN) return k * 100U; (void)mm_tick(setmode, 0U, 0U); } return timeout_ms;
}

static void t_mc_master_logic(void)
{
    uint32_t t, k; MCtlOut_t o;
    /* Auto: the master starts the slave by itself */
    mm_start(1U); t = mm_until_run(60000U, 0U);
    CHECK(g_ctrl.state == CTRL_RUN && t >= 4000U, "Auto: the master starts the slave after its on-delay (RUN after %u ms)", (unsigned)t);
    for (k = 0U; k < 120U; k++) (void)mm_tick(0U, 0U, 0U);
    CHECK((double)W.vout > 228.5 && (double)W.vout < 231.5, "the slave regulates to the master's Set Voltage 230 V (%.2f V)", (double)W.vout);
    /* Set-Mode opens: the slave stops its PWM and trips the output */
    o = mm_tick(1U, 0U, 0U);
    CHECK(o.run == 0U && g_ctrl.run_request == 0U, "Set-Mode opened: in the SAME master tick the slave's run request is 0");
    for (k = 0U; k < 90U; k++) (void)mm_tick(1U, 0U, 0U);
    CHECK(g_ctrl.state == CTRL_OFF && !inv_enabled && g_ctrl.fault == CF_NONE, "9 s later the slave is OFF: PWM stopped, output tripped, and it is not a fault (state %d, inverter %d)", g_ctrl.state, inv_enabled);
    for (k = 0U; k < 600U; k++) (void)mm_tick(1U, 0U, 0U);
    CHECK(g_ctrl.state == CTRL_OFF && !is_running(), "and it stays off for a minute while Set-Mode is open (the master keeps sending run = 0)");
    /* a new Set Voltage made in Set-Mode reaches the slave and is regulated to after leaving */
    mm_s.set_v = 240U; mm_s.maxcorr_v = 25U;
    for (k = 0U; k < 10U; k++) (void)mm_tick(1U, 0U, 0U);
    CHECK(SlaveCtrl_SetvDv() == 2400U && SlaveCtrl_MaxCorrDv() == 250U, "the slave has the new Set Voltage 2400 and Max Correction 250 (while stopped, it takes them at once)");
    t = mm_until_run(60000U, 0U);
    for (k = 0U; k < 120U; k++) (void)mm_tick(0U, 0U, 0U);
    CHECK(g_ctrl.state == CTRL_RUN && (double)W.vout > 238.5 && (double)W.vout < 241.5, "after leaving Set-Mode it starts again and regulates to 240 V (%.2f V, RUN after %u ms)", (double)W.vout, (unsigned)t);
    CHECK(SlaveCtrl_SetvDv() == 2400U, "and reports 2400 as the set voltage in use");
    /* a Set Voltage changed while the slave RUNS is ignored until it stops (the master only changes it in Set-Mode, but the rule is the slave's) */
    mm_s.set_v = 230U; for (k = 0U; k < 100U; k++) (void)mm_tick(0U, 0U, 0U);
    CHECK(SlaveCtrl_SetvDv() == 2400U && (double)W.vout > 238.5, "a change while running is not adopted (the slave keeps regulating to 240 V)");
    /* a trip: Auto clears it by itself */
    mm_s.set_v = 240U;
    SlaveCtrl_OnFltIrq(); run_for(100000);
    CHECK(g_ctrl.state == CTRL_LATCHED, "a hardware trip latches the slave");
    { int c0 = mm_clr_changes; t = mm_until_run(90000U, 0U);
      CHECK(g_ctrl.state == CTRL_RUN && mm_clr_changes > c0, "Auto: the master changes the clear counter and the slave runs again by itself (RUN after %u ms, %d clear changes)", (unsigned)t, mm_clr_changes - c0); }
    /* Manual: stop at the fault, wait for START */
    mm_start(0U); (void)mm_tick(0U, 1U, 0U); t = mm_until_run(60000U, 0U);
    CHECK(g_ctrl.state == CTRL_RUN, "Manual: START runs it (after %u ms)", (unsigned)t);
    for (k = 0U; k < 100U; k++) (void)mm_tick(0U, 0U, 0U);
    SlaveCtrl_OnFltIrq(); run_for(100000);
    for (k = 0U; k < 600U; k++) (void)mm_tick(0U, 0U, 0U);
    CHECK(g_ctrl.state == CTRL_LATCHED && !is_running(), "Manual: after the trip it stays stopped for a minute");
    (void)mm_tick(0U, 1U, 0U); t = mm_until_run(60000U, 0U);
    CHECK(g_ctrl.state == CTRL_RUN, "and START clears it and runs it again (RUN after %u ms)", (unsigned)t);
    /* STOP */
    (void)mm_tick(0U, 0U, 1U); for (k = 0U; k < 90U; k++) (void)mm_tick(0U, 0U, 0U);
    CHECK(g_ctrl.state == CTRL_OFF && !inv_enabled, "STOP: the slave is off");
    check_invariants("master control: real master logic");
    printf("  real master logic against the real slave: it starts by itself (Auto), Set-Mode stops PWM at once, new settings arrive and are regulated to, a trip is cleared (Auto) or waits for START (Manual), STOP\n");
}

/* ---- the WHOLE real master application (keys, screens, fault engine, control) against the REAL slave controller. The master sees the slave only through what the slave SENDS ($E frames as
 * bytes, and its status values) and the slave sees the master only through the $M frames the master sends, as bytes. Keys are raw levels with real timing. ---- */
#include "m_app.h"
static uint8_t ma_fm[8U * MFLASH_PAGE];
static int ma_rd(uint32_t a, uint8_t *b, uint32_t n) { memcpy(b, &ma_fm[a], n); return 0; }
static int ma_pg(uint32_t a, const uint8_t *b, uint32_t n) { uint32_t i; for (i = 0U; i < n; i++) ma_fm[a + i] &= b[i]; return 0; }
static int ma_er(uint32_t a) { memset(&ma_fm[a], 0xFF, MFLASH_PAGE); return 0; }
static const MFlash_t MAFL = { ma_rd, ma_pg, ma_er };
static MApp_t ma; static MAppData_t md; static char ma_f[4][21]; static uint8_t ma_k[4]; static uint32_t ma_t_tx, ma_seq, ma_last_run; static uint8_t ma_have_last; static MCtlOut_t ma_last;
static int ma_link_up; static uint8_t ma_buzz_seen; static int ma_mute_m;

static void ma_start(uint8_t restart_auto, float vin)
{
    mc_world(231000000ULL, vin);
    e2e_cnt = 0; e2e_seq_m = 0U; SL2_Init(&HK2, mc_ms());
#if (SLAVE_VOLTAGE_CAL != 0)
    e2e_cal_have_store = 0; e2e_cal_saves = 0; e2e_cal_fail_save = 0; SlaveCal_Init(&E2E_CAL_HW);
#endif
#if (SLAVE_TUNE_MODE != 0)
    memset(tl_mem, 0xFF, sizeof tl_mem); tl_flash_fail = 0; tl_last_ms = mc_ms(); SlaveProfile_Init(&TL_FL, TPA, TPB); SlaveTuneLink_Init(&TL_HW);
#endif
    memset(ma_fm, 0xFF, sizeof ma_fm); memset(ma_k, 0, sizeof ma_k); memset(&md, 0, sizeof md);
    MApp_Init(&ma, &MAFL, 0U, MFLASH_PAGE, 2U * MFLASH_PAGE, 3U * MFLASH_PAGE, mc_ms());
    ma.settings.restart_auto = restart_auto; ma.settings.ip_lv = 180U; ma.settings.ip_hv = 280U; ma.settings.op_lv = 200U; ma.settings.op_hv = 250U;      /* the limits these tests were written for; the NEW default limits have their own tests */
    MF_Init(&ma.eng, &ma.settings, mc_ms());
    ma_t_tx = 0U; ma_seq = 0U; ma_have_last = 0U; ma_link_up = 1; ma_buzz_seen = 0U; ma_mute_m = 0;
}
/* 10 ms of the master: what the slave sent, the key levels, the application, the $M frame (every 100 ms and at once on a change), 10 ms of the slave */
static void ma_ms10(void)
{
    LinkE_t e; uint32_t now = mc_ms();
    md.have = (uint8_t)(ma_link_up != 0); md.link_ok = md.have;
    md.vin_dv = (uint16_t)(W.vin * 10.0f); md.vout_dv = (uint16_t)(W.vout * 10.0f); md.freq_chz = 5000U;
    md.state = SlaveCtrl_LinkState(); md.fault = (uint8_t)SlaveCtrl_FaultCode();
    md.load_ok = 1U; md.load_da = 100U;
    if (e2e_last_e(&e)) { md.e_valid = 1U; md.in_band = (uint8_t)((e.flags2 & LINKV2_FLAG_IN_BAND) != 0U); md.clr_echo = e.clr; md.iout_rms_ca = e.iout_rms_ca; md.iout_pk_ca = e.iout_pk_ca; md.iout_valid = (uint8_t)((e.flags2 & LINKV2_FLAG_IOUT_VALID) != 0U); md.iout_clip = (uint8_t)((e.flags2 & LINKV2_FLAG_IOUT_CLIP) != 0U); }
    {   int ri; LinkV2Frame_t rf;                                                                 /* the slave's answers to a $K ($R) reach the master application */
        for (ri = 0; ri < e2e_cnt; ri++) if (LinkV2_Parse(e2e_f[ri], (uint32_t)e2e_n[ri], &rf) == LINK_OK)
        {
            if (rf.type == LINKV2_R) MApp_OnKResult(&ma, rf.u.r.id, rf.u.r.status, rf.u.r.value);
            else if (rf.type == LINKV2_T) { md.tune_step = rf.u.t.step; md.tune_result = rf.u.t.result; md.tune_ticks = rf.u.t.ticks; md.tune_vin_dv = rf.u.t.vin_dv; md.tune_dv_dv = rf.u.t.dv; md.tune_n++; }
        } }
    e2e_cnt = 0;                                                                                  /* keep only the NEWEST $E: the capture buffer must never fill up and go stale */
    MApp_Step(&ma, ma_k[0], ma_k[1], ma_k[2], ma_k[3], &md, now, ma_f);
    if (MApp_Engine(&ma)->buzzer != 0U) ma_buzz_seen = 1U;
    {   MCalK_t ck; if (MApp_TakeKCommand(&ma, &ck, now) != 0U)                                    /* a calibration command (and its retries) goes to the slave */
        {   LinkK_t k; char b[96]; int32_t n;
            k.id = ck.id; k.cmd = (ck.cmd == MAPP_KCMD_CALV) ? LINKCMD_CALV : ((ck.cmd == MAPP_KCMD_CALO) ? LINKCMD_CALO : ((ck.cmd == MAPP_KCMD_TUNE) ? LINKCMD_TUNE : ((ck.cmd == MAPP_KCMD_TABORT) ? LINKCMD_TABORT : LINKCMD_CALR))); k.num = ck.num; k.letter = ck.letter; k.name[0] = '\0';
            n = LinkV2_BuildK(b, sizeof b, &k); if (n > 0) e2e_bytes(b, (uint32_t)n); } }
    if ((MApp_ControlValid(&ma) != 0U) && (ma_mute_m == 0))
    {
        const MCtlOut_t *c = MApp_Control(&ma);
        if ((ma_have_last == 0U) || (c->run != ma_last.run) || (c->clr != ma_last.clr) || (c->setv_dv != ma_last.setv_dv) || (c->maxc_dv != ma_last.maxc_dv) || ((uint32_t)(now - ma_t_tx) >= 100U))
        {
            LinkM_t m; char b[96]; int32_t n;
            m.seq = (uint8_t)ma_seq++; m.run = c->run; m.setv_dv = c->setv_dv; m.clr = c->clr; m.maxc_dv = c->maxc_dv;
            n = LinkV2_BuildM(b, sizeof b, &m); if (n > 0) e2e_bytes(b, (uint32_t)n);
            ma_t_tx = now; ma_last = *c; ma_have_last = 1U;
        }
    }
    e2e_step(10);
}
static void ma_run(uint32_t ms) { uint32_t k; for (k = 0U; k < ms / 10U; k++) ma_ms10(); }
static uint32_t ma_until(int (*cond)(void), uint32_t timeout_ms) { uint32_t k; for (k = 0U; k < timeout_ms / 10U; k++) { if (cond()) return k * 10U; ma_ms10(); } return timeout_ms; }
static int ma_c_run(void) { return g_ctrl.state == CTRL_RUN; }
static int ma_c_contactor(void) { return MApp_Engine(&ma)->contactor[0] != 0U; }
static int ma_c_latched(void) { return g_ctrl.state == CTRL_LATCHED; }
static void ma_tap(int key, uint32_t ms) { ma_k[key] = 1U; ma_run(ms); ma_k[key] = 0U; ma_run(120U); }

static void t_mc_master_app(void)
{
    uint32_t t, t_run, t_con; uint32_t k;
    /* ---- Auto: power-up on-delay, the slave runs, and the contactor closes ONLY after IN_BAND has held ---- */
    ma_start(1U, 225.0f);
    t_run = ma_until(ma_c_run, 40000U);
    CHECK(g_ctrl.state == CTRL_RUN && t_run >= 4500U, "Auto: the master starts the slave after the 5 s on-delay (RUN after %u ms)", (unsigned)t_run);
    CHECK(!ma_c_contactor(), "at that moment the contactor is still OPEN");
    t_con = ma_until(ma_c_contactor, 30000U);
    CHECK(ma_c_contactor() && t_con >= 900U && (double)W.vout > 227.5 && (double)W.vout < 232.5, "the contactor closes when the output is AT the set voltage (%.1f V) and has stayed there for about 1 s (%u ms after the slave started)", (double)W.vout, (unsigned)t_con);
    ma_run(8000U);
    CHECK((double)W.vout > 228.5 && (double)W.vout < 231.5 && ma_c_contactor(), "regulating to 230 V with the contactor closed (%.2f V)", (double)W.vout);
    CHECK(ma.eout.fault[0] == 0U && MLog_Count(&ma.log) == 0U && !MApp_Engine(&ma)->buzzer, "no fault, nothing logged, buzzer off");
    /* ---- Set-Mode by the raw keys: the contactor opens and the slave stops IN THE SAME 10 ms step ---- */
    ma_k[0] = ma_k[3] = 1U;
    for (k = 0U; k < 400U && !MApp_SetModeOpen(&ma); k++) ma_ms10();
    CHECK(MApp_SetModeOpen(&ma) && !ma_c_contactor() && g_ctrl.run_request == 0U, "MENU+LOG for 2 s opens Set-Mode: in that same step the contactor is open and the slave's run request is 0");
    ma_k[0] = ma_k[3] = 0U; ma_run(9000U);
    CHECK(g_ctrl.state == CTRL_OFF && !inv_enabled && g_ctrl.fault == CF_NONE && MLog_Count(&ma.log) == 0U, "9 s later the slave is OFF, PWM stopped, no fault, nothing logged (Set-Mode is not a fault)");
    /* edit Set Voltage 230 -> 240 and leave with MENU+LOG */
    ma_tap(0, 100U); ma_tap(0, 100U); ma_tap(2, 100U); ma_tap(1, 100U); ma_tap(0, 100U);
    ma_k[0] = ma_k[3] = 1U; ma_run(2150U); ma_k[0] = ma_k[3] = 0U; ma_run(300U);
    CHECK(!MApp_SetModeOpen(&ma) && ma.settings.set_v == 240U, "saved: Set Voltage 240 V");
    t = ma_until(ma_c_contactor, 60000U);
    CHECK(ma_c_contactor() && g_ctrl.state == CTRL_RUN, "after leaving Set-Mode the unit restarts by itself and the contactor closes again (%u ms)", (unsigned)t);
    ma_run(10000U);
    CHECK((double)W.vout > 238.5 && (double)W.vout < 241.5 && SlaveCtrl_SetvDv() == 2400U, "and regulates to the NEW Set Voltage: 240 V (%.2f V; the slave reports %u)", (double)W.vout, (unsigned)SlaveCtrl_SetvDv());
    /* ---- STOP and START keys ---- */
    ma_tap(2, 100U);
    CHECK(!ma_c_contactor() && ma.ctl_out.run == 0U, "STOP: the contactor opens and run = 0");
    ma_run(8000U); CHECK(g_ctrl.state == CTRL_OFF && !ma_c_contactor(), "the slave stops and nothing restarts it by itself");
    ma_tap(1, 100U); t = ma_until(ma_c_contactor, 60000U);
    CHECK(ma_c_contactor(), "START: it starts and the contactor closes again (%u ms)", (unsigned)t);
    /* ---- an inrush trip: the contactor opens at once, it is logged, the buzzer sounds, and it retries after 30 s ---- */
    ma_run(10000U); ma_buzz_seen = 0U;
    SlaveCtrl_OnFltIrq();
    ma_run(300U);
    CHECK(!ma_c_contactor() && g_ctrl.state == CTRL_LATCHED, "a hardware trip: the contactor is open within 0.3 s and the slave is latched");
    CHECK(MLog_Count(&ma.log) == 1U && ma_buzz_seen, "it is logged once (%u) and the buzzer sounded", (unsigned)MLog_Count(&ma.log));
    { MLogRec_t r; CHECK(MLog_Get(&ma.log, 0U, &r) == 0 && r.type == MLOG_T_INRUSH && r.slave_code == 2U, "as an Inrush, with the slave's code 2"); }
    ma_run(25000U); CHECK(!ma_c_contactor() && g_ctrl.state == CTRL_LATCHED, "25 s later: still latched and open (the retry waits 30 s)");
    t = ma_until(ma_c_run, 20000U);
    CHECK(g_ctrl.state == CTRL_RUN, "after 30 s the master clears the slave and it runs again (RUN %u ms later)", (unsigned)t);
    t = ma_until(ma_c_contactor, 30000U); CHECK(ma_c_contactor(), "and the contactor closes once IN_BAND has held (%u ms)", (unsigned)t);
    CHECK(!MApp_Engine(&ma)->buzzer, "the buzzer is off again");
    /* ---- three more inrush trips in a row: after the fourth the retries are used up: a 5-minute wait, then ONE more attempt; if that trips too it latches; START tries again ---- */
    for (k = 0U; k < 3U; k++)
    {
        ma_run(3000U); SlaveCtrl_OnFltIrq(); ma_run(500U);
        if (k < 2U) { (void)ma_until(ma_c_run, 40000U); ma_run(500U); }
    }
    ma_run(100000U);
    CHECK(g_ctrl.state == CTRL_LATCHED && !ma_c_contactor() && ma.eng.u[0].st == (uint8_t)U_FAULT && MF_RestartLeftS(&ma.eng, 0U) > 150U, "after the 4th trip the retries are used up: the long wait, not latched yet; the screen counts about 3 more minutes (%u s)", (unsigned)MF_RestartLeftS(&ma.eng, 0U));
    t = ma_until(ma_c_run, 240000U);
    CHECK(g_ctrl.state == CTRL_RUN, "after the 5 minutes the master makes ONE more attempt and the slave runs (%u ms)", (unsigned)t);
    ma_run(3000U); SlaveCtrl_OnFltIrq(); ma_run(500U); ma_run(100000U);
    CHECK(g_ctrl.state == CTRL_LATCHED && !ma_c_contactor() && ma.eng.u[0].st == (uint8_t)U_LATCHED, "that extra attempt trips too: latched, contactor open, even 100 s later");
    ma_tap(2, 100U); CHECK(!MApp_Engine(&ma)->buzzer && ma.eng.u[0].st == (uint8_t)U_LATCHED, "STOP silences the buzzer; the latch stays");
    CHECK(MLog_Count(&ma.log) >= 5U, "every attempt was logged (%u entries)", (unsigned)MLog_Count(&ma.log));
    ma_tap(1, 100U); t = ma_until(ma_c_run, 40000U); CHECK(g_ctrl.state == CTRL_RUN, "START on the latched inrush tries again (%u ms)", (unsigned)t);
    /* ---- link lost: the contactor opens at once, and it restarts by itself when the link returns ---- */
    ma_start(1U, 225.0f); (void)ma_until(ma_c_contactor, 60000U);
    ma_link_up = 0; ma_run(100U);
    CHECK(!ma_c_contactor() && ma.eout.fault[0] == MLOG_T_LINK_LOST, "no frames from the slave: the contactor opens at once, Slave Link Lost");
    ma_link_up = 1; t = ma_until(ma_c_contactor, 60000U);
    CHECK(ma_c_contactor(), "frames back: it restarts by itself and the contactor closes again (%u ms)", (unsigned)t);
    /* ---- a low input: I/P LV after the Volt Cut Delay, restart 5 V inside the limit ---- */
    ma_run(5000U); W.vin = 175.0f; ma_run(4500U);
    CHECK(ma_c_contactor(), "175 V (the I/P LV is 180 V): 4.5 s is not yet a trip");
    ma_run(1500U);
    CHECK(!ma_c_contactor() && ma.eout.fault[0] == MLOG_T_IP_LV, "after the 5 s Volt Cut Delay: I/P LV, contactor open");
    W.vin = 183.0f; ma_run(30000U);
    CHECK(!ma_c_contactor(), "183 V is not 5 V inside the limit: no restart after 30 s (hysteresis)");
    W.vin = 215.0f; t = ma_until(ma_c_contactor, 80000U);          /* inside the master's limit by 5 V AND inside this slave's own start window */
    CHECK(ma_c_contactor(), "215 V: it restarts by itself and the contactor closes (%u ms)", (unsigned)t);
    /* ---- Manual: nothing starts by itself; START does ---- */
    ma_start(0U, 225.0f); ma_run(30000U);
    CHECK(g_ctrl.state == CTRL_OFF && !ma_c_contactor(), "Manual: 30 s after power-up nothing has started");
    ma_tap(1, 100U); t = ma_until(ma_c_contactor, 60000U); CHECK(ma_c_contactor(), "START starts it and the contactor closes (%u ms)", (unsigned)t);
    /* ---- the REAL slave stops WITHOUT a fault (it hears no $M for a second): PWM off, so the master must open the load contactor ---- */
    ma_start(1U, 225.0f); (void)ma_until(ma_c_contactor, 60000U); ma_run(5000U);
    CHECK(ma_c_contactor() && g_ctrl.state == CTRL_RUN, "running, contactor closed");
    ma_mute_m = 1;                                                                  /* the commands stop arriving; the slave's own status frames still reach the master */
    { uint32_t k, t_off = 0U, t_open = 0U;
      for (k = 0U; k < 400U; k++)
      {
          ma_ms10();
          if (t_off == 0U && g_ctrl.state != CTRL_RUN) t_off = (k + 1U) * 10U;
          if (t_open == 0U && !ma_c_contactor()) t_open = (k + 1U) * 10U;
      }
      CHECK(t_off >= 900U && t_off <= 1600U && g_ctrl.fault == CF_NONE, "no $M for 1 s: the slave stops by itself, with NO fault code (left RUN at %u ms, fault %d)", (unsigned)t_off, g_ctrl.fault);
      CHECK(t_open != 0U && t_open >= t_off && t_open <= t_off + 600U, "and the master opens the contactor within 0.6 s of the slave leaving RUN (%u ms after, %u ms after it left)", (unsigned)t_open, (unsigned)(t_open - t_off));
      CHECK(MLog_Count(&ma.log) >= 1U, "logged"); }
    ma_mute_m = 0; t = ma_until(ma_c_contactor, 60000U); CHECK(ma_c_contactor(), "commands back: it restarts and the contactor closes again (%u ms)", (unsigned)t);
    check_invariants("master control: the master application");
    printf("  the REAL master application against the REAL slave: Auto start, the contactor only after IN_BAND, Set-Mode by raw keys, a new Set Voltage regulated to, STOP/START, an inrush with retries and the latch, link loss, input low, Manual\n");
}


#if (SLAVE_INJ_GUARD != 0)
/* ---- input steps while running: the guard protects the first moments, then the output must come back FAST and never stay stuck ---- */
static double mc_back_within(double band_v, uint32_t limit_ms, uint16_t setv_dv)
{
    uint32_t k;
    for (k = 0U; k < limit_ms / 20U; k++)
    {
        SlaveCtrl_OnControl(1U, setv_dv, 1U, 0U, mc_ms()); run_for(20000);
        if (g_ctrl.state == CTRL_RUN && fabs((double)W.vout - ((double)setv_dv / 10.0)) <= band_v) return (double)(k * 20U) / 1000.0;
    }
    return -1.0;
}
static void t_mc_input_steps(void)
{
    double t; uint32_t k;
    /* a boost step: 230 -> 200 V */
    mc_world(250000000ULL, 230.0f); (void)mc_until_run(2300U, 1U, 60000U); mc_for(15000U, 1U, 2300U, 1U, 0U);
    W.vin = 200.0f; t = mc_back_within(2.0, 8000U, 2300U);
    CHECK(t >= 0.0 && t <= 3.0 && g_ctrl.fault == CF_NONE, "boost step 230 -> 200 V: the output is back within 2 V of 230 V in %.1f s (limit 3 s), no fault", t);
    /* a buck step: 200 -> 250 V. With the guard's old deadlock the output stayed at 250 V for ever. */
    mc_for(10000U, 1U, 2300U, 1U, 0U);
    W.vin = 250.0f; t = mc_back_within(2.0, 8000U, 2300U);
    CHECK(t >= 0.0 && t <= 3.0 && g_ctrl.fault == CF_NONE, "buck step 200 -> 250 V: back within 2 V in %.1f s (limit 3 s): the guard releases (it used to deadlock and hold the output at the input)", t);
    mc_for(20000U, 1U, 2300U, 1U, 0U);
    CHECK((double)W.vout > 228.0 && (double)W.vout < 232.0, "and it stays there: %.1f V 20 s later", (double)W.vout);
    /* a small step crossing the set voltage */
    W.vin = 245.0f; t = mc_back_within(2.0, 8000U, 2300U); CHECK(t >= 0.0 && t <= 3.0, "small buck step to 245 V: back in %.1f s", t);
    /* several fast swings in a row (the variac): the guard acts, the output follows within seconds each time, nothing latches, nothing deadlocks */
    mc_world(251000000ULL, 215.0f); (void)mc_until_run(2300U, 1U, 60000U); mc_for(15000U, 1U, 2300U, 1U, 0U);
    for (k = 0U; k < 8U; k++)
    {
        W.vin = (k & 1U) ? 205.0f : 255.0f; mc_for(1500U, 1U, 2300U, 1U, 0U);          /* a swing every 1.5 s */
        CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "swing %u: still running, no fault", (unsigned)(k + 1U));
    }
    W.vin = 225.0f; t = mc_back_within(2.0, 10000U, 2300U);
    CHECK(t >= 0.0 && t <= 4.0, "after 8 swings the output settles at 230 V within %.1f s of the last one", t);
    CHECK(SlaveCtrl_GuardCount() >= 8U, "the guard acted on the swings (%u events)", (unsigned)SlaveCtrl_GuardCount());
    printf("  input steps: boost and buck steps recover in about 2 s, 8 fast swings never stick, the guard releases every time\n");
}
#endif

#if (SLAVE_INJ_GUARD != 0)
/* ---- the MASTER owns the limits: an input the old narrowed slave (trip below 195 V) would have refused, inside the slave's built-in envelope (140 to 300 V) ---- */
static void t_mc_master_limits(void)
{
    uint32_t t;
    ma_start(1U, 185.0f); ma.settings.ip_lv = 170U; ma.settings.set_v = 205U;                 /* the master's I/P LV 170 V, Set Voltage 205 V (above the default O/P LV of 200 V, as Set-Mode requires) */
    t = ma_until(ma_c_contactor, 90000U); ma_run(15000U);
    CHECK(ma_c_contactor() && g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE && (double)W.vout > 203.0 && (double)W.vout < 207.0, "185 V in, I/P LV 170 V, Set Voltage 205 V: it starts, regulates to %.1f V and the contactor closes (%u ms)", (double)W.vout, (unsigned)t);
    /* the master's own delayed rule decides at its limit: 165 V is below I/P LV 170 V: the master waits the delay, then trips (not the slave, which is still inside its envelope) */
    W.vin = 165.0f; ma_run(2000U);
    CHECK(ma_c_contactor() && g_ctrl.fault == CF_NONE, "input at 165 V (below the master's I/P LV): after 2 s the contactor is still closed: the Volt Cut Delay is being waited");
    ma_run(5000U);
    CHECK(!ma_c_contactor() && MLog_Count(&ma.log) >= 1U && g_ctrl.fault == CF_NONE, "after the 5 s delay the MASTER opens it and logs it; the slave itself raised no fault (%d)", g_ctrl.fault);
    printf("  master owns the limits: 185 V in with I/P LV 170 V and Set Voltage 205 V runs; the master trips at its own limit after its own delay\n");
}
#endif


/* ---- the unit works from 90 V: the slave's input trip is 80 V (starts from 90 V), the master's I/P LV can be 90 V ---- */
static void t_mc_from_90v(void)
{
    uint32_t t; MLogRec_t r;
    /* 96 V in, Set Voltage 100 V with its default limits (I/P 90-300, O/P 90-280) */
    ma_start(1U, 96.0f); ma.settings.set_v = 100U; MS_DefaultLimits(&ma.settings, 0, &ma.settings.ip_lv, &ma.settings.ip_hv, &ma.settings.op_lv, &ma.settings.op_hv);
    t = ma_until(ma_c_contactor, 90000U); ma_run(10000U);
    CHECK(ma_c_contactor() && g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE && (double)W.vout > 98.0 && (double)W.vout < 102.0,
          "96 V in, Set Voltage 100 V: the slave starts, regulates to %.1f V and the contactor closes (%u ms; state %d, fault %d)", (double)W.vout, (unsigned)t, g_ctrl.state, g_ctrl.fault);
    CHECK(MLog_Count(&ma.log) == 0U, "and nothing is logged (the master's I/P LV is 90 V, the slave trips only below 80 V)");
    /* below the master's I/P LV of 90 V it is not started (Input LV); just inside it (92 V) is still within the 5 V hysteresis after a violation; 96 V starts it */
    ma_start(1U, 88.0f); ma.settings.set_v = 100U; MS_DefaultLimits(&ma.settings, 0, &ma.settings.ip_lv, &ma.settings.ip_hv, &ma.settings.op_lv, &ma.settings.op_hv);
    ma_run(20000U);
    CHECK(!ma_c_contactor() && MApp_Control(&ma)->run == 0U && g_ctrl.state == CTRL_OFF, "88 V in, below I/P LV 90 V: it is not started (Input LV), the slave is not even asked (state %d)", g_ctrl.state);
    W.vin = 92.0f; ma_run(20000U);
    CHECK(MApp_Control(&ma)->run == 0U && !ma_c_contactor(), "92 V: inside I/P LV but not 5 V inside it after the violation: still not started (hysteresis)");
    W.vin = 96.0f; t = ma_until(ma_c_contactor, 60000U); ma_run(5000U);
    CHECK(ma_c_contactor() && g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "96 V: 5 V inside: it starts, regulates and the contactor closes (%u ms)", (unsigned)t);
    /* the master's own delayed limit: running at 96 V, the input falls to 85 V (above the slave's 80 V trip): the master waits its Volt Cut Delay, then trips and logs */
    ma_start(1U, 96.0f); ma.settings.set_v = 100U; MS_DefaultLimits(&ma.settings, 0, &ma.settings.ip_lv, &ma.settings.ip_hv, &ma.settings.op_lv, &ma.settings.op_hv);
    (void)ma_until(ma_c_contactor, 90000U); ma_run(8000U);
    W.vin = 85.0f; ma_run(3000U);
    CHECK(ma_c_contactor() && g_ctrl.fault == CF_NONE, "input falls to 85 V: after 3 s the contactor is still closed (the Volt Cut Delay is 5 s, the slave does not trip above 80 V)");
    ma_run(4000U);
    CHECK(!ma_c_contactor() && MLog_Count(&ma.log) >= 1U && MLog_Get(&ma.log, 0U, &r) == 0 && r.type == MLOG_T_IP_LV && g_ctrl.fault == CF_NONE, "after the 5 s the MASTER opens it and logs I/P LV; the slave raised no fault of its own (%d)", g_ctrl.fault);
    printf("  works from 90 V: starts and regulates from 96 V, refused at 88 V, the 5 V hysteresis at the low edge, the master trips at its own I/P LV after its own delay\n");
}




#ifdef FAST_CORRECTION_TEST
/* ---- the FAST CORRECTION profile (guard restart slope 60 V/s, freeze debounce 150 ms): input steps are answered about twice as fast, and nothing else gets worse ---- */
static void t_mc_fast_correction(void)
{
    uint32_t k; double t5 = -1.0, t2 = -1.0, mx = 0.0;
    /* a 50 V boost step (230 -> 180 V): within 5 V of the set voltage in about 0.9 s (1.7 s at 30 V/s) */
    mc_world(250000000ULL, 230.0f); (void)mc_until_run(2300U, 1U, 60000U); mc_for(15000U, 1U, 2300U, 1U, 0U);
    W.vin = 180.0f;
    for (k = 0U; k < 400U; k++) { SlaveCtrl_OnControl(1U, 2300U, 1U, 0U, mc_ms()); run_for(20000); if (g_ctrl.state == CTRL_RUN && fabs((double)W.vout - 230.0) <= 5.0) { t5 = (double)(k * 20U) / 1000.0; break; } }
    CHECK(t5 >= 0.0 && t5 <= 1.2 && g_ctrl.fault == CF_NONE, "a 50 V boost step (230 -> 180 V): within 5 V in %.2f s (limit 1.2 s; 1.7 s at the old 30 V/s), no fault", t5);
    t2 = mc_back_within(2.0, 4000U, 2300U); CHECK(t2 >= 0.0 && t2 <= 0.5, "and within 2 V %.2f s after that", t2);
    for (k = 0U; k < 400U; k++) { double d; SlaveCtrl_OnControl(1U, 2300U, 1U, 0U, mc_ms()); run_for(20000); d = fabs((double)W.vout - 230.0); if (d > mx) mx = d; }
    CHECK(mx < 3.0 && g_ctrl.state == CTRL_RUN, "it settles: the largest error in the next 8 s is %.2f V, still running", mx);
    /* a 70 V buck step (180 -> 250 V) */
    mc_for(10000U, 1U, 2300U, 1U, 0U); W.vin = 250.0f; t5 = -1.0;
    for (k = 0U; k < 400U; k++) { SlaveCtrl_OnControl(1U, 2300U, 1U, 0U, mc_ms()); run_for(20000); if (g_ctrl.state == CTRL_RUN && fabs((double)W.vout - 230.0) <= 5.0) { t5 = (double)(k * 20U) / 1000.0; break; } }
    CHECK(t5 >= 0.0 && t5 <= 1.0 && g_ctrl.fault == CF_NONE, "a buck step (180 -> 250 V): within 5 V in %.2f s (limit 1.0 s), no fault", t5);
    /* a small step, from a settled 230 V: under the 15 V of the guard, so the slope does not apply: it must not get SLOWER */
    mc_for(10000U, 1U, 2300U, 1U, 0U); W.vin = 230.0f; mc_for(10000U, 1U, 2300U, 1U, 0U); W.vin = 220.0f; t2 = -1.0;
    for (k = 0U; k < 400U; k++) { SlaveCtrl_OnControl(1U, 2300U, 1U, 0U, mc_ms()); run_for(20000); if (g_ctrl.state == CTRL_RUN && fabs((double)W.vout - 230.0) <= 2.0) { t2 = (double)(k * 20U) / 1000.0; break; } }
    CHECK(t2 >= 0.0 && t2 <= 0.5, "a 10 V step (230 -> 220 V, no guard): within 2 V in %.2f s (limit 0.5 s)", t2);
    printf("  fast correction profile (guard restart slope 60 V/s): a 50 V step is answered in about 0.9 s instead of 1.7 s; no faults, it settles, small steps are not slower\n");
}
#endif


/* ---- the converter-side current through the REAL link: the slave's $E (optional fields) -> the master's Converter Current page ---- */
#if (SLAVE_TUNE_MODE != 0)
/* ---- TUNE MODE: the real master application starts a sweep over the link, the REAL controller starts itself and applies the sweep's duty open loop against the plant, the real tune core measures, the profile is stored ---- */
typedef struct { double max_inj, max_wrong, max_slew; uint32_t max_eff; int saw_run, contactor, master_run, fault; uint32_t t_run; } TnObs_t;
static TnObs_t tn; static double tn_prev_eff;
static void tn_sample(void)
{
    double inj = (double)(W.vout - W.vin), eff = (g_ctrl.duty_counts > (uint16_t)POWER_DEADTIME_TICKS) ? (double)(g_ctrl.duty_counts - (uint16_t)POWER_DEADTIME_TICKS) : 0.0;
    if (inj > tn.max_inj) tn.max_inj = inj;
    if (-inj > tn.max_wrong) tn.max_wrong = -inj;
    if (eff > (double)tn.max_eff) tn.max_eff = (uint32_t)eff;
    if (((eff - tn_prev_eff) / 0.01) > tn.max_slew) tn.max_slew = (eff - tn_prev_eff) / 0.01;
    tn_prev_eff = eff;
    if (g_ctrl.state == CTRL_RUN) { tn.saw_run = 1; if (tn.t_run == 0U) tn.t_run = mc_ms(); }
    if (ma_c_contactor()) tn.contactor = 1;
    if (ma_have_last != 0U && ma_last.run != 0U) tn.master_run = 1;
    if (g_ctrl.fault != 0U) tn.fault = (int)g_ctrl.fault;
}
static void tn_run(uint32_t ms) { uint32_t k; for (k = 0U; k < ms / 10U; k++) { ma_run(10U); tn_sample(); } }
static int tn_until_result(uint32_t max_ms) { uint32_t k; for (k = 0U; k < max_ms / 10U; k++) { ma_run(10U); tn_sample(); if (ma.tune_state == 3U) return 1; } return 0; }
static void tn_begin_at(double gain_k, uint16_t maxc_v, float vin)
{
    ma_start(0U, vin);                                                        /* Auto mode, no automatic start: the unit stays stopped */
    SlaveCtrl_ClearLatch();                                                   /* a latched fault of the previous scenario is not part of this one */
    W.plant_curve = 0; W.plant_gain = (double)Tune_RefG(vin) * gain_k;           /* gain_k = 1: the bench transformer at THIS input; negative: connected backwards; 0: no transformer */
    ma.settings.maxcorr_v = maxc_v;
    memset(&tn, 0, sizeof tn); tn_prev_eff = 0.0;
    ma_run(3000U);
}
static void tn_begin(double gain_k, uint16_t maxc_v) { tn_begin_at(gain_k, maxc_v, 230.0f); }
static void t_mc_tune(void)
{
    char fr[MUI_ROWS][MUI_COLS + 1U]; int got;
    /* ---- a whole sweep: Max Correction 120 V, the bench transformer, at 160 V in (the headroom under the output limit is then 110 V: room for the whole 2752-tick range) ---- */
    tn_begin_at(1.0, 120U, 160.0f); CHECK(g_ctrl.state == CTRL_OFF && !ma_c_contactor(), "the unit is stopped before the sweep (state %d)", (int)g_ctrl.state);
    ma.ui.req |= MUI_REQ_RETUNE; tn_run(2000U);
    CHECK(ma.tune_state != 0U && SlaveTuneLink_Active() && SlaveCtrl_TuneRunning(), "Re-tune: the master asks, the slave accepts and the controller is in tune mode");
    got = tn_until_result(120000U); MApp_TuneScreen(&ma, mc_ms(), fr);
    CHECK(got && strstr(fr[0], "Tuning Complete") != NULL, "the master shows 'Tuning Complete' ('%s' / '%s')", fr[0], fr[1]);
    CHECK(tn.saw_run && !tn.master_run, "the controller started ITSELF through the start sequence and reached RUN, while the master never asked it to run (run = 0 in every $M)");
    CHECK(!tn.contactor, "the master's output contactor stayed open the whole time");
    CHECK(SlaveProfile_Tuned() == 1U, "the profile is stored in the slave");
    { TuneProfile_t pf; CHECK(SlaveProfile_Get(&pf) == 1U && pf.cap_ticks == 2752U && pf.kib_m >= 800U && pf.kib_m <= 1250U, "the stored profile: cap %u ticks, gain scale %u (the plant IS the reference curve times 1.0)", (unsigned)pf.cap_ticks, (unsigned)pf.kib_m); }
    CHECK(tn.max_eff <= 2752U && tn.max_eff >= 2500U, "the largest effective duty applied was %u ticks (the cap is 2752; the last step asks for 2752)", (unsigned)tn.max_eff);
    CHECK(tn.max_inj <= 0.8 * 110.0 + 6.0, "the injection never went above 80 %% of the smaller of Max Correction and the headroom, 110 V (+ 6 V of overshoot allowed): %.1f V of 88", tn.max_inj);
    CHECK(tn.max_wrong < 3.0, "and never the wrong way (%.2f V)", tn.max_wrong);
    CHECK(tn.max_slew <= (double)CTRL_TUNE_SLEW_TPS * 1.2 + 50.0, "the duty never rose faster than the slew limit: %.0f ticks/s (limit %.0f)", tn.max_slew, (double)CTRL_TUNE_SLEW_TPS);
    tn_run(15000U);
    CHECK(g_ctrl.state == CTRL_OFF && g_ctrl.duty_counts == 0U && g_ctrl.fault == 0U && !SlaveCtrl_TuneRunning() && fabs((double)(W.vout - W.vin)) < 1.5, "afterwards the controller is OFF with zero duty, no fault, the output is back at the input (state %d, fault %d, %.2f V)", (int)g_ctrl.state, (int)g_ctrl.fault, (double)(W.vout - W.vin));
    /* the unit works normally afterwards: START regulates 215 -> 230 V and the contactor closes */
    W.vin = 215.0f; W.vout = 215.0f; ma_run(1500U); ma_tap(1, 100U); (void)ma_until(ma_c_contactor, 90000U);
    CHECK(ma_c_contactor() && !SlaveCtrl_TuneRunning() && fabs((double)W.vout - 230.0) < 3.0, "START after a tune: the regulator takes over, 215 V in gives %.1f V out, the contactor is closed", (double)W.vout);

    /* ---- STOP on the master in the middle of the sweep ---- */
    { uint32_t k;
      tn_begin(1.0, 120U); ma.ui.req |= MUI_REQ_RETUNE; for (k = 0U; k < 6000U && tn.max_eff < 600U; k++) tn_run(10U);
      CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.duty_counts > 0U, "a sweep is under way: %u ticks applied", (unsigned)tn.max_eff);
      ma_k[2] = 1U; tn_run(100U); ma_k[2] = 0U; tn_run(500U);
      CHECK(g_ctrl.duty_counts == 0U && ma.tune_state == 3U && ma.tune_result == 6U, "STOP on the master: the duty is ZERO within 0.6 s and the master shows the result 'stopped' (duty %u, result %u)", (unsigned)g_ctrl.duty_counts, (unsigned)ma.tune_result);
      tn_run(15000U); CHECK(g_ctrl.state == CTRL_OFF && g_ctrl.fault == 0U && SlaveProfile_Tuned() == 0U && !SlaveCtrl_TuneRunning(), "then the controller is OFF without a fault and nothing is stored"); }
    /* ---- the transformer connected backwards ---- */
    tn_begin(-1.0, 120U); ma.ui.req |= MUI_REQ_RETUNE; got = tn_until_result(120000U);
    CHECK(got && ma.tune_result == 2U && SlaveProfile_Tuned() == 0U, "a reversed transformer: the master is told FLIPPED (result %u), nothing is stored", (unsigned)ma.tune_result);
    CHECK(tn.max_wrong < 6.0, "the output went at most %.1f V the wrong way before the sweep stopped", tn.max_wrong);
    tn_run(15000U); CHECK(g_ctrl.state == CTRL_OFF && g_ctrl.fault == 0U && g_ctrl.duty_counts == 0U, "and the controller is OFF, without a fault, at zero duty");
    /* ---- Max Correction 30 V: a short sweep, a low injection ---- */
    tn_begin(1.0, 30U); ma.ui.req |= MUI_REQ_RETUNE; got = tn_until_result(120000U);
    { TuneProfile_t pf; CHECK(got && ma.tune_result == 1U && SlaveProfile_Get(&pf) == 1U && pf.cap_ticks < 1000U && pf.maxc_dv == 300U, "Max Correction 30 V: FINISHED, a profile with the cap %u (< 1000) made at 30 V", (unsigned)pf.cap_ticks); }
    CHECK(tn.max_inj <= 0.8 * 30.0 + 4.0 && g_ctrl.fault == 0U, "the injection stayed at or below 24 V (+ 4 V): %.1f V, no fault", tn.max_inj);
    /* ---- the master goes away in the middle of a sweep: the controller stops itself ---- */
    { uint32_t k;
      tn_begin(1.0, 120U); ma.ui.req |= MUI_REQ_RETUNE; for (k = 0U; k < 6000U && tn.max_eff < 600U; k++) tn_run(10U);
      ma_mute_m = 1; tn_run(1500U);
      CHECK(!SlaveCtrl_TuneRunning() && g_ctrl.duty_counts == 0U, "no $M for 1.5 s: tune mode is off in the controller and the duty is zero");
      if (ma.tune_state != 3U) (void)tn_until_result(4000U);
      CHECK(ma.tune_state == 3U && ma.tune_result == 4U, "the master is told the sweep was aborted (result %u)", (unsigned)ma.tune_result);
      tn_run(8000U); CHECK(g_ctrl.state == CTRL_OFF && g_ctrl.fault == 0U && SlaveProfile_Tuned() == 0U, "the controller ends OFF without a fault and nothing is stored (state %d, fault %d, tuned %u)", (int)g_ctrl.state, (int)g_ctrl.fault, (unsigned)SlaveProfile_Tuned());
      ma_mute_m = 0; }
    /* ---- the controller's OWN limit, whatever the sweep asks: asked for the full 2752 ticks with Max Correction 30 V ---- */
    { uint32_t k;
      tn_begin(1.0, 30U);
      CHECK(SlaveCtrl_TuneBegin() == 1U, "the controller accepts tune mode when it is OFF, nothing is latched, the master is there and Max Correction is set");
      for (k = 0U; k < 3000U && !SlaveCtrl_TuneReady(); k++) tn_run(10U);
      CHECK(SlaveCtrl_TuneReady(), "it started itself and is in RUN");
      SlaveCtrl_TuneRequest(2752U); tn_run(5000U);
      CHECK((g_ctrl.state == CTRL_FAULT || g_ctrl.state == CTRL_LATCHED || g_ctrl.state == CTRL_RESHUNT) && g_ctrl.fault == CF_VOUT_HIGH, "asked for the full duty at 30 V of Max Correction: the controller stops at once with the over-voltage fault (state %d, fault %d)", (int)g_ctrl.state, (int)g_ctrl.fault);
      CHECK(tn.max_inj <= 30.0 + 8.0 + 4.0 && g_ctrl.duty_counts == 0U, "the output rose by at most %.1f V (the limit is 30 + 8) and the duty is zero", tn.max_inj);
      CHECK(SlaveCtrl_TuneBegin() == 0U, "while a fault is latched, tune mode is refused");
      CHECK(!SlaveCtrl_TuneRunning(), "the fault ENDED tune mode in the controller");
      SlaveCtrl_ClearLatch(); tn_run(6000U);
      CHECK(g_ctrl.state == CTRL_OFF && !SlaveCtrl_TuneRunning() && g_ctrl.duty_counts == 0U, "after the fault is cleared the unit stays OFF: it does NOT start itself (state %d)", (int)g_ctrl.state); }
    /* ---- the hardware cap: whatever duty is asked for, the controller never applies more than 2752 effective ticks ---- */
    { uint32_t k;
      tn_begin_at(1.0, 120U, 160.0f);
      CHECK(SlaveCtrl_TuneBegin() == 1U, "tune mode accepted"); for (k = 0U; k < 3000U && !SlaveCtrl_TuneReady(); k++) tn_run(10U);
      SlaveCtrl_TuneRequest(60000U); tn_run(5000U);
      CHECK(tn.max_eff == 2752U && g_ctrl.fault == 0U, "a request of 60000 ticks: the duty is capped at %u (2752), no fault", (unsigned)tn.max_eff);
      SlaveCtrl_TuneRequest(0U); tn_run(60U); CHECK(g_ctrl.duty_counts == 0U, "a request of 0 in the middle of the run: the duty falls AT ONCE (within 60 ms), it does not ramp down (duty %u)", (unsigned)g_ctrl.duty_counts);
      SlaveCtrl_TuneRequest(1200U); tn_run(1500U); SlaveCtrl_TuneEnd(); tn_run(200U); CHECK(g_ctrl.duty_counts == 0U, "SlaveCtrl_TuneEnd: the duty is zero within 0.2 s");
      tn_run(8000U); CHECK(g_ctrl.state == CTRL_OFF && g_ctrl.fault == 0U, "and the controller ends OFF without a fault"); }
    /* ---- a reset of the controller (a power cycle of the slave) ends tune mode ---- */
    tn_begin(1.0, 30U); CHECK(SlaveCtrl_TuneBegin() == 1U && SlaveCtrl_TuneRunning(), "tune mode is on"); SlaveCtrl_Init(); CHECK(!SlaveCtrl_TuneRunning(), "SlaveCtrl_Init (a slave reset) ends it: the unit does not come up in tune mode");
    /* ---- the refusals ---- */
    tn_begin(1.0, 0U); CHECK(SlaveCtrl_TuneBegin() == 0U, "Max Correction not set: the controller refuses");
    ma.ui.req |= MUI_REQ_RETUNE; tn_run(800U); CHECK(ma.tune_state == 0U && strstr(ma_f[2], "Set Max Correction") != NULL, "and the master says 'Set Max Correction' ('%s' / '%s'; engine run %u, slave state %u, unit state %d)", ma_f[1], ma_f[2], (unsigned)ma.eout.run[0], (unsigned)md.state, (int)g_ctrl.state);
    printf("  tune mode on the real controller: a whole sweep (the controller starts itself, the duty slew-limited and capped, the profile stored), STOP, a reversed transformer, Max Correction 30 V, the master lost, the controller's own limit, the refusals\n");
}
#if (SLAVE_TUNE_APPLY != 0)
/* ---- THE PROFILE IN USE: the cap and gain scale, the same-transformer check, refusing to run untuned, Re-tune out of "Re-tune Required" ---- */
static int tn_wait_contactor(uint32_t max_ms) { (void)ma_until(ma_c_contactor, max_ms); return ma_c_contactor() ? 1 : 0; }
static void tn_start_at(float vin) { W.vin = vin; W.vout = vin; ma_run(2000U); tn_prev_eff = 0.0; ma_tap(1, 100U); }
static void tn_stop_unit(void) { uint32_t k; ma_tap(2, 100U); for (k = 0U; k < 3000U && g_ctrl.state != CTRL_OFF; k++) tn_run(10U); ma_run(500U); }
static int tn_master_retune_latched(void) { return (ma.eng.u[0].st == (uint8_t)U_LATCHED) && (ma.eng.u[0].fclass == (uint8_t)FC_RETUNE); }
static void tn_do_retune(double gain_k, float vin, uint16_t maxc)
{   /* change the world to a tuning input and the given transformer, run the sweep from the master, end with the unit stopped */
    W.vin = vin; W.vout = vin; W.plant_gain = (double)Tune_RefG(vin) * gain_k; ma.settings.maxcorr_v = maxc; ma_run(1500U); ma.ui.req |= MUI_REQ_RETUNE; (void)tn_until_result(120000U); tn_run(15000U);
}
static void t_mc_tune_apply(void)
{
    int got; TuneProfile_t pf; float gb, gu;
    /* ---- ENFORCE: no profile, no run ---- */
    tn_begin_at(1.0, 120U, 215.0f);
    tn_start_at(215.0f); tn_run(8000U);
    CHECK(g_ctrl.fault == CF_NOT_TUNED && g_ctrl.state == CTRL_LATCHED && !ma_c_contactor() && g_ctrl.duty_counts == 0U, "a START without a profile: the slave latches fault 18 and does not run (state %d, fault %d)", (int)g_ctrl.state, (int)g_ctrl.fault);
    CHECK(tn_master_retune_latched(), "the master latches 'Re-tune Required' (class RETUNE)");
    CHECK(tn.max_inj < 2.0, "nothing was injected: %.2f V", tn.max_inj);
    /* ---- Re-tune works out of that state: the latch does not block its own cure ---- */
    W.vin = 160.0f; W.vout = 160.0f; ma_run(1500U); ma.ui.req |= MUI_REQ_RETUNE; tn_run(1500U);
    CHECK(ma.tune_state != 0U && SlaveCtrl_TuneRunning(), "Re-tune is accepted although fault 18 is latched: the master asks, the controller clears the latch and starts");
    got = tn_until_result(120000U); MApp_TuneScreen(&ma, mc_ms(), (char (*)[MUI_COLS + 1U])ma_f);
    CHECK(got && ma.tune_result == 1U && SlaveProfile_Tuned() == 1U, "the sweep finishes and the profile is stored (result %u)", (unsigned)ma.tune_result);
    tn_run(15000U);
    CHECK(!tn_master_retune_latched() && g_ctrl.state == CTRL_OFF && g_ctrl.fault == 0U, "the master's 'Re-tune Required' is cleared by the successful tune, the slave is OFF without a fault");
    /* ---- the profile in use: START regulates, the cap and the gain scale are the profile's ---- */
    (void)SlaveProfile_Get(&pf);
    tn_start_at(215.0f); got = tn_wait_contactor(90000U); tn_run(6000U);
    CHECK(got && g_ctrl.fault == 0U && fabs((double)W.vout - 230.0) < 3.0, "START with the profile: the regulator takes over, 215 V in gives %.1f V out, no fault, the contactor is closed", (double)W.vout);
    Reg_GetGainScale(&gb, &gu); CHECK(fabsf(gb - (float)pf.kib_m * 0.001f) < 0.002f && fabsf(gu - (float)pf.kiu_m * 0.001f) < 0.002f, "the regulator's gain scale is the profile's (boost %.3f, buck %.3f)", (double)gb, (double)gu);
    /* ---- no false alarm: the same transformer at several inputs (boost starts) ---- */
    { static const float VIN[4] = { 185.0f, 195.0f, 208.0f, 222.0f }; int i, bad_runs = 0;
      for (i = 0; i < 4; i++) { tn_stop_unit(); tn_start_at(VIN[i]); got = tn_wait_contactor(90000U); tn_run(4000U); if (!(got && g_ctrl.fault == 0U && fabs((double)W.vout - 230.0) < 3.5)) { bad_runs++; printf("      (input %.0f V: contactor %d, fault %d, out %.1f V)\n", (double)VIN[i], got, (int)g_ctrl.fault, (double)W.vout); } }
      CHECK(bad_runs == 0, "the SAME transformer started at 185, 195, 208 and 222 V (the master's input window starts at 180 V): no false 'different transformer' alarm, regulates to 230 V every time"); }
    /* ---- a DIFFERENT transformer: the check stops the start, latches, and the master says Re-tune Required ---- */
    tn_stop_unit(); W.plant_gain = (double)Tune_RefG(190.0f) * 1.6; memset(&tn, 0, sizeof tn); tn_prev_eff = 0.0;
    tn_start_at(190.0f); tn_run(12000U);
    CHECK(g_ctrl.fault == CF_RETUNE_MISMATCH && g_ctrl.state == CTRL_LATCHED && !ma_c_contactor(), "a transformer with 1.6 times the gain: fault 19 is latched and the unit does not run (state %d, fault %d)", (int)g_ctrl.state, (int)g_ctrl.fault);
    CHECK(tn_master_retune_latched(), "the master latches 'Re-tune Required'");
    CHECK(tn.max_inj < 55.0, "the output was never taken far above the input before the check stopped it: %.1f V of injection", tn.max_inj);
    /* ---- the installer accepts the new transformer: Re-tune, then it runs ---- */
    tn_do_retune(1.6, 160.0f, 120U);
    (void)SlaveProfile_Get(&pf); Reg_GetGainScale(&gb, &gu);
    CHECK(SlaveProfile_Tuned() == 1U && !tn_master_retune_latched() && g_ctrl.fault == 0U && pf.kib_m >= 540U && pf.kib_m <= 720U, "Re-tune with the new transformer: finished, the latch is cleared, the new gain scale is %u (about 1/1.6)", (unsigned)pf.kib_m);
    tn_start_at(190.0f); got = tn_wait_contactor(90000U); tn_run(5000U);
    CHECK(got && g_ctrl.fault == 0U && fabs((double)W.vout - 230.0) < 3.0, "and the unit now runs on the new transformer: %.1f V out, no fault", (double)W.vout);
    /* ---- the cap of a profile made with a low Max Correction limits the regulator ---- */
    tn_stop_unit(); tn_do_retune(1.0, 230.0f, 30U); (void)SlaveProfile_Get(&pf);
    CHECK(SlaveProfile_Tuned() == 1U && pf.cap_ticks < 1200U, "a profile made at 230 V with Max Correction 30 V has the cap %u (< 1200)", (unsigned)pf.cap_ticks);
    memset(&tn, 0, sizeof tn); tn_prev_eff = 0.0; W.plant_gain = (double)Tune_RefG(190.0f) * 1.0; tn_start_at(190.0f); tn_run(25000U);
    CHECK(tn.max_eff <= (uint32_t)pf.cap_ticks && g_ctrl.fault == 0U && W.vout < 228.0, "a start from 190 V needs about 1300 ticks but the regulator is capped at the profile's %u: it applied at most %u, the output stays at %.1f V, no fault", (unsigned)pf.cap_ticks, (unsigned)tn.max_eff, (double)W.vout);
    /* ---- a profile limited only by the OUTPUT HEADROOM (tuned at a high input) is not a limit of the transformer: its cap is NOT applied ---- */
    tn_stop_unit(); tn_do_retune(1.0, 230.0f, 120U); (void)SlaveProfile_Get(&pf);
    CHECK(SlaveProfile_Tuned() == 1U && pf.cap_ticks < 1200U && pf.maxc_dv < 1200U && pf.knee == 0U, "a profile made at 230 V with Max Correction 120 V is limited by the headroom: cap %u, made with %u (tenths of a volt), no knee", (unsigned)pf.cap_ticks, (unsigned)pf.maxc_dv);
    memset(&tn, 0, sizeof tn); tn_prev_eff = 0.0; W.plant_gain = (double)Tune_RefG(190.0f) * 1.0; tn_start_at(190.0f); got = tn_wait_contactor(90000U); tn_run(6000U);
    CHECK(got && g_ctrl.fault == 0U && fabs((double)W.vout - 230.0) < 3.0 && tn.max_eff > (uint32_t)pf.cap_ticks, "a start from 190 V needs about 1300 ticks: the regulator is NOT held to that profile's %u: it applied %u, the output is %.1f V, no fault", (unsigned)pf.cap_ticks, (unsigned)tn.max_eff, (double)W.vout);
    printf("  the profile in use on the real controller: refusing to run untuned, Re-tune out of the latch, the cap and gain scale, no false alarm at four inputs, a different transformer stopped, Re-tune accepts it\n");
}
#endif
#endif

static void t_mc_iout(void)
{
    ma_start(1U, 225.0f); (void)ma_until(ma_c_contactor, 90000U); ma_run(3000U);
    /* a slave without the I-OUT channel: the old 6-field $E, the page says "Not measured" */
    e2e_iout_on = 0U; { int q; ma_tap(3, 100U); for (q = 0; q < 7; q++) ma_tap(0, 100U); } ma_run(500U); MApp_IoutPage(&ma, 0U, ma_f); CHECK(MUI_ShowsIout(&ma.ui) != 0U, "LOG, MENU x7: the Converter Current page");
    CHECK(strstr(ma_f[3], "Not measured") != NULL && strstr(ma_f[1], "--.--") != NULL, "a slave without the channel: 'Not measured' ('%s' / '%s')", ma_f[3], ma_f[1]);
    { int i, six = 0, eight = 0; LinkV2Frame_t f; char c; for (i = 0; i < e2e_cnt; i++) if (LinkV2_Parse(e2e_f[i], (uint32_t)e2e_n[i], &f) == LINK_OK && f.type == LINKV2_E) { int j, commas = 0; for (j = 0; (c = e2e_f[i][j]) != '\0' && j < e2e_n[i]; j++) if (c == ',') commas++; if (commas == 6) six++; if (commas == 8) eight++; }
      CHECK(eight == 0, "and it sends the 6-field frame of the first protocol, never the 8-field one (%d six, %d eight)", six, eight); }
    /* a slave that measures it: the values arrive through the link and are shown */
    e2e_iout_on = 1U; e2e_iout_rms = 589U; e2e_iout_pk = 831U; e2e_iout_clip = 0U; ma_run(1500U); MApp_IoutPage(&ma, 0U, ma_f);
    CHECK(strncmp(ma_f[1], "RMS    5.89 A", 13) == 0 && strncmp(ma_f[2], "Peak   8.31 A", 13) == 0 && strstr(ma_f[3], "CT primary current") != NULL, "a slave that measures 5.89 A / 8.31 A: shown through the real link ('%s' / '%s' / '%s')", ma_f[1], ma_f[2], ma_f[3]);
    e2e_iout_rms = 0U; e2e_iout_pk = 0U; ma_run(1500U); MApp_IoutPage(&ma, 0U, ma_f); CHECK(strncmp(ma_f[1], "RMS    0.00 A", 13) == 0 && strstr(ma_f[3], "Not measured") == NULL, "a measured zero (no load) is 0.00 A, not 'Not measured' (the VALID flag says it is measured) ('%s' / '%s')", ma_f[1], ma_f[3]);
    e2e_iout_rms = 700U; e2e_iout_pk = 1011U; e2e_iout_clip = 1U; ma_run(1500U); MApp_IoutPage(&ma, 0U, ma_f); CHECK(strstr(ma_f[3], "OVER RANGE") != NULL, "the clip flag reaches the page ('%s')", ma_f[3]);
    e2e_iout_on = 0U; e2e_iout_clip = 0U;
    printf("  converter current end to end: the optional $E fields through the real link library to the master page; a slave without the channel keeps sending the old 6-field frame\n");
}

#if (SLAVE_VOLTAGE_CAL != 0)
/* ---- voltage calibration end to end: the real master application -> $K frames -> the real slave link and calibration logic -> $R frames -> the master's screen ---- */
static void ma_pages(int menus) { int q; ma_tap(3, 100U); for (q = 0; q < menus; q++) ma_tap(0, 100U); }       /* LOG, then MENU x menus */
static int ma_msg_has(int row, const char *s) { return strstr(ma_f[row], s) != NULL; }
static void t_mc_vcal(void)
{
    uint32_t t;
    ma_start(1U, 225.0f); (void)ma_until(ma_c_contactor, 90000U); ma_run(3000U);
    CHECK(ma_c_contactor() && g_ctrl.state == CTRL_RUN, "the unit is running with the contactor closed");
    ma_pages(5); ma_run(200U); CHECK(MUI_ShowsVCal(&ma.ui) == 1U, "LOG, MENU x5: the I/P Calibration page");
    /* running: refused by the master, nothing reaches the slave */
    ma.ui.cal_val = 2302U; ma_tap(3, 100U); ma_run(200U);
    CHECK(ma_msg_has(1, "Stop The Unit") && e2e_cal_saves == 0 && e2e_cal_applied[0] == 10000U, "running: 'Stop The Unit First' on the master, nothing stored (%s)", ma_f[1]);
    /* STOP on the page stops the unit */
    ma_run(1700U); ma_tap(2, 100U); t = ma_until(ma_c_latched, 100U); (void)t; { uint32_t g = 0U; while (g_ctrl.state != CTRL_OFF && g < 8000U) { ma_run(10U); g += 10U; } }
    CHECK(g_ctrl.state == CTRL_OFF && !ma_c_contactor() && MUI_ShowsVCal(&ma.ui) == 1U, "STOP on the page stops the unit (slave state %d) and the page stays", g_ctrl.state);
    W.vin = 226.8f; W.vout = 226.8f; ma_run(500U);
    /* the input: the meter says 230.2 V where the nominal chain reads 226.8 V */
    ma.ui.cal_val = 2302U; ma_tap(3, 100U);
    { uint32_t g = 0U; while ((e2e_cal_saves == 0) && g < 4000U) { ma_run(10U); g += 10U; } CHECK(e2e_cal_saves == 1 && g >= 900U && g <= 2000U, "the slave averaged for about a second and stored once (%u ms, %d saves)", (unsigned)g, e2e_cal_saves); }
    ma_run(300U);
    CHECK(e2e_cal_applied[0] == 10150U && e2e_cal_applied[1] == 10000U && e2e_cal_stored[0] == 10150U, "the input ratio is 1.0150, the output untouched (%u %u)", e2e_cal_applied[0], e2e_cal_applied[1]);
    CHECK(ma_msg_has(1, "Calibrated") && ma_msg_has(2, "I/P +1.50 %"), "the master shows 'Calibrated' and 'I/P +1.50 %%' ('%s' / '%s')", ma_f[1], ma_f[2]);
    /* the output page */
    ma_run(2000U); ma_tap(0, 100U); ma_run(200U); CHECK(MUI_ShowsVCal(&ma.ui) == 2U, "MENU: the O/P Calibration page");
    ma.ui.cal_val = 2250U; ma_tap(3, 100U); ma_run(1800U);
    CHECK(e2e_cal_applied[1] == 9921U && e2e_cal_applied[0] == 10150U && ma_msg_has(2, "O/P -0.79 %"), "the output: 225.0 V against 226.8 V is -0.79 %%, the input kept (%u, '%s')", e2e_cal_applied[1], ma_f[2]);
    /* refused by the slave: out of range, nothing stored */
    ma_run(2000U); { int s0 = e2e_cal_saves; ma.ui.cal_val = 2600U; ma_tap(3, 100U); ma_run(1800U); CHECK(ma_msg_has(1, "Out Of Range") && e2e_cal_saves == s0 && e2e_cal_applied[1] == 9921U, "260.0 V against 226.8 V: the slave answers OUT_OF_RANGE, the master says so, nothing changes ('%s')", ma_f[1]); }
    /* a flash failure: the old value stays */
    ma_run(2000U); e2e_cal_fail_save = 1; ma.ui.cal_val = 2250U; ma_tap(3, 100U); ma_run(1800U);
    CHECK(ma_msg_has(1, "Save Failed") && e2e_cal_applied[1] == 9921U, "a failed flash write: 'Save Failed', the old calibration stays in force ('%s')", ma_f[1]); e2e_cal_fail_save = 0;
    /* typing 000.0 resets a channel */
    ma_run(2000U); ma.ui.cal_val = 0U; ma_tap(3, 100U); ma_run(500U);
    CHECK(ma_msg_has(1, "Reset To Nominal") && e2e_cal_applied[1] == 10000U && e2e_cal_applied[0] == 10150U, "000.0 resets the OUTPUT only ('%s')", ma_f[1]);
    /* a power cycle of the slave: the stored calibration is applied again */
    { uint16_t keep = e2e_cal_stored[0]; e2e_cal_applied[0] = e2e_cal_applied[1] = 10000U; SlaveCal_Init(&E2E_CAL_HW); CHECK(e2e_cal_applied[0] == keep && e2e_cal_applied[1] == 10000U, "after a slave restart the stored calibration comes back (%u)", e2e_cal_applied[0]); }
    /* the unit can be started again afterwards */
    ma_run(2000U); ma_tap(0, 100U); ma_run(200U); CHECK(MUI_ShowsVCal(&ma.ui) == 0U, "MENU leaves the pages"); ma_tap(1, 100U); (void)ma_until(ma_c_contactor, 90000U);
    CHECK(ma_c_contactor(), "START on the Run screen runs the unit again");
    printf("  voltage calibration end to end: refused while running, STOP on the page, 1 s average, ratio and screen text, out of range, flash failure, reset, restart, run again\n");
}
#endif

#if (SLAVE_TF_RUN_CHECK != 0)
/* ---- the transformer-connection (FLIP) check at every start, with the real master ---- */
static void ma_flip_case(float vin, double gain, double offset, uint32_t *t_latch_ms, double *max_wrong_v, int *latched)
{
    uint32_t k; double wrong = 0.0;
    ma_start(1U, vin);
    if (gain != 0.0) { W.plant_curve = 0; W.plant_gain = gain; }                      /* a NEGATIVE gain: the transformer connected the wrong way round */
    W.out_offset = offset;
    *t_latch_ms = 0U; *latched = 0;
    for (k = 0U; k < 600U; k++)                                                        /* 60 s in 100 ms steps */
    {
        double dev = (vin < 230.0f) ? (double)(W.vin - W.vout) : (double)(W.vout - W.vin);      /* how far the output has moved AGAINST the direction it was told to go */
        if (dev > wrong && g_ctrl.state != CTRL_LATCHED) wrong = dev;
        ma_run(100U);
        if (g_ctrl.state == CTRL_LATCHED && *latched == 0) { *latched = 1; *t_latch_ms = (k + 1U) * 100U; }
    }
    *max_wrong_v = wrong;
}
static void t_mc_flip(void)
{
    uint32_t t; double wrong; int latched; MLogRec_t r;
    /* a correctly connected transformer: no fault, it regulates and the contactor closes */
    ma_flip_case(215.0f, 0.0, 0.0, &t, &wrong, &latched);
    CHECK(!latched && g_ctrl.fault != CF_TRANSFORMER_FLIP && ma_c_contactor(), "connected correctly (boost, 215 V in): no flip fault, the contactor is closed (latched %d, fault %d)", latched, g_ctrl.fault);
    ma_flip_case(245.0f, 0.0, 0.0, &t, &wrong, &latched);
    CHECK(!latched && ma_c_contactor(), "connected correctly (buck, 245 V in): no flip fault, the contactor is closed");
    /* BOOST needed, the transformer connected BACKWARDS: the slave stops with fault 17, latched, after a deviation of only a few volts */
    ma_flip_case(215.0f, -0.02, 0.0, &t, &wrong, &latched);
    CHECK(latched && g_ctrl.state == CTRL_LATCHED && g_ctrl.fault == CF_TRANSFORMER_FLIP, "backwards (boost): the slave latches fault 17 (state %d, fault %d, after %u ms)", g_ctrl.state, g_ctrl.fault, (unsigned)t);
    CHECK(wrong <= 3.5 && t <= 15000U, "the output moved against the command by only %.1f V before the stop, within %u ms of the run command", wrong, (unsigned)t);
    CHECK(!ma_c_contactor() && ma.eout.fault[0] == MLOG_T_FLIP && ma.eout.buzzer != 0U, "the contactor never closed; the master shows Transformer Flip and sounds the buzzer (fault %u, buzzer %u)", (unsigned)ma.eout.fault[0], (unsigned)ma.eout.buzzer);
    CHECK(MLog_Count(&ma.log) >= 1U && MLog_Get(&ma.log, 0U, &r) == 0 && r.type == MLOG_T_FLIP && r.slave_code == 17U, "logged as Transformer Flip with code 17");
    CHECK(MApp_Control(&ma)->run == 0U && g_ctrl.state == CTRL_LATCHED, "and it stays stopped for the rest of the minute: a wiring fault is never retried by itself");
    /* BUCK needed, backwards */
    ma_flip_case(245.0f, -0.02, 0.0, &t, &wrong, &latched);
    CHECK(latched && g_ctrl.fault == CF_TRANSFORMER_FLIP && wrong <= 3.5, "backwards (buck, 245 V in): fault 17 too, a wrong-way deviation of only %.1f V", wrong);
    /* a unit that needs no correction is not tested and not at risk */
    ma_flip_case(230.0f, -0.02, 0.0, &t, &wrong, &latched);
    CHECK(!latched && g_ctrl.fault == CF_NONE, "230 V in at a Set Voltage of 230 V, even with a backwards transformer: nothing to correct, no verdict, no fault");
    /* the two voltage sensors differ by 0.8 V: a correct transformer is not blamed, a backwards one is still found */
    ma_flip_case(215.0f, 0.0, 0.8, &t, &wrong, &latched);
    CHECK(!latched && ma_c_contactor(), "a 0.8 V offset between the sensors: a correct transformer is NOT called backwards");
    ma_flip_case(215.0f, -0.02, 0.8, &t, &wrong, &latched);
    CHECK(latched && g_ctrl.fault == CF_TRANSFORMER_FLIP, "a 0.8 V sensor offset does not hide a backwards transformer");
    /* an exaggerated sensor offset (+-5 V): the baseline is the ONLY thing that keeps a correct transformer from looking backwards (-5 V) and a backwards one from looking correct (+5 V) */
    ma_flip_case(215.0f, 0.0, -5.0, &t, &wrong, &latched);
    CHECK(!latched && g_ctrl.fault == CF_NONE, "a -5 V sensor offset: a correct transformer is NOT called backwards");
    ma_flip_case(215.0f, -0.02, 5.0, &t, &wrong, &latched);
    CHECK(latched && g_ctrl.fault == CF_TRANSFORMER_FLIP, "a +5 V sensor offset does not hide a backwards transformer");
    /* only about 1 V of correction is needed and the output sensor has +-2 V of noise: the true effect is small and the noise is large, so neither a lone window nor a single poor reading may decide. 60 s, correct transformer: no verdict, no fault. */
    { uint32_t k, ns = 777U; int lat = 0;
      ma_start(1U, 229.0f);
      for (k = 0U; k < 3000U; k++) { ns = ns * 1664525U + 1013904223U; W.out_offset = ((double)((ns >> 12) % 2001U) / 1000.0 - 1.0) * 2.0; ma_run(20U); if (g_ctrl.state == CTRL_LATCHED) lat = 1; }
      CHECK(!lat && g_ctrl.fault != CF_TRANSFORMER_FLIP, "needing ~1 V of correction with +-2 V of sensor noise for a minute: no false flip (latched %d, fault %d)", lat, g_ctrl.fault); }
    /* big sensor offsets of EITHER sign: the recorded baseline must cancel them (a correct transformer with -2 V is not called backwards, a backwards one with +2 V is not hidden) */
    ma_flip_case(215.0f, 0.0, -2.0, &t, &wrong, &latched);
    CHECK(!latched && g_ctrl.fault == CF_NONE, "a -2.0 V offset between the sensors: a correct transformer is NOT called backwards");
    ma_flip_case(215.0f, -0.02, 2.0, &t, &wrong, &latched);
    CHECK(latched && g_ctrl.fault == CF_TRANSFORMER_FLIP, "a +2.0 V offset between the sensors does not hide a backwards transformer");
    /* measurement noise on the output sensor (+-0.6 V, a new value every 20 ms): never a false flip over a minute, and a backwards transformer is still found */
    { uint32_t k, ns = 4242U; int lat = 0;
      ma_start(1U, 215.0f);
      for (k = 0U; k < 3000U; k++) { ns = ns * 1664525U + 1013904223U; W.out_offset = ((double)((ns >> 12) % 2001U) / 1000.0 - 1.0) * 0.6; ma_run(20U); if (g_ctrl.state == CTRL_LATCHED) lat = 1; }
      CHECK(!lat && g_ctrl.fault != CF_TRANSFORMER_FLIP && ma_c_contactor(), "+-0.6 V of noise on the output sensor for a minute: a correct transformer is never called backwards");
      ma_start(1U, 215.0f); W.plant_curve = 0; W.plant_gain = -0.02; lat = 0;
      for (k = 0U; k < 3000U; k++) { ns = ns * 1664525U + 1013904223U; W.out_offset = ((double)((ns >> 12) % 2001U) / 1000.0 - 1.0) * 0.6; ma_run(20U); if (g_ctrl.state == CTRL_LATCHED && g_ctrl.fault == CF_TRANSFORMER_FLIP) lat = 1; }
      CHECK(lat, "and with the same noise a backwards transformer is still found"); }
    printf("  transformer flip check: correct boost/buck pass, backwards boost/buck latch fault 17 within a few volts, nothing to correct = no verdict, a sensor offset neither fools it nor hides it\n");
}
#endif

#if (SLAVE_GLITCH_SKIP != 0)
/* ---- the master with the REAL slave: the glitch rate observed on the bench (6 in 10 minutes, F16) must not open the load ---- */
static void t_mc_glitch_master(void)
{
    uint32_t k, s0; MLogRec_t r;
    ma_start(1U, 228.0f); (void)ma_until(ma_c_contactor, 90000U); ma_run(8000U);
    CHECK(ma_c_contactor() && g_ctrl.state == CTRL_RUN, "running with the contactor closed");
    s0 = SlaveCtrl_GlitchSkips();
    for (k = 0U; k < 6U; k++) { W.preen_glitch = 1; ma_run(100000U); }                  /* 6 glitches in 10 simulated minutes */
    CHECK(ma_c_contactor() && g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE && MLog_Count(&ma.log) == 0U,
          "6 last-moment glitches in 10 minutes: the contactor stays closed, nothing is logged (contactor %d, state %d, fault %d, %u logged)", (int)ma_c_contactor(), g_ctrl.state, g_ctrl.fault, (unsigned)MLog_Count(&ma.log));
    CHECK(SlaveCtrl_GlitchSkips() - s0 == 6U, "all six were counted (%u)", (unsigned)(SlaveCtrl_GlitchSkips() - s0));
    /* a real problem: three in a row. The slave stops with F16, the master opens the contactor and logs it with the code. */
    W.preen_glitch = 3; ma_run(3000U);
    CHECK(!ma_c_contactor() && g_ctrl.fault == CF_PRE_ENERGISE, "three in a row: the slave stops with F16 and the contactor opens (contactor %d, fault %d)", (int)ma_c_contactor(), g_ctrl.fault);
    CHECK(MLog_Count(&ma.log) >= 1U && MLog_Get(&ma.log, 0U, &r) == 0 && r.type == MLOG_T_MAINS && r.slave_code == 16U, "and it is logged as Mains Unstable with code 16");
    printf("  glitch with the master: 6 rare F16 glitches in 10 minutes leave the load connected; three in a row open it and log F16\n");
}
#endif
#if (SLAVE_TEST_NO_VOLTAGE_CUTS != 0)
/* ---- TEST build: the slave applies NO voltage limits of its own ---- */
static void t_mc_no_voltage_cuts(void)
{
    uint32_t t;
    mc_world(240000000ULL, 130.0f);                                  /* below the 140 V input envelope: the standard slave refuses to start here */
    t = mc_until_run(2300U, 1U, 60000U);
    mc_for(10000U, 1U, 2300U, 1U, 0U);
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "130 V input (below the 140 V envelope): it starts and runs, no fault (state %d, fault %d, RUN after %u ms)", g_ctrl.state, g_ctrl.fault, (unsigned)t);
    mc_world(241000000ULL, 305.0f);                                  /* above the 300 V envelope */
    t = mc_until_run(3000U, 1U, 60000U);
    mc_for(10000U, 1U, 3000U, 1U, 0U);
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "305 V input (above the 300 V envelope): it starts and runs, no fault (state %d, fault %d)", g_ctrl.state, g_ctrl.fault);
    mc_world(242000000ULL, 270.0f);
    t = mc_until_run(2950U, 1U, 60000U);
    mc_for(15000U, 1U, 2950U, 1U, 0U);
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE && (double)W.vout > 285.0, "regulating to 295 V, above the old output guard: no output-high trip (%.1f V, state %d, fault %d)", (double)W.vout, g_ctrl.state, g_ctrl.fault);
    mc_for(30000U, 1U, 2950U, 1U, 0U);
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "and it keeps running for another 30 s");
    check_invariants("master control: no voltage cuts");
    printf("  TEST build, no voltage cuts: starts at 130 V and at 305 V, regulates to 295 V, no trip\n");
}
#endif
#endif /* TEST_MC */
