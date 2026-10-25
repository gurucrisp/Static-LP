/* The autotune END TO END: the real master application (Re-tune request, screens, STOP) talking through real link FRAMES to the real slave chain (link layer, tune link, tune core, profile store) with a simulated transformer. */
#define main test_master_app_main
#include "test_master_app.c"
#undef main
#include <math.h>
#include "slave_link2.h"
#include "slave_tune_link.h"
#include "slave_profile.h"

/* ---- the slave's flash for the profile (a RAM model that can refuse) ---- */
#define PA 0x1000UL
#define PB 0x1800UL
static uint8_t smem[4096]; static int sflash_fail;
static int s_rd(uint32_t a, uint8_t *b, uint32_t n) { memcpy(b, &smem[a - PA], n); return 0; }
static int s_er(uint32_t a) { if (sflash_fail) return -1; memset(&smem[a - PA], 0xFF, 2048U); return 0; }
static int s_pr(uint32_t a, const uint8_t *b, uint32_t n) { uint32_t i; if (sflash_fail) return -1; for (i = 0U; i < n; i++) smem[a - PA + i] &= b[i]; return 0; }
static const SFlash_t SFL = { s_rd, s_pr, s_er };
static uint8_t s_stopped(void) { return (uint8_t)(D.state == 0U); }
static uint16_t s_maxc(void) { return (uint16_t)(A.settings.maxcorr_v * 10U); }
static uint8_t s_can(void) { return 1U; } static uint8_t s_busy(void) { return 0U; }
/* a fake controller: about 6 s to reach RUN, then it applies the requested duty */
static uint8_t c_on; static uint16_t c_req; static uint32_t c_ready_at;
static uint8_t c_begin(void) { c_on = 1U; c_ready_at = T + 6000U; c_req = 0U; return 1U; }
static void c_request(uint16_t k) { c_req = k; }
static void c_end(void) { c_on = 0U; c_req = 0U; }
static uint8_t c_ready(void) { return (uint8_t)((c_on != 0U) && (T >= c_ready_at)); }
static uint8_t c_alive(void) { return c_on; }
static uint16_t c_fault(void) { return 0U; }
static uint16_t c_headroom(void) { return 3000U; }
static const TuneLinkHw_t SHW = { s_maxc, s_stopped, s_can, s_busy, c_begin, c_request, c_end, c_ready, c_alive, c_fault, c_headroom };
/* ---- the slave's link: frames to the master ---- */
static uint8_t s_tx(const char *f, uint32_t n)
{
    LinkV2Frame_t fr; if (LinkV2_Parse(f, n, &fr) != LINK_OK) return 1U;
    if (fr.type == LINKV2_H) { D.prof_tuned = fr.u.h.tuned; D.prof_cap = fr.u.h.prof_cap; D.prof_kib = fr.u.h.prof_kib; D.prof_kiu = fr.u.h.prof_kiu; D.prof_vref_dv = fr.u.h.prof_vref_dv; D.prof_maxc_dv = fr.u.h.prof_maxc_dv; D.prof_n = fr.u.h.prof_n; D.prof_knee = fr.u.h.prof_knee; }
    else if (fr.type == LINKV2_R) MApp_OnKResult(&A, fr.u.r.id, fr.u.r.status, fr.u.r.value);
    else if (fr.type == LINKV2_T) { D.tune_step = fr.u.t.step; D.tune_result = fr.u.t.result; D.tune_ticks = fr.u.t.ticks; D.tune_vin_dv = fr.u.t.vin_dv; D.tune_dv_dv = fr.u.t.dv; D.tune_n++; }
    return 1U;
}
static void s_on_control(const LinkM_t *m, uint32_t now) { (void)m; (void)now; }
static void s_fill_e(LinkE_t *e) { e->setv_dv = 2300U; e->flags2 = SlaveTuneLink_Flags2(); }
static void s_fill_h(LinkH_t *h)
{
    TuneProfile_t p;
    h->fw = 932UL; h->vin_min_dv = 800U; h->vin_max_dv = 3000U; h->tuned = SlaveProfile_Tuned(); h->vout_max_dv = 2800U;
    if (SlaveProfile_Get(&p) == 1U) { h->prof_cap = p.cap_ticks; h->prof_kib = p.kib_m; h->prof_kiu = p.kiu_m; h->prof_vref_dv = p.vin_ref_dv; h->prof_maxc_dv = p.maxc_dv; h->prof_n = p.n; h->prof_knee = p.knee; }     /* as the slave's glue does */
}
static int  s_tcmd(uint8_t c, const char *n, int32_t v, uint8_t *s, int32_t *val, uint32_t now) { return SlaveTuneLink_Command(c, n, v, s, val, now); }
static int  s_tfr(LinkT_t *f, uint32_t now) { return SlaveTuneLink_NextFrame(f, now); }
static const SL2_Hooks_t SHK = { s_on_control, s_fill_e, s_fill_h, s_tx, 0, 0, s_tcmd, s_tfr };
/* ---- the simulated transformer ---- */
static double pk, pdead, psign, pinj; static uint32_t prs = 7U; static double pg(void) { double q = 0.0; int i; for (i = 0; i < 3; i++) { prs = prs * 1664525U + 1013904223U; q += (double)((prs >> 8) & 0xFFFFFFU) / 8388608.0 - 1.0; } return q / 1.732; }
static uint32_t last_slave_ms;
/* run both ends: the master every 5 ms (and its commands to the slave as BYTES), the slave every 10 ms */
static void co_run(uint32_t ms)
{
    uint32_t e = T + ms;
    while (T < e)
    {
        MCalK_t kk; run(5U);
        while (MApp_TakeKCommand(&A, &kk, T) != 0U)
        {
            char b[LINKV2_MAX_LEN + 8]; LinkK_t k; int32_t n, i; memset(&k, 0, sizeof k); k.id = kk.id; k.cmd = (kk.cmd == MAPP_KCMD_TUNE) ? LINKCMD_TUNE : LINKCMD_TABORT;
            n = LinkV2_BuildK(b, sizeof b, &k); for (i = 0; i < n; i++) SL2_Rx((uint8_t)b[i], T);
        }
        if (MApp_HelloDue(&A, T) != 0U)
        {   /* the master asks the slave for its hello again (a tune has stored a profile): as the glue does, a $K HELLO frame */
            char b[LINKV2_MAX_LEN + 8]; LinkK_t k; int32_t n, i; memset(&k, 0, sizeof k); k.id = MApp_NextKId(&A); k.cmd = LINKCMD_HELLO;
            n = LinkV2_BuildK(b, sizeof b, &k); for (i = 0; i < n; i++) SL2_Rx((uint8_t)b[i], T);
        }
        if ((uint32_t)(T - last_slave_ms) >= 10U)
        {
            TuneIn_t in; double eff, target;
            last_slave_ms = T; eff = ((double)((c_ready() != 0U) ? c_req : 0U) > pdead) ? ((double)((c_ready() != 0U) ? c_req : 0U) - pdead) : 0.0;           /* the plant sees the duty the CONTROLLER applies */
            target = psign * pk * (double)Tune_RefG(230.0f) * eff; pinj += (target - pinj) * (1.0 - exp(-0.01 / 0.3));
            in.vin = (float)(230.0 + 0.05 * pg()); in.vout = (float)(230.2 + pinj + 0.1 * pg()); in.meas_valid = 1U; in.fault = 0U; in.vin_in_window = 1U; in.maxc_v = (float)A.settings.maxcorr_v; in.hw_cap_ticks = 2752U; in.iout_valid = 1U; in.iout_clip = 0U; in.iout_rms = 1.0f; in.iout_pk = 4.0f;
            SlaveTuneLink_Service(T, &in); SL2_Service(T);
        }
    }
}
/* run both ends until the master shows a RESULT, and take the screen at that moment (it goes away by itself after 6 s); returns the seconds it took, or -1 */
static int until_result(int max_s, char fr[MUI_ROWS][MUI_COLS + 1U])
{
    int s; uint32_t g;
    for (s = 0; s < max_s * 10; s++) { co_run(100U); if (A.tune_state == 3U) { MApp_TuneScreen(&A, T, fr); (void)g; return s / 10; } }
    MApp_TuneScreen(&A, T, fr); return -1;
}
static void e2e_fresh(void)
{
    fresh(); if (Tune_Running()) Tune_Abort();
    memset(smem, 0xFF, sizeof smem); sflash_fail = 0; pk = 1.0; pdead = 40.0; psign = 1.0; pinj = 0.0;
    c_on = 0U; c_req = 0U; SlaveProfile_Init(&SFL, PA, PB); SlaveTuneLink_Init(&SHW); SL2_Init(&SHK, T); last_slave_ms = T;
    to_run(); run(2000U); stop_unit();                                   /* the master unit is stopped (the slave reports state 0) */
    A.settings.maxcorr_v = 120U;
    D.e_valid = 1U; D.h_valid = 1U;                                      /* as in the real unit: the master has heard the slave's hello at power-up, so it does NOT ask again on its own */
}
int main(void)
{
    char fr[MUI_ROWS][MUI_COLS + 1U]; int q, ever_on;
    /* Max Correction not set: the slave refuses, the master says so */
    e2e_fresh(); A.settings.maxcorr_v = 0U; A.ui.req |= MUI_REQ_RETUNE; co_run(500U);
    CHECK(!MApp_ShowsTune(&A) && strstr(row(2), "Set Max Correction") != NULL && !Tune_Running(), "Max Correction 0: the slave answers 'not allowed, reason 2', the master shows 'Set Max Correction' ('%s' / '%s')", row(1), row(2));
    /* a whole sweep */
    e2e_fresh(); A.ui.req |= MUI_REQ_RETUNE; co_run(300U);
    CHECK(MApp_ShowsTune(&A) && SlaveTuneLink_Active() && c_on, "Re-tune: TUNE goes to the slave as a frame, the controller is told to start up, the master shows the tuning screen");
    ever_on = 0; co_run(7000U); MApp_TuneScreen(&A, T, fr); CHECK(MApp_ShowsTune(&A) && fr_has(fr, "Measuring baseline"), "after 7 s (the controller is still starting up) the master is still waiting with 'Measuring baseline', fed by the slave's heartbeats ('%s' / '%s')", fr[1], fr[2]);
    co_run(9000U); MApp_TuneScreen(&A, T, fr); CHECK(fr_has(fr, "Step  1 of 10") || fr_has(fr, "Step  2 of 10") || fr_has(fr, "Step  3 of 10"), "after 16 s the sweep is under way and the master shows the steps ('%s' / '%s')", fr[1], fr[2]);
    { int secs = -1; for (q = 0; q < 600 && secs < 0; q++) { co_run(100U); if (A.eout.run[0] != 0U || A.eout.contactor[0] != 0U) ever_on = 1; if (A.tune_state == 3U) { MApp_TuneScreen(&A, T, fr); secs = 1; } }
      CHECK(secs > 0 && fr_has(fr, "Tuning Complete"), "the sweep ends with 'Tuning Complete' on the master ('%s' / '%s')", fr[0], fr[1]); }
    CHECK(SlaveProfile_Tuned() == 1U && !Tune_Running(), "and the slave has stored the profile"); CHECK(!ever_on, "the master never started the unit during the sweep");
    { TuneProfile_t p; CHECK(SlaveProfile_Get(&p) == 1U && p.cap_ticks == 2752U && p.kib_m >= 940U && p.kib_m <= 1060U, "the stored profile: cap %u, gain scale %u", (unsigned)p.cap_ticks, (unsigned)p.kib_m); }
    co_run(7000U); CHECK(!MApp_ShowsTune(&A), "the result goes away by itself");
    /* the Transformer Profile page (LOG, then MENU x8) shows what the slave stored, as it came over the link in $H */
    CHECK(strstr(row(1), "Cap") == NULL, "(before: the master has no profile numbers yet)");
    co_run(1500U);                                                                          /* the master asked for the slave's hello when the tune finished: $H now carries the profile */
    tap(3, 100U); for (q = 0; q < 8; q++) tap(0, 100U); run(100U); MApp_ProfilePage(&A, T, fr);
    { TuneProfile_t pf; (void)SlaveProfile_Get(&pf);
      CHECK(MApp_ShowsProfile(&A) && strstr(fr[0], "Transformer Profile") != NULL && strncmp(fr[1], "Cap 2752 ticks", 14) == 0, "the page shows the stored cap ('%s')", fr[1]);
      CHECK(fr[2][6] == (char)('0' + ((pf.kib_m / 1000U) % 10U)) && fr[2][8] == (char)('0' + ((pf.kib_m / 100U) % 10U)) && fr[2][9] == (char)('0' + ((pf.kib_m / 10U) % 10U)) && fr[2][10] == (char)('0' + (pf.kib_m % 10U)), "the boost gain scale on the page is the stored %u thousandths ('%s')", (unsigned)pf.kib_m, fr[2]);
      CHECK(strncmp(fr[3], "At 230V Max 120V n10", 20) == 0, "the input it was made at, the Max Correction used and the 10 points: 'At 230V Max 120V n10' ('%s')", fr[3]); }
    /* STOP on the master aborts the slave's sweep */
    e2e_fresh(); A.ui.req |= MUI_REQ_RETUNE; co_run(17000U); CHECK(Tune_Running() && MApp_ShowsTune(&A), "a sweep is running");
    tap(2, 100U); co_run(1500U); MApp_TuneScreen(&A, T, fr);
    CHECK(!Tune_Running() && SlaveProfile_Tuned() == 0U && fr_has(fr, "Tuning Stopped"), "STOP: the master sends TABORT as a frame, the slave stops, nothing is stored, the master says 'Tuning Stopped' ('%s')", fr[0]);
    /* the transformer is the wrong way round */
    e2e_fresh(); psign = -1.0; A.ui.req |= MUI_REQ_RETUNE; q = until_result(50, fr);
    CHECK(q >= 0 && fr_has(fr, "Reversed Transformer") && SlaveProfile_Tuned() == 0U, "a reversed transformer: 'Reversed Transformer' on the master, nothing stored ('%s')", fr[1]);
    /* the slave's flash refuses the profile */
    e2e_fresh(); sflash_fail = 1; A.ui.req |= MUI_REQ_RETUNE; q = until_result(55, fr);
    CHECK(q >= 0 && fr_has(fr, "Aborted By Slave") && !fr_has(fr, "Tuning Complete") && SlaveProfile_Tuned() == 0U, "the flash refuses: the master is told it was aborted, never 'Tuning Complete' ('%s')", fr[1]);
    printf("  autotune end to end: the master application and the slave chain through real link frames: refusal, a whole sweep, STOP, a reversed transformer, a refusing flash\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
