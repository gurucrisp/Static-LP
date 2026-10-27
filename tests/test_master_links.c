/* The Y and B slave links (master_links.c): the decoders of the two links are independent, the data for the application (the PORT is the identity, the letter is ignored), the $M and hello timing, and a CLOSED LOOP: simulated Y and B slaves send real frames, the application answers with real $M frames. No hardware. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "master_links.h"
#include "m_app.h"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- frames a slave would send ---- */
static uint16_t g_frame_ntc = 3000U;      /* the NTC reading the next status frame carries */
static int s_frame(char *b, char ph, uint8_t seq, uint16_t vin, uint16_t vout, uint8_t state, uint16_t fault)
{
    LinkStatus_t st; memset(&st, 0, sizeof st); st.phase = ph; st.seq = seq; st.vin_dv = vin; st.vout_dv = vout; st.freq_chz = 5000U; st.state = state; st.fault = fault; st.ntc_raw = g_frame_ntc;      /* default 3000: a cool heat sink, about 42 degC (2000 would be 77 degC: above the 75 degC default, the units would trip on overheat) */
    return (int)Link_BuildStatus(b, 96U, &st);
}
static uint16_t g_e_sq_id, g_e_sq_delay, g_e_sq_period;      /* the phase sequence result the next $E frame carries (0 = none: a 6 field frame as before) */
static uint16_t g_e_iout_pk; static uint8_t g_e_iout_clip;                                 /* v0.9.55: the converter-current peak the next $E carries (0 = none) */
static int e_frame(char *b, uint8_t seq, uint16_t flags2, uint8_t clr)
{
    LinkE_t e; memset(&e, 0, sizeof e); e.seq = seq; e.setv_dv = 2300U; e.flags2 = flags2; e.clr = clr; e.sq_id = g_e_sq_id; e.sq_delay_us = g_e_sq_delay; e.sq_period_us = g_e_sq_period;
    if (g_e_iout_pk != 0U) { e.iout_pk_ca = g_e_iout_pk; e.iout_rms_ca = (uint16_t)(g_e_iout_pk / 2U); e.flags2 = (uint16_t)(e.flags2 | LINKV2_FLAG_IOUT_VALID | ((g_e_iout_clip != 0U) ? LINKV2_FLAG_IOUT_CLIP : 0U)); }
    return (int)LinkV2_BuildE(b, 96U, &e);
}
static int h_frame_p(char *b, uint16_t cap)
{
    LinkH_t h; memset(&h, 0, sizeof h); h.proto = 2U; h.fw = 948U; h.vin_min_dv = 1400U; h.vin_max_dv = 2500U; h.rst = LINKV2_RST_POWER_ON; h.tuned = 1U; h.vout_max_dv = 2400U;
    if (cap != 0U) { h.prof_cap = cap; h.prof_kib = 950U; h.prof_kiu = 1010U; h.prof_vref_dv = 2000U; h.prof_maxc_dv = 600U; h.prof_n = 10U; }
    return (int)LinkV2_BuildH(b, 96U, &h);
}
static int h_frame(char *b) { return h_frame_p(b, 0U); }
/* the receive rings, as the DMA fills them */
#define RING 256U
static volatile uint8_t ring[2][RING]; static uint32_t wr[2], rd[2];
static void push(uint8_t i, const char *b, int n) { int k; for (k = 0; k < n; k++) { ring[i][wr[i]] = (uint8_t)b[k]; wr[i] = (wr[i] + 1U) & (RING - 1U); } }
static void feed(uint8_t i, uint32_t now) { MLinks_Feed(i, ring[i], RING, &rd[i], wr[i], now); }
static void reset(void) { MLinks_Init(); memset(wr, 0, sizeof wr); memset(rd, 0, sizeof rd); memset((void *)ring, 0, sizeof ring); }

static void t_module(void)
{
    char b[96]; int n; MAppSlave_t s0, s1; MCtlOut_t c; uint32_t t;
    /* the two links are independent */
    reset(); n = s_frame(b, 'Y', 1U, 2300U, 2300U, 2U, 0U); push(0, b, n); n = e_frame(b, 1U, LINKV2_FLAG_IN_BAND, 7U); push(0, b, n);
    n = s_frame(b, 'B', 1U, 2250U, 2250U, 1U, 0U); push(1, b, n);
    feed(0, 1000U); feed(1, 1000U); MLinks_Fill(0, &s0, 1050U); MLinks_Fill(1, &s1, 1050U);
    CHECK(s0.have == 1U && s0.link_ok == 1U && s0.vin_dv == 2300U && s0.vout_dv == 2300U && s0.state == 2U && s0.e_valid == 1U && s0.in_band == 1U && s0.clr_echo == 7U, "link 0 (Y): its own readings, state, $E in band and clear echo 7");
    CHECK(s1.have == 1U && s1.vin_dv == 2250U && s1.state == 1U && s1.e_valid == 0U && s1.in_band == 0U && s1.clr_echo == 0U, "link 1 (B): its own readings and state, no $E heard on it (the Y one is not mixed in)");
    /* a corrupt frame on one link is counted there only */
    n = s_frame(b, 'Y', 2U, 2300U, 2300U, 2U, 0U); b[n - 4] ^= 1; push(0, b, n); feed(0, 1100U); feed(1, 1100U);
    CHECK(MLinks_Mon(0)->frames_bad >= 1U && MLinks_Mon(1)->frames_bad == 0U, "a corrupt frame is counted as bad on the link it arrived on only (%u / %u)", (unsigned)MLinks_Mon(0)->frames_bad, (unsigned)MLinks_Mon(1)->frames_bad);
    /* freshness: 500 ms of silence */
    MLinks_Fill(0, &s0, 1000U + 499U); CHECK(s0.have == 1U, "499 ms after its last frame the link is still alive");
    MLinks_Fill(0, &s0, 1000U + 600U); CHECK(s0.have == 0U && s0.link_ok == 0U && s0.vin_dv == 0U && s0.state == 0U && s0.in_band == 0U, "600 ms of silence: no link, no readings, not in band");
    CHECK(s0.e_valid == 1U && s0.clr_echo == 7U, "but what was last heard for the clear counter stays");
    /* v0.9.53: the identity of a slave is its PORT ONLY: the letter in its $S frames is ignored, whatever it is */
    { const char L[3] = { 'R', 'Y', 'B' }; uint8_t k, p; MasterMon_t v;
      for (p = 0U; p < 2U; p++) for (k = 0U; k < 3U; k++)
      {   MAppSlave_t s; reset(); n = s_frame(b, L[k], 1U, 2300U, 2300U, 2U, 0U); push(p, b, n); n = e_frame(b, 1U, LINKV2_FLAG_IN_BAND, 3U); push(p, b, n);
          feed(p, 1000U); MLinks_Fill(p, &s, 1020U);
          CHECK(s.have == 1U && s.link_ok == 1U && s.e_valid == 1U && s.vin_dv == 2300U && s.in_band == 1U, "a board that says '%c' on the %c port is the %c slave: accepted, with its $E", L[k], p ? 'B' : 'Y', p ? 'B' : 'Y');
          CHECK(MLinks_HelloDue(p, 1020U) == 1U, "and it is asked for its hello ('%c' on the %c port)", L[k], p ? 'B' : 'Y');
          MLinks_DisplayMon(p, &v); CHECK(v.last.phase == (p ? 'B' : 'Y'), "the monitor page names the PORT's phase ('%c' on the %c port shows %c)", L[k], p ? 'B' : 'Y', v.last.phase);
      }
      reset(); MLinks_DisplayMon(0U, &v); CHECK(v.have_frame == 0U, "a silent port: the monitor page shows nothing heard"); }
    /* the $H of a board on the Y port reaches the application */
    reset(); n = s_frame(b, 'R', 1U, 2300U, 2300U, 2U, 0U); push(0, b, n); n = e_frame(b, 1U, 0U, 3U); push(0, b, n); n = h_frame(b); push(0, b, n);
    feed(0, 1000U); MLinks_Fill(0, &s0, 1020U); CHECK(s0.h_valid == 1U && MLinks_HelloDue(0, 1020U) == 0U, "its hello is used and not asked again");
    /* v0.9.54: the link's own $K ids (hello, SEQ) stay 1..127 forever: 128..255 are the Re-tune's */
    { int q, bad = 0; reset(); for (q = 0; q < 600; q++) { LinkV2Frame_t f; LinkV2Rx_t rx; int32_t m, j, ty = -1; LinkV2Rx_Init(&rx); m = (q & 1) ? MLinks_BuildHello(0, b, sizeof b) : MLinks_BuildSeq(0, 1U, b, sizeof b); for (j = 0; j < m; j++) { int32_t r = LinkV2Rx_Feed(&rx, (uint8_t)b[j], &f); if (r >= 0) ty = r; } if ((ty != (int32_t)LINKV2_K) || (f.u.k.id == 0U) || (f.u.k.id > 127U)) bad++; }
      CHECK(bad == 0, "600 hello and SEQ frames on a link: every id 1..127 (%d outside)", bad); }
    /* the OVERHEAT flag the slave may set one day reaches the engine */
    reset(); n = s_frame(b, 'Y', 1U, 2300U, 2300U, 2U, 0U); push(0, b, n); n = e_frame(b, 1U, LINKV2_FLAG_OVERHEAT, 0U); push(0, b, n); feed(0, 1000U); MLinks_Fill(0, &s0, 1020U);
    CHECK(s0.heat == 1U && s0.in_band == 0U, "the slave's OVERHEAT flag becomes heat = 1");
    MLinks_Fill(0, &s0, 1000U + 900U); CHECK(s0.heat == 0U, "and only while its $E is fresh");
    /* the heat-sink NTC reading of the slave is passed on, only while the link is fresh */
    { LinkStatus_t st; memset(&st, 0, sizeof st); st.phase = 'B'; st.seq = 1U; st.vin_dv = 2300U; st.vout_dv = 2300U; st.freq_chz = 5000U; st.state = 2U; st.ntc_raw = 2042U;
      reset(); n = (int)Link_BuildStatus(b, 96U, &st); push(1, b, n); feed(1, 1000U); MLinks_Fill(1, &s1, 1020U);
      CHECK(s1.ntc_valid == 1U && s1.ntc_raw == 2042U, "the B slave's NTC reading (2042) reaches the application (%u)", (unsigned)s1.ntc_raw);
      MLinks_Fill(1, &s1, 1700U); CHECK(s1.ntc_valid == 0U && s1.ntc_raw == 0U, "and is gone when the link is silent"); }
    /* $S keeps arriving but the $E stops: after 500 ms the slave is no longer "in band" */
    reset(); n = s_frame(b, 'Y', 1U, 2300U, 2300U, 2U, 0U); push(0, b, n); n = e_frame(b, 1U, LINKV2_FLAG_IN_BAND, 0U); push(0, b, n); feed(0, 1000U); MLinks_Fill(0, &s0, 1100U);
    CHECK(s0.in_band == 1U, "in band while its $E is fresh");
    n = s_frame(b, 'Y', 2U, 2300U, 2300U, 2U, 0U); push(0, b, n); feed(0, 1700U); MLinks_Fill(0, &s0, 1710U);
    CHECK(s0.have == 1U && s0.in_band == 0U, "$S is fresh but the $E is 700 ms old: not in band any more");
    /* a frame split across two reads and across the ring's wrap */
    reset(); wr[0] = RING - 10U; rd[0] = RING - 10U;
    n = s_frame(b, 'Y', 9U, 2311U, 2299U, 2U, 0U); push(0, b, n / 2); feed(0, 1000U); push(0, b + n / 2, n - n / 2); feed(0, 1001U); MLinks_Fill(0, &s0, 1010U);
    CHECK(s0.have == 1U && s0.vin_dv == 2311U && s0.vout_dv == 2299U, "a frame split over two reads and over the end of the ring is decoded (vin %u)", (unsigned)s0.vin_dv);
    /* the $M frames */
    reset(); c.run = 1U; c.clr = 5U; c.setv_dv = 2300U; c.maxc_dv = 500U;
    CHECK(MLinks_BuildM(0, &c, 0U, 0U, b, sizeof b) == 0, "nothing is sent while the application says it is not ready");
    n = (int)MLinks_BuildM(0, &c, 1U, 1000U, b, sizeof b);
    { LinkV2Rx_t rx; LinkV2Frame_t f; int32_t k, got = 0; LinkV2Rx_Init(&rx); for (k = 0; k < n; k++) if (LinkV2Rx_Feed(&rx, (uint8_t)b[k], &f) == (int32_t)LINKV2_M) got = 1;
      CHECK(n > 0 && got && f.u.m.run == 1U && f.u.m.clr == 5U && f.u.m.setv_dv == 2300U && f.u.m.maxc_dv == 500U, "the first $M is sent at once and carries run 1, clear 5, set 230.0 V, Max Correction 50.0 V"); }
    CHECK(MLinks_BuildM(0, &c, 1U, 1050U, b, sizeof b) == 0 && MLinks_BuildM(0, &c, 1U, 1099U, b, sizeof b) == 0, "the same content is not sent again within 100 ms");
    CHECK(MLinks_BuildM(0, &c, 1U, 1100U, b, sizeof b) > 0, "but it is, as a keep-alive, after 100 ms");
    c.run = 0U; CHECK(MLinks_BuildM(0, &c, 1U, 1110U, b, sizeof b) > 0, "a change of the run level is sent AT ONCE");
    c.clr = 6U; CHECK(MLinks_BuildM(0, &c, 1U, 1111U, b, sizeof b) > 0, "so is a change of the clear counter");
    c.setv_dv = 2200U; CHECK(MLinks_BuildM(0, &c, 1U, 1112U, b, sizeof b) > 0, "and of the set voltage");
    c.maxc_dv = 400U; CHECK(MLinks_BuildM(0, &c, 1U, 1113U, b, sizeof b) > 0, "and of the Max Correction");
    CHECK(MLinks_BuildM(1, &c, 1U, 1114U, b, sizeof b) > 0, "link 1 has its own timer: its first frame is due although link 0 just sent");
    { LinkV2Rx_t rx; LinkV2Frame_t f; int32_t k; uint8_t s_a = 0U, s_b = 0U; reset();
      for (t = 0U; t < 3U; t++) { n = (int)MLinks_BuildM(0, &c, 1U, 1000U + t * 100U, b, sizeof b); LinkV2Rx_Init(&rx); for (k = 0; k < n; k++) if (LinkV2Rx_Feed(&rx, (uint8_t)b[k], &f) == (int32_t)LINKV2_M) s_a = f.u.m.seq; }
      n = (int)MLinks_BuildM(1, &c, 1U, 1000U, b, sizeof b); LinkV2Rx_Init(&rx); for (k = 0; k < n; k++) if (LinkV2Rx_Feed(&rx, (uint8_t)b[k], &f) == (int32_t)LINKV2_M) s_b = f.u.m.seq;
      CHECK(s_a == 2U && s_b == 0U, "each link counts its own sequence number (%u after three frames, %u after one)", (unsigned)s_a, (unsigned)s_b); }
    /* the hello request */
    reset(); CHECK(MLinks_HelloDue(0, 1000U) == 0U, "no hello is asked for before anything is heard");
    n = s_frame(b, 'Y', 1U, 2300U, 2300U, 2U, 0U); push(0, b, n); n = e_frame(b, 1U, 0U, 0U); push(0, b, n); feed(0, 1000U);
    CHECK(MLinks_HelloDue(0, 1000U) == 1U && MLinks_HelloDue(0, 1001U) == 0U && MLinks_HelloDue(0, 2999U) == 0U && MLinks_HelloDue(0, 3000U) == 1U, "$E heard and no $H: asked once, and again every 2 s");
    { LinkV2Rx_t rx; LinkV2Frame_t f; int32_t k; uint8_t id1 = 0U, id2 = 0U; n = (int)MLinks_BuildHello(0, b, sizeof b); LinkV2Rx_Init(&rx);
      for (k = 0; k < n; k++) if (LinkV2Rx_Feed(&rx, (uint8_t)b[k], &f) == (int32_t)LINKV2_K) { id1 = f.u.k.id; CHECK(f.u.k.cmd == LINKCMD_HELLO, "the request is a $K HELLO"); }
      n = (int)MLinks_BuildHello(0, b, sizeof b); LinkV2Rx_Init(&rx); for (k = 0; k < n; k++) if (LinkV2Rx_Feed(&rx, (uint8_t)b[k], &f) == (int32_t)LINKV2_K) id2 = f.u.k.id;
      CHECK(id1 != 0U && id2 == (uint8_t)(id1 + 1U), "with an id that counts up (%u, %u)", (unsigned)id1, (unsigned)id2); }
    n = h_frame(b); push(0, b, n); feed(0, 3100U); CHECK(MLinks_HelloDue(0, 6000U) == 0U && MLinks_Fill != 0, "after the $H arrives it is not asked for again");
    /* bad indices are harmless */
    { uint32_t r = 5U; MAppSlave_t z; memset(&z, 0xAA, sizeof z); MLinks_Feed(9, ring[0], RING, &r, 0U, 0U); MLinks_Fill(9, &z, 0U);
      CHECK(z.have == 0U && z.vin_dv == 0U && MLinks_BuildM(9, &c, 1U, 0U, b, sizeof b) == 0 && MLinks_HelloDue(9, 0U) == 0U && MLinks_BuildHello(9, b, sizeof b) == 0, "an index outside the two links does nothing"); }
    printf("  module: independent decoders, freshness, the port is the identity (the letter is ignored), ring wrap, $M timing and sequence numbers, hello\n");
}

/* the R link still goes through Mon_ProcessBytes into g_mon, exactly as before the decoder was made per link */
static void t_r_link(void)
{
    char b[96]; int n, k; uint8_t rr[RING]; uint32_t rrd = 0U, rwr = 0U;
    Mon_Reset(); reset();
    n = s_frame(b, 'R', 4U, 2210U, 2301U, 2U, 0U); for (k = 0; k < n; k++) rr[rwr++] = (uint8_t)b[k];
    n = e_frame(b, 4U, LINKV2_FLAG_IN_BAND, 9U); for (k = 0; k < n; k++) rr[rwr++] = (uint8_t)b[k];
    n = h_frame(b); for (k = 0; k < n; k++) rr[rwr++] = (uint8_t)b[k];
    Mon_ProcessBytes(rr, RING, &rrd, rwr, 1000U);
    CHECK(g_mon.have_frame == 1U && g_mon.last.phase == 'R' && g_mon.last.vin_dv == 2210U && g_mon.last.vout_dv == 2301U && g_mon.last.state == 2U && g_mon.frames_ok == 1U, "the R link: the $S frame is decoded into g_mon (vin %u)", (unsigned)g_mon.last.vin_dv);
    CHECK(g_mon.e_valid == 1U && g_mon.e_clr == 9U && (g_mon.e_flags2 & LINKV2_FLAG_IN_BAND) != 0U && g_mon.e_setv_dv == 2300U && g_mon.h_valid == 1U && g_mon.h_vout_max_dv == 2400U, "and $E and $H too (clear echo %u, vout max %u)", (unsigned)g_mon.e_clr, (unsigned)g_mon.h_vout_max_dv);
    CHECK(rrd == rwr && g_mon.bytes_rx == rwr, "every byte was consumed and counted");
    CHECK(MLinks_Mon(0)->have_frame == 0U && MLinks_Mon(1)->have_frame == 0U, "and nothing reached the Y and B links");
    printf("  R link: Mon_ProcessBytes still fills g_mon ($S, $E, $H), the Y and B links are separate\n");
}

/* ======================= the closed loop ======================= */
static uint8_t fm[8 * MFLASH_PAGE];
static int f_read(uint32_t a, uint8_t *b, uint32_t n) { memcpy(b, &fm[a], n); return 0; }
static int f_prog(uint32_t a, const uint8_t *b, uint32_t n) { uint32_t i; for (i = 0U; i < n; i++) fm[a + i] &= b[i]; return 0; }
static int f_erase(uint32_t a) { memset(&fm[a], 0xFF, MFLASH_PAGE); return 0; }
static const MFlash_t FL = { f_read, f_prog, f_erase };
static MApp_t A; static MAppData_t D; static uint32_t T; static char F[4][21]; static uint8_t K[4];

typedef struct
{   /* a simulated slave on a cable */
    char     letter; uint8_t silent, seq, state, fault, run_seen, clr_seen, h_pending, h_sent, rx_started;
    uint16_t vin, vout, ntc; uint32_t t_s, t_e, t_run, n_m; LinkV2Rx_t rx;
    uint32_t phi_us; uint8_t armed, no_seq; uint16_t arm_round, sq_id, sq_delay, sq_period;      /* the phase sequence: the time of its rising zero-cross within a period, armed by $K SEQ, the last result */
    uint16_t iout_pk; uint8_t iout_clip, n_cal, last_cal_cmd; int32_t last_cal_num;      /* v0.9.55: its converter-current peak (hundredths of an ampere, 0 = not measured) and the calibration commands it got */
    uint8_t  last_kid, tuning, tune_fail, n_tune, n_abort, max_aux_id, n_hello_rx; uint16_t prof_cap; uint32_t tune_t0, tune_t_last; uint8_t tune_k;   /* v0.9.54: its Re-tune, as the real slave (a $K with the id of the last one is a retry: not executed) */
} Sim_t;
static uint16_t raw_of(double tc) { double k = tc + 273.15, r = 10000.0 * exp(3950.0 * (1.0 / k - 1.0 / 298.15)); return (uint16_t)(4095.0 * r / (r + 1500.0) + 0.5); }
static Sim_t SM[2]; static uint32_t T_now_sim;
static void sim_init(Sim_t *s, char letter) { memset(s, 0, sizeof *s); s->letter = letter; s->vin = 2300U; s->vout = 2300U; s->ntc = 3000U; s->h_pending = 3U; s->phi_us = 0U; LinkV2Rx_Init(&s->rx); }
static void sim_tick(int link, uint32_t now)
{
    Sim_t *s = &SM[link]; char b[96]; int n;
    if (s->silent != 0U) return;
    if ((s->run_seen != 0U) && (s->state == 0U)) { s->state = 1U; s->t_run = now; }
    if ((s->state == 1U) && ((uint32_t)(now - s->t_run) >= 500U)) s->state = 2U;
    if ((s->run_seen == 0U) && (s->state != 3U)) s->state = 0U;
    s->vout = (s->state == 2U) ? 2300U : s->vin;
    if ((uint32_t)(now - s->t_s) >= 10U) { s->t_s = now; g_frame_ntc = s->ntc; n = s_frame(b, s->letter, s->seq++, s->vin, s->vout, s->state, s->fault); g_frame_ntc = 3000U; push((uint8_t)link, b, n); }
    if ((uint32_t)(now - s->t_e) >= 100U) { s->t_e = now; g_e_sq_id = s->sq_id; g_e_sq_delay = s->sq_delay; g_e_sq_period = s->sq_period; g_e_iout_pk = s->iout_pk; g_e_iout_clip = s->iout_clip; n = e_frame(b, s->seq, (uint16_t)((s->state == 2U) ? LINKV2_FLAG_IN_BAND : 0U), s->clr_seen); g_e_sq_id = 0U; g_e_sq_delay = 0U; g_e_sq_period = 0U; g_e_iout_pk = 0U; push((uint8_t)link, b, n); }
    if (s->h_pending != 0U) { s->h_pending--; n = h_frame_p(b, s->prof_cap); push((uint8_t)link, b, n); }
    if ((s->tuning != 0U) && ((uint32_t)(now - s->tune_t_last) >= 500U))
    {   /* a $T frame every 500 ms: steps 1, 2, 3, then the result */
        LinkT_t t; memset(&t, 0, sizeof t); s->tune_t_last = now; s->tune_k++; t.step = s->tune_k; t.ticks = (uint16_t)(100U * s->tune_k); t.vin_dv = 2000U; t.dv = (int16_t)(40 * s->tune_k);
        if (s->tune_k >= 4U) { t.result = (s->tune_fail != 0U) ? LINKV2_T_NO_RESPONSE : LINKV2_T_FINISHED; s->tuning = 0U; if (s->tune_fail == 0U) s->prof_cap = (uint16_t)(1200U + (uint16_t)link); }
        n = (int)LinkV2_BuildT(b, 96U, &t); push((uint8_t)link, b, n);
    }
}
static void sim_rx(int link, const char *b, int n)       /* the $M and $K frames the master sent to this slave */
{
    Sim_t *s = &SM[link]; LinkV2Frame_t f; int k;
    if (s->silent != 0U) return;
    for (k = 0; k < n; k++)
    {
        int32_t t = LinkV2Rx_Feed(&s->rx, (uint8_t)b[k], &f);
        if (t == (int32_t)LINKV2_M) { s->n_m++; s->run_seen = f.u.m.run; if (f.u.m.clr != s->clr_seen) { s->clr_seen = f.u.m.clr; if (s->state == 3U) { s->fault = 0U; s->state = 0U; } } }
        else if (t == (int32_t)LINKV2_K)
        {
            char rb[64]; int rn; LinkR_t r; uint8_t retry = (uint8_t)((f.u.k.id == s->last_kid) && (s->last_kid != 0U));
            s->last_kid = f.u.k.id;
            if ((f.u.k.cmd == LINKCMD_HELLO) || (f.u.k.cmd == LINKCMD_SEQ)) { if (f.u.k.id > s->max_aux_id) s->max_aux_id = f.u.k.id; }
            if (retry != 0U) { if ((f.u.k.cmd == LINKCMD_TUNE) || (f.u.k.cmd == LINKCMD_TABORT)) { r.id = f.u.k.id; r.status = 0U; r.value = 0; rn = (int)LinkV2_BuildR(rb, sizeof rb, &r); push((uint8_t)link, rb, rn); } continue; }      /* answered again, executed once */
            if (f.u.k.cmd == LINKCMD_HELLO) { s->h_pending = 1U; s->n_hello_rx++; }
            else if ((f.u.k.cmd == LINKCMD_SEQ) && (s->no_seq == 0U)) { s->armed = 1U; s->arm_round = (uint16_t)f.u.k.num; }
            else if (f.u.k.cmd == LINKCMD_TUNE) { s->n_tune++; s->tuning = 1U; s->tune_k = 0U; s->tune_t0 = s->tune_t_last = T_now_sim; r.id = f.u.k.id; r.status = 0U; r.value = 0; rn = (int)LinkV2_BuildR(rb, sizeof rb, &r); push((uint8_t)link, rb, rn); }
            else if ((f.u.k.cmd == LINKCMD_CALV) || (f.u.k.cmd == LINKCMD_CALO) || (f.u.k.cmd == LINKCMD_CALR))
            {   s->n_cal++; s->last_cal_cmd = (uint8_t)f.u.k.cmd; s->last_cal_num = f.u.k.num; r.id = f.u.k.id; r.status = 0U; r.value = 10050; rn = (int)LinkV2_BuildR(rb, sizeof rb, &r); push((uint8_t)link, rb, rn); }
            else if (f.u.k.cmd == LINKCMD_TABORT)
            {   LinkT_t tt; s->n_abort++; r.id = f.u.k.id; r.status = 0U; r.value = 0; rn = (int)LinkV2_BuildR(rb, sizeof rb, &r); push((uint8_t)link, rb, rn);
                if (s->tuning != 0U) { s->tuning = 0U; memset(&tt, 0, sizeof tt); tt.result = LINKV2_T_BY_MASTER; rn = (int)LinkV2_BuildT(rb, sizeof rb, &tt); push((uint8_t)link, rb, rn); } }
        }
    }
}
/* ---- the mains the three slaves see: a period and each phase's rising zero-cross time. KIND: 0 = R-Y-B (Y lags R by a third, B by two thirds), 1 = R-B-Y, 2 = Y on R's phase ---- */
static uint32_t MAINS_T = 20000U, PHI_R = 3000U; static int g_kind = 0; static uint8_t r_armed, r_no_seq; static uint16_t r_round; static int n_arm, n_sync; static uint32_t t_arm_ms, t_sync_ms; static uint8_t last_round;
static uint32_t rnd32(void) { static uint32_t s = 0x9E3779B1U; s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
static void apply_kind(void)
{
    uint32_t th = MAINS_T / 3U;
    SM[0].phi_us = (g_kind == 0) ? PHI_R + th : ((g_kind == 1) ? PHI_R + 2U * th : PHI_R);
    SM[1].phi_us = (g_kind == 0) ? PHI_R + 2U * th : ((g_kind == 1) ? PHI_R + th : PHI_R + 2U * th);
}
static void mains_sync(uint32_t t_ms)
{   /* the master's SYNC byte reaches the three slaves at the same instant (a few tens of microseconds apart); each measures its time to its next rising edge, with a little jitter */
    int i;
    for (i = 0; i < 3; i++)
    {
        uint32_t phi = (i == 0) ? PHI_R : SM[i - 1].phi_us, sync = t_ms * 1000U + (rnd32() % 60U), edge, d; uint8_t armed = (i == 0) ? r_armed : SM[i - 1].armed; uint16_t round = (i == 0) ? r_round : SM[i - 1].arm_round;
        if ((armed == 0U) || ((i > 0) && (SM[i - 1].silent != 0U))) continue;
        edge = (phi + (rnd32() % 101U) + MAINS_T - 50U) % MAINS_T; d = (edge + MAINS_T - (sync % MAINS_T)) % MAINS_T;
        if (i == 0) { D.sq_id = round; D.sq_delay_us = (uint16_t)d; D.sq_period_us = (uint16_t)MAINS_T; r_armed = 0U; }
        else { SM[i - 1].sq_id = round; SM[i - 1].sq_delay = (uint16_t)d; SM[i - 1].sq_period = (uint16_t)MAINS_T; SM[i - 1].armed = 0U; }
    }
}
static void seq_actions(void)
{
    uint8_t round, act = MApp_TakeSeqAction(&A, &round); char b[96]; int32_t n; int i;
    if (act == 1U)
    {   /* ARM on the three links */
        n_arm++; t_arm_ms = T; last_round = round;
        for (i = 0; i < 2; i++) { n = MLinks_BuildSeq((uint8_t)i, round, b, sizeof b); if (n > 0) sim_rx(i, b, (int)n); }
        if (r_no_seq == 0U) { r_armed = 1U; r_round = round; }
    }
    else if (act == 2U) { n_sync++; t_sync_ms = T; mains_sync(T); }
}
/* v0.9.54: the R slave's Re-tune, as the glue sees it (its $R goes to MApp_OnKResult, its $T frames into D) */
static uint8_t r_last_kid, r_tuning, r_k, r_fail, r_n_tune, r_n_abort, r_ack_pending, r_ack_id; static uint32_t r_t_last; static int g_max_tuning, g_order[3], g_n_order;
static uint8_t r_n_cal;
static void r_k_cmd(const MCalK_t *ck)
{
    if ((ck->cmd == MAPP_KCMD_CALV) || (ck->cmd == MAPP_KCMD_CALO) || (ck->cmd == MAPP_KCMD_CALR)) r_n_cal++;
    uint8_t retry = (uint8_t)((ck->id == r_last_kid) && (r_last_kid != 0U)); r_last_kid = ck->id;
    if ((ck->cmd != MAPP_KCMD_TUNE) && (ck->cmd != MAPP_KCMD_TABORT)) return;
    r_ack_pending = 1U; r_ack_id = ck->id; if (retry != 0U) return;
    if (ck->cmd == MAPP_KCMD_TUNE) { r_n_tune++; r_tuning = 1U; r_k = 0U; r_t_last = T; }
    else { r_n_abort++; if (r_tuning != 0U) { r_tuning = 0U; D.tune_n++; D.tune_step = 0U; D.tune_result = LINKV2_T_BY_MASTER; } }
}
static void r_tick(void)
{
    if (r_ack_pending != 0U) { r_ack_pending = 0U; MApp_OnKResult(&A, r_ack_id, 0U, 0); }
    if ((r_tuning != 0U) && ((uint32_t)(T - r_t_last) >= 500U))
    {   r_t_last = T; r_k++; D.tune_n++; D.tune_step = r_k; D.tune_ticks = (uint16_t)(100U * r_k); D.tune_dv_dv = (int16_t)(40 * r_k); D.tune_result = 0U;
        if (r_k >= 4U) { D.tune_result = (r_fail != 0U) ? LINKV2_T_NO_RESPONSE : LINKV2_T_FINISHED; r_tuning = 0U; if (r_fail == 0U) D.prof_cap = 1111U; } }
}
static void loop(uint32_t ms)
{
    uint32_t e = T + ms;
    while (T < e)
    {
        int i; char b[96]; int32_t n; static uint32_t r_seen_yb[2];
        T_now_sim = T;
        for (i = 0; i < 2; i++) { sim_tick(i, T); feed((uint8_t)i, T); MLinks_Fill((uint8_t)i, &D.yb[i], T); }
        r_tick();
        MApp_Step(&A, K[0], K[1], K[2], K[3], &D, T, F);
        seq_actions();
        {   /* the glue: a $K goes on the link of its phase (v0.9.54) */
            MCalK_t ck; LinkK_t k;
            while (MApp_TakeKCommand(&A, &ck, T) != 0U)
            {
                if (ck.ph == 0U) { r_k_cmd(&ck); continue; }
                memset(&k, 0, sizeof k); k.id = ck.id; k.cmd = (ck.cmd == MAPP_KCMD_TUNE) ? LINKCMD_TUNE : ((ck.cmd == MAPP_KCMD_TABORT) ? LINKCMD_TABORT : ((ck.cmd == MAPP_KCMD_CALV) ? LINKCMD_CALV : ((ck.cmd == MAPP_KCMD_CALO) ? LINKCMD_CALO : LINKCMD_CALR))); k.num = ck.num; k.letter = ck.letter;
                n = LinkV2_BuildK(b, sizeof b, &k); if ((n > 0) && (ck.ph <= 2U)) sim_rx(ck.ph - 1U, b, (int)n);
            }
            for (i = 0; i < 2; i++) { const MasterMon_t *ym = MLinks_Mon((uint8_t)i); if (ym->r_n != r_seen_yb[i]) { r_seen_yb[i] = ym->r_n; MApp_OnKResultPh(&A, (uint8_t)(1 + i), ym->r_id, ym->r_status, ym->r_value); } }
        }
        for (i = 0; i < 2; i++)
        {
            n = MLinks_BuildM((uint8_t)i, MApp_ControlPh(&A, (uint8_t)(1 + i)), MApp_ControlValidPh(&A, (uint8_t)(1 + i)), T, b, sizeof b); if (n > 0) sim_rx(i, b, (int)n);
            if ((MApp_TakeYbHello(&A, (uint8_t)i) | MLinks_HelloDue((uint8_t)i, T)) != 0U) { n = MLinks_BuildHello((uint8_t)i, b, sizeof b); if (n > 0) sim_rx(i, b, (int)n); }
        }
        {   /* never two stages tuning at once; the order the phases started in */
            int nt = (int)r_tuning + (int)SM[0].tuning + (int)SM[1].tuning; if (nt > g_max_tuning) g_max_tuning = nt;
            if ((g_n_order < 3) && ((r_tuning && (g_n_order == 0 || g_order[g_n_order - 1] != 0)) )) g_order[g_n_order++] = 0;
            if ((g_n_order < 3) && SM[0].tuning && (g_n_order == 0 || g_order[g_n_order - 1] != 1)) g_order[g_n_order++] = 1;
            if ((g_n_order < 3) && SM[1].tuning && (g_n_order == 0 || g_order[g_n_order - 1] != 2)) g_order[g_n_order++] = 2;
        }
        T += 10U;
    }
}
static int g_kind0 = 0, g_phase_seq0 = 0, g_noseq0 = 0, g_lettersR0 = 0;      /* presets applied before the first pass of the loop in setup() */
static void setup(uint8_t mode)
{
    memset(fm, 0xFF, sizeof fm); memset(&D, 0, sizeof D); memset(K, 0, sizeof K); T = 0U; reset();
    r_n_cal = 0U; r_last_kid = 0U; r_tuning = 0U; r_k = 0U; r_fail = 0U; r_n_tune = 0U; r_n_abort = 0U; r_ack_pending = 0U; g_max_tuning = 0; g_n_order = 0;
    D.have = 1U; D.link_ok = 1U; D.vin_dv = 2300U; D.vout_dv = 2300U; D.freq_chz = 5000U; D.state = 2U; D.e_valid = 1U; D.h_valid = 1U; D.in_band = 1U; D.load_ok = 1U;
    MApp_Init(&A, &FL, 0U, MFLASH_PAGE, 2U * MFLASH_PAGE, 3U * MFLASH_PAGE, T);
    A.settings.mode = mode; A.settings.scroll = 0U; A.settings.phase_seq = (uint8_t)g_phase_seq0; MF_Init(&A.eng, &A.settings, T);
    sim_init(&SM[0], 'Y'); sim_init(&SM[1], 'B'); if (g_lettersR0 != 0) { SM[0].letter = 'R'; SM[1].letter = 'R'; } r_armed = 0U; r_no_seq = (uint8_t)g_noseq0; SM[0].no_seq = (uint8_t)g_noseq0; SM[1].no_seq = (uint8_t)g_noseq0; n_arm = 0; n_sync = 0; g_kind = g_kind0; apply_kind();
    loop(MUI_SPLASH_MS + 200U);
}
static void t_loop(void)
{
    const MFOut_t *o;
    /* DOMESTIC: R from the test, Y and B through the links */
    setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && o->contactor[1] == 1U && o->contactor[2] == 1U && SM[0].state == 2U && SM[1].state == 2U, "closed loop, Domestic: the application starts the Y and B slaves over the links and the three contactors close (%u %u %u, slaves %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2], (unsigned)SM[0].state, (unsigned)SM[1].state);
    CHECK(MLinks_Mon(0)->frames_bad == 0U && MLinks_Mon(1)->frames_bad == 0U && SM[0].n_m > 100U && SM[1].n_m > 100U, "no bad frame on either link, and each slave received more than 100 $M frames (%u, %u)", (unsigned)SM[0].n_m, (unsigned)SM[1].n_m);
    CHECK(MLinks_Mon(0)->h_valid == 1U && MLinks_Mon(1)->h_valid == 1U, "both slaves' hellos were received");
    /* the Y heat sink gets hot (85 degC in its status frames): only the Y unit trips, and it restarts when cooled */
    SM[0].ntc = raw_of(85.0); loop(12500U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && o->contactor[1] == 0U && o->contactor[2] == 1U && o->fault[1] == MLOG_T_HEAT, "the Y slave reports an 85 degC heat sink over the link: ONLY the Y unit trips on Overheat (contactors %u %u %u, faults %u %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2], (unsigned)o->fault[0], (unsigned)o->fault[1], (unsigned)o->fault[2]);
    SM[0].ntc = raw_of(55.0); loop(90000U); o = MApp_Engine(&A);
    CHECK(o->contactor[1] == 1U && o->contactor[0] == 1U && o->contactor[2] == 1U, "cooled to 55 degC: the Y unit restarts by itself (%u %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2]);
    /* a cable cut to Y */
    SM[0].silent = 1U; loop(4000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && o->contactor[1] == 0U && o->contactor[2] == 1U && o->fault[1] != 0U, "the Y cable is cut: ONLY the Y unit trips (contactors %u %u %u, faults %u %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2], (unsigned)o->fault[0], (unsigned)o->fault[1], (unsigned)o->fault[2]);
    CHECK(SM[1].run_seen == 1U && SM[1].state == 2U, "the B slave keeps running, still told run = 1");
    SM[0].silent = 0U; SM[0].state = 0U; SM[0].run_seen = 0U; loop(60000U); o = MApp_Engine(&A);
    CHECK(o->contactor[1] == 1U && o->contactor[0] == 1U && o->contactor[2] == 1U, "the Y cable is back: the Y unit restarts by itself and closes again (%u %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2]);
    /* an inrush trip of B, then START */
    SM[1].fault = 1U; SM[1].state = 3U; loop(2000U); o = MApp_Engine(&A);
    CHECK(o->contactor[2] == 0U && o->contactor[1] == 1U && o->contactor[0] == 1U && o->fault[2] == MLOG_T_INRUSH, "an inrush trip of the B slave: only the B unit is off");
    { uint8_t clr0 = SM[1].clr_seen, y0 = SM[0].clr_seen; K[1] = 1U; loop(150U); K[1] = 0U; loop(30000U); o = MApp_Engine(&A);
      CHECK(SM[1].clr_seen == (uint8_t)(clr0 + 1U) && SM[1].clr_seen == MApp_ControlPh(&A, 2)->clr && SM[0].clr_seen == y0, "START retries B: its clear counter reaches the slave over the link (%u -> %u) and the Y slave's does not move (%u -> %u)", (unsigned)clr0, (unsigned)SM[1].clr_seen, (unsigned)y0, (unsigned)SM[0].clr_seen);
      CHECK(o->contactor[2] == 1U && SM[1].state == 2U, "and the B unit runs again"); }
    /* the letters do not matter: the Y board says B and the B board says Y (two boards swapped in their letters, not in their cables) */
    setup((uint8_t)M_MODE_DOMESTIC); SM[0].letter = 'B'; SM[1].letter = 'Y'; loop(15000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && o->contactor[1] == 1U && o->contactor[2] == 1U && SM[0].run_seen != 0U && SM[1].run_seen != 0U, "boards that say B on the Y port and Y on the B port: all three units run (contactors %u %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2]);
    SM[1].silent = 1U; loop(4000U); o = MApp_Engine(&A);
    CHECK(o->contactor[2] == 0U && o->contactor[1] == 1U, "the board on the B PORT goes silent (it says Y): the B unit trips, not the Y one");
    /* INDUSTRIAL */
    setup((uint8_t)M_MODE_INDUSTRIAL); loop(15000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && o->contactor[1] == 0U && o->contactor[2] == 0U && SM[0].state == 2U && SM[1].state == 2U, "closed loop, Industrial: all three slaves run and ONE contactor (PA10) closes (%u %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2]);
    SM[0].silent = 1U; loop(4000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 0U && SM[1].run_seen == 0U && SM[1].state == 0U, "the Y cable is cut: the contactor opens and the B slave is told to stop too (run %u, state %u)", (unsigned)SM[1].run_seen, (unsigned)SM[1].state);
    setup((uint8_t)M_MODE_INDUSTRIAL); SM[1].silent = 1U; loop(15000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 0U && SM[0].run_seen == 0U, "Industrial with the B slave never connected: nothing starts");
    printf("  closed loop: simulated Y and B slaves over the real frames: start, cable cut and recovery, inrush trip and START, letters ignored (the port decides), Industrial\n");
}


/* ======================= the phase sequence, end to end ======================= */
static int line_v(int row, const char *name) { char fmt[24]; int v = -1; (void)snprintf(fmt, sizeof fmt, "%s : %%d", name); if (sscanf(F[row], fmt, &v) != 1) v = -1; return v; }
static void page1(void) { int k; for (k = 0; k < 3 && strstr(F[0], "R-Y") == NULL; k++) { K[0] = 1U; loop(150U); K[0] = 0U; loop(120U); } }      /* the page with the phase-to-phase voltages (MENU shifts the page; Display Scroll is off) */
static void t_sequence_loop(void)
{
    const MFOut_t *o; const MPhase_t *ph;
    /* DOMESTIC, R-Y-B: measured over the real frames, the line voltages on the Run screen */
    g_kind0 = 0; g_phase_seq0 = 0; g_noseq0 = 0; setup((uint8_t)M_MODE_DOMESTIC); A.settings.scroll = 0U; loop(4000U); ph = MApp_Phase(&A);
    CHECK(ph->state == MPH_OK && ph->have_ang != 0U && abs((int)ph->ang_dd[1] - 1200) < 60 && abs((int)ph->ang_dd[2] - 2400) < 60, "Domestic, R-Y-B: measured over the frames: OK, Y at %u and B at %u tenths of a degree", (unsigned)ph->ang_dd[1], (unsigned)ph->ang_dd[2]);
    CHECK(n_arm >= 3 && (n_sync == n_arm || n_sync == n_arm - 1), "an ARM and a SYNC per round (%d, %d)", n_arm, n_sync);
    CHECK((int)(t_sync_ms - t_arm_ms) >= 20 && (int)(t_sync_ms - t_arm_ms) <= 40, "the SYNC comes 20 to 40 ms after the ARM (%d ms)", (int)(t_sync_ms - t_arm_ms));
    page1(); CHECK(line_v(0, "R-Y") >= 396 && line_v(0, "R-Y") <= 401 && line_v(1, "Y-B") >= 396 && line_v(1, "Y-B") <= 401 && line_v(2, "B-R") >= 396 && line_v(2, "B-R") <= 401, "the phase-to-phase voltages of three 230 V phases: R-Y %d, Y-B %d, B-R %d V (398)", line_v(0, "R-Y"), line_v(1, "Y-B"), line_v(2, "B-R"));
    o = MApp_Engine(&A); CHECK(o->contactor[0] == 1U && o->contactor[1] == 1U && o->contactor[2] == 1U, "and the three units run");
    /* R-B-Y: the angles swap, the line voltages are the same, Domestic does not care */
    g_kind = 1; apply_kind(); loop(25000U); ph = MApp_Phase(&A);
    CHECK(ph->state == MPH_REVERSE && abs((int)ph->ang_dd[1] - 2400) < 60 && abs((int)ph->ang_dd[2] - 1200) < 60, "the sequence is reversed: REVERSE, Y at %u and B at %u", (unsigned)ph->ang_dd[1], (unsigned)ph->ang_dd[2]);
    o = MApp_Engine(&A); CHECK(o->contactor[0] == 1U && o->contactor[1] == 1U && o->contactor[2] == 1U, "Domestic has no sequence check: the three units keep running");
    { int ok; page1(); ok = line_v(0, "R-Y") >= 396 && line_v(0, "R-Y") <= 401; CHECK(ok, "and the line voltage is still 398 V (%d)", line_v(0, "R-Y")); }
    /* INDUSTRIAL with the Phase Sequence check on: it waits for the measurement, then starts */
    g_kind0 = 0; g_phase_seq0 = 1; setup((uint8_t)M_MODE_INDUSTRIAL);
    { uint32_t t_ok = 0U, t_c = 0U, k; for (k = 0U; k < 150U; k++) { loop(100U); if ((t_ok == 0U) && (MApp_Phase(&A)->state == MPH_OK)) t_ok = T; if ((t_c == 0U) && (MApp_Engine(&A)->contactor[0] != 0U)) t_c = T; }
      CHECK(t_ok != 0U && t_c != 0U && t_c >= t_ok, "Industrial, Phase Sequence on: the unit closes its contactor only AFTER the sequence was measured OK (measured at %u ms, closed at %u ms)", (unsigned)t_ok, (unsigned)t_c); }
    o = MApp_Engine(&A); ph = MApp_Phase(&A);
    CHECK(ph->state == MPH_OK && o->contactor[0] == 1U && o->contactor[1] == 0U && o->contactor[2] == 0U, "measured OK: the unit starts and closes the one contactor (state %u, contactors %u %u %u)", (unsigned)ph->state, (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2]);
    /* the sequence turns over while it runs: it trips (after three rounds, 5 s apart), and recovers when it is right again */
    g_kind = 1; apply_kind(); loop(30000U); o = MApp_Engine(&A);
    CHECK(MApp_Phase(&A)->state == MPH_REVERSE && o->contactor[0] == 0U && o->shown == MLOG_T_PHASE_SEQ, "the sequence turns over while running: REVERSE is measured and the unit trips: Phase Sequence (shown %u)", (unsigned)o->shown);
    g_kind = 0; apply_kind(); loop(60000U); o = MApp_Engine(&A);
    CHECK(MApp_Phase(&A)->state == MPH_OK && o->contactor[0] == 1U, "right again: it restarts by itself");
    /* INDUSTRIAL started with the wrong sequence, and with the same phase twice: never starts */
    g_kind0 = 1; g_phase_seq0 = 1; setup((uint8_t)M_MODE_INDUSTRIAL); loop(20000U); o = MApp_Engine(&A);
    CHECK(MApp_Phase(&A)->state == MPH_REVERSE && o->contactor[0] == 0U && o->run[0] == 0U && MApp_ControlPh(&A, 1)->run == 0U, "Industrial on a reversed sequence: never starts, no slave is told to run");
    g_kind0 = 2; g_phase_seq0 = 1; setup((uint8_t)M_MODE_INDUSTRIAL); loop(20000U); o = MApp_Engine(&A);
    CHECK(MApp_Phase(&A)->state == MPH_SAME && o->contactor[0] == 0U, "Industrial with the Y slave on R's phase: SAME, never starts");
    /* v0.9.54 (bench: "it shows 5 s and hangs there"): a sequence measured wrong BEFORE the start held the on-delay at "Start in 05 s" with no reason. It must SAY why. */
    CHECK(strstr(A.status, "Start in") == NULL && strstr(A.status, "Phase Seq Same Ph") != NULL, "SAME before the start: the status row says so ('%s'), no frozen countdown", A.status);
    CHECK(strstr(F[1], "SAME PHASE") != NULL && F[0][0] == ' ' && strspn(F[0], " ") == 20U, "v0.9.57: Industrial, the status page says it alone in the middle ('%s')", F[1]);
    CHECK(o->shown == MLOG_T_PHASE_SEQ, "and the fault display shows Phase Sequence (shown %u)", (unsigned)o->shown);
    g_kind0 = 1; g_phase_seq0 = 1; setup((uint8_t)M_MODE_INDUSTRIAL); loop(20000U); o = MApp_Engine(&A);
    CHECK(strstr(A.status, "Start in") == NULL && strstr(A.status, "Phase Seq Reverse") != NULL && o->shown == MLOG_T_PHASE_SEQ && strstr(F[1], "PHASE REVERSE") != NULL, "REVERSE before the start: 'Phase Seq Reverse' ('%s'), Phase Sequence shown (%u), status page '%s'", A.status, (unsigned)o->shown, F[1]);
    g_kind = 0; apply_kind(); { uint32_t k, seen_cd = 0U, t_c = 0U; char prev[21] = ""; int counts = 0;
      for (k = 0U; k < 300U; k++) { loop(100U); if (strstr(A.status, "Start in") != NULL) { seen_cd = 1U; if (strcmp(prev, A.status) != 0) { counts++; memcpy(prev, A.status, sizeof prev); } } if ((t_c == 0U) && (MApp_Engine(&A)->contactor[0] != 0U)) t_c = T; }
      CHECK(t_c != 0U && seen_cd != 0U && counts >= 3, "the sequence is put right: the countdown RUNS (%d different values) and the unit starts by itself (closed at %u ms)", counts, (unsigned)t_c); }
    o = MApp_Engine(&A); CHECK(o->shown != MLOG_T_PHASE_SEQ && strstr(A.status, "Phase Seq") == NULL, "and the Phase Sequence message is gone");
    g_kind0 = 1; g_phase_seq0 = 0; setup((uint8_t)M_MODE_INDUSTRIAL);
    { uint32_t k; int said = 0, cd = 0; for (k = 0U; k < 80U; k++) { loop(100U); if (strstr(A.status, "Phase Seq") != NULL) said = 1; if (strstr(A.status, "Start in") != NULL) cd = 1; }
      CHECK(said == 0 && cd == 1 && MApp_Phase(&A)->state == MPH_REVERSE, "Phase Sequence = No, reversed: the start countdown runs and never names the sequence"); }
    loop(12000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && o->shown != MLOG_T_PHASE_SEQ, "Phase Sequence = No: a reversed sequence is not reported as a fault and the unit runs");
    g_kind0 = 1; g_phase_seq0 = 1; setup((uint8_t)M_MODE_DOMESTIC); loop(20000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && o->contactor[1] == 1U && o->contactor[2] == 1U && o->shown != MLOG_T_PHASE_SEQ, "Domestic never checks it, even with the setting on");
    g_kind0 = 2; g_phase_seq0 = 0; setup((uint8_t)M_MODE_INDUSTRIAL); loop(20000U); o = MApp_Engine(&A);
    CHECK(MApp_Phase(&A)->state == MPH_SAME && o->contactor[0] == 1U, "with the Phase Sequence check OFF the same measurement is only shown: it starts");
    /* slaves built without the measurement: the rounds back off, Domestic is not affected, Industrial with the check waits and says so */
    g_kind0 = 0; g_phase_seq0 = 1; g_noseq0 = 1; setup((uint8_t)M_MODE_INDUSTRIAL); loop(12000U); o = MApp_Engine(&A);
    CHECK(MApp_Phase(&A)->state == MPH_UNKNOWN && o->contactor[0] == 0U && strstr(F[3], "Checking Phase Seq") != NULL, "slaves that never answer the rounds: unknown, Industrial with the check does not start and says why ('%s')", F[3]);
    CHECK(n_arm >= 2 && n_arm <= 8, "and the rounds back off to one every 2 s or so (%d in 12 s, not 40)", n_arm);
    g_phase_seq0 = 0; setup((uint8_t)M_MODE_DOMESTIC); loop(12000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && o->contactor[1] == 1U && o->contactor[2] == 1U && MApp_Phase(&A)->state == MPH_UNKNOWN, "Domestic with such slaves runs normally, the sequence just stays unknown");
    /* a slave that goes silent: the decision is gone at once, and comes back */
    g_noseq0 = 0; g_kind0 = 0; setup((uint8_t)M_MODE_DOMESTIC); loop(4000U); CHECK(MApp_Phase(&A)->state == MPH_OK, "(measured again)");
    SM[0].silent = 1U; loop(1200U); CHECK(MApp_Phase(&A)->state == MPH_UNKNOWN && MApp_Phase(&A)->have_ang == 0U, "the Y slave goes silent: no decision, no angles");
    SM[0].silent = 0U; loop(8000U); CHECK(MApp_Phase(&A)->state == MPH_OK, "it comes back: measured again");
    /* Single mode: nothing is ever sent */
    setup((uint8_t)M_MODE_SINGLE); loop(8000U); CHECK(n_arm == 0 && n_sync == 0 && MApp_Phase(&A)->state == MPH_UNKNOWN, "Single mode: no round is ever started (%d)", n_arm);
    /* the round id wraps from 255 to 1, never 0 */
    setup((uint8_t)M_MODE_DOMESTIC); A.seq_round = 253U; { int seen0 = 0, k; for (k = 0; k < 30; k++) { loop(300U); if (last_round == 0U) seen0 = 1; } CHECK(!seen0 && MApp_Phase(&A)->state == MPH_OK, "round ids 254, 255, 1, 2 ...: never 0, and the measurement goes on (last round %u)", (unsigned)last_round); }
    printf("  phase sequence end to end: measured over the real frames, line voltages on the Run screen, reverse and same phase, Industrial waits and says so, a mid-run swap trips and recovers, slaves without it, a silent slave, Single, id wrap\n");
}


/* ======================= THREE IDENTICAL BOARDS: every one says R, the PORT is the identity ======================= */
static void t_all_r_boards(void)
{
    const MFOut_t *o; const MPhase_t *ph; MasterMon_t v;
    g_lettersR0 = 1; g_kind0 = 0; g_phase_seq0 = 0; g_noseq0 = 0; setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); o = MApp_Engine(&A); ph = MApp_Phase(&A);
    CHECK(o->contactor[0] == 1U && o->contactor[1] == 1U && o->contactor[2] == 1U && SM[0].state == 2U && SM[1].state == 2U, "three identical boards (all built as R): Domestic runs: the Y and B slaves are told to run and the three contactors close (%u %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2]);
    CHECK(MLinks_Mon(0)->h_valid == 1U && MLinks_Mon(1)->h_valid == 1U, "their hellos arrive too");
    CHECK(ph->state == MPH_OK && ph->have_ang != 0U, "the sequence is measured: OK");
    page1(); CHECK(line_v(0, "R-Y") >= 396 && line_v(0, "R-Y") <= 401 && line_v(1, "Y-B") >= 396 && line_v(1, "Y-B") <= 401, "and the phase-to-phase voltages are on the Run screen (R-Y %d, Y-B %d V)", line_v(0, "R-Y"), line_v(1, "Y-B"));
    MLinks_DisplayMon(0U, &v); CHECK(v.last.phase == 'Y', "the monitor page names the Y port 'Y'"); MLinks_DisplayMon(1U, &v); CHECK(v.last.phase == 'B', "and the B port 'B'");
    /* Industrial with the sequence check, same boards */
    g_phase_seq0 = 1; setup((uint8_t)M_MODE_INDUSTRIAL); loop(15000U); o = MApp_Engine(&A);
    CHECK(MApp_Phase(&A)->state == MPH_OK && o->contactor[0] == 1U, "Industrial with Phase Sequence on and three identical boards: measured OK and it starts");
    /* a Y board that is lost, and comes back */
    setup((uint8_t)M_MODE_DOMESTIC); loop(4000U); SM[0].silent = 1U; loop(4000U); o = MApp_Engine(&A);
    CHECK(o->contactor[1] == 0U && o->contactor[0] == 1U && o->contactor[2] == 1U, "the Y board goes silent: only the Y unit trips");
    SM[0].silent = 0U; SM[0].state = 0U; SM[0].run_seen = 0U; loop(60000U); o = MApp_Engine(&A); CHECK(o->contactor[1] == 1U, "and recovers");
    g_lettersR0 = 0;
    printf("  identical boards: all three say R, the PORT is the identity: Domestic and Industrial run, the sequence and line voltages work, the monitor page names the port's phase\n");
}


/* ======================= v0.9.54: PER-PHASE RE-TUNE (R, Y, B, or all three one by one) over each phase's own link ======================= */
static void retune(uint8_t sel) { A.ui.retune_sel = sel; A.ui.req |= MUI_REQ_RETUNE; }
static void stop_all(void) { K[2] = 1U; loop(150U); K[2] = 0U; loop(1500U); D.state = 0U; loop(100U); }
static int fr_row_has(char fr[4][21], int r, const char *t) { return strstr(fr[r], t) != NULL; }
static void t_retune_phase(void)
{
    char fr[4][21]; const MFOut_t *o; uint32_t k;
    /* Domestic, Re-tune Y: the TUNE goes on the Y link only, the screen says Y, the units stay off, Y's new profile is fetched */
    g_kind0 = 0; g_phase_seq0 = 0; g_noseq0 = 0; setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); stop_all();
    { uint8_t h0 = SM[0].n_hello_rx, h1 = SM[1].n_hello_rx;
    retune(2U); loop(300U); MApp_TuneScreen(&A, T, fr);
    CHECK(SM[0].n_tune == 1U && SM[1].n_tune == 0U && r_n_tune == 0U, "Re-tune Y: TUNE on the Y link only (R %u, Y %u, B %u)", (unsigned)r_n_tune, (unsigned)SM[0].n_tune, (unsigned)SM[1].n_tune);
    CHECK((A.tune_id & 0x80U) != 0U && SM[0].max_aux_id < 128U, "its id is 128..255 (%u) and the link's own hello / SEQ ids stay below 128 (%u): the slave never takes one for a retry of the other", (unsigned)A.tune_id, (unsigned)SM[0].max_aux_id);
    CHECK(MApp_ShowsTune(&A) && fr_row_has(fr, 0, "Transformer Tuning") && fr[0][19] == 'Y', "the tuning screen names the phase: '%s'", fr[0]);
    loop(1200U); MApp_TuneScreen(&A, T, fr); o = MApp_Engine(&A);
    CHECK(fr_row_has(fr, 1, "Step") && o->contactor[0] == 0U && o->contactor[1] == 0U && o->contactor[2] == 0U, "its steps arrive over the Y link ('%s') and every unit is held off", fr[1]);
    loop(1500U); MApp_TuneScreen(&A, T, fr);
    CHECK(fr_row_has(fr, 0, "Tuning Complete") && fr[0][19] == 'Y' && fr_row_has(fr, 3, "Press any key"), "the result: '%s' / '%s'", fr[0], fr[3]);
    { loop(600U);
      CHECK(SM[0].n_hello_rx > h0 && SM[1].n_hello_rx == h1 && D.yb[0].prof_cap == 1200U, "the Y slave (only) is asked for its hello again and its new profile arrives (cap %u)", (unsigned)D.yb[0].prof_cap); } }
    MApp_ProfilePage(&A, 3000U, fr); CHECK(fr_row_has(fr, 0, "Transformer Prof. Y") && fr_row_has(fr, 1, "Cap 1200"), "the profile page shows Y's profile in its turn ('%s' / '%s')", fr[0], fr[1]);
    MApp_ProfilePage(&A, 0U, fr); CHECK(fr_row_has(fr, 0, "Transformer Prof. R") && fr_row_has(fr, 1, "No profile"), "and R's in its own ('%s' / '%s')", fr[0], fr[1]);
    MApp_ProfilePage(&A, 6000U, fr); CHECK(fr_row_has(fr, 0, "Transformer Prof. B"), "and B's ('%s')", fr[0]);
    /* a second Re-tune of Y whose id would come out the same as the first (the counter wrapped by 128): it must still be executed, not taken for a retry */
    { uint8_t first = A.tune_id, n0 = SM[0].n_tune; K[0] = 1U; loop(150U); K[0] = 0U; loop(300U);
      A.k_id = (uint8_t)(first & 0x7FU); retune(2U); loop(300U);
      CHECK(SM[0].n_tune == (uint8_t)(n0 + 1U) && A.tune_id != first, "a second Y Re-tune with the same id coming up (%u): a different id is used (%u) and the slave executes it (%u tunes)", (unsigned)first, (unsigned)A.tune_id, (unsigned)SM[0].n_tune);
      loop(3000U); K[0] = 1U; loop(150U); K[0] = 0U; loop(300U); }
    /* Industrial, Re-tune B */
    setup((uint8_t)M_MODE_INDUSTRIAL); loop(15000U); stop_all(); retune(3U); loop(4000U); MApp_TuneScreen(&A, T, fr);
    CHECK(SM[1].n_tune == 1U && SM[0].n_tune == 0U && r_n_tune == 0U && fr_row_has(fr, 0, "Tuning Complete") && fr[0][19] == 'B', "Industrial, Re-tune B: on the B link only, '%s'", fr[0]);
    /* ALL, ONE BY ONE: R, then Y, then B, never two at once */
    setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); stop_all(); retune(4U);
    { int seen_next = 0; for (k = 0U; k < 400U; k++) { loop(100U); if (MApp_ShowsTune(&A)) { MApp_TuneScreen(&A, T, fr); if (fr_row_has(fr, 3, "Next: Y Phase")) seen_next |= 1; if (fr_row_has(fr, 3, "Next: B Phase")) seen_next |= 2; } }
      CHECK(r_n_tune == 1U && SM[0].n_tune == 1U && SM[1].n_tune == 1U, "All: each phase tuned once (R %u, Y %u, B %u)", (unsigned)r_n_tune, (unsigned)SM[0].n_tune, (unsigned)SM[1].n_tune);
      CHECK(g_max_tuning == 1 && g_n_order == 3 && g_order[0] == 0 && g_order[1] == 1 && g_order[2] == 2, "in the order R, Y, B and NEVER two stages at once (most at once %d)", g_max_tuning);
      CHECK(seen_next == 3, "between them the screen says what comes next ('Next: Y Phase...', 'Next: B Phase...': %d)", seen_next); }
    CHECK(D.prof_cap == 1111U && D.yb[0].prof_cap == 1200U && D.yb[1].prof_cap == 1201U && !MApp_ShowsTune(&A), "all three profiles stored and fetched (%u %u %u), the result screen gone by itself", (unsigned)D.prof_cap, (unsigned)D.yb[0].prof_cap, (unsigned)D.yb[1].prof_cap);
    /* All, Y fails: B is never tuned */
    setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); stop_all(); SM[0].tune_fail = 1U; retune(4U);
    { int failed_y = 0; for (k = 0U; k < 200U; k++) { loop(100U); if (MApp_ShowsTune(&A)) { MApp_TuneScreen(&A, T, fr); if (fr_row_has(fr, 0, "Tuning Failed") && fr[0][19] == 'Y' && fr_row_has(fr, 3, "Press any key")) failed_y = 1; } }
      CHECK(r_n_tune == 1U && SM[0].n_tune == 1U && SM[1].n_tune == 0U && failed_y, "All with Y failing: R tuned, Y failed and said so, B NOT started (B %u)", (unsigned)SM[1].n_tune); }
    /* All, the result of R dismissed by a key: the chain ends */
    setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); stop_all(); retune(4U); loop(2600U); K[0] = 1U; loop(150U); K[0] = 0U; loop(10000U);
    CHECK(r_n_tune == 1U && SM[0].n_tune == 0U, "All: a key on R's result ends it there (Y %u)", (unsigned)SM[0].n_tune);
    /* STOP during the Y sweep: TABORT on the Y link */
    setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); stop_all(); retune(2U); loop(1200U); K[2] = 1U; loop(150U); K[2] = 0U; loop(500U); MApp_TuneScreen(&A, T, fr);
    CHECK(SM[0].n_abort == 1U && r_n_abort == 0U && fr_row_has(fr, 0, "Tuning Stopped") && fr[0][19] == 'Y', "STOP during Y's sweep: TABORT on the Y link (%u), '%s'", (unsigned)SM[0].n_abort, fr[0]);
    /* an answer on the wrong link never starts a phase's sweep */
    setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); stop_all(); retune(1U); loop(10U);
    CHECK(A.tune_state == 1U, "(R asked)"); MApp_OnKResultPh(&A, 1U, A.tune_id, 0U, 0); MApp_OnKResultPh(&A, 2U, A.tune_id, 0U, 0); CHECK(A.tune_state == 1U, "R's TUNE answered on the Y or B link: ignored");
    MApp_OnKResultPh(&A, 0U, A.tune_id, 0U, 0); CHECK(A.tune_state == 2U, "answered on the R link: it runs");
    setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); stop_all(); SM[0].silent = 1U; loop(1000U); retune(2U); loop(10U);
    CHECK(A.tune_state == 0U && SM[0].n_tune == 0U, "Re-tune Y with the Y link silent: refused ('Slave Link Lost'), nothing sent"); SM[0].silent = 0U;
    setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); stop_all();
    retune(2U); loop(10U); { uint8_t id = A.tune_id; CHECK(A.tune_state == 1U && (id & 0x80U) != 0U, "(Y asked)"); MApp_OnKResult(&A, id, 0U, 0); CHECK(A.tune_state == 1U, "Y's TUNE answered on the R link: ignored"); }
    /* the Re-tune latch: only the tuned phase's is cleared */
    setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); SM[0].fault = 18U; SM[0].state = 3U; SM[1].fault = 18U; SM[1].state = 3U; loop(3000U);
    CHECK(A.eng.u[1].st == (uint8_t)U_LATCHED && A.eng.u[2].st == (uint8_t)U_LATCHED, "Y and B latch 'Re-tune Required' (%u %u)", (unsigned)A.eng.u[1].st, (unsigned)A.eng.u[2].st);
    retune(3U); loop(4000U);
    CHECK(SM[1].n_tune == 1U && A.eng.u[2].st != (uint8_t)U_LATCHED && A.eng.u[1].st == (uint8_t)U_LATCHED, "Re-tune B from its latch: B's latch is cleared, Y's stays (%u %u)", (unsigned)A.eng.u[1].st, (unsigned)A.eng.u[2].st);
    /* Single: the choice is always R */
    setup((uint8_t)M_MODE_SINGLE); loop(8000U); stop_all(); retune(2U); loop(300U);
    CHECK(r_n_tune == 1U && SM[0].n_tune == 0U, "Single: a Re-tune always goes to R (R %u, Y %u)", (unsigned)r_n_tune, (unsigned)SM[0].n_tune);
    printf("  per-phase Re-tune: each phase on its own link, ids apart, screens name the phase, All = R then Y then B never together, a failure or a key ends it, STOP aborts the right one, latches per phase, profiles per phase\n");
}

/* ======================= v0.9.55: PER-PHASE LOAD AND VOLTAGE CALIBRATION, the default Overload Current 25 A ======================= */
static void press_log(void) { loop(MUI_MSG_MS + 100U); K[3] = 1U; loop(150U); K[3] = 0U; loop(200U); }      /* (a key while a message is up only dismisses it) */
static void t_percal(void)
{
    const MFOut_t *o;
    /* Y has no load reading until ITS Slave Load Cal is done */
    g_kind0 = 0; g_phase_seq0 = 0; g_noseq0 = 0; setup((uint8_t)M_MODE_DOMESTIC); A.settings.scroll = 0U; SM[0].iout_pk = 300U; SM[1].iout_pk = 300U; loop(15000U);
    CHECK(A.settings.ovl_current == 25U, "the default Overload Current is 25 A (%u)", (unsigned)A.settings.ovl_current);
    CHECK(strstr(F[1], "Y:") != NULL && strstr(F[1], "--.-A") != NULL, "Y not calibrated: no load reading ('%s')", F[1]);
    /* the Slave Load Cal page of Y: the zero (00.0 with the unit running and no load), then the gain at a known load */
    A.ui.svc = 9U; A.ui.cal_ph = 1U; loop(100U); A.ui.cal_val = 0U; press_log();
    CHECK(A.settings.sl_zero_yb[0] == 300U && A.settings.sl_zero_ca == 0U && A.settings.sl_zero_yb[1] == 0U, "the Y zero is Y's peak at no load (%u), R's and B's are untouched", (unsigned)A.settings.sl_zero_yb[0]);
    SM[0].iout_pk = 1300U; loop(500U); A.ui.cal_val = 100U; press_log();
    CHECK(A.settings.sl_gain_yb[0] == 1000U && A.settings.sl_gain == 0U && A.settings.sl_gain_yb[1] == 0U, "a 10.0 A load 10 A of peak above the zero: Y's gain 1.000 (%u), R's and B's untouched", (unsigned)A.settings.sl_gain_yb[0]);
    { MStore_t chk; MSettings_t c; (void)chk; c = A.settings; CHECK(MS_Validate(&c, 0) == MS_OK, "and the settings are valid"); }
    A.ui.svc = 0U; A.ui.page = 0U; loop(1500U);
    CHECK(strstr(F[1], "10.0A") != NULL && strstr(F[2], "--.-A") != NULL, "the Run page: Y shows 10.0 A, B still none ('%s' / '%s')", F[1], F[2]);
    /* the overload of Y trips Y only */
    SM[0].iout_pk = 300U + 2600U; loop(5000U); o = MApp_Engine(&A); CHECK(o->contactor[1] == 1U, "26 A on Y: not yet (Overload Cut Delay 10 s)");
    loop(7000U); o = MApp_Engine(&A);
    CHECK(o->contactor[1] == 0U && o->contactor[0] == 1U && o->contactor[2] == 1U && o->fault[1] == MLOG_T_OVERLOAD, "26 A on Y for 10 s: Y trips on overload, R and B run (%u %u %u, fault %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2], (unsigned)o->fault[1]);
    /* a clipped or missing reading is not a load reading */
    setup((uint8_t)M_MODE_DOMESTIC); A.settings.scroll = 0U; A.settings.sl_zero_yb[1] = 300U; A.settings.sl_gain_yb[1] = 1000U; SM[1].iout_pk = 0U; loop(15000U);
    CHECK(strstr(F[2], "--.-A") != NULL, "B calibrated but its slave sends no current: no load reading ('%s')", F[2]);
    SM[1].iout_pk = 800U; loop(1500U); CHECK(strstr(F[2], "5.0A") != NULL, "and with one: 5.0 A ('%s')", F[2]);
    SM[1].iout_pk = 4000U; SM[1].iout_clip = 1U; loop(14000U); o = MApp_Engine(&A);
    CHECK(strstr(F[2], "--.-A") != NULL && o->contactor[2] == 1U, "a CLIPPED reading (37 A) is not a load reading: --.-A and no overload trip on it ('%s')", F[2]); SM[1].iout_clip = 0U;
    /* voltage calibration of Y: the CALV goes on the Y link, its answer comes back on it */
    setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); stop_all();
    A.ui.svc = 5U; A.ui.cal_ph = 1U; loop(100U); A.ui.cal_val = 2312U; press_log();
    CHECK(SM[0].n_cal == 1U && SM[0].last_cal_cmd == (uint8_t)LINKCMD_CALV && SM[0].last_cal_num == 2312 && SM[1].n_cal == 0U && r_n_cal == 0U, "I/P Calibration Y: CALV 231.2 V on the Y link only (Y %u, B %u, R %u)", (unsigned)SM[0].n_cal, (unsigned)SM[1].n_cal, (unsigned)r_n_cal);
    CHECK(A.vcal_wait == 0U && (A.vcal_id & 0x80U) != 0U, "and its answer on the Y link ends the wait (id %u, 128..255)", (unsigned)A.vcal_id);
    A.ui.svc = 6U; A.ui.cal_ph = 2U; loop(100U); A.ui.cal_val = 0U; press_log();
    CHECK(SM[1].n_cal == 1U && SM[1].last_cal_cmd == (uint8_t)LINKCMD_CALR && r_n_cal == 0U, "O/P Calibration B, 000.0: CALR on the B link");
    A.ui.svc = 5U; A.ui.cal_ph = 0U; loop(100U); A.ui.cal_val = 2300U; press_log();
    CHECK(r_n_cal == 1U && SM[0].n_cal == 1U && SM[1].n_cal == 1U, "I/P Calibration R: on the R link, as before");
    /* the Y slave running: refused; the answer on the wrong link is ignored */
    setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); A.ui.svc = 5U; A.ui.cal_ph = 1U; loop(100U); A.ui.cal_val = 2312U; press_log();
    CHECK(SM[0].n_cal == 0U, "Y running: 'Stop The Unit First', nothing sent");
    loop(MUI_MSG_MS + 100U); stop_all(); SM[0].silent = 1U; loop(1000U); A.ui.svc = 5U; A.ui.cal_ph = 1U; loop(100U); A.ui.cal_val = 2312U; press_log();
    CHECK(SM[0].n_cal == 0U && A.vcal_wait == 0U, "Y silent: 'Slave Link Lost', nothing sent"); SM[0].silent = 0U; loop(2000U);
    A.ui.svc = 5U; A.ui.cal_ph = 1U; loop(MUI_MSG_MS + 100U); A.ui.cal_val = 2312U; SM[0].silent = 1U; K[3] = 1U; loop(150U); K[3] = 0U; loop(200U); SM[0].silent = 0U;      /* the Y slave does not answer this one: the answers below are injected */
    if (A.vcal_wait != 0U) { uint8_t id = A.vcal_id; MApp_OnKResult(&A, id, 0U, 10050); CHECK(A.vcal_wait == 1U, "Y's calibration answered on the R link: ignored"); MApp_OnKResultPh(&A, 2U, id, 0U, 10050); CHECK(A.vcal_wait == 1U, "or on the B link: ignored"); MApp_OnKResultPh(&A, 1U, id, 0U, 10050); CHECK(A.vcal_wait == 0U, "on the Y link: taken"); }
    else CHECK(0, "(the Y calibration should be waiting)");
    printf("  per-phase calibration: Y and B load from their own Slave Load Cal (zero, gain, overload trip of that phase only, no reading without one), I/P and O/P calibration on the chosen phase's link, refusals, answers on the wrong link ignored, Overload Current 25 A\n");
}

/* ======================= v0.9.57: A MISSING PHASE trips at once; Industrial says it in the middle of the screen; no "Slave Waits" in three phases ======================= */
static uint32_t until_open(int c, uint32_t max_ms) { uint32_t t0 = T; while ((T - t0) < max_ms) { loop(10U); if (MApp_Engine(&A)->contactor[c] == 0U) return T - t0; } return 999999U; }
static void t_phase_missing(void)
{
    const MFOut_t *o; uint32_t dt;
    /* Industrial, the Y phase goes (its slave goes silent with it) */
    g_kind0 = 0; g_phase_seq0 = 0; g_noseq0 = 0; setup((uint8_t)M_MODE_INDUSTRIAL); loop(15000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "(Industrial running)");
    SM[0].silent = 1U; dt = until_open(0, 5000U); o = MApp_Engine(&A);
    CHECK(dt <= 700U && o->shown == MLOG_T_PHASE_MISS, "Industrial, Y goes: the unit opens in %u ms (the link timeout, no Volt Cut Delay) and shows Phase Missing (%u)", (unsigned)dt, (unsigned)o->shown);
    loop(200U); CHECK(strstr(F[1], "PHASE MISSING") != NULL && strstr(F[2], "Y Phase") != NULL && strspn(F[0], " ") == 20U && strspn(F[3], " ") == 20U, "the screen: 'PHASE MISSING' / 'Y Phase' in the middle, nothing else ('%s' / '%s')", F[1], F[2]);
    { MLogRec_t r; CHECK(MLog_Count(&A.log) >= 1U && MLog_Get(&A.log, 0U, &r) == 0 && r.type == MLOG_T_PHASE_MISS && r.phase == 1U, "logged: Phase Missing, phase Y"); }
    SM[0].silent = 0U; loop(30000U); o = MApp_Engine(&A); CHECK(o->contactor[0] == 1U && o->shown != MLOG_T_PHASE_MISS, "Y back: it restarts by itself");
    /* Y still heard but its input collapsed: tripped in about 0.2 s, not after the 5 s Volt Cut Delay */
    SM[0].vin = 500U; dt = until_open(0, 6000U); o = MApp_Engine(&A);
    CHECK(dt <= 400U && o->shown == MLOG_T_PHASE_MISS, "Y's input at 50 V: open in %u ms, Phase Missing", (unsigned)dt);
    SM[0].vin = 2300U; loop(30000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "(back)");
    SM[0].vin = 1000U; dt = until_open(0, 3000U); CHECK(dt <= 400U && MApp_Engine(&A)->shown == MLOG_T_PHASE_MISS, "Y at 100 V with R and B at 230 V (under half): Phase Missing in %u ms", (unsigned)dt);
    SM[0].vin = 2300U; loop(30000U);
    SM[0].vin = 500U; loop(100U); SM[0].vin = 2300U; loop(2000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "a 100 ms dip to 50 V is not a missing phase");
    SM[0].vin = 1800U; loop(3000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U && MApp_Engine(&A)->shown != MLOG_T_PHASE_MISS, "Y at 180 V (over half): not missing (the I/P limits judge it, after their delay)"); SM[0].vin = 2300U; loop(2000U);
    /* the B phase missing at power-up: never starts, says so */
    setup((uint8_t)M_MODE_INDUSTRIAL); SM[1].silent = 1U; loop(15000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 0U && o->run[0] == 0U && o->shown == MLOG_T_PHASE_MISS && strstr(F[1], "PHASE MISSING") != NULL && strstr(F[2], "B Phase") != NULL, "B missing from the start: nothing runs, 'PHASE MISSING' / 'B Phase' ('%s' / '%s')", F[1], F[2]);
    SM[1].silent = 0U; loop(20000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "B comes: it starts");
    setup((uint8_t)M_MODE_INDUSTRIAL); SM[1].vin = 1000U; loop(15000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 0U && o->run[0] == 0U && o->shown == MLOG_T_PHASE_MISS, "B at 100 V from the start (above I/P LV 90 V, under half of R and Y): it does not start, Phase Missing (%u)", (unsigned)o->shown);
    CHECK(MLog_Count(&A.log) == 0U && SM[0].n_m > 0U && SM[0].run_seen == 0U, "and it never even TRIES (no start-and-trip cycles in the log: %u entries)", (unsigned)MLog_Count(&A.log));
    /* Domestic: only the missing phase's unit, and its row says it */
    setup((uint8_t)M_MODE_DOMESTIC); loop(15000U); SM[0].silent = 1U; dt = until_open(1, 3000U); o = MApp_Engine(&A);
    CHECK(dt <= 700U && o->contactor[0] == 1U && o->contactor[2] == 1U && o->fault[1] == MLOG_T_PHASE_MISS, "Domestic, Y goes: only Y opens (%u ms), R and B run", (unsigned)dt);
    loop(300U); CHECK(strstr(F[1], "Y : PH MISSING") != NULL, "and the status page's Y row says it ('%s')", F[1]);
    SM[0].silent = 0U; loop(1500U); CHECK(MApp_Engine(&A)->contactor[1] == 0U && strstr(F[1], "Y : PH MISSING") != NULL, "Y back: during the restart wait its row still names the trip ('%s')", F[1]);
    loop(30000U); CHECK(MApp_Engine(&A)->contactor[1] == 1U, "Y back: Y restarts");
    /* the Y and B rows of the status page show their own faults now */
    SM[1].vin = 3050U; loop(1500U); CHECK(strstr(F[2], "B : IP HV Appear") != NULL, "B's input over the limit: 'B : IP HV Appear' on its row ('%s')", F[2]); SM[1].vin = 2300U;
    /* Single: never */
    setup((uint8_t)M_MODE_SINGLE); loop(10000U); SM[0].silent = 1U; SM[1].silent = 1U; loop(3000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "Single mode: the Y and B links mean nothing");
    setup((uint8_t)M_MODE_SINGLE); loop(10000U); D.vin_dv = 1000U; loop(1000U); o = MApp_Engine(&A); CHECK(o->contactor[0] == 1U && o->shown != MLOG_T_PHASE_MISS, "Single, R at 100 V with Y and B links alive at 230 V: no Phase Missing (only the I/P limits, after their delay)"); D.vin_dv = 2300U;
    /* no "Slave Waits" in the three-phase modes (the R slave's start-up wait showed as 'Slave Waits: R') */
    setup((uint8_t)M_MODE_DOMESTIC); D.state = 1U; D.wait_valid = 1U; D.wait_ctx = 0x40U; { uint32_t k; int saw = 0; for (k = 0U; k < 100U; k++) { loop(100U); if (strstr(A.status, "Slave Waits") != NULL) saw = 1; } CHECK(saw == 0, "Domestic: never 'Slave Waits'"); }
    setup((uint8_t)M_MODE_SINGLE); D.wait_valid = 1U; D.wait_ctx = 0x40U; { uint32_t k; int saw = 0; for (k = 0U; k < 100U; k++) { D.state = 1U; loop(100U); if (strstr(A.status, "Slave Waits") != NULL) saw = 1; } CHECK(saw == 1, "Single still says what its slave waits for"); }
    printf("  missing phase: a silent slave or a collapsed input trips at once (Industrial all, Domestic that phase), the screen says which in the middle, logged, restarts; Y/B rows show their faults; no 'Slave Waits' in three phases\n");
}
int main(void)
{
    t_module(); t_r_link(); t_loop(); t_sequence_loop(); t_all_r_boards(); t_retune_phase(); t_percal(); t_phase_missing();
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
