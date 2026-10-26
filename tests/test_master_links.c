/* The Y and B slave links (master_links.c): the decoders of the two links are independent, the data for the application (and the phase-letter check), the $M and hello timing, and a CLOSED LOOP: simulated Y and B slaves send real frames, the application answers with real $M frames. No hardware. */
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
static int e_frame(char *b, uint8_t seq, uint16_t flags2, uint8_t clr)
{
    LinkE_t e; memset(&e, 0, sizeof e); e.seq = seq; e.setv_dv = 2300U; e.flags2 = flags2; e.clr = clr; e.sq_id = g_e_sq_id; e.sq_delay_us = g_e_sq_delay; e.sq_period_us = g_e_sq_period;
    return (int)LinkV2_BuildE(b, 96U, &e);
}
static int h_frame(char *b)
{
    LinkH_t h; memset(&h, 0, sizeof h); h.proto = 2U; h.fw = 948U; h.vin_min_dv = 1400U; h.vin_max_dv = 2500U; h.rst = LINKV2_RST_POWER_ON; h.tuned = 1U; h.vout_max_dv = 2400U;
    return (int)LinkV2_BuildH(b, 96U, &h);
}
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
    /* the phase letter: the wrong slave on a cable counts as nothing heard */
    reset(); n = s_frame(b, 'R', 1U, 2300U, 2300U, 2U, 0U); push(0, b, n); n = e_frame(b, 1U, LINKV2_FLAG_IN_BAND, 3U); push(0, b, n); n = h_frame(b); push(0, b, n);
    feed(0, 1000U); MLinks_Fill(0, &s0, 1020U);
    CHECK(MLinks_PhaseBad(0) == 1U && s0.have == 0U && s0.link_ok == 0U && s0.e_valid == 0U && s0.h_valid == 0U && s0.vin_dv == 0U, "an R slave on the Y cable: PhaseBad, and the application sees NOTHING (not even $E or $H)");
    CHECK(MLinks_HelloDue(0, 1020U) == 0U, "and it is not asked for a hello");
    reset(); n = s_frame(b, 'Y', 1U, 2300U, 2300U, 2U, 0U); push(1, b, n); feed(1, 1000U); MLinks_Fill(1, &s1, 1020U);
    CHECK(MLinks_PhaseBad(1) == 1U && s1.have == 0U, "a Y slave on the B cable: refused too");
    reset(); n = s_frame(b, 'B', 1U, 2300U, 2300U, 2U, 0U); push(1, b, n); feed(1, 1000U); MLinks_Fill(1, &s1, 1020U);
    CHECK(MLinks_PhaseBad(1) == 0U && s1.have == 1U, "a B slave on the B cable: accepted");
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
      CHECK(z.have == 0U && z.vin_dv == 0U && MLinks_BuildM(9, &c, 1U, 0U, b, sizeof b) == 0 && MLinks_HelloDue(9, 0U) == 0U && MLinks_BuildHello(9, b, sizeof b) == 0 && MLinks_PhaseBad(9) == 0U, "an index outside the two links does nothing"); }
    printf("  module: independent decoders, freshness, the phase letter, ring wrap, $M timing and sequence numbers, hello\n");
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
} Sim_t;
static uint16_t raw_of(double tc) { double k = tc + 273.15, r = 10000.0 * exp(3950.0 * (1.0 / k - 1.0 / 298.15)); return (uint16_t)(4095.0 * r / (r + 1500.0) + 0.5); }
static Sim_t SM[2];
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
    if ((uint32_t)(now - s->t_e) >= 100U) { s->t_e = now; g_e_sq_id = s->sq_id; g_e_sq_delay = s->sq_delay; g_e_sq_period = s->sq_period; n = e_frame(b, s->seq, (uint16_t)((s->state == 2U) ? LINKV2_FLAG_IN_BAND : 0U), s->clr_seen); g_e_sq_id = 0U; g_e_sq_delay = 0U; g_e_sq_period = 0U; push((uint8_t)link, b, n); }
    if (s->h_pending != 0U) { s->h_pending--; n = h_frame(b); push((uint8_t)link, b, n); }
}
static void sim_rx(int link, const char *b, int n)       /* the $M and $K frames the master sent to this slave */
{
    Sim_t *s = &SM[link]; LinkV2Frame_t f; int k;
    if (s->silent != 0U) return;
    for (k = 0; k < n; k++)
    {
        int32_t t = LinkV2Rx_Feed(&s->rx, (uint8_t)b[k], &f);
        if (t == (int32_t)LINKV2_M) { s->n_m++; s->run_seen = f.u.m.run; if (f.u.m.clr != s->clr_seen) { s->clr_seen = f.u.m.clr; if (s->state == 3U) { s->fault = 0U; s->state = 0U; } } }
        else if (t == (int32_t)LINKV2_K) { if (f.u.k.cmd == LINKCMD_HELLO) s->h_pending = 1U; else if ((f.u.k.cmd == LINKCMD_SEQ) && (s->no_seq == 0U)) { s->armed = 1U; s->arm_round = (uint16_t)f.u.k.num; } }
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
static void loop(uint32_t ms)
{
    uint32_t e = T + ms;
    while (T < e)
    {
        int i; char b[96]; int32_t n;
        for (i = 0; i < 2; i++) { sim_tick(i, T); feed((uint8_t)i, T); MLinks_Fill((uint8_t)i, &D.yb[i], T); }
        MApp_Step(&A, K[0], K[1], K[2], K[3], &D, T, F);
        seq_actions();
        for (i = 0; i < 2; i++)
        {
            n = MLinks_BuildM((uint8_t)i, MApp_ControlPh(&A, (uint8_t)(1 + i)), MApp_ControlValidPh(&A, (uint8_t)(1 + i)), T, b, sizeof b); if (n > 0) sim_rx(i, b, (int)n);
            if (MLinks_HelloDue((uint8_t)i, T) != 0U) { n = MLinks_BuildHello((uint8_t)i, b, sizeof b); if (n > 0) sim_rx(i, b, (int)n); }
        }
        T += 10U;
    }
}
static int g_kind0 = 0, g_phase_seq0 = 0, g_noseq0 = 0;      /* presets applied before the first pass of the loop in setup() */
static void setup(uint8_t mode)
{
    memset(fm, 0xFF, sizeof fm); memset(&D, 0, sizeof D); memset(K, 0, sizeof K); T = 0U; reset();
    D.have = 1U; D.link_ok = 1U; D.vin_dv = 2300U; D.vout_dv = 2300U; D.freq_chz = 5000U; D.state = 2U; D.e_valid = 1U; D.h_valid = 1U; D.in_band = 1U; D.load_ok = 1U;
    MApp_Init(&A, &FL, 0U, MFLASH_PAGE, 2U * MFLASH_PAGE, 3U * MFLASH_PAGE, T);
    A.settings.mode = mode; A.settings.scroll = 0U; A.settings.phase_seq = (uint8_t)g_phase_seq0; MF_Init(&A.eng, &A.settings, T);
    sim_init(&SM[0], 'Y'); sim_init(&SM[1], 'B'); r_armed = 0U; r_no_seq = (uint8_t)g_noseq0; SM[0].no_seq = (uint8_t)g_noseq0; SM[1].no_seq = (uint8_t)g_noseq0; n_arm = 0; n_sync = 0; g_kind = g_kind0; apply_kind();
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
    /* the wrong letter on the B cable */
    setup((uint8_t)M_MODE_DOMESTIC); SM[1].letter = 'Y'; loop(20000U); o = MApp_Engine(&A);
    CHECK(MLinks_PhaseBad(1) == 1U && o->contactor[0] == 1U && o->contactor[1] == 1U && o->contactor[2] == 0U && SM[1].run_seen == 0U, "a Y slave on the B cable: the B unit never starts and its slave is never told to run (contactors %u %u %u, B told run %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2], (unsigned)SM[1].run_seen);
    /* INDUSTRIAL */
    setup((uint8_t)M_MODE_INDUSTRIAL); loop(15000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && o->contactor[1] == 0U && o->contactor[2] == 0U && SM[0].state == 2U && SM[1].state == 2U, "closed loop, Industrial: all three slaves run and ONE contactor (PA10) closes (%u %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2]);
    SM[0].silent = 1U; loop(4000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 0U && SM[1].run_seen == 0U && SM[1].state == 0U, "the Y cable is cut: the contactor opens and the B slave is told to stop too (run %u, state %u)", (unsigned)SM[1].run_seen, (unsigned)SM[1].state);
    setup((uint8_t)M_MODE_INDUSTRIAL); SM[1].silent = 1U; loop(15000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 0U && SM[0].run_seen == 0U, "Industrial with the B slave never connected: nothing starts");
    printf("  closed loop: simulated Y and B slaves over the real frames: start, cable cut and recovery, inrush trip and START, wrong letter, Industrial\n");
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

int main(void)
{
    t_module(); t_r_link(); t_loop(); t_sequence_loop();
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
