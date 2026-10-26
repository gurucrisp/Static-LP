/* Tests of the slave's version-2 link logic (slave/Core/Src/slave_link2.c), driven by FAKE hooks. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "slave_link2.h"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- fake hooks ---- */
static int      n_ctrl; static LinkM_t last_m; static uint32_t last_ctrl_now;
static LinkE_t  e_fill;  static LinkH_t h_fill;
static int      tx_full;
static char     cap[96][100]; static int cap_len[96]; static int ncap;

static void h_on_control(const LinkM_t *m, uint32_t now) { n_ctrl++; last_m = *m; last_ctrl_now = now; }
static void h_fill_e(LinkE_t *e) { *e = e_fill; }
static void h_fill_h(LinkH_t *h) { *h = h_fill; }
static uint8_t h_tx(const char *f, uint32_t n)
{
    if (tx_full || ncap >= 96 || n >= sizeof cap[0]) return 0U;
    memcpy(cap[ncap], f, n); cap[ncap][n] = '\0'; cap_len[ncap] = (int)n; ncap++;
    return 1U;
}
static const SL2_Hooks_t HOOKS = { h_on_control, h_fill_e, h_fill_h, h_tx, 0, 0, 0, 0, 0 };
/* what the slave's real glue does when NO profile is stored: it sets the 7 original fields and leaves the optional profile fields alone */
static void h_fill_h_partial(LinkH_t *h) { h->fw = 939UL; h->vin_min_dv = 800U; h->vin_max_dv = 3000U; h->rst = 0U; h->tuned = 0U; h->vout_max_dv = 2800U; }
static const SL2_Hooks_t HOOKS_PARTIAL = { h_on_control, h_fill_e, h_fill_h_partial, h_tx, 0, 0, 0, 0, 0 };
static void dirty_stack(void) { volatile uint8_t junk[3072]; unsigned i; for (i = 0U; i < sizeof junk; i++) junk[i] = 0xA5U; }

static void reset_world(uint32_t t0)
{
    n_ctrl = 0; ncap = 0; tx_full = 0;
    memset(&e_fill, 0, sizeof e_fill); memset(&h_fill, 0, sizeof h_fill);
    e_fill.setv_dv = 2300U; e_fill.flags2 = 0x0003U; e_fill.clr = 3U; e_fill.reach_lo_dv = 2020U; e_fill.reach_hi_dv = 2700U;
    h_fill.fw = 900UL; h_fill.vin_min_dv = 800U; h_fill.vin_max_dv = 3100U; h_fill.rst = 0U; h_fill.tuned = 1U; h_fill.vout_max_dv = 3200U;
    SL2_Init(&HOOKS, t0);
}

static uint32_t mk(char *out, const char *body)
{
    static const char hexd[] = "0123456789ABCDEF";
    uint32_t n = 0U, i, bl = (uint32_t)strlen(body); uint8_t cs = Link_Checksum(body, bl);
    out[n++] = '$'; for (i = 0U; i < bl; i++) out[n++] = body[i];
    out[n++] = '*'; out[n++] = hexd[cs >> 4]; out[n++] = hexd[cs & 15U]; out[n++] = '\r'; out[n++] = '\n'; out[n] = '\0';
    return n;
}
static void feed(const char *s, uint32_t n, uint32_t now) { uint32_t i; for (i = 0U; i < n; i++) SL2_Rx((uint8_t)s[i], now); }
static void feed_body(const char *body, uint32_t now) { char l[160]; uint32_t n = mk(l, body); feed(l, n, now); }
static int parse_cap(int idx, LinkV2Frame_t *f) { return (int)LinkV2_Parse(cap[idx], (uint32_t)cap_len[idx], f); }
static int count_type(LinkV2Type_t t) { int i, c = 0; LinkV2Frame_t f; for (i = 0; i < ncap; i++) if (parse_cap(i, &f) == LINK_OK && f.type == t) c++; return c; }

/* ------------------------------------------------------------------------------------------------------------------ */
static void t_hello_boot(void)
{
    LinkV2Frame_t f; uint32_t t;
    reset_world(1000U);
    SL2_Service(1000U);
    CHECK(count_type(LINKV2_H) == 1, "the first hello goes out at once (%d)", count_type(LINKV2_H));
    CHECK(parse_cap(0, &f) == LINK_OK && f.type == LINKV2_H && f.u.h.proto == 2U && f.u.h.fw == 900UL && f.u.h.vin_min_dv == 800U && f.u.h.vin_max_dv == 3100U && f.u.h.tuned == 1U && f.u.h.vout_max_dv == 3200U, "the hello carries the hook's values and protocol 2");
    for (t = 1001U; t < 1100U; t += 7U) SL2_Service(t);
    CHECK(count_type(LINKV2_H) == 1, "no second hello before 100 ms (%d)", count_type(LINKV2_H));
    SL2_Service(1100U);
    CHECK(count_type(LINKV2_H) == 2, "the second hello at 100 ms (%d)", count_type(LINKV2_H));
    SL2_Service(1200U);
    CHECK(count_type(LINKV2_H) == 3, "the third at 200 ms (%d)", count_type(LINKV2_H));
    for (t = 1201U; t < 3000U; t += 13U) SL2_Service(t);
    CHECK(count_type(LINKV2_H) == 3, "and no more by themselves (%d)", count_type(LINKV2_H));
    printf("  hello: three at start, 100 ms apart, no more by itself\n");
}

static void t_extended(void)
{
    LinkV2Frame_t f; uint32_t t; int i, e_idx[40], ne = 0;
    reset_world(5000U);
    SL2_Service(5000U);                                         /* the boot hello */
    for (t = 5001U; t <= 6050U; t += 5U)
    {
        int before = ncap;
        SL2_Service(t);
        for (i = before; i < ncap; i++) if (parse_cap(i, &f) == LINK_OK && f.type == LINKV2_E && ne < 40) e_idx[ne++] = i;
    }
    CHECK(ne == 10, "ten $E in the first 1050 ms (%d)", ne);
    CHECK(parse_cap(e_idx[0], &f) == LINK_OK && f.u.e.seq == 0U && f.u.e.setv_dv == 2300U && f.u.e.flags2 == 3U && f.u.e.clr == 3U && f.u.e.reach_lo_dv == 2020U && f.u.e.reach_hi_dv == 2700U, "the first $E carries the hook's values");
    for (i = 0; i < ne; i++) CHECK(parse_cap(e_idx[i], &f) == LINK_OK && f.u.e.seq == (uint8_t)i, "$E number %d has seq %d", i, i);
    /* the first $E is not before 100 ms */
    reset_world(9000U); SL2_Service(9000U); SL2_Service(9090U);
    CHECK(count_type(LINKV2_E) == 0, "no $E before 100 ms");
    SL2_Service(9100U);
    CHECK(count_type(LINKV2_E) == 1, "the first $E at 100 ms");
    /* a full queue: the frame is lost but its seq is spent, so the master sees the gap */
    ncap = 0; tx_full = 1; SL2_Service(9200U); tx_full = 0;
    CHECK(count_type(LINKV2_E) == 0, "nothing queued while full");
    SL2_Service(9300U);
    CHECK(count_type(LINKV2_E) == 1 && parse_cap(ncap - 1, &f) == LINK_OK && f.u.e.seq == 2U, "the next $E carries seq 2: the lost one used seq 1 (got %u)", (unsigned)f.u.e.seq);
    CHECK(SL2_Stats()->tx_dropped >= 1U, "the lost frame is counted");
    printf("  extended status: every 100 ms, seq per attempt, a lost frame leaves a gap\n");
}

static void t_control(void)
{
    char l[160]; uint32_t n; int k;
    reset_world(100U);
    feed_body("M,41,1,2300,3,300", 2222U);
    CHECK(n_ctrl == 1 && last_m.seq == 41U && last_m.run == 1U && last_m.setv_dv == 2300U && last_m.clr == 3U && last_m.maxc_dv == 300U && last_ctrl_now == 2222U, "a valid $M calls on_control once with its fields and the time (calls %d)", n_ctrl);
    feed_body("M,42,1,999,3,300", 3000U);
    CHECK(n_ctrl == 1, "setv 99.9 V is out of range: not delivered");
    feed_body("M,42,2,2300,3,300", 3000U);
    CHECK(n_ctrl == 1, "run 2: not delivered");
    n = mk(l, "M,42,1,2300,3,300"); l[n - 4U] ^= 1;                    /* corrupt a checksum digit */
    feed(l, n, 3000U);
    CHECK(n_ctrl == 1, "a bad checksum: not delivered");
    feed("garbage \x01\x02 $M,4", 20U, 3000U);
    feed_body("M,43,0,2400,4,0", 3100U);
    CHECK(n_ctrl == 2 && last_m.run == 0U && last_m.setv_dv == 2400U && last_m.maxc_dv == 0U, "a good one after garbage and a lost tail is delivered");
    for (k = 0; k < 200; k++) feed_body("M,1,1,2300,0,300", 4000U + (uint32_t)k);
    CHECK(n_ctrl == 202, "200 more arrive (%d)", n_ctrl);
    CHECK(SL2_Stats()->m_ok == 202U, "m_ok counts the valid ones (%u)", (unsigned)SL2_Stats()->m_ok);
    CHECK(SL2_Stats()->rx_bad >= 3U, "the rejected ones are counted bad (%u)", (unsigned)SL2_Stats()->rx_bad);
    feed_body("Z,1,2,3", 5000U);
    CHECK(SL2_Stats()->rx_unknown == 1U, "an unknown frame type is counted as unknown, not bad (%u)", (unsigned)SL2_Stats()->rx_unknown);
    printf("  $M: delivered with its fields and the time; out-of-range, bad-checksum and garbage are not\n");
}

static void t_commands(void)
{
    LinkV2Frame_t f; int before; uint32_t t = 10000U;
    reset_world(t);
    SL2_Service(t); SL2_Service(t + 100U); SL2_Service(t + 200U);   /* the three boot hellos out of the way */
    ncap = 0; t += 300U;
    feed_body("K,5,HELLO", t);
    SL2_Service(t);
    CHECK(ncap >= 2 && parse_cap(0, &f) == LINK_OK && f.type == LINKV2_R && f.u.r.id == 5U && f.u.r.status == 0U, "HELLO is answered with R,5,0 FIRST");
    CHECK(parse_cap(1, &f) == LINK_OK && f.type == LINKV2_H, "and then the hello frame itself");
    /* the master's retry with the same id: answered again, NOT executed again */
    before = ncap;
    feed_body("K,5,HELLO", t + 200U);
    SL2_Service(t + 200U);
    {   int i, r5 = 0, first_after = -1;
        for (i = before; i < ncap; i++) { if (parse_cap(i, &f) == LINK_OK && f.type == LINKV2_R && f.u.r.id == 5U && f.u.r.status == 0U) { r5++; if (first_after < 0) first_after = i; } }
        CHECK(r5 == 1 && first_after == before, "a repeated id is answered again with the same answer, as the first frame of the pass ...");
    }
    CHECK(count_type(LINKV2_H) == 1, "... and the hello is NOT sent a second time (%d)", count_type(LINKV2_H));
    CHECK(SL2_Stats()->k_repeat == 1U, "the repeat is counted");
    /* a new id executes again */
    before = ncap; feed_body("K,6,HELLO", t + 400U); SL2_Service(t + 400U);
    CHECK(count_type(LINKV2_H) == 2, "a new id sends the hello again (%d)", count_type(LINKV2_H));
    /* every other command: UNKNOWN_CMD (status 6) in this firmware version */
    ncap = 0;
    feed_body("K,7,TUNE", t + 500U); feed_body("K,8,CALV,2302", t + 500U); feed_body("K,9,FROBNICATE", t + 500U); feed_body("K,10,TPARAM,CAP,1000", t + 500U);
    SL2_Service(t + 500U);
    {   int i, ok = 1; static const int ids[4] = { 7, 8, 9, 10 };
        for (i = 0; i < 4; i++) { if (!(parse_cap(i, &f) == LINK_OK && f.type == LINKV2_R && f.u.r.id == (uint8_t)ids[i] && f.u.r.status == LINKV2_ST_UNKNOWN_CMD)) ok = 0; }
        CHECK(ok, "TUNE, CALV, an unknown word and TPARAM are all answered UNKNOWN_CMD with their own ids, in order"); }
    /* a command is never executed by being repeated, whatever it is */
    printf("  $K: answered at once; a repeated id is answered again but never executed twice; the rest answered UNKNOWN_CMD\n");
}

static void t_queues(void)
{
    LinkV2Frame_t f; int i; uint32_t t = 20000U;
    reset_world(t);
    SL2_Service(t); SL2_Service(t + 100U); SL2_Service(t + 200U);
    ncap = 0; t += 300U;
    for (i = 1; i <= 6; i++) { char b[40]; snprintf(b, sizeof b, "K,%d,TUNE", 20 + i); feed_body(b, t); }
    SL2_Service(t);
    CHECK(count_type(LINKV2_R) == 4 && SL2_Stats()->r_dropped == 2U, "of six commands at once, four are answered and two overflow the response queue and are counted (%d answered, %u dropped)", count_type(LINKV2_R), (unsigned)SL2_Stats()->r_dropped);
    CHECK(parse_cap(0, &f) == LINK_OK && f.u.r.id == 21U, "the oldest are the ones answered (first id %u)", (unsigned)f.u.r.id);
    /* a full transmit queue holds the response back until there is room */
    ncap = 0; tx_full = 1;
    feed_body("K,40,TUNE", t + 100U);
    SL2_Service(t + 100U); SL2_Service(t + 105U); SL2_Service(t + 110U);
    CHECK(count_type(LINKV2_R) == 0, "nothing goes out while the queue is full");
    tx_full = 0; SL2_Service(t + 115U);
    CHECK(count_type(LINKV2_R) == 1 && parse_cap(0, &f) == LINK_OK && f.u.r.id == 40U, "the response goes out once there is room, exactly once");
    SL2_Service(t + 116U); SL2_Service(t + 117U);
    CHECK(count_type(LINKV2_R) == 1, "and is not sent again");
    printf("  queues: response overflow counted, a full transmit queue holds a response back and releases it exactly once\n");
}

static void t_wrap(void)
{
    /* The comparison "has this time been reached" must survive the wrap of the 32-bit millisecond counter. A plain >= only misbehaves when a frame is scheduled
     * shortly BEFORE the wrap and another pass comes before the counter itself wraps, so ONE alignment can hide it: try many (1 ms passes, 40 start offsets). */
    uint32_t off; int worst_e_lo = 999, worst_e_hi = 0, bad_h = 0; LinkV2Frame_t f; int i;
    for (off = 0U; off < 400U; off += 10U)
    {
        uint32_t k, t0 = 0xFFFFFE00U + off; int e_total = 0, h_total = 0;
        reset_world(t0);
        for (k = 0U; k < 2000U; k++) { SL2_Service(t0 + k); if (ncap > 80) ncap = 0; }      /* a COUNT, not "t < t0 + 2000": that sum wraps */
        /* the capture buffer was recycled, so count from the module's own statistics */
        e_total = (int)SL2_Stats()->e_sent; h_total = (int)SL2_Stats()->h_sent;
        if (e_total < worst_e_lo) worst_e_lo = e_total;
        if (e_total > worst_e_hi) worst_e_hi = e_total;
        if (h_total != 3) bad_h++;
    }
    CHECK(bad_h == 0, "exactly three hellos in every one of 40 alignments across the wrap (%d wrong)", bad_h);
    CHECK(worst_e_lo >= 19 && worst_e_hi <= 20, "the $E cadence is 19 or 20 in 2 s in every alignment across the wrap (lowest %d, highest %d)", worst_e_lo, worst_e_hi);
    /* a $K across the wrap */
    reset_world(0xFFFFFFF0U); SL2_Service(0xFFFFFFF0U); ncap = 0;
    feed_body("K,3,HELLO", 0xFFFFFFFAU); SL2_Service(0x00000004U);
    {   int r = 0; for (i = 0; i < ncap; i++) if (parse_cap(i, &f) == LINK_OK && f.type == LINKV2_R) r++;
        CHECK(r == 1, "a command received just before the wrap is answered just after it"); }
    printf("  wrap: hellos and the $E cadence are right in 40 different alignments across the 32-bit millisecond wrap (E per 2 s: %d..%d)\n", worst_e_lo, worst_e_hi);
}

static uint32_t rs = 777U;
static uint32_t rnd(void) { rs = (rs * 1664525U) + 1013904223U; return rs >> 8; }

static void t_fuzz(void)
{
    uint32_t i; static const char al[] = "0123456789,,$$**ABCDEFKMHRTSVOZ\r\n";
    reset_world(1U);
    for (i = 0U; i < 2000000U; i++)
    {
        uint8_t b = ((rnd() & 3U) == 0U) ? (uint8_t)(rnd() & 255U) : (uint8_t)al[rnd() % (sizeof al - 1U)];
        SL2_Rx(b, i);
        if ((i % 1000U) == 0U) { ncap = 0; SL2_Service(i); }
    }
    CHECK(n_ctrl == (int)SL2_Stats()->m_ok, "every delivery is a counted valid $M (%d vs %u)", n_ctrl, (unsigned)SL2_Stats()->m_ok);
    CHECK(n_ctrl < 20, "random bytes (with a 1-in-256 chance per corrupted frame to pass the checksum) hardly ever deliver a control frame (%d of 2,000,000 bytes)", n_ctrl);
    /* NULL hooks must not crash */
    {   SL2_Hooks_t none = { 0, 0, 0, 0, 0, 0, 0, 0, 0 }; char l[160]; uint32_t n; SL2_Init(&none, 5U); n = mk(l, "M,1,1,2300,0,300"); feed(l, n, 5U); n = mk(l, "K,1,HELLO"); feed(l, n, 5U); SL2_Service(10U); SL2_Service(500U); }
    CHECK(1, "NULL hooks do not crash");
    printf("  fuzz: 2,000,000 random bytes: no crash, deliveries all counted and rare; NULL hooks are safe\n");
}



/* ---- v0.9.51: $K SEQ arms the phase sequence measurement ---- */
static int seq_calls; static uint8_t seq_round; static uint32_t seq_now;
static void h_seq_arm(uint8_t round, uint32_t now) { seq_calls++; seq_round = round; seq_now = now; }
static int fe_min_calls, fe_min_clean;
static void dirty_big(void) { volatile uint8_t junk[65536]; unsigned i; for (i = 0U; i < sizeof junk; i++) junk[i] = 0xA5U; }
static void h_fill_e_min(LinkE_t *e) { fe_min_calls++; if ((e->sq_id == 0U) && (e->sq_delay_us == 0U) && (e->sq_period_us == 0U)) fe_min_clean++; e->setv_dv = 2300U; }                              /* sets almost nothing: whatever the module did not initialise would show */
static const SL2_Hooks_t HOOKS_SEQ = { h_on_control, h_fill_e, h_fill_h, h_tx, 0, 0, 0, 0, h_seq_arm };
static void t_seq(void)
{
    LinkV2Frame_t f; int i, found = 0;
    reset_world(1000U); SL2_Init(&HOOKS_SEQ, 1000U); seq_calls = 0; ncap = 0;
    feed_body("K,5,SEQ,12", 2000U); SL2_Service(2000U);
    CHECK(seq_calls == 1 && seq_round == 12U && seq_now == 2000U, "$K,5,SEQ,12 calls the hook once with the round 12 and the time (%d, %u, %u)", seq_calls, (unsigned)seq_round, (unsigned)seq_now);
    for (i = 0; i < ncap; i++) if (parse_cap(i, &f) == LINK_OK && f.type == LINKV2_R && f.u.r.id == 5U) { found = 1; CHECK(f.u.r.status == LINKV2_ST_OK, "and is answered OK (status %u)", (unsigned)f.u.r.status); }
    CHECK(found, "with an $R for id 5");
    feed_body("K,5,SEQ,12", 2010U); SL2_Service(2010U);
    CHECK(seq_calls == 1, "the master's retry of the same id is answered again but NOT executed twice (%d)", seq_calls);
    feed_body("K,6,SEQ,13", 2100U); SL2_Service(2100U); CHECK(seq_calls == 2 && seq_round == 13U, "the next id is a new round (13)");
    feed_body("K,7,SEQ,255", 2200U); SL2_Service(2200U); CHECK(seq_calls == 3 && seq_round == 255U, "round 255 is accepted");
    feed_body("K,8,SEQ,0", 2300U); SL2_Service(2300U); feed_body("K,9,SEQ,256", 2310U); SL2_Service(2310U); feed_body("K,10,SEQ", 2320U); SL2_Service(2320U);
    CHECK(seq_calls == 3, "round 0, round 256 and no round at all are rejected by the parser: the hook is not called (%d)", seq_calls);
    /* a build without the measurement: UNKNOWN_CMD */
    reset_world(1000U); ncap = 0; feed_body("K,5,SEQ,12", 2000U); SL2_Service(2000U); found = 0;
    for (i = 0; i < ncap; i++) if (parse_cap(i, &f) == LINK_OK && f.type == LINKV2_R && f.u.r.id == 5U) { found = 1; CHECK(f.u.r.status == LINKV2_ST_UNKNOWN_CMD, "a build without the hook answers UNKNOWN_CMD (status %u)", (unsigned)f.u.r.status); }
    CHECK(found, "(the answer was sent)");
    /* a build whose fill_e hook does NOT set the sequence fields (no SLAVE_SEQ_SYNC): the $E frame must carry no garbage in them, whatever was on the stack */
    { static const SL2_Hooks_t HOOKS_PART = { h_on_control, h_fill_e_min, h_fill_h, h_tx, 0, 0, 0, 0, 0 }; int idx = -1, k;
      SL2_Init(&HOOKS_PART, 1000U); ncap = 0; fe_min_calls = 0; fe_min_clean = 0; dirty_big(); SL2_Service(1000U); dirty_big(); SL2_Service(1200U); dirty_big(); SL2_Service(1400U); dirty_big(); SL2_Service(1600U);
      CHECK(fe_min_calls >= 3 && fe_min_clean == fe_min_calls, "the structure the link layer hands to fill_e has the sequence fields ZEROED (%d of %d calls clean, after 64 KB of dirtied stack)", fe_min_clean, fe_min_calls);
      for (k = 0; k < ncap; k++) if (parse_cap(k, &f) == LINK_OK && f.type == LINKV2_E) idx = k;
      CHECK(idx >= 0 && parse_cap(idx, &f) == LINK_OK && f.u.e.sq_id == 0U && f.u.e.sq_delay_us == 0U && f.u.e.sq_period_us == 0U && f.u.e.iout_rms_ca == 0U, "a fill_e hook that leaves the sequence fields alone: the $E carries zeros there, not stack garbage (sq_id %u)", (unsigned)f.u.e.sq_id);
      { int commas = 0, j; for (j = 0; j < cap_len[idx]; j++) if (cap[idx][j] == ',') commas++; CHECK(commas == 6, "and is the 6 field frame of old (%d commas)", commas); } }
    printf("  sequence command: $K SEQ <round> calls the hook once (retries answered, not repeated), 1..255 only, UNKNOWN_CMD without the hook\n");
}

/* ---- voltage calibration over $K: started, answered later, retries not executed twice ---- */
static int cal_calls, cal_polls, cal_immediate; static uint8_t cal_imm_status, cal_st, last_cmd; static int32_t cal_val, last_num; static char last_letter; static uint32_t cal_t0;
static int h_cal_cmd(uint8_t cmd, int32_t num, char letter, uint8_t *st, int32_t *val, uint32_t now)
{
    cal_calls++; last_cmd = cmd; last_num = num; last_letter = letter;
    if (cal_immediate) { *st = cal_imm_status; *val = 0; return 1; }
    cal_t0 = now; return 0;
}
static int h_cal_poll(uint8_t *st, int32_t *val, uint32_t now) { cal_polls++; if ((uint32_t)(now - cal_t0) >= 1000U) { *st = cal_st; *val = cal_val; return 1; } return 0; }
static const SL2_Hooks_t HOOKS_CAL = { h_on_control, h_fill_e, h_fill_h, h_tx, h_cal_cmd, h_cal_poll, 0, 0, 0 };
static int count_r(uint8_t id, int *status, int32_t *value) { int i, c = 0; LinkV2Frame_t f; for (i = 0; i < ncap; i++) if (parse_cap(i, &f) == LINK_OK && f.type == LINKV2_R && f.u.r.id == id) { c++; *status = f.u.r.status; *value = f.u.r.value; } return c; }

static void t_calibration(void)
{
    uint32_t t = 20000U; int st = -1; int32_t val = 0;
    reset_world(t); SL2_Init(&HOOKS_CAL, t); cal_calls = cal_polls = 0; cal_immediate = 0; cal_st = 0U; cal_val = 10152;
    SL2_Service(t); SL2_Service(t + 100U); SL2_Service(t + 200U); ncap = 0; t += 300U;
    /* started: the hook gets the command and the number; NO answer yet */
    feed_body("K,5,CALV,2302", t); SL2_Service(t);
    CHECK(cal_calls == 1 && last_cmd == (uint8_t)LINKCMD_CALV && last_num == 2302, "$K CALV 2302 reaches the hook once with its number (calls %d, cmd %u, num %d)", cal_calls, last_cmd, (int)last_num);
    CHECK(count_r(5, &st, &val) == 0, "and there is NO answer while the averaging runs");
    SL2_Service(t + 500U); CHECK(count_r(5, &st, &val) == 0, "still none at 0.5 s");
    /* the master's retry of the same id while it is pending: not executed again, not answered yet */
    feed_body("K,5,CALV,2302", t + 600U); SL2_Service(t + 600U);
    CHECK(cal_calls == 1 && count_r(5, &st, &val) == 0, "a retry of the id that is being worked on is NOT executed again and not answered yet (calls %d)", cal_calls);
    SL2_Service(t + 1000U);
    CHECK(count_r(5, &st, &val) == 1 && st == (int)LINKV2_ST_OK && val == 10152, "after 1 s the answer comes: R,5,0,10152 (found %d, status %d, value %d)", count_r(5, &st, &val), st, (int)val);
    /* a retry after it was answered (the master missed the answer): the SAME answer again, still one execution */
    ncap = 0; feed_body("K,5,CALV,2302", t + 1300U); SL2_Service(t + 1300U);
    CHECK(count_r(5, &st, &val) == 1 && st == (int)LINKV2_ST_OK && val == 10152 && cal_calls == 1, "a retry after the answer gets the same answer again and is not executed twice");
    /* another calibration while one is averaging: NOT_ALLOWED at once, the running one is not disturbed */
    ncap = 0; t += 2000U; feed_body("K,6,CALO,2250", t); SL2_Service(t); ncap = 0;
    feed_body("K,7,CALV,2302", t + 100U); SL2_Service(t + 100U);
    CHECK(count_r(7, &st, &val) == 1 && st == (int)LINKV2_ST_NOT_ALLOWED && cal_calls == 2, "a command while one is averaging: R,7,7 (NOT_ALLOWED) at once, the hook is not called again (status %d, calls %d)", st, cal_calls);
    ncap = 0; SL2_Service(t + 1000U); CHECK(count_r(6, &st, &val) == 1 && st == (int)LINKV2_ST_OK, "the running one (id 6) still finishes");
    /* an answer that is ready at once (NOT_STOPPED, BAD_ARG...) goes out in the same pass */
    reset_world(t); SL2_Init(&HOOKS_CAL, t); cal_calls = 0; cal_immediate = 1; cal_imm_status = (uint8_t)LINKV2_ST_NOT_STOPPED;
    SL2_Service(t); SL2_Service(t + 100U); SL2_Service(t + 200U); ncap = 0; t += 300U;
    feed_body("K,9,CALV,2302", t); SL2_Service(t);
    CHECK(count_r(9, &st, &val) == 1 && st == (int)LINKV2_ST_NOT_STOPPED, "an immediate refusal is answered in the same pass: R,9,4");
    /* CALR carries its letter */
    cal_immediate = 1; cal_imm_status = 0U; ncap = 0; feed_body("K,10,CALR,O", t + 100U); SL2_Service(t + 100U);
    CHECK(last_cmd == (uint8_t)LINKCMD_CALR && last_letter == 'O' && count_r(10, &st, &val) == 1 && st == 0, "CALR O reaches the hook with its letter");
    /* a build without a calibration hook: UNKNOWN_CMD, as before */
    reset_world(t); SL2_Service(t); SL2_Service(t + 100U); SL2_Service(t + 200U); ncap = 0; feed_body("K,11,CALV,2302", t + 300U); SL2_Service(t + 300U);
    CHECK(count_r(11, &st, &val) == 1 && st == (int)LINKV2_ST_UNKNOWN_CMD, "without the hooks CALV is UNKNOWN_CMD");
    printf("  $K calibration: started and answered after the averaging, a retry is never executed twice, another command meanwhile is NOT_ALLOWED, immediate refusals at once\n");
}

int main(void)
{
    t_hello_boot();
    t_seq();
    {   /* v0.9.39: a hook that does not set the optional profile fields must not make the hello disappear (they were stack garbage, the builder refused the frame) */
        LinkV2Frame_t f; int k, ok = 0;
        reset_world(5000U); SL2_Init(&HOOKS_PARTIAL, 5000U); dirty_stack(); SL2_Service(5000U);
        for (k = 0; k < ncap; k++) if (parse_cap(k, &f) == LINK_OK && f.type == LINKV2_H) ok = 1;
        CHECK(ok && count_type(LINKV2_H) == 1, "a hook that leaves the profile fields alone (no profile stored), on a dirty stack: the hello is still sent (%d)", count_type(LINKV2_H));
        CHECK(ok && f.u.h.prof_cap == 0U && f.u.h.prof_kib == 0U && f.u.h.prof_kiu == 0U && f.u.h.prof_vref_dv == 0U && f.u.h.prof_maxc_dv == 0U && f.u.h.prof_n == 0U && f.u.h.prof_knee == 0U && f.u.h.fw == 939UL, "and its profile fields read as 0");
        CHECK(ok && strchr(cap[0], 'H') != NULL && ((int)strlen(cap[0]) < 60), "and it is the short 7-field frame of the first protocol (%d bytes)", (int)strlen(cap[0]));
        SL2_Init(&HOOKS, 6000U);
    }
    t_extended();
    t_control();
    t_commands();
    t_calibration();
    t_queues();
    t_wrap();
    t_fuzz();
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
