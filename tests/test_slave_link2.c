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
static const SL2_Hooks_t HOOKS = { h_on_control, h_fill_e, h_fill_h, h_tx };

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
    {   SL2_Hooks_t none = { 0, 0, 0, 0 }; char l[160]; uint32_t n; SL2_Init(&none, 5U); n = mk(l, "M,1,1,2300,0,300"); feed(l, n, 5U); n = mk(l, "K,1,HELLO"); feed(l, n, 5U); SL2_Service(10U); SL2_Service(500U); }
    CHECK(1, "NULL hooks do not crash");
    printf("  fuzz: 2,000,000 random bytes: no crash, deliveries all counted and rare; NULL hooks are safe\n");
}

int main(void)
{
    t_hello_boot();
    t_extended();
    t_control();
    t_commands();
    t_queues();
    t_wrap();
    t_fuzz();
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
