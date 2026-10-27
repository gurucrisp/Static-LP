/* Tests of the master core storage: m_store (two-slot records), m_settings (defaults, validation, pack) and m_log (fault log), over a flash MODEL that can lose power at any point. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "m_settings.h"
#include "m_log.h"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ---- the flash model: 6 pages of 2 KB. prog works on 8-byte words into erased bytes. A power cut after `budget` units (8 per word, 256 per erase) leaves a TORN operation:
 * a program leaves the words before the cut written and the word at the cut half written; an erase leaves the first part blank and the rest garbage (0xA5). After the cut every call fails until restore. */
#define NPAGES 6
static uint8_t  fm[NPAGES * MFLASH_PAGE];
static int32_t  budget;            /* < 0: no cut armed */
static uint8_t  dead;
static uint32_t units_used;

static int f_read(uint32_t a, uint8_t *b, uint32_t n) { if (a + n > sizeof fm) return 1; memcpy(b, &fm[a], n); return 0; }
static int f_prog(uint32_t a, const uint8_t *b, uint32_t n)
{
    uint32_t w;
    if (dead) return 1;
    if (((a | n) & 7U) != 0U || a + n > sizeof fm) return 2;
    for (w = 0U; w < n; w += 8U)
    {
        uint32_t k; int torn = 0;
        if (budget >= 0 && budget < 8) { torn = 1; }
        for (k = 0U; k < 8U; k++)
        {
            if (torn && k >= (uint32_t)(budget > 0 ? budget : 0) && k >= 4U) break;        /* a torn word: at most its first 4 bytes */
            if (torn && k >= (uint32_t)budget) break;
            if (fm[a + w + k] != 0xFFU && b[w + k] != fm[a + w + k]) { printf("  MODEL ERROR: programming a byte that is not erased at 0x%X\n", (unsigned)(a + w + k)); g_fail++; }
            fm[a + w + k] &= b[w + k];
        }
        units_used += 8U;
        if (budget >= 0) { budget -= 8; if (budget < 0) { dead = 1; return 3; } }
        if (torn) { dead = 1; return 3; }
    }
    return 0;
}
static int f_erase(uint32_t a)
{
    uint32_t i, done;
    if (dead) return 1;
    if ((a % MFLASH_PAGE) != 0U || a + MFLASH_PAGE > sizeof fm) return 2;
    done = MFLASH_PAGE;
    if (budget >= 0 && budget < 256) { done = (uint32_t)budget * 8U; }
    for (i = 0U; i < MFLASH_PAGE; i++) fm[a + i] = (i < done) ? 0xFFU : 0xA5U;
    units_used += 256U;
    if (budget >= 0) { budget -= 256; if (budget < 0) { dead = 1; return 3; } }
    return 0;
}
static const MFlash_t FL = { f_read, f_prog, f_erase };
static void flash_blank(void) { memset(fm, 0xFF, sizeof fm); budget = -1; dead = 0; units_used = 0U; }
static void power_restore(void) { budget = -1; dead = 0; }

#define PG(n) ((uint32_t)(n) * MFLASH_PAGE)

/* ------------------------------------------------------------------------------------------------------------------ */
static void t_store_basic(void)
{
    MStore_t s; uint8_t a[32], b[32], r[32]; int i;
    flash_blank();
    for (i = 0; i < 32; i++) { a[i] = (uint8_t)(i + 1); b[i] = (uint8_t)(200 - i); }
    MStore_Init(&s, &FL, PG(0), PG(1), 3U, 32U);
    CHECK(MStore_Load(&s, r) == 0U, "a blank flash holds no record");
    CHECK(MStore_Save(&s, a) == 0 && MStore_Load(&s, r) == 1U && memcmp(r, a, 32) == 0, "save then load round-trips");
    CHECK(MStore_Save(&s, b) == 0 && MStore_Load(&s, r) == 1U && memcmp(r, b, 32) == 0 && s.seq == 2U, "the second save wins (seq %u)", (unsigned)s.seq);
    { MStore_t t; MStore_Init(&t, &FL, PG(0), PG(1), 3U, 32U); CHECK(MStore_Load(&t, r) == 1U && memcmp(r, b, 32) == 0, "a fresh start finds the newest"); }
    { MStore_t t; MStore_Init(&t, &FL, PG(0), PG(1), 4U, 32U); CHECK(MStore_Load(&t, r) == 0U, "another record version is not loaded"); }
    { MStore_t t; MStore_Init(&t, &FL, PG(0), PG(1), 3U, 24U); CHECK(MStore_Load(&t, r) == 0U, "another record length is not loaded"); }
    /* two independent stores in different pages do not touch each other (settings and calibration) */
    { MStore_t c; uint8_t cal[16] = { 9, 8, 7 }; MStore_Init(&c, &FL, PG(2), PG(3), 1U, 16U); MStore_Save(&c, cal);
      MStore_Init(&s, &FL, PG(0), PG(1), 3U, 32U); CHECK(MStore_Load(&s, r) == 1U && memcmp(r, b, 32) == 0, "a second store in other pages leaves the first alone"); }
    /* the sequence wraps correctly: 40 saves */
    MStore_Init(&s, &FL, PG(0), PG(1), 3U, 32U); MStore_Load(&s, r);
    for (i = 0; i < 40; i++) { a[0] = (uint8_t)i; if (MStore_Save(&s, a) != 0) break; }
    CHECK(i == 40 && MStore_Load(&s, r) == 1U && r[0] == 39U, "40 saves in a row, the last one is the one found");
    printf("  store: save/load, newest wins, version and length checked, independent stores, 40 saves\n");
}

static void t_store_powercut(void)
{
    MStore_t s; uint8_t a[32], b[32], c[32], r[32]; int i, cuts, bad = 0, got_a = 0, got_b = 0, none = 0, heal_bad = 0; uint32_t total;
    for (i = 0; i < 32; i++) { a[i] = (uint8_t)(i * 3 + 1); b[i] = (uint8_t)(255 - i * 5); c[i] = (uint8_t)(i ^ 0x5A); }
    /* how many units does one save use? */
    flash_blank(); MStore_Init(&s, &FL, PG(0), PG(1), 1U, 32U); MStore_Save(&s, a); units_used = 0U; MStore_Save(&s, b); total = units_used;
    for (cuts = 0; cuts <= (int)total + 8; cuts += 1)
    {   /* the first save is complete; the SECOND is cut after `cuts` units */
        flash_blank(); MStore_Init(&s, &FL, PG(0), PG(1), 1U, 32U); MStore_Save(&s, a);
        budget = cuts; (void)MStore_Save(&s, b);
        power_restore();
        MStore_Init(&s, &FL, PG(0), PG(1), 1U, 32U);
        if (MStore_Load(&s, r) == 0U) { bad++; continue; }
        if (memcmp(r, a, 32) == 0) got_a++; else if (memcmp(r, b, 32) == 0) got_b++; else bad++;
        /* it heals: the next save works and is what is found */
        if (!(MStore_Save(&s, c) == 0 && MStore_Load(&s, r) == 1U && memcmp(r, c, 32) == 0)) heal_bad++;
    }
    CHECK(bad == 0, "a power cut at any of %u points of the SECOND save leaves the old record or the new one, never anything else (%d bad)", (unsigned)total, bad);
    CHECK(got_a > 0 && got_b > 0, "both outcomes occur: old record (%d) and new record (%d)", got_a, got_b);
    CHECK(heal_bad == 0, "after every cut the next save works and is found (%d failures)", heal_bad);
    /* the very FIRST save cut anywhere: nothing, or the record; never garbage */
    for (cuts = 0, bad = 0, none = 0, got_a = 0; cuts <= (int)total + 8; cuts++)
    {
        flash_blank(); MStore_Init(&s, &FL, PG(0), PG(1), 1U, 32U);
        budget = cuts; (void)MStore_Save(&s, a); power_restore();
        MStore_Init(&s, &FL, PG(0), PG(1), 1U, 32U);
        if (MStore_Load(&s, r) == 0U) none++; else if (memcmp(r, a, 32) == 0) got_a++; else bad++;
    }
    CHECK(bad == 0 && none > 0 && got_a > 0, "the very first save cut anywhere gives nothing or the record (nothing %d, record %d, garbage %d)", none, got_a, bad);
    printf("  store power cuts: %u cut points of a save (and of a first save): never a mixture, always heals\n", (unsigned)total);
}

static void t_store_bitflip(void)
{
    MStore_t s; uint8_t a[32], b[32], r[32]; uint32_t bit, wrong = 0U, tot = 0U, fallback = 0U; int i;
    for (i = 0; i < 32; i++) { a[i] = (uint8_t)(i + 7); b[i] = (uint8_t)(i * 9); }
    flash_blank(); MStore_Init(&s, &FL, PG(0), PG(1), 1U, 32U); MStore_Save(&s, a); MStore_Save(&s, b);         /* b is the newest, in slot 1; a (older, valid) is in slot 0 */
    for (bit = 0U; bit < (8U + 32U + 8U + 8U) * 8U; bit++)
    {
        uint8_t save = fm[PG(1) + bit / 8U];
        fm[PG(1) + bit / 8U] = (uint8_t)(save ^ (1U << (bit % 8U)));
        MStore_Init(&s, &FL, PG(0), PG(1), 1U, 32U);
        tot++;
        if (MStore_Load(&s, r) != 0U) { if (memcmp(r, b, 32) == 0) {} else if (memcmp(r, a, 32) == 0) fallback++; else wrong++; }
        fm[PG(1) + bit / 8U] = save;
    }
    CHECK(wrong == 0U, "flipping any single bit of the newest slot (%u flips) never yields a corrupted record (%u wrong)", (unsigned)tot, (unsigned)wrong);
    CHECK(fallback > 100U, "and the damage falls back to the older valid record (%u times)", (unsigned)fallback);
    printf("  store bit flips: %u single-bit corruptions of the newest slot: none accepted wrong\n", (unsigned)tot);
}

/* ---- settings ---- */
static uint32_t rs = 4242U;
static uint32_t rnd(void) { rs = (rs * 1664525U) + 1013904223U; return rs >> 8; }

static int ms_eq(const MSettings_t *a, const MSettings_t *b)
{
    return a->mode == b->mode && a->set_v == b->set_v && a->maxcorr_v == b->maxcorr_v && a->ip_lv == b->ip_lv && a->ip_hv == b->ip_hv && a->op_lv == b->op_lv && a->op_hv == b->op_hv &&
           a->volt_cut_delay == b->volt_cut_delay && a->ovl_current == b->ovl_current && a->ovl_cut_delay == b->ovl_cut_delay && a->heat_delay == b->heat_delay && a->phase_seq == b->phase_seq &&
           a->scroll == b->scroll && a->scroll_time == b->scroll_time && a->password == b->password && a->restart_auto == b->restart_auto && a->restart_time == b->restart_time &&
           a->load_src == b->load_src && a->sl_zero_ca == b->sl_zero_ca && a->sl_gain == b->sl_gain && a->heat_temp == b->heat_temp && a->heat_reset == b->heat_reset &&
           a->sl_zero_yb[0] == b->sl_zero_yb[0] && a->sl_gain_yb[0] == b->sl_gain_yb[0] && a->sl_zero_yb[1] == b->sl_zero_yb[1] && a->sl_gain_yb[1] == b->sl_gain_yb[1];
}

static void t_settings_validate(void)
{
    MSettings_t d, s; MLimits_t lim = { 80U, 300U, 280U };            /* what the slave announces with the v0.9.17 master-control config */
    MS_Defaults(&d);
    CHECK(MS_Validate(&d, 0) == MS_OK && MS_Validate(&d, &lim) == MS_OK, "the factory defaults are valid (and inside the slave's window and guard)");
    CHECK(d.mode == 0U && d.set_v == 230U && d.ip_lv == 90U && d.ip_hv == 300U && d.op_lv == 110U && d.op_hv == 280U && d.volt_cut_delay == 5U && d.ovl_cut_delay == 10U && d.heat_delay == 10U &&
          d.phase_seq == 0U && d.scroll == 1U && d.scroll_time == 5U && d.password == 0U && d.restart_auto == 1U && d.restart_time == 5U && d.maxcorr_v == 0U && d.ovl_current == MASTER_RATED_CURRENT_A, "the defaults are the agreed ones (Max Correction not set: the window is the whole 90..300 V)");
#define EDGE(field, lo, hi, elo, ehi) do { s = d; s.field = (lo); CHECK(MS_Validate(&s, 0) == MS_OK, #field " minimum %d is valid", (int)(lo)); s.field = (hi); CHECK(MS_Validate(&s, 0) == MS_OK, #field " maximum %d is valid", (int)(hi)); \
        s.field = (lo) - 1; CHECK(MS_Validate(&s, 0) == (elo), #field " below the minimum is refused"); s.field = (hi) + 1; CHECK(MS_Validate(&s, 0) == (ehi), #field " above the maximum is refused"); } while (0)
    EDGE(volt_cut_delay, 1, 99, MS_BAD_VOLT_DELAY, MS_BAD_VOLT_DELAY);
    EDGE(ovl_current, 1, 180, MS_BAD_OVL_CURRENT, MS_BAD_OVL_CURRENT);
    EDGE(ovl_cut_delay, 1, 99, MS_BAD_OVL_DELAY, MS_BAD_OVL_DELAY);
    EDGE(heat_delay, 1, 99, MS_BAD_HEAT_DELAY, MS_BAD_HEAT_DELAY);
    EDGE(scroll_time, 1, 99, MS_BAD_SCROLL_TIME, MS_BAD_SCROLL_TIME);
    EDGE(restart_time, 1, 999, MS_BAD_RESTART_TIME, MS_BAD_RESTART_TIME);
    s = d; s.password = 999U; CHECK(MS_Validate(&s, 0) == MS_OK, "password 999 is valid"); s.password = 1000U; CHECK(MS_Validate(&s, 0) == MS_BAD_PASSWORD, "1000 is not");
    s = d; s.mode = 3U; CHECK(MS_Validate(&s, 0) == MS_BAD_MODE, "mode 3 is refused");
    /* Max Correction: 0 = not set, otherwise 10 to 199 (at least the 10 V margin to the Set Voltage) */
    s = d; s.maxcorr_v = 0U; CHECK(MS_Validate(&s, 0) == MS_OK, "Max Correction 0 (not set) is valid");
    { uint16_t m; int bad = 0; for (m = 1U; m <= 9U; m++) { s = d; s.maxcorr_v = m; if (MS_Validate(&s, 0) != MS_BAD_MAXCORR) bad++; } CHECK(bad == 0, "Max Correction 1 to 9 V is refused: a window narrower than the 10 V margin is impossible"); }
    s = d; s.maxcorr_v = 10U; s.ip_lv = 220U; s.ip_hv = 240U; s.op_lv = 220U; s.op_hv = 240U; CHECK(MS_Validate(&s, 0) == MS_OK, "Max Correction 10 V is valid (the window 220..240 V: every limit sits exactly on the margin)");
    s = d; s.maxcorr_v = 199U; CHECK(MS_Validate(&s, 0) == MS_OK, "Max Correction 199 is valid (the window is cut to 90..300 V)"); s.maxcorr_v = 200U; CHECK(MS_Validate(&s, 0) == MS_BAD_MAXCORR, "200 is not");
    /* the Set Voltage: 100 to 290 (the limits must fit 10 V either side inside 90..300 V) */
    s = d; s.set_v = 99U; s.ip_lv = 90U; s.op_lv = 90U; CHECK(MS_Validate(&s, 0) == MS_BAD_SET_V, "Set Voltage 99 is refused");
    s = d; s.set_v = 100U; s.ip_lv = 90U; s.op_lv = 90U; CHECK(MS_Validate(&s, 0) == MS_OK, "Set Voltage 100 is valid (I/P LV 90 V, O/P LV 90 V: exactly the 10 V margin)");
    s = d; s.set_v = 290U; s.op_hv = 300U; CHECK(MS_Validate(&s, 0) == MS_OK, "Set Voltage 290 is valid (I/P HV 300, O/P HV 300)"); CHECK(MS_Validate(&s, &lim) == MS_BAD_OP_HV, "but not with a slave whose output guard is 280 V (O/P HV would have to be 300)");
    s = d; s.set_v = 291U; CHECK(MS_Validate(&s, 0) == MS_BAD_SET_V, "Set Voltage 291 is refused: the I/P HV could not be 10 V above it inside 300 V");
    /* the window: Set Voltage -/+ Max Correction. Set 230, Max Correction 50: 180..280 */
    s = d; s.maxcorr_v = 50U; s.ip_lv = 180U; s.ip_hv = 280U; s.op_lv = 200U; s.op_hv = 260U;
    CHECK(MS_WinLo(&s) == 180U && MS_WinHi(&s) == 280U && MS_Validate(&s, 0) == MS_OK, "Set 230, Max Correction 50: the window is 180..280 V and the defaults sit in it");
    { MSettings_t t = s; t.ip_lv = 179U; CHECK(MS_Validate(&t, 0) == MS_BAD_IP_LV, "I/P LV below the window (179) is refused"); t.ip_lv = 220U; t.op_lv = 220U; CHECK(MS_Validate(&t, 0) == MS_OK, "I/P LV may be moved up to 10 V below Set (220)"); t.ip_lv = 221U; CHECK(MS_Validate(&t, 0) == MS_BAD_IP_LV, "but not closer (221)"); }
    { MSettings_t t = s; t.ip_hv = 281U; CHECK(MS_Validate(&t, 0) == MS_BAD_IP_HV, "I/P HV above the window (281) is refused"); t.ip_hv = 240U; t.op_hv = 240U; CHECK(MS_Validate(&t, 0) == MS_OK, "I/P HV may be moved down to 10 V above Set (240)"); t.ip_hv = 239U; CHECK(MS_Validate(&t, 0) == MS_BAD_IP_HV, "but not closer (239)"); }
    { MSettings_t t = s; t.op_lv = 179U; CHECK(MS_Validate(&t, 0) == MS_BAD_OP_LV, "O/P LV below I/P LV is refused"); t.op_lv = 220U; CHECK(MS_Validate(&t, 0) == MS_OK, "O/P LV up to 10 V below Set is valid"); t.op_lv = 221U; CHECK(MS_Validate(&t, 0) == MS_BAD_OP_LV, "not closer"); }
    { MSettings_t t = s; t.op_hv = 281U; CHECK(MS_Validate(&t, 0) == MS_BAD_OP_HV, "O/P HV above I/P HV is refused"); t.op_hv = 240U; CHECK(MS_Validate(&t, 0) == MS_OK, "O/P HV down to 10 V above Set is valid"); t.op_hv = 239U; CHECK(MS_Validate(&t, 0) == MS_BAD_OP_HV, "not closer"); }
    s = d; s.ip_lv = 89U; CHECK(MS_Validate(&s, 0) == MS_BAD_IP_LV, "Max Correction not set: I/P LV 89 is below the 90 V the unit works from"); s.ip_lv = 90U; CHECK(MS_Validate(&s, 0) == MS_OK, "90 is valid");
    s = d; s.ip_hv = 301U; CHECK(MS_Validate(&s, 0) == MS_BAD_IP_HV, "I/P HV 301 is above the fixed 300 V maximum"); 
    s = d; s.ip_lv = 230U; CHECK(MS_Validate(&s, 0) == MS_BAD_IP_LV, "I/P LV equal to Set V is refused"); s = d; s.ip_hv = 230U; CHECK(MS_Validate(&s, 0) == MS_BAD_IP_HV, "I/P HV equal to Set V is refused");
    /* the slave's window and guard (D7, D26) on top of that */
    { MLimits_t l2 = { 120U, 300U, 280U }; s = d; s.ip_lv = 119U; s.op_lv = 130U; CHECK(MS_Validate(&s, &l2) == MS_BAD_IP_LV && MS_Validate(&s, 0) == MS_OK, "an I/P LV below the slave's window is refused only when the window is known"); s.ip_lv = 120U; CHECK(MS_Validate(&s, &l2) == MS_OK, "exactly at the window edge is valid"); }
    s = d; s.op_hv = 281U; s.ip_hv = 300U; CHECK(MS_Validate(&s, &lim) == MS_BAD_OP_HV, "an O/P HV above the slave's output guard is refused"); s.op_hv = 280U; CHECK(MS_Validate(&s, &lim) == MS_OK, "exactly at the guard is valid");
    s = d; s.phase_seq = 2U; CHECK(MS_Validate(&s, 0) == MS_BAD_FLAG, "a flag of 2 is refused");
    printf("  settings validation: defaults, every range edge, Max Correction 0 or 10..199, Set Voltage 100..290, the window and the 10 V margins, the slave's window and guard\n");
}

/* the default limits and the rule that a limit at its old default follows to the new one */
static void t_settings_limits_model(void)
{
    static const struct { uint16_t set, mc, ilv, ihv, olv, ohv; } T[] = {
        { 230U,   0U,  90U, 300U, 110U, 280U }, { 230U,  50U, 180U, 280U, 200U, 260U }, { 230U,  40U, 190U, 270U, 210U, 250U }, { 230U,  25U, 205U, 255U, 220U, 240U },
        { 230U,  12U, 218U, 242U, 220U, 240U }, { 230U, 199U,  90U, 300U, 110U, 280U }, { 100U,   0U,  90U, 300U,  90U, 280U }, { 120U,  60U,  90U, 180U, 110U, 160U }, { 290U,  40U, 250U, 300U, 270U, 300U } };
    unsigned i; MSettings_t s, o, n; MLimits_t lim = { 80U, 300U, 280U }; int bad = 0; uint32_t k;
    for (i = 0U; i < sizeof T / sizeof T[0]; i++)
    {
        uint16_t a, b, c, e; MS_Defaults(&s); s.set_v = T[i].set; s.maxcorr_v = T[i].mc; MS_DefaultLimits(&s, 0, &a, &b, &c, &e);
        CHECK(a == T[i].ilv && b == T[i].ihv && c == T[i].olv && e == T[i].ohv, "Set %u Max Correction %u: defaults I/P %u-%u, O/P %u-%u (got %u-%u, %u-%u)", T[i].set, T[i].mc, T[i].ilv, T[i].ihv, T[i].olv, T[i].ohv, a, b, c, e);
        s.ip_lv = a; s.ip_hv = b; s.op_lv = c; s.op_hv = e; CHECK(MS_Validate(&s, 0) == MS_OK, "and the defaults are valid");
    }
    { uint16_t a, b, c, e; MS_Defaults(&s); MS_DefaultLimits(&s, &lim, &a, &b, &c, &e); CHECK(e == 280U, "O/P HV default 280 with a slave guard of 280"); lim.out_guard_v = 265U; MS_DefaultLimits(&s, &lim, &a, &b, &c, &e); CHECK(e == 265U, "kept under a slave guard of 265 V"); lim.out_guard_v = 280U; }
    /* the limits that sit at their defaults follow a change; a limit the user moved stays (clipped into the new window) */
    MS_Defaults(&o); o.maxcorr_v = 50U; { uint16_t a, b, c, e; MS_DefaultLimits(&o, 0, &a, &b, &c, &e); o.ip_lv = a; o.ip_hv = b; o.op_lv = c; o.op_hv = e; }
    n = o; n.set_v = 240U; MS_Rederive(&n, &o, 0);
    CHECK(n.ip_lv == 190U && n.ip_hv == 290U && n.op_lv == 210U && n.op_hv == 270U && MS_Validate(&n, 0) == MS_OK, "Set Voltage 230 -> 240 with every limit at its default: they all follow (I/P 190-290, O/P 210-270) (%u-%u %u-%u)", n.ip_lv, n.ip_hv, n.op_lv, n.op_hv);
    n = o; n.ip_lv = 200U; o = n; n.set_v = 240U; MS_Rederive(&n, &o, 0);        /* (the user moved I/P LV to 200; o is the state before the Set Voltage edit) */
    CHECK(n.ip_lv == 200U && n.ip_hv == 290U && MS_Validate(&n, 0) == MS_OK, "Set 230 -> 240: the I/P LV the user moved to 200 STAYS, the I/P HV that was at its default follows to 290 (%u %u)", n.ip_lv, n.ip_hv);
    MS_Defaults(&o); o.maxcorr_v = 50U; { uint16_t a, b, c, e; MS_DefaultLimits(&o, 0, &a, &b, &c, &e); o.ip_lv = 230U - 10U; o.ip_hv = b; o.op_lv = 220U; o.op_hv = e; }
    n = o; n.maxcorr_v = 20U; MS_Rederive(&n, &o, 0);
    CHECK(n.ip_lv == 220U && MS_Validate(&n, 0) == MS_OK, "Max Correction 50 -> 20: I/P LV moved by the user to 220 stays inside the new window 210..250 (%u)", n.ip_lv);
    n = o; n.set_v = 300U; MS_Rederive(&n, &o, 0); CHECK(MS_Validate(&n, 0) == MS_BAD_SET_V, "Set Voltage 300 is invalid whatever the limits do");
    /* property: from ANY valid set, changing the Set Voltage and/or Max Correction to ANY valid pair and rederiving always gives a valid set */
    for (k = 0U; k < 200000U; k++)
    {
        uint16_t lo, hi;
        MS_Defaults(&o); o.set_v = (uint16_t)(100U + rnd() % 191U); o.maxcorr_v = (rnd() & 1U) ? 0U : (uint16_t)(10U + rnd() % 190U);
        lo = MS_WinLo(&o); hi = MS_WinHi(&o);
        o.ip_lv = (uint16_t)(lo + rnd() % (uint32_t)(o.set_v - 10U - lo + 1U)); o.ip_hv = (uint16_t)(o.set_v + 10U + rnd() % (uint32_t)(hi - o.set_v - 10U + 1U));
        o.op_lv = (uint16_t)(o.ip_lv + rnd() % (uint32_t)(o.set_v - 10U - o.ip_lv + 1U)); o.op_hv = (uint16_t)(o.set_v + 10U + rnd() % (uint32_t)(o.ip_hv - o.set_v - 10U + 1U));
        if (rnd() % 4U == 0U) { uint16_t a, b, c, e; MS_DefaultLimits(&o, 0, &a, &b, &c, &e); o.ip_lv = a; o.ip_hv = b; o.op_lv = c; o.op_hv = e; }       /* often a set that is all defaults */
        if (MS_Validate(&o, 0) != MS_OK) { bad++; continue; }
        n = o; if (rnd() & 1U) n.set_v = (uint16_t)(100U + rnd() % 191U); if (rnd() & 1U) n.maxcorr_v = (rnd() & 1U) ? 0U : (uint16_t)(10U + rnd() % 190U);
        MS_Rederive(&n, &o, 0);
        if (MS_Validate(&n, 0) != MS_OK) bad++;
    }
    CHECK(bad == 0, "200,000 random valid sets and random valid Set Voltage / Max Correction changes: the rederived set is always valid (%d not)", bad);
    printf("  limits model: the default limits for 9 Set Voltage / Max Correction pairs, the follow-the-default rule, 200,000 random changes always valid\n");
}

static void t_settings_store(void)
{
    MStore_t st; MSettings_t a, b, c; uint8_t raw[MS_PACK_LEN]; int i, bad = 0;
    flash_blank(); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
    CHECK(MS_Load(&st, &a) == 0U, "blank flash: defaults are used"); { MSettings_t d; MS_Defaults(&d); CHECK(ms_eq(&a, &d), "and they ARE the defaults"); }
    a.set_v = 240U; a.maxcorr_v = 40U; { uint16_t x1, x2, x3, x4; MS_DefaultLimits(&a, 0, &x1, &x2, &x3, &x4); a.ip_lv = (uint16_t)(x1 + 5U); a.ip_hv = x2; a.op_lv = x3; a.op_hv = x4; } a.mode = 2U; a.restart_auto = 0U; a.password = 321U; a.ovl_current = 120U;
    CHECK(MS_Save(&st, &a) == 0, "save");
    MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
    CHECK(MS_Load(&st, &b) == 1U && b.set_v == 240U && b.ip_lv == 205U && b.mode == 2U && b.restart_auto == 0U && b.password == 321U && b.maxcorr_v == 40U && b.ovl_current == 120U, "the settings come back after a restart");
    c = a; c.set_v = 50U;
    CHECK(MS_Save(&st, &c) != 0, "an invalid set is not written");
    MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN); CHECK(MS_Load(&st, &b) == 1U && b.set_v == 240U, "and the stored one is untouched");
    /* a record that passes its CRC but breaks the rules is not trusted: write one directly */
    MS_Pack(&a, raw); raw[1] = 20U; raw[2] = 0U;                                   /* set voltage 20 */
    { MStore_t t; flash_blank(); MStore_Init(&t, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN); MStore_Save(&t, raw); MStore_Init(&t, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
      CHECK(MS_Load(&t, &b) == 0U && b.set_v == 230U, "a CRC-valid record that fails the rules gives the defaults"); }
    /* pack / unpack: 100,000 random valid sets */
    for (i = 0; i < 100000; i++)
    {
        MSettings_t x, y; uint8_t p[MS_PACK_LEN];
        MS_Defaults(&x); x.mode = (uint8_t)(rnd() % 3U); x.set_v = (uint16_t)(100U + rnd() % 191U); x.maxcorr_v = (rnd() & 1U) ? 0U : (uint16_t)(10U + rnd() % 190U);
        { uint16_t lo = MS_WinLo(&x), hi = MS_WinHi(&x);
          x.ip_lv = (uint16_t)(lo + rnd() % (uint32_t)(x.set_v - 10U - lo + 1U)); x.ip_hv = (uint16_t)(x.set_v + 10U + rnd() % (uint32_t)(hi - x.set_v - 10U + 1U));
          x.op_lv = (uint16_t)(x.ip_lv + rnd() % (uint32_t)(x.set_v - 10U - x.ip_lv + 1U)); x.op_hv = (uint16_t)(x.set_v + 10U + rnd() % (uint32_t)(x.ip_hv - x.set_v - 10U + 1U)); }
        x.volt_cut_delay = (uint8_t)(1U + rnd() % 99U); x.ovl_current = (uint16_t)(1U + rnd() % 180U); x.ovl_cut_delay = (uint8_t)(1U + rnd() % 99U); x.heat_delay = (uint8_t)(1U + rnd() % 99U);
        x.phase_seq = (uint8_t)(rnd() & 1U); x.scroll = (uint8_t)(rnd() & 1U); x.scroll_time = (uint8_t)(1U + rnd() % 99U); x.password = (uint16_t)(rnd() % 1000U); x.restart_auto = (uint8_t)(rnd() & 1U); x.restart_time = (uint16_t)(1U + rnd() % 999U);
        x.load_src = (uint8_t)(rnd() & 1U); x.sl_zero_ca = (uint16_t)(rnd() % 4001U); x.sl_gain = (rnd() & 1U) ? 0U : (uint16_t)(200U + rnd() % 4801U);
        x.heat_temp = (uint8_t)(50U + rnd() % 71U); x.heat_reset = (uint8_t)(35U + rnd() % (uint32_t)(x.heat_temp - 5U - 35U + 1U));
        MS_Pack(&x, p); MS_Unpack(&y, p);
        if (!(MS_Validate(&x, 0) == MS_OK && ms_eq(&x, &y))) bad++;
    }
    CHECK(bad == 0, "100,000 random valid settings survive pack and unpack exactly (%d differ)", bad);
    printf("  settings storage: round trip, invalid never written, a rule-breaking record not trusted, 100,000 pack round trips\n");
}


/* v0.9.45 to v0.9.50: the settings record. Version 2 is 48 bytes: bytes 0 to 31 are laid out EXACTLY as in version 1 (32 bytes, firmware up to v0.9.49), byte 32 is the Overheat Restart. A version 1 record must still load, with every setting carried over. */
static void t_settings_slave_load(void)
{
    MStore_t st, o1; MSettings_t a, b; uint8_t raw[MS_PACK_LEN], def[MS_PACK_LEN], v1[MS_PACK_LEN_V1]; int i, same = 1; uint32_t k;
    MS_Defaults(&a); a.set_v = 235U; a.maxcorr_v = 45U; { uint16_t x1, x2, x3, x4; MS_DefaultLimits(&a, 0, &x1, &x2, &x3, &x4); a.ip_lv = x1; a.ip_hv = x2; a.op_lv = x3; a.op_hv = x4; }
    MS_Pack(&a, def);
    CHECK(def[26] == 0U && def[27] == MS_HEAT_TEMP_DEFAULT && def[28] == 0U && def[29] == 0U && def[30] == 0U && def[31] == 0U && def[32] == 65U, "defaults: bytes 26, 28 to 31 are zero, byte 27 = 75 (Overheat Temp), byte 32 = 65 (Overheat Restart)");
    { int zero = 1; for (k = 33U; k < MS_PACK_LEN; k++) if (def[k] != 0U) zero = 0; CHECK(zero, "bytes 33 to 47 are zero (room for the three-phase calibrations)"); }
    a.load_src = 1U; a.sl_zero_ca = 250U; a.sl_gain = 1282U; a.heat_temp = 90U; a.heat_reset = 80U; MS_Pack(&a, raw);
    for (i = 0; i < 26; i++) if (raw[i] != def[i]) same = 0;
    CHECK(same, "setting the newer fields changes NOTHING in bytes 0 to 25");
    CHECK(raw[26] == 1U && raw[27] == 90U && (raw[28] | (raw[29] << 8)) == 250 && (raw[30] | (raw[31] << 8)) == 1282 && raw[32] == 80U, "Load Source at byte 26, Overheat Temp at 27, the zero at 28 and 29, the gain at 30 and 31, Overheat Restart at 32");
    MS_Unpack(&b, raw); CHECK(b.load_src == 1U && b.sl_zero_ca == 250U && b.sl_gain == 1282U && b.heat_temp == 90U && b.heat_reset == 80U && b.set_v == 235U && b.maxcorr_v == 45U, "and they unpack to the same values");
    /* a save and a load of the new record */
    flash_blank(); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
    CHECK(MS_Save(&st, &a) == 0, "a record with them is saved"); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
    CHECK(MS_Load(&st, &b) == 1U && ms_eq(&a, &b), "and comes back exactly");
    /* ---- MIGRATION: a record as firmware up to v0.9.49 wrote it (version 1, 32 bytes) ---- */
    memcpy(v1, raw, MS_PACK_LEN_V1);                                                      /* the same bytes 0 to 31 */
    flash_blank(); MStore_Init(&o1, &FL, PG(0), PG(1), MS_VERSION_V1, MS_PACK_LEN_V1); CHECK(MStore_Save(&o1, v1) == 0, "a version 1 record (32 bytes) is written, as the older firmware did");
    MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
    CHECK(MS_Load(&st, &b) == 1U && b.set_v == 235U && b.maxcorr_v == 45U && b.load_src == 1U && b.sl_zero_ca == 250U && b.sl_gain == 1282U && b.heat_temp == 90U, "it LOADS: every setting is carried over (set voltage, Max Correction, Load Source, the calibration, Overheat Temp)");
    CHECK(b.heat_reset == 80U && MS_Validate(&b, 0) == MS_OK, "and the Overheat Restart gets its default, 10 degC below the Overheat Temp (%u)", (unsigned)b.heat_reset);
    CHECK(MS_Save(&st, &b) == 0, "the next save writes the new record");
    { MStore_t chk; MSettings_t c; MStore_Init(&chk, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN); CHECK(MS_Load(&chk, &c) == 1U && ms_eq(&b, &c), "which is what a restart then loads");
      MStore_Init(&chk, &FL, PG(0), PG(1), MS_VERSION_V1, MS_PACK_LEN_V1); CHECK(MStore_Load(&chk, v1) == 1U, "and the old record was NOT erased by that save: it went into the other page (a power cut during it would have migrated again)"); }
    /* a record from before v0.9.49: byte 27 was always zero */
    memcpy(v1, raw, MS_PACK_LEN_V1); v1[27] = 0U; v1[26] = 0U; v1[28] = 0U; v1[29] = 0U; v1[30] = 0U; v1[31] = 0U;
    flash_blank(); MStore_Init(&o1, &FL, PG(0), PG(1), MS_VERSION_V1, MS_PACK_LEN_V1); (void)MStore_Save(&o1, v1); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
    CHECK(MS_Load(&st, &b) == 1U && b.set_v == 235U && b.load_src == 0U && b.sl_gain == 0U && b.heat_temp == 75U && b.heat_reset == 65U, "a record from before v0.9.45 loads too: Master CT, not calibrated, Overheat Temp 75, Overheat Restart 65");
    /* a version 1 record that breaks the rules is not trusted */
    memcpy(v1, raw, MS_PACK_LEN_V1); v1[27] = 49U;
    flash_blank(); MStore_Init(&o1, &FL, PG(0), PG(1), MS_VERSION_V1, MS_PACK_LEN_V1); (void)MStore_Save(&o1, v1); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
    CHECK(MS_Load(&st, &b) == 0U && b.set_v == 230U, "a version 1 record with an Overheat Temp of 49 (breaks the rules) gives the defaults");
    /* what is refused */
    flash_blank(); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
    { MSettings_t x; static const struct { int field; uint16_t v; } BAD[] = { {0, 2}, {0, 255}, {1, 4001}, {1, 65535}, {2, 199}, {2, 5001}, {2, 100}, {2, 65535} };
      unsigned q; for (q = 0U; q < sizeof BAD / sizeof BAD[0]; q++) { x = a; if (BAD[q].field == 0) x.load_src = (uint8_t)BAD[q].v; else if (BAD[q].field == 1) x.sl_zero_ca = BAD[q].v; else x.sl_gain = BAD[q].v;
          CHECK(MS_Validate(&x, 0) == MS_BAD_FLAG && MS_Save(&st, &x) != 0, "refused: field %d = %u (%d)", BAD[q].field, (unsigned)BAD[q].v, (int)MS_Validate(&x, 0)); } }
    { static const uint8_t ht[] = { 49U, 121U, 255U }; unsigned q; MSettings_t x;
      for (q = 0U; q < sizeof ht; q++) { x = a; x.heat_temp = ht[q]; CHECK(MS_Validate(&x, 0) == MS_BAD_HEAT_TEMP && MS_Save(&st, &x) != 0, "refused: Overheat Temp %u", (unsigned)ht[q]); }
      x = a; x.heat_temp = 50U; x.heat_reset = 40U; CHECK(MS_Validate(&x, 0) == MS_OK, "Overheat Temp 50 (with a restart of 40) is valid"); x.heat_temp = 120U; x.heat_reset = 110U; CHECK(MS_Validate(&x, 0) == MS_OK, "and 120"); }
    /* THE OVERHEAT RESTART: at least 35, at most the Overheat Temp - 5 */
    { static const struct { uint8_t temp, reset; int ok; } RR[] = {
          {75U, 34U, 0}, {75U, 35U, 1}, {75U, 65U, 1}, {75U, 70U, 1}, {75U, 71U, 0}, {75U, 75U, 0}, {75U, 0U, 0}, {75U, 255U, 0},
          {50U, 35U, 1}, {50U, 45U, 1}, {50U, 46U, 0}, {50U, 34U, 0}, {120U, 115U, 1}, {120U, 116U, 0}, {120U, 35U, 1}, {90U, 85U, 1}, {90U, 86U, 0}, {60U, 55U, 1}, {60U, 56U, 0} };
      unsigned q; MSettings_t x;
      for (q = 0U; q < sizeof RR / sizeof RR[0]; q++)
      {   x = a; x.heat_temp = RR[q].temp; x.heat_reset = RR[q].reset;
          CHECK((MS_Validate(&x, 0) == MS_OK) == (RR[q].ok != 0) && (RR[q].ok != 0 || MS_Validate(&x, 0) == MS_BAD_HEAT_RESET), "Overheat Temp %u, Overheat Restart %u: %s (%d)", (unsigned)RR[q].temp, (unsigned)RR[q].reset, RR[q].ok ? "valid" : "refused", (int)MS_Validate(&x, 0)); }
      { MSettings_t y = a; y.heat_temp = 60U; y.heat_reset = 65U; MS_HeatResetFollow(&y); CHECK(y.heat_reset == 50U && MS_Validate(&y, 0) == MS_OK, "the restart follows a lowered Overheat Temp: 65 with a temperature of 60 becomes 50 (%u)", (unsigned)y.heat_reset);
        y.heat_temp = 60U; y.heat_reset = 55U; MS_HeatResetFollow(&y); CHECK(y.heat_reset == 55U, "55 with 60 already fits: untouched"); y.heat_temp = 60U; y.heat_reset = 56U; MS_HeatResetFollow(&y); CHECK(y.heat_reset == 50U, "56 with 60 does not: 50");
        y.heat_temp = 255U; y.heat_reset = 200U; MS_HeatResetFollow(&y); CHECK(y.heat_reset == 200U, "an Overheat Temp that is itself out of range is left to the validation"); }
      CHECK(MS_HeatResetDefault(75U) == 65U && MS_HeatResetDefault(50U) == 40U && MS_HeatResetDefault(120U) == 110U && MS_HeatResetDefault(45U) == 35U && MS_HeatResetDefault(40U) == 35U, "the default restart is 10 degC below, never below 35 (75 -> %u, 50 -> %u, 45 -> %u)", (unsigned)MS_HeatResetDefault(75U), (unsigned)MS_HeatResetDefault(50U), (unsigned)MS_HeatResetDefault(45U)); }
    { MSettings_t x = a; x.sl_gain = 0U; x.sl_zero_ca = 4000U; CHECK(MS_Validate(&x, 0) == MS_OK, "gain 0 (not calibrated) and a zero of 40.00 A are valid"); x.sl_gain = 200U; CHECK(MS_Validate(&x, 0) == MS_OK, "so are the ends of the gain range, 0.200"); x.sl_gain = 5000U; CHECK(MS_Validate(&x, 0) == MS_OK, "and 5.000"); }
    /* a version 2 record whose bytes break the rules but whose CRC is good is not trusted either */
    memcpy(raw, def, MS_PACK_LEN); raw[26] = 7U; flash_blank(); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN); (void)MStore_Save(&st, raw); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
    CHECK(MS_Load(&st, &b) == 0U && b.load_src == 0U && b.sl_gain == 0U, "a record with Load Source 7 and a good CRC is not trusted: the defaults");
    memcpy(raw, def, MS_PACK_LEN); raw[32] = 34U; flash_blank(); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN); (void)MStore_Save(&st, raw); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
    CHECK(MS_Load(&st, &b) == 0U && b.heat_reset == 65U, "nor one with an Overheat Restart of 34: the defaults");
    /* v0.9.55: the Y and B slave load calibrations in bytes 33 to 40 (zero Y, gain Y, zero B, gain B): no new version, an older version 2 record has zeros there = not calibrated */
    CHECK(MASTER_RATED_CURRENT_A == 25U && a.ovl_current == 25U, "v0.9.55: the default Overload Current is 25 A (%u)", (unsigned)MASTER_RATED_CURRENT_A);
    MS_Pack(&a, raw);                                                                     /* (raw was edited by the checks above) */
    { MSettings_t y = a, z; uint8_t ry[MS_PACK_LEN]; int same2 = 1;
      y.sl_zero_yb[0] = 310U; y.sl_gain_yb[0] = 1300U; y.sl_zero_yb[1] = 4000U; y.sl_gain_yb[1] = 5000U; MS_Pack(&y, ry);
      for (k = 0U; k < 33U; k++) if (ry[k] != raw[k]) same2 = 0;
      CHECK(same2 && (ry[33] | (ry[34] << 8)) == 310 && (ry[35] | (ry[36] << 8)) == 1300 && (ry[37] | (ry[38] << 8)) == 4000 && (ry[39] | (ry[40] << 8)) == 5000, "Y zero at 33-34, Y gain at 35-36, B zero at 37-38, B gain at 39-40; bytes 0 to 32 unchanged");
      { int z2 = 1; for (k = 41U; k < MS_PACK_LEN; k++) if (ry[k] != 0U) z2 = 0; CHECK(z2, "bytes 41 to 47 stay free"); }
      MS_Unpack(&z, ry); CHECK(ms_eq(&y, &z), "they unpack to the same values");
      flash_blank(); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN); CHECK(MS_Save(&st, &y) == 0, "saved"); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
      CHECK(MS_Load(&st, &z) == 1U && ms_eq(&y, &z), "and loaded back exactly");
      /* a version 2 record written by v0.9.50 .. v0.9.54: bytes 33 to 47 zero */
      flash_blank(); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN); (void)MStore_Save(&st, raw); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
      CHECK(MS_Load(&st, &z) == 1U && z.sl_gain == 1282U && z.sl_gain_yb[0] == 0U && z.sl_gain_yb[1] == 0U && z.sl_zero_yb[0] == 0U && z.sl_zero_yb[1] == 0U, "a record from v0.9.50 to v0.9.54 loads: R's calibration kept, Y and B not calibrated");
      memcpy(v1, raw, MS_PACK_LEN_V1); flash_blank(); MStore_Init(&o1, &FL, PG(0), PG(1), MS_VERSION_V1, MS_PACK_LEN_V1); (void)MStore_Save(&o1, v1); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
      CHECK(MS_Load(&st, &z) == 1U && z.sl_gain_yb[0] == 0U && z.sl_gain_yb[1] == 0U, "and a version 1 record migrates with Y and B not calibrated");
      { static const struct { int f; uint16_t v; } BY[] = { {0, 4001}, {1, 199}, {1, 5001}, {2, 4001}, {3, 100}, {3, 65535} }; unsigned q; MSettings_t x;
        for (q = 0U; q < sizeof BY / sizeof BY[0]; q++) { x = y; if (BY[q].f == 0) x.sl_zero_yb[0] = BY[q].v; else if (BY[q].f == 1) x.sl_gain_yb[0] = BY[q].v; else if (BY[q].f == 2) x.sl_zero_yb[1] = BY[q].v; else x.sl_gain_yb[1] = BY[q].v;
            CHECK(MS_Validate(&x, 0) == MS_BAD_FLAG, "refused: Y/B calibration field %d = %u", BY[q].f, (unsigned)BY[q].v); }
        x = y; x.sl_gain_yb[0] = 0U; x.sl_gain_yb[1] = 200U; CHECK(MS_Validate(&x, 0) == MS_OK, "gain 0 (not calibrated) and 0.200 are valid"); }
      memcpy(ry, raw, MS_PACK_LEN); ry[35] = 0x10U; ry[36] = 0x27U; flash_blank(); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN); (void)MStore_Save(&st, ry); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
      CHECK(MS_Load(&st, &z) == 0U, "a record with a Y gain of 10000 and a good CRC is not trusted: the defaults"); }
    printf("  settings record: version 2 (48 bytes), bytes 0 to 31 as before, a version 1 record migrates with every setting, the first save does not erase it, the ranges, Y and B load calibrations at 33 to 40\n");
}

static void t_settings_powercut(void)
{
    MStore_t st; MSettings_t a, b, r; int cuts, bad = 0; uint32_t total;
    MS_Defaults(&a); a.set_v = 225U; MS_Defaults(&b); b.set_v = 245U; b.mode = 1U;
    flash_blank(); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN); MS_Save(&st, &a); units_used = 0U; MS_Save(&st, &b); total = units_used;
    for (cuts = 0; cuts <= (int)total + 8; cuts++)
    {
        flash_blank(); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN); MS_Save(&st, &a);
        budget = cuts; (void)MS_Save(&st, &b); power_restore();
        MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
        if (!(MS_Load(&st, &r) == 1U && ((r.set_v == 225U && r.mode == 0U) || (r.set_v == 245U && r.mode == 1U)))) bad++;
    }
    CHECK(bad == 0, "settings: a power cut at any of %u points of a save leaves the old settings or the new, never defaults by accident (%d bad)", (unsigned)total, bad);
    printf("  settings power cuts: %u cut points: always the old or the new set\n", (unsigned)total);
}

/* ---- the fault log ---- */
static MLogRec_t mk(uint8_t type, uint32_t n)
{
    MLogRec_t r; r.type = type; r.phase = (uint8_t)(n & 3U); r.slave_code = (uint8_t)(n % 20U); r.vin_dv = (uint16_t)(1500U + (n % 1500U)); r.vout_dv = (uint16_t)(2000U + (n % 600U)); r.load_da = (uint16_t)(n % 1800U); r.run_min = n * 7U;
    return r;
}
static int rec_eq(const MLogRec_t *a, const MLogRec_t *b) { return a->type == b->type && a->phase == b->phase && a->slave_code == b->slave_code && a->vin_dv == b->vin_dv && a->vout_dv == b->vout_dv && a->load_da == b->load_da && a->run_min == b->run_min; }

static MLog_t LG;
static void t_log_basic(void)
{
    MLogRec_t r, g; uint32_t n; int bad = 0; uint16_t i;
    flash_blank(); MLog_Init(&LG, &FL, PG(4), PG(5));
    CHECK(MLog_Count(&LG) == 0U && MLog_Get(&LG, 0U, &g) == -1, "an empty log has no entries");
    r = mk(MLOG_T_IP_LV, 1U); CHECK(MLog_Append(&LG, &r) == 0 && MLog_Count(&LG) == 1U && MLog_Get(&LG, 0U, &g) == 0 && rec_eq(&r, &g), "one entry round-trips (type, phase, slave code, voltages, load, running time)");
    r.phase = 3U; r.slave_code = 63U; r.type = MLOG_T_MAINS; r.run_min = 0xFFFFFFFFU; r.vin_dv = 65535U; MLog_Append(&LG, &r); MLog_Get(&LG, 0U, &g);
    CHECK(rec_eq(&r, &g), "the extremes round-trip: phase 3, slave code 63, 4,294,967,295 minutes, 6553.5 V");
    flash_blank(); MLog_Init(&LG, &FL, PG(4), PG(5));
    for (n = 1U; n <= 300U; n++) { MLogRec_t x = mk((uint8_t)(1U + n % 13U), n); if (MLog_Append(&LG, &x) != 0) bad++; }
    CHECK(bad == 0, "300 appends, across the wrap of both pages, all succeed");
    CHECK(MLog_Count(&LG) == 100U, "the log shows 100 entries (%u)", (unsigned)MLog_Count(&LG));
    bad = 0; for (i = 0U; i < 100U; i++) { MLogRec_t want = mk((uint8_t)(1U + (300U - i) % 13U), 300U - i); if (MLog_Get(&LG, i, &g) != 0 || !rec_eq(&want, &g)) bad++; }
    CHECK(bad == 0, "entry 00 is the newest and 99 is 99 older, in order (%d wrong)", bad);
    CHECK(MLog_Get(&LG, 100U, &g) == -1, "there is no entry 100");
    /* a restart finds the same log */
    { MLog_t L2; MLog_Init(&L2, &FL, PG(4), PG(5)); bad = 0; for (i = 0U; i < 100U; i++) { MLogRec_t a2, b2; if (MLog_Get(&LG, i, &a2) != 0 || MLog_Get(&L2, i, &b2) != 0 || !rec_eq(&a2, &b2)) bad++; }
      CHECK(bad == 0 && MLog_Count(&L2) == 100U, "after a restart the same 100 entries are found"); 
      { MLogRec_t y = mk(MLOG_T_FLIP, 999U); MLog_Append(&L2, &y); MLog_Get(&L2, 0U, &g); CHECK(rec_eq(&y, &g), "and the next append continues the order"); } }
    CHECK(MLog_Clear(&LG) == 0 && MLog_Count(&LG) == 0U, "Clear Fault Log empties it");
    { MLogRec_t y = mk(MLOG_T_OVERLOAD, 5U); MLog_Append(&LG, &y); MLog_Get(&LG, 0U, &g); CHECK(MLog_Count(&LG) == 1U && rec_eq(&y, &g), "and it can be used again"); }
    /* at least 128 entries always remain, even right after a page erase: fill exactly to the boundary and one more */
    flash_blank(); MLog_Init(&LG, &FL, PG(4), PG(5));
    for (n = 1U; n <= 128U + 128U + 1U; n++) { MLogRec_t x = mk(MLOG_T_IP_HV, n); MLog_Append(&LG, &x); }
    CHECK(LG.n >= 128U, "right after the older page was erased, at least 128 records remain (%u)", (unsigned)LG.n);
    printf("  log: round trip, 300 appends across the wrap, newest first, restart, clear\n");
}

static void t_log_powercut(void)
{
    /* The points that matter: an empty log, the middle of a page, the LAST slot of a page, the first append after a page is full (it erases the other page), and the wrap onwards.
     * For every cut point of the next append, after a restart the log must read: [the new entry, only if it was written whole] + [every older entry in order], shown up to 100,
     * and the next append must work and be entry 00. A torn entry must never be visible. */
    static const uint32_t FILLS[] = { 0U, 1U, 5U, 127U, 128U, 129U, 200U, 255U, 256U, 257U, 300U };
    uint32_t fill, cuts, bad = 0U, cases = 0U, new_seen = 0U, new_absent = 0U, total = 0U; MLogRec_t g;
    for (fill = 0U; fill < sizeof FILLS / sizeof FILLS[0]; fill++)
    {
        uint32_t nrec = FILLS[fill], n;
        flash_blank(); MLog_Init(&LG, &FL, PG(4), PG(5));
        for (n = 1U; n <= nrec; n++) { MLogRec_t y = mk((uint8_t)(1U + n % 13U), n); MLog_Append(&LG, &y); }
        units_used = 0U; { MLogRec_t y = mk(MLOG_T_HEAT, nrec + 1U); MLog_Append(&LG, &y); } total = units_used;
        for (cuts = 0U; cuts <= total + 8U; cuts++)
        {
            MLog_t L2; MLogRec_t x; uint16_t cnt, i, first, expect; uint32_t shown_bad = 0U;
            flash_blank(); MLog_Init(&LG, &FL, PG(4), PG(5));
            for (n = 1U; n <= nrec; n++) { MLogRec_t y = mk((uint8_t)(1U + n % 13U), n); MLog_Append(&LG, &y); }
            x = mk(MLOG_T_HEAT, nrec + 1U); budget = (int32_t)cuts; (void)MLog_Append(&LG, &x); power_restore();
            MLog_Init(&L2, &FL, PG(4), PG(5)); cnt = MLog_Count(&L2); cases++;
            first = (uint16_t)((cnt > 0U && MLog_Get(&L2, 0U, &g) == 0 && rec_eq(&g, &x)) ? 1U : 0U);
            if (first) new_seen++; else new_absent++;
            expect = (uint16_t)(((nrec + first) > MLOG_SHOWN) ? MLOG_SHOWN : (nrec + first));
            if (cnt != expect) shown_bad++;
            for (i = 0U; i < cnt; i++)
            {
                MLogRec_t want, got;
                if (i < first) want = x; else { uint32_t wn = nrec - (uint32_t)(i - first); want = mk((uint8_t)(1U + wn % 13U), wn); }
                if (MLog_Get(&L2, i, &got) != 0 || !rec_eq(&want, &got)) { shown_bad++; break; }
            }
            { MLogRec_t z = mk(MLOG_T_FLIP, 9999U); if (!(MLog_Append(&L2, &z) == 0 && MLog_Get(&L2, 0U, &g) == 0 && rec_eq(&z, &g))) shown_bad++; }
            if (shown_bad != 0U) { bad++; if (bad <= 4U) printf("    case: %u records before, cut at %u of %u: count %u (expected %u), first=%u\n", (unsigned)nrec, (unsigned)cuts, (unsigned)total, (unsigned)cnt, (unsigned)expect, (unsigned)first); }
        }
    }
    CHECK(bad == 0U, "a power cut at EVERY point of an append: %u cases (empty log, mid page, last slot, page erase, wrap): the old entries intact and in order, the new one whole or absent, the next append works (%u bad)", (unsigned)cases, (unsigned)bad);
    CHECK(new_seen > 0U && new_absent > 0U, "both outcomes occurred: the new entry written (%u) and absent (%u)", (unsigned)new_seen, (unsigned)new_absent);
    printf("  log power cuts: %u cut cases incl. the page-erase append: never a torn entry visible, nothing older lost but the erased page\n", (unsigned)cases);
}

/* A write that FAILS without a restart (a transient flash error): the next append must not reuse the half-written slot (that would program into non-erased bytes). */
static void t_log_transient(void)
{
    uint32_t fill, cuts, bad = 0U, cases = 0U, f0 = (uint32_t)g_fail; MLogRec_t g;
    static const uint32_t FILLS[] = { 0U, 7U, 127U, 128U, 200U };
    for (fill = 0U; fill < sizeof FILLS / sizeof FILLS[0]; fill++)
        for (cuts = 0U; cuts < 40U; cuts += 1U)
        {
            uint32_t n, nrec = FILLS[fill]; MLogRec_t x, z; MLog_t L2; uint16_t i; int ok = 1;
            flash_blank(); MLog_Init(&LG, &FL, PG(4), PG(5));
            for (n = 1U; n <= nrec; n++) { MLogRec_t y = mk((uint8_t)(1U + n % 13U), n); MLog_Append(&LG, &y); }
            x = mk(MLOG_T_HEAT, nrec + 1U); budget = (int32_t)cuts; (void)MLog_Append(&LG, &x);
            power_restore();                                                                /* the SAME MLog_t carries on: no re-scan */
            z = mk(MLOG_T_FLIP, 7777U); cases++;
            if (MLog_Append(&LG, &z) != 0) ok = 0;
            if (!(MLog_Get(&LG, 0U, &g) == 0 && rec_eq(&z, &g))) ok = 0;
            MLog_Init(&L2, &FL, PG(4), PG(5));                                              /* and a restart afterwards sees the same newest entry and the same order */
            if (!(MLog_Get(&L2, 0U, &g) == 0 && rec_eq(&z, &g))) ok = 0;
            for (i = 1U; i < MLog_Count(&LG) && i < MLog_Count(&L2) && i < 20U; i++) { MLogRec_t a2, b2; if (MLog_Get(&LG, i, &a2) != 0 || MLog_Get(&L2, i, &b2) != 0 || !rec_eq(&a2, &b2)) { ok = 0; break; } }
            if (!ok) bad++;
        }
    CHECK(bad == 0U && (uint32_t)g_fail == f0, "a failed write followed by more appends WITHOUT a restart (%u cases): the half-written slot is skipped, never programmed into again (%u bad)", (unsigned)cases, (unsigned)bad);
    printf("  log transient failures: %u cases: the half-written slot is never reused\n", (unsigned)cases);
}

int main(void)
{
    t_store_basic();
    t_store_powercut();
    t_store_bitflip();
    t_settings_validate();
    t_settings_limits_model();
    t_settings_store();
    t_settings_slave_load();
    t_settings_powercut();
    t_log_basic();
    t_log_powercut();
    t_log_transient();
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
