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
           a->scroll == b->scroll && a->scroll_time == b->scroll_time && a->password == b->password && a->restart_auto == b->restart_auto && a->restart_time == b->restart_time;
}

static void t_settings_validate(void)
{
    MSettings_t d, s; MLimits_t lim = { 80U, 310U, 265U };
    MS_Defaults(&d);
    CHECK(MS_Validate(&d, 0) == MS_OK && MS_Validate(&d, &lim) == MS_OK, "the factory defaults are valid (and inside the slave's window and guard)");
    CHECK(d.mode == 0U && d.set_v == 230U && d.ip_lv == 180U && d.ip_hv == 280U && d.op_lv == 200U && d.op_hv == 250U && d.volt_cut_delay == 5U && d.ovl_cut_delay == 10U && d.heat_delay == 10U &&
          d.phase_seq == 0U && d.scroll == 1U && d.scroll_time == 5U && d.password == 0U && d.restart_auto == 1U && d.restart_time == 5U && d.maxcorr_v == 0U && d.ovl_current == MASTER_RATED_CURRENT_A, "the defaults are the agreed ones");
#define EDGE(field, lo, hi, elo, ehi) do { s = d; s.field = (lo); CHECK(MS_Validate(&s, 0) == MS_OK, #field " minimum %d is valid", (int)(lo)); s.field = (hi); CHECK(MS_Validate(&s, 0) == MS_OK, #field " maximum %d is valid", (int)(hi)); \
        s.field = (lo) - 1; CHECK(MS_Validate(&s, 0) == (elo), #field " below the minimum is refused"); s.field = (hi) + 1; CHECK(MS_Validate(&s, 0) == (ehi), #field " above the maximum is refused"); } while (0)
    EDGE(volt_cut_delay, 1, 99, MS_BAD_VOLT_DELAY, MS_BAD_VOLT_DELAY);
    EDGE(ovl_current, 1, 180, MS_BAD_OVL_CURRENT, MS_BAD_OVL_CURRENT);
    EDGE(ovl_cut_delay, 1, 99, MS_BAD_OVL_DELAY, MS_BAD_OVL_DELAY);
    EDGE(heat_delay, 1, 99, MS_BAD_HEAT_DELAY, MS_BAD_HEAT_DELAY);
    EDGE(scroll_time, 1, 99, MS_BAD_SCROLL_TIME, MS_BAD_SCROLL_TIME);
    EDGE(restart_time, 1, 999, MS_BAD_RESTART_TIME, MS_BAD_RESTART_TIME);
    s = d; s.maxcorr_v = 199U; CHECK(MS_Validate(&s, 0) == MS_OK, "Max Correction 199 is valid"); s.maxcorr_v = 200U; CHECK(MS_Validate(&s, 0) == MS_BAD_MAXCORR, "200 is not");
    s = d; s.password = 999U; CHECK(MS_Validate(&s, 0) == MS_OK, "password 999 is valid"); s.password = 1000U; CHECK(MS_Validate(&s, 0) == MS_BAD_PASSWORD, "1000 is not");
    s = d; s.mode = 3U; CHECK(MS_Validate(&s, 0) == MS_BAD_MODE, "mode 3 is refused");
    s = d; s.set_v = 99U; s.ip_lv = 50U; s.op_lv = 50U; CHECK(MS_Validate(&s, 0) == MS_BAD_SET_V, "Set Voltage 99 is refused"); s.set_v = 301U; s.ip_hv = 320U; s.op_hv = 320U; CHECK(MS_Validate(&s, 0) == MS_BAD_SET_V, "301 is refused");
    s = d; s.set_v = 100U; s.ip_lv = 90U; s.op_lv = 95U; CHECK(MS_Validate(&s, 0) == MS_OK, "Set Voltage 100 with lower cut-offs is valid");
    /* the cross rules: strict */
    s = d; s.ip_lv = 230U; CHECK(MS_Validate(&s, 0) == MS_BAD_IP_LV, "I/P LV equal to Set V is refused");
    s = d; s.ip_lv = 229U; CHECK(MS_Validate(&s, 0) == MS_OK, "one volt below is valid");
    s = d; s.ip_hv = 230U; CHECK(MS_Validate(&s, 0) == MS_BAD_IP_HV, "I/P HV equal to Set V is refused");
    s = d; s.op_lv = 230U; CHECK(MS_Validate(&s, 0) == MS_BAD_OP_LV, "O/P LV equal to Set V is refused");
    s = d; s.op_hv = 230U; CHECK(MS_Validate(&s, 0) == MS_BAD_OP_HV, "O/P HV equal to Set V is refused");
    /* the slave's window and guard (D7, D26) */
    s = d; s.ip_lv = 79U; CHECK(MS_Validate(&s, &lim) == MS_BAD_IP_LV && MS_Validate(&s, 0) == MS_OK, "an I/P LV below the slave's window is refused only when the window is known");
    s = d; s.ip_lv = 80U; CHECK(MS_Validate(&s, &lim) == MS_OK, "exactly at the window edge is valid");
    s = d; s.ip_hv = 311U; CHECK(MS_Validate(&s, &lim) == MS_BAD_IP_HV, "an I/P HV above the window is refused");
    s = d; s.op_hv = 266U; CHECK(MS_Validate(&s, &lim) == MS_BAD_OP_HV, "an O/P HV above the slave's output guard is refused");
    s = d; s.op_hv = 265U; CHECK(MS_Validate(&s, &lim) == MS_OK, "exactly at the guard is valid");
    s = d; s.phase_seq = 2U; CHECK(MS_Validate(&s, 0) == MS_BAD_FLAG, "a flag of 2 is refused");
    printf("  settings validation: defaults, every range edge, the strict cross rules, the slave's window and guard\n");
}

static void t_settings_store(void)
{
    MStore_t st; MSettings_t a, b, c; uint8_t raw[32]; int i, bad = 0;
    flash_blank(); MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
    CHECK(MS_Load(&st, &a) == 0U, "blank flash: defaults are used"); { MSettings_t d; MS_Defaults(&d); CHECK(ms_eq(&a, &d), "and they ARE the defaults"); }
    a.set_v = 240U; a.ip_lv = 170U; a.mode = 2U; a.restart_auto = 0U; a.password = 321U; a.maxcorr_v = 25U; a.ovl_current = 120U;
    CHECK(MS_Save(&st, &a) == 0, "save");
    MStore_Init(&st, &FL, PG(0), PG(1), MS_VERSION, MS_PACK_LEN);
    CHECK(MS_Load(&st, &b) == 1U && b.set_v == 240U && b.ip_lv == 170U && b.mode == 2U && b.restart_auto == 0U && b.password == 321U && b.maxcorr_v == 25U && b.ovl_current == 120U, "the settings come back after a restart");
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
        MSettings_t x, y; uint8_t p[32];
        MS_Defaults(&x); x.mode = (uint8_t)(rnd() % 3U); x.set_v = (uint16_t)(100U + rnd() % 201U); x.maxcorr_v = (uint16_t)(rnd() % 200U);
        x.ip_lv = (uint16_t)(1U + rnd() % (x.set_v - 1U)); x.ip_hv = (uint16_t)(x.set_v + 1U + rnd() % 100U); x.op_lv = (uint16_t)(1U + rnd() % (x.set_v - 1U)); x.op_hv = (uint16_t)(x.set_v + 1U + rnd() % 100U);
        x.volt_cut_delay = (uint8_t)(1U + rnd() % 99U); x.ovl_current = (uint16_t)(1U + rnd() % 180U); x.ovl_cut_delay = (uint8_t)(1U + rnd() % 99U); x.heat_delay = (uint8_t)(1U + rnd() % 99U);
        x.phase_seq = (uint8_t)(rnd() & 1U); x.scroll = (uint8_t)(rnd() & 1U); x.scroll_time = (uint8_t)(1U + rnd() % 99U); x.password = (uint16_t)(rnd() % 1000U); x.restart_auto = (uint8_t)(rnd() & 1U); x.restart_time = (uint16_t)(1U + rnd() % 999U);
        MS_Pack(&x, p); MS_Unpack(&y, p);
        if (!(MS_Validate(&x, 0) == MS_OK && ms_eq(&x, &y))) bad++;
    }
    CHECK(bad == 0, "100,000 random valid settings survive pack and unpack exactly (%d differ)", bad);
    printf("  settings storage: round trip, invalid never written, a rule-breaking record not trusted, 100,000 pack round trips\n");
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
    t_settings_store();
    t_settings_powercut();
    t_log_basic();
    t_log_powercut();
    t_log_transient();
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
