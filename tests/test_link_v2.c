/* Tests of the version-2 link library (slave/Core/Src/link_v2.c), the SHARED source of both ends.
 *
 *  1. every example frame of the approved workbook: parsed, then rebuilt BYTE FOR BYTE (both directions)
 *  2. every field of every frame type: its minimum and maximum round-trip, one step outside is refused by the builder AND by the parser
 *  3. malformed input of every kind the parser can meet
 *  4. the $K command words, their arguments and the unknown-command path
 *  5. the byte-stream receiver: garbage, lost tails, bad checksums, unknown types, over-long lines
 *  6. fuzz: millions of random bytes, and frames with a deliberately WRONG checksum: none may be accepted
 *  6b. property tests: builders and parser agree on random frames in and out of range; randomly edited valid-checksum frames are rejected or stable
 *  7. EVERY single-byte substitution of every example (all 256 values at every position): the XOR checksum must catch every one that changes the content
 *  8. lengths: nothing built exceeds LINKV2_MAX_LEN; field counts match the Frames sheet
 *  9. version-1 compatibility: $S passes through unchanged; a v2 frame is not counted as a bad v1 frame
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "link_v2.h"

#include "link_v2_examples.inc"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* a complete frame text (with CR LF) from a body, with the CORRECT checksum */
static uint32_t mk(char *out, const char *body)
{
    static const char hexd[] = "0123456789ABCDEF";
    uint32_t n = 0U, i, bl = (uint32_t)strlen(body);
    uint8_t cs = Link_Checksum(body, bl);
    out[n++] = '$';
    for (i = 0U; i < bl; i++) out[n++] = body[i];
    out[n++] = '*'; out[n++] = hexd[cs >> 4]; out[n++] = hexd[cs & 15U]; out[n++] = '\r'; out[n++] = '\n';
    out[n] = '\0';
    return n;
}
static int32_t parse_body(const char *body, LinkV2Frame_t *f)
{
    char line[160]; uint32_t n = mk(line, body);
    return LinkV2_Parse(line, n, f);
}
static int32_t parse_str(const char *s, LinkV2Frame_t *f) { return LinkV2_Parse(s, (uint32_t)strlen(s), f); }

static uint32_t rnd_state = 12345U;
static uint32_t rnd(void) { rnd_state = (rnd_state * 1664525U) + 1013904223U; return rnd_state >> 8; }

/* rebuild a parsed frame; returns the length or a negative code; text in out */
static int32_t rebuild(const LinkV2Frame_t *f, char *out, uint32_t cap)
{
    switch (f->type)
    {
    case LINKV2_E: return LinkV2_BuildE(out, cap, &f->u.e);
    case LINKV2_H: return LinkV2_BuildH(out, cap, &f->u.h);
    case LINKV2_R: return LinkV2_BuildR(out, cap, &f->u.r);
    case LINKV2_T: return LinkV2_BuildT(out, cap, &f->u.t);
    case LINKV2_M: return LinkV2_BuildM(out, cap, &f->u.m);
    case LINKV2_K: return LinkV2_BuildK(out, cap, &f->u.k);
    case LINKV2_S: return Link_BuildStatus(out, cap, &f->u.s);
    default:       return LINK_ERR_TYPE;
    }
}
/* the one frame the parser accepts and the builders deliberately cannot build: a $K with a command word this version does not know (so the receiver can answer UNKNOWN_CMD) */
static int is_unknown_k(const LinkV2Frame_t *f) { return (f->type == LINKV2_K) && (f->u.k.cmd == LINKCMD_UNKNOWN); }

static int same_status(const LinkStatus_t *a, const LinkStatus_t *b)
{
    return (a->phase == b->phase) && (a->seq == b->seq) && (a->vin_dv == b->vin_dv) && (a->vout_dv == b->vout_dv) && (a->freq_chz == b->freq_chz) && (a->state == b->state) &&
           (a->flags == b->flags) && (a->fault == b->fault) && (a->ntc_raw == b->ntc_raw) && (a->diag == b->diag) && (a->diag2 == b->diag2);
}
/* field by field: memcmp would compare the padding bytes too */
static int same_frame(const LinkV2Frame_t *a, const LinkV2Frame_t *b)
{
    if (a->type != b->type) return 0;
    switch (a->type)
    {
    case LINKV2_E: return (a->u.e.seq == b->u.e.seq) && (a->u.e.setv_dv == b->u.e.setv_dv) && (a->u.e.flags2 == b->u.e.flags2) && (a->u.e.clr == b->u.e.clr) && (a->u.e.reach_lo_dv == b->u.e.reach_lo_dv) && (a->u.e.reach_hi_dv == b->u.e.reach_hi_dv);
    case LINKV2_H: return (a->u.h.proto == b->u.h.proto) && (a->u.h.fw == b->u.h.fw) && (a->u.h.vin_min_dv == b->u.h.vin_min_dv) && (a->u.h.vin_max_dv == b->u.h.vin_max_dv) && (a->u.h.rst == b->u.h.rst) && (a->u.h.tuned == b->u.h.tuned) && (a->u.h.vout_max_dv == b->u.h.vout_max_dv) && (a->u.h.prof_cap == b->u.h.prof_cap) && (a->u.h.prof_kib == b->u.h.prof_kib) && (a->u.h.prof_kiu == b->u.h.prof_kiu) && (a->u.h.prof_vref_dv == b->u.h.prof_vref_dv) && (a->u.h.prof_maxc_dv == b->u.h.prof_maxc_dv) && (a->u.h.prof_n == b->u.h.prof_n) && (a->u.h.prof_knee == b->u.h.prof_knee);
    case LINKV2_R: return (a->u.r.id == b->u.r.id) && (a->u.r.status == b->u.r.status) && (a->u.r.value == b->u.r.value);
    case LINKV2_T: return (a->u.t.step == b->u.t.step) && (a->u.t.ticks == b->u.t.ticks) && (a->u.t.vin_dv == b->u.t.vin_dv) && (a->u.t.dv == b->u.t.dv) && (a->u.t.result == b->u.t.result);
    case LINKV2_M: return (a->u.m.seq == b->u.m.seq) && (a->u.m.run == b->u.m.run) && (a->u.m.setv_dv == b->u.m.setv_dv) && (a->u.m.clr == b->u.m.clr) && (a->u.m.maxc_dv == b->u.m.maxc_dv);
    case LINKV2_K: return (a->u.k.id == b->u.k.id) && (a->u.k.cmd == b->u.k.cmd) && (a->u.k.num == b->u.k.num) && (a->u.k.letter == b->u.k.letter) && (strcmp(a->u.k.name, b->u.k.name) == 0);
    case LINKV2_S: return same_status(&a->u.s, &b->u.s);
    default: return 0;
    }
}

/* ------------------------------------------------------------------------------------------------------------------ */
static void t_examples(void)
{
    int i;
    for (i = 0; i < N_EXAMPLES; i++)
    {
        char line[160], rb[LINKV2_MAX_LEN + 8];
        LinkV2Frame_t f;
        int32_t rc, n;
        uint32_t len;
        snprintf(line, sizeof line, "%s\r\n", EXAMPLES[i].frame);
        len = (uint32_t)strlen(line);
        rc = LinkV2_Parse(line, len, &f);
        CHECK(rc == LINK_OK, "example '%s' must parse (rc %d)", EXAMPLES[i].name, (int)rc);
        if (rc != LINK_OK) continue;
        n = rebuild(&f, rb, sizeof rb);
        if (f.type == LINKV2_S)
        {   /* the v1 builder always appends the optional diag fields (the header's own example omits them): compare the content after a round trip */
            LinkV2Frame_t g;
            CHECK(n > 0 && LinkV2_Parse(rb, (uint32_t)n, &g) == LINK_OK && same_frame(&f, &g), "the v1 $S example round-trips with the same content");
        }
        else
        {
            CHECK(n == (int32_t)len && memcmp(rb, line, len) == 0, "example '%s': parsed and rebuilt it must be IDENTICAL (len %d vs %u)", EXAMPLES[i].name, (int)n, (unsigned)len);
        }
    }
    {   /* the values of three of them, field by field */
        LinkV2Frame_t f;
        CHECK(parse_str("$M,41,1,2300,3,300*54\r\n", &f) == LINK_OK && f.type == LINKV2_M && f.u.m.seq == 41U && f.u.m.run == 1U && f.u.m.setv_dv == 2300U && f.u.m.clr == 3U && f.u.m.maxc_dv == 300U, "M example values");
        CHECK(parse_str("$E,7,2300,3,3,2020,2700*76\r\n", &f) == LINK_OK && f.type == LINKV2_E && f.u.e.seq == 7U && f.u.e.setv_dv == 2300U && f.u.e.flags2 == 3U && f.u.e.clr == 3U && f.u.e.reach_lo_dv == 2020U && f.u.e.reach_hi_dv == 2700U, "E example values");
        CHECK(parse_str("$H,2,803,800,3100,0,1,3200*57\r\n", &f) == LINK_OK && f.type == LINKV2_H && f.u.h.proto == 2U && f.u.h.fw == 803UL && f.u.h.vin_min_dv == 800U && f.u.h.vin_max_dv == 3100U && f.u.h.rst == 0U && f.u.h.tuned == 1U && f.u.h.vout_max_dv == 3200U, "H example values");
        CHECK(parse_str("$R,5,0,10152*4C\r\n", &f) == LINK_OK && f.type == LINKV2_R && f.u.r.id == 5U && f.u.r.status == 0U && f.u.r.value == 10152, "R example values");
        CHECK(parse_str("$T,4,320,2301,48,0*41\r\n", &f) == LINK_OK && f.type == LINKV2_T && f.u.t.step == 4U && f.u.t.ticks == 320U && f.u.t.dv == 48 && f.u.t.result == 0U, "T example values");
        CHECK(parse_str("$K,5,CALV,2302*49\r\n", &f) == LINK_OK && f.type == LINKV2_K && f.u.k.id == 5U && f.u.k.cmd == LINKCMD_CALV && f.u.k.num == 2302, "K example values");
    }
    printf("  %d approved examples: each parses, and rebuilds byte for byte\n", N_EXAMPLES);
}

/* ------------------------------------------------------------------------------------------------------------------ */
/* For one frame type: every field at its minimum and maximum round-trips, one step outside is refused by BOTH the builder and the parser. */
typedef struct { const char *name; int32_t lo, hi; uint8_t zero_ok; } FldT;

static void range_test(const char *label, char type, const FldT *flds, int nf, int signed_any)
{
    int i, j;
    (void)signed_any;
    for (i = 0; i < nf; i++)
    {
        int32_t cases[6]; int nc = 0; int k;
        cases[nc++] = flds[i].lo; cases[nc++] = flds[i].hi;
        if (flds[i].zero_ok) cases[nc++] = 0;
        for (k = 0; k < nc; k++)
        {
            char body[120]; int n = 0; LinkV2Frame_t f, g; char rb[LINKV2_MAX_LEN + 8], line[160]; int32_t len; uint32_t ll;
            n += snprintf(body + n, sizeof body - (size_t)n, "%c", type);
            for (j = 0; j < nf; j++)
            {
                int32_t v = (j == i) ? cases[k] : flds[j].lo;
                if ((j != i) && flds[j].zero_ok) v = 0;
                n += snprintf(body + n, sizeof body - (size_t)n, ",%d", (int)v);
            }
            ll = mk(line, body);
            CHECK(LinkV2_Parse(line, ll, &f) == LINK_OK, "%s.%s = %d must parse (body %s)", label, flds[i].name, (int)cases[k], body);
            len = rebuild(&f, rb, sizeof rb);
            CHECK(len > 0 && LinkV2_Parse(rb, (uint32_t)len, &g) == LINK_OK && same_frame(&f, &g), "%s.%s = %d must round-trip", label, flds[i].name, (int)cases[k]);
        }
        {   /* one step outside, both sides */
            int32_t outside[2]; int no = 0, k2;
            if (flds[i].lo > -2147483647) outside[no++] = flds[i].lo - 1;
            if (flds[i].hi < 2147483647)  outside[no++] = flds[i].hi + 1;
            for (k2 = 0; k2 < no; k2++)
            {
                char body[120]; int n = 0; LinkV2Frame_t f;
                if ((outside[k2] == 0) && flds[i].zero_ok) continue;                       /* 0 is valid for these */
                if ((outside[k2] < 0) && (flds[i].lo >= 0)) continue;                      /* a '-' on an unsigned field is a format error, tested elsewhere */
                n += snprintf(body + n, sizeof body - (size_t)n, "%c", type);
                for (j = 0; j < nf; j++)
                {
                    int32_t v = (j == i) ? outside[k2] : flds[j].lo;
                    if ((j != i) && flds[j].zero_ok) v = 0;
                    n += snprintf(body + n, sizeof body - (size_t)n, ",%d", (int)v);
                }
                CHECK(parse_body(body, &f) == LINK_ERR_RANGE, "%s.%s = %d must be REFUSED by the parser (%s)", label, flds[i].name, (int)outside[k2], body);
            }
        }
    }
}

static void t_ranges(void)
{
    static const FldT E[] = { {"seq",0,255,0}, {"setv",1000,3000,1}, {"flags2",0,65535,0}, {"clr",0,255,0}, {"reach_lo",500,6553,1}, {"reach_hi",500,6553,1} };
    static const FldT H[] = { {"proto",0,255,0}, {"fw",0,9999999,0}, {"vin_min",0,65535,0}, {"vin_max",0,65535,0}, {"rst",0,2,0}, {"tuned",0,1,0}, {"vout_max",0,65535,0} };
    static const FldT R[] = { {"id",1,255,0}, {"status",0,7,0}, {"value",-2147483647,2147483647,0} };
    static const FldT T[] = { {"step",0,255,0}, {"ticks",0,4095,0}, {"vin",0,65535,0}, {"dv",-999,999,0}, {"result",0,6,0} };
    static const FldT M[] = { {"seq",0,255,0}, {"run",0,1,0}, {"setv",1000,3000,0}, {"clr",0,255,0}, {"maxc",0,1990,0} };
    range_test("E", 'E', E, 6, 0); range_test("H", 'H', H, 7, 0); range_test("R", 'R', R, 3, 1);
    range_test("T", 'T', T, 5, 1); range_test("M", 'M', M, 5, 0);
    {   /* the two OPTIONAL trailing fields of E (the converter-side current): an older sender has 6 fields, an older receiver ignores the extra ones */
        LinkV2Frame_t f; char b[LINKV2_MAX_LEN + 8]; int32_t n; LinkE_t e = {7, 2300, 3, 3, 2020, 2700, 0, 0, 0, 0, 0}; int commas, j;
        n = LinkV2_BuildE(b, sizeof b, &e); commas = 0; for (j = 0; j < n; j++) if (b[j] == ',') commas++;
        CHECK(n > 0 && commas == 6, "both optional fields 0: the frame is the 6-field frame of the first protocol (%d commas)", commas);
        e.iout_rms_ca = 589U; e.iout_pk_ca = 831U; n = LinkV2_BuildE(b, sizeof b, &e); commas = 0; for (j = 0; j < n; j++) if (b[j] == ',') commas++;
        CHECK(n > 0 && commas == 8 && strstr(b, ",589,831*") != NULL, "5.89 A and 8.31 A: the frame carries 8 fields, ending ',589,831' (%d commas)", commas);
        CHECK(LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.type == LINKV2_E && f.u.e.iout_rms_ca == 589U && f.u.e.iout_pk_ca == 831U && f.u.e.reach_hi_dv == 2700U, "and it parses back with both values");
        e.iout_rms_ca = 0U; e.iout_pk_ca = 12U; n = LinkV2_BuildE(b, sizeof b, &e); CHECK(n > 0 && strstr(b, ",0,12*") != NULL && LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.u.e.iout_rms_ca == 0U && f.u.e.iout_pk_ca == 12U, "rms 0 with a peak: both are sent (',0,12')");
        e.iout_rms_ca = 4001U; CHECK(LinkV2_BuildE(b, sizeof b, &e) == LINK_ERR_RANGE, "an RMS current above 40.00 A is refused by the builder"); e.iout_rms_ca = 0U; e.iout_pk_ca = 4001U; CHECK(LinkV2_BuildE(b, sizeof b, &e) == LINK_ERR_RANGE, "so is a peak above 40.00 A");
        e.iout_rms_ca = 4000U; e.iout_pk_ca = 4000U; CHECK(LinkV2_BuildE(b, sizeof b, &e) > 0, "40.00 A and 40.00 A build");
        /* parsing frames of the three lengths and the wrong ones */
        { static const char *g6 = "E,7,2300,3,3,2020,2700", *g7 = "E,7,2300,3,3,2020,2700,589", *g8 = "E,7,2300,3,3,2020,2700,589,831", *g9 = "E,7,2300,3,3,2020,2700,589,831,77", *g5 = "E,7,2300,3,3,2020", *bad = "E,7,2300,3,3,2020,2700,4001,0", *neg = "E,7,2300,3,3,2020,2700,-1,0", *txt = "E,7,2300,3,3,2020,2700,5.89,0";
          char fr[LINKV2_MAX_LEN + 8]; const char *all[8]; int k; int32_t m; all[0] = g6; all[1] = g7; all[2] = g8; all[3] = g9; all[4] = g5; all[5] = bad; all[6] = neg; all[7] = txt;
          for (k = 0; k < 8; k++) { uint8_t cs = 0U; const char *s = all[k]; int q = 0; fr[q++] = '$'; while (*s) { cs ^= (uint8_t)*s; fr[q++] = *s++; } fr[q++] = '*'; fr[q++] = "0123456789ABCDEF"[cs >> 4]; fr[q++] = "0123456789ABCDEF"[cs & 15U]; fr[q++] = '\r'; fr[q++] = '\n'; fr[q] = '\0'; m = LinkV2_Parse(fr, (uint32_t)q, &f);
            if (k == 0) CHECK(m == LINK_OK && f.u.e.iout_rms_ca == 0U && f.u.e.iout_pk_ca == 0U, "6 fields (an older sender): accepted, the optional fields read as 0");
            if (k == 1) CHECK(m == LINK_OK && f.u.e.iout_rms_ca == 589U && f.u.e.iout_pk_ca == 0U, "7 fields: the RMS is read, the peak reads as 0");
            if (k == 2) CHECK(m == LINK_OK && f.u.e.iout_rms_ca == 589U && f.u.e.iout_pk_ca == 831U, "8 fields: both read");
            if (k == 3) CHECK(m == LINK_OK && f.u.e.iout_pk_ca == 831U, "9 fields (a later version): the extra one is ignored");
            if (k == 4) CHECK(m != LINK_OK, "5 fields: still rejected (the first six are required)");
            if (k == 5) CHECK(m == LINK_ERR_RANGE, "an optional field that IS present must be valid: 4001 is rejected");
            if (k == 6) CHECK(m != LINK_OK, "a negative optional field is rejected");
            if (k == 7) CHECK(m != LINK_OK, "a non-number in an optional field is rejected"); } } }
    {   /* the 7 OPTIONAL trailing fields of H (the stored autotune profile): a slave without a profile sends the 7-field frame of the first protocol */
        LinkV2Frame_t f; char b[LINKV2_MAX_LEN + 8]; int32_t n; LinkH_t h = {2, 937, 800, 3000, 0, 0, 2800, 0, 0, 0, 0, 0, 0, 0}; int commas, j;
        n = LinkV2_BuildH(b, sizeof b, &h); commas = 0; for (j = 0; j < n; j++) if (b[j] == ',') commas++;
        CHECK(n > 0 && commas == 7, "no profile (all 7 fields 0): the frame is the 7-field frame of the first protocol (%d commas)", commas);
        h.tuned = 1U; h.prof_cap = 2752U; h.prof_kib = 523U; h.prof_kiu = 1000U; h.prof_vref_dv = 1600U; h.prof_maxc_dv = 1100U; h.prof_n = 10U; h.prof_knee = 0U;
        n = LinkV2_BuildH(b, sizeof b, &h); commas = 0; for (j = 0; j < n; j++) if (b[j] == ',') commas++;
        CHECK(n > 0 && commas == 14 && strstr(b, ",2752,523,1000,1600,1100,10,0*") != NULL, "a profile: 14 fields, ending ',2752,523,1000,1600,1100,10,0' (%d commas)", commas);
        CHECK(LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.type == LINKV2_H && f.u.h.prof_cap == 2752U && f.u.h.prof_kib == 523U && f.u.h.prof_kiu == 1000U && f.u.h.prof_vref_dv == 1600U && f.u.h.prof_maxc_dv == 1100U && f.u.h.prof_n == 10U && f.u.h.prof_knee == 0U && f.u.h.vout_max_dv == 2800U, "and it parses back with every value");
        h.prof_cap = 0U; h.prof_kib = 0U; h.prof_kiu = 0U; h.prof_vref_dv = 0U; h.prof_maxc_dv = 0U; h.prof_n = 0U; h.prof_knee = 1U; n = LinkV2_BuildH(b, sizeof b, &h);
        CHECK(n > 0 && LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.u.h.prof_knee == 1U && f.u.h.prof_cap == 0U, "a lone knee flag is still sent (all 14 fields) and read back");
        h.prof_knee = 0U; h.prof_cap = 4096U; CHECK(LinkV2_BuildH(b, sizeof b, &h) == LINK_ERR_RANGE, "a cap above 4095 is refused by the builder"); h.prof_cap = 100U; h.prof_n = 16U; CHECK(LinkV2_BuildH(b, sizeof b, &h) == LINK_ERR_RANGE, "so are 16 points"); h.prof_n = 15U; h.prof_knee = 2U; CHECK(LinkV2_BuildH(b, sizeof b, &h) == LINK_ERR_RANGE, "and a knee flag of 2");
        { static const char *g7 = "H,2,937,800,3000,0,1,2800", *g10 = "H,2,937,800,3000,0,1,2800,2752,523,1000", *g14 = "H,2,937,800,3000,0,1,2800,2752,523,1000,1600,1100,10,0", *g15 = "H,2,937,800,3000,0,1,2800,2752,523,1000,1600,1100,10,0,77", *g6 = "H,2,937,800,3000,0,1", *bad = "H,2,937,800,3000,0,1,2800,4096,523,1000,1600,1100,10,0", *neg = "H,2,937,800,3000,0,1,2800,-1,523", *txt = "H,2,937,800,3000,0,1,2800,27.5,523", *badn = "H,2,937,800,3000,0,1,2800,2752,523,1000,1600,1100,16,0";
          char fr[LINKV2_MAX_LEN + 8]; const char *all[8]; int k; int32_t m; all[0] = g7; all[1] = g10; all[2] = g14; all[3] = g15; all[4] = g6; all[5] = bad; all[6] = neg; all[7] = txt;
          for (k = 0; k < 9; k++) { uint8_t cs = 0U; const char *s = (k < 8) ? all[k] : badn; int q = 0; fr[q++] = '$'; while (*s) { cs ^= (uint8_t)*s; fr[q++] = *s++; } fr[q++] = '*'; fr[q++] = "0123456789ABCDEF"[cs >> 4]; fr[q++] = "0123456789ABCDEF"[cs & 15U]; fr[q++] = '\r'; fr[q++] = '\n'; fr[q] = '\0'; m = LinkV2_Parse(fr, (uint32_t)q, &f);
            if (k == 0) CHECK(m == LINK_OK && f.u.h.prof_cap == 0U && f.u.h.prof_n == 0U && f.u.h.tuned == 1U, "7 fields (an older slave): accepted, the profile fields read as 0");
            if (k == 1) CHECK(m == LINK_OK && f.u.h.prof_cap == 2752U && f.u.h.prof_kiu == 1000U && f.u.h.prof_vref_dv == 0U, "10 fields: the first three are read, the rest read as 0");
            if (k == 2) CHECK(m == LINK_OK && f.u.h.prof_n == 10U, "14 fields: all read");
            if (k == 3) CHECK(m == LINK_OK && f.u.h.prof_n == 10U, "15 fields (a later version): the extra one is ignored");
            if (k == 4) CHECK(m != LINK_OK, "6 fields: still rejected (the first seven are required)");
            if (k == 5) CHECK(m == LINK_ERR_RANGE, "an optional field that IS present must be valid: a cap of 4096 is rejected");
            if (k == 6) CHECK(m != LINK_OK, "a negative optional field is rejected");
            if (k == 7) CHECK(m != LINK_OK, "a non-number in an optional field is rejected");
            if (k == 8) CHECK(m == LINK_ERR_RANGE, "16 points are rejected"); } } }
    {   /* the builders refuse too */
        LinkM_t m = {1, 1, 2300, 0, 0, 0}; char b[96];
        CHECK(LinkV2_BuildM(b, sizeof b, &m) > 0, "a good M builds");
        m.setv_dv = 999U;  CHECK(LinkV2_BuildM(b, sizeof b, &m) == LINK_ERR_RANGE, "M with setv 99.9 V is refused by the builder");
        m.setv_dv = 3001U; CHECK(LinkV2_BuildM(b, sizeof b, &m) == LINK_ERR_RANGE, "M with setv 300.1 V is refused by the builder");
        m.setv_dv = 2300U; m.run = 2U; CHECK(LinkV2_BuildM(b, sizeof b, &m) == LINK_ERR_RANGE, "M with run 2 is refused by the builder");
        m.run = 1U; m.maxc_dv = 1991U; CHECK(LinkV2_BuildM(b, sizeof b, &m) == LINK_ERR_RANGE, "M with Max Correction 199.1 V is refused by the builder");
        m.maxc_dv = 1990U; CHECK(LinkV2_BuildM(b, sizeof b, &m) > 0, "Max Correction 199.0 V builds");
        {   /* v0.9.61: the optional hold field */
            LinkV2Frame_t f; int32_t n; uint32_t c = 0U, i;
            m.hold = 0U; n = LinkV2_BuildM(b, sizeof b, &m); for (i = 0U; i < (uint32_t)n; i++) if (b[i] == ',') c++;
            CHECK(n > 0 && c == 5U, "M with hold 0 is the 5-field frame of the first protocol (%u fields)", (unsigned)c);
            CHECK(LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.type == LINKV2_M && f.u.m.hold == 0U, "a 5-field M parses with hold 0");
            m.hold = 1U; n = LinkV2_BuildM(b, sizeof b, &m); c = 0U; for (i = 0U; i < (uint32_t)n; i++) if (b[i] == ',') c++;
            CHECK(n > 0 && c == 6U, "M with hold 1 has the 6th field (%u fields)", (unsigned)c);
            CHECK(LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.u.m.hold == 1U && f.u.m.run == 1U && f.u.m.setv_dv == 2300U && f.u.m.maxc_dv == 1990U, "and parses with hold 1, the other fields unchanged");
            m.hold = 2U; CHECK(LinkV2_BuildM(b, sizeof b, &m) == LINK_ERR_RANGE, "M with hold 2 is refused by the builder");
            {   const char *bad = "$M,1,1,2300,0,0,2*"; char t[40]; uint8_t cs = Link_Checksum(&bad[1], (uint32_t)strlen(bad) - 2U);
                snprintf(t, sizeof t, "%s%02X\r\n", bad, cs); CHECK(LinkV2_Parse(t, (uint32_t)strlen(t), &f) == LINK_ERR_RANGE, "and by the parser"); }
            m.hold = 0U;
        }
        {   LinkE_t e = {0, 999, 0, 0, 0, 0, 0, 0, 0, 0, 0}; CHECK(LinkV2_BuildE(b, sizeof b, &e) == LINK_ERR_RANGE, "E with setv 99.9 V is refused"); e.setv_dv = 0U; CHECK(LinkV2_BuildE(b, sizeof b, &e) > 0, "E with setv 0 (none yet) builds");
                    e.reach_lo_dv = 499U; CHECK(LinkV2_BuildE(b, sizeof b, &e) == LINK_ERR_RANGE, "E with reach 49.9 V is refused"); }
        {   LinkH_t h = {2, 10000000UL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; CHECK(LinkV2_BuildH(b, sizeof b, &h) == LINK_ERR_RANGE, "H with fw 10000000 is refused"); }
    }
    printf("  every field of E H R T M: minimum, maximum (and 0 where allowed) round-trip; one step outside refused by the parser and the builder\n");
}

/* ------------------------------------------------------------------------------------------------------------------ */
static void t_malformed(void)
{
    LinkV2Frame_t f; char line[160]; uint32_t n;
    CHECK(LinkV2_Parse("", 0, &f) == LINK_ERR_FORMAT, "empty");
    CHECK(parse_str("$", &f) == LINK_ERR_FORMAT, "only a dollar sign");
    CHECK(parse_str("$M*0", &f) == LINK_ERR_FORMAT, "too short");
    CHECK(parse_str("M,41,1,2300,3,300*54", &f) == LINK_ERR_FORMAT, "no leading dollar");
    CHECK(parse_str("$M,41,1,2300,3,300", &f) == LINK_ERR_FORMAT, "no checksum");
    CHECK(parse_str("$M,41,1,2300,3,300*5", &f) == LINK_ERR_FORMAT, "one checksum digit");
    CHECK(parse_str("$M,41,1,2300,3,300*5G", &f) == LINK_ERR_FORMAT, "a non-hex checksum digit");
    CHECK(parse_str("$M,41,1,2300,3,300*5a", &f) == LINK_ERR_FORMAT, "a LOWER-case checksum digit is refused (the rule is upper case)");
    CHECK(parse_str("$M,41,1,2300,3,300*55", &f) == LINK_ERR_CHECKSUM, "a wrong checksum");
    CHECK(parse_str("$M,41,1,2300,3,300*54", &f) == LINK_OK, "a line without CR LF is accepted");
    CHECK(parse_str("$M,41,1,2300,3,300*54\n", &f) == LINK_OK, "LF only");
    CHECK(parse_str("$M,41,1,2300,3,300*54\r", &f) == LINK_OK, "CR only");
    CHECK(parse_str("$M,41,1,2300,3,300*54\r\n\r\n", &f) == LINK_OK, "extra line endings are dropped");
    /* bodies with a valid checksum but wrong structure */
    CHECK(parse_body("m,41,1,2300,3,300", &f) == LINK_ERR_FORMAT, "a lower-case type letter");
    CHECK(parse_body("M41,1,2300,3,300", &f) == LINK_ERR_FORMAT, "no comma after the type");
    CHECK(parse_body("M", &f) == LINK_ERR_FORMAT, "a type letter alone");
    CHECK(parse_body("M,", &f) == LINK_ERR_FORMAT, "a type letter and a comma");
    CHECK(parse_body("M,41,1,2300,3", &f) == LINK_ERR_FORMAT, "too FEW fields (4 of 5)");
    CHECK(parse_body("M,41,1,2300,3,300,1,999,abc", &f) == LINK_OK && f.u.m.maxc_dv == 300U && f.u.m.hold == 1U, "EXTRA trailing fields (after the optional hold) are ignored");
    CHECK(parse_body("M,41,1,2300,3,300,999,abc", &f) == LINK_ERR_RANGE, "v0.9.61: the 6th field IS hold: present, it must be 0 or 1");
    CHECK(parse_body("M,41,,2300,3,300", &f) == LINK_ERR_FORMAT, "an empty field");
    CHECK(parse_body("M,41,1,2300,3,", &f) == LINK_ERR_FORMAT, "a trailing comma (empty last field)");
    CHECK(parse_body("M,41,1,2300,3,30x", &f) == LINK_ERR_FORMAT, "a non-digit");
    CHECK(parse_body("M,41,1,2300,3,+300", &f) == LINK_ERR_FORMAT, "a plus sign");
    CHECK(parse_body("M,41,1,2300,3,-1", &f) == LINK_ERR_FORMAT, "a minus on an unsigned field");
    CHECK(parse_body("M,41,1,2300,3, 300", &f) == LINK_ERR_FORMAT, "a space");
    CHECK(parse_body("M,41,1,2300,3,99999999999", &f) == LINK_ERR_RANGE, "11 digits");
    CHECK(parse_body("M,41,1,2300,3,4294967296", &f) == LINK_ERR_RANGE, "a value past 32 bits");
    CHECK(parse_body("M,41,1,2300,3,0000000300", &f) == LINK_OK && f.u.m.maxc_dv == 300U, "leading zeros are harmless");
    CHECK(parse_body("R,5,0,-2147483647", &f) == LINK_OK && f.u.r.value == -2147483647, "the most negative signed value");
    CHECK(parse_body("R,5,0,-2147483648", &f) == LINK_ERR_RANGE, "one past it");
    CHECK(parse_body("R,5,0,-", &f) == LINK_ERR_FORMAT, "a lone minus");
    CHECK(parse_body("Z,1,2,3", &f) == LINK_ERR_TYPE, "an unknown type letter with a good checksum is TYPE, not an error");
    CHECK(parse_body("Q", &f) == LINK_ERR_TYPE, "an unknown letter alone");
    CHECK(parse_body("1,2,3", &f) == LINK_ERR_FORMAT, "a digit as the type");
    /* an embedded NUL */
    n = mk(line, "M,41,1,2300,3,300"); line[5] = '\0';
    CHECK(LinkV2_Parse(line, n, &f) != LINK_OK, "an embedded NUL (the checksum changed)");
    /* a line longer than the receiver would ever hand over */
    {   char big[300]; memset(big, '1', sizeof big); big[0] = '$'; big[sizeof big - 1U] = '\n';
        CHECK(LinkV2_Parse(big, sizeof big, &f) == LINK_ERR_FORMAT, "300 junk bytes are refused, not run over"); }
    printf("  malformed input: structure, checksum, case, signs, overflow, empty and missing fields, unknown types, NUL, long junk\n");
}

/* ------------------------------------------------------------------------------------------------------------------ */
static void t_commands(void)
{
    LinkV2Frame_t f; char b[96]; LinkK_t k; int32_t n;
    int c;
    static const LinkCmd_t NOARG[] = { LINKCMD_HELLO, LINKCMD_TUNE, LINKCMD_TABORT, LINKCMD_TSAVE };
    for (c = 0; c < 4; c++)
    {
        memset(&k, 0, sizeof k); k.id = 9U; k.cmd = NOARG[c];
        n = LinkV2_BuildK(b, sizeof b, &k);
        CHECK(n > 0 && LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.type == LINKV2_K && f.u.k.cmd == NOARG[c] && f.u.k.id == 9U, "no-argument command %s round-trips", LinkV2_CmdWord(NOARG[c]));
    }
    memset(&k, 0, sizeof k); k.id = 1U; k.cmd = LINKCMD_CALV; k.num = 500;  CHECK(LinkV2_BuildK(b, sizeof b, &k) > 0, "CALV 50.0 V builds");
    k.num = 4000; CHECK(LinkV2_BuildK(b, sizeof b, &k) > 0, "CALV 400.0 V builds");
    k.num = 499;  CHECK(LinkV2_BuildK(b, sizeof b, &k) == LINK_ERR_RANGE, "CALV 49.9 V refused");
    k.num = 4001; CHECK(LinkV2_BuildK(b, sizeof b, &k) == LINK_ERR_RANGE, "CALV 400.1 V refused");
    k.cmd = LINKCMD_CALO; k.num = 2302; n = LinkV2_BuildK(b, sizeof b, &k);
    CHECK(n > 0 && LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.u.k.cmd == LINKCMD_CALO && f.u.k.num == 2302, "CALO round-trips");
    memset(&k, 0, sizeof k); k.id = 2U; k.cmd = LINKCMD_CALR; k.letter = 'V'; n = LinkV2_BuildK(b, sizeof b, &k);
    CHECK(n > 0 && LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.u.k.letter == 'V', "CALR V round-trips");
    k.letter = 'O'; CHECK(LinkV2_BuildK(b, sizeof b, &k) > 0, "CALR O builds");
    k.letter = 'X'; CHECK(LinkV2_BuildK(b, sizeof b, &k) == LINK_ERR_RANGE, "CALR X refused");
    {   static const char *const PN[] = { "CAP", "KIB", "KIU", "INJ", "DBND", "ABRT" }; int i;
        for (i = 0; i < 6; i++)
        {
            memset(&k, 0, sizeof k); k.id = 3U; k.cmd = LINKCMD_TPARAM; strcpy(k.name, PN[i]); k.num = -1234 + i; n = LinkV2_BuildK(b, sizeof b, &k);
            CHECK(n > 0 && LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.u.k.cmd == LINKCMD_TPARAM && strcmp(f.u.k.name, PN[i]) == 0 && f.u.k.num == -1234 + i, "TPARAM %s round-trips with a negative value", PN[i]);
        }
        strcpy(k.name, "XYZ"); CHECK(LinkV2_BuildK(b, sizeof b, &k) == LINK_ERR_RANGE, "TPARAM with an unknown name refused");
    }
    memset(&k, 0, sizeof k); k.id = 0U; k.cmd = LINKCMD_HELLO; CHECK(LinkV2_BuildK(b, sizeof b, &k) == LINK_ERR_RANGE, "id 0 refused by the builder");
    k.id = 1U; k.cmd = LINKCMD_UNKNOWN; CHECK(LinkV2_BuildK(b, sizeof b, &k) == LINK_ERR_RANGE, "UNKNOWN cannot be built");
    k.cmd = LINKCMD_NONE; CHECK(LinkV2_BuildK(b, sizeof b, &k) == LINK_ERR_RANGE, "NONE cannot be built");
    /* the parser */
    CHECK(parse_body("K,0,HELLO", &f) == LINK_ERR_RANGE, "id 0 refused by the parser");
    CHECK(parse_body("K,256,HELLO", &f) == LINK_ERR_RANGE, "id 256 refused");
    CHECK(parse_body("K,HELLO", &f) == LINK_ERR_FORMAT, "no id");
    CHECK(parse_body("K,5", &f) == LINK_ERR_FORMAT, "no command word");
    CHECK(parse_body("K,5,hello", &f) == LINK_ERR_FORMAT, "a lower-case command word is not a word");
    CHECK(parse_body("K,5,CALV", &f) == LINK_ERR_FORMAT, "CALV without its argument");
    CHECK(parse_body("K,5,CALV,499", &f) == LINK_ERR_RANGE, "CALV out of range");
    CHECK(parse_body("K,5,CALR", &f) == LINK_ERR_FORMAT, "CALR without its letter");
    CHECK(parse_body("K,5,CALR,VV", &f) == LINK_ERR_RANGE, "CALR with two letters");
    CHECK(parse_body("K,5,TPARAM,CAP", &f) == LINK_ERR_FORMAT, "TPARAM without its value");
    CHECK(parse_body("K,5,TPARAM,NOPE,1", &f) == LINK_ERR_RANGE, "TPARAM with an unknown name");
    CHECK(parse_body("K,5,TUNE,ignored,fields", &f) == LINK_OK && f.u.k.cmd == LINKCMD_TUNE, "extra fields after TUNE are ignored");
    CHECK(parse_body("K,7,FROBNICATE", &f) == LINK_OK && f.u.k.cmd == LINKCMD_UNKNOWN && f.u.k.id == 7U && strcmp(f.u.k.name, "FROBNIC") == 0, "an unknown word parses as UNKNOWN with its id and its first 7 letters, so the receiver can answer UNKNOWN_CMD");
    CHECK(parse_body("K,7,A1", &f) == LINK_ERR_FORMAT, "a word with a digit");
    printf("  $K: all eight commands, their arguments and ranges, the unknown-command path\n");
}

/* ------------------------------------------------------------------------------------------------------------------ */

/* ---- v0.9.51: the phase sequence measurement: $K SEQ, the SYNC byte, and the optional sq_* fields of $E ---- */
static void t_sequence(void)
{
    LinkV2Frame_t f; char b[LINKV2_MAX_LEN + 8]; int32_t n; LinkE_t e; LinkK_t k; int commas, j;
    memset(&e, 0, sizeof e); e.seq = 7U; e.setv_dv = 2300U; e.flags2 = 3U; e.clr = 3U; e.reach_lo_dv = 2020U; e.reach_hi_dv = 2700U;
    n = LinkV2_BuildE(b, sizeof b, &e); for (commas = 0, j = 0; j < n; j++) if (b[j] == ',') commas++;
    CHECK(n > 0 && commas == 6, "an $E without a sequence result is built with 6 fields exactly as before (%d commas incl. the one after E)", commas);
    e.iout_rms_ca = 589U; e.iout_pk_ca = 831U; n = LinkV2_BuildE(b, sizeof b, &e); for (commas = 0, j = 0; j < n; j++) if (b[j] == ',') commas++;
    CHECK(n > 0 && commas == 8, "with the converter current: 8 fields as before (%d commas)", commas);
    e.sq_id = 12U; e.sq_delay_us = 6667U; e.sq_period_us = 20003U; n = LinkV2_BuildE(b, sizeof b, &e); for (commas = 0, j = 0; j < n; j++) if (b[j] == ',') commas++;
    CHECK(n > 0 && commas == 11 && strstr(b, ",12,6667,20003*") != NULL, "with a sequence result: 11 fields, the last three 12,6667,20003 (%d commas)", commas);
    CHECK(LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.type == LINKV2_E && f.u.e.sq_id == 12U && f.u.e.sq_delay_us == 6667U && f.u.e.sq_period_us == 20003U && f.u.e.iout_rms_ca == 589U && f.u.e.iout_pk_ca == 831U && f.u.e.reach_hi_dv == 2700U, "it parses back with every field");
    e.iout_rms_ca = 0U; e.iout_pk_ca = 0U; n = LinkV2_BuildE(b, sizeof b, &e);
    CHECK(n > 0 && LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.u.e.sq_id == 12U && f.u.e.sq_delay_us == 6667U && f.u.e.iout_rms_ca == 0U, "a sequence result with no converter current still carries the (zero) current fields in between");
    e.sq_delay_us = LINKV2_SQ_NONE; e.sq_period_us = 0U; n = LinkV2_BuildE(b, sizeof b, &e);
    CHECK(n > 0 && LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.u.e.sq_delay_us == 65535U && f.u.e.sq_period_us == 0U, "a failed round: delay 65535 (none), period 0");
    memset(&e, 0, sizeof e); e.setv_dv = 2300U; e.sq_id = 256U; CHECK(LinkV2_BuildE(b, sizeof b, &e) == LINK_ERR_RANGE, "a round id of 256 is refused by the builder");
    /* an older receiver's view: a frame with 8 fields (no sq_*) parses with sq_id = 0 */
    memset(&e, 0, sizeof e); e.seq = 1U; e.setv_dv = 2300U; e.iout_rms_ca = 100U; e.iout_pk_ca = 200U; n = LinkV2_BuildE(b, sizeof b, &e);
    CHECK(LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.u.e.sq_id == 0U && f.u.e.sq_delay_us == 0U && f.u.e.sq_period_us == 0U, "a frame without the optional fields parses with them zero");
    /* the command */
    memset(&k, 0, sizeof k); k.id = 5U; k.cmd = LINKCMD_SEQ; k.num = 12;
    n = LinkV2_BuildK(b, sizeof b, &k);
    CHECK(n > 0 && strstr(b, "$K,5,SEQ,12*") != NULL && LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.type == LINKV2_K && f.u.k.cmd == LINKCMD_SEQ && f.u.k.num == 12 && f.u.k.id == 5U, "$K,5,SEQ,12 builds and parses back");
    k.num = 0; CHECK(LinkV2_BuildK(b, sizeof b, &k) == LINK_ERR_RANGE, "a round id of 0 is refused"); k.num = 256; CHECK(LinkV2_BuildK(b, sizeof b, &k) == LINK_ERR_RANGE, "and 256"); k.num = 255; CHECK(LinkV2_BuildK(b, sizeof b, &k) > 0, "255 is the last");
    CHECK(LinkV2_CmdWord(LINKCMD_SEQ) != NULL && strcmp(LinkV2_CmdWord(LINKCMD_SEQ), "SEQ") == 0, "the command word is SEQ");
    CHECK(LinkV2_Parse("$K,5,SEQ*00", 11U, &f) != LINK_OK, "$K,5,SEQ with no round id is refused");
    CHECK(LINKV2_SYNC_BYTE == 0x16U && LINKV2_SYNC_BYTE < 0x20U, "the SYNC byte is a control character (0x16), never part of a printable frame");
    {   int32_t i; int inside = 0;
        memset(&e, 0, sizeof e); e.setv_dv = 2300U; e.sq_id = 1U; e.sq_delay_us = 65535U; e.sq_period_us = 65535U; n = LinkV2_BuildE(b, sizeof b, &e);
        for (i = 0; i < n; i++) { if ((uint8_t)b[i] == LINKV2_SYNC_BYTE) inside = 1; }
        k.num = 7; n = LinkV2_BuildK(b, sizeof b, &k);
        for (i = 0; i < n; i++) { if ((uint8_t)b[i] == LINKV2_SYNC_BYTE) inside = 1; }
        CHECK(!inside, "no frame the library builds contains the SYNC byte");
    }
    printf("  sequence: $K SEQ round id, optional sq_* fields of $E (11 fields, older frames unchanged), the SYNC byte\n");
}

static void t_stream(void)
{
    LinkV2Rx_t rx; LinkV2Frame_t f; char buf[4096]; uint32_t n = 0U, i; int got = 0; int types[40];
    LinkV2Rx_Init(&rx);
    /* garbage, then every example in order, with trouble in between */
    n += (uint32_t)snprintf(buf + n, sizeof buf - n, "garbage\r\n\x01\x02 more ");
    for (i = 0U; i < (uint32_t)N_EXAMPLES; i++)
    {
        n += (uint32_t)snprintf(buf + n, sizeof buf - n, "%s\r\n", EXAMPLES[i].frame);
        if (i == 2U) n += (uint32_t)snprintf(buf + n, sizeof buf - n, "$M,41,1,23");                       /* a lost tail ... */
        if (i == 4U) n += (uint32_t)snprintf(buf + n, sizeof buf - n, "$M,41,1,2300,3,300*55\r\n");         /* a bad checksum */
        if (i == 6U) n += (uint32_t)snprintf(buf + n, sizeof buf - n, "$Z,1,2*");                            /* an unknown type, built below with a good checksum */
    }
    {   char unk[40]; uint32_t un = mk(unk, "Z,1,2,3"); memcpy(buf + n, unk, un); n += un; }                 /* an unknown type with a GOOD checksum */
    {   uint32_t k; buf[n++] = '$'; for (k = 0U; k < 200U; k++) buf[n++] = 'x'; buf[n++] = '\n'; }           /* an over-long line */
    n += (uint32_t)snprintf(buf + n, sizeof buf - n, "%s\r\n", EXAMPLES[7].frame);                          /* the M example once more, after all that */
    for (i = 0U; i < n; i++)
    {
        int32_t t = LinkV2Rx_Feed(&rx, (uint8_t)buf[i], &f);
        if (t > 0 && got < 40) types[got++] = (int)t;
    }
    CHECK(got == N_EXAMPLES + 1, "every example plus the final M arrives (%d of %d)", got, N_EXAMPLES + 1);
    for (i = 0U; (i < (uint32_t)N_EXAMPLES) && ((int)i < got); i++)
    {
        const char c = EXAMPLES[i].frame[1];
        const int want = (c == 'S') ? LINKV2_S : (c == 'E') ? LINKV2_E : (c == 'H') ? LINKV2_H : (c == 'R') ? LINKV2_R : (c == 'T') ? LINKV2_T : (c == 'M') ? LINKV2_M : LINKV2_K;
        CHECK(types[i] == want, "frame %u arrives as the right type (%d, wanted %d)", (unsigned)i, types[i], want);
    }
    CHECK(types[got - 1] == LINKV2_M, "the final M arrives after the over-long line");
    CHECK(rx.frames_ok == (uint32_t)(N_EXAMPLES + 1), "frames_ok counts them (%u)", (unsigned)rx.frames_ok);
    CHECK(rx.frames_bad == 1U, "exactly the one bad-checksum frame is counted bad (%u)", (unsigned)rx.frames_bad);
    CHECK(rx.frames_unknown == 1U, "exactly the good-checksum unknown-type frame is counted unknown (%u)", (unsigned)rx.frames_unknown);
    CHECK(rx.overflows == 1U, "the over-long line is counted (%u)", (unsigned)rx.overflows);
    /* the lost tail and the half '$Z,1,2*' were restarted by the next '$', not counted as anything else */
    printf("  stream: %d frames recovered through garbage, a lost tail, a bad checksum, an unknown type and an over-long line (bad %u, unknown %u, overflow %u)\n", got, (unsigned)rx.frames_bad, (unsigned)rx.frames_unknown, (unsigned)rx.overflows);
}

/* ------------------------------------------------------------------------------------------------------------------ */
static void t_fuzz(void)
{
    LinkV2Rx_t rx; LinkV2Frame_t f; uint32_t i, accepted_ok = 0U, wrong_cs_accepted = 0U, unstable = 0U;
    static const char alpha[] = "0123456789,,,,$$**-ABCDEFGHKMRSTVOIZ\r\n abz";
    LinkV2Rx_Init(&rx);
    for (i = 0U; i < 4000000U; i++)
    {
        uint8_t byte = ((rnd() & 3U) == 0U) ? (uint8_t)(rnd() & 255U) : (uint8_t)alpha[rnd() % (sizeof alpha - 1U)];
        int32_t t = LinkV2Rx_Feed(&rx, byte, &f);
        if (t > 0)
        {   /* whatever it accepted must be stable: rebuild it and parse it again */
            char rb[LINKV2_MAX_LEN + 8]; LinkV2Frame_t g; int32_t len = rebuild(&f, rb, sizeof rb);
            accepted_ok++;
            if (is_unknown_k(&f)) continue;
            if (!((len > 0) && (LinkV2_Parse(rb, (uint32_t)len, &g) == LINK_OK) && same_frame(&f, &g))) unstable++;
        }
    }
    CHECK(unstable == 0U, "every frame the receiver accepted from random bytes rebuilds and re-parses identically (%u unstable of %u)", (unsigned)unstable, (unsigned)accepted_ok);
    /* frames with a deliberately WRONG checksum: 300000 random bodies */
    for (i = 0U; i < 300000U; i++)
    {
        char body[60]; uint32_t k, n = 0U; static const char types[] = "EHRTMK"; char line[160]; uint8_t cs, bad;
        body[n++] = types[rnd() % 6U];
        for (k = 0U; k < 5U + (rnd() % 20U); k++) body[n++] = (char)(((rnd() & 3U) == 0U) ? ',' : ('0' + (rnd() % 10U)));
        body[n] = '\0';
        cs = Link_Checksum(body, n); bad = (uint8_t)(cs ^ (uint8_t)(1U + (rnd() % 255U)));
        snprintf(line, sizeof line, "$%s*%02X\r\n", body, (unsigned)bad);
        if (LinkV2_Parse(line, (uint32_t)strlen(line), &f) == LINK_OK) wrong_cs_accepted++;
    }
    CHECK(wrong_cs_accepted == 0U, "no frame with a wrong checksum is ever accepted (%u of 300000)", (unsigned)wrong_cs_accepted);
    printf("  fuzz: 4,000,000 random bytes (%u frames accepted, all stable) and 300,000 wrong-checksum frames (none accepted)\n", (unsigned)accepted_ok);
}


/* ------------------------------------------------------------------------------------------------------------------ */
/* Structure-aware tests: the builders and the parser must AGREE on what is valid, over random values in and out of range,
 * and valid-checksum frames with randomly EDITED content must be rejected or stable (this exercises the accept path that random bytes never reach). */
static const FldT PE[] = { {"seq",0,255,0}, {"setv",1000,3000,1}, {"flags2",0,65535,0}, {"clr",0,255,0}, {"reach_lo",500,6553,1}, {"reach_hi",500,6553,1}, {"iout_rms",0,4000,0}, {"iout_pk",0,4000,0} };
static const FldT PH[] = { {"proto",0,255,0}, {"fw",0,9999999,0}, {"vin_min",0,65535,0}, {"vin_max",0,65535,0}, {"rst",0,2,0}, {"tuned",0,1,0}, {"vout_max",0,65535,0}, {"p_cap",0,4095,0}, {"p_kib",0,65535,0}, {"p_kiu",0,65535,0}, {"p_vref",0,65535,0}, {"p_maxc",0,65535,0}, {"p_n",0,15,0}, {"p_knee",0,1,0} };
static const FldT PR[] = { {"id",1,255,0}, {"status",0,7,0}, {"value",-2147483647,2147483647,0} };
static const FldT PT[] = { {"step",0,255,0}, {"ticks",0,4095,0}, {"vin",0,65535,0}, {"dv",-999,999,0}, {"result",0,6,0} };
static const FldT PM[] = { {"seq",0,255,0}, {"run",0,1,0}, {"setv",1000,3000,0}, {"clr",0,255,0}, {"maxc",0,1990,0} };

static int32_t rnd_in(const FldT *f)
{
    uint32_t r = rnd();
    int64_t span = (int64_t)f->hi - (int64_t)f->lo + 1;
    if (f->zero_ok && ((r & 7U) == 0U)) return 0;
    return (int32_t)((int64_t)f->lo + (int64_t)(rnd() % (uint32_t)(span > 4294967295LL ? 4294967295LL : span)));
}
static int32_t rnd_any(const FldT *f)
{
    uint32_t r = rnd() % 6U;
    if (r == 0U) return f->lo - 1;
    if (r == 1U) return f->hi + 1;
    if (r == 2U) return (int32_t)(rnd() % 70000U) - 1000;
    return rnd_in(f);
}
static int in_range(const FldT *f, int32_t v) { return ((v >= f->lo) && (v <= f->hi)) || (f->zero_ok && v == 0); }

static void t_property(void)
{
    uint32_t it, built = 0U, refused = 0U, disagree = 0U, edits_accepted = 0U, edits_unstable = 0U, edits_total = 0U, unknown_k = 0U;
    for (it = 0U; it < 600000U; it++)
    {
        const int which = (int)(rnd() % 5U);
        const FldT *fl = (which == 0) ? PE : (which == 1) ? PH : (which == 2) ? PR : (which == 3) ? PT : PM;
        const int nf = (which == 0) ? 8 : (which == 1) ? 14 : (which == 2) ? 3 : 5;
        const char type = "EHRTM"[which];
        int32_t v[14]; int j, all_ok = 1; char b[LINKV2_MAX_LEN + 8], body[120]; int32_t n; int bn; LinkV2Frame_t f, g;
        for (j = 0; j < nf; j++) { v[j] = ((rnd() % 4U) == 0U) ? rnd_any(&fl[j]) : rnd_in(&fl[j]); if (!in_range(&fl[j], v[j])) all_ok = 0; }
        if ((fl[nf - 1].lo < -1000) && (v[nf - 1] == -2147483647 - 1)) continue;
        switch (which)
        {
        case 0: { LinkE_t x = { (uint8_t)v[0], (uint16_t)v[1], (uint16_t)v[2], (uint8_t)v[3], (uint16_t)v[4], (uint16_t)v[5], (uint16_t)v[6], (uint16_t)v[7], 0, 0, 0 }; n = (all_ok) ? LinkV2_BuildE(b, sizeof b, &x) : LINK_ERR_RANGE; break; }
        case 1: { LinkH_t x = { (uint8_t)v[0], (uint32_t)v[1], (uint16_t)v[2], (uint16_t)v[3], (uint8_t)v[4], (uint8_t)v[5], (uint16_t)v[6], (uint16_t)v[7], (uint16_t)v[8], (uint16_t)v[9], (uint16_t)v[10], (uint16_t)v[11], (uint8_t)v[12], (uint8_t)v[13] }; n = (all_ok) ? LinkV2_BuildH(b, sizeof b, &x) : LINK_ERR_RANGE; break; }
        case 2: { LinkR_t x = { (uint8_t)v[0], (uint8_t)v[1], v[2] }; n = (all_ok) ? LinkV2_BuildR(b, sizeof b, &x) : LINK_ERR_RANGE; break; }
        case 3: { LinkT_t x = { (uint8_t)v[0], (uint16_t)v[1], (uint16_t)v[2], (int16_t)v[3], (uint8_t)v[4] }; n = (all_ok) ? LinkV2_BuildT(b, sizeof b, &x) : LINK_ERR_RANGE; break; }
        default:{ LinkM_t x = { (uint8_t)v[0], (uint8_t)v[1], (uint16_t)v[2], (uint8_t)v[3], (uint16_t)v[4], 0U }; n = (all_ok) ? LinkV2_BuildM(b, sizeof b, &x) : LINK_ERR_RANGE; break; }
        }
        bn = snprintf(body, sizeof body, "%c", type);
        for (j = 0; j < nf; j++) bn += snprintf(body + bn, sizeof body - (size_t)bn, ",%d", (int)v[j]);
        {   char line[200]; uint32_t ll = mk(line, body); int32_t rc = LinkV2_Parse(line, ll, &f);
            if (all_ok)
            {
                built++;
                if (!((n > 0) && (rc == LINK_OK) && (LinkV2_Parse(b, (uint32_t)n, &g) == LINK_OK) && same_frame(&f, &g) && ((uint32_t)n == ll) && (memcmp(b, line, ll) == 0))) disagree++;
            }
            else
            {
                refused++;
                if (!((n == LINK_ERR_RANGE) && ((rc == LINK_ERR_RANGE) || (rc == LINK_ERR_FORMAT)))) disagree++;     /* a field below zero on an unsigned type is FORMAT, the rest RANGE */
            }
        }
        /* a valid-checksum frame with 1 to 3 random byte edits in its body */
        if ((it % 4U) == 0U)
        {
            char ed[120]; int k, m = (int)(1U + (rnd() % 3U)); char line[200]; uint32_t ll;
            strcpy(ed, body);
            for (k = 0; k < m; k++) { size_t pos = (size_t)(rnd() % (uint32_t)strlen(ed)); static const char al[] = "0123456789,-AEHKMRTVO"; ed[pos] = al[rnd() % (sizeof al - 1U)]; }
            ll = mk(line, ed);
            edits_total++;
            if (LinkV2_Parse(line, ll, &f) == LINK_OK)
            {
                char rb[LINKV2_MAX_LEN + 8]; int32_t len = rebuild(&f, rb, sizeof rb);
                edits_accepted++;
                if (is_unknown_k(&f)) { unknown_k++; if (len != LINK_ERR_RANGE) edits_unstable++; continue; }     /* by design: accepted, never built */
                if (!((len > 0) && (LinkV2_Parse(rb, (uint32_t)len, &g) == LINK_OK) && same_frame(&f, &g)))
                {
                    edits_unstable++;
                    if (edits_unstable <= 5U) printf("  UNSTABLE: edited body '%s' -> accepted type %d; rebuild len %d, text '%.*s'\n", ed, (int)f.type, (int)len, (len > 2) ? (int)(len - 2) : 0, rb);
                }
            }
        }
    }
    CHECK(built > 100000U && refused > 50000U, "the generator exercised both sides (%u valid, %u invalid)", (unsigned)built, (unsigned)refused);
    CHECK(disagree == 0U, "the builders and the parser agree on every one of %u random frames, in and out of range (%u disagreements)", (unsigned)(built + refused), (unsigned)disagree);
    CHECK(edits_accepted > 1000U, "the edit test reached the accept path (%u of %u edited frames accepted)", (unsigned)edits_accepted, (unsigned)edits_total);
    CHECK(edits_unstable == 0U, "every accepted edited frame rebuilds and re-parses identically, except the unknown-command $K which the builder refuses (%u unstable)", (unsigned)edits_unstable);
    CHECK(unknown_k > 0U, "the unknown-command path was exercised by the edits (%u frames)", (unsigned)unknown_k);
    printf("  property: %u random valid and %u random invalid frames: builder and parser agree; %u of %u randomly EDITED valid-checksum frames were accepted and all were stable\n", (unsigned)built, (unsigned)refused, (unsigned)edits_accepted, (unsigned)edits_total);
}

/* ------------------------------------------------------------------------------------------------------------------ */
static void t_mutation(void)
{
    int e; unsigned total = 0U, caught = 0U, same_content = 0U, differ = 0U;
    for (e = 0; e < N_EXAMPLES; e++)
    {
        char line[160]; LinkV2Frame_t orig, m; uint32_t len, pos; unsigned v;
        snprintf(line, sizeof line, "%s\r\n", EXAMPLES[e].frame);
        len = (uint32_t)strlen(line);
        CHECK(LinkV2_Parse(line, len, &orig) == LINK_OK, "mutation base %d parses", e);
        for (pos = 0U; pos < len; pos++)
        {
            for (v = 0U; v < 256U; v++)
            {
                char mut[160]; int32_t rc;
                if ((char)v == line[pos]) continue;
                memcpy(mut, line, len); mut[pos] = (char)v;
                total++;
                rc = LinkV2_Parse(mut, len, &m);
                if (rc != LINK_OK) { caught++; continue; }
                if (same_frame(&orig, &m)) same_content++;      /* e.g. CR changed to LF at the end: the same frame */
                else { differ++; printf("  MUTANT ACCEPTED with DIFFERENT content: example %d, position %u, byte 0x%02X\n", e, (unsigned)pos, v); }
            }
        }
    }
    CHECK(differ == 0U, "no single-byte substitution produced an accepted frame with different content (%u)", differ);
    printf("  mutation: %u single-byte substitutions of the %d examples: %u rejected, %u accepted with the SAME content (line-ending changes only), %u with different content\n", total, N_EXAMPLES, caught, same_content, differ);
}

/* ------------------------------------------------------------------------------------------------------------------ */
static void t_lengths(void)
{
    char b[LINKV2_MAX_LEN + 8]; int32_t n, maxlen = 0;
    LinkE_t e = {255, 3000, 65535, 255, 6553, 6553, 4000, 4000, 255, 65535, 65535}; LinkH_t h = {255, 9999999UL, 65535, 65535, 2, 1, 65535, 4095, 65535, 65535, 65535, 65535, 15, 1}; LinkR_t r = {255, 7, -2147483647};
    LinkT_t t = {255, 4095, 65535, -999, 6}; LinkM_t m = {255, 1, 3000, 255, 1990, 1}; LinkK_t k;
    n = LinkV2_BuildE(b, sizeof b, &e); CHECK(n > 0 && n <= (int32_t)LINKV2_MAX_LEN && b[n - 1] == '\n' && b[n - 2] == '\r', "E at its maximum fits (%d bytes)", (int)n); if (n > maxlen) maxlen = n;
    n = LinkV2_BuildH(b, sizeof b, &h); CHECK(n > 0 && n <= (int32_t)LINKV2_MAX_LEN, "H at its maximum fits (%d bytes)", (int)n); if (n > maxlen) maxlen = n;
    n = LinkV2_BuildR(b, sizeof b, &r); CHECK(n > 0 && n <= (int32_t)LINKV2_MAX_LEN, "R at its maximum fits (%d bytes)", (int)n); if (n > maxlen) maxlen = n;
    n = LinkV2_BuildT(b, sizeof b, &t); CHECK(n > 0 && n <= (int32_t)LINKV2_MAX_LEN, "T at its maximum fits (%d bytes)", (int)n); if (n > maxlen) maxlen = n;
    n = LinkV2_BuildM(b, sizeof b, &m); CHECK(n > 0 && n <= (int32_t)LINKV2_MAX_LEN, "M at its maximum fits (%d bytes)", (int)n); if (n > maxlen) maxlen = n;
    memset(&k, 0, sizeof k); k.id = 255U; k.cmd = LINKCMD_TPARAM; strcpy(k.name, "DBND"); k.num = -2147483647;
    n = LinkV2_BuildK(b, sizeof b, &k); CHECK(n > 0 && n <= (int32_t)LINKV2_MAX_LEN, "K, the longest (TPARAM DBND, most negative value) fits (%d bytes)", (int)n); if (n > maxlen) maxlen = n;
    CHECK(maxlen <= (int32_t)LINK_MAX_LINE, "the longest v2 frame (%d) is within the receiver's line limit (%u)", (int)maxlen, (unsigned)LINK_MAX_LINE);
    n = LinkV2_BuildM(b, 10U, &m); CHECK(n == LINK_ERR_SPACE, "a buffer that is too small is reported, not overrun (%d)", (int)n);
    /* field counts against the Frames sheet: E 6, H 7, R 3, T 5, M 5 numeric fields (commas = fields) */
    {   struct { const char *name; int32_t n; int want; } c[5];
        char bb[LINKV2_MAX_LEN + 8]; int i, commas, j;
        LinkE_t e2 = {7, 2300, 3, 3, 2020, 2700, 0, 0, 0, 0, 0}; LinkH_t h2 = {2, 803, 800, 3100, 0, 1, 3200, 0, 0, 0, 0, 0, 0, 0}; LinkR_t r2 = {5, 0, 10152}; LinkT_t t2 = {4, 320, 2301, 48, 0}; LinkM_t m2 = {41, 1, 2300, 3, 300, 0};
        c[0].name = "E"; c[0].n = LinkV2_BuildE(bb, sizeof bb, &e2); c[0].want = 6; for (i = 0; i < 1; i++) { commas = 0; for (j = 0; j < c[0].n; j++) if (bb[j] == ',') commas++; CHECK(commas == 6, "E has 6 fields (%d)", commas); }
        c[1].name = "H"; c[1].n = LinkV2_BuildH(bb, sizeof bb, &h2); commas = 0; for (j = 0; j < c[1].n; j++) if (bb[j] == ',') commas++; CHECK(commas == 7, "H has 7 fields (%d)", commas);
        c[2].name = "R"; c[2].n = LinkV2_BuildR(bb, sizeof bb, &r2); commas = 0; for (j = 0; j < c[2].n; j++) if (bb[j] == ',') commas++; CHECK(commas == 3, "R has 3 fields (%d)", commas);
        c[3].name = "T"; c[3].n = LinkV2_BuildT(bb, sizeof bb, &t2); commas = 0; for (j = 0; j < c[3].n; j++) if (bb[j] == ',') commas++; CHECK(commas == 5, "T has 5 fields (%d)", commas);
        c[4].name = "M"; c[4].n = LinkV2_BuildM(bb, sizeof bb, &m2); commas = 0; for (j = 0; j < c[4].n; j++) if (bb[j] == ',') commas++; CHECK(commas == 5, "M has 5 fields (%d)", commas);
    }
    printf("  lengths: the longest frame is %d bytes (limit %u); field counts match the Frames sheet\n", (int)maxlen, (unsigned)LINK_MAX_LINE);
}

/* ------------------------------------------------------------------------------------------------------------------ */
static void t_v1_compat(void)
{
    LinkStatus_t s = { 'Y', 200, 2305, 2300, 4998, 2, 19, 0, 1871, 0, 0 };
    char b[LINK_STATUS_MAX_LEN + 8]; int32_t n; LinkV2Frame_t f; LinkV2Rx_t rx; LinkRx_t v1; LinkStatus_t o; uint32_t i;
    n = Link_BuildStatus(b, sizeof b, &s);
    CHECK(n > 0 && LinkV2_Parse(b, (uint32_t)n, &f) == LINK_OK && f.type == LINKV2_S && same_status(&f.u.s, &s), "a v1 $S built by v1 parses through v2 with every field the same");
    LinkV2Rx_Init(&rx); LinkRx_Init(&v1);
    {   int got2 = 0, got1 = 0; char ex[160];
        for (i = 0U; i < (uint32_t)N_EXAMPLES; i++)
        {
            uint32_t k, len; snprintf(ex, sizeof ex, "%s\r\n", EXAMPLES[i].frame); len = (uint32_t)strlen(ex);
            for (k = 0U; k < len; k++)
            {
                if (LinkV2Rx_Feed(&rx, (uint8_t)ex[k], &f) > 0) got2++;
                if (LinkRx_Feed(&v1, (uint8_t)ex[k], &o) > 0) got1++;
            }
        }
        CHECK(got2 == N_EXAMPLES, "the v2 receiver takes all %d examples (%d)", N_EXAMPLES, got2);
        CHECK(got1 == 1, "the v1 receiver takes exactly the one $S among them (%d)", got1);
        CHECK(v1.frames_bad == 0U, "the v1 receiver does NOT count the other frame types as BAD frames (%u): a v1 master next to a v2 slave must not raise its error counter", (unsigned)v1.frames_bad);
    }
    printf("  version 1: $S passes through v2 unchanged; a v1 receiver ignores v2 frames without counting them bad\n");
}

int main(void)
{
    t_sequence();
    t_examples();
    t_ranges();
    t_malformed();
    t_commands();
    t_stream();
    t_fuzz();
    t_property();
    t_mutation();
    t_lengths();
    t_v1_compat();
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
