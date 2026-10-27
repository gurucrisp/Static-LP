/**
  ******************************************************************************
  * @file    link_v2.c
  * @brief   Master <-> slave link, version 2. See link_v2.h. SHARED SOURCE (slave, master, PC tests).
  ******************************************************************************
  */
#include "link_v2.h"

#define INT32_LIM  2147483647

/* ---- the documented range of every numeric field, ONE table per frame type, used by the builders AND the parser ---- */
typedef struct
{
    int32_t lo;
    int32_t hi;
    uint8_t zero_ok;     /* the value 0 is valid although it is outside lo..hi ("0 or 1000..3000") */
} Fld_t;

static const Fld_t F_E[11] = { {0, 255, 0}, {1000, 3000, 1}, {0, 65535, 0}, {0, 255, 0}, {500, 6553, 1}, {500, 6553, 1}, {0, 4000, 0}, {0, 4000, 0}, {0, 255, 0}, {0, 65535, 0}, {0, 65535, 0} };
static const Fld_t F_H[14] = { {0, 255, 0}, {0, 9999999, 0}, {0, 65535, 0}, {0, 65535, 0}, {0, 2, 0}, {0, 1, 0}, {0, 65535, 0}, {0, 4095, 0}, {0, 65535, 0}, {0, 65535, 0}, {0, 65535, 0}, {0, 65535, 0}, {0, 15, 0}, {0, 1, 0} };
static const Fld_t F_R[3] = { {1, 255, 0}, {0, 7, 0}, {-INT32_LIM, INT32_LIM, 0} };
static const Fld_t F_T[5] = { {0, 255, 0}, {0, 4095, 0}, {0, 65535, 0}, {-999, 999, 0}, {0, 6, 0} };
static const Fld_t F_M[6] = { {0, 255, 0}, {0, 1, 0}, {1000, 3000, 0}, {0, 255, 0}, {0, 1990, 0}, {0, 1, 0} };
static const Fld_t F_K_ID  = {1, 255, 0};
static const Fld_t F_K_CAL = {500, 4000, 0};
static const Fld_t F_K_SEQ = {1, 255, 0};
static const Fld_t F_K_VAL = {-INT32_LIM, INT32_LIM, 0};

/* zero a frame before it is filled: no uninitialised bytes (struct padding) in anything handed back */
static void zero_frame(LinkV2Frame_t *f)
{
    uint8_t *p = (uint8_t *)f;
    uint32_t i;
    for (i = 0U; i < (uint32_t)sizeof(*f); i++) p[i] = 0U;
}

static uint8_t field_ok(const Fld_t *f, int32_t v)
{
    if ((v >= f->lo) && (v <= f->hi)) return 1U;
    if ((f->zero_ok != 0U) && (v == 0)) return 1U;
    return 0U;
}

/* ---- command words ----------------------------------------------------------------------------------------- */
typedef struct { LinkCmd_t cmd; const char *word; } CmdEnt_t;
static const CmdEnt_t CMDS[] =
{
    { LINKCMD_HELLO,  "HELLO"  }, { LINKCMD_CALV,   "CALV"   }, { LINKCMD_CALO,  "CALO"  },
    { LINKCMD_CALR,   "CALR"   }, { LINKCMD_TUNE,   "TUNE"   }, { LINKCMD_TABORT, "TABORT" },
    { LINKCMD_TPARAM, "TPARAM" }, { LINKCMD_TSAVE,  "TSAVE"  }, { LINKCMD_SEQ, "SEQ" }
};
#define N_CMDS ((uint32_t)(sizeof(CMDS) / sizeof(CMDS[0])))

static const char *const PNAMES[] = { "CAP", "KIB", "KIU", "INJ", "DBND", "ABRT" };
#define N_PNAMES ((uint32_t)(sizeof(PNAMES) / sizeof(PNAMES[0])))

const char *LinkV2_CmdWord(LinkCmd_t cmd)
{
    uint32_t i;
    for (i = 0U; i < N_CMDS; i++)
    {
        if (CMDS[i].cmd == cmd) return CMDS[i].word;
    }
    return "";
}

/* ---- small string helpers (no libc: the same source runs on both MCUs) ------------------------------------- */
static uint32_t slen(const char *s)
{
    uint32_t n = 0U;
    while (s[n] != '\0') n++;
    return n;
}

static uint8_t tok_is(const char *t, uint32_t n, const char *word)
{
    uint32_t i;
    if (slen(word) != n) return 0U;
    for (i = 0U; i < n; i++)
    {
        if (t[i] != word[i]) return 0U;
    }
    return 1U;
}

static int32_t hexv(char c)
{
    if ((c >= '0') && (c <= '9')) return (int32_t)(c - '0');
    if ((c >= 'A') && (c <= 'F')) return (int32_t)(c - 'A') + 10;
    return -1;
}

/* ---- writer ----------------------------------------------------------------------------------------------- */
typedef struct
{
    char     b[LINKV2_MAX_LEN];
    uint32_t n;
    uint8_t  over;
} Wr_t;

static void w_char(Wr_t *w, char c)
{
    if (w->n >= (LINKV2_MAX_LEN - 6U)) { w->over = 1U; return; }     /* room for "$" "*HH" CR LF around the body */
    w->b[w->n++] = c;
}

static void w_str(Wr_t *w, const char *s)
{
    while (*s != '\0') { w_char(w, *s); s++; }
}

static void w_i32(Wr_t *w, int32_t v)
{
    char     tmp[12];
    uint32_t k = 0U;
    uint32_t u;
    if (v < 0) { w_char(w, '-'); u = (uint32_t)(-(v + 1)) + 1U; }
    else       { u = (uint32_t)v; }
    if (u == 0U) { w_char(w, '0'); return; }
    while (u != 0U) { tmp[k++] = (char)('0' + (u % 10U)); u /= 10U; }
    while (k != 0U) { w_char(w, tmp[--k]); }
}

/* '$' + body + '*' + two hex digits + CR LF into out. */
static int32_t w_finish(const Wr_t *w, char *out, uint32_t cap)
{
    static const char hexd[] = "0123456789ABCDEF";
    uint32_t i, n = 0U;
    uint8_t  cs;
    if (w->over != 0U) return LINK_ERR_SPACE;
    if (cap < (w->n + 6U)) return LINK_ERR_SPACE;
    cs = Link_Checksum(w->b, w->n);
    out[n++] = '$';
    for (i = 0U; i < w->n; i++) out[n++] = w->b[i];
    out[n++] = '*';
    out[n++] = hexd[(cs >> 4) & 0x0FU];
    out[n++] = hexd[cs & 0x0FU];
    out[n++] = '\r';
    out[n++] = '\n';
    return (int32_t)n;
}

/* A frame that is just a type letter and its numeric fields, each checked against its table. */
static int32_t build_nums(char type, const Fld_t *tab, const int32_t *v, uint32_t cnt, char *out, uint32_t cap)
{
    Wr_t     w;
    uint32_t i;
    w.n = 0U; w.over = 0U;
    for (i = 0U; i < cnt; i++)
    {
        if (field_ok(&tab[i], v[i]) == 0U) return LINK_ERR_RANGE;
    }
    w_char(&w, type);
    for (i = 0U; i < cnt; i++) { w_char(&w, ','); w_i32(&w, v[i]); }
    return w_finish(&w, out, cap);
}

int32_t LinkV2_BuildE(char *out, uint32_t cap, const LinkE_t *f)
{
    const int32_t v[11] = { f->seq, f->setv_dv, f->flags2, f->clr, f->reach_lo_dv, f->reach_hi_dv, f->iout_rms_ca, f->iout_pk_ca, f->sq_id, f->sq_delay_us, f->sq_period_us };
    /* CANONICAL: the two optional fields are left out when both are 0, so a slave without the I-OUT channel sends exactly the frame of the first protocol (6 fields) */
    return build_nums('E', F_E, v, (f->sq_id != 0U) ? 11U : (((f->iout_rms_ca != 0U) || (f->iout_pk_ca != 0U)) ? 8U : 6U), out, cap);
}

int32_t LinkV2_BuildH(char *out, uint32_t cap, const LinkH_t *f)
{
    const int32_t v[14] = { f->proto, (int32_t)f->fw, f->vin_min_dv, f->vin_max_dv, f->rst, f->tuned, f->vout_max_dv, f->prof_cap, f->prof_kib, f->prof_kiu, f->prof_vref_dv, f->prof_maxc_dv, f->prof_n, f->prof_knee };
    const uint8_t has_prof = (uint8_t)((f->prof_cap != 0U) || (f->prof_kib != 0U) || (f->prof_kiu != 0U) || (f->prof_vref_dv != 0U) || (f->prof_maxc_dv != 0U) || (f->prof_n != 0U) || (f->prof_knee != 0U));
    if (f->fw > 9999999UL) return LINK_ERR_RANGE;
    /* CANONICAL: the 7 profile fields are left out when all are 0 (no profile stored): the frame is then exactly the 7-field frame of the first protocol */
    return build_nums('H', F_H, v, (has_prof != 0U) ? 14U : 7U, out, cap);
}

int32_t LinkV2_BuildR(char *out, uint32_t cap, const LinkR_t *f)
{
    const int32_t v[3] = { f->id, f->status, f->value };
    return build_nums('R', F_R, v, 3U, out, cap);
}

int32_t LinkV2_BuildT(char *out, uint32_t cap, const LinkT_t *f)
{
    const int32_t v[5] = { f->step, f->ticks, f->vin_dv, f->dv, f->result };
    return build_nums('T', F_T, v, 5U, out, cap);
}

int32_t LinkV2_BuildM(char *out, uint32_t cap, const LinkM_t *f)
{
    const int32_t v[6] = { f->seq, f->run, f->setv_dv, f->clr, f->maxc_dv, f->hold };
    return build_nums('M', F_M, v, (f->hold != 0U) ? 6U : 5U, out, cap);      /* CANONICAL: hold = 0 is left out (the frame of the first protocol) */
}

int32_t LinkV2_BuildK(char *out, uint32_t cap, const LinkK_t *f)
{
    Wr_t     w;
    uint32_t i;
    uint8_t  known = 0U;
    w.n = 0U; w.over = 0U;
    if (field_ok(&F_K_ID, f->id) == 0U) return LINK_ERR_RANGE;
    for (i = 0U; i < N_CMDS; i++) { if (CMDS[i].cmd == f->cmd) known = 1U; }
    if (known == 0U) return LINK_ERR_RANGE;                           /* NONE and UNKNOWN cannot be built */
    w_str(&w, "K,"); w_i32(&w, f->id); w_char(&w, ','); w_str(&w, LinkV2_CmdWord(f->cmd));
    switch (f->cmd)
    {
    case LINKCMD_CALV:
    case LINKCMD_CALO:
        if (field_ok(&F_K_CAL, f->num) == 0U) return LINK_ERR_RANGE;
        w_char(&w, ','); w_i32(&w, f->num);
        break;
    case LINKCMD_SEQ:
        if (field_ok(&F_K_SEQ, f->num) == 0U) return LINK_ERR_RANGE;
        w_char(&w, ','); w_i32(&w, f->num);
        break;
    case LINKCMD_CALR:
        if ((f->letter != 'V') && (f->letter != 'O')) return LINK_ERR_RANGE;
        w_char(&w, ','); w_char(&w, f->letter);
        break;
    case LINKCMD_TPARAM:
    {
        uint8_t ok = 0U;
        for (i = 0U; i < N_PNAMES; i++)
        {
            if (tok_is(f->name, slen(f->name), PNAMES[i]) != 0U) ok = 1U;
        }
        if ((ok == 0U) || (field_ok(&F_K_VAL, f->num) == 0U)) return LINK_ERR_RANGE;
        w_char(&w, ','); w_str(&w, f->name); w_char(&w, ','); w_i32(&w, f->num);
        break;
    }
    default:
        break;
    }
    return w_finish(&w, out, cap);
}

/* ---- parser ------------------------------------------------------------------------------------------------ */

/* The next comma-separated token of [*pp, end). Empty tokens are a format error. Returns 1 and advances, or 0. */
static uint8_t next_tok(const char **pp, const char *end, const char **t, uint32_t *n)
{
    const char *p = *pp;
    const char *s = p;
    if (p >= end) return 0U;
    while ((p < end) && (*p != ',')) p++;
    if (p == s) return 0U;                       /* empty field */
    *t = s; *n = (uint32_t)(p - s);
    *pp = (p < end) ? (p + 1) : p;               /* skip the comma (a trailing comma leaves an empty token that the next call rejects) */
    return 1U;
}

/* Decimal integer: optional '-' (only if neg_ok), 1..10 digits, within +-2147483647. Returns LINK_OK, LINK_ERR_FORMAT or LINK_ERR_RANGE. */
static int32_t parse_i32(const char *t, uint32_t n, uint8_t neg_ok, int32_t *out)
{
    uint32_t i = 0U;
    uint8_t  neg = 0U;
    int64_t  v = 0;
    if ((n > 0U) && (t[0] == '-'))
    {
        if (neg_ok == 0U) return LINK_ERR_FORMAT;
        neg = 1U; i = 1U;
    }
    if ((n - i) == 0U) return LINK_ERR_FORMAT;
    if ((n - i) > 10U) return LINK_ERR_RANGE;
    for (; i < n; i++)
    {
        if ((t[i] < '0') || (t[i] > '9')) return LINK_ERR_FORMAT;
        v = (v * 10) + (int64_t)(t[i] - '0');
    }
    if (neg != 0U) v = -v;
    if ((v > (int64_t)INT32_LIM) || (v < -(int64_t)INT32_LIM)) return LINK_ERR_RANGE;
    *out = (int32_t)v;
    return LINK_OK;
}

/* cnt numeric fields, each checked against its table. More fields than cnt are ignored; fewer are a format error. */
static int32_t parse_nums(const char *p, const char *end, const Fld_t *tab, uint32_t cnt, int32_t *v)
{
    uint32_t    i;
    const char *t = p;
    uint32_t    n = 0U;
    for (i = 0U; i < cnt; i++)
    {
        int32_t rc;
        if (next_tok(&p, end, &t, &n) == 0U) return LINK_ERR_FORMAT;
        rc = parse_i32(t, n, (uint8_t)((tab[i].lo < 0) ? 1U : 0U), &v[i]);
        if (rc != LINK_OK) return rc;
        if (field_ok(&tab[i], v[i]) == 0U) return LINK_ERR_RANGE;
    }
    return LINK_OK;
}

/* The first cnt_req fields are required; the following ones up to cnt_max are OPTIONAL (an older sender does not have them: they read as 0). A field that IS present must be valid. Tokens beyond cnt_max are ignored. */
static int32_t parse_nums_opt(const char *p, const char *end, const Fld_t *tab, uint32_t cnt_req, uint32_t cnt_max, int32_t *v)
{
    uint32_t    i;
    const char *t = p;
    uint32_t    n = 0U;
    for (i = 0U; i < cnt_max; i++)
    {
        int32_t rc;
        if (next_tok(&p, end, &t, &n) == 0U)
        {
            if (i < cnt_req) return LINK_ERR_FORMAT;
            v[i] = 0;                                                    /* an optional field that is not there */
            continue;
        }
        rc = parse_i32(t, n, (uint8_t)((tab[i].lo < 0) ? 1U : 0U), &v[i]);
        if (rc != LINK_OK) return rc;
        if (field_ok(&tab[i], v[i]) == 0U) return LINK_ERR_RANGE;
    }
    return LINK_OK;
}

static int32_t parse_k(const char *p, const char *end, LinkK_t *k)
{
    const char *t = p;
    uint32_t    n = 0U, i;
    int32_t     rc, id = 0;
    LinkK_t     r;
    r.id = 0U; r.cmd = LINKCMD_NONE; r.num = 0; r.letter = '\0';
    for (i = 0U; i < 8U; i++) r.name[i] = '\0';
    for (i = 0U; i < (uint32_t)sizeof(r); i++) ((uint8_t *)&r)[i] = 0U;          /* padding too */
    r.cmd = LINKCMD_NONE;

    if (next_tok(&p, end, &t, &n) == 0U) return LINK_ERR_FORMAT;
    rc = parse_i32(t, n, 0U, &id);
    if (rc != LINK_OK) return rc;
    if (field_ok(&F_K_ID, id) == 0U) return LINK_ERR_RANGE;
    r.id = (uint8_t)id;

    if (next_tok(&p, end, &t, &n) == 0U) return LINK_ERR_FORMAT;
    for (i = 0U; i < n; i++)
    {
        if ((t[i] < 'A') || (t[i] > 'Z')) return LINK_ERR_FORMAT;
    }
    r.cmd = LINKCMD_UNKNOWN;
    for (i = 0U; i < N_CMDS; i++)
    {
        if (tok_is(t, n, CMDS[i].word) != 0U) r.cmd = CMDS[i].cmd;
    }
    if (r.cmd == LINKCMD_UNKNOWN)
    {
        for (i = 0U; (i < n) && (i < 7U); i++) r.name[i] = t[i];
        *k = r;
        return LINK_OK;                           /* the receiver answers UNKNOWN_CMD */
    }

    switch (r.cmd)
    {
    case LINKCMD_CALV:
    case LINKCMD_CALO:
        if (next_tok(&p, end, &t, &n) == 0U) return LINK_ERR_FORMAT;
        rc = parse_i32(t, n, 0U, &r.num);
        if (rc != LINK_OK) return rc;
        if (field_ok(&F_K_CAL, r.num) == 0U) return LINK_ERR_RANGE;
        break;
    case LINKCMD_SEQ:
        if (next_tok(&p, end, &t, &n) == 0U) return LINK_ERR_FORMAT;
        rc = parse_i32(t, n, 0U, &r.num);
        if (rc != LINK_OK) return rc;
        if (field_ok(&F_K_SEQ, r.num) == 0U) return LINK_ERR_RANGE;
        break;
    case LINKCMD_CALR:
        if (next_tok(&p, end, &t, &n) == 0U) return LINK_ERR_FORMAT;
        if ((n != 1U) || ((t[0] != 'V') && (t[0] != 'O'))) return LINK_ERR_RANGE;
        r.letter = t[0];
        break;
    case LINKCMD_TPARAM:
    {
        uint8_t ok = 0U;
        if (next_tok(&p, end, &t, &n) == 0U) return LINK_ERR_FORMAT;
        for (i = 0U; i < N_PNAMES; i++)
        {
            if (tok_is(t, n, PNAMES[i]) != 0U)
            {
                uint32_t j;
                ok = 1U;
                for (j = 0U; j < n; j++) r.name[j] = t[j];
            }
        }
        if (ok == 0U) return LINK_ERR_RANGE;
        if (next_tok(&p, end, &t, &n) == 0U) return LINK_ERR_FORMAT;
        rc = parse_i32(t, n, 1U, &r.num);
        if (rc != LINK_OK) return rc;
        break;
    }
    default:
        break;                                    /* HELLO, TUNE, TABORT, TSAVE: no argument; anything after is ignored */
    }
    *k = r;
    return LINK_OK;
}

int32_t LinkV2_Parse(const char *line, uint32_t len, LinkV2Frame_t *out)
{
    const char *body, *p, *end;
    uint32_t    blen;
    int32_t     hi, lo, rc;
    int32_t     v[14];                       /* the longest frame: H with its 7 optional profile fields */
    LinkV2Frame_t f;

    zero_frame(&f);
    while ((len > 0U) && ((line[len - 1U] == '\n') || (line[len - 1U] == '\r'))) len--;
    if ((len < 5U) || (line[0] != '$') || (line[len - 3U] != '*')) return LINK_ERR_FORMAT;     /* "$X*HH" is the shortest frame */
    hi = hexv(line[len - 2U]); lo = hexv(line[len - 1U]);
    if ((hi < 0) || (lo < 0)) return LINK_ERR_FORMAT;
    body = &line[1];
    blen = len - 4U;
    if (Link_Checksum(body, blen) != (uint8_t)((hi << 4) | lo)) return LINK_ERR_CHECKSUM;

    if ((body[0] < 'A') || (body[0] > 'Z')) return LINK_ERR_FORMAT;
    if (body[0] == 'S')
    {
        {   LinkStatus_t st;
            rc = Link_ParseStatus(line, len, &st);
            if (rc != LINK_OK) return rc;
            f.u.s = st;
        }
        f.type = LINKV2_S;
        *out = f;
        return LINK_OK;
    }
    if ((body[0] != 'E') && (body[0] != 'H') && (body[0] != 'R') && (body[0] != 'T') && (body[0] != 'M') && (body[0] != 'K')) return LINK_ERR_TYPE;
    if ((blen < 3U) || (body[1] != ',')) return LINK_ERR_FORMAT;
    p = body + 2;
    end = body + blen;

    switch (body[0])
    {
    case 'E':
        rc = parse_nums_opt(p, end, F_E, 6U, 11U, v);
        if (rc != LINK_OK) return rc;
        f.type = LINKV2_E;
        f.u.e.seq = (uint8_t)v[0]; f.u.e.setv_dv = (uint16_t)v[1]; f.u.e.flags2 = (uint16_t)v[2];
        f.u.e.clr = (uint8_t)v[3]; f.u.e.reach_lo_dv = (uint16_t)v[4]; f.u.e.reach_hi_dv = (uint16_t)v[5]; f.u.e.iout_rms_ca = (uint16_t)v[6]; f.u.e.iout_pk_ca = (uint16_t)v[7];
        f.u.e.sq_id = (uint16_t)v[8]; f.u.e.sq_delay_us = (uint16_t)v[9]; f.u.e.sq_period_us = (uint16_t)v[10];
        break;
    case 'H':
        rc = parse_nums_opt(p, end, F_H, 7U, 14U, v);
        if (rc != LINK_OK) return rc;
        f.type = LINKV2_H;
        f.u.h.proto = (uint8_t)v[0]; f.u.h.fw = (uint32_t)v[1]; f.u.h.vin_min_dv = (uint16_t)v[2]; f.u.h.vin_max_dv = (uint16_t)v[3];
        f.u.h.rst = (uint8_t)v[4]; f.u.h.tuned = (uint8_t)v[5]; f.u.h.vout_max_dv = (uint16_t)v[6];
        f.u.h.prof_cap = (uint16_t)v[7]; f.u.h.prof_kib = (uint16_t)v[8]; f.u.h.prof_kiu = (uint16_t)v[9]; f.u.h.prof_vref_dv = (uint16_t)v[10]; f.u.h.prof_maxc_dv = (uint16_t)v[11]; f.u.h.prof_n = (uint8_t)v[12]; f.u.h.prof_knee = (uint8_t)v[13];
        break;
    case 'R':
        rc = parse_nums(p, end, F_R, 3U, v);
        if (rc != LINK_OK) return rc;
        f.type = LINKV2_R;
        f.u.r.id = (uint8_t)v[0]; f.u.r.status = (uint8_t)v[1]; f.u.r.value = v[2];
        break;
    case 'T':
        rc = parse_nums(p, end, F_T, 5U, v);
        if (rc != LINK_OK) return rc;
        f.type = LINKV2_T;
        f.u.t.step = (uint8_t)v[0]; f.u.t.ticks = (uint16_t)v[1]; f.u.t.vin_dv = (uint16_t)v[2]; f.u.t.dv = (int16_t)v[3]; f.u.t.result = (uint8_t)v[4];
        break;
    case 'M':
        rc = parse_nums_opt(p, end, F_M, 5U, 6U, v);
        if (rc != LINK_OK) return rc;
        f.type = LINKV2_M;
        f.u.m.seq = (uint8_t)v[0]; f.u.m.run = (uint8_t)v[1]; f.u.m.setv_dv = (uint16_t)v[2]; f.u.m.clr = (uint8_t)v[3]; f.u.m.maxc_dv = (uint16_t)v[4]; f.u.m.hold = (uint8_t)v[5];
        break;
    default:   /* 'K' */
        rc = parse_k(p, end, &f.u.k);
        if (rc != LINK_OK) return rc;
        f.type = LINKV2_K;
        break;
    }
    *out = f;
    return LINK_OK;
}

/* ---- stream receiver ---------------------------------------------------------------------------------------- */
void LinkV2Rx_Init(LinkV2Rx_t *r)
{
    r->len = 0U; r->active = 0U;
    r->frames_ok = 0U; r->frames_bad = 0U; r->frames_unknown = 0U; r->overflows = 0U;
}

int32_t LinkV2Rx_Feed(LinkV2Rx_t *r, uint8_t byte, LinkV2Frame_t *out)
{
    LinkV2Frame_t tmp;

    if (byte == (uint8_t)'$')
    {
        r->buf[0] = '$'; r->len = 1U; r->active = 1U;       /* a '$' always (re)starts a frame, so a lost tail cannot stall us */
        return 0;
    }
    if (r->active == 0U) return 0;                           /* garbage between frames */
    if (byte == (uint8_t)'\n')
    {
        int32_t rc = LinkV2_Parse(r->buf, r->len, &tmp);
        r->active = 0U; r->len = 0U;
        if (rc == LINK_OK)       { *out = tmp; r->frames_ok++; return (int32_t)tmp.type; }
        if (rc == LINK_ERR_TYPE) { r->frames_unknown++; return 0; }
        r->frames_bad++;
        return 0;
    }
    if (r->len >= LINK_MAX_LINE)
    {
        r->active = 0U; r->len = 0U; r->overflows++;
        return 0;
    }
    r->buf[r->len++] = (char)byte;
    return 0;
}
