/**
  ******************************************************************************
  * @file    link_proto.c
  * @brief   Master <-> slave link protocol v1. See link_proto.h for the format.
  *          Shared by the slave and the master; contains no hardware access.
  ******************************************************************************
  */

#include "link_proto.h"

uint8_t Link_Checksum(const char *p, uint32_t n)
{
    uint8_t cs = 0U;
    uint32_t i;
    for (i = 0U; i < n; i++)
    {
        cs ^= (uint8_t)p[i];
    }
    return cs;
}

/* ------------------------------------------------------------------------- */
/* Building                                                                  */
/* ------------------------------------------------------------------------- */

/* Appends v in decimal; returns the number of characters written (1..10). */
static uint32_t put_uint(char *p, uint32_t v)
{
    char     tmp[10];
    uint32_t n = 0U;
    uint32_t i;
    do
    {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    } while (v != 0U);
    for (i = 0U; i < n; i++)
    {
        p[i] = tmp[n - 1U - i];
    }
    return n;
}

int32_t Link_BuildStatus(char *out, uint32_t cap, const LinkStatus_t *s)
{
    static const char hexd[] = "0123456789ABCDEF";
    uint32_t n = 0U;
    uint8_t  cs;

    if (cap < LINK_STATUS_MAX_LEN)
    {
        return LINK_ERR_SPACE;
    }
    if (((s->phase != 'R') && (s->phase != 'Y') && (s->phase != 'B')) ||
        (s->state > 9U) || (s->ntc_raw > 4095U))
    {
        return LINK_ERR_RANGE;
    }

    out[n++] = '$';
    out[n++] = 'S';
    out[n++] = ',';
    out[n++] = s->phase;                         out[n++] = ',';
    n += put_uint(&out[n], s->seq);              out[n++] = ',';
    n += put_uint(&out[n], s->vin_dv);           out[n++] = ',';
    n += put_uint(&out[n], s->vout_dv);          out[n++] = ',';
    n += put_uint(&out[n], s->freq_chz);         out[n++] = ',';
    n += put_uint(&out[n], s->state);            out[n++] = ',';
    n += put_uint(&out[n], s->flags);            out[n++] = ',';
    n += put_uint(&out[n], s->fault);            out[n++] = ',';
    n += put_uint(&out[n], s->ntc_raw);          out[n++] = ',';
    n += put_uint(&out[n], s->diag);             out[n++] = ',';
    n += put_uint(&out[n], s->diag2);

    cs = Link_Checksum(&out[1], n - 1U);         /* everything after '$' */
    out[n++] = '*';
    out[n++] = hexd[(cs >> 4) & 0x0FU];
    out[n++] = hexd[cs & 0x0FU];
    out[n++] = '\r';
    out[n++] = '\n';
    return (int32_t)n;
}

/* ------------------------------------------------------------------------- */
/* Parsing                                                                   */
/* ------------------------------------------------------------------------- */

/* Upper-case only, exactly as the spec says: a frame then has ONE valid
 * encoding, so any single-bit error in it is rejected. */
static int32_t hexval(char c)
{
    if ((c >= '0') && (c <= '9')) return (int32_t)(c - '0');
    if ((c >= 'A') && (c <= 'F')) return (int32_t)(c - 'A') + 10;
    return -1;
}

/* Reads an unsigned decimal number (1..6 digits) from *pp, then requires the
 * terminator: ',' (more fields follow) or the end of the body. On success
 * advances *pp past the terminator. */
static int32_t parse_field(const char **pp, const char *end, uint32_t maxv,
                           uint32_t *out, uint8_t last)
{
    const char *p = *pp;
    uint32_t v = 0U;
    uint32_t digits = 0U;

    while ((p < end) && (*p >= '0') && (*p <= '9'))
    {
        if (++digits > 6U) return LINK_ERR_FORMAT;
        v = (v * 10U) + (uint32_t)(*p - '0');
        p++;
    }
    if (digits == 0U) return LINK_ERR_FORMAT;
    if (v > maxv)     return LINK_ERR_RANGE;

    if (last)
    {
        if (p != end) return LINK_ERR_FORMAT;       /* extra data after the last field */
    }
    else
    {
        if ((p >= end) || (*p != ',')) return LINK_ERR_FORMAT;
        p++;                                         /* skip the comma */
    }
    *out = v;
    *pp  = p;
    return LINK_OK;
}

int32_t Link_ParseStatus(const char *line, uint32_t len, LinkStatus_t *s)
{
    const char *body;
    const char *end;
    const char *p;
    uint32_t    blen;
    int32_t     hi, lo, rc;
    uint32_t    v[10];
    LinkStatus_t t;

    /* Drop the line ending. */
    while ((len > 0U) && ((line[len - 1U] == '\n') || (line[len - 1U] == '\r')))
    {
        len--;
    }

    /* "$" + at least "S,R,0,0,0,0,0,0,0,0" + "*HH" */
    if ((len < 8U) || (line[0] != '$') || (line[len - 3U] != '*'))
    {
        return LINK_ERR_FORMAT;
    }

    hi = hexval(line[len - 2U]);
    lo = hexval(line[len - 1U]);
    if ((hi < 0) || (lo < 0))
    {
        return LINK_ERR_FORMAT;
    }
    body = &line[1];
    blen = len - 4U;                              /* between '$' and '*' */
    if (Link_Checksum(body, blen) != (uint8_t)((hi << 4) | lo))
    {
        return LINK_ERR_CHECKSUM;
    }

    /* Frame type */
    if (body[0] != 'S')
    {
        return ((body[0] >= 'A') && (body[0] <= 'Z')) ? LINK_ERR_TYPE : LINK_ERR_FORMAT;
    }
    if ((blen < 5U) || (body[1] != ','))
    {
        return LINK_ERR_FORMAT;
    }

    end = body + blen;
    p   = body + 2;

    /* Phase letter */
    if ((p + 1 >= end) || (p[1] != ','))
    {
        return LINK_ERR_FORMAT;
    }
    if ((p[0] != 'R') && (p[0] != 'Y') && (p[0] != 'B'))
    {
        return LINK_ERR_RANGE;
    }
    t.phase = p[0];
    p += 2;

    /* seq, vin, vout, freq, state, flags, fault, ntc [, diag [, diag2]] - the last two fields are optional */
    {
        const char *q;
        uint32_t    commas = 0U;
        uint32_t    extra;                       /* number of optional fields present: 0, 1 or 2 */
        for (q = p; q < end; q++) { if (*q == ',') commas++; }
        if ((commas < 7U) || (commas > 9U)) return LINK_ERR_FORMAT;       /* 8..10 numeric fields = 7..9 commas */
        extra = commas - 7U;

        rc = parse_field(&p, end, 255U,   &v[0], 0U); if (rc != LINK_OK) return rc;
        rc = parse_field(&p, end, 65535U, &v[1], 0U); if (rc != LINK_OK) return rc;
        rc = parse_field(&p, end, 65535U, &v[2], 0U); if (rc != LINK_OK) return rc;
        rc = parse_field(&p, end, 65535U, &v[3], 0U); if (rc != LINK_OK) return rc;
        rc = parse_field(&p, end, 9U,     &v[4], 0U); if (rc != LINK_OK) return rc;
        rc = parse_field(&p, end, 65535U, &v[5], 0U); if (rc != LINK_OK) return rc;
        rc = parse_field(&p, end, 65535U, &v[6], 0U); if (rc != LINK_OK) return rc;
        rc = parse_field(&p, end, 4095U,  &v[7], (uint8_t)((extra == 0U) ? 1U : 0U)); if (rc != LINK_OK) return rc;
        v[8] = 0U; v[9] = 0U;
        if (extra >= 1U) { rc = parse_field(&p, end, 65535U, &v[8], (uint8_t)((extra == 1U) ? 1U : 0U)); if (rc != LINK_OK) return rc; }
        if (extra >= 2U) { rc = parse_field(&p, end, 65535U, &v[9], 1U); if (rc != LINK_OK) return rc; }
    }

    t.seq      = (uint8_t)v[0];
    t.vin_dv   = (uint16_t)v[1];
    t.vout_dv  = (uint16_t)v[2];
    t.freq_chz = (uint16_t)v[3];
    t.state    = (uint8_t)v[4];
    t.flags    = (uint16_t)v[5];
    t.fault    = (uint16_t)v[6];
    t.ntc_raw  = (uint16_t)v[7];
    t.diag     = (uint16_t)v[8];
    t.diag2    = (uint16_t)v[9];
    *s = t;
    return LINK_OK;
}

/* ------------------------------------------------------------------------- */
/* Byte-stream receiver                                                      */
/* ------------------------------------------------------------------------- */

void LinkRx_Init(LinkRx_t *r)
{
    r->len        = 0U;
    r->active     = 0U;
    r->frames_ok  = 0U;
    r->frames_bad = 0U;
    r->overflows  = 0U;
    r->frames_other = 0U;
}

int32_t LinkRx_Feed(LinkRx_t *r, uint8_t byte, LinkStatus_t *out)
{
    LinkStatus_t tmp;

    if (byte == (uint8_t)'$')
    {
        /* A '$' always (re)starts a frame, so a lost tail cannot stall us. */
        r->buf[0] = '$';
        r->len    = 1U;
        r->active = 1U;
        return 0;
    }
    if (!r->active)
    {
        return 0;                                  /* garbage between frames */
    }
    if (byte == (uint8_t)'\n')
    {
        int32_t rc = Link_ParseStatus(r->buf, r->len, &tmp);
        r->active = 0U;
        r->len    = 0U;
        if (rc == LINK_OK)
        {
            *out = tmp;
            r->frames_ok++;
            return 1;
        }
        if (rc == LINK_ERR_TYPE)
        {
            r->frames_other++;                     /* a v2 frame next to a v1 receiver is not an error */
            return 0;
        }
        r->frames_bad++;
        return 0;
    }
    if (r->len >= LINK_MAX_LINE)
    {
        r->active = 0U;
        r->len    = 0U;
        r->overflows++;
        return 0;
    }
    r->buf[r->len++] = (char)byte;
    return 0;
}
