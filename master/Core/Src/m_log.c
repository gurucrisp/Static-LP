/** @file m_log.c  @brief The fault log. See m_log.h. */
#include "m_log.h"

static void w16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static uint16_t r16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static uint8_t blank(const uint8_t *b) { uint32_t i; for (i = 0U; i < MLOG_REC_BYTES; i++) if (b[i] != 0xFFU) return 0U; return 1U; }
static uint8_t valid(const uint8_t *b) { return (M_Crc16(b, 14U) == r16(&b[14])) ? 1U : 0U; }

static void encode(uint8_t *b, const MLogRec_t *r, uint16_t seq)
{
    b[0] = r->type; b[1] = (uint8_t)((r->phase & 3U) | ((r->slave_code & 63U) << 2));
    w16(&b[2], r->vin_dv); w16(&b[4], r->vout_dv); w16(&b[6], r->load_da);
    b[8] = (uint8_t)r->run_min; b[9] = (uint8_t)(r->run_min >> 8); b[10] = (uint8_t)(r->run_min >> 16); b[11] = (uint8_t)(r->run_min >> 24);
    w16(&b[12], seq); w16(&b[14], M_Crc16(b, 14U));
}
static void decode(const uint8_t *b, MLogRec_t *r)
{
    r->type = b[0]; r->phase = (uint8_t)(b[1] & 3U); r->slave_code = (uint8_t)(b[1] >> 2);
    r->vin_dv = r16(&b[2]); r->vout_dv = r16(&b[4]); r->load_da = r16(&b[6]);
    r->run_min = (uint32_t)b[8] | ((uint32_t)b[9] << 8) | ((uint32_t)b[10] << 16) | ((uint32_t)b[11] << 24);
}

/* insert into the newest-first index */
static void index_add(MLog_t *l, uint16_t seq, uint32_t addr)
{
    uint16_t i = 0U, j;
    if (l->n >= MLOG_SLOTS) return;
    while ((i < l->n) && (((int16_t)(l->idx[i].seq - seq)) > 0)) i++;           /* keep newer ones in front */
    for (j = l->n; j > i; j--) l->idx[j] = l->idx[j - 1U];
    l->idx[i].seq = seq; l->idx[i].addr = addr; l->n++;
}

void MLog_Init(MLog_t *l, const MFlash_t *fl, uint32_t page_a, uint32_t page_b)
{
    uint8_t b[MLOG_REC_BYTES]; uint16_t p, s; uint8_t have = 0U; uint16_t best_seq = 0U; uint8_t best_page = 0U;
    uint16_t last_nonblank[2] = { 0U, 0U }; uint8_t any_nonblank[2] = { 0U, 0U };
    l->fl = fl; l->page[0] = page_a; l->page[1] = page_b; l->n = 0U; l->seq = 0U; l->active = 0U; l->next = 0U;
    for (p = 0U; p < 2U; p++)
    {
        for (s = 0U; s < MLOG_SLOTS_PAGE; s++)
        {
            uint32_t addr = l->page[p] + ((uint32_t)s * MLOG_REC_BYTES);
            if (fl->read(addr, b, MLOG_REC_BYTES) != 0) continue;
            if (blank(b) != 0U) continue;
            last_nonblank[p] = s; any_nonblank[p] = 1U;
            if (valid(b) != 0U)
            {
                uint16_t sq = r16(&b[12]);
                index_add(l, sq, addr);
                if ((have == 0U) || (((int16_t)(sq - best_seq)) > 0)) { best_seq = sq; best_page = (uint8_t)p; have = 1U; }
            }
        }
    }
    if (have != 0U)
    {
        l->seq = best_seq; l->active = best_page;
    }
    else
    {   /* no valid record: junk only, or blank. Append to a page that has no junk if there is one. */
        l->active = (any_nonblank[0] == 0U) ? 0U : ((any_nonblank[1] == 0U) ? 1U : 0U);
    }
    l->next = (any_nonblank[l->active] != 0U) ? (uint16_t)(last_nonblank[l->active] + 1U) : 0U;      /* after the last non-blank slot: junk is skipped, never overwritten */
}

int MLog_Append(MLog_t *l, const MLogRec_t *r)
{
    uint8_t b[MLOG_REC_BYTES]; uint32_t addr; uint16_t seq = (uint16_t)(l->seq + 1U);
    if (l->next >= MLOG_SLOTS_PAGE)
    {   /* the active page is full: erase the OTHER (older) page and continue there; at least 128 records stay */
        uint8_t other = (uint8_t)(l->active ^ 1U); uint16_t i, k = 0U;
        if (l->fl->erase(l->page[other]) != 0) return 1;
        for (i = 0U; i < l->n; i++)                                              /* drop the erased page's entries from the index */
            if ((l->idx[i].addr < l->page[other]) || (l->idx[i].addr >= (l->page[other] + MFLASH_PAGE))) l->idx[k++] = l->idx[i];
        l->n = k; l->active = other; l->next = 0U;
    }
    addr = l->page[l->active] + ((uint32_t)l->next * MLOG_REC_BYTES);
    encode(b, r, seq);
    l->next++;                                                                    /* the slot is used from now on, even if the write is cut */
    if (l->fl->prog(addr, b, MLOG_REC_BYTES) != 0) return 2;
    l->seq = seq;
    index_add(l, seq, addr);
    return 0;
}

uint16_t MLog_Count(const MLog_t *l) { return (l->n > MLOG_SHOWN) ? (uint16_t)MLOG_SHOWN : l->n; }

int MLog_Get(const MLog_t *l, uint16_t index, MLogRec_t *out)
{
    uint8_t b[MLOG_REC_BYTES];
    if (index >= MLog_Count(l)) return -1;
    if (l->fl->read(l->idx[index].addr, b, MLOG_REC_BYTES) != 0) return -1;
    if (valid(b) == 0U) return -1;
    decode(b, out);
    return 0;
}

int MLog_Clear(MLog_t *l)
{
    if (l->fl->erase(l->page[0]) != 0) return 1;
    if (l->fl->erase(l->page[1]) != 0) return 2;
    l->n = 0U; l->next = 0U; l->active = 0U;
    return 0;                                                                     /* the sequence keeps counting: it only orders what is there */
}
