/** @file m_store.c  @brief Power-fail-safe two-slot record store. See slave_store.h. */
#include "slave_store.h"

#define SLOT_MAGIC   0x5AC3U
#define COMMIT_WORD  0x434F4D4D49545F31ULL            /* "COMMIT_1" */

uint16_t S_Crc16(const uint8_t *p, uint32_t n)        /* CRC-16/CCITT-FALSE */
{
    uint16_t c = 0xFFFFU; uint32_t i; uint8_t b;
    for (i = 0U; i < n; i++)
    {
        c ^= (uint16_t)((uint16_t)p[i] << 8);
        for (b = 0U; b < 8U; b++) c = (uint16_t)(((c & 0x8000U) != 0U) ? (((uint32_t)c << 1) ^ 0x1021U) : ((uint32_t)c << 1));
    }
    return c;
}

static uint32_t pl_pad(const SStore_t *s) { return ((uint32_t)s->payload_len + 7U) & ~7U; }
static uint32_t body_len(const SStore_t *s) { return 8U + pl_pad(s) + 8U; }   /* header, payload, crc word */

void SStore_Init(SStore_t *s, const SFlash_t *fl, uint32_t page_a, uint32_t page_b, uint8_t version, uint8_t payload_len)
{
    s->fl = fl; s->addr[0] = page_a; s->addr[1] = page_b; s->version = version;
    s->payload_len = (payload_len > SSTORE_MAX_PAYLOAD) ? (uint8_t)SSTORE_MAX_PAYLOAD : payload_len;
    s->seq = 0U; s->cur = -1;
}

static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

/* returns 1 and fills payload/seq if the slot is a complete, valid record */
static uint8_t read_slot(const SStore_t *s, uint8_t slot, uint8_t *payload, uint32_t *seq)
{
    uint8_t buf[8U + SSTORE_MAX_PAYLOAD + 8U + 8U];
    uint32_t bl = body_len(s), i;
    uint16_t crc;
    if (s->fl->read(s->addr[slot], buf, bl + 8U) != 0) return 0U;
    if ((uint16_t)(buf[0] | (buf[1] << 8)) != SLOT_MAGIC) return 0U;
    if ((buf[2] != s->version) || (buf[3] != s->payload_len)) return 0U;
    for (i = 0U; i < 8U; i++) if (buf[bl + i] != (uint8_t)(COMMIT_WORD >> (8U * i))) return 0U;      /* no commit word: the write was cut */
    crc = (uint16_t)(buf[bl - 8U] | (buf[bl - 7U] << 8));
    if (S_Crc16(buf, bl - 8U) != crc) return 0U;
    for (i = 0U; i < s->payload_len; i++) payload[i] = buf[8U + i];
    *seq = get32(&buf[4]);
    return 1U;
}

uint8_t SStore_Load(SStore_t *s, uint8_t *payload)
{
    uint8_t a[SSTORE_MAX_PAYLOAD], b[SSTORE_MAX_PAYLOAD]; uint32_t sa = 0U, sb = 0U;
    uint8_t va = read_slot(s, 0U, a, &sa), vb = read_slot(s, 1U, b, &sb), pick, i;
    s->cur = -1;
    if ((va == 0U) && (vb == 0U)) return 0U;
    if (va && vb) pick = (((int32_t)(sa - sb)) >= 0) ? 0U : 1U; else pick = va ? 0U : 1U;
    s->cur = (int8_t)pick; s->seq = (pick == 0U) ? sa : sb;
    for (i = 0U; i < s->payload_len; i++) payload[i] = (pick == 0U) ? a[i] : b[i];
    return 1U;
}

int SStore_Save(SStore_t *s, const uint8_t *payload)
{
    uint8_t buf[8U + SSTORE_MAX_PAYLOAD + 8U], commit[8]; uint32_t bl = body_len(s), i; uint16_t crc;
    uint8_t slot = (s->cur == 0) ? 1U : 0U;                       /* the slot that is not the current one */
    for (i = 0U; i < bl; i++) buf[i] = 0xFFU;
    buf[0] = (uint8_t)(SLOT_MAGIC & 0xFFU); buf[1] = (uint8_t)(SLOT_MAGIC >> 8); buf[2] = s->version; buf[3] = s->payload_len;
    put32(&buf[4], s->seq + 1U);
    for (i = 0U; i < s->payload_len; i++) buf[8U + i] = payload[i];
    crc = S_Crc16(buf, bl - 8U);
    buf[bl - 8U] = (uint8_t)crc; buf[bl - 7U] = (uint8_t)(crc >> 8);
    for (i = 0U; i < 8U; i++) commit[i] = (uint8_t)(COMMIT_WORD >> (8U * i));
    if (s->fl->erase(s->addr[slot]) != 0) return 1;
    if (s->fl->prog(s->addr[slot], buf, bl) != 0) return 2;
    if (s->fl->prog(s->addr[slot] + bl, commit, 8U) != 0) return 3;    /* LAST: only now does the slot count */
    s->seq++; s->cur = (int8_t)slot;
    return 0;
}
