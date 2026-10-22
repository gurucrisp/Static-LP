/**
  * @file    slave_store.h
  * @brief   Slave: a copy of the master core store (renamed so both can be linked in one test): power-fail-safe record storage in flash (settings and calibration use it; the fault log has its own append-only layout, m_log.h).
  *          PURE LOGIC: the flash is reached through SFlash_t, so the PC tests can cut the power at every byte of a write.
  *
  * Two slots, one flash page each. A save erases the slot that is NOT the current one, programs the record, and programs a COMMIT word LAST. A slot counts only if its commit word, version,
  * length and CRC-16 are all right; of two valid slots the one with the higher sequence number wins. So a power cut at any moment leaves the old record or the new one, never a mixture.
  */
#ifndef SLAVE_STORE_H
#define SLAVE_STORE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define SFLASH_PAGE        2048U     /* STM32G070: 2 KB pages; programming is by 8-byte double words into erased (0xFF) bytes */
#define SSTORE_MAX_PAYLOAD 96U

typedef struct
{
    int (*read)(uint32_t addr, uint8_t *buf, uint32_t n);
    int (*prog)(uint32_t addr, const uint8_t *buf, uint32_t n);   /* addr and n multiples of 8; the bytes must be erased; 0 = ok */
    int (*erase)(uint32_t page_addr);                             /* one page; 0 = ok */
} SFlash_t;

typedef struct
{
    const SFlash_t *fl;
    uint32_t  addr[2];          /* the two page addresses */
    uint8_t   version;          /* a record of another version is ignored (the caller then uses defaults) */
    uint8_t   payload_len;      /* bytes, at most SSTORE_MAX_PAYLOAD */
    uint32_t  seq;              /* sequence of the newest valid slot */
    int8_t    cur;              /* index of the newest valid slot, -1 = none */
} SStore_t;

void    SStore_Init(SStore_t *s, const SFlash_t *fl, uint32_t page_a, uint32_t page_b, uint8_t version, uint8_t payload_len);
/* 1 = a valid record was found and copied to payload, 0 = none (blank, damaged or another version) */
uint8_t SStore_Load(SStore_t *s, uint8_t *payload);
/* 0 = saved, non-zero = a flash call failed (the previous record, if any, is still intact) */
int     SStore_Save(SStore_t *s, const uint8_t *payload);
uint16_t S_Crc16(const uint8_t *p, uint32_t n);

#ifdef __cplusplus
}
#endif
#endif
