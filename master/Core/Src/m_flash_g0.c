/** @file m_flash_g0.c  @brief STM32G070 flash driver for the master core. See m_flash_g0.h. */
#include "m_flash_g0.h"
#include "main.h"
#include <string.h>

/* The error flags to clear before a flash operation. FLASH_FLAG_ALL_ERRORS exists in some G0 HALs and not in the G070's, so the mask is built from the individual flags, each only if this HAL has it. */
#ifdef FLASH_FLAG_OPERR
#define MF_F1 FLASH_FLAG_OPERR
#else
#define MF_F1 0U
#endif
#ifdef FLASH_FLAG_PROGERR
#define MF_F2 FLASH_FLAG_PROGERR
#else
#define MF_F2 0U
#endif
#ifdef FLASH_FLAG_WRPERR
#define MF_F3 FLASH_FLAG_WRPERR
#else
#define MF_F3 0U
#endif
#ifdef FLASH_FLAG_PGAERR
#define MF_F4 FLASH_FLAG_PGAERR
#else
#define MF_F4 0U
#endif
#ifdef FLASH_FLAG_SIZERR
#define MF_F5 FLASH_FLAG_SIZERR
#else
#define MF_F5 0U
#endif
#ifdef FLASH_FLAG_PGSERR
#define MF_F6 FLASH_FLAG_PGSERR
#else
#define MF_F6 0U
#endif
#ifdef FLASH_FLAG_MISERR
#define MF_F7 FLASH_FLAG_MISERR
#else
#define MF_F7 0U
#endif
#ifdef FLASH_FLAG_FASTERR
#define MF_F8 FLASH_FLAG_FASTERR
#else
#define MF_F8 0U
#endif
#ifdef FLASH_FLAG_RDERR
#define MF_F9 FLASH_FLAG_RDERR
#else
#define MF_F9 0U
#endif
#ifdef FLASH_FLAG_OPTVERR
#define MF_F10 FLASH_FLAG_OPTVERR
#else
#define MF_F10 0U
#endif
#define MF_CLEAR_ERRORS() do { __HAL_FLASH_CLEAR_FLAG(MF_F1); __HAL_FLASH_CLEAR_FLAG(MF_F2); __HAL_FLASH_CLEAR_FLAG(MF_F3); __HAL_FLASH_CLEAR_FLAG(MF_F4); __HAL_FLASH_CLEAR_FLAG(MF_F5); \
                             __HAL_FLASH_CLEAR_FLAG(MF_F6); __HAL_FLASH_CLEAR_FLAG(MF_F7); __HAL_FLASH_CLEAR_FLAG(MF_F8); __HAL_FLASH_CLEAR_FLAG(MF_F9); __HAL_FLASH_CLEAR_FLAG(MF_F10); } while (0)

extern uint32_t _sidata, _sdata, _edata;        /* the linker's: where .data is stored in flash, and its RAM start and end */

uint32_t MFlashG0_ImageEnd(void)
{
    return (uint32_t)&_sidata + ((uint32_t)&_edata - (uint32_t)&_sdata);
}

static uint8_t window_ok(uint32_t addr, uint32_t n)
{
    if ((addr < MFLASHG0_LOW) || ((addr + n) > MFLASHG0_HIGH)) return 0U;       /* only our own pages */
    if (addr < MFlashG0_ImageEnd()) return 0U;                                  /* never into the program */
    return 1U;
}

static int g0_read(uint32_t addr, uint8_t *buf, uint32_t n)
{
    if ((addr < 0x08000000UL) || ((addr + n) > 0x08020000UL)) return 1;
    memcpy(buf, (const void *)addr, n);
    return 0;
}

static int g0_prog(uint32_t addr, const uint8_t *buf, uint32_t n)
{
    uint32_t i; int rc = 0;
    if (((addr | n) & 7U) != 0U) return 2;
    if (window_ok(addr, n) == 0U) return 3;
    HAL_FLASH_Unlock();
    MF_CLEAR_ERRORS();
    for (i = 0U; i < n; i += 8U)
    {
        uint64_t w; memcpy(&w, &buf[i], 8U);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, w) != HAL_OK) { rc = 4; break; }
    }
    HAL_FLASH_Lock();
    return rc;
}

static int g0_erase(uint32_t page_addr)
{
    FLASH_EraseInitTypeDef e; uint32_t err = 0U; int rc = 0;
    if ((page_addr % 2048U) != 0U) return 2;
    if (window_ok(page_addr, 2048U) == 0U) return 3;
    HAL_FLASH_Unlock();
    MF_CLEAR_ERRORS();
    e.TypeErase = FLASH_TYPEERASE_PAGES; e.Page = (page_addr - 0x08000000UL) / 2048U; e.NbPages = 1U;
    if (HAL_FLASHEx_Erase(&e, &err) != HAL_OK) rc = 4;
    HAL_FLASH_Lock();
    return rc;
}

static const MFlash_t s_iface = { g0_read, g0_prog, g0_erase };
const MFlash_t *MFlashG0_Iface(void) { return &s_iface; }
