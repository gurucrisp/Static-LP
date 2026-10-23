/** @file slave_flash.c  @brief Flash access of the calibration record on the STM32G0B1. See slave_flash.h. */
#include "slave_features.h"
#if (SLAVE_VOLTAGE_CAL != 0) || (SLAVE_AUTOTUNE != 0)
#include <string.h>
#include "main.h"
#include "slave_flash.h"

#define SFL_BASE        0x08000000UL
#define SFL_BANK2_BASE  0x08040000UL
#define SFL_PAGE        2048UL

extern uint32_t _sidata, _sdata, _edata;                 /* from the linker script: where the initialised data is stored, and how big it is */

#ifndef SLAVE_FLASH_TEST_IMAGE_END                         /* a PC test supplies its own */
uint32_t SlaveFlash_ImageEnd(void) { return (uint32_t)(uintptr_t)&_sidata + (uint32_t)(((uintptr_t)&_edata) - ((uintptr_t)&_sdata)); }
#endif

#ifdef SLAVE_FLASH_TEST_IMAGE_END                         /* a PC test reads its model of the flash */
extern uint8_t SlaveFlashTest_Rd8(uint32_t addr);
#define SFL_RD8(a)   SlaveFlashTest_Rd8(a)
#else
#define SFL_RD8(a)   (*(const volatile uint8_t *)(uintptr_t)(a))
#endif

static uint32_t s_reason, s_err, s_dual;
uint32_t SlaveFlash_Diag(void) { return (s_reason * 1000000UL) + (s_dual * 100000UL) + (s_err & 0xFFFFUL); }
static void dual_now(void)
{
#ifdef FLASH_OPTR_DBANK
    s_dual = (uint32_t)(((FLASH->OPTR & FLASH_OPTR_DBANK) != 0U) ? 1U : 0U);
#else
    s_dual = 0U;
#endif
}
static int fail(uint32_t reason, uint32_t err) { s_reason = reason; s_err = err; return -1; }

static uint8_t allowed(uint32_t addr, uint32_t n)
{   /* only the two calibration pages, and only above the program image */
    if ((addr < SFLASH_PROF_PAGE_A) || ((addr + n) > (SFLASH_CAL_PAGE_B + SFL_PAGE))) return 0U;       /* only the four pages: profile A, B and calibration A, B */
    if (SlaveFlash_ImageEnd() > SFLASH_PROF_PAGE_A) return 0U;                      /* the image has grown into them: refuse everything */
    return 1U;
}

static int fl_read(uint32_t addr, uint8_t *buf, uint32_t n) { memcpy(buf, (const void *)(uintptr_t)addr, n); return 0; }

static int fl_prog(uint32_t addr, const uint8_t *buf, uint32_t n)
{
    uint32_t i;
    dual_now();
    if ((addr | n) & 7UL) return fail(2U, 0U);
    if (allowed(addr, n) == 0U) return fail(1U, 0U);
    HAL_FLASH_Unlock();
    for (i = 0U; i < n; i += 8U)
    {
        uint64_t d; memcpy(&d, buf + i, 8U);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, d) != HAL_OK) { uint32_t e = HAL_FLASH_GetError(); HAL_FLASH_Lock(); return fail(3U, e); }
    }
    HAL_FLASH_Lock();
    return 0;
}

static int fl_erase(uint32_t page_addr)
{
    FLASH_EraseInitTypeDef e; uint32_t err = 0U; HAL_StatusTypeDef st;
    dual_now();
    if (((page_addr - SFL_BASE) % SFL_PAGE) != 0UL) return fail(2U, 0U);
    if (allowed(page_addr, SFL_PAGE) == 0U) return fail(1U, 0U);
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    /* The page number is counted INSIDE the bank (0..127) whatever the flash mode, and the calibration pages are pages 126 and 127 of bank 1: the same number in single-bank and in dual-bank mode.
     * (v0.9.22 used the last two pages of the chip, 254 and 255 in single-bank mode: the page number field holds only 0..127, the erase hit another page and the calibration could be saved once only.) */
    if (page_addr >= SFL_BANK2_BASE) { e.Banks = FLASH_BANK_2; e.Page = (page_addr - SFL_BANK2_BASE) / SFL_PAGE; }
    else { e.Banks = FLASH_BANK_1; e.Page = (page_addr - SFL_BASE) / SFL_PAGE; }
    e.NbPages = 1U;
    HAL_FLASH_Unlock();
    st = HAL_FLASHEx_Erase(&e, &err);
    HAL_FLASH_Lock();
    if (st != HAL_OK) return fail(3U, HAL_FLASH_GetError());
    if (err != 0xFFFFFFFFUL) return fail(4U, err);                                  /* the page that failed */
    {   /* blank check: the HAL said OK; is the page really empty? (programming a page that is not blank fails: this is how "writes once, never again" would show) */
        uint32_t i; for (i = 0U; i < SFL_PAGE; i++) if (SFL_RD8(page_addr + i) != 0xFFU) return fail(5U, i);
    }
    return 0;
}

const SFlash_t g_slave_flash = { fl_read, fl_prog, fl_erase };
#endif
