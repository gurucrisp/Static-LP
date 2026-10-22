/** @file slave_flash.c  @brief Flash access of the calibration record on the STM32G0B1. See slave_flash.h. */
#include "slave_features.h"
#if (SLAVE_VOLTAGE_CAL != 0)
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

static uint8_t allowed(uint32_t addr, uint32_t n)
{   /* only the two calibration pages, and only above the program image */
    if ((addr < SFLASH_CAL_PAGE_A) || ((addr + n) > (SFLASH_CAL_PAGE_B + SFL_PAGE))) return 0U;
    if (SlaveFlash_ImageEnd() > SFLASH_CAL_PAGE_A) return 0U;                       /* the image has grown into the calibration pages: refuse everything */
    return 1U;
}

static int fl_read(uint32_t addr, uint8_t *buf, uint32_t n) { memcpy(buf, (const void *)(uintptr_t)addr, n); return 0; }

static int fl_prog(uint32_t addr, const uint8_t *buf, uint32_t n)
{
    uint32_t i;
    if ((((addr | n) & 7UL) != 0UL) || (allowed(addr, n) == 0U)) return -1;
    HAL_FLASH_Unlock();
    for (i = 0U; i < n; i += 8U)
    {
        uint64_t d; memcpy(&d, buf + i, 8U);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, d) != HAL_OK) { HAL_FLASH_Lock(); return -1; }
    }
    HAL_FLASH_Lock();
    return 0;
}

static int fl_erase(uint32_t page_addr)
{
    FLASH_EraseInitTypeDef e; uint32_t err = 0U; HAL_StatusTypeDef st; uint8_t dual = 0U;
    if ((((page_addr - SFL_BASE) % SFL_PAGE) != 0UL) || (allowed(page_addr, SFL_PAGE) == 0U)) return -1;
#ifdef FLASH_OPTR_DBANK
    dual = (uint8_t)(((FLASH->OPTR & FLASH_OPTR_DBANK) != 0U) ? 1U : 0U);            /* dual-bank mode: the pages are numbered inside each bank */
#endif
    e.TypeErase = FLASH_TYPEERASE_PAGES;
    if ((dual != 0U) && (page_addr >= SFL_BANK2_BASE)) { e.Banks = FLASH_BANK_2; e.Page = (page_addr - SFL_BANK2_BASE) / SFL_PAGE; }
    else { e.Banks = FLASH_BANK_1; e.Page = (page_addr - SFL_BASE) / SFL_PAGE; }
    e.NbPages = 1U;
    HAL_FLASH_Unlock();
    st = HAL_FLASHEx_Erase(&e, &err);
    HAL_FLASH_Lock();
    return ((st == HAL_OK) && (err == 0xFFFFFFFFUL)) ? 0 : -1;
}

const SFlash_t g_slave_flash = { fl_read, fl_prog, fl_erase };
#endif
