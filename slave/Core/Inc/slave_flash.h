/**
  * @file    slave_flash.h
  * @brief   The slave's flash access for the calibration record (STM32G0B1, 2 KB pages): the LAST TWO pages of the 512 KB flash, 0x0807F000 and 0x0807F800, as the two slots of the store (slave_store.h).
  *          The driver refuses any address outside those two pages, so it can never touch the program. It works in single-bank and in dual-bank mode (the page number is derived from the address and the DBANK option bit).
  *          Erasing or programming stalls the CPU for tens of milliseconds: the calibration only runs while the unit is stopped, so no interrupt that matters is waiting.
  */
#ifndef SLAVE_FLASH_H
#define SLAVE_FLASH_H
#include <stdint.h>
#include "slave_store.h"
#ifdef __cplusplus
extern "C" {
#endif

#define SFLASH_CAL_PAGE_A   0x0807F000UL
#define SFLASH_CAL_PAGE_B   0x0807F800UL

extern const SFlash_t g_slave_flash;
uint32_t SlaveFlash_Diag(void);            /* why the LAST failed erase / program failed, packed in decimal: reason x 1,000,000 + dual-bank flag x 100,000 + the HAL error flags or the failing page (16 bits).
                                              reason 1 = address refused by the guard, 2 = misaligned, 3 = the HAL returned an error (the flags are HAL_FLASH_GetError()), 4 = the erase reported a failing page */
uint32_t SlaveFlash_ImageEnd(void);        /* the first address after the program image in flash: the calibration pages must lie above it */

#ifdef __cplusplus
}
#endif
#endif
