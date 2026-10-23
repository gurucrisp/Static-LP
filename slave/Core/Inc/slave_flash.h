/**
  * @file    slave_flash.h
  * @brief   The slave's flash access for the calibration record (STM32G0B1, 2 KB pages): pages 126 and 127 (the last two pages of the first 256 KB), 0x0803F000 and 0x0803F800, as the two slots of the store (slave_store.h).
  *          The driver refuses any address outside those two pages, so it can never touch the program. They are pages 126 and 127 of bank 1: the same page numbers in single-bank and in dual-bank mode. The program must end below 0x0803F000 (252 KB).
  *          Erasing or programming stalls the CPU for tens of milliseconds: the calibration only runs while the unit is stopped, so no interrupt that matters is waiting.
  */
#ifndef SLAVE_FLASH_H
#define SLAVE_FLASH_H
#include <stdint.h>
#include "slave_store.h"
#ifdef __cplusplus
extern "C" {
#endif

#define SFLASH_CAL_PAGE_A   0x0803F000UL
#define SFLASH_CAL_PAGE_B   0x0803F800UL

extern const SFlash_t g_slave_flash;
uint32_t SlaveFlash_Diag(void);            /* why the LAST failed erase / program failed, packed in decimal: reason x 1,000,000 + dual-bank flag x 100,000 + the HAL error flags or the failing page (16 bits).
                                              reason 1 = address refused by the guard, 2 = misaligned, 3 = the HAL returned an error (the flags are HAL_FLASH_GetError()), 4 = the erase reported a failing page */
uint32_t SlaveFlash_ImageEnd(void);        /* the first address after the program image in flash: the calibration pages must lie above it */

#ifdef __cplusplus
}
#endif
#endif
