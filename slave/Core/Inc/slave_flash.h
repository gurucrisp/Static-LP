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
uint32_t SlaveFlash_ImageEnd(void);        /* the first address after the program image in flash: the calibration pages must lie above it */

#ifdef __cplusplus
}
#endif
#endif
