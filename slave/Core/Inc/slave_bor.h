/**
  ******************************************************************************
  * @file    slave_bor.h
  * @brief   v0.9.79: the brown-out reset (BOR) option bytes, checked at every power-up. HEADER ONLY (no project change), PURE C: the PC tests run the decision.
  *
  * Why: a slave lost its IGBTs AND its STM32G0B1 during continuous output-phase switching. The slave is powered from its own phase; the STM32G0 leaves the factory
  * with BOR OFF, so on a sagging supply the MCU keeps running far below a safe voltage (only the power-down reset at about 1.7 V stops it) and its gate pins can do
  * anything. With BOR on at level 3 the MCU is held in reset below about 2.8 V (falling) and released above about 2.9 V (rising).
  *
  * What (SlaveFlash_EnsureBor, slave_flash.c, at the top of main): BOR set at the level or higher = nothing to do. Not set = write ONLY the BOR fields of the
  * user option byte (FLASH->OPTR; every other bit, the read protection included, is kept) and reload the option bytes, which resets the MCU once. If the boot that
  * follows an option-byte reload still finds BOR off, it does NOT try again (no reset loop): it runs and reports it (LINKV2_FLAG_NO_BOR, the master shows NOBOR).
  ******************************************************************************
  */
#ifndef SLAVE_BOR_H
#define SLAVE_BOR_H

#include <stdint.h>

/* FLASH_OPTR of the STM32G0B1 (ST's stm32g0b1xx.h): RDP [7:0], BOR_EN bit 8, BORR_LEV [10:9] (RISING), BORF_LEV [12:11] (FALLING).
 * v0.9.80: v0.9.79 had the two level fields the other way round; the build guard in slave_flash.c refused it on the real headers (both are written to the same
 * level, so the bits written would have been the same) */
#define BOR_OPTR_EN        (1UL << 8)
#define BOR_OPTR_R_POS     9U
#define BOR_OPTR_F_POS     11U
#define BOR_OPTR_MASK      (BOR_OPTR_EN | (3UL << BOR_OPTR_F_POS) | (3UL << BOR_OPTR_R_POS))
#ifndef SLAVE_BOR_LEVEL
#define SLAVE_BOR_LEVEL    3U          /* 0..3: falling about 2.0 / 2.2 / 2.5 / 2.8 V, rising about 2.1 / 2.3 / 2.6 / 2.9 V */
#endif
#if (SLAVE_BOR_LEVEL > 3U)
#error "SLAVE_BOR_LEVEL: 0 to 3"
#endif

#define BOR_OK        0U               /* BOR is on at the level (or higher) */
#define BOR_PROGRAM   1U               /* write it, then reload the option bytes (the MCU resets) */
#define BOR_GAVE_UP   2U               /* still off right after a reload: do not try again, report it */

static inline uint8_t Bor_IsSet(uint32_t optr)
{
    return (uint8_t)(((optr & BOR_OPTR_EN) != 0UL) && (((optr >> BOR_OPTR_F_POS) & 3UL) >= SLAVE_BOR_LEVEL) && (((optr >> BOR_OPTR_R_POS) & 3UL) >= SLAVE_BOR_LEVEL));
}
/* the option register as it must be: only the BOR fields changed */
static inline uint32_t Bor_Wanted(uint32_t optr)
{
    return (optr & ~BOR_OPTR_MASK) | BOR_OPTR_EN | ((uint32_t)SLAVE_BOR_LEVEL << BOR_OPTR_F_POS) | ((uint32_t)SLAVE_BOR_LEVEL << BOR_OPTR_R_POS);
}
/* after_reload = this boot follows an option-byte reload (RCC_CSR OBLRSTF) */
static inline uint8_t Bor_Decide(uint32_t optr, uint8_t after_reload)
{
    if (Bor_IsSet(optr) != 0U) return BOR_OK;
    if (after_reload != 0U)    return BOR_GAVE_UP;
    return BOR_PROGRAM;
}

#endif /* SLAVE_BOR_H */
