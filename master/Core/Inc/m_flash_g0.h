/**
  * @file    m_flash_g0.h
  * @brief   The STM32G070 flash driver behind MFlash_t (hardware glue; the PC tests use a RAM model instead).
  *          SAFETY: it only ever erases or programs pages inside MFLASHG0_LOW .. MFLASHG0_HIGH (the pages of m_app.h, below the old firmware's settings page at 0x0801F800, which it never
  *          touches) AND only above the end of the program image (the linker's _sidata + the size of .data). If the program ever grows into those pages, saving fails with an error
  *          ("Save Failed") instead of destroying code.
  */
#ifndef M_FLASH_G0_H
#define M_FLASH_G0_H
#include "m_store.h"
#ifdef __cplusplus
extern "C" {
#endif

#define MFLASHG0_LOW    0x0801C800UL
#define MFLASHG0_HIGH   0x0801F800UL     /* exclusive */

const MFlash_t *MFlashG0_Iface(void);
uint32_t        MFlashG0_ImageEnd(void);   /* the first flash address after the program image */

#ifdef __cplusplus
}
#endif
#endif
