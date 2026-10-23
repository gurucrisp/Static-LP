/**
  * @file    slave_profile.h
  * @brief   The transformer (autotune) PROFILE in the slave's flash: two CRC-protected copies in two pages (the same store as the voltage calibration, D20), validated every time it is loaded. Not touched by Factory Reset,
  *          by the calibration or by a normal firmware update; a programmer's MASS erase wipes it (then the next first start tunes again). A power cut during a save leaves the old profile or the new one, never a mixture.
  *          PURE LOGIC on top of slave_store: the flash is reached through SFlash_t, so the PC tests can lose power in the middle of any write.
  */
#ifndef SLAVE_PROFILE_H
#define SLAVE_PROFILE_H
#include <stdint.h>
#include "slave_store.h"
#include "slave_tune.h"
#ifdef __cplusplus
extern "C" {
#endif

#define SPROFILE_PAYLOAD  64U          /* the profile is padded to this (TUNE_BYTES(12) = 61) */
#define SPROFILE_VERSION  1U

void    SlaveProfile_Init(const SFlash_t *fl, uint32_t page_a, uint32_t page_b);     /* once at start-up: loads and validates what is stored */
uint8_t SlaveProfile_Tuned(void);                                                    /* 1 = a valid profile is stored ($H tuned) */
uint8_t SlaveProfile_Get(TuneProfile_t *p);                                          /* 1 = p holds the valid profile */
/* Store a profile: 0 = stored and now the active one; -1 = the profile is not valid (nothing written); 1, 2, 3 = the flash failed at that stage of the store (the previous profile stays) */
int     SlaveProfile_Save(const TuneProfile_t *p);

#ifdef __cplusplus
}
#endif
#endif
