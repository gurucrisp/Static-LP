/**
  * @file    master_rtc.h
  * @brief   The master's real-time clock (STM32G070 RTC, 32.768 kHz crystal, backup battery on VBAT). Direct register access: it does not need the HAL RTC module or a CubeMX RTC configuration.
  *          First start (or the battery was lost): the 32.768 kHz crystal (LSE) is started and chosen as the RTC clock; if it does not start within 3 s the internal LSI (about 32 kHz, NOT accurate)
  *          is used instead and MRtc_UsingLse() says 0. The calendar then reads "not set" until Date and Time are entered in Set-Mode. Later starts leave the running clock alone.
  */
#ifndef MASTER_RTC_H
#define MASTER_RTC_H
#include <stdint.h>
#include "m_clock.h"
#ifdef __cplusplus
extern "C" {
#endif

void    MRtc_Init(void);                       /* once at start-up */
uint8_t MRtc_Get(MDateTime_t *t);              /* 1 = the clock has been set and *t is the date and time; 0 = never set (or lost) */
void    MRtc_Set(const MDateTime_t *t);        /* set date and time (seconds become 0); t must be valid (MC_Valid) */
void    MRtc_Status(uint8_t *src, uint16_t *bdcr, uint16_t *icsr);   /* for the Clock page: src 0 = not started, 1 = crystal (LSE), 2 = the internal LSI; the low 16 bits of RCC->BDCR and RTC->ICSR */
uint32_t MRtc_BkpRead(uint8_t reg);                     /* the battery-backed registers TAMP BKP0R..BKP4R (reg 0..4): only after MRtc_Init, which enables the backup-domain access */
void     MRtc_BkpWrite(uint8_t reg, uint32_t value);
uint8_t MRtc_UsingLse(void);                   /* 1 = running from the crystal, 0 = from the internal LSI (inaccurate) or not started */

#ifdef __cplusplus
}
#endif
#endif
