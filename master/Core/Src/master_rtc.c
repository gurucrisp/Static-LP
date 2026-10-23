/** @file master_rtc.c  @brief The STM32G070 real-time clock by direct register access. See master_rtc.h. */
#include "main.h"
#include "master_rtc.h"

/* register bits (RM0454): numbers, so that nothing depends on a CMSIS macro name */
#define APBENR1_PWREN      (1UL << 28)
#define APBENR1_RTCAPBEN   (1UL << 10)
#define PWR_CR1_DBP        (1UL << 8)
#define BDCR_LSEON         (1UL << 0)
#define BDCR_LSERDY        (1UL << 1)
#define BDCR_RTCSEL_LSE    (1UL << 8)
#define BDCR_RTCSEL_LSI    (2UL << 8)
#define BDCR_RTCEN         (1UL << 15)
#define BDCR_BDRST         (1UL << 16)
#define CSR_LSION          (1UL << 0)
#define CSR_LSIRDY         (1UL << 1)
#define ICSR_INITS         (1UL << 4)
#define ICSR_RSF           (1UL << 5)
#define ICSR_INITF         (1UL << 6)
#define ICSR_INIT          (1UL << 7)
#define CR_FMT             (1UL << 6)

static uint8_t s_lse, s_ok;

static uint8_t wait_bit(volatile uint32_t *reg, uint32_t bit, uint32_t want_set, uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((((*reg & bit) != 0U) ? 1U : 0U) != want_set) { if ((uint32_t)(HAL_GetTick() - t0) >= ms) return 0U; }
    return 1U;
}

void MRtc_Init(void)
{
    RCC->APBENR1 |= (APBENR1_PWREN | APBENR1_RTCAPBEN);          /* the power and RTC bus clocks */
    PWR->CR1 |= PWR_CR1_DBP;                                      /* the backup domain may be written */
    if ((RCC->BDCR & BDCR_RTCEN) == 0U)
    {   /* the RTC has never been enabled, or the battery was lost: nothing to keep. Pick the clock, then set the prescalers. */
        RCC->BDCR |= BDCR_BDRST; RCC->BDCR &= ~BDCR_BDRST;       /* reset the backup domain so that the clock source can be written */
        RCC->BDCR |= BDCR_LSEON;
        s_lse = wait_bit(&RCC->BDCR, BDCR_LSERDY, 1U, 3000UL);
        if (s_lse != 0U) RCC->BDCR |= BDCR_RTCSEL_LSE;
        else { RCC->CSR |= CSR_LSION; (void)wait_bit(&RCC->CSR, CSR_LSIRDY, 1U, 100UL); RCC->BDCR |= BDCR_RTCSEL_LSI; }
        RCC->BDCR |= BDCR_RTCEN;
        RTC->WPR = 0xCAUL; RTC->WPR = 0x53UL;
        RTC->ICSR |= ICSR_INIT;
        if (wait_bit(&RTC->ICSR, ICSR_INITF, 1U, 100UL) != 0U)
        {
            RTC->PRER = (127UL << 16) | ((s_lse != 0U) ? 255UL : 249UL);       /* 128 x 256 = 32768 Hz (the LSI is only about 32 kHz) */
            RTC->CR &= ~CR_FMT;                                                  /* 24 hour */
            RTC->ICSR &= ~ICSR_INIT;
        }
        RTC->WPR = 0xFFUL;
    }
    else s_lse = (uint8_t)(((RCC->BDCR >> 8) & 3UL) == 1UL);      /* a running clock is left alone; which source it uses */
    RTC->ICSR &= ~ICSR_RSF; (void)wait_bit(&RTC->ICSR, ICSR_RSF, 1U, 100UL);   /* the shadow registers are synchronised before the first read */
    s_ok = 1U;
}

uint8_t MRtc_UsingLse(void) { return s_lse; }

void MRtc_Status(uint8_t *src, uint16_t *bdcr, uint16_t *icsr)
{
    uint32_t sel = (RCC->BDCR >> 8) & 3UL;
    *src = (uint8_t)((s_ok == 0U) ? 0U : ((sel == 1UL) ? 1U : ((sel == 2UL) ? 2U : 0U)));      /* what the backup domain says it uses, not what we hoped for */
    *bdcr = (uint16_t)(RCC->BDCR & 0xFFFFUL); *icsr = (uint16_t)(RTC->ICSR & 0xFFFFUL);
}

uint8_t MRtc_Get(MDateTime_t *t)
{
    uint32_t tr, dr;
    if (s_ok == 0U) return 0U;
    if ((RTC->ICSR & ICSR_INITS) == 0U) return 0U;                /* the year is still 2000: never set (or the battery was lost) */
    tr = RTC->TR; dr = RTC->DR;                                    /* the time first, then the date: that keeps the pair consistent */
    t->hour   = (uint8_t)(((tr >> 20) & 3UL) * 10UL + ((tr >> 16) & 15UL));
    t->minute = (uint8_t)(((tr >> 12) & 7UL) * 10UL + ((tr >> 8) & 15UL));
    t->day    = (uint8_t)(((dr >> 4) & 3UL) * 10UL + (dr & 15UL));
    t->month  = (uint8_t)(((dr >> 12) & 1UL) * 10UL + ((dr >> 8) & 15UL));
    t->year   = (uint16_t)(2000UL + ((dr >> 20) & 15UL) * 10UL + ((dr >> 16) & 15UL));
    return MC_Valid(t);
}

void MRtc_Set(const MDateTime_t *t)
{
    uint32_t tr, dr, wd, yy;
    if ((s_ok == 0U) || (MC_Valid(t) == 0U)) return;
    yy = (uint32_t)(t->year - 2000U);
    wd = ((MC_ToMinutes(t) / 1440UL) + 5UL) % 7UL + 1UL;           /* 2000-01-01 was a Saturday; the RTC counts Monday = 1 .. Sunday = 7 */
    tr = ((uint32_t)(t->hour / 10U) << 20) | ((uint32_t)(t->hour % 10U) << 16) | ((uint32_t)(t->minute / 10U) << 12) | ((uint32_t)(t->minute % 10U) << 8);       /* seconds 0 */
    dr = ((yy / 10UL) << 20) | ((yy % 10UL) << 16) | (wd << 13) | ((uint32_t)(t->month / 10U) << 12) | ((uint32_t)(t->month % 10U) << 8) | ((uint32_t)(t->day / 10U) << 4) | (uint32_t)(t->day % 10U);
    RTC->WPR = 0xCAUL; RTC->WPR = 0x53UL;
    RTC->ICSR |= ICSR_INIT;
    if (wait_bit(&RTC->ICSR, ICSR_INITF, 1U, 100UL) != 0U) { RTC->TR = tr; RTC->DR = dr; RTC->ICSR &= ~ICSR_INIT; }
    RTC->WPR = 0xFFUL;
}

/* The battery-backed backup registers of the TAMP block (the running-hours counter lives in them). A switch, not pointer arithmetic: nothing depends on how the CMSIS structure is laid out. */
uint32_t MRtc_BkpRead(uint8_t reg)
{
    switch (reg) { case 0U: return TAMP->BKP0R; case 1U: return TAMP->BKP1R; case 2U: return TAMP->BKP2R; case 3U: return TAMP->BKP3R; case 4U: return TAMP->BKP4R; default: return 0UL; }
}

void MRtc_BkpWrite(uint8_t reg, uint32_t value)
{
    switch (reg) { case 0U: TAMP->BKP0R = value; break; case 1U: TAMP->BKP1R = value; break; case 2U: TAMP->BKP2R = value; break; case 3U: TAMP->BKP3R = value; break; case 4U: TAMP->BKP4R = value; break; default: break; }
}
