/* Tests for master_rtc.c against a software model of the STM32G070 RTC registers. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "main.h"
#include "master_rtc.h"
RCC_TypeDef rcc_m; PWR_TypeDef pwr_m; RTC_TypeDef rtc_m;
static uint32_t tick; static int lse_works = 1; static uint32_t inits_when_dr_year;
static int tot = 0, bad = 0;
#define CHECK(c, ...) do { tot++; if (!(c)) { bad++; printf("      FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
/* the "hardware": every poll advances time and lets the peripheral react */
uint32_t HAL_GetTick(void)
{
    tick += 1U;
    if ((rcc_m.BDCR & 1UL) && lse_works && tick > 500U) rcc_m.BDCR |= 2UL;                   /* the crystal needs a while to start */
    if (rcc_m.CSR & 1UL) rcc_m.CSR |= 2UL;                                                   /* LSI ready */
    if (rtc_m.ICSR & 0x80UL) rtc_m.ICSR |= 0x40UL; else rtc_m.ICSR &= ~0x40UL;               /* INITF follows INIT */
    rtc_m.ICSR |= 0x20UL;                                                                    /* RSF */
    if (((rtc_m.DR >> 16) & 0xFFUL) != 0UL) rtc_m.ICSR |= 0x10UL; else rtc_m.ICSR &= ~0x10UL;   /* INITS: the year is not 00 */
    (void)inits_when_dr_year;
    return tick;
}
static void power_on_battery_lost(void) { memset(&rcc_m, 0, sizeof rcc_m); memset(&pwr_m, 0, sizeof pwr_m); memset(&rtc_m, 0, sizeof rtc_m); rtc_m.DR = 0x2101UL; tick = 0U; }   /* the reset value of DR: 2000-01-01 */
/* the real peripheral updates its status bits as soon as initialisation mode ends; the model does it on the next poll, so tests let one poll pass after a write */
static void settle(void) { (void)HAL_GetTick(); }
static MDateTime_t mk(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi) { MDateTime_t t; t.year = y; t.month = mo; t.day = d; t.hour = h; t.minute = mi; return t; }

int main(void)
{
    MDateTime_t t, u;
    /* first start, the crystal works: LSE chosen, the RTC enabled, the prescalers 127 / 255, 24 hour, the clock reads "not set" */
    power_on_battery_lost(); lse_works = 1; MRtc_Init();
    CHECK((pwr_m.CR1 & (1UL << 8)) != 0U && (rcc_m.APBENR1 & ((1UL << 28) | (1UL << 10))) == ((1UL << 28) | (1UL << 10)), "the power and RTC bus clocks are on and the backup domain is writable");
    CHECK(MRtc_UsingLse() == 1U && ((rcc_m.BDCR >> 8) & 3UL) == 1UL && (rcc_m.BDCR & (1UL << 15)) != 0U, "the crystal is chosen as the RTC clock and the RTC is enabled (BDCR %08lX)", (unsigned long)rcc_m.BDCR);
    CHECK(rtc_m.PRER == ((127UL << 16) | 255UL) && (rtc_m.CR & (1UL << 6)) == 0U && (rtc_m.ICSR & 0x80UL) == 0U, "prescalers 127 / 255 (32768 Hz), 24 hour, initialisation mode left (PRER %08lX)", (unsigned long)rtc_m.PRER);
    CHECK(MRtc_Get(&t) == 0U, "a clock that was never set reads 'not set'");
    /* set and read back: the BCD encoding, the weekday, every kind of date */
    t = mk(2026U, 10U, 21U, 14U, 32U); MRtc_Set(&t);
    CHECK(rtc_m.TR == 0x00143200UL, "14:32 is written as BCD 0x143200, seconds 0 (TR %08lX)", (unsigned long)rtc_m.TR);
    CHECK((rtc_m.DR & 0xFF1FFFUL) == 0x261021UL, "21/10/2026 is written as BCD (DR %08lX)", (unsigned long)rtc_m.DR);
    CHECK(((rtc_m.DR >> 13) & 7UL) == 3UL, "21 October 2026 is a Wednesday: weekday 3 (got %lu)", (unsigned long)((rtc_m.DR >> 13) & 7UL));
    settle(); CHECK(MRtc_Get(&u) == 1U && u.year == 2026U && u.month == 10U && u.day == 21U && u.hour == 14U && u.minute == 32U, "and it reads back");
    t = mk(2000U + 1U, 1U, 1U, 0U, 0U); MRtc_Set(&t); CHECK(((rtc_m.DR >> 13) & 7UL) == 1UL, "1 January 2001 was a Monday (weekday 1)");
    t = mk(2024U, 2U, 29U, 23U, 59U); MRtc_Set(&t); settle(); CHECK(MRtc_Get(&u) == 1U && u.year == 2024U && u.month == 2U && u.day == 29U && u.hour == 23U && u.minute == 59U, "29/02/2024 23:59 round trips");
    { int ok = 1; uint16_t y; uint8_t mo, d; for (y = 2001U; y <= 2099U && ok; y++) for (mo = 1U; mo <= 12U && ok; mo++) { d = MC_DaysInMonth(y, mo); t = mk(y, mo, d, (uint8_t)(y % 24U), (uint8_t)(mo * 4U)); MRtc_Set(&t); settle();
        if (!(MRtc_Get(&u) == 1U && u.year == t.year && u.month == t.month && u.day == t.day && u.hour == t.hour && u.minute == t.minute)) ok = 0; }
      CHECK(ok, "the last day of every month of 2001..2099 sets and reads back through the register encoding"); }
    { int ok = 1; uint16_t y; for (y = 2001U; y <= 2099U; y++) { MDateTime_t a = mk(y, 3U, 1U, 0U, 0U); uint32_t days = MC_ToMinutes(&a) / 1440UL; uint32_t wd = (days + 5UL) % 7UL + 1UL; MRtc_Set(&a); if (((rtc_m.DR >> 13) & 7UL) != wd) ok = 0; }
      CHECK(ok, "the weekday written is consistent for 99 years"); }
    /* an invalid date is never written */
    t = mk(2026U, 10U, 21U, 14U, 32U); MRtc_Set(&t); { uint32_t before_tr = rtc_m.TR, before_dr = rtc_m.DR; MDateTime_t bad_t = mk(2026U, 2U, 30U, 1U, 1U); MRtc_Set(&bad_t); CHECK(rtc_m.TR == before_tr && rtc_m.DR == before_dr, "31/02-style dates are refused by the driver"); }
    /* a later start with the clock running: left alone */
    { uint32_t tr = rtc_m.TR, dr = rtc_m.DR, bd = rcc_m.BDCR, prer = rtc_m.PRER; tick = 0U; MRtc_Init();
      CHECK(rtc_m.TR == tr && rtc_m.DR == dr && rcc_m.BDCR == bd && rtc_m.PRER == prer && MRtc_UsingLse() == 1U, "a later start leaves the running clock and its settings alone");
      CHECK(MRtc_Get(&u) == 1U && u.year == 2026U && u.day == 21U, "and the time is still there"); }
    /* the crystal does not start: after 3 s the internal LSI is used, flagged as such */
    power_on_battery_lost(); lse_works = 0; MRtc_Init();
    CHECK(MRtc_UsingLse() == 0U && ((rcc_m.BDCR >> 8) & 3UL) == 2UL && (rcc_m.BDCR & (1UL << 15)) != 0U && rtc_m.PRER == ((127UL << 16) | 249UL), "no crystal: the LSI is chosen, prescalers 127 / 249, and MRtc_UsingLse() says 0 (BDCR %08lX)", (unsigned long)rcc_m.BDCR);
    CHECK(tick >= 3000U, "it waited the full 3 s for the crystal first (%u)", (unsigned)tick);
    t = mk(2026U, 10U, 21U, 14U, 32U); MRtc_Set(&t); settle(); CHECK(MRtc_Get(&u) == 1U && u.hour == 14U, "the clock can still be set and read on the LSI");
    /* the battery is lost while the unit is off: the year is back to 2000, the clock reads 'not set' */
    rtc_m.DR = 0x2101UL; tick += 10U; (void)HAL_GetTick(); CHECK(MRtc_Get(&u) == 0U, "a year of 00 (the reset value) reads 'not set'");
    printf("  RTC driver (against a register model): crystal start, LSI fallback, BCD encoding and weekday for 99 years, invalid dates refused, a running clock left alone, 'not set' detected\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", tot, bad, bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
