/* Tests for m_clock: the calendar arithmetic behind the fault-log time stamps and the Set-Mode Date / Time items. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "m_clock.h"
static int tot = 0, bad = 0;
#define CHECK(c, ...) do { tot++; if (!(c)) { bad++; printf("      FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
static MDateTime_t mk(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi) { MDateTime_t t; t.year = y; t.month = mo; t.day = d; t.hour = h; t.minute = mi; return t; }
static uint32_t rs = 99U; static uint32_t rnd(void) { rs = rs * 1664525U + 1013904223U; return rs >> 8; }

int main(void)
{
    uint32_t y, mo; MDateTime_t t, u;
    /* days in month, leap years: every 4th year, but not 1900/2100 style centuries, and 2000 is a leap year */
    CHECK(MC_DaysInMonth(2024U, 2U) == 29U && MC_DaysInMonth(2025U, 2U) == 28U && MC_DaysInMonth(2000U, 2U) == 29U && MC_DaysInMonth(2100U, 2U) == 28U && MC_DaysInMonth(1900U, 2U) == 28U, "February in 2024, 2025, 2000, 2100, 1900");
    CHECK(MC_DaysInMonth(2026U, 1U) == 31U && MC_DaysInMonth(2026U, 4U) == 30U && MC_DaysInMonth(2026U, 6U) == 30U && MC_DaysInMonth(2026U, 9U) == 30U && MC_DaysInMonth(2026U, 11U) == 30U && MC_DaysInMonth(2026U, 12U) == 31U, "the month lengths");
    CHECK(MC_DaysInMonth(2026U, 0U) == 0U && MC_DaysInMonth(2026U, 13U) == 0U, "a month outside 1..12 has no days");
    { uint32_t sum = 0U; for (mo = 1U; mo <= 12U; mo++) sum += MC_DaysInMonth(2026U, (uint8_t)mo); CHECK(sum == 365U, "a common year has 365 days"); sum = 0U; for (mo = 1U; mo <= 12U; mo++) sum += MC_DaysInMonth(2028U, (uint8_t)mo); CHECK(sum == 366U, "a leap year 366"); }
    /* validity */
    t = mk(2026U, 10U, 21U, 14U, 32U); CHECK(MC_Valid(&t), "21/10/2026 14:32 is valid");
    t = mk(2024U, 2U, 29U, 0U, 0U); CHECK(MC_Valid(&t), "29/02/2024 is valid"); t = mk(2025U, 2U, 29U, 0U, 0U); CHECK(!MC_Valid(&t), "29/02/2025 is not");
    t = mk(2026U, 4U, 31U, 0U, 0U); CHECK(!MC_Valid(&t), "31/04 is not"); t = mk(2026U, 4U, 30U, 23U, 59U); CHECK(MC_Valid(&t), "30/04 23:59 is valid");
    t = mk(2026U, 4U, 0U, 0U, 0U); CHECK(!MC_Valid(&t), "day 0 is not"); t = mk(2026U, 0U, 1U, 0U, 0U); CHECK(!MC_Valid(&t), "month 0 is not"); t = mk(2026U, 13U, 1U, 0U, 0U); CHECK(!MC_Valid(&t), "month 13 is not");
    t = mk(2026U, 1U, 1U, 24U, 0U); CHECK(!MC_Valid(&t), "hour 24 is not"); t = mk(2026U, 1U, 1U, 0U, 60U); CHECK(!MC_Valid(&t), "minute 60 is not");
    t = mk(2000U, 1U, 1U, 0U, 0U); CHECK(!MC_Valid(&t), "2000 is the RTC's 'never set' year: not valid"); t = mk(2001U, 1U, 1U, 0U, 0U); CHECK(MC_Valid(&t), "2001 is");
    t = mk(2099U, 12U, 31U, 23U, 59U); CHECK(MC_Valid(&t), "2099-12-31 23:59 is the last valid minute"); t = mk(2100U, 1U, 1U, 0U, 0U); CHECK(!MC_Valid(&t), "2100 is not");
    /* anchors */
    t = mk(2000U, 1U, 1U, 0U, 0U); CHECK(MC_ToMinutes(&t) == 0U, "2000-01-01 00:00 is minute 0");
    t = mk(2000U, 1U, 1U, 0U, 1U); CHECK(MC_ToMinutes(&t) == 1U, "one minute later is 1"); t = mk(2000U, 1U, 2U, 0U, 0U); CHECK(MC_ToMinutes(&t) == 1440U, "one day later is 1440");
    t = mk(2001U, 1U, 1U, 0U, 0U); CHECK(MC_ToMinutes(&t) == 366UL * 1440UL, "2000 had 366 days");
    t = mk(2026U, 10U, 21U, 14U, 32U); { uint32_t m = MC_ToMinutes(&t); MC_FromMinutes(m, &u); CHECK(u.year == 2026U && u.month == 10U && u.day == 21U && u.hour == 14U && u.minute == 32U, "21/10/2026 14:32 comes back from its minutes"); }
    /* the whole range: every day, with random times, round-trips; the minute counter is strictly increasing day by day, and the last minute is the maximum */
    { uint32_t prev = 0U, k = 0U, days = 0U; int ok = 1, mono = 1;
      for (y = 2001U; y <= 2099U; y++) for (mo = 1U; mo <= 12U; mo++) { uint8_t d, dm = MC_DaysInMonth((uint16_t)y, (uint8_t)mo);
          for (d = 1U; d <= dm; d++) { uint8_t h = (uint8_t)(rnd() % 24U), mi = (uint8_t)(rnd() % 60U); uint32_t m;
              t = mk((uint16_t)y, (uint8_t)mo, d, h, mi); m = MC_ToMinutes(&t); MC_FromMinutes(m, &u);
              if (!(u.year == t.year && u.month == t.month && u.day == t.day && u.hour == t.hour && u.minute == t.minute)) ok = 0;
              { MDateTime_t mid = mk((uint16_t)y, (uint8_t)mo, d, 0U, 0U); uint32_t mm = MC_ToMinutes(&mid); if (k != 0U && mm <= prev) mono = 0; prev = mm; k++; }
              days++; } }
      CHECK(ok, "every day of 2001..2099 with a random time round-trips through minutes"); CHECK(mono, "midnight of each day is later than the one before"); CHECK(days == 36159U || days == 36160U, "99 years of days counted (%u)", (unsigned)days); }
    /* random minutes round-trip and are valid */
    { int ok = 1; uint32_t k; for (k = 0U; k < 200000U; k++) { uint32_t m = 366UL * 1440UL + (rnd() % (99UL * 366UL * 1440UL)); MC_FromMinutes(m, &t); if (!MC_Valid(&t) || MC_ToMinutes(&t) != m) { ok = 0; break; } }
      CHECK(ok, "200,000 random minutes in 2001..2099: the date is valid and converts back exactly"); }
    /* stamps */
    t = mk(2026U, 10U, 21U, 14U, 32U); { uint32_t s = MC_Stamp(&t); CHECK(MC_IsStamp(s) && !MC_IsStamp(MC_ToMinutes(&t)) && MC_StampMinutes(s) == MC_ToMinutes(&t), "a stamp carries the top bit; the minutes are recoverable"); }
    CHECK(!MC_IsStamp(0U) && !MC_IsStamp(7425U) && !MC_IsStamp(0x7FFFFFFFUL), "an old running-time value is never mistaken for a stamp");
    t = mk(2099U, 12U, 31U, 23U, 59U); CHECK(MC_ToMinutes(&t) < MC_STAMP_FLAG, "even the last valid minute fits below the stamp bit");
    printf("  clock: leap years and month lengths, validity edges, every day of 2001..2099 round trips, 200,000 random minutes, stamps\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", tot, bad, bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
