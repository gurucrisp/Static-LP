/** @file m_clock.c  @brief Date and time arithmetic. See m_clock.h. */
#include "m_clock.h"

static uint8_t is_leap(uint16_t y) { return (uint8_t)((((y % 4U) == 0U) && ((y % 100U) != 0U)) || ((y % 400U) == 0U)); }

uint8_t MC_DaysInMonth(uint16_t year, uint8_t month)
{
    static const uint8_t D[12] = { 31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U };
    if ((month < 1U) || (month > 12U)) return 0U;
    return (uint8_t)((month == 2U) ? (is_leap(year) ? 29U : 28U) : D[month - 1U]);
}

uint8_t MC_Valid(const MDateTime_t *t)
{
    if ((t->year < MC_YEAR_MIN) || (t->year > MC_YEAR_MAX)) return 0U;
    if ((t->month < 1U) || (t->month > 12U)) return 0U;
    if ((t->day < 1U) || (t->day > MC_DaysInMonth(t->year, t->month))) return 0U;
    if ((t->hour > 23U) || (t->minute > 59U)) return 0U;
    return 1U;
}

uint32_t MC_ToMinutes(const MDateTime_t *t)
{
    uint32_t days = 0U, y, m;
    for (y = 2000U; y < t->year; y++) days += is_leap((uint16_t)y) ? 366U : 365U;
    for (m = 1U; m < t->month; m++) days += MC_DaysInMonth(t->year, (uint8_t)m);
    days += (uint32_t)(t->day - 1U);
    return ((days * 24UL) + t->hour) * 60UL + t->minute;
}

void MC_FromMinutes(uint32_t minutes, MDateTime_t *t)
{
    uint32_t days = minutes / 1440UL, rem = minutes % 1440UL; uint16_t y = 2000U; uint8_t m = 1U, dm;
    for (;;) { uint32_t n = is_leap(y) ? 366UL : 365UL; if (days < n) break; days -= n; y++; if (y > 9999U) break; }
    for (;;) { dm = MC_DaysInMonth(y, m); if (days < dm) break; days -= dm; m++; if (m > 12U) { m = 12U; break; } }
    t->year = y; t->month = m; t->day = (uint8_t)(days + 1UL); t->hour = (uint8_t)(rem / 60UL); t->minute = (uint8_t)(rem % 60UL);
}

uint32_t MC_Stamp(const MDateTime_t *t) { return MC_ToMinutes(t) | MC_STAMP_FLAG; }
uint8_t  MC_IsStamp(uint32_t f) { return (uint8_t)((f & MC_STAMP_FLAG) != 0U); }
uint32_t MC_StampMinutes(uint32_t f) { return f & ~MC_STAMP_FLAG; }
