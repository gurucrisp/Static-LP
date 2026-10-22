/**
  * @file    m_clock.h
  * @brief   Master core: date and time (dd/mm/yyyy HH:MM) for the fault log and the Set-Mode Date and Time items. PURE LOGIC: no hardware.
  *          The real-time clock peripheral (crystal + backup battery) is read and written by the glue (master_monitor.c); this file only does the calendar arithmetic.
  *          Years 2001..2099 are valid (2000 is the RTC's "never set" year). A fault-log record stores its time as MINUTES SINCE 2000-01-01 00:00 with the top bit set (MC_STAMP_FLAG)
  *          in the field that used to hold the running time in minutes: a record without the top bit is an old record (or one made while the clock was not set) and shows its running time.
  */
#ifndef M_CLOCK_H
#define M_CLOCK_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MC_YEAR_MIN     2001U
#define MC_YEAR_MAX     2099U
#define MC_STAMP_FLAG   0x80000000UL

typedef struct { uint16_t year; uint8_t month, day, hour, minute; } MDateTime_t;     /* month 1..12, day 1..31, hour 0..23, minute 0..59 */

uint8_t  MC_DaysInMonth(uint16_t year, uint8_t month);          /* 0 for a month outside 1..12 */
uint8_t  MC_Valid(const MDateTime_t *t);                        /* 1 if every field is in range, the day exists in that month (leap years) and the year is 2001..2099 */
uint32_t MC_ToMinutes(const MDateTime_t *t);                    /* minutes since 2000-01-01 00:00 (t must be valid) */
void     MC_FromMinutes(uint32_t minutes, MDateTime_t *t);      /* the inverse */
uint32_t MC_Stamp(const MDateTime_t *t);                        /* MC_ToMinutes | MC_STAMP_FLAG: the value stored in a log record */
uint8_t  MC_IsStamp(uint32_t run_min_field);                    /* 1 if a log record's field holds a clock time */
uint32_t MC_StampMinutes(uint32_t run_min_field);               /* the minutes since 2000 of a stamp */

#ifdef __cplusplus
}
#endif
#endif
