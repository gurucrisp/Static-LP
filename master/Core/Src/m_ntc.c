/**
  * @file    m_ntc.c
  * @brief   NTC reading to temperature. See m_ntc.h.
  */
#include "m_ntc.h"
#include "m_ntc_table.h"

#define MNTC_SHORT_BELOW   150U     /* a reading this low is a short, not a hot sink (130 degC reads about 714) */

uint8_t MNtc_RawToDeci(uint16_t raw, int16_t *t_dc)
{
    uint32_t i;
    *t_dc = 0;
    if (raw > 4095U) raw = 4095U;
    if (raw < MNTC_SHORT_BELOW) { *t_dc = (int16_t)((MNTC_T_FIRST_C + (MNTC_N - 1) * MNTC_T_STEP_C) * 10); return MNTC_SHORT; }
    if (raw > s_ntc_raw[0]) { *t_dc = (int16_t)(MNTC_T_FIRST_C * 10); return MNTC_OPEN; }                      /* colder than -10 degC, or no NTC */
    if (raw <= s_ntc_raw[MNTC_N - 1U]) { *t_dc = (int16_t)((MNTC_T_FIRST_C + (MNTC_N - 1) * MNTC_T_STEP_C) * 10); return MNTC_OK; }      /* hotter than the table: clamped, far above any trip */
    for (i = 0U; i + 1U < MNTC_N; i++)
    {
        if ((raw <= s_ntc_raw[i]) && (raw >= s_ntc_raw[i + 1U]))
        {
            uint32_t span = (uint32_t)s_ntc_raw[i] - (uint32_t)s_ntc_raw[i + 1U], into = (uint32_t)s_ntc_raw[i] - (uint32_t)raw;
            int32_t base = (MNTC_T_FIRST_C + (int32_t)i * MNTC_T_STEP_C) * 10;
            *t_dc = (int16_t)(base + (int32_t)((into * (uint32_t)(MNTC_T_STEP_C * 10) + (span / 2U)) / span));
            return MNTC_OK;
        }
    }
    return MNTC_OPEN;
}
