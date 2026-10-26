/**
  * @file    m_ntc.h
  * @brief   The heat-sink NTC: the slave's raw reading (12 bit ADC count of PA5) to temperature (v0.9.49). No hardware, no maths library.
  *
  * The circuit is 3.3 V -- 1.5 kohm -- PA5 -- 10 kohm NTC -- GND, read by the slave's ADC (the same 3.3 V reference, so the reading is ratiometric). The conversion is a
  * table (tools/gen_ntc_table.py, beta 3950 by default) with linear interpolation every 5 degC: within 1 degC of the beta equation. The NTC's own beta value decides how
  * close that is to the real temperature (a 3435 part reads about 7 degC cooler than it is at 75 degC): check one point against a thermometer.
  */
#ifndef M_NTC_H
#define M_NTC_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#define MNTC_OK     0U
#define MNTC_OPEN   1U      /* reading at the top of the scale: no NTC fitted / a broken wire (colder than the table: shown, never a trip) */
#define MNTC_SHORT  2U      /* reading near zero: the NTC or the wire is shorted (fail-safe: counted as overheat) */

/* raw = 0..4095. *t_dc = temperature in tenths of a degree Celsius, clamped to the table's ends (-10.0 .. 130.0), valid for MNTC_OK only. */
uint8_t MNtc_RawToDeci(uint16_t raw, int16_t *t_dc);

#ifdef __cplusplus
}
#endif
#endif
