/**
  * @file    slave_tunechk.h
  * @brief   The START-UP CHECK that the same transformer is fitted: while the regulator ramps up in BOOST, the injection that is really measured is compared with the one the stored profile predicts for that duty and input
  *          voltage (Tune_ModelCheck: 30 % or 1 V). A different transformer (or a changed winding) gives a different gain: MISMATCH. Pure logic, no hardware: the controller feeds it one fresh measurement at a time.
  *
  *          HOW IT AVOIDS FALSE ALARMS: the sensors' own offset is measured first (the first samples, almost no correction applied) and taken off; the duty is LOW-PASS FILTERED with the plant's time constant (the output lags the duty by
  *          about 0.3 s); a window of samples counts only while the duty is steady (it moved less than max(60 ticks, 10 %) since the window began), the correction is in the BOOST direction, the filtered duty is at least 150 ticks (the
  *          profile predicts at least 3 V) and the input did not jump (3 V); a MISMATCH needs TWO windows in a row. Buck is not checked: the profile describes the boost injection. A wrong SIGN is reported as FLIP (the
  *          controller treats it like its existing connected-backwards check).
  *          The tolerances (TUNE_CHECK_TOL_REL / _ABS_V in slave_tune.h) are FIRST VALUES, to be set from the bench.
  */
#ifndef SLAVE_TUNECHK_H
#define SLAVE_TUNECHK_H
#include <stdint.h>
#include "slave_tune.h"
#ifdef __cplusplus
extern "C" {
#endif

#define TUNECHK_BASE_TICKS     30.0f     /* below this duty the samples measure the sensors' offset */
#define TUNECHK_BASE_SAMPLES   8U
#define TUNECHK_MIN_TICKS      150.0f    /* the filtered duty must be at least this to judge (the profile then predicts about 3 V) */
#define TUNECHK_WINDOW         16U       /* fresh samples per window (about 0.3 s) */
#define TUNECHK_STEADY_ABS     60.0f     /* the duty may move this much (or 10 %) inside a window */
#define TUNECHK_JUMP_V         3.0f      /* an input change larger than this between samples restarts the window */
#define TUNECHK_TAU_MS         330.0f    /* the plant's lag, for filtering the duty */
#define TUNECHK_STRIKES        2U        /* mismatching windows in a row before MISMATCH is reported */
#define TUNECHK_WATCH_MS       25000UL   /* give up (no verdict) this long after the start */

#define TUNECHK_CONT      0U             /* no verdict yet */
#define TUNECHK_OK        1U             /* the same transformer: checking is over */
#define TUNECHK_MISMATCH  2U             /* a different gain: report it, stay off */
#define TUNECHK_FLIP      3U             /* the wrong sign */

typedef struct
{
    TuneProfile_t p;
    uint8_t  have, state, strikes;
    uint32_t t0_ms, last_ms;
    float    base_sum, base, d_f, vin_prev;
    uint32_t base_n, wn;
    float    w_dv, w_d, w_vin, w_d0;
} TuneChk_t;

void    TuneChk_Init(TuneChk_t *c, const TuneProfile_t *p, uint32_t now_ms);                    /* p = NULL: no profile, the check stays silent */
uint8_t TuneChk_Update(TuneChk_t *c, uint32_t now_ms, float d, uint8_t boost, float vin, float vout);   /* one FRESH measurement */
/* v0.9.68: end the check WITHOUT a verdict (the load is being connected: the profile describes the transformer at no load, and under load the injection is lower for the
 * same duty, so a judgement now would be a false MISMATCH). Nothing happens if a verdict was already reached. */
void    TuneChk_Stop(TuneChk_t *c);

#ifdef __cplusplus
}
#endif
#endif
