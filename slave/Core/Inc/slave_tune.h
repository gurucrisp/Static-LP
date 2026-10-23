/**
  * @file    slave_tune.h
  * @brief   Transformer AUTOTUNE, the profile it produces, and the start-up MODEL CHECK (design D13 to D21 of the protocol workbook). PURE LOGIC: no hardware, no flash, no link. The controller feeds it the measurements
  *          and applies the duty it asks for; the PC tests run it against a simulated family of transformers.
  *
  *          THE SWEEP (no load, the master keeps the output contactor open, input steady): a zero-duty baseline of 3 s, then the effective on-time is stepped up (16, 32, 64, ... 2752 ticks), 3 s a step (1.5 s settle,
  *          1.5 s averaged). Every step is checked BEFORE it is applied: it is shortened so that its predicted injection stays at or below 80 % of the installer's Max Correction. The sweep stops at once, with nothing stored, on:
  *          a flipped transformer, no response, a fault / FLT / break, an over-voltage or an unsteady input (more than 1.5 V peak to peak); and it ends with a stored profile at the end of the list, at 80 % of Max Correction,
  *          or at a saturation knee (the slope falls below 40 % of the previous one: the cap is then the step before the knee).
  *
  *          THE PROFILE: the measured curve (injection at the sweep's input voltage for each step), the cap (ticks), and the regulator gain scale (the loop gain follows the transformer's gain: scale = reference gain / measured gain,
  *          limited to 0.1 .. 1.5). How the injection changes with the INPUT voltage is taken from the shape of a reference curve measured on the bench (the one the host tests use): the sweep is made at one input voltage only.
  *
  *          THE MODEL CHECK (every start): the injection measured during the soft start is compared with what the profile predicts for that input voltage. Wrong sign = flipped; off by more than the tolerance = the transformer
  *          no longer matches the profile (the master then shows "Re-tune required" and the slave refuses closed loop until it has been re-tuned: it NEVER re-tunes by itself).
  */
#ifndef SLAVE_TUNE_H
#define SLAVE_TUNE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* ---- $T result codes (link_v2.h LINKV2_T_*) ---- */
#define TUNE_R_STEP        0U
#define TUNE_R_FINISHED    1U
#define TUNE_R_FLIPPED     2U
#define TUNE_R_NO_RESPONSE 3U
#define TUNE_R_ABORTED     4U
#define TUNE_R_UNSTEADY    5U
#define TUNE_R_BY_MASTER   6U

/* ---- timing and limits (numbers I chose; see the workbook, to be confirmed on the bench) ---- */
#define TUNE_BASE_MS         3000UL
#define TUNE_STEP_MS         3000UL
#define TUNE_SETTLE_MS       1500UL      /* the first part of a step is not averaged */
#define TUNE_WAIT_STEADY_MS  30000UL     /* how long to wait for a steady input before giving up */
#define TUNE_STEADY_PP_V     1.5f        /* the input may move this much (peak to peak) during a window */
#define TUNE_STALE_MS        300UL       /* no valid measurement for this long: abort */
#define TUNE_LIMIT_FRAC      0.80f       /* no step is predicted to inject more than this fraction of Max Correction */
#define TUNE_OVERSHOOT_V     5.0f        /* the output more than this above input + Max Correction: abort */
#define TUNE_KNEE_FRAC       0.40f       /* a slope below this fraction of the previous one is a saturation knee */
#define TUNE_NO_RESP_V       0.5f        /* after a step of 256 ticks or more the injection must be at least this */
#define TUNE_FLIP_V          1.0f        /* an injection more negative than this (after 128 ticks or more) is a flipped transformer */
#define TUNE_SCALE_MIN       0.1f
#define TUNE_SCALE_MAX       1.5f
#define TUNE_N_STEPS         10U
#define TUNE_MAX_POINTS      12U
#define TUNE_VERSION         1U
#define TUNE_BUCK_RATIO      2.0f        /* the buck gain is taken as this times the boost gain (the reference design's ratio terms, 1.8 / 0.9): an ASSUMPTION for the reach only */

typedef struct
{
    uint8_t  version;                    /* TUNE_VERSION */
    uint8_t  n;                          /* measured points */
    uint8_t  knee;                       /* 1 = a saturation knee limited the cap */
    uint16_t vin_ref_dv;                 /* the input voltage of the sweep, tenths of a volt */
    uint16_t maxc_dv;                    /* the Max Correction it was made with, tenths of a volt */
    uint16_t cap_ticks;                  /* the duty cap */
    uint16_t kib_m, kiu_m;               /* regulator gain scale for boost and buck, thousandths */
    uint16_t ticks[TUNE_MAX_POINTS];
    int16_t  dv_dv[TUNE_MAX_POINTS];     /* the injection at that step, tenths of a volt, at vin_ref */
} TuneProfile_t;

#define TUNE_BYTES(n)  (13U + 4U * (uint32_t)(n))     /* the serialised size (<= 61 bytes for 12 points; the store takes 96) */

typedef struct
{
    float    vin, vout;                  /* volts */
    uint8_t  meas_valid;
    uint8_t  fault;                      /* FLT, break, any latched fault, over-voltage ... : abort */
    uint8_t  vin_in_window;              /* the input is inside the slave's window */
    float    maxc_v;                     /* Max Correction, volts (0 = not set) */
    uint16_t hw_cap_ticks;               /* the hardware duty ceiling (CTRL_STAGE3_MAX_EFF_TICKS) */
} TuneIn_t;

typedef struct
{
    uint8_t  active;                     /* the tune is running: the controller applies request_ticks as an OPEN-LOOP boost duty and keeps its regulator off */
    uint16_t request_ticks;              /* effective on-time to apply now (0 = none) */
    uint8_t  frame_ready;                /* a $T frame is ready (taken by Tune_TakeFrame) */
    uint8_t  done;                       /* finished, successfully or not: the result is in the last frame */
} TuneOut_t;

typedef struct { uint8_t step; uint16_t ticks; uint16_t vin_dv; int16_t dv_dv; uint8_t result; } TuneFrame_t;

/* start: 0 = started, 1 = Max Correction not set, 2 = already running */
uint8_t Tune_Start(float maxc_v, uint32_t now_ms);
void    Tune_Abort(void);                                           /* $K TABORT: stops at once, nothing stored, result 6 */
void    Tune_Service(uint32_t now_ms, const TuneIn_t *in, TuneOut_t *out);   /* call every measurement update (10 ms) */
uint8_t Tune_TakeFrame(TuneFrame_t *f);                             /* 1 = a frame was waiting */
uint8_t Tune_GetProfile(TuneProfile_t *p);                          /* 1 = the profile of the last FINISHED sweep */
uint8_t Tune_Running(void);

/* ---- the profile ---- */
float   Tune_RefG(float vin);                                       /* the bench curve's gain, volts per tick, at this input voltage (the shape of the injection against the input) */
float   Tune_PredictDv(const TuneProfile_t *p, float ticks, float vin);   /* the injection the profile predicts, volts, at this input voltage */
uint8_t Tune_ProfileValid(const TuneProfile_t *p);                  /* 1 = in range: n, cap, scales, monotonic ticks */
uint32_t Tune_Serialize(const TuneProfile_t *p, uint8_t *buf, uint32_t max);   /* bytes written, 0 = no room */
uint8_t Tune_Deserialize(const uint8_t *buf, uint32_t len, TuneProfile_t *p);  /* 1 = ok and valid */
uint16_t Tune_ReachLoDv(const TuneProfile_t *p, float setv);        /* lowest input (tenths of a volt) from which the set voltage can be reached at the cap; 0 = unknown */
uint16_t Tune_ReachHiDv(const TuneProfile_t *p, float setv);        /* highest input (tenths of a volt) from which it can be reached (buck, see TUNE_BUCK_RATIO); 0 = unknown */

/* ---- the model check at start-up ---- */
#define TUNE_CHECK_OK        0U
#define TUNE_CHECK_FLIP      1U      /* wrong sign */
#define TUNE_CHECK_MISMATCH  2U      /* the gain differs from the profile by more than the tolerance */
#define TUNE_CHECK_SKIP      3U      /* too little predicted injection to judge */
#ifndef TUNE_CHECK_TOL_REL
#define TUNE_CHECK_TOL_REL   0.30f   /* relative tolerance: a first value, to be set from the bench (D13) */
#endif
#ifndef TUNE_CHECK_TOL_ABS_V
#define TUNE_CHECK_TOL_ABS_V 1.0f
#endif
uint8_t Tune_ModelCheck(const TuneProfile_t *p, float ticks, float vin, float dv_measured);

#ifdef __cplusplus
}
#endif
#endif
