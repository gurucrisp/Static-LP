/**
  ******************************************************************************
  * @file    slave_reg.h
  * @brief   Output-voltage regulator for the slave: INTEGRAL controller with
  *          buck/boost gain scheduling + freeze band + soft-start ramp.
  *          PURE C - no hardware access.
  *
  * This is the control law of the old, hardware-tested firmware WITHOUT its
  * proportional and derivative terms. Why they are gone (design study in
  * tests/study_pid.c, ORIGINAL controller against a plant with the real
  * structure - static gain, one duty update per half-cycle, 20 ms RMS
  * measurement, noise):
  *   - P:  gain about 0.3 counts/V against about 600 counts/(V*s) for the
  *         integral term - it contributes nothing.
  *   - D:  worse than useless. It differentiates a delayed, noisy RMS value,
  *         doubles the duty jitter, and at high plant gain (0.06 V per duty
  *         count and above) makes the loop OSCILLATE where the integral
  *         controller stays perfectly stable.
  * Everything else is the old behaviour on purpose. tests/test_reg_equiv.c
  * compiles the ORIGINAL functions with their P and D gains forced to zero and
  * proves this code gives bit-identical results to them.
  *
  * MEASURE THE PLANT GAIN AT STAGE 2: the fixed 96-count duty moves the output
  * by g * 96 volts, so g = (change in Vout) / 96. The integral gain is tuned for
  * g up to about 0.1 V per count; if you measure more than that, reduce Ki.
  *
  * WHAT IS REGULATED
  *   - feedback (PID input)  : vout, the measured OUTPUT voltage
  *   - target                : sp_v, a RAMPED setpoint that starts at the
  *                             present input voltage and slides to target_v
  *   - mode (buck / boost)   : decided from the INPUT voltage vin relative to
  *                             sp_v (boost when vin < sp_v)
  *   - output                : duty in timer counts, 0 .. REG_PWM_PERIOD
  *                             (same magnitude is used on both polarities)
  *   - freeze band           : while the INPUT is within +/-0.75 V of the
  *                             ramped setpoint (+/-1.5 V once frozen) no
  *                             correction is applied at all (duty 0).
  *
  * Call Reg_Service() every main-loop pass.  The PID itself only executes
  * every REG_TIMESTEP_MS (5 ms), like the old firmware.
  ******************************************************************************
  */
#ifndef SLAVE_REG_H
#define SLAVE_REG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "slave_config.h"        /* so that REG_KI_SCALE and REG_FREEZE_DEBOUNCE_MS can be set there */

/* ---- Constants carried over from the old firmware ------------------------ */
#define REG_PWM_PERIOD            3199.0f     /* PWM_PERIOD of the old firmware (ARR) */
#define REG_TIMESTEP_MS           5U          /* controller execution period          */
#ifndef REG_FREEZE_DEBOUNCE_MS
#define REG_FREEZE_DEBOUNCE_MS    300U        /* FREEZE_DEBOUNCE_MS: how long the input must stay inside / outside the +-0.75 V band before the correction freezes / resumes. The original value; it is the delay before a SMALL input change (under 15 V) is answered. */
#endif
#define REG_FREEZE_BAND_V         0.75f
#define REG_BLEND_BAND_V          0.2f        /* buck/boost gain blending band        */
/* Scale of the integral gain (both boost and buck). 1.0 = the original firmware's gain (the host tests compare against it). DO NOT RAISE IT without measuring: the gain also sets how far the output moves the WRONG way
 * (a transformer connected backwards adds instead of subtracts) before the flip check stops the unit. Measured in the simulation, buck unit, limit of the test 3.5 V: x1.0 = 1.46 V, x1.5 = 3.38 V (almost no margin left),
 * x1.75 = 4.9 V, x2.0 = 6.8 V (the flip test fails). The simulation also showed NO gain in speed from a higher gain; the guard ramp (CTRL_GUARD_RESTART_SLOPE_VPS) is what limits the recovery. */
#ifndef REG_KI_SCALE
#define REG_KI_SCALE              1.0f
#endif
#define REG_RATIO_BUCK            ((1.8f) / 5.78f)   /* transformer ratio terms        */
#define REG_RATIO_BOOST           ((0.9f) / 5.78f)

#define REG_SOFTSTART_SLOW_TIME_MS    2000U
#define REG_SOFTSTART_SLOW_SLOPE_VPS  3.0f
#define REG_SOFTSTART_FAST_SLOPE_VPS  10.0f
#define REG_SOFTSTART_FINAL_SLOPE_VPS 5.0f
#define REG_SOFTSTART_FINAL_BAND_V    3.0f
#define REG_SOFTSTART_MAX_DT_S        0.050f

typedef struct
{
    /* setpoints */
    float    target_v;          /* voltage_setpoint      : final goal, e.g. 230 V  */
    float    sp_v;              /* voltage_setpoint_pid  : ramped setpoint          */

    /* integral controller */
    float    ki;                /* scheduled integral gain, counts per (volt * second) */
    float    out_min, out_max;
    float    integral;          /* IS the output (clamped to out_min..out_max) */
    uint32_t last_time_ms;
    uint32_t step_ms;
    uint8_t  enabled;
    float    output;            /* pwm_output: duty in timer counts */

    /* freeze-band debounce (static locals of the old function) */
    uint8_t  was_in_freeze;
    uint8_t  freeze_pending_state;
    uint32_t freeze_pending_since_ms;
    uint8_t  freeze_active;     /* pid_freeze_active: 1 while duty is held at 0 */

    /* soft-start ramp settings: set by Reg_Init to the OLD firmware's constants (REG_SOFTSTART_*), the controller may override them (slave_config.h) */
    float    ss_slow_slope_vps;    /* V/s during the first ss_slow_time_ms */
    float    ss_fast_slope_vps;    /* V/s afterwards, while the setpoint is more than REG_SOFTSTART_FINAL_BAND_V from the target */
    float    ss_final_slope_vps;   /* V/s in the last REG_SOFTSTART_FINAL_BAND_V */
    uint32_t ss_slow_time_ms;

    /* soft-start ramp (static locals of the old main-loop block) */
    uint32_t ramp_last_ms;
    uint32_t softstart_begin_ms;
    uint8_t  was_armed;
} Reg_t;

/* Initialise. The ramped setpoint starts at vin (so the first correction
 * demand is ~0), the integrator is cleared and enabled. */
void  Reg_Init(Reg_t *r, float target_v, float vin, float vout, uint32_t now_ms);

/* One main-loop pass: soft-start ramp, then the controller, in the old order.
 * armed = regulation allowed (inverter running). When not armed the ramped
 * setpoint simply follows vin, which keeps the duty at 0. */
void  Reg_Service(Reg_t *r, uint32_t now_ms, float vin, float vout, uint8_t armed);

/* Duty request in timer counts, 0 .. REG_PWM_PERIOD (before any limiter). */
float Reg_Duty(const Reg_t *r);
/* The transformer profile's gain scale (reference gain / measured gain), for boost and for buck: multiplies the integral gain of that direction. 1.0 = unchanged (the default, and what every host test uses). Survives Reg_Init. */
void  Reg_SetGainScale(float boost, float buck);
void  Reg_GetGainScale(float *boost, float *buck);

/* Boost when the INPUT is below the ramped setpoint (old: vrms_in < voltage_setpoint_pid). */
uint8_t Reg_IsBoost(const Reg_t *r, float vin);

/* Exposed separately so the host test can drive them one by one. */
void  Reg_Step(Reg_t *r, uint32_t now_ms, float vin, float vout);
void  Reg_RampStep(Reg_t *r, uint32_t now_ms, float slope_v_per_s);

#ifdef __cplusplus
}
#endif

#endif /* SLAVE_REG_H */
