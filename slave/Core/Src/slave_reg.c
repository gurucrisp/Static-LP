/**
  ******************************************************************************
  * @file    slave_reg.c
  * @brief   Output-voltage regulator (see slave_reg.h).
  *
  * KEEP THE ARITHMETIC EXACTLY AS WRITTEN: every expression below mirrors the
  * old firmware's AutoPID_Compute / ramp_setpoint_slope_ts (with its P and D
  * gains zero), including the order of floating-point operations, because
  * test_reg_equiv.c checks bit-for-bit equality against the original code.
  ******************************************************************************
  */
#include "slave_reg.h"

/* Gain scheduling constants, as in the old firmware (computed the same way). */
static const float s_ratio_buck  = REG_RATIO_BUCK;
static const float s_ratio_boost = REG_RATIO_BOOST;

void Reg_Init(Reg_t *r, float target_v, float vin, float vout, uint32_t now_ms)
{
    (void)vout;
    r->target_v = target_v;
    r->sp_v     = vin;                 /* start at the present input: first demand ~ 0 */

    r->ki       = 0.0f;
    r->out_min  = 0.0f;
    r->out_max  = REG_PWM_PERIOD;
    r->integral = 0.0f;

    r->last_time_ms = now_ms;
    r->step_ms      = REG_TIMESTEP_MS;
    r->enabled      = 1U;
    r->output       = 0.0f;

    r->was_in_freeze           = 0U;
    r->freeze_pending_state    = 0U;
    r->freeze_pending_since_ms = 0U;
    r->freeze_active           = 0U;

    r->ss_slow_slope_vps   = REG_SOFTSTART_SLOW_SLOPE_VPS;
    r->ss_fast_slope_vps   = REG_SOFTSTART_FAST_SLOPE_VPS;
    r->ss_final_slope_vps  = REG_SOFTSTART_FINAL_SLOPE_VPS;
    r->ss_slow_time_ms     = REG_SOFTSTART_SLOW_TIME_MS;

    r->ramp_last_ms        = 0U;
    r->softstart_begin_ms  = 0U;
    r->was_armed           = 0U;
}

void Reg_Step(Reg_t *r, uint32_t now, float vin, float vout)
{
    uint32_t elapsed;
    float    dt;
    float    freeze_min, freeze_max;
    uint8_t  raw_in_band;
    uint8_t  in_freeze_band;
    float    band, t;
    float    Ki_boost, Ki_buck;
    float    boostErr, buckErr, error;

    if (!r->enabled) return;

    elapsed = now - r->last_time_ms;
    if (elapsed < r->step_ms) return;
    dt = elapsed / 1000.0f;

    freeze_min = r->sp_v - REG_FREEZE_BAND_V;
    freeze_max = r->sp_v + REG_FREEZE_BAND_V;

    raw_in_band = (vin >= freeze_min && vin <= freeze_max);
    if (r->was_in_freeze)
    {
        raw_in_band = (vin >= (freeze_min - REG_FREEZE_BAND_V) && vin <= (freeze_max + REG_FREEZE_BAND_V));
    }

    /* Debounce the freeze/unfreeze transition: stops the duty snapping between 0
     * and active regulation when the INPUT sits on the band edge. */
    in_freeze_band = r->was_in_freeze;
    if (raw_in_band != r->was_in_freeze)
    {
        if ((r->freeze_pending_since_ms == 0U) || (r->freeze_pending_state != raw_in_band))
        {
            r->freeze_pending_since_ms = now;
            r->freeze_pending_state    = raw_in_band;
        }
        else if ((now - r->freeze_pending_since_ms) >= REG_FREEZE_DEBOUNCE_MS)
        {
            in_freeze_band = raw_in_band;
        }
    }
    else
    {
        r->freeze_pending_since_ms = 0U;
    }

    if (in_freeze_band)
    {
        r->output       = 0.0f;
        r->integral     = 0.0f;
        r->last_time_ms = now;
        r->was_in_freeze = 1U;
        r->freeze_active = 1U;
        return;
    }

    r->was_in_freeze = 0U;
    r->freeze_active = 0U;

    /* soft buck/boost blending 't' around the setpoint */
    band = REG_BLEND_BAND_V;
    t = 0.5f * (1.0f + (vin - r->sp_v) / band);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    Ki_boost = s_ratio_boost * (1.5f * 2600.0f);
    Ki_buck  = s_ratio_buck  * (1.5f * 2000.0f);
    r->ki = (1.0f - t) * Ki_boost + t * Ki_buck;

    boostErr = r->sp_v - vout;
    buckErr  = vout - r->sp_v;
    error    = (1.0f - t) * boostErr + t * buckErr;

    r->integral += r->ki * error * dt;
    if (r->integral > r->out_max) r->integral = r->out_max;     /* anti-windup: the clamp IS the limit */
    if (r->integral < r->out_min) r->integral = r->out_min;

    r->output       = r->integral;
    r->last_time_ms = now;
}

void Reg_RampStep(Reg_t *r, uint32_t now, float slope_v_per_s)
{
    uint32_t elapsed_ms;
    float    dt, step, new_sp;

    if (slope_v_per_s <= 0.0f) return;

    if (r->ramp_last_ms == 0U)
    {
        r->ramp_last_ms = now;
        return;
    }

    elapsed_ms = (uint32_t)(now - r->ramp_last_ms);
    if (elapsed_ms == 0U) return;

    r->ramp_last_ms = now;

    dt = (float)elapsed_ms * 0.001f;
    if (dt > REG_SOFTSTART_MAX_DT_S) dt = REG_SOFTSTART_MAX_DT_S;

    step   = slope_v_per_s * dt;
    new_sp = r->sp_v;

    if (r->target_v > new_sp)
    {
        new_sp += step;
        if (new_sp > r->target_v) new_sp = r->target_v;
    }
    else if (r->target_v < new_sp)
    {
        new_sp -= step;
        if (new_sp < r->target_v) new_sp = r->target_v;
    }
    else
    {
        return;
    }

    r->sp_v = new_sp;
}

void Reg_Service(Reg_t *r, uint32_t now, float vin, float vout, uint8_t armed)
{
    if (armed && !r->was_armed)
    {
        /* Bumpless start: ramped setpoint = present input, PID cleared. */
        (void)vout;
        r->sp_v        = vin;
        r->integral    = 0.0f;
        r->output      = 0.0f;

        r->ramp_last_ms       = now;
        r->softstart_begin_ms = now;
        r->was_armed          = 1U;
    }

    if (armed)
    {
        uint32_t softstart_elapsed_ms = (uint32_t)(now - r->softstart_begin_ms);
        float    voltage_difference   = r->sp_v - r->target_v;
        float    selected_slope;

        if (voltage_difference < 0.0f) voltage_difference = -voltage_difference;

        if (softstart_elapsed_ms < r->ss_slow_time_ms)               selected_slope = r->ss_slow_slope_vps;
        else if (voltage_difference <= REG_SOFTSTART_FINAL_BAND_V)   selected_slope = r->ss_final_slope_vps;
        else                                                         selected_slope = r->ss_fast_slope_vps;

        Reg_RampStep(r, now, selected_slope);
    }
    else
    {
        r->was_armed          = 0U;
        r->ramp_last_ms       = 0U;
        r->softstart_begin_ms = 0U;
        r->sp_v               = vin;
    }

    Reg_Step(r, now, vin, vout);
}

float Reg_Duty(const Reg_t *r)
{
    return r->output;
}

uint8_t Reg_IsBoost(const Reg_t *r, float vin)
{
    return (uint8_t)(vin < r->sp_v);
}
