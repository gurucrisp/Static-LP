void AutoPID_Compute(AutoPID *pid)
{
    if (!pid->enabled) return;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - pid->lastTime;
    if (elapsed < pid->timeStepMs) return;
    float dt = elapsed / 1000.0f;

    // const float freeze_min = *(pid->setpoint) - 0.5f;
     const float freeze_min = *(pid->setpoint) - 0.75f;
     const float freeze_max = *(pid->setpoint) + 0.75f;

    static bool was_in_freeze = false;
    static uint32_t freeze_pending_since_ms = 0U;
    static bool freeze_pending_state = false;

    bool raw_in_band = (vrms_in >= freeze_min && vrms_in <= freeze_max);
    if (was_in_freeze)
    {
		raw_in_band = (vrms_in >= (freeze_min - 0.75f) && vrms_in <= (freeze_max + 0.75f));
	}

    /* Debounce the freeze/unfreeze transition. Without this, vrms_in
     * sitting close to the boundary (real mains noise/ripple, ADC/RMS
     * measurement jitter) can cross it every few samples, repeatedly
     * snapping pwm_output between 0 and active regulation - seen as
     * continuous "regulating -> pulses off -> regulating" cycling. The
     * raw condition must hold steady for FREEZE_DEBOUNCE_MS before the
     * actual freeze state is allowed to change; until then, keep
     * whatever state was already in effect. */
    bool in_freeze_band = was_in_freeze;
    if (raw_in_band != was_in_freeze)
    {
        if ((freeze_pending_since_ms == 0U) || (freeze_pending_state != raw_in_band))
        {
            freeze_pending_since_ms = now;
            freeze_pending_state    = raw_in_band;
        }
        else if ((now - freeze_pending_since_ms) >= FREEZE_DEBOUNCE_MS)
        {
            in_freeze_band = raw_in_band;   /* confirmed - take effect now */
        }
    }
    else
    {
        freeze_pending_since_ms = 0U;   /* raw condition matches current state already */
    }

    if (in_freeze_band)
    {
    	*(pid->output) = 0.0f;
        pid->integral = 0.0f;
         pid->derivative = 0.0f;
        pid->lastTime = now;
        was_in_freeze = true;
        pid_freeze_active = 1U;   /* TEMPORARY bench-diagnostic observability */
        // Leave pid->integral, pid->derivative and pid->lastError untouched for smoother resume.
        return;
    }

    was_in_freeze = false;
    pid_freeze_active = 0U;   /* TEMPORARY bench-diagnostic observability */
    // --- normal PID processing (not frozen)

    // compute soft mode blending 't' around setpoint
    float band = 0.2f;
    float t = 0.5f * (1.0f + (vrms_in - *(pid->setpoint)) / band);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    // gains interpolation
//    float Kp_boost = ratio_bb * 2.0f, Ki_boost = ratio_bb * (1.5f * 2600.0f), Kd_boost = 1.0f;
//    float Kp_buck  = ratio_bb * 1.0f, Ki_buck  = ratio_bb * (1.5f * 2000.0f) , Kd_buck  = 0.6f;

//    float Kp_boost = ratio_boost * 2.0f, Ki_boost = ratio_boost * (1.5f * 2600.0f), Kd_boost = 1.0f;  1.8
//    float Kp_buck  =  ratio_buck * 1.0f, Ki_buck  = ratio_buck * (1.5f * 2000.0f) , Kd_buck  = 0.6f;  2.3

    float Kp_boost = ratio_boost * 2.0f, Ki_boost = ratio_boost * (1.5f * 2600.0f), Kd_boost = 1.0f;  // 1.2
    float Kp_buck  =  ratio_buck * 1.0f, Ki_buck  = ratio_buck * (1.5f * 2000.0f) , Kd_buck  = 0.6f;  // 2.3

    float d_alpha = 0.9f;

    pid->Kp = (1.0f - t) * Kp_boost + t * Kp_buck;
    pid->Ki = (1.0f - t) * Ki_boost + t * Ki_buck;
    pid->Kd = (1.0f - t) * Kd_boost + t * Kd_buck;
    pid->Kp *= (float)g_kp_scale;
    pid->Kd *= (float)g_kd_scale;

    // error calculation (do NOT force input to setpoint)
    float boostErr = *(pid->setpoint) - *(pid->input);
    float buckErr  = *(pid->input)   - *(pid->setpoint);
    float error_raw = (1.0f - t) * boostErr + t * buckErr;
    float error = error_raw;

    // derivative (filtered)
    float deriv_unf = (error - pid->lastError) / dt;
    pid->derivative = d_alpha * pid->derivative + (1.0f - d_alpha) * deriv_unf;
    // compute unclamped output for anti-windup check
    float out_unclamped = pid->Kp * error + pid->integral + pid->Kd * pid->derivative;

    // conditional integration (anti-windup)
    if (!((out_unclamped > pid->outputMax && error > 0.0f) ||
          (out_unclamped < pid->outputMin && error < 0.0f)))
    {
        pid->integral += pid->Ki * error * dt;
    }
    else
    {
        // small leak to avoid permanent wind-up
        pid->integral *= 0.999f;
    }

    // clamp integral
    if (pid->integral > pid->integralMax) pid->integral = pid->integralMax;
    if (pid->integral < pid->integralMin) pid->integral = pid->integralMin;
    // final output (signed)
    float out = pid->Kp * error + pid->integral + pid->Kd * pid->derivative;
    // clamp final output to allowed range (preserve sign)
    if (out > pid->outputMax) out = pid->outputMax;
    if (out < pid->outputMin) out = pid->outputMin;

    // Store output and remember it for freeze-holding
    *(pid->output) = out;

    pid->lastOutput = out;    // <-- IMPORTANT: persist the exact last value (signed)
    pid->lastError = error;
    pid->lastTime = now;

}
