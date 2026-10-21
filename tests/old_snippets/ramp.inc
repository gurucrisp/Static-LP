static inline void ramp_setpoint_slope_ts(volatile float *sp,
                                          float target,
                                          float slope_V_per_s,
                                          uint32_t *last_ms)
{
    uint32_t now;
    uint32_t elapsed_ms;
    float dt;
    float step;
    float new_sp;

    /*
     * Basic input protection.
     * slope must always be positive.
     */
    if ((sp == NULL) || (last_ms == NULL) || (slope_V_per_s <= 0.0f))
    {
        return;
    }

    now = HAL_GetTick();

    /*
     * First execution after ramp initialization.
     * Capture the current time without changing the setpoint.
     */
    if (*last_ms == 0U)
    {
        *last_ms = now;
        return;
    }

    /*
     * Unsigned subtraction safely handles HAL_GetTick() rollover.
     */
    elapsed_ms = (uint32_t)(now - *last_ms);

    if (elapsed_ms == 0U)
    {
        return;
    }

    /*
     * Save the timestamp for the next ramp calculation.
     */
    *last_ms = now;

    /*
     * Convert elapsed time from milliseconds to seconds.
     */
    dt = (float)elapsed_ms * 0.001f;

    /*
     * Prevent a large setpoint jump if the main loop was delayed
     * by blocking code, UART activity, relay handling, or any
     * other lengthy operation.
     */
    if (dt > SOFTSTART_MAX_DT_S)
    {
        dt = SOFTSTART_MAX_DT_S;
    }

    /*
     * Calculate the allowed voltage change during this execution.
     */
    step = slope_V_per_s * dt;

    /*
     * Take a local copy of the current PID setpoint.
     */
    new_sp = *sp;

    /*
     * Ramp upward for boost operation.
     */
    if (target > new_sp)
    {
        new_sp += step;

        if (new_sp > target)
        {
            new_sp = target;
        }
    }
    /*
     * Ramp downward for buck operation.
     */
    else if (target < new_sp)
    {
        new_sp -= step;

        if (new_sp < target)
        {
            new_sp = target;
        }
    }
    /*
     * Setpoint has already reached the target.
     */
    else
    {
        return;
    }

    /*
     * Protect the shared floating-point variable because it is
     * also accessed from the TIM4 PID interrupt.
     */
    __disable_irq();
    *sp = new_sp;
    __enable_irq();
}
