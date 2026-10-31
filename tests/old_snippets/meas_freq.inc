uint32_t SlaveMeas_WindowFrames(float period_us)
{
    float    f = (period_us / (float)MEAS_SAMPLE_PERIOD_US) + 0.5f;
    uint32_t n = (f < 1.0f) ? 1U : (uint32_t)f;
    if (n > MEAS_WINDOW_MAX) n = MEAS_WINDOW_MAX;
    return n;
}

float SlaveMeas_MainsPeriodFromZc(uint32_t p)
{
    if ((p >= 15000U) && (p <= 27000U))   /* one rising edge per mains cycle (37-66 Hz; v0.9.76: was 25000 = exactly 40 Hz) */
    {
        return (float)p;
    }
    if ((p >= 7500U) && (p <= 13500U))    /* one rising edge per half-cycle (74-133 Hz)    */
    {
        return 2.0f * (float)p;
    }
    return 0.0f;
}
