int8_t SlaveMeas_SignFromRing(const volatile uint16_t *buf, uint32_t frames, uint32_t frames_done,
                              uint32_t channels, uint32_t ch, int32_t mean_sum,
                              uint32_t back, uint32_t span, int32_t min_abs_sum,
                              int32_t *dev_sum_out)
{
    uint32_t k, idx;
    int32_t  sum = 0, dev;

    if ((frames == 0U) || (span == 0U))
    {
        if (dev_sum_out != 0) *dev_sum_out = 0;
        return 0;
    }

    for (k = 0U; k < span; k++)
    {
        /* newest complete frame is frames_done - 1; step back 'back' + k frames */
        idx  = (frames_done + frames - 1U - back - k) % frames;
        sum += (int32_t)buf[(idx * channels) + ch];
    }
    dev = sum - mean_sum;

    if (dev_sum_out != 0) *dev_sum_out = dev;
    if (dev >  min_abs_sum) return  1;
    if (dev < -min_abs_sum) return -1;
    return 0;
}
