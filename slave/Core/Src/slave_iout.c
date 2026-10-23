/** @file slave_iout.c  @brief Converter-side current from the I-OUT ADC statistics. See slave_iout.h. */
#include "slave_iout.h"

void Iout_Compute(float rms_counts, float mean_counts, uint16_t min_counts, uint16_t max_counts, IoutResult_t *r)
{
    float up = (float)max_counts - mean_counts, down = mean_counts - (float)min_counts, pk = (up > down) ? up : down;
    if (rms_counts < 0.0f) rms_counts = 0.0f;
    if (pk < 0.0f) pk = 0.0f;
    r->irms_a = rms_counts * IOUT_A_PER_COUNT * IOUT_CAL;
    r->ipk_a  = pk * IOUT_A_PER_COUNT * IOUT_CAL;
    r->clipped = (uint8_t)(((max_counts >= IOUT_CLIP_HI) || (min_counts <= IOUT_CLIP_LO)) ? 1U : 0U);
}

uint16_t Iout_ToCentiAmps(float amps)
{
    float c = amps * 100.0f + 0.5f;
    if (c < 0.0f) c = 0.0f;
    if (c > 4000.0f) c = 4000.0f;
    return (uint16_t)c;
}
