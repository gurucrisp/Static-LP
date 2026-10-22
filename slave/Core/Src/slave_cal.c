/** @file slave_cal.c  @brief Voltage calibration of the slave. See slave_cal.h. */
#include "slave_cal.h"
#include "link_v2.h"

static const SlaveCalHw_t *s_hw;
static uint16_t s_ratio[2] = { 10000U, 10000U };
static struct { uint8_t active, ch; uint32_t t0, last_cnt, n; float sum, mn, mx; int32_t meter_dv; } s_job;

static uint8_t ratio_ok(uint16_t r) { return (uint8_t)((r >= SCAL_RATIO_MIN) && (r <= SCAL_RATIO_MAX)); }

void SlaveCal_Init(const SlaveCalHw_t *hw)
{
    uint16_t r[2];
    s_hw = hw; s_ratio[0] = 10000U; s_ratio[1] = 10000U; s_job.active = 0U;
    if ((hw->store_load(r) == 0) && (ratio_ok(r[0]) != 0U) && (ratio_ok(r[1]) != 0U)) { s_ratio[0] = r[0]; s_ratio[1] = r[1]; }      /* a record outside the accepted range is ignored */
    hw->set_ratio(0U, s_ratio[0]); hw->set_ratio(1U, s_ratio[1]);
}

uint16_t SlaveCal_Ratio(uint8_t ch) { return s_ratio[ch & 1U]; }
uint8_t  SlaveCal_Busy(void) { return s_job.active; }

/* apply and store a new pair of ratios; on a failed store the old ones go back and the answer is FLASH_FAIL */
static uint8_t commit(uint8_t ch, uint16_t ratio)
{
    uint16_t nr[2]; uint16_t old = s_ratio[ch];
    nr[0] = s_ratio[0]; nr[1] = s_ratio[1]; nr[ch] = ratio;
    s_hw->set_ratio(ch, ratio);
    if (s_hw->store_save(nr) != 0) { s_hw->set_ratio(ch, old); return (uint8_t)LINKV2_ST_FLASH_FAIL; }
    s_ratio[ch] = ratio;
    return (uint8_t)LINKV2_ST_OK;
}

int SlaveCal_Command(uint8_t cmd, int32_t num_dv, char letter, uint8_t *status, int32_t *value, uint32_t now_ms)
{
    *value = 0;
    if (s_job.active != 0U) { *status = (uint8_t)LINKV2_ST_NOT_ALLOWED; return 1; }                     /* one calibration at a time */
    if (s_hw->is_stopped() == 0U) { *status = (uint8_t)LINKV2_ST_NOT_STOPPED; return 1; }                 /* never while the unit regulates */
    if (cmd == SCAL_CMD_CALR)
    {
        uint8_t ch = (letter == 'V') ? 0U : ((letter == 'O') ? 1U : 2U);
        if (ch > 1U) { *status = (uint8_t)LINKV2_ST_BAD_ARG; return 1; }
        *status = commit(ch, 10000U); *value = (*status == (uint8_t)LINKV2_ST_OK) ? 10000 : (int32_t)s_ratio[ch];
        return 1;
    }
    if (((cmd != SCAL_CMD_CALV) && (cmd != SCAL_CMD_CALO)) || (num_dv < 500) || (num_dv > 4000)) { *status = (uint8_t)LINKV2_ST_BAD_ARG; return 1; }
    s_job.active = 1U; s_job.ch = (uint8_t)((cmd == SCAL_CMD_CALV) ? 0U : 1U); s_job.t0 = now_ms; s_job.n = 0U; s_job.sum = 0.0f; s_job.mn = 1.0e30f; s_job.mx = -1.0f;
    s_job.last_cnt = s_hw->update_count(); s_job.meter_dv = num_dv;
    return 0;
}

int SlaveCal_Poll(uint8_t *status, int32_t *value, uint32_t now_ms)
{
    uint32_t cnt; float avg, reading_v, ratio_f; uint32_t ratio;
    *value = 0;
    if (s_job.active == 0U) return 0;
    if (s_hw->is_stopped() == 0U) { s_job.active = 0U; *status = (uint8_t)LINKV2_ST_NOT_STOPPED; return 1; }       /* a run command arrived during the averaging: nothing is changed */
    cnt = s_hw->update_count();
    if (cnt != s_job.last_cnt)
    {
        uint8_t valid = 0U; float raw = s_hw->raw_counts(s_job.ch, &valid);
        s_job.last_cnt = cnt;
        if (valid != 0U) { s_job.sum += raw; s_job.n++; if (raw < s_job.mn) s_job.mn = raw; if (raw > s_job.mx) s_job.mx = raw; }
    }
    if ((uint32_t)(now_ms - s_job.t0) < SCAL_AVG_MS) return 0;
    /* the window is over: decide */
    s_job.active = 0U;
    if (s_job.n < SCAL_MIN_SAMPLES) { *status = (uint8_t)LINKV2_ST_BAD_SIGNAL; return 1; }
    avg = s_job.sum / (float)s_job.n;
    if (avg <= 0.0f) { *status = (uint8_t)LINKV2_ST_BAD_SIGNAL; return 1; }
    if ((((s_job.mx - s_job.mn) / avg) * 100.0f) > SCAL_SPREAD_PCT) { *status = (uint8_t)LINKV2_ST_BAD_SIGNAL; return 1; }          /* the input is not steady */
    reading_v = avg * s_hw->nominal_gain(s_job.ch);                                    /* what the NOMINAL chain would read: the correction is always against nominal, so repeating it does not compound */
    if (reading_v < SCAL_MIN_READING_V) { *status = (uint8_t)LINKV2_ST_BAD_SIGNAL; return 1; }
    ratio_f = ((float)s_job.meter_dv / 10.0f) / reading_v * 10000.0f;
    ratio = (ratio_f > 60000.0f) ? 60000UL : (uint32_t)(ratio_f + 0.5f);
    *value = (int32_t)ratio;
    if ((ratio < SCAL_RATIO_MIN) || (ratio > SCAL_RATIO_MAX)) { *status = (uint8_t)LINKV2_ST_OUT_OF_RANGE; return 1; }
    *status = commit(s_job.ch, (uint16_t)ratio);
    if (*status != (uint8_t)LINKV2_ST_OK) *value = (int32_t)s_ratio[s_job.ch];
    return 1;
}
