/** @file slave_tune_link.c  @brief The autotune seen from the master's link. See slave_tune_link.h. */
#include "slave_tune_link.h"
#include "slave_profile.h"

static const TuneLinkHw_t *s_hw;
static uint16_t s_req;
static LinkT_t  s_q[STL_QUEUE]; static uint8_t s_qh, s_qn;
static LinkT_t  s_final; static uint8_t s_final_valid, s_rep_left; static uint32_t s_rep_next;

static void push(const LinkT_t *f)
{
    if (s_qn >= STL_QUEUE) { s_qh = (uint8_t)((s_qh + 1U) % STL_QUEUE); s_qn--; }                    /* the oldest goes: the master only misses progress, never the result (it is repeated) */
    s_q[(s_qh + s_qn) % STL_QUEUE] = *f; s_qn++;
}

void SlaveTuneLink_Init(const TuneLinkHw_t *hw) { s_hw = hw; s_req = 0U; s_qh = 0U; s_qn = 0U; s_final_valid = 0U; s_rep_left = 0U; s_rep_next = 0U; }

int SlaveTuneLink_Command(uint8_t cmd, const char *name, int32_t num, uint8_t *status, int32_t *value, uint32_t now_ms)
{
    (void)name; (void)num;
    *value = 0;
    if (cmd == (uint8_t)LINKCMD_TABORT) { Tune_Abort(); *status = (uint8_t)LINKV2_ST_OK; return 1; }
    if (cmd == (uint8_t)LINKCMD_TUNE)
    {
        uint16_t mc;
        if (Tune_Running() != 0U) { *status = (uint8_t)LINKV2_ST_NOT_ALLOWED; *value = 4; return 1; }
        if ((s_hw == 0) || (s_hw->can_run == 0) || (s_hw->can_run() == 0U)) { *status = (uint8_t)LINKV2_ST_NOT_ALLOWED; *value = 1; return 1; }
        mc = (s_hw->maxc_dv != 0) ? s_hw->maxc_dv() : 0U;
        if (mc == 0U) { *status = (uint8_t)LINKV2_ST_NOT_ALLOWED; *value = 2; return 1; }
        if ((s_hw->stopped != 0) && (s_hw->stopped() == 0U)) { *status = (uint8_t)LINKV2_ST_NOT_STOPPED; return 1; }
        if ((s_hw->other_busy != 0) && (s_hw->other_busy() != 0U)) { *status = (uint8_t)LINKV2_ST_NOT_ALLOWED; *value = 3; return 1; }
        if (Tune_Start((float)mc / 10.0f, now_ms) != 0U) { *status = (uint8_t)LINKV2_ST_NOT_ALLOWED; *value = 4; return 1; }
        s_qh = 0U; s_qn = 0U; s_final_valid = 0U; s_rep_left = 0U;                                   /* a new sweep: the old progress is forgotten */
        *status = (uint8_t)LINKV2_ST_OK; return 1;
    }
    return 0;                                                                                         /* TPARAM, TSAVE: not in this version */
}

void SlaveTuneLink_Service(uint32_t now_ms, const TuneIn_t *in)
{
    TuneOut_t out; TuneFrame_t tf; uint8_t finished = 0U, final_result = 0U; LinkT_t lf, last_lf; uint8_t have_last = 0U;
    Tune_Service(now_ms, in, &out);
    s_req = out.request_ticks;
    while (Tune_TakeFrame(&tf) != 0U)
    {
        lf.step = tf.step; lf.ticks = tf.ticks; lf.vin_dv = tf.vin_dv; lf.dv = tf.dv_dv; lf.result = tf.result;
        if (tf.result == TUNE_R_STEP) push(&lf);
        else { finished = 1U; final_result = tf.result; last_lf = lf; have_last = 1U; }
    }
    if ((finished != 0U) && (have_last != 0U))
    {
        if (final_result == TUNE_R_FINISHED)
        {   /* the profile is stored BEFORE the master is told: if the flash refuses, the result is ABORTED and nothing is stored */
            TuneProfile_t p;
            if ((Tune_GetProfile(&p) == 0U) || (SlaveProfile_Save(&p) != 0)) last_lf.result = (uint8_t)TUNE_R_ABORTED;
        }
        push(&last_lf); s_final = last_lf; s_final_valid = 1U; s_rep_left = (uint8_t)STL_FINAL_REPEATS; s_rep_next = now_ms + STL_FINAL_GAP_MS;
    }
}

int SlaveTuneLink_NextFrame(LinkT_t *f, uint32_t now_ms)
{
    if (s_qn > 0U) { *f = s_q[s_qh]; s_qh = (uint8_t)((s_qh + 1U) % STL_QUEUE); s_qn--; return 1; }
    if ((s_final_valid != 0U) && (s_rep_left > 0U) && (((int32_t)(now_ms - s_rep_next)) >= 0)) { *f = s_final; s_rep_left--; s_rep_next = now_ms + STL_FINAL_GAP_MS; return 1; }
    return 0;
}

uint16_t SlaveTuneLink_RequestTicks(void) { return s_req; }
uint8_t  SlaveTuneLink_Active(void) { return Tune_Running(); }

uint16_t SlaveTuneLink_Flags2(void)
{
    uint16_t f = 0U;
    if (Tune_Running() != 0U) f = (uint16_t)(f | LINKV2_FLAG_BUSY);
    if ((SlaveProfile_Tuned() == 0U) || ((s_hw != 0) && (s_hw->maxc_dv != 0) && (s_hw->maxc_dv() == 0U))) f = (uint16_t)(f | LINKV2_FLAG_NOT_TUNED);
    return f;
}
