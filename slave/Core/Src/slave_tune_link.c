/** @file slave_tune_link.c  @brief The autotune seen from the master's link. See slave_tune_link.h. */
#include "slave_tune_link.h"
#include "slave_profile.h"

static const TuneLinkHw_t *s_hw;
static uint16_t s_req;
static LinkT_t  s_q[STL_QUEUE]; static uint8_t s_qh, s_qn;
static LinkT_t  s_final; static uint8_t s_final_valid, s_rep_left; static uint32_t s_rep_next;
static uint8_t  s_prep, s_swept; static uint32_t s_prep_t0, s_last_out_ms; static uint16_t s_vin_dv;
static uint8_t  s_save_pend, s_save_seen_off; static uint32_t s_save_t0, s_save_off_since; static TuneProfile_t s_save_prof; static LinkT_t s_save_frame;     /* the profile waiting for the stage to be OFF */

static void push(const LinkT_t *f)
{
    if (s_qn >= STL_QUEUE) { s_qh = (uint8_t)((s_qh + 1U) % STL_QUEUE); s_qn--; }                    /* the oldest goes: the master only misses progress, never the result (it is repeated) */
    s_q[(s_qh + s_qn) % STL_QUEUE] = *f; s_qn++;
}

static void hw_end(void) { if ((s_hw != 0) && (s_hw->end != 0)) s_hw->end(); if ((s_hw != 0) && (s_hw->request != 0)) s_hw->request(0U); s_req = 0U; }

/* the end of a run that never reached or left the sweep: one final frame, repeated */
static void final_push(const LinkT_t *f, uint32_t now_ms) { push(f); s_final = *f; s_final_valid = 1U; s_rep_left = (uint8_t)STL_FINAL_REPEATS; s_rep_next = now_ms + STL_FINAL_GAP_MS; }
static void final_frame(uint8_t result, uint32_t now_ms)
{
    LinkT_t f; f.step = 0U; f.ticks = 0U; f.vin_dv = s_vin_dv; f.dv = 0; f.result = result;
    final_push(&f, now_ms);
}
/* the profile is written to flash ONLY when the controller has been OFF for STL_SAVE_SETTLE_MS: a flash erase stalls the CPU and the commutation interrupts with it */
static void save_step(uint32_t now_ms)
{
    const uint8_t off = (uint8_t)((s_hw == 0) || (s_hw->stopped == 0) || (s_hw->stopped() != 0U));
    LinkT_t f = s_save_frame;
    if (off != 0U)
    {
        if (s_save_seen_off == 0U) { s_save_seen_off = 1U; s_save_off_since = now_ms; }
        if ((uint32_t)(now_ms - s_save_off_since) >= STL_SAVE_SETTLE_MS)
        {
            f.result = (SlaveProfile_Save(&s_save_prof) == 0) ? (uint8_t)TUNE_R_FINISHED : (uint8_t)TUNE_R_ABORTED; if (f.result == (uint8_t)TUNE_R_ABORTED) { f.dv = 0; }
            s_save_pend = 0U; final_push(&f, now_ms);
            return;
        }
    }
    else s_save_seen_off = 0U;
    if ((uint32_t)(now_ms - s_save_t0) >= STL_SAVE_TIMEOUT_MS) { f.result = (uint8_t)TUNE_R_ABORTED; f.dv = 0; s_save_pend = 0U; final_push(&f, now_ms); }     /* it never reached OFF: NOTHING is written */
}

void SlaveTuneLink_Init(const TuneLinkHw_t *hw)
{
    s_hw = hw; s_req = 0U; s_qh = 0U; s_qn = 0U; s_final_valid = 0U; s_rep_left = 0U; s_rep_next = 0U; s_prep = 0U; s_swept = 0U; s_prep_t0 = 0U; s_last_out_ms = 0U; s_vin_dv = 0U; s_save_pend = 0U; s_save_seen_off = 0U;
}

int SlaveTuneLink_Command(uint8_t cmd, const char *name, int32_t num, uint8_t *status, int32_t *value, uint32_t now_ms)
{
    (void)name; (void)num;
    *value = 0;
    if (cmd == (uint8_t)LINKCMD_TABORT)
    {
        if (s_prep != 0U) { s_prep = 0U; hw_end(); final_frame((uint8_t)TUNE_R_BY_MASTER, now_ms); }       /* still starting up: stop the controller, report the abort */
        else if (s_save_pend != 0U) { s_save_pend = 0U; hw_end(); final_frame((uint8_t)TUNE_R_BY_MASTER, now_ms); }   /* the sweep is over and the profile is waiting for the stage to be OFF: cancelled, NOTHING is written */
        else Tune_Abort();                                                                                  /* a sweep: its final frame comes from the next service pass */
        *status = (uint8_t)LINKV2_ST_OK; return 1;
    }
    if (cmd == (uint8_t)LINKCMD_TUNE)
    {
        uint16_t mc;
        if ((Tune_Running() != 0U) || (s_prep != 0U)) { *status = (uint8_t)LINKV2_ST_NOT_ALLOWED; *value = 4; return 1; }
        if ((s_hw == 0) || (s_hw->can_run == 0) || (s_hw->can_run() == 0U)) { *status = (uint8_t)LINKV2_ST_NOT_ALLOWED; *value = 1; return 1; }
        mc = (s_hw->maxc_dv != 0) ? s_hw->maxc_dv() : 0U;
        if (mc == 0U) { *status = (uint8_t)LINKV2_ST_NOT_ALLOWED; *value = 2; return 1; }
        if ((s_hw->stopped != 0) && (s_hw->stopped() == 0U)) { *status = (uint8_t)LINKV2_ST_NOT_STOPPED; return 1; }
        if ((s_hw->other_busy != 0) && (s_hw->other_busy() != 0U)) { *status = (uint8_t)LINKV2_ST_NOT_ALLOWED; *value = 3; return 1; }
        if ((s_hw->headroom_dv != 0) && (s_hw->headroom_dv() < STL_MIN_HEADROOM_DV)) { *status = (uint8_t)LINKV2_ST_NOT_ALLOWED; *value = 6; return 1; }
        if ((s_hw->begin != 0) && (s_hw->begin() == 0U)) { *status = (uint8_t)LINKV2_ST_NOT_ALLOWED; *value = 5; return 1; }
        s_qh = 0U; s_qn = 0U; s_final_valid = 0U; s_rep_left = 0U;                                          /* a new run: the old progress is forgotten */
        s_prep = 1U; s_swept = 0U; s_prep_t0 = now_ms; s_last_out_ms = now_ms;
        *status = (uint8_t)LINKV2_ST_OK; return 1;
    }
    return 0;                                                                                               /* TPARAM, TSAVE: not in this version */
}

void SlaveTuneLink_Service(uint32_t now_ms, const TuneIn_t *in)
{
    TuneOut_t out; TuneFrame_t tf; uint8_t finished = 0U, final_result = 0U; LinkT_t lf, last_lf; uint8_t have_last = 0U; TuneIn_t in2 = *in;
    s_vin_dv = (uint16_t)((in->vin > 0.0f) ? (in->vin * 10.0f + 0.5f) : 0.0f);
    /* the preparation: the controller goes through its start sequence; the sweep starts when it is in RUN and settled */
    if (s_prep != 0U)
    {
        if ((s_hw != 0) && (s_hw->fault != 0) && (s_hw->fault() != 0U)) { s_prep = 0U; hw_end(); final_frame((uint8_t)TUNE_R_ABORTED, now_ms); }
        else if ((s_hw != 0) && (s_hw->alive != 0) && (s_hw->alive() == 0U)) { s_prep = 0U; hw_end(); final_frame((uint8_t)TUNE_R_ABORTED, now_ms); }
        else if ((s_hw != 0) && (s_hw->ready != 0) && (s_hw->ready() != 0U))
        {
            uint16_t mc = (s_hw->maxc_dv != 0) ? s_hw->maxc_dv() : 0U;
            if ((s_hw->headroom_dv != 0) && (s_hw->headroom_dv() < mc)) mc = s_hw->headroom_dv();            /* the sweep's limit: never more than the output may rise before its over-voltage limit */
            s_prep = 0U;
            if ((mc < STL_MIN_HEADROOM_DV) || (Tune_Start((float)mc / 10.0f, now_ms) != 0U)) { hw_end(); final_frame((uint8_t)TUNE_R_ABORTED, now_ms); }
            else s_swept = 1U;
        }
        else if ((uint32_t)(now_ms - s_prep_t0) >= STL_PREP_TIMEOUT_MS) { s_prep = 0U; hw_end(); final_frame((uint8_t)TUNE_R_ABORTED, now_ms); }
    }
    /* during the sweep: a fault in the controller, or tune mode ending under us (the master link lost), aborts it */
    if (Tune_Running() != 0U)
    {
        if ((s_hw != 0) && (s_hw->fault != 0) && (s_hw->fault() != 0U)) in2.fault = 1U;
        if ((s_hw != 0) && (s_hw->alive != 0) && (s_hw->alive() == 0U)) in2.fault = 1U;
    }
    Tune_Service(now_ms, &in2, &out);
    s_req = out.request_ticks;
    if ((s_hw != 0) && (s_hw->request != 0) && (s_swept != 0U)) s_hw->request(s_req);                        /* the duty the sweep asks for: the controller applies it, with its own limits */
    while (Tune_TakeFrame(&tf) != 0U)
    {
        lf.step = tf.step; lf.ticks = tf.ticks; lf.vin_dv = tf.vin_dv; lf.dv = tf.dv_dv; lf.result = tf.result;
        if (tf.result == TUNE_R_STEP) push(&lf);
        else { finished = 1U; final_result = tf.result; last_lf = lf; have_last = 1U; }
    }
    if ((finished != 0U) && (have_last != 0U))
    {
        if (final_result == TUNE_R_FINISHED)
        {   /* the profile is stored BEFORE the master is told FINISHED, but NEVER while the stage is energised: the controller is stopped first (below) and the write waits until it is OFF (save_step) */
            if (Tune_GetProfile(&s_save_prof) == 0U) { last_lf.result = (uint8_t)TUNE_R_ABORTED; last_lf.dv = 0; final_push(&last_lf, now_ms); }
            else { s_save_pend = 1U; s_save_seen_off = 0U; s_save_t0 = now_ms; s_save_frame = last_lf; }
        }
        else final_push(&last_lf, now_ms);
    }
    if ((out.done != 0U) || (finished != 0U)) { s_swept = 0U; hw_end(); }                                    /* the sweep is over, however it ended: the controller stops FIRST */
    if (s_save_pend != 0U) save_step(now_ms);
}

int SlaveTuneLink_NextFrame(LinkT_t *f, uint32_t now_ms)
{
    if (s_qn > 0U) { *f = s_q[s_qh]; s_qh = (uint8_t)((s_qh + 1U) % STL_QUEUE); s_qn--; s_last_out_ms = now_ms; return 1; }
    if ((s_final_valid != 0U) && (s_rep_left > 0U) && (((int32_t)(now_ms - s_rep_next)) >= 0)) { *f = s_final; s_rep_left--; s_rep_next = now_ms + STL_FINAL_GAP_MS; s_last_out_ms = now_ms; return 1; }
    if ((SlaveTuneLink_Active() != 0U) && ((uint32_t)(now_ms - s_last_out_ms) >= STL_HEARTBEAT_MS))
    {   /* nothing to report for 2 s: "still here, preparing / measuring the baseline" (step 0) */
        f->step = 0U; f->ticks = 0U; f->vin_dv = s_vin_dv; f->dv = 0; f->result = (uint8_t)TUNE_R_STEP; s_last_out_ms = now_ms; return 1;
    }
    return 0;
}

uint16_t SlaveTuneLink_RequestTicks(void) { return s_req; }
uint8_t  SlaveTuneLink_Active(void) { return (uint8_t)((s_prep != 0U) || (Tune_Running() != 0U) || (s_save_pend != 0U)); }

uint16_t SlaveTuneLink_Flags2(void)
{
    uint16_t f = 0U;
    if (SlaveTuneLink_Active() != 0U) f = (uint16_t)(f | LINKV2_FLAG_BUSY);
    if ((SlaveProfile_Tuned() == 0U) || ((s_hw != 0) && (s_hw->maxc_dv != 0) && (s_hw->maxc_dv() == 0U))) f = (uint16_t)(f | LINKV2_FLAG_NOT_TUNED);
    return f;
}
