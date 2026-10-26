/**
  ******************************************************************************
  * @file    slave_link2.c
  * @brief   The slave's side of link version 2. See slave_link2.h. Pure logic: no HAL.
  ******************************************************************************
  */
#include "slave_link2.h"

static SL2_Hooks_t  s_hk;
static LinkV2Rx_t   s_rx;
static SL2_Stats_t  s_st;

static uint32_t s_e_next;            /* when the next $E is due                         */
static uint8_t  s_e_seq;
static uint32_t s_h_next;            /* when the next boot $H is due                    */
static uint8_t  s_h_boot_left;
static uint8_t  s_h_pending;         /* a HELLO asked for one                           */
static LinkR_t  s_rq[SL2_R_QUEUE];   /* responses waiting to be sent, oldest first      */
static uint8_t  s_rq_n;
static uint8_t  s_last_id;           /* the id of the last $K handled; 0 = none yet      */
static LinkT_t   s_t_hold; static uint8_t s_t_has;      /* a $T frame taken from the hook that the transmitter has not accepted yet */
static uint32_t s_now_ms;            /* the time of the last SL2_Service / SL2_Rx call, for the hooks */
static uint8_t  s_cal_pending, s_cal_id;   /* a calibration is averaging: the id it will be answered with */
static LinkR_t  s_last_resp;         /* what it was answered with, so a retry gets the same answer */

/* wrap-safe "now has reached t" for a free-running 32-bit millisecond counter */
static uint8_t reached(uint32_t now, uint32_t t) { return (((int32_t)(now - t)) >= 0) ? 1U : 0U; }

static void push_r(uint8_t id, uint8_t status, int32_t value)
{
    if (s_rq_n >= SL2_R_QUEUE) { s_st.r_dropped++; return; }
    s_rq[s_rq_n].id = id; s_rq[s_rq_n].status = status; s_rq[s_rq_n].value = value;
    s_rq_n++;
}

void SL2_Init(const SL2_Hooks_t *hooks, uint32_t now_ms)
{
    uint32_t i;
    uint8_t *p = (uint8_t *)&s_st;
    s_hk = *hooks;
    LinkV2Rx_Init(&s_rx);
    for (i = 0U; i < (uint32_t)sizeof(s_st); i++) p[i] = 0U;
    s_e_next = now_ms + SL2_E_PERIOD_MS;
    s_e_seq = 0U;
    s_h_next = now_ms;                       /* the first hello at once, then every 100 ms */
    s_h_boot_left = (uint8_t)SL2_H_BOOT_COUNT;
    s_h_pending = 0U;
    s_rq_n = 0U;
    s_last_id = 0U; s_cal_pending = 0U; s_cal_id = 0U; s_t_has = 0U;
    s_last_resp.id = 0U; s_last_resp.status = 0U; s_last_resp.value = 0;
}

static void handle_k(const LinkK_t *k)
{
    s_st.k_ok++;
    if ((s_cal_pending != 0U) && (k->id == s_cal_id)) { s_st.k_repeat++; return; }      /* the master's retry of the command that is being worked on: it is answered when it is done */
    if ((k->id == s_last_id) && (s_last_id != 0U))
    {   /* the master's retry: answer again with the SAME answer, execute nothing */
        s_st.k_repeat++;
        push_r(s_last_resp.id, s_last_resp.status, s_last_resp.value);
        return;
    }
    {
        uint8_t status = LINKV2_ST_UNKNOWN_CMD;      /* every command except HELLO is not in this firmware version */
        int32_t value = 0;
        if (k->cmd == LINKCMD_HELLO) { s_h_pending = 1U; status = LINKV2_ST_OK; }
        else if (((k->cmd == LINKCMD_TUNE) || (k->cmd == LINKCMD_TABORT) || (k->cmd == LINKCMD_TPARAM) || (k->cmd == LINKCMD_TSAVE)) && (s_hk.tune_command != 0))
        {   /* the autotune commands are answered at once; a command this build does not know stays UNKNOWN_CMD */
            if (s_hk.tune_command((uint8_t)k->cmd, k->name, k->num, &status, &value, s_now_ms) == 0) { status = LINKV2_ST_UNKNOWN_CMD; value = 0; }
        }
        else if ((k->cmd == LINKCMD_SEQ) && (s_hk.seq_arm != 0))
        {   /* the phase sequence round: executed once (a retry of the same id is answered from s_last_resp above) */
            s_hk.seq_arm((uint8_t)k->num, s_now_ms); status = LINKV2_ST_OK;
        }
        else if (((k->cmd == LINKCMD_CALV) || (k->cmd == LINKCMD_CALO) || (k->cmd == LINKCMD_CALR)) && (s_hk.cal_command != 0))
        {
            if (s_cal_pending != 0U) { status = LINKV2_ST_NOT_ALLOWED; }                          /* another calibration is averaging */
            else if (s_hk.cal_command((uint8_t)k->cmd, k->num, k->letter, &status, &value, s_now_ms) == 0)
            {   /* started: no answer yet; the id is remembered so that a retry is not executed twice */
                s_cal_pending = 1U; s_cal_id = k->id; s_last_id = k->id;
                return;
            }
        }
        s_last_id = k->id;
        s_last_resp.id = k->id; s_last_resp.status = status; s_last_resp.value = value;
        push_r(k->id, status, value);
    }
}

void SL2_Rx(uint8_t byte, uint32_t now_ms)
{
    s_now_ms = now_ms;
    LinkV2Frame_t f;
    int32_t t = LinkV2Rx_Feed(&s_rx, byte, &f);
    if (t == (int32_t)LINKV2_M)
    {
        s_st.m_ok++;
        if (s_hk.on_control != 0) s_hk.on_control(&f.u.m, now_ms);
    }
    else if (t == (int32_t)LINKV2_K)
    {
        handle_k(&f.u.k);
    }
    s_st.rx_bad = s_rx.frames_bad;
    s_st.rx_unknown = s_rx.frames_unknown;
}

static uint8_t send(const char *frame, int32_t n)
{
    uint8_t ok = 0U;
    if ((n > 0) && (s_hk.tx != 0)) ok = s_hk.tx(frame, (uint32_t)n);
    if (ok == 0U) s_st.tx_dropped++;
    return ok;
}

void SL2_Service(uint32_t now_ms)
{
    char    b[LINKV2_MAX_LEN + 8U];
    int32_t n;

    s_now_ms = now_ms;
    /* 0. a calibration that is averaging: poll it, and queue its answer when it is done */
    if ((s_cal_pending != 0U) && (s_hk.cal_poll != 0))
    {
        uint8_t cst = 0U; int32_t cval = 0;
        if (s_hk.cal_poll(&cst, &cval, now_ms) != 0) { s_cal_pending = 0U; s_last_resp.id = s_cal_id; s_last_resp.status = cst; s_last_resp.value = cval; push_r(s_cal_id, cst, cval); }
    }
    /* 1. responses first: a command is answered as soon as the queue has room */
    while (s_rq_n > 0U)
    {
        n = LinkV2_BuildR(b, sizeof b, &s_rq[0]);
        if ((n > 0) && (s_hk.tx != 0) && (s_hk.tx(b, (uint32_t)n) != 0U))
        {
            uint8_t i;
            s_st.r_sent++;
            for (i = 1U; i < s_rq_n; i++) s_rq[i - 1U] = s_rq[i];
            s_rq_n--;
        }
        else if (n <= 0) { uint8_t i; for (i = 1U; i < s_rq_n; i++) s_rq[i - 1U] = s_rq[i]; s_rq_n--; }   /* unbuildable: drop it */
        else { s_st.tx_dropped++; break; }           /* no room now: try again on the next pass */
    }

    /* 2. hello: three at start, and whenever a HELLO asked for one */
    if (((s_h_boot_left > 0U) && reached(now_ms, s_h_next)) || (s_h_pending != 0U))
    {
        LinkH_t h;
        h.proto = (uint8_t)LINKV2_PROTO_VERSION; h.fw = 0U; h.vin_min_dv = 0U; h.vin_max_dv = 0U; h.rst = 0U; h.tuned = 0U; h.vout_max_dv = 0U;
        h.prof_cap = 0U; h.prof_kib = 0U; h.prof_kiu = 0U; h.prof_vref_dv = 0U; h.prof_maxc_dv = 0U; h.prof_n = 0U; h.prof_knee = 0U;      /* EVERY field: an optional one left unset would hold stack garbage and the builder would refuse the whole frame (no hello at all) */
        if (s_hk.fill_h != 0) s_hk.fill_h(&h);
        h.proto = (uint8_t)LINKV2_PROTO_VERSION;
        n = LinkV2_BuildH(b, sizeof b, &h);
        if (send(b, n) != 0U)
        {
            s_st.h_sent++;
            if (s_h_pending != 0U) s_h_pending = 0U;
            else { s_h_boot_left--; s_h_next = now_ms + SL2_H_PERIOD_MS; }
        }
    }

    /* 3. extended status every 100 ms; its seq counts every attempt, so a gap shows */
    if (reached(now_ms, s_e_next))
    {
        LinkE_t e;
        e.seq = s_e_seq; e.setv_dv = 0U; e.flags2 = 0U; e.clr = 0U; e.reach_lo_dv = 0U; e.reach_hi_dv = 0U; e.iout_rms_ca = 0U; e.iout_pk_ca = 0U; e.sq_id = 0U; e.sq_delay_us = 0U; e.sq_period_us = 0U;
        if (s_hk.fill_e != 0) s_hk.fill_e(&e);
        e.seq = s_e_seq;
        s_e_seq++;
        s_e_next = now_ms + SL2_E_PERIOD_MS;
        n = LinkV2_BuildE(b, sizeof b, &e);
        if (send(b, n) != 0U) s_st.e_sent++;
    }

    /* 4. the autotune's progress frames ($T): a frame is taken from the hook, sent, and kept until the transmitter accepts it */
    if ((s_t_has == 0U) && (s_hk.tune_frame != 0) && (s_hk.tune_frame(&s_t_hold, now_ms) != 0)) s_t_has = 1U;
    if (s_t_has != 0U)
    {
        n = LinkV2_BuildT(b, sizeof b, &s_t_hold);
        if (n <= 0) s_t_has = 0U;                                              /* a frame the builder refuses (out of range) is dropped, never retried for ever */
        else if ((s_hk.tx != 0) && (s_hk.tx(b, (uint32_t)n) != 0U)) s_t_has = 0U;
    }
}

const SL2_Stats_t *SL2_Stats(void) { return &s_st; }
