/**
  ******************************************************************************
  * @file    slave_ctrl.c
  * @brief   Converter + inverter control (see slave_ctrl.h for the design rules).
  *
  * INTERRUPT CODE (SlaveCtrl_On*) is integer-only and short on purpose: the
  * Cortex-M0+ has no FPU and the converter dead time is only 40 us.
  * MAIN-LOOP CODE (SlaveCtrl_Service) makes every decision that can wait.
  ******************************************************************************
  */
#include <stddef.h>
#include "slave_ctrl.h"
#include "link_v2.h"          /* LINKV2_FLAG_* for the $E frame */
#include "slave_ctrl_hw.h"
#include "slave_power.h"
#include "slave_reg.h"
#include "slave_relay.h"

SlaveCtrl_t g_ctrl;

static uint32_t s_commut_us;            /* time of the latest commutation (for the guard blanking) */

/* Duty ceiling in timer counts (bootstrapped gate drivers: see CTRL_DUTY_MAX_PCT) */
#define CTRL_DUTY_MAX_COUNTS   ((uint16_t)((CTRL_DUTY_MAX_PCT * POWER_PERIOD_TICKS) / 100UL))

/* The CCR to command for an EFFECTIVE high-side on-time of eff ticks: zero stays zero, anything else gets the dead time added (a pulse shorter than the dead time
 * is not generated at all). The step from CCR 0 to dead time + 1 changes the real output by one tick: continuous. Capped at the duty ceiling. */
#define CTRL_EFF_TO_CCR(eff)   ((uint16_t)((((uint32_t)(eff) == 0U) ? 0U : (((uint32_t)(eff) + (uint32_t)POWER_DEADTIME_TICKS) > (uint32_t)CTRL_DUTY_MAX_COUNTS) \
                                          ? (uint32_t)CTRL_DUTY_MAX_COUNTS : ((uint32_t)(eff) + (uint32_t)POWER_DEADTIME_TICKS))))
#if (SLAVE_POWER_STAGE >= 3)
_Static_assert((CTRL_STAGE3_MAX_EFF_TICKS + POWER_DEADTIME_TICKS) <= CTRL_DUTY_MAX_COUNTS, "CTRL_STAGE3_MAX_EFF_TICKS plus the dead time exceeds the duty ceiling (CTRL_DUTY_MAX_PCT)");
static volatile uint16_t s_reg_eff;      /* stage 3: the regulator's EFFECTIVE on-time in ticks (what the cap limits), for the display (diag2) */
static volatile uint8_t  s_reg_boost;
#endif

/* ------------------------------------------------------------------------- */
/* Zero-cross hypothesis trackers (owned by the interrupt)                    */
/* ------------------------------------------------------------------------- */
typedef struct
{
    uint32_t t_last;        /* time of the previous event of this hypothesis       */
    int8_t   sign_last;     /* sign read at that event                             */
    uint8_t  rising_last;   /* its edge type                                       */
    uint8_t  have_prev;
    uint8_t  good;          /* consecutive valid events                            */
} Trk_t;

static Trk_t s_trk_all;     /* hypothesis A: every edge is a zero-cross            */
static Trk_t s_trk_fall;    /* hypothesis B: every falling edge is a zero-cross    */

/* An event is "valid" if it follows the previous one by about half a mains
 * period AND the measured sign flipped (and, for hypothesis A, the edge type
 * alternated). A pulse train fails A (the pulse end is not a zero-cross), a
 * square wave fails B (falling edges are a whole period apart), a single pulse
 * per cycle fails both - which is exactly the point. */
#define TRK_WHY_INTERVAL  1U     /* the spacing to the previous event is outside 7.5..12.5 ms      */
#define TRK_WHY_SEQUENCE  2U     /* the sign did not flip / the edge type did not alternate        */

static uint8_t TrkUpdate(Trk_t *k, uint32_t now_us, uint8_t rising, int8_t sign,
                         uint8_t need_alternating_edge, uint32_t *dt_out, uint8_t *why)
{
    uint8_t ok = 0U;

    *dt_out = 0U;
    *why    = 0U;
    if (k->have_prev != 0U)
    {
        uint32_t dt = (uint32_t)(now_us - k->t_last);
        *dt_out = dt;
        if ((dt < CTRL_EVENT_MIN_US) || (dt > CTRL_EVENT_MAX_US)) *why |= TRK_WHY_INTERVAL;
        if (!((sign != 0) && (k->sign_last != 0) && (sign != k->sign_last) &&
              ((need_alternating_edge == 0U) || (rising != k->rising_last)))) *why |= TRK_WHY_SEQUENCE;
        if (*why == 0U) ok = 1U;
    }

    if (ok != 0U) { if (k->good < 255U) k->good++; }
    else          { k->good = 0U; }

    k->have_prev   = 1U;
    k->t_last      = now_us;
    k->sign_last   = sign;
    k->rising_last = rising;
    return ok;
}

/* ------------------------------------------------------------------------- */
/* Pure helper: which inverter leg carries the duty                           */
/* ------------------------------------------------------------------------- */
/* Same table as the hardware-tested old firmware:
 *   positive half:  boost -> leg 1 (TIM15), buck -> leg 2 (TIM1)
 *   negative half:  boost -> leg 2 (TIM1),  buck -> leg 1 (TIM15)
 * The idle leg gets 0 (its low side stays on). */
void SlaveCtrl_LegsFor(int8_t polarity, uint8_t boost, uint16_t duty,
                       uint16_t *leg1_tim15, uint16_t *leg2_tim1)
{
    uint8_t leg1_active = (polarity > 0) ? (boost != 0U) : (boost == 0U);
    *leg1_tim15 = leg1_active ? duty : 0U;
    *leg2_tim1  = leg1_active ? 0U : duty;
}

/* ------------------------------------------------------------------------- */
/* Interrupt-side                                                             */
/* ------------------------------------------------------------------------- */
/* SOFT fault (everything except an overcurrent): the converter goes off and the inverter duty to ZERO. The inverter is
 * NOT disabled here - its low sides keep the transformer primary shorted (rule R3) until the main loop has closed the
 * relay shunts and only then disables it. */
static void IsrFail(uint16_t code)
{
    SlavePower_ConverterOffNow();
    if (g_ctrl.inv_active != 0U) SlavePower_SetLegDuty(0U, 0U);
    CtrlHw_DisarmDeadtime();
    CtrlHw_DisarmPredictOff();
    CtrlHw_DisarmWatchdog();
    g_ctrl.run_allowed = 0U;
    g_ctrl.inv_active  = 0U;
    g_ctrl.duty_ok     = 0U;
    g_ctrl.conv_pol    = 0;
    if (g_ctrl.isr_fault == 0U) g_ctrl.isr_fault = code;
}

void SlaveCtrl_OnZcEdge(uint32_t now_us, uint8_t rising)
{
    int8_t   sign_in, sign_out;
    int32_t  dev_in, dev_out;
    uint8_t  selected, ok_all, ok_fall = 0U, ok, why_all = 0U, why_fall = 0U, why;
    uint32_t dt_all = 0U, dt_fall = 0U, interval;

    selected = (uint8_t)((g_ctrl.mode == CTRL_ZC_BOTH) ||
                         ((g_ctrl.mode == CTRL_ZC_FALL) && (rising == 0U)));

    /* 1) FIRST, before anything else: a zero-cross event ends the half-cycle,
     *    so the energised diagonal is dropped (and the inverter stops chopping
     *    during the transition). */
    if ((selected != 0U) && (g_ctrl.conv_pol != 0))
    {
        SlavePower_ConverterOffNow();
        g_ctrl.conv_pol = 0;
        if (g_ctrl.inv_active != 0U) SlavePower_SetLegDuty(0U, 0U);
    }

    /* 2) Sign of the mains just before this edge (integer, a few samples). */
    sign_in  = CtrlHw_Sign(0U, &dev_in);
    sign_out = CtrlHw_Sign(1U, &dev_out);

    /* 3) Keep both hypotheses up to date, whatever the current mode. */
    ok_all = TrkUpdate(&s_trk_all, now_us, rising, sign_in, 1U, &dt_all, &why_all);
    if (rising == 0U) ok_fall = TrkUpdate(&s_trk_fall, now_us, rising, sign_in, 0U, &dt_fall, &why_fall);
    g_ctrl.good_all  = s_trk_all.good;
    g_ctrl.good_fall = s_trk_fall.good;

    if (selected == 0U) return;           /* a pulse end, or the pattern is not known yet */

    g_ctrl.t_event_us   = now_us;
    g_ctrl.last_sign_in  = sign_in;
    g_ctrl.last_sign_out = sign_out;
    g_ctrl.last_dev_in   = dev_in;
    g_ctrl.last_dev_out  = dev_out;

    ok       = (g_ctrl.mode == CTRL_ZC_BOTH) ? ok_all : ok_fall;
    why      = (g_ctrl.mode == CTRL_ZC_BOTH) ? why_all : why_fall;
    interval = (g_ctrl.mode == CTRL_ZC_BOTH) ? dt_all : dt_fall;
    if (ok != 0U) g_ctrl.interval_us = interval;     /* half-cycle length, kept current even while idle */

    if ((ok != 0U) && (sign_in != 0))                /* PASSIVE mapping check against the old firmware (see slave_ctrl.h) */
    {
        const int8_t mine = (int8_t)(-sign_in);      /* the diagonal this firmware will choose for the NEXT half-cycle */
        const int8_t old  = (rising != 0U) ? (int8_t)-1 : (int8_t)1;     /* old: rising -> 3L+4H (negative), falling -> 3H+4L (positive) */
        if ((g_ctrl.map_agree >= 127U) || (g_ctrl.map_disagree >= 127U)) { g_ctrl.map_agree >>= 1; g_ctrl.map_disagree >>= 1; }
        if (mine == old) g_ctrl.map_agree++; else g_ctrl.map_disagree++;
    }

    if (g_ctrl.run_allowed == 0U) return; /* monitoring only: nothing may be energised */

    if (sign_in == 0)                                  { IsrFail(CF_POLARITY);       return; }
    if (ok == 0U)                                      /* say WHICH check failed: wrong TIME (F03) or wrong SIGN sequence (F15) */
    {
        IsrFail(((why & TRK_WHY_INTERVAL) != 0U || why == 0U) ? CF_ZC_PATTERN : CF_ZC_SEQUENCE);
        return;
    }
    if ((sign_out != 0) && (sign_out != sign_in))      { IsrFail(CF_SENSE_MISMATCH); return; }

    g_ctrl.events++;
    g_ctrl.pending_pol = (int8_t)(-sign_in);           /* the NEXT half-cycle has the opposite sign */

    {
        uint32_t dly = CTRL_DEADTIME_US;
#if (SLAVE_CONV_FIRST_AT_PEAK != 0)
        /* EXPERIMENT: the FIRST diagonal after a start is turned on at the line peak (a quarter cycle after the event), not 40 us after it */
        if ((g_ctrl.first_pending != 0U) && ((SLAVE_CONV_FIRST_POL == 0) || (g_ctrl.pending_pol == (int8_t)SLAVE_CONV_FIRST_POL))) dly = interval / 2U;
#endif
        CtrlHw_ArmDeadtime((uint32_t)(now_us + dly));                               /* new diagonal on   */
    }
    CtrlHw_ArmPredictOff((uint32_t)(now_us + interval - CTRL_PREDICT_LEAD_US));     /* ...and off again, */
    CtrlHw_ArmWatchdog((uint32_t)(now_us + interval + CTRL_WD_MARGIN_US));          /* event overdue     */
}

#if (SLAVE_LEGACY_LOWSIDES != 0)
static volatile uint8_t  s_lows_armed;          /* the inverter outputs are enabled at zero duty (low sides on) for the converter start */
static volatile uint32_t s_lows_base_comm;      /* g_ctrl.commutations when the converter was started */
#endif

void SlaveCtrl_OnDeadtimeIrq(void)
{
    int8_t   pol;
    uint16_t l1, l2;

    if ((g_ctrl.run_allowed == 0U) || (g_ctrl.isr_fault != 0U)) return;

    pol = g_ctrl.pending_pol;

#if (SLAVE_CONV_FIRST_POL != 0)
    /* DIAGNOSTIC: after a start, skip half-cycles until the chosen polarity comes; everything after the first diagonal is unchanged. */
    if ((g_ctrl.first_pending != 0U) && (pol != (int8_t)SLAVE_CONV_FIRST_POL)) return;
#endif

    /* LAST CHECK before energising: the newest samples must not clearly show the
     * OPPOSITE sign. If they do, this zero-cross event came early (noise, a glitch) -
     * the line has not reversed yet - and energising now would conduct against it. */
    {
        int8_t s = CtrlHw_SignLatest(1U);
        if ((s != 0) && (pol != 0) && (s != pol)) { IsrFail(CF_PRE_ENERGISE); return; }
    }

#if (SLAVE_CONV_SWAP != 0)       /* DANGEROUS if set without a scope check: see slave_config.h */
    if      (pol > 0) SlavePower_ConverterNegative();
    else if (pol < 0) SlavePower_ConverterPositive();
#else
    if      (pol > 0) SlavePower_ConverterPositive();
    else if (pol < 0) SlavePower_ConverterNegative();
#endif
    else              return;

    g_ctrl.conv_pol = pol;
    g_ctrl.first_pending = 0U;
    g_ctrl.commutations++;
    s_commut_us = CtrlHw_NowUs();

#if (SLAVE_LEGACY_LOWSIDES != 0) && (SLAVE_LOWSIDES_OFF_AFTER > 0)
    /* the old firmware: the FIRST diagonal runs with the inverter low sides still on, they are disabled from the SECOND diagonal on */
    if ((s_lows_armed != 0U) && ((uint32_t)(g_ctrl.commutations - s_lows_base_comm) >= (uint32_t)SLAVE_LOWSIDES_OFF_AFTER))
    {
        SlavePower_InverterDisable();
        s_lows_armed = 0U;
    }
#endif

    if ((g_ctrl.inv_active != 0U) && (g_ctrl.duty_ok != 0U))     /* rule R1: nonzero duty only with the shunts open */
    {
        SlaveCtrl_LegsFor(pol, g_ctrl.boost, g_ctrl.duty_counts, &l1, &l2);
        SlavePower_SyncCounters();
        SlavePower_SetLegDuty(l1, l2);
    }
}

/* Predicted next zero-cross: the diagonal comes OFF now, whether or not the
 * event arrives. If it does, the dead-time compare energises the next one; if
 * it does not (lost pulse, dead IC), the converter simply stays off. */
void SlaveCtrl_OnPredictOffIrq(void)
{
    if (g_ctrl.conv_pol != 0)
    {
        SlavePower_ConverterOffNow();
        g_ctrl.conv_pol = 0;
        if (g_ctrl.inv_active != 0U) SlavePower_SetLegDuty(0U, 0U);
    }
}

/* 20 kHz polarity guard: while a diagonal is on, the newest ADC samples must not
 * show the OPPOSITE sign. Catches what the zero-cross timing cannot: a phase jump,
 * a wrong-way sense, a missing event after a discontinuity. */
static uint8_t s_guard_bad;             /* consecutive ticks that showed the wrong sign */

void SlaveCtrl_OnSampleTick(void)
{
    int8_t s;

    if (g_ctrl.conv_pol == 0) { s_guard_bad = 0U; return; }
    /* signed: s_commut_us is written by a higher-priority interrupt that may have run after the clock read */
    if ((int32_t)(CtrlHw_NowUs() - s_commut_us) < (int32_t)CTRL_GUARD_BLANK_US) { s_guard_bad = 0U; return; }

    s = CtrlHw_SignLatest(0U);
    if ((s != 0) && (s != g_ctrl.conv_pol))
    {
        if (++s_guard_bad >= CTRL_GUARD_CONFIRM_TICKS) { s_guard_bad = 0U; IsrFail(CF_LINE_REVERSED); }
    }
    else
    {
        s_guard_bad = 0U;                /* a single odd sample is noise, not a reversal */
    }
}

void SlaveCtrl_OnWatchdogIrq(void)
{
    if (g_ctrl.run_allowed != 0U) IsrFail(CF_ZC_LOST);
    else                          SlavePower_AllOffNow();
}

uint16_t SlaveCtrl_TripCapture(void)
{
    return g_ctrl.trip_cap;
}


/* The diag2 field carries ONE thing at a time. The mapping-check counters are only ever INCREMENTED (never cleared), so once the converter is idle
 * they stay non-zero for good: they must NOT shadow the transformer result while the converter runs (v0.6.11 did exactly that, so the result was never
 * shown). Policy: a running trip while in FAULT > the transformer result while the converter runs (nothing else, the mapping counters are stale then) >
 * the passive mapping check. */
uint16_t SlaveCtrl_RegWord(void)
{
#if (SLAVE_POWER_STAGE >= 3)
    if (g_ctrl.state != CTRL_RUN) return 0U;
    {
        const uint32_t eff = s_reg_eff;
        uint32_t q = (eff + 2U) / 4U;
        if (q > 1023U) q = 1023U;
        return (uint16_t)(0x1000U | ((s_reg_boost != 0U) ? 0x0800U : 0U) | ((eff >= (uint32_t)(CTRL_STAGE3_MAX_EFF_TICKS - 1U)) ? 0x0400U : 0U) | (uint16_t)q);
    }
#else
    return 0U;
#endif
}

uint16_t SlaveCtrl_Diag2(void)
{
    const uint16_t trip = SlaveCtrl_TripCapture();
    if ((SlaveCtrl_FaultContext() != 0U) && (trip != 0U)) return trip;
#if (SLAVE_POWER_STAGE >= 3)
    if ((g_ctrl.state >= CTRL_RECT) && (g_ctrl.state <= CTRL_RESHUNT)) return SlaveCtrl_RegWord();       /* stage 3: the regulator's state (0 outside RUN) */
#else
    if ((g_ctrl.state >= CTRL_RECT) && (g_ctrl.state <= CTRL_RESHUNT)) return SlaveCtrl_TfWord();
#endif
    return SlaveCtrl_MapWord();
}

uint16_t SlaveCtrl_MapWord(void)
{
    if ((g_ctrl.map_agree == 0U) && (g_ctrl.map_disagree == 0U)) return 0U;
    return (uint16_t)(0x4000U | (uint16_t)(g_ctrl.map_agree & 127U) | ((uint16_t)(g_ctrl.map_disagree & 127U) << 7));
}

/* Called from the FLT / break interrupts BEFORE anything is switched off. SIGNED age (see the zero-cross race fix). */
static void CaptureTrip(void)
{
    int32_t  age = (int32_t)(CtrlHw_NowUs() - g_ctrl.t_event_us);
    uint32_t u;
    uint16_t w;

    if (age < 0) age = 0;
    u = (uint32_t)age >> 4;
    if (u > 1023U) u = 1023U;
    w = (uint16_t)(0x8000U | u);
    if (g_ctrl.conv_pol > 0)      w |= 0x0400U;
    else if (g_ctrl.conv_pol < 0) w |= 0x0800U;
    g_ctrl.trip_cap = w;
}

uint16_t SlaveCtrl_DiagWord(void)
{
    return (uint16_t)(((uint16_t)(g_ctrl.flt_idle & 31U))        | ((uint16_t)(g_ctrl.flt_run & 31U) << 5) |
                      ((uint16_t)(g_ctrl.brk_idle & 7U) << 10)   | ((uint16_t)(g_ctrl.brk_run & 7U) << 13));
}

void SlaveCtrl_OnBreakIrq(uint32_t flags)
{
    /* The FLT interrupt (IRQ 6) is served BEFORE the break interrupt (IRQ 13 / 20) and clears run_allowed: an event that is
     * part of a running incident that the other interrupt has already started (isr_fault set) still counts as RUNNING. */
    if ((g_ctrl.run_allowed != 0U) || (g_ctrl.isr_fault != 0U)) { if (g_ctrl.brk_run  < 7U)  g_ctrl.brk_run++;  }
    else                                                        { if (g_ctrl.brk_idle < 7U)  g_ctrl.brk_idle++; }
    if (g_ctrl.run_allowed != 0U) CaptureTrip();
    SlavePower_AllOffNow();                 /* converter pins have no hardware break */
    CtrlHw_DisarmDeadtime();
    CtrlHw_DisarmPredictOff();
    CtrlHw_DisarmWatchdog();
    g_ctrl.break_flags |= (uint8_t)flags;
    if (g_ctrl.run_allowed != 0U)
    {
        g_ctrl.run_allowed = 0U;
        g_ctrl.inv_active  = 0U;
        g_ctrl.duty_ok     = 0U;
        g_ctrl.conv_pol    = 0;
        if (g_ctrl.isr_fault == 0U) g_ctrl.isr_fault = CF_BREAK;
    }
}

void SlaveCtrl_OnFltIrq(void)
{
    SlavePower_AllOffNow();       /* FIRST (decision D25). On the target EXTI2_3_IRQHandler has already done it: the second call is harmless and costs nothing that matters. */
    if ((g_ctrl.run_allowed != 0U) || (g_ctrl.isr_fault != 0U)) { if (g_ctrl.flt_run  < 31U) g_ctrl.flt_run++;  }
    else                                                        { if (g_ctrl.flt_idle < 31U) g_ctrl.flt_idle++; }
    if (g_ctrl.run_allowed != 0U) CaptureTrip();                /* reads only a timer count and two controller fields: independent of the output stage */
    CtrlHw_DisarmDeadtime();
    CtrlHw_DisarmPredictOff();
    CtrlHw_DisarmWatchdog();
    if (g_ctrl.run_allowed != 0U)
    {
        g_ctrl.run_allowed = 0U;
        g_ctrl.inv_active  = 0U;
        g_ctrl.duty_ok     = 0U;
        g_ctrl.conv_pol    = 0;
        if (g_ctrl.isr_fault == 0U) g_ctrl.isr_fault = CF_FLT;
    }
}

/* ------------------------------------------------------------------------- */
/* Main-loop side                                                             */
/* ------------------------------------------------------------------------- */
static Reg_t    s_reg;
/* The regulated output. A constant equal to the old compile-time CTRL_TARGET_V in every build without master control, so those builds behave exactly as before; with master
 * control it is the set voltage the master sent, adopted while the unit is not energised. */
static float    s_target_v = (float)CTRL_TARGET_V;

#if (SLAVE_INJ_GUARD != 0) && (SLAVE_POWER_STAGE != 2)
/* ---- the injection guard: never reverse the injection at a large correction, and never follow an input step with the old correction ---- */
static uint8_t  s_dir_applied;        /* the direction the converter is told to use                                   */
static uint8_t  s_guard_on;           /* the correction is being held at zero                                         */
static uint32_t s_guard_until_ms;
static float    s_vin_ref;
static uint32_t s_vin_cnt;
static uint32_t s_guard_events;
uint32_t SlaveCtrl_GuardCount(void) { return s_guard_events; }
#else
uint32_t SlaveCtrl_GuardCount(void) { return 0U; }
#endif
static float    s_last_vout;                 /* the latest output measurement, for the IN_BAND flag */
static uint8_t  s_last_vout_valid;
static uint16_t s_mc_setv_dv;                /* the set voltage in use as the master sent it; 0 = none accepted yet */
static uint8_t  s_mc_clr, s_mc_clr_valid;
static uint32_t s_mc_last_m_ms;
static uint8_t  s_mc_link_lost;
static uint16_t s_mc_maxc_dv;
static uint32_t s_restart_at_ms;
static uint32_t s_last_trip_ms;
static uint32_t s_state_since_ms;
static uint32_t s_rect_base_comm;

#if (SLAVE_TF_CHECK != 0) || (SLAVE_POWER_STAGE == 2)
static volatile uint16_t s_tf_word;      /* the transformer result for diag2 (converter-only check or stage-2 injection test) */
#endif

#if (SLAVE_TF_CHECK != 0)
/* Transformer-connection check: average (Vout - Vin) over a window once the converter has really been running. One result per converter start. */
static float             s_tf_sum;
static uint32_t          s_tf_n, s_tf_seq, s_tf_t0_ms;
static uint8_t           s_tf_run, s_tf_done;
#endif
static uint32_t s_rect_ref_ms;
static uint8_t  s_rect_ref_set;

#if (SLAVE_TF_CHECK != 0)
static void TfCheckStep(const CtrlMeas_t *m, uint32_t now_ms)
{
    if ((s_rect_ref_set == 0U) || ((uint32_t)(now_ms - s_rect_ref_ms) < CTRL_TF_SETTLE_MS))
    {
        s_tf_run = 0U; s_tf_done = 0U;                       /* not running yet / a new start: measure again */
        return;
    }
    if ((s_tf_done != 0U) || (m->valid == 0U)) return;
    if (s_tf_run == 0U)
    {
        s_tf_run = 1U; s_tf_sum = 0.0f; s_tf_n = 0U; s_tf_seq = m->update_count; s_tf_t0_ms = now_ms;
    }
    if (m->update_count != s_tf_seq)
    {
        s_tf_seq = m->update_count;
        s_tf_sum += (m->vout - m->vin);
        s_tf_n++;
    }
    if ((uint32_t)(now_ms - s_tf_t0_ms) >= CTRL_TF_WINDOW_MS)
    {
        s_tf_done = 1U;
        if (s_tf_n >= CTRL_TF_MIN_SAMPLES)
        {
            const float mean = s_tf_sum / (float)s_tf_n;
            int32_t     dv10 = (int32_t)((mean >= 0.0f) ? (mean * 10.0f + 0.5f) : (mean * 10.0f - 0.5f));
            const uint16_t verdict = (mean > CTRL_TF_DEADBAND_V) ? 1U : ((mean < -CTRL_TF_DEADBAND_V) ? 2U : 3U);
            if (dv10 >  1023) dv10 =  1023;
            if (dv10 < -1024) dv10 = -1024;
            s_tf_word = (uint16_t)(0x2000U | (uint16_t)(verdict << 11) | ((uint16_t)dv10 & 0x07FFU));
        }
    }
}
#endif

#if (SLAVE_POWER_STAGE == 2)
/* ---- Stage 2: the one-shot open-loop INJECTION TEST (phases and verdict: see slave_ctrl.h, CTRL_INJ_*) ---- */
typedef enum { INJ_BASE = 0, INJ_RAMP, INJ_HOLD, INJ_DOWN, INJ_DONE } InjPhase_t;
static uint8_t  s_inj_phase, s_inj_boost;
static uint32_t s_inj_t0_ms, s_inj_seq, s_inj_base_n, s_inj_hold_n;
static float    s_inj_base_sum, s_inj_hold_sum, s_inj_base;

static void InjReset(uint32_t now_ms)
{
    s_inj_phase = (uint8_t)INJ_BASE; s_inj_t0_ms = now_ms; s_inj_seq = 0U; s_inj_base_n = 0U; s_inj_hold_n = 0U;
    s_inj_base_sum = 0.0f; s_inj_hold_sum = 0.0f; s_inj_base = 0.0f; s_inj_boost = 0U;
    s_tf_word = 0U;                                     /* the previous result is replaced at every start */
}

static void InjResult(float eff, uint16_t verdict)
{
    int32_t dv10 = (int32_t)((eff >= 0.0f) ? (eff * 10.0f + 0.5f) : (eff * 10.0f - 0.5f));
    if (dv10 >  1023) dv10 =  1023;
    if (dv10 < -1024) dv10 = -1024;
    s_tf_word = (uint16_t)(0x2000U | (uint16_t)(verdict << 11) | ((uint16_t)dv10 & 0x07FFU));
}

#define INJ_CCR(eff)  CTRL_EFF_TO_CCR(eff)           /* (stage 2: the injection test uses the shared mapping) */

/* One pass of the test: returns the CCR (counts) for this pass and keeps s_inj_phase / s_inj_boost current. */
static uint16_t InjStep(const CtrlMeas_t *m, uint32_t now_ms)
{
    const uint32_t t     = (uint32_t)(now_ms - s_inj_t0_ms);
    const uint8_t  fresh = (uint8_t)((m->valid != 0U) && (m->update_count != s_inj_seq));
    const float    sgn   = (s_inj_boost != 0U) ? 1.0f : -1.0f;
    float          chg   = 0.0f;                        /* the change in (Vout - Vin) against the baseline, counted in the EXPECTED direction */
    if (fresh) s_inj_seq = m->update_count;
    if ((fresh != 0U) && (s_inj_phase >= (uint8_t)INJ_RAMP) && (s_inj_phase <= (uint8_t)INJ_HOLD))
    {
        chg = sgn * ((m->vout - m->vin) - s_inj_base);
        if ((chg > CTRL_INJ_ABORT_V) || (chg < -CTRL_INJ_ABORT_V))
        {
            InjResult(chg, 3U);                         /* far more than expected, in either direction: stop injecting at once */
            s_inj_phase = (uint8_t)INJ_DONE;
            return 0U;
        }
    }
    switch (s_inj_phase)
    {
    case INJ_BASE:
        if (fresh != 0U) { s_inj_base_sum += (m->vout - m->vin); s_inj_base_n++; }
        if (t >= CTRL_INJ_BASE_MS)
        {
            if (s_inj_base_n < CTRL_TF_MIN_SAMPLES) { InjResult(0.0f, 3U); s_inj_phase = (uint8_t)INJ_DONE; return 0U; }
            s_inj_base  = s_inj_base_sum / (float)s_inj_base_n;
            s_inj_boost = (uint8_t)(m->vin < s_target_v);                /* fixed for the whole test */
            s_inj_phase = (uint8_t)INJ_RAMP; s_inj_t0_ms = now_ms;
        }
        return 0U;
    case INJ_RAMP:
        if (t >= CTRL_INJ_RAMP_MS) { s_inj_phase = (uint8_t)INJ_HOLD; s_inj_t0_ms = now_ms; return INJ_CCR(CTRL_STAGE2_DUTY_COUNTS); }
        return INJ_CCR(((uint32_t)CTRL_STAGE2_DUTY_COUNTS * t) / CTRL_INJ_RAMP_MS);
    case INJ_HOLD:
        if ((fresh != 0U) && (t >= CTRL_INJ_SETTLE_MS)) { s_inj_hold_sum += (m->vout - m->vin); s_inj_hold_n++; }
        if (t >= CTRL_INJ_HOLD_MS)
        {
            if (s_inj_hold_n < CTRL_TF_MIN_SAMPLES) InjResult(0.0f, 3U);
            else
            {
                const float eff = sgn * ((s_inj_hold_sum / (float)s_inj_hold_n) - s_inj_base);
                InjResult(eff, (eff > CTRL_INJ_DEADBAND_V) ? 1U : ((eff < -CTRL_INJ_DEADBAND_V) ? 2U : 3U));
            }
            s_inj_phase = (uint8_t)INJ_DOWN; s_inj_t0_ms = now_ms;
        }
        return INJ_CCR(CTRL_STAGE2_DUTY_COUNTS);
    case INJ_DOWN:
        if (t >= CTRL_INJ_DOWN_MS) { s_inj_phase = (uint8_t)INJ_DONE; return 0U; }
        return INJ_CCR(((uint32_t)CTRL_STAGE2_DUTY_COUNTS * (CTRL_INJ_DOWN_MS - t)) / CTRL_INJ_DOWN_MS);
    default:
        return 0U;
    }
}
#endif

uint16_t SlaveCtrl_TfWord(void)
{
#if (SLAVE_TF_CHECK != 0) || (SLAVE_POWER_STAGE == 2)
    return s_tf_word;
#else
    return 0U;
#endif
}
static uint32_t s_last_upd;
static uint32_t s_last_meas_ms;
static uint8_t  s_vin_bad, s_vout_bad;
static uint8_t  s_meas_seen;

static void StopOutputs(void)
{
    uint32_t p = CtrlHw_Lock();
    SlavePower_AllOffNow();
    CtrlHw_DisarmDeadtime();
    CtrlHw_DisarmPredictOff();
    CtrlHw_DisarmWatchdog();
    g_ctrl.run_allowed = 0U;
    g_ctrl.inv_active  = 0U;
    g_ctrl.duty_ok     = 0U;
    g_ctrl.conv_pol    = 0;
    g_ctrl.duty_counts = 0U;
#if (SLAVE_LEGACY_LOWSIDES != 0)
    s_lows_armed = 0U;                       /* AllOffNow has disabled the inverter outputs */
#endif
    CtrlHw_Unlock(p);
}

/* A restart never trusts the previous synchronisation. */
static void Resync(void)
{
    uint32_t p = CtrlHw_Lock();
    s_trk_all.have_prev  = 0U; s_trk_all.good  = 0U; s_trk_all.sign_last  = 0;
    s_trk_fall.have_prev = 0U; s_trk_fall.good = 0U; s_trk_fall.sign_last = 0;
    g_ctrl.mode        = CTRL_ZC_UNKNOWN;
    g_ctrl.good_all    = 0U;
    g_ctrl.good_fall   = 0U;
    g_ctrl.isr_fault   = 0U;
    g_ctrl.break_flags = 0U;
    g_ctrl.pending_pol = 0;
    g_ctrl.run_allowed = 0U;
    g_ctrl.inv_active  = 0U;
    g_ctrl.duty_ok     = 0U;
    g_ctrl.conv_pol    = 0;
    CtrlHw_Unlock(p);
}

static uint8_t IsEscalating(uint16_t code)
{
    switch (code)
    {
        case CF_BREAK: case CF_FLT: case CF_ZC_PATTERN: case CF_ZC_SEQUENCE: case CF_PRE_ENERGISE: case CF_POLARITY:
        case CF_SENSE_MISMATCH: case CF_VOUT_HIGH: case CF_INV_ENABLE: case CF_LINE_REVERSED: case CF_RELAY: case CF_NO_COMMUTATION:
            return 1U;
        default:
            return 0U;      /* ZC_LOST, VIN_RANGE, MEAS_STALE: conditions, not hardware events */
    }
}

static uint16_t s_resh_fault;            /* fault to report once the shunts have closed (0 = a normal stop) */

/* Overcurrent-class faults: everything is switched off AT ONCE (the hardware has already done so for break / FLT). */
static uint8_t IsHardFault(uint16_t code)
{
    return (uint8_t)((code == CF_BREAK) || (code == CF_FLT) || (code == CF_RELAY));
}

/* Duty to zero NOW (low sides on, primary shorted by the IGBTs) and no further duty until the shunts are open again. */
static void ZeroDutyNow(void)
{
    uint32_t p = CtrlHw_Lock();
    g_ctrl.duty_ok     = 0U;
    g_ctrl.duty_counts = 0U;
    if (g_ctrl.inv_active != 0U) SlavePower_SetLegDuty(0U, 0U);
    CtrlHw_Unlock(p);
}

/* Unwind with the primary kept controlled: duty zero, then the shunts closed, and only then (CTRL_RESHUNT handler)
 * is the inverter disabled. code == 0: a normal stop (the converter keeps running meanwhile). */
static void BeginReshunt(uint16_t code, uint32_t now_ms)
{
    ZeroDutyNow();
    if (code != 0U)
    {
        uint32_t p = CtrlHw_Lock();
        SlavePower_ConverterOffNow();
        CtrlHw_DisarmDeadtime();
        CtrlHw_DisarmPredictOff();
        CtrlHw_DisarmWatchdog();
        g_ctrl.run_allowed = 0U;
        g_ctrl.inv_active  = 0U;
        g_ctrl.conv_pol    = 0;
        CtrlHw_Unlock(p);
    }
    SlaveRelay_ShortNow();
    s_resh_fault     = code;
    g_ctrl.isr_fault = 0U;
    g_ctrl.state     = CTRL_RESHUNT;
    s_state_since_ms = now_ms;
}

static void FinishFault(uint16_t code, uint32_t now_ms)
{
    uint32_t delay;

    StopOutputs();                        /* converter off, inverter disabled */
    SlaveRelay_ShortNow();                /* shunts closed (a no-op if they already are) */
    s_resh_fault     = 0U;
    g_ctrl.fault     = code;
    g_ctrl.isr_fault = 0U;
    g_ctrl.state     = CTRL_FAULT;
    s_state_since_ms = now_ms;
    s_last_trip_ms   = now_ms;

    if (SLAVE_MASTER_CONTROL != 0)
    {
        g_ctrl.state = CTRL_LATCHED;             /* D5: with a master EVERY fault latches until the clear counter changes: no automatic retries, no escalation count */
        return;
    }
    if (IsEscalating(code) != 0U)
    {
        if (g_ctrl.fault_count < 255U) g_ctrl.fault_count++;
        if (g_ctrl.fault_count >= CTRL_LATCH_AFTER)
        {
            g_ctrl.state = CTRL_LATCHED;           /* manual reset only */
            return;
        }
        delay = (uint32_t)g_ctrl.fault_count * CTRL_RESTART_STEP_MS;
        if (delay > CTRL_RESTART_MAX_MS) delay = CTRL_RESTART_MAX_MS;
    }
    else
    {
        delay = CTRL_CONDITION_RETRY_MS;
    }
    s_restart_at_ms = (uint32_t)(now_ms + delay);
}

static void EnterFault(uint16_t code, uint32_t now_ms)
{
    if (g_ctrl.state != CTRL_RESHUNT)
    {
        /* Snapshot BEFORE anything changes: where we were, which pattern mode, and whether the hardware
         * fault lines are still asserted right now (still low = real and lasting; high = a glitch). */
        uint8_t ctx = (uint8_t)(CTRL_CTX_VALID | ((uint8_t)g_ctrl.state & CTRL_CTX_STATE_MASK) |
                                (uint8_t)((g_ctrl.mode & 3U) << CTRL_CTX_MODE_SHIFT));
        if (SlavePower_FltAsserted() != 0U)       ctx |= CTRL_CTX_FLT_LOW;
        if (SlavePower_BreakInputActive() != 0U)  ctx |= CTRL_CTX_BKIN_LOW;
        g_ctrl.fault_ctx = ctx;
    }
    else
    {
        /* already unwinding with the primary controlled */
        g_ctrl.isr_fault = 0U;
        if (IsHardFault(code) != 0U)        FinishFault(code, now_ms);    /* overcurrent-class: off at once */
        else if (s_resh_fault == 0U)        s_resh_fault = code;          /* report it when the unwinding ends */
        return;
    }

    /* Keep the primary controlled while the shunts close, unless this is an overcurrent-class fault or the
     * shunts are already closed / the inverter is not enabled (nothing to unwind). */
#if (SLAVE_RELAYS_MODE == 2)
    /* LEGACY: SlaveRelay_IsShorted() only means "the sequence is complete" - the nets are OPEN (W on) whenever the inverter is being driven, and it is the zero-duty
     * low sides that control the primary. So: unwind (duty zero, coils released, inverter disabled only after the W contact has closed) whenever the inverter is ACTIVE. */
    if ((IsHardFault(code) == 0U) && (SlavePower_InverterIsEnabled() != 0U) && (g_ctrl.inv_active != 0U))
#else
    if ((IsHardFault(code) == 0U) && (SlavePower_InverterIsEnabled() != 0U) && (SlaveRelay_IsShorted() == 0U))
#endif
    {
        BeginReshunt(code, now_ms);
        return;
    }
    FinishFault(code, now_ms);
}

/* Debounced range checks, evaluated once per NEW RMS result. */
static void TrackMeas(const CtrlMeas_t *m, uint32_t now_ms)
{
    if (m->valid == 0U) return;

    if ((s_meas_seen == 0U) || (m->update_count != s_last_upd))
    {
        s_meas_seen    = 1U;
        s_last_upd     = m->update_count;
        s_last_meas_ms = now_ms;

#if (SLAVE_TEST_NO_VOLTAGE_CUTS == 0)
        if ((m->vin < CTRL_VIN_MIN_V) || (m->vin > CTRL_VIN_MAX_V)) { if (s_vin_bad < 255U) s_vin_bad++; }
        else                                                          { s_vin_bad = 0U; }

        if (m->vout > CTRL_VOUT_MAX_V) { if (s_vout_bad < 255U) s_vout_bad++; }
        else                            { s_vout_bad = 0U; }
#else
        s_vin_bad = 0U; s_vout_bad = 0U;                      /* TEST: no input-range and no output-high trip */
#endif
    }
}

static uint8_t SelectedGood(void)
{
    if (g_ctrl.mode == CTRL_ZC_BOTH) return g_ctrl.good_all;
    if (g_ctrl.mode == CTRL_ZC_FALL) return g_ctrl.good_fall;
    return 0U;
}

/* Checks common to every state in which something is energised. Returns the
 * fault code, or 0 if all is well. */
static uint16_t RunningChecks(uint32_t now_ms, uint32_t now_us, uint8_t relay_expect)
{
    if ((relay_expect == 1U) && (SlaveRelay_IsShorted() == 0U))           return CF_RELAY;
    if ((relay_expect == 2U) && (SlaveRelay_IsFree() == 0U))              return CF_RELAY;
    if (SlavePower_FltAsserted() != 0U)                                   return CF_FLT;
    if (s_vin_bad  >= CTRL_RANGE_DEBOUNCE)                                return CF_VIN_RANGE;
    if (s_vout_bad >= CTRL_RANGE_DEBOUNCE)                                return CF_VOUT_HIGH;
    if ((uint32_t)(now_ms - s_last_meas_ms) > CTRL_MEAS_STALE_MS)         return CF_MEAS_STALE;
    /* Backs up the compare watchdog. SIGNED age on purpose: t_event_us is written by the zero-cross
     * INTERRUPT, which can fire after this pass read the clock - the timestamp is then newer than
     * 'now_us' and an unsigned subtraction would wrap to ~4e9 ("no zero-cross for an hour"). */
    if ((int32_t)(now_us - g_ctrl.t_event_us) > (int32_t)(CTRL_WATCHDOG_US + 5000U)) return CF_ZC_STALE;
    return 0U;
}

void SlaveCtrl_Init(void)
{
    uint32_t i;
    uint8_t *p = (uint8_t *)&g_ctrl;

    CtrlHw_Init();
    StopOutputs();
    SlaveRelay_Init();

    for (i = 0U; i < (uint32_t)sizeof(g_ctrl); i++) p[i] = 0U;
    g_ctrl.state       = CTRL_OFF;
    g_ctrl.mode        = CTRL_ZC_UNKNOWN;
    g_ctrl.run_request = (uint8_t)(SLAVE_BENCH_AUTORUN != 0);       /* with master control SLAVE_BENCH_AUTORUN is 0: the master decides */
    s_mc_last_m_ms = CtrlHw_NowMs();                                 /* the link timeout runs from the start */
    s_mc_link_lost = 0U; s_mc_clr_valid = 0U; s_mc_clr = 0U; s_mc_setv_dv = 0U; s_mc_maxc_dv = 0U;
    s_last_vout_valid = 0U;
    s_target_v = (float)CTRL_TARGET_V;
    s_vin_bad = s_vout_bad = 0U;
    s_meas_seen = 0U;
#if (SLAVE_TF_CHECK != 0) || (SLAVE_POWER_STAGE == 2)
    s_tf_word = 0U;
#endif
#if (SLAVE_TF_CHECK != 0)
    s_tf_run = 0U; s_tf_done = 0U; s_tf_n = 0U; s_tf_sum = 0.0f;
#endif
    Resync();
}

void SlaveCtrl_SetRunRequest(uint8_t on)
{
    g_ctrl.run_request = (uint8_t)(on != 0U);
}

void SlaveCtrl_ClearLatch(void)
{
    if (g_ctrl.state == CTRL_LATCHED)
    {
        g_ctrl.fault_count = 0U;
        g_ctrl.fault       = CF_NONE;
        g_ctrl.state       = CTRL_OFF;
    }
}

void SlaveCtrl_OnControl(uint8_t run, uint16_t setv_dv, uint8_t clr, uint16_t maxc_dv, uint32_t now_ms)
{
#if (SLAVE_MASTER_CONTROL != 0)
    const uint8_t energised = (uint8_t)((g_ctrl.state >= CTRL_RECT) && (g_ctrl.state <= CTRL_RESHUNT));
    s_mc_last_m_ms = now_ms;
    s_mc_link_lost = 0U;
    g_ctrl.run_request = (uint8_t)(run != 0U);                    /* a LEVEL: every $M restates it */
    if (energised == 0U)
    {
        s_target_v   = (float)setv_dv * 0.1f;                     /* picked up the moment the unit stops, since the master repeats it */
        s_mc_setv_dv = setv_dv;
    }
    s_mc_maxc_dv = maxc_dv;
    if (s_mc_clr_valid == 0U)
    {
        s_mc_clr = clr; s_mc_clr_valid = 1U;                      /* the first $M after a start only adopts the counter */
    }
    else if (clr != s_mc_clr)
    {
        s_mc_clr = clr;
        SlaveCtrl_ClearLatch();                                   /* acts only if LATCHED; the normal start checks the cause again */
    }
#else
    (void)run; (void)setv_dv; (void)clr; (void)maxc_dv; (void)now_ms;
#endif
}

uint16_t SlaveCtrl_Flags2(void)
{
    uint16_t f = 0U;
#if (SLAVE_MASTER_CONTROL != 0)
    if ((g_ctrl.run_request != 0U) && (s_mc_link_lost == 0U)) f |= LINKV2_FLAG_RUN_ACK;
    if (s_mc_link_lost != 0U)                                  f |= LINKV2_FLAG_LINK_LOST;
#endif
    if ((g_ctrl.state == CTRL_RUN) && (s_last_vout_valid != 0U))
    {
        float d = s_last_vout - s_target_v;
        if (d < 0.0f) d = -d;
        if (d <= CTRL_IN_BAND_V) f |= LINKV2_FLAG_IN_BAND;
    }
#if (SLAVE_POWER_STAGE >= 3)
    if ((g_ctrl.state == CTRL_RUN) && (s_reg_eff >= (uint16_t)(CTRL_STAGE3_MAX_EFF_TICKS - 1U))) f |= LINKV2_FLAG_AT_CAP;
#endif
    if (g_ctrl.state == CTRL_LATCHED) f |= LINKV2_FLAG_FAULT_LATCHED;
    return f;
}

uint16_t SlaveCtrl_SetvDv(void)
{
#if (SLAVE_MASTER_CONTROL != 0)
    return s_mc_setv_dv;
#else
    return (uint16_t)((s_target_v * 10.0f) + 0.5f);
#endif
}

uint8_t  SlaveCtrl_ClrEcho(void)    { return s_mc_clr; }
uint16_t SlaveCtrl_MaxCorrDv(void)  { return s_mc_maxc_dv; }

uint8_t SlaveCtrl_LinkState(void)
{
    switch (g_ctrl.state)
    {
        case CTRL_OFF:       return 0U;
        case CTRL_SYNC:
        case CTRL_RECT:
        case CTRL_INV_START:
        case CTRL_UNSHUNT:
        case CTRL_RESHUNT:   return 1U;
        case CTRL_RUN:       return 2U;
        default:             return 3U;     /* FAULT, LATCHED */
    }
}

uint8_t SlaveCtrl_WaitContext(void)
{
    return (g_ctrl.state == CTRL_SYNC) ? g_ctrl.wait_ctx : 0U;
}

uint8_t SlaveCtrl_FaultContext(void)
{
    return ((g_ctrl.state == CTRL_FAULT) || (g_ctrl.state == CTRL_LATCHED)) ? g_ctrl.fault_ctx : 0U;
}

uint16_t SlaveCtrl_FaultCode(void)
{
    return ((g_ctrl.state == CTRL_FAULT) || (g_ctrl.state == CTRL_LATCHED)) ? g_ctrl.fault : 0U;
}

void SlaveCtrl_Service(void)
{
    CtrlMeas_t m;
    uint32_t   now_ms = CtrlHw_NowMs();
    uint32_t   now_us = CtrlHw_NowUs();
    uint16_t   f;

    SlavePower_BreakIrqRearm();            /* break interrupts stay masked while a BKIN is held */
    CtrlHw_GetMeas(&m);
    TrackMeas(&m, now_ms);
    s_last_vout = m.vout; s_last_vout_valid = m.valid;

    if (SLAVE_MASTER_CONTROL != 0)
    {   /* D9: no valid $M for the timeout = the master is gone: stop. It starts again only when valid $M frames return with run = 1. */
        if ((uint32_t)(now_ms - s_mc_last_m_ms) > CTRL_LINK_TIMEOUT_MS)
        {
            s_mc_link_lost = 1U;
            g_ctrl.run_request = 0U;
        }
    }

    /* Relay coils: the primary is FREE (shunts open) only in UNSHUNT and RUN, SHORTED otherwise. The duty is already
     * ZERO whenever the state changes into or out of those (rule R2): it is zeroed in the pass that changes the state,
     * and this request is issued at the top of the NEXT pass. */
    SlaveRelay_Request((uint8_t)((g_ctrl.state == CTRL_UNSHUNT) || (g_ctrl.state == CTRL_RUN)));
    if ((g_ctrl.state >= CTRL_RECT) && (g_ctrl.state <= CTRL_RUN)) SlaveRelay_Hold();   /* legacy mode: keep the relays while the converter / inverter run - NOT while unwinding (CTRL_RESHUNT): a hold there would re-arm the SH pulse */
    SlaveRelay_Service(now_ms);

    /* Choose the zero-cross hypothesis once exactly one of them has proven itself. */
    if (g_ctrl.mode == CTRL_ZC_UNKNOWN)
    {
        uint8_t a = (uint8_t)(g_ctrl.good_all  >= CTRL_SYNC_GOOD_EVENTS);
        uint8_t b = (uint8_t)(g_ctrl.good_fall >= CTRL_SYNC_GOOD_EVENTS);
        if      (a && !b) g_ctrl.mode = CTRL_ZC_BOTH;
        else if (b && !a) g_ctrl.mode = CTRL_ZC_FALL;
    }

    /* The escalation counter resets after 5 minutes of fault-free running. */
    if ((g_ctrl.fault_count != 0U) && (g_ctrl.state == CTRL_RUN) &&
        ((uint32_t)(now_ms - s_last_trip_ms) >= CTRL_COUNTER_RESET_MS))
    {
        g_ctrl.fault_count = 0U;
    }

    /* A fault raised inside an interrupt is handled here, once. */
    if ((g_ctrl.isr_fault != 0U) &&
        (g_ctrl.state != CTRL_FAULT) && (g_ctrl.state != CTRL_LATCHED))
    {
        EnterFault(g_ctrl.isr_fault, now_ms);
        return;
    }

    switch (g_ctrl.state)
    {
    case CTRL_OFF:
#if (SLAVE_LEGACY_LOWSIDES != 0)
        if (s_lows_armed != 0U) { SlavePower_InverterDisable(); s_lows_armed = 0U; }     /* stopped: the lows go off */
#endif
        if (g_ctrl.run_request != 0U)
        {
            Resync();                              /* every start begins with a fresh sync */
            g_ctrl.state     = CTRL_SYNC;
            s_state_since_ms = now_ms;
        }
        break;

    case CTRL_SYNC:
        if (g_ctrl.run_request == 0U) { g_ctrl.state = CTRL_OFF; break; }

        {
            /* Work out WHICH conditions are not met, so that the master can show it. */
            uint8_t w = CTRL_CTX_VALID;
            if (!((g_ctrl.mode != CTRL_ZC_UNKNOWN) && (SelectedGood() >= CTRL_SYNC_GOOD_EVENTS) &&
                  ((int32_t)(now_us - g_ctrl.t_event_us) < (int32_t)CTRL_SYNC_FRESH_US)))      w |= CTRL_WAIT_ZC;
            if (m.valid == 0U)                                                                   w |= CTRL_WAIT_MEAS;
#if (SLAVE_TEST_NO_VOLTAGE_CUTS == 0)
            if ((s_vin_bad != 0U) || (m.vin < (CTRL_VIN_MIN_V + CTRL_VIN_START_HYST_V)) ||
                (m.vin > (CTRL_VIN_MAX_V - CTRL_VIN_START_HYST_V)))                              w |= CTRL_WAIT_VIN;   /* start window is NARROWER than the trip window */
#endif
            if (s_vout_bad != 0U)                                                                w |= CTRL_WAIT_VOUT;
            if (SlavePower_FltAsserted() != 0U)                                                  w |= CTRL_WAIT_FLT;
            if (SlavePower_BreakInputActive() != 0U)                                             w |= CTRL_WAIT_BKIN;
            if (SlaveRelay_IsShorted() == 0U)                                                    w |= CTRL_WAIT_RELAY;
            g_ctrl.wait_ctx = w;
            /* everything EXCEPT the relays is ready: start (legacy mode) / keep the relay sequence going */
            if ((w & (uint8_t)~(CTRL_CTX_VALID | CTRL_WAIT_RELAY)) == 0U)
            {
                SlaveRelay_Hold();
#if (SLAVE_LEGACY_LOWSIDES != 0)
                /* the old firmware: inverter outputs ON at zero duty (low sides on = the primary shorted by the IGBTs) before and through the
                 * relay sequence and the converter's first diagonal */
                if (s_lows_armed == 0U) { if (SlavePower_InverterEnable() != 0U) s_lows_armed = 1U; }
#endif
            }
        }

        if ((g_ctrl.wait_ctx & ~CTRL_CTX_VALID) == 0U)
        {
            /* The relays are SHORTED (bypass) and settled: the converter may start. */
            uint32_t p = CtrlHw_Lock();
            g_ctrl.isr_fault   = 0U;
            g_ctrl.break_flags = 0U;
            g_ctrl.pending_pol = 0;
            g_ctrl.duty_counts = 0U;
            g_ctrl.duty_ok     = 0U;
            g_ctrl.boost       = (uint8_t)(m.vin < s_target_v);
            g_ctrl.first_pending = 1U;
#if (SLAVE_LEGACY_LOWSIDES != 0)
            s_lows_base_comm = g_ctrl.commutations;
#endif
            g_ctrl.run_allowed = 1U;
            CtrlHw_ArmWatchdog((uint32_t)(CtrlHw_NowUs() + CTRL_WATCHDOG_US));
            CtrlHw_Unlock(p);

            s_rect_base_comm = g_ctrl.commutations;
            s_rect_ref_set   = 0U;
            g_ctrl.state     = CTRL_RECT;
            s_state_since_ms = now_ms;
        }
        break;

    case CTRL_RECT:
        if (g_ctrl.run_request == 0U) { StopOutputs(); g_ctrl.state = CTRL_OFF; break; }   /* shunts closed, no inverter: stop at once */
        f = RunningChecks(now_ms, now_us, 1U);                                              /* relays must be SHORTED */
        if (f != 0U) { EnterFault(f, now_ms); break; }

        g_ctrl.boost = (uint8_t)(m.vin < s_target_v);

        if ((s_rect_ref_set == 0U) && (g_ctrl.commutations >= (s_rect_base_comm + 3U)))
        {
            s_rect_ref_ms  = now_ms;                /* the converter has really been running */
            s_rect_ref_set = 1U;
        }
#if (SLAVE_TF_CHECK != 0)
        TfCheckStep(&m, now_ms);
#endif
        if ((s_rect_ref_set == 0U) && ((uint32_t)(now_ms - s_state_since_ms) > 500U))
        {
            EnterFault(CF_NO_COMMUTATION, now_ms);  /* events arrive but nothing commutates */
            break;
        }

#if (SLAVE_POWER_STAGE >= 2)
        if ((s_rect_ref_set != 0U) && ((uint32_t)(now_ms - s_rect_ref_ms) >= CTRL_RECT_TO_INV_MS))
        {
            /* Rule R4: the inverter is enabled while the shunts are STILL CLOSED; zero duty = low sides on. */
            if ((SlavePower_InverterIsEnabled() == 0U) && (SlavePower_InverterEnable() == 0U)) { EnterFault(CF_INV_ENABLE, now_ms); break; }   /* legacy: already on at zero duty */
            g_ctrl.duty_counts = 0U;
            g_ctrl.duty_ok     = 0U;
            g_ctrl.inv_active  = 1U;
            g_ctrl.state       = CTRL_INV_START;
            s_state_since_ms   = now_ms;
        }
#endif
        break;

    case CTRL_INV_START:
        if (g_ctrl.run_request == 0U) { StopOutputs(); g_ctrl.state = CTRL_OFF; break; }   /* shunts closed, duty zero: stop at once */
        f = RunningChecks(now_ms, now_us, 1U);
        if (f != 0U) { EnterFault(f, now_ms); break; }

        g_ctrl.duty_counts = 0U;
        g_ctrl.boost       = (uint8_t)(m.vin < s_target_v);

        if ((uint32_t)(now_ms - s_state_since_ms) >= CTRL_INV_ZERO_DUTY_MS)    /* bootstrap supplies charged */
        {
            g_ctrl.state     = CTRL_UNSHUNT;                                    /* the relay request goes out at the next pass */
            s_state_since_ms = now_ms;
        }
        break;

    case CTRL_UNSHUNT:
        if (g_ctrl.run_request == 0U) { BeginReshunt(0U, now_ms); break; }
        f = RunningChecks(now_ms, now_us, 0U);                                  /* relays are in transition: no expectation */
        if (f != 0U) { EnterFault(f, now_ms); break; }

        g_ctrl.duty_counts = 0U;
        g_ctrl.boost       = (uint8_t)(m.vin < s_target_v);

        if (SlaveRelay_IsFree() != 0U)                                          /* shunts open and settled */
        {
            uint32_t p = CtrlHw_Lock();
            Reg_Init(&s_reg, s_target_v, m.vin, m.vout, now_ms);
#if (SLAVE_INJ_GUARD != 0) && (SLAVE_POWER_STAGE != 2)
            s_guard_on = 0U; s_dir_applied = (uint8_t)(m.vin < s_target_v); s_vin_ref = m.vin; s_vin_cnt = m.update_count;
#endif
#if (SLAVE_POWER_STAGE >= 3)
            s_reg.out_max = (float)CTRL_STAGE3_MAX_EFF_TICKS;              /* the cap: the integrator IS the output, so this is also the anti-wind-up */
#endif
            s_reg.ss_slow_slope_vps  = CTRL_SOFTSTART_SLOW_SLOPE_VPS;       /* the soft-start ramp: the old firmware's values unless the config says otherwise */
            s_reg.ss_fast_slope_vps  = CTRL_SOFTSTART_FAST_SLOPE_VPS;
            s_reg.ss_final_slope_vps = CTRL_SOFTSTART_FINAL_SLOPE_VPS;
            s_reg.ss_slow_time_ms    = (uint32_t)CTRL_SOFTSTART_SLOW_TIME_MS;
#if (SLAVE_POWER_STAGE == 2)
            InjReset(now_ms);
#endif
            g_ctrl.duty_ok   = 1U;                                              /* only now may duty be nonzero (rule R1) */
            g_ctrl.state     = CTRL_RUN;
            s_state_since_ms = now_ms;
            CtrlHw_Unlock(p);
        }
        break;

    case CTRL_RUN:
        if (g_ctrl.run_request == 0U) { BeginReshunt(0U, now_ms); break; }
        f = RunningChecks(now_ms, now_us, 2U);                                  /* relays must be FREE */
        if (f != 0U) { EnterFault(f, now_ms); break; }

#if (SLAVE_POWER_STAGE == 2)
        g_ctrl.duty_counts = InjStep(&m, now_ms);                                /* zero / ramp / hold / ramp down / zero: see CTRL_INJ_* */
        g_ctrl.boost       = ((s_inj_phase >= (uint8_t)INJ_RAMP) && (s_inj_phase <= (uint8_t)INJ_DOWN)) ? s_inj_boost : (uint8_t)(m.vin < s_target_v);
#else
        {
            float d;
            Reg_Service(&s_reg, now_ms, m.vin, m.vout, 1U);
            d = Reg_Duty(&s_reg);                                   /* the EFFECTIVE high-side on-time in ticks (the integral controller's output) */
            if (d > (float)CTRL_STAGE3_MAX_EFF_TICKS) d = (float)CTRL_STAGE3_MAX_EFF_TICKS;
            if (d < 0.0f) d = 0.0f;
#if (SLAVE_INJ_GUARD != 0)
            {   /* the guard: see slave_features.h. A step of the input, or a wish to reverse the direction at a large correction, takes the correction to ZERO; it stays zero until the input has been steady,
                 * then the direction is reversed (if wanted) and the regulator restarts from zero through its soft-start ramp. */
                const uint8_t want = Reg_IsBoost(&s_reg, m.vin);
                uint8_t trigger = 0U;
                if (m.update_count != s_vin_cnt)
                {
                    float dv = m.vin - s_vin_ref; if (dv < 0.0f) dv = -dv;
                    s_vin_cnt = m.update_count; s_vin_ref = m.vin;
                    if (dv >= CTRL_VIN_STEP_V) trigger = 1U;                              /* whatever the correction is now: the output measurement is stale for a cycle or two after a step, and the regulator would wind up on it */
                }
                /* a wish to reverse at a large correction starts a hold, but must NOT re-arm one that is already running: during the hold the regulator keeps integrating the stale error and its output d winds up to the cap,
                 * so this test would see "large d and a direction mismatch" on every pass and the guard would never release (the output then stays at the input: found in simulation on a 200 -> 250 V step). */
                if ((s_guard_on == 0U) && (want != s_dir_applied) && (d > (float)CTRL_DIR_FLIP_MAX_TICKS)) trigger = 1U;
                if (trigger != 0U)
                {
                    if (s_guard_on == 0U) s_guard_events++;
                    s_guard_on = 1U; s_guard_until_ms = now_ms + CTRL_GUARD_HOLD_MS;           /* (re)start the hold */
                }
                if (s_guard_on != 0U)
                {
                    d = 0.0f;
                    if ((int32_t)(now_ms - s_guard_until_ms) >= 0)
                    {   /* steady again: restart the regulator from zero and take the direction it now wants */
                        s_guard_on = 0U;
                        Reg_Init(&s_reg, s_target_v, m.vin, m.vout, now_ms);
#if (SLAVE_POWER_STAGE >= 3)
                        s_reg.out_max = (float)CTRL_STAGE3_MAX_EFF_TICKS;              /* the SAME cap as at the start: Reg_Init resets it to a full period, and the integrator IS the output */
#endif
                        s_reg.ss_slow_slope_vps  = CTRL_GUARD_RESTART_SLOPE_VPS;       /* a fast ramp, no slow first phase: the output is already live */
                        s_reg.ss_fast_slope_vps  = CTRL_GUARD_RESTART_SLOPE_VPS;
                        s_reg.ss_final_slope_vps = CTRL_GUARD_RESTART_FINAL_VPS;
                        s_reg.ss_slow_time_ms    = 0U;
                        s_dir_applied = Reg_IsBoost(&s_reg, m.vin);
                    }
                }
                else s_dir_applied = want;                                                     /* a small correction: the direction may follow at once */
                g_ctrl.boost = s_dir_applied;
            }
#else
            g_ctrl.boost       = Reg_IsBoost(&s_reg, m.vin);
#endif
            g_ctrl.duty_counts = CTRL_EFF_TO_CCR((uint32_t)d);      /* + the dead time: a pulse at or below it outputs nothing */
#if (SLAVE_POWER_STAGE >= 3)
            s_reg_eff          = (uint16_t)d;                        /* for the display (diag2) */
            s_reg_boost        = g_ctrl.boost;
#endif
        }
#endif
        break;

    case CTRL_RESHUNT:
        /* duty is zero, the shunt coils are commanded closed; the inverter stays ENABLED (low sides on) until they have. */
        if ((SlaveRelay_UnwindDone() != 0U) || ((uint32_t)(now_ms - s_state_since_ms) > CTRL_RESHUNT_TIMEOUT_MS))
        {
            if (s_resh_fault != 0U)
            {
                FinishFault(s_resh_fault, now_ms);
            }
            else
            {
                StopOutputs();
                g_ctrl.state = CTRL_OFF;
            }
        }
        break;

    case CTRL_FAULT:
        if (((int32_t)(now_ms - s_restart_at_ms) >= 0) &&
            (SlavePower_FltAsserted() == 0U) && (SlavePower_BreakInputActive() == 0U))
        {
            Resync();                              /* mandatory fresh synchronisation */
            g_ctrl.state     = CTRL_OFF;           /* OFF -> SYNC on the next pass if still requested */
            s_state_since_ms = now_ms;
        }
        break;

    case CTRL_LATCHED:
    default:
        break;
    }
}
