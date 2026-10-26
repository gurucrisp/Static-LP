/** @file m_faults.c  @brief The master's fault and restart engine. See m_faults.h. */
#include "m_faults.h"

#define MF_CLR_SETTLE_MS   1000UL     /* after a clear pulse the slave's old fault code is ignored this long: it needs a moment to answer the new counter */

uint8_t MF_Rank(uint8_t type)
{
    switch (type)
    {
    case MLOG_T_FLIP:        return 100U;
    case MLOG_T_INRUSH:      return 90U;
    case MLOG_T_POWER_STAGE: return 80U;
    case MLOG_T_RETUNE:      return 75U;
    case MLOG_T_HEAT:        return 70U;
    case MLOG_T_OVERLOAD:    return 60U;
    case MLOG_T_IP_LV: case MLOG_T_IP_HV: return 50U;
    case MLOG_T_PHASE_SEQ: case MLOG_T_MAINS: return 45U;
    case MLOG_T_LINK_LOST:   return 40U;
    case MLOG_T_OP_LV: case MLOG_T_OP_HV: return 30U;
    default:                 return 0U;
    }
}

static uint32_t age(uint32_t now, uint32_t t) { return (uint32_t)(now - t); }

static uint8_t n_units(uint8_t mode) { return (mode == (uint8_t)M_MODE_DOMESTIC) ? 3U : 1U; }
static uint8_t pmask(const MFaults_t *f, uint8_t u)
{
    if (f->mode == (uint8_t)M_MODE_DOMESTIC)   return (uint8_t)(1U << u);
    if (f->mode == (uint8_t)M_MODE_INDUSTRIAL) return 7U;
    return 1U;                                                          /* Single: the R phase only */
}

void MF_Init(MFaults_t *f, const MSettings_t *s, uint32_t now_ms)
{
    uint8_t *p = (uint8_t *)f; uint32_t i; uint8_t u;
    for (i = 0U; i < (uint32_t)sizeof(*f); i++) p[i] = 0U;
    f->mode = s->mode; f->nunits = n_units(s->mode);
    for (u = 0U; u < f->nunits; u++)
    {
        f->u[u].st = (s->restart_auto != 0U) ? (uint8_t)U_ONDELAY : (uint8_t)U_OFF;      /* power-up: Auto waits the Restart Time (the on-delay), Manual waits for START */
        f->u[u].t_enter = now_ms;
    }
    f->max_retries = (uint8_t)MF_RETRIES;
    f->inited = 1U;
}

void MF_SetMaxRetries(MFaults_t *f, uint8_t n) { f->max_retries = n; }
uint16_t MF_RestartLeftS(const MFaults_t *f, uint8_t unit) { return (unit < 3U) ? f->u[unit].left_s : 0U; }

void MF_RetuneDone(MFaults_t *f)
{
    uint8_t u;
    for (u = 0U; u < f->nunits; u++)
        if ((f->u[u].st == (uint8_t)U_LATCHED) && (f->u[u].fclass == (uint8_t)FC_RETUNE)) { f->u[u].st = (uint8_t)U_OFF; f->u[u].ftype = 0U; f->u[u].fclass = 0U; }
}

typedef struct { uint8_t type, cls, phase, code; } Cause_t;

static void classify_slave(uint8_t code, uint16_t vin_dv, uint16_t set_v, Cause_t *c);
uint8_t MF_SlaveFaultType(uint8_t code, uint16_t vin_dv, uint16_t set_v)
{
    Cause_t c; if (code == 0U) return 0U;
    classify_slave(code, vin_dv, set_v, &c); return c.type;
}

static void classify_slave(uint8_t code, uint16_t vin_dv, uint16_t set_v, Cause_t *c)
{
    c->code = code;
    switch (code)
    {
    case 1: case 2:                                     c->type = MLOG_T_INRUSH;      c->cls = (uint8_t)FC_RETRY;  break;
    case 7: case 12: case 14:                           c->type = MLOG_T_POWER_STAGE; c->cls = (uint8_t)FC_RETRY;  break;
    case 3: case 4: case 5: case 8: case 10: case 11: case 13: case 15: case 16:
                                                        c->type = MLOG_T_MAINS;       c->cls = (uint8_t)FC_VOLT;   break;
    case 6:                                             c->type = MLOG_T_OP_HV;       c->cls = (uint8_t)FC_VOLT;   break;
    case 9:                                             c->type = (vin_dv < (uint16_t)(set_v * 10U)) ? MLOG_T_IP_LV : MLOG_T_IP_HV; c->cls = (uint8_t)FC_VOLT; break;
    case 17:                                            c->type = MLOG_T_FLIP;        c->cls = (uint8_t)FC_LATCH;  break;
    case 18: case 19:                                   c->type = MLOG_T_RETUNE;      c->cls = (uint8_t)FC_RETUNE; break;
    default:                                            c->type = MLOG_T_POWER_STAGE; c->cls = (uint8_t)FC_RETRY;  break;   /* an unknown code is treated as serious */
    }
}

static void consider(Cause_t *best, uint8_t *have, uint8_t type, uint8_t cls, uint8_t phase, uint8_t code)
{
    if ((*have == 0U) || (MF_Rank(type) > MF_Rank(best->type))) { best->type = type; best->cls = cls; best->phase = phase; best->code = code; *have = 1U; }
}

void MF_Tick(MFaults_t *f, const MSettings_t *s, const MFIn_t *in, uint32_t now, MFOut_t *out)
{
    uint8_t p, u;
    uint32_t vdelay = (uint32_t)s->volt_cut_delay * 1000UL, rdelay = (uint32_t)s->restart_time * 1000UL;
    uint8_t *o = (uint8_t *)out; uint32_t i;
    for (i = 0U; i < (uint32_t)sizeof(*out); i++) o[i] = 0U;
    if (f->inited == 0U) MF_Init(f, s, now);

    /* ---- Set-Mode: the output is off while it is open (D10); on leaving it the units start again as at power-up ---- */
    if ((in->setmode != 0U) && (f->setmode_prev == 0U))
        for (u = 0U; u < f->nunits; u++) if (f->u[u].st != (uint8_t)U_LATCHED) f->u[u].st = (uint8_t)U_OFF;
    if ((in->setmode == 0U) && (f->setmode_prev != 0U))
    {
        if (f->mode != s->mode)
        {   /* the mode was changed: rebuild the units (nothing is energised here) */
            uint8_t keep = f->buzzer; MF_Init(f, s, now); f->buzzer = keep;
        }
        else for (u = 0U; u < f->nunits; u++) if (f->u[u].st == (uint8_t)U_OFF) { f->u[u].st = (s->restart_auto != 0U) ? (uint8_t)U_ONDELAY : (uint8_t)U_OFF; f->u[u].t_enter = now; }
    }
    f->setmode_prev = in->setmode;

    /* ---- the limits, per phase (hysteresis memory and the time each violation began) ---- */
    for (p = 0U; p < 3U; p++)
    {
        const MPhaseIn_t *ph = &in->ph[p];
        if (ph->link_ok == 0U) continue;                                  /* no valid readings: keep what we know; the link fault handles it */
        if (ph->vin_dv < (uint16_t)(s->ip_lv * 10U))                      { if (f->ip_lv[p] == 0U) { f->ip_lv[p] = 1U; f->t_ip_lv[p] = now; } }
        else if (ph->vin_dv >= (uint16_t)((s->ip_lv + MF_HYST_V) * 10U))  f->ip_lv[p] = 0U;
        if (ph->vin_dv > (uint16_t)(s->ip_hv * 10U))                      { if (f->ip_hv[p] == 0U) { f->ip_hv[p] = 1U; f->t_ip_hv[p] = now; } }
        else if (ph->vin_dv <= (uint16_t)((s->ip_hv - MF_HYST_V) * 10U))  f->ip_hv[p] = 0U;
        if (ph->heat != 0U) { if (f->a_heat[p] == 0U) { f->a_heat[p] = 1U; f->t_heat[p] = now; } } else f->a_heat[p] = 0U;
    }

    /* ---- every unit ---- */
    for (u = 0U; u < f->nunits; u++)
    {
        MUnit_t *un = &f->u[u]; uint8_t mask = pmask(f, u);
        Cause_t best = { 0U, 0U, 0U, 0U }; uint8_t have = 0U; uint8_t can_start = 1U;
        const uint8_t live = (uint8_t)(un->st == (uint8_t)U_ON), energised = (uint8_t)((un->st == (uint8_t)U_ON) || (un->st == (uint8_t)U_STARTING));

        /* output limits and overload only count while the output is LIVE (the contactor closed) */
        for (p = 0U; p < 3U; p++)
        {
            const MPhaseIn_t *ph = &in->ph[p];
            if ((mask & (1U << p)) == 0U) continue;
            if ((live == 0U) || (ph->link_ok == 0U)) { f->op_lv[p] = 0U; f->op_hv[p] = 0U; f->a_ovl[p] = 0U; f->hv_fast[p] = 0U; continue; }
            if (ph->vout_dv < (uint16_t)(s->op_lv * 10U))                     { if (f->op_lv[p] == 0U) { f->op_lv[p] = 1U; f->t_op_lv[p] = now; } }
            else if (ph->vout_dv >= (uint16_t)((s->op_lv + MF_HYST_V) * 10U)) f->op_lv[p] = 0U;
            if (ph->vout_dv > (uint16_t)(s->op_hv * 10U))                     { if (f->op_hv[p] == 0U) { f->op_hv[p] = 1U; f->t_op_hv[p] = now; } }
            else if (ph->vout_dv <= (uint16_t)((s->op_hv - MF_HYST_V) * 10U)) f->op_hv[p] = 0U;
            if (ph->vout_dv > (uint16_t)((s->op_hv + MF_OP_HV_FAST_V) * 10U)) { if (f->hv_fast[p] == 0U) { f->hv_fast[p] = 1U; f->t_hv_fast[p] = now; } } else f->hv_fast[p] = 0U;
            if (ph->load_da > (uint16_t)(s->ovl_current * 10U))               { if (f->a_ovl[p] == 0U) { f->a_ovl[p] = 1U; f->t_ovl[p] = now; } } else f->a_ovl[p] = 0U;
        }

        for (p = 0U; p < 3U; p++)
        {
            const MPhaseIn_t *ph = &in->ph[p];
            if ((mask & (1U << p)) == 0U) continue;
            /* what stops a START */
            if ((f->ip_lv[p] != 0U) || (f->ip_hv[p] != 0U) || (f->a_heat[p] != 0U) || (ph->link_ok == 0U)) can_start = 0U;
            if (energised == 0U) continue;
            /* what TRIPS a running unit */
            if (ph->link_ok == 0U) consider(&best, &have, MLOG_T_LINK_LOST, (uint8_t)FC_VOLT, p, 0U);
            if ((ph->slave_fault != 0U) && (age(now, un->t_enter) >= MF_CLR_SETTLE_MS || un->clr_owed == 0U))
            {   Cause_t c; classify_slave(ph->slave_fault, ph->vin_dv, s->set_v, &c); consider(&best, &have, c.type, c.cls, p, c.code); }
            if (ph->ct_peak != 0U) consider(&best, &have, MLOG_T_INRUSH, (uint8_t)FC_RETRY, p, 0U);
            /* the inverter is OFF but the load is still connected: whatever the reason (mains unstable, a slave that stopped without a fault code), the slave leaving RUNNING for MF_INV_OFF_MS opens the contactor */
            if ((live != 0U) && (ph->state_valid != 0U) && (ph->slave_state != 2U))
            {
                if (f->st_bad[p] == 0U) { f->st_bad[p] = 1U; f->t_st_bad[p] = now; }
                else if (age(now, f->t_st_bad[p]) >= MF_INV_OFF_MS) consider(&best, &have, MLOG_T_MAINS, (uint8_t)FC_VOLT, p, 0U);
            }
            else f->st_bad[p] = 0U;
            if ((f->ip_lv[p] != 0U) && (age(now, f->t_ip_lv[p]) >= vdelay)) consider(&best, &have, MLOG_T_IP_LV, (uint8_t)FC_VOLT, p, 0U);
            if ((f->ip_hv[p] != 0U) && (age(now, f->t_ip_hv[p]) >= vdelay)) consider(&best, &have, MLOG_T_IP_HV, (uint8_t)FC_VOLT, p, 0U);
            if ((f->op_lv[p] != 0U) && (age(now, f->t_op_lv[p]) >= vdelay)) consider(&best, &have, MLOG_T_OP_LV, (uint8_t)FC_VOLT, p, 0U);
            if ((f->op_hv[p] != 0U) && (age(now, f->t_op_hv[p]) >= vdelay)) consider(&best, &have, MLOG_T_OP_HV, (uint8_t)FC_VOLT, p, 0U);
            if ((f->hv_fast[p] != 0U) && (age(now, f->t_hv_fast[p]) >= MF_OP_HV_FAST_MS)) consider(&best, &have, MLOG_T_OP_HV, (uint8_t)FC_VOLT, p, 0U);     /* a gross over-voltage: no waiting for the delay */
            if ((f->a_ovl[p] != 0U) && (age(now, f->t_ovl[p]) >= (uint32_t)s->ovl_cut_delay * 1000UL)) consider(&best, &have, MLOG_T_OVERLOAD, (uint8_t)FC_START, p, 0U);
            if ((f->a_heat[p] != 0U) && (age(now, f->t_heat[p]) >= (uint32_t)s->heat_delay * 1000UL)) consider(&best, &have, MLOG_T_HEAT, (uint8_t)FC_HEAT, p, 0U);
        }
        if ((f->mode == (uint8_t)M_MODE_INDUSTRIAL) && (s->phase_seq != 0U) && (in->seq_state != 0U))
        {
            can_start = 0U;
            if (energised != 0U) consider(&best, &have, MLOG_T_PHASE_SEQ, (uint8_t)FC_VOLT, 3U, 0U);
        }
        else if ((f->mode == (uint8_t)M_MODE_INDUSTRIAL) && (s->phase_seq != 0U) && (in->seq_unknown != 0U)) can_start = 0U;       /* not measured yet: no START (it is only ever consulted by the states that start; a running unit is not tripped by it) */

        /* ---- a trip ---- */
        if ((energised != 0U) && (have != 0U))
        {
            uint8_t lp = (best.phase > 2U) ? 0U : best.phase;
            if (out->n_log < 6U)
            {
                MLogRec_t *r = &out->log[out->n_log++].rec;
                r->type = best.type; r->phase = best.phase; r->slave_code = best.code;
                r->vin_dv = in->ph[lp].vin_dv; r->vout_dv = in->ph[lp].vout_dv; r->load_da = in->ph[lp].load_da; r->run_min = in->run_min;
            }
            un->ftype = best.type; un->fclass = best.cls; un->fphase = best.phase; un->fcode = best.code; un->t_enter = now; un->clr_owed = 0U;
            un->st = (uint8_t)U_FAULT;
            if (best.cls == (uint8_t)FC_RETRY)
            {   /* inrush / power stage: 0 retries = latch at once (commissioning); else the retries, then ONE long wait and ONE extra attempt, then the latch */
                if ((f->max_retries == 0U) || (un->final_try != 0U)) un->st = (uint8_t)U_LATCHED;
                else { un->retries++; if (un->retries > f->max_retries) un->long_wait = 1U; }
            }
            if (best.cls == (uint8_t)FC_HEAT)
            {   /* overheat: remember the trip; the third within 30 minutes needs a manual START */
                uint8_t k, c = 0U; un->t_heat_trip[un->heat_n % 3U] = now; un->heat_n = (uint8_t)((un->heat_n + 1U) % 250U);
                for (k = 0U; k < 3U; k++) if ((un->t_heat_trip[k] != 0U) && (age(now, un->t_heat_trip[k]) <= MF_HEAT_WINDOW_MS)) c++;
                if (c >= MF_HEAT_MAX_TRIPS) un->heat_manual = 1U;
            }
            if ((best.cls == (uint8_t)FC_LATCH) || (best.cls == (uint8_t)FC_RETUNE)) un->st = (uint8_t)U_LATCHED;
            if ((f->mode != (uint8_t)M_MODE_DOMESTIC) || (best.type == MLOG_T_FLIP)) f->buzzer = 1U;       /* D28: the flip sounds in Domestic too */
        }
        else
        {   /* ---- the state machine ---- */
            uint8_t attempt = 0U;
            if ((in->key_stop != 0U) && (un->st == (uint8_t)U_LATCHED))
            {
                f->buzzer = 0U;                                   /* STOP silences the buzzer; a latched fault stays latched until power off and on */
            }
            else if (in->key_stop != 0U)
            {
                un->st = (uint8_t)U_OFF; un->ftype = 0U; un->fclass = 0U; un->clr_owed = 0U; f->buzzer = 0U;
            }
            else switch (un->st)
            {
            case U_OFF:
                if ((in->key_start != 0U) && (can_start != 0U) && (in->setmode == 0U)) attempt = 1U;
                break;
            case U_ONDELAY:
                if (can_start == 0U) un->t_enter = now;
                else if (age(now, un->t_enter) >= rdelay) attempt = 1U;
                if (in->key_start != 0U && can_start != 0U) attempt = 1U;
                break;
            case U_STARTING:
            {
                uint8_t all = 1U;
                for (p = 0U; p < 3U; p++) if (((mask & (1U << p)) != 0U) && (in->ph[p].in_band == 0U)) all = 0U;
                if (all == 0U) un->inband_run = 0U;
                else if (un->inband_run == 0U) { un->inband_run = 1U; un->t_inband = now; }
                else if (age(now, un->t_inband) >= MF_CLOSE_HOLD_MS) { un->st = (uint8_t)U_ON; un->t_on = now; }       /* the contactor closes once the output has been at the set voltage for 1 s */
                break;
            }
            case U_ON:
                if ((un->retries > 0U) && (age(now, un->t_on) >= MF_FORGIVE_MS)) { un->retries = 0U; un->long_wait = 0U; un->final_try = 0U; }
                break;
            case U_FAULT:
                if (un->fclass == (uint8_t)FC_VOLT)
                {
                    uint8_t cond = can_start;                                                   /* inputs back inside by the hysteresis, no heat, link ok, sequence ok */
                    if (cond == 0U) un->t_enter = now;
                    if (s->restart_auto != 0U) { if ((cond != 0U) && (age(now, un->t_enter) >= rdelay)) attempt = 1U; }
                    else if ((cond != 0U) && (in->key_start != 0U)) attempt = 1U;
                }
                else if (un->fclass == (uint8_t)FC_START)
                {
                    if ((can_start != 0U) && (in->key_start != 0U)) attempt = 1U;               /* Overload and Heat: START only */
                }
                else if (un->fclass == (uint8_t)FC_RETRY)
                {   /* the retry interval is the Restart Time but never under 30 s; after the retries are used up: 5 minutes, then ONE more attempt. START restarts at once, as a fresh start. */
                    uint32_t wait = (un->long_wait != 0U) ? MF_FINAL_WAIT_MS : ((rdelay > MF_RETRY_MS) ? rdelay : MF_RETRY_MS);
                    if ((in->key_start != 0U) && (can_start != 0U)) { attempt = 1U; un->retries = 0U; un->long_wait = 0U; un->final_try = 0U; }
                    else if (age(now, un->t_enter) >= wait) { attempt = 1U; if (un->long_wait != 0U) { un->long_wait = 0U; un->final_try = 1U; } }
                }
                else if (un->fclass == (uint8_t)FC_HEAT)
                {   /* the pin is high again (can_start) for the Restart Time: restart by itself (Auto, and not after 3 trips in 30 min); START restarts too */
                    uint8_t cool = can_start;
                    if (cool == 0U) un->t_enter = now;
                    if ((in->key_start != 0U) && (cool != 0U)) { uint8_t k; attempt = 1U; un->heat_manual = 0U; un->heat_n = 0U; for (k = 0U; k < 3U; k++) un->t_heat_trip[k] = 0U; }
                    else if ((s->restart_auto != 0U) && (un->heat_manual == 0U) && (cool != 0U) && (age(now, un->t_enter) >= rdelay)) attempt = 1U;
                }
                break;
            case U_LATCHED:
                /* an inrush / power-stage latch after the long wait and the extra attempt: START tries again (not in commissioning mode, 0 retries: that latch stays until power off and on) */
                if ((un->fclass == (uint8_t)FC_RETRY) && (f->max_retries > 0U) && (in->key_start != 0U) && (can_start != 0U)) { attempt = 1U; un->retries = 0U; un->long_wait = 0U; un->final_try = 0U; }
                break;
            default:
                break;
            }
            un->left_s = 0U;                                                  /* for the screen: how long until the next automatic restart attempt */
            if (un->st == (uint8_t)U_FAULT)
            {
                uint32_t wait = 0U, a = age(now, un->t_enter);
                if (un->fclass == (uint8_t)FC_RETRY) wait = (un->long_wait != 0U) ? MF_FINAL_WAIT_MS : ((rdelay > MF_RETRY_MS) ? rdelay : MF_RETRY_MS);
                else if ((un->fclass == (uint8_t)FC_HEAT) && (s->restart_auto != 0U) && (un->heat_manual == 0U) && (can_start != 0U)) wait = rdelay;
                if ((wait != 0U) && (a < wait)) un->left_s = (uint16_t)(((wait - a) + 999UL) / 1000UL);
            }
            if ((attempt != 0U) && (in->setmode == 0U))
            {
                un->st = (uint8_t)U_STARTING; un->t_enter = now; un->inband_run = 0U; un->clr_owed = 1U;
                for (p = 0U; p < 3U; p++) if ((mask & (1U << p)) != 0U) out->clr_pulse[p] = 1U;  /* the slaves latched their faults: the counter must change */
            }
        }
        if ((un->st == (uint8_t)U_STARTING) && (age(now, un->t_enter) >= MF_CLR_SETTLE_MS)) un->clr_owed = 0U;
        out->unit_state[u] = un->st;

        /* ---- outputs of this unit ---- */
        for (p = 0U; p < 3U; p++)
        {
            if ((mask & (1U << p)) == 0U) continue;
            out->run[p] = (uint8_t)((un->st == (uint8_t)U_STARTING) || (un->st == (uint8_t)U_ON));
            if ((un->st == (uint8_t)U_FAULT) || (un->st == (uint8_t)U_LATCHED))
            {
                if ((un->fphase == 3U) || (un->fphase == p)) out->fault[p] = un->ftype;
            }
        }
        if (un->st == (uint8_t)U_ON)
        {   if (f->mode == (uint8_t)M_MODE_DOMESTIC) out->contactor[u] = 1U; else out->contactor[0] = 1U; }
    }

    /* ---- what is violated right now, for the "Appear" statuses ---- */
    for (p = 0U; p < 3U; p++)
    {
        uint8_t t = 0U;
        if (f->ip_lv[p] != 0U) t = MLOG_T_IP_LV;
        if ((f->ip_hv[p] != 0U) && (MF_Rank(MLOG_T_IP_HV) >= MF_Rank(t))) t = MLOG_T_IP_HV;
        if ((f->op_lv[p] != 0U) && (MF_Rank(MLOG_T_OP_LV) > MF_Rank(t))) t = MLOG_T_OP_LV;
        if ((f->op_hv[p] != 0U) && (MF_Rank(MLOG_T_OP_HV) > MF_Rank(t))) t = MLOG_T_OP_HV;
        if ((f->a_ovl[p] != 0U) && (MF_Rank(MLOG_T_OVERLOAD) > MF_Rank(t))) t = MLOG_T_OVERLOAD;
        if ((f->a_heat[p] != 0U) && (MF_Rank(MLOG_T_HEAT) > MF_Rank(t))) t = MLOG_T_HEAT;
        out->appear[p] = t;
    }
    { uint8_t best = 0U, bp = 0U, r;
      for (p = 0U; p < 3U; p++) { r = MF_Rank(out->fault[p]); if (r > MF_Rank(best)) { best = out->fault[p]; bp = p; } }
      if (best == 0U) for (p = 0U; p < 3U; p++) { r = MF_Rank(out->appear[p]); if (r > MF_Rank(best)) { best = out->appear[p]; bp = p; } }
      out->shown = best; out->shown_phase = bp; }
    {   uint8_t anyf = 0U;                                                   /* the buzzer stops by itself when no unit is faulted any more (an Auto restart) */
        for (u = 0U; u < f->nunits; u++) if ((f->u[u].st == (uint8_t)U_FAULT) || (f->u[u].st == (uint8_t)U_LATCHED)) anyf = 1U;
        if (anyf == 0U) f->buzzer = 0U;
    }
    out->buzzer = f->buzzer;
}
