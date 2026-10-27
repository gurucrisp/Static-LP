/** @file m_ctl.c  @brief What the master tells the slave. See m_ctl.h. */
#include "m_ctl.h"

void MCtl_Init(MCtl_t *c)
{
    uint8_t *p = (uint8_t *)c; uint32_t i;
    for (i = 0U; i < (uint32_t)sizeof(*c); i++) p[i] = 0U;
    c->first = 1U;
}

void MCtl_Step(MCtl_t *c, const MSettings_t *s, uint8_t setmode, uint8_t key_start, uint8_t key_stop, uint8_t link_ok, uint8_t slave_fault, uint32_t now, MCtlOut_t *out)
{
    const uint32_t rdelay = (uint32_t)s->restart_time * 1000UL;
    const uint8_t  autorst = (uint8_t)(s->restart_auto != 0U);

    if (setmode != 0U)
    {   /* Set-Mode: the output is off while it is open (D10): run = 0 at once */
        c->user_on = 0U; c->waiting = 0U; c->fault_seen = 0U; c->setmode_prev = 1U; c->first = 0U;
    }
    else
    {
        if ((c->setmode_prev != 0U) || (c->first != 0U))
        {   /* just left Set-Mode, or power-up: Auto waits the Restart Time, Manual waits for START */
            c->setmode_prev = 0U; c->first = 0U;
            if (autorst != 0U) { c->waiting = 1U; c->t_ref = now; }
        }
        if (key_stop != 0U) { c->user_on = 0U; c->waiting = 0U; }
        if ((key_start != 0U) && (c->user_on == 0U))
        {
            c->user_on = 1U; c->waiting = 0U; c->fault_seen = 0U;       /* a start begins a fresh fault episode */
            if (slave_fault != 0U) { c->clr++; c->t_clr = now; }       /* START after a fault: the slave is latched, the counter must change */
        }
        if ((c->waiting != 0U) && ((uint32_t)(now - c->t_ref) >= rdelay))
        {
            c->waiting = 0U; c->user_on = 1U; c->fault_seen = 0U;
            if (slave_fault != 0U) { c->clr++; c->t_clr = now; }
        }
        if ((c->user_on != 0U) && (link_ok != 0U) && (slave_fault != 0U) && (((uint32_t)(now - c->t_clr) >= MCTL_SETTLE_MS) || (c->t_clr == 0U)))
        {
            if (c->fault_seen == 0U)
            {
                c->fault_seen = 1U; c->t_fault = now;
                if (autorst == 0U) c->user_on = 0U;                    /* Manual: stop at the fault, START restarts */
            }
            else if ((autorst != 0U) && ((uint32_t)(now - c->t_fault) >= rdelay)) { c->clr++; c->t_clr = now; c->t_fault = now; }     /* Auto: clear it and let the slave try again */
        }
        else if (slave_fault == 0U) c->fault_seen = 0U;
    }
    out->run     = (uint8_t)((c->user_on != 0U) && (setmode == 0U));
    out->clr     = c->clr;
    out->setv_dv = (uint16_t)(s->set_v * 10U);
    out->maxc_dv = (uint16_t)(s->maxcorr_v * 10U);
    out->hold    = 0U;
}
