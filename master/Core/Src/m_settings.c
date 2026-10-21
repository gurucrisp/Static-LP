/** @file m_settings.c  @brief Set-Mode settings: defaults, validation, pack. See m_settings.h. */
#include "m_settings.h"



void MS_Defaults(MSettings_t *s)
{
    s->mode = (uint8_t)M_MODE_SINGLE; s->set_v = 230U; s->maxcorr_v = 0U;
    s->ip_lv = 180U; s->ip_hv = 280U; s->op_lv = 200U; s->op_hv = 250U;
    s->volt_cut_delay = 5U; s->ovl_current = MASTER_RATED_CURRENT_A; s->ovl_cut_delay = 10U; s->heat_delay = 10U;
    s->phase_seq = 0U; s->scroll = 1U; s->scroll_time = 5U; s->password = 0U; s->restart_auto = 1U; s->restart_time = 5U;
}

MSErr_t MS_Validate(const MSettings_t *s, const MLimits_t *lim)
{
    if (s->mode > (uint8_t)M_MODE_DOMESTIC)                         return MS_BAD_MODE;
    if ((s->set_v < 100U) || (s->set_v > 300U))                      return MS_BAD_SET_V;
    if (s->maxcorr_v > 199U)                                         return MS_BAD_MAXCORR;
    if (s->ip_lv >= s->set_v)                                        return MS_BAD_IP_LV;
    if (s->ip_hv <= s->set_v)                                        return MS_BAD_IP_HV;
    if (s->op_lv >= s->set_v)                                        return MS_BAD_OP_LV;
    if (s->op_hv <= s->set_v)                                        return MS_BAD_OP_HV;
    if (lim != 0)
    {
        if (s->ip_lv < lim->win_lo_v)                                return MS_BAD_IP_LV;      /* below the slave's window it would stop first, with no master fault (D7) */
        if (s->ip_hv > lim->win_hi_v)                                return MS_BAD_IP_HV;
        if (s->op_hv > lim->out_guard_v)                             return MS_BAD_OP_HV;      /* above the slave's own output guard (D26) */
    }
    if ((s->volt_cut_delay < 1U) || (s->volt_cut_delay > 99U))       return MS_BAD_VOLT_DELAY;
    if ((s->ovl_current < 1U) || (s->ovl_current > 180U))            return MS_BAD_OVL_CURRENT;
    if ((s->ovl_cut_delay < 1U) || (s->ovl_cut_delay > 99U))         return MS_BAD_OVL_DELAY;
    if ((s->heat_delay < 1U) || (s->heat_delay > 99U))               return MS_BAD_HEAT_DELAY;
    if ((s->scroll_time < 1U) || (s->scroll_time > 99U))             return MS_BAD_SCROLL_TIME;
    if (s->password > 999U)                                          return MS_BAD_PASSWORD;
    if ((s->restart_time < 1U) || (s->restart_time > 999U))          return MS_BAD_RESTART_TIME;
    if ((s->phase_seq > 1U) || (s->scroll > 1U) || (s->restart_auto > 1U)) return MS_BAD_FLAG;
    return MS_OK;
}

static void w16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static uint16_t r16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

void MS_Pack(const MSettings_t *s, uint8_t o[MS_PACK_LEN])
{
    uint32_t i; for (i = 0U; i < MS_PACK_LEN; i++) o[i] = 0U;
    o[0] = s->mode; w16(&o[1], s->set_v); w16(&o[3], s->maxcorr_v);
    w16(&o[5], s->ip_lv); w16(&o[7], s->ip_hv); w16(&o[9], s->op_lv); w16(&o[11], s->op_hv);
    o[13] = s->volt_cut_delay; w16(&o[14], s->ovl_current); o[16] = s->ovl_cut_delay; o[17] = s->heat_delay;
    o[18] = s->phase_seq; o[19] = s->scroll; o[20] = s->scroll_time; w16(&o[21], s->password); o[23] = s->restart_auto; w16(&o[24], s->restart_time);
}

void MS_Unpack(MSettings_t *s, const uint8_t o[MS_PACK_LEN])
{
    s->mode = o[0]; s->set_v = r16(&o[1]); s->maxcorr_v = r16(&o[3]);
    s->ip_lv = r16(&o[5]); s->ip_hv = r16(&o[7]); s->op_lv = r16(&o[9]); s->op_hv = r16(&o[11]);
    s->volt_cut_delay = o[13]; s->ovl_current = r16(&o[14]); s->ovl_cut_delay = o[16]; s->heat_delay = o[17];
    s->phase_seq = o[18]; s->scroll = o[19]; s->scroll_time = o[20]; s->password = r16(&o[21]); s->restart_auto = o[23]; s->restart_time = r16(&o[24]);
}

uint8_t MS_Load(MStore_t *st, MSettings_t *s)
{
    uint8_t b[MS_PACK_LEN];
    if (st->payload_len != MS_PACK_LEN) st->payload_len = (uint8_t)MS_PACK_LEN;
    if (MStore_Load(st, b) != 0U)
    {
        MSettings_t t; MS_Unpack(&t, b);
        if (MS_Validate(&t, 0) == MS_OK) { *s = t; return 1U; }     /* a record that passes its CRC but not the rules is not trusted either */
    }
    MS_Defaults(s);
    return 0U;
}

int MS_Save(MStore_t *st, const MSettings_t *s)
{
    uint8_t b[MS_PACK_LEN];
    if (MS_Validate(s, 0) != MS_OK) return -1;                      /* never write what Load would refuse */
    MS_Pack(s, b);
    return MStore_Save(st, b);
}
