/** @file m_settings.c  @brief Set-Mode settings: defaults, validation, pack. See m_settings.h. */
#include "m_settings.h"



uint16_t MS_WinLo(const MSettings_t *s)
{
    uint16_t lo = 0U;
    if ((s->maxcorr_v != 0U) && (s->set_v > s->maxcorr_v)) lo = (uint16_t)(s->set_v - s->maxcorr_v);
    return (lo < MS_VIN_MIN_V) ? (uint16_t)MS_VIN_MIN_V : lo;
}
uint16_t MS_WinHi(const MSettings_t *s)
{
    uint16_t hi = (s->maxcorr_v == 0U) ? (uint16_t)MS_VIN_MAX_V : (uint16_t)(s->set_v + s->maxcorr_v);
    return (hi > MS_VIN_MAX_V) ? (uint16_t)MS_VIN_MAX_V : hi;
}
void MS_DefaultLimits(const MSettings_t *s, const MLimits_t *lim, uint16_t *ip_lv, uint16_t *ip_hv, uint16_t *op_lv, uint16_t *op_hv)
{
    uint16_t ilv = MS_WinLo(s), ihv = MS_WinHi(s);
    if (lim != 0) { if (ilv < lim->win_lo_v) ilv = lim->win_lo_v; if (ihv > lim->win_hi_v) ihv = lim->win_hi_v; }
    uint16_t olv = (uint16_t)(ilv + MS_OUT_OFFSET_V), ohv = (ihv > MS_OUT_OFFSET_V) ? (uint16_t)(ihv - MS_OUT_OFFSET_V) : 0U;
    if (olv > (uint16_t)(s->set_v - MS_LIM_MARGIN_V)) olv = (uint16_t)(s->set_v - MS_LIM_MARGIN_V);       /* never closer than the margin to the Set Voltage */
    if ((lim != 0) && (ohv > lim->out_guard_v)) ohv = lim->out_guard_v;                                  /* the slave would trip there first */
    if (ohv < (uint16_t)(s->set_v + MS_LIM_MARGIN_V)) ohv = (uint16_t)(s->set_v + MS_LIM_MARGIN_V);   /* but never closer than the margin to the Set Voltage (Validate then names the guard) */
    *ip_lv = ilv; *ip_hv = ihv; *op_lv = olv; *op_hv = ohv;
}
static uint16_t clip16(uint16_t v, uint16_t lo, uint16_t hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }
void MS_ApplyLim(MSettings_t *s, const MLimits_t *lim)
{
    if (lim == 0) return;
    if (s->ip_lv < lim->win_lo_v) s->ip_lv = lim->win_lo_v;
    if (s->ip_hv > lim->win_hi_v) s->ip_hv = lim->win_hi_v;
    if (s->op_lv < s->ip_lv) s->op_lv = s->ip_lv;
    if (s->op_hv > s->ip_hv) s->op_hv = s->ip_hv;
    if (s->op_hv > lim->out_guard_v) s->op_hv = lim->out_guard_v;
}
void MS_Rederive(MSettings_t *s, const MSettings_t *old, const MLimits_t *lim)
{
    uint16_t o_ilv, o_ihv, o_olv, o_ohv, n_ilv, n_ihv, n_olv, n_ohv;
    MS_DefaultLimits(old, lim, &o_ilv, &o_ihv, &o_olv, &o_ohv);
    MS_DefaultLimits(s, lim, &n_ilv, &n_ihv, &n_olv, &n_ohv);
    s->ip_lv = (old->ip_lv == o_ilv) ? n_ilv : clip16(old->ip_lv, MS_WinLo(s), (uint16_t)(s->set_v - MS_LIM_MARGIN_V));
    s->ip_hv = (old->ip_hv == o_ihv) ? n_ihv : clip16(old->ip_hv, (uint16_t)(s->set_v + MS_LIM_MARGIN_V), MS_WinHi(s));
    s->op_lv = (old->op_lv == o_olv) ? n_olv : clip16(old->op_lv, s->ip_lv, (uint16_t)(s->set_v - MS_LIM_MARGIN_V));
    s->op_hv = (old->op_hv == o_ohv) ? n_ohv : clip16(old->op_hv, (uint16_t)(s->set_v + MS_LIM_MARGIN_V), s->ip_hv);
    if (s->op_lv < s->ip_lv) s->op_lv = s->ip_lv;          /* an output limit never lies outside its input limit, even when only the input one was moved by the user */
    if (s->op_hv > s->ip_hv) s->op_hv = s->ip_hv;
}

uint8_t MS_HeatResetDefault(uint8_t heat_temp)
{
    return (heat_temp >= (uint8_t)(MS_HEAT_RESET_MIN + 10U)) ? (uint8_t)(heat_temp - 10U) : (uint8_t)MS_HEAT_RESET_MIN;
}

void MS_HeatResetFollow(MSettings_t *s)
{
    if ((s->heat_temp >= MS_HEAT_TEMP_MIN) && (s->heat_temp <= MS_HEAT_TEMP_MAX) && ((uint32_t)s->heat_reset + MS_HEAT_RESET_GAP > (uint32_t)s->heat_temp)) s->heat_reset = MS_HeatResetDefault(s->heat_temp);
}

void MS_Defaults(MSettings_t *s)
{
    s->mode = (uint8_t)M_MODE_SINGLE; s->set_v = 230U; s->maxcorr_v = 0U;
    MS_DefaultLimits(s, 0, &s->ip_lv, &s->ip_hv, &s->op_lv, &s->op_hv);
    s->volt_cut_delay = 5U; s->ovl_current = MASTER_RATED_CURRENT_A; s->ovl_cut_delay = 10U; s->heat_delay = 10U;
    s->phase_seq = 0U; s->scroll = 1U; s->scroll_time = 5U; s->password = 0U; s->restart_auto = 1U; s->restart_time = 5U;
    s->load_src = 0U; s->sl_zero_ca = 0U; s->sl_gain = 0U; s->heat_temp = MS_HEAT_TEMP_DEFAULT; s->heat_reset = MS_HeatResetDefault(MS_HEAT_TEMP_DEFAULT);
}

MSErr_t MS_Validate(const MSettings_t *s, const MLimits_t *lim)
{
    if (s->mode > (uint8_t)M_MODE_DOMESTIC)                         return MS_BAD_MODE;
    if ((s->set_v < (MS_VIN_MIN_V + MS_LIM_MARGIN_V)) || (s->set_v > (MS_VIN_MAX_V - MS_LIM_MARGIN_V))) return MS_BAD_SET_V;      /* 100..290: the limits must fit 10 V either side */
    if ((s->maxcorr_v > 199U) || ((s->maxcorr_v != 0U) && (s->maxcorr_v < MS_LIM_MARGIN_V))) return MS_BAD_MAXCORR;      /* 0 = not set, else at least the 10 V margin */
    if ((s->ip_lv < MS_WinLo(s)) || ((uint16_t)(s->ip_lv + MS_LIM_MARGIN_V) > s->set_v))                      return MS_BAD_IP_LV;   /* inside the window, 10 V below Set at most */
    if ((s->ip_hv > MS_WinHi(s)) || (s->ip_hv < (uint16_t)(s->set_v + MS_LIM_MARGIN_V)))                      return MS_BAD_IP_HV;
    if ((s->op_lv < s->ip_lv) || ((uint16_t)(s->op_lv + MS_LIM_MARGIN_V) > s->set_v))                         return MS_BAD_OP_LV;   /* between the input limit and 10 V below Set */
    if ((s->op_hv > s->ip_hv) || (s->op_hv < (uint16_t)(s->set_v + MS_LIM_MARGIN_V)))                         return MS_BAD_OP_HV;
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
    if ((s->heat_temp < MS_HEAT_TEMP_MIN) || (s->heat_temp > MS_HEAT_TEMP_MAX)) return MS_BAD_HEAT_TEMP;
    if ((s->heat_reset < MS_HEAT_RESET_MIN) || ((uint32_t)s->heat_reset + MS_HEAT_RESET_GAP > (uint32_t)s->heat_temp)) return MS_BAD_HEAT_RESET;
    if ((s->phase_seq > 1U) || (s->scroll > 1U) || (s->restart_auto > 1U)) return MS_BAD_FLAG;
    if ((s->load_src > 1U) || (s->sl_zero_ca > 4000U) || ((s->sl_gain != 0U) && ((s->sl_gain < 200U) || (s->sl_gain > 5000U)))) return MS_BAD_FLAG;
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
    o[32] = s->heat_reset;
    o[26] = s->load_src; o[27] = s->heat_temp; w16(&o[28], s->sl_zero_ca); w16(&o[30], s->sl_gain);       /* bytes 26 to 31 were always zero: old records read as "master CT, not calibrated, overheat 75 degC" */
}

void MS_Unpack(MSettings_t *s, const uint8_t o[MS_PACK_LEN])
{
    s->mode = o[0]; s->set_v = r16(&o[1]); s->maxcorr_v = r16(&o[3]);
    s->ip_lv = r16(&o[5]); s->ip_hv = r16(&o[7]); s->op_lv = r16(&o[9]); s->op_hv = r16(&o[11]);
    s->volt_cut_delay = o[13]; s->ovl_current = r16(&o[14]); s->ovl_cut_delay = o[16]; s->heat_delay = o[17];
    s->phase_seq = o[18]; s->scroll = o[19]; s->scroll_time = o[20]; s->password = r16(&o[21]); s->restart_auto = o[23]; s->restart_time = r16(&o[24]);
    s->load_src = o[26]; s->heat_temp = (o[27] == 0U) ? MS_HEAT_TEMP_DEFAULT : o[27]; s->heat_reset = (o[32] == 0U) ? MS_HeatResetDefault(s->heat_temp) : o[32]; s->sl_zero_ca = r16(&o[28]); s->sl_gain = r16(&o[30]);
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
    {   /* v0.9.50: the record of the firmware up to v0.9.49 (version 1, 32 bytes): every setting is carried over, the Overheat Restart gets its default. The first save writes the new record
         * into the OTHER page than the old one (the store is told where the newest slot is), so a power cut during that save leaves the old record to be migrated again. */
        MStore_t old; uint8_t b1[MS_PACK_LEN_V1], bn[MS_PACK_LEN]; uint32_t i;
        MStore_Init(&old, st->fl, st->addr[0], st->addr[1], MS_VERSION_V1, MS_PACK_LEN_V1);
        if (MStore_Load(&old, b1) != 0U)
        {
            MSettings_t t;
            for (i = 0U; i < MS_PACK_LEN; i++) bn[i] = (i < MS_PACK_LEN_V1) ? b1[i] : 0U;
            MS_Unpack(&t, bn);
            if (MS_Validate(&t, 0) == MS_OK) { *s = t; st->cur = old.cur; st->seq = old.seq; return 1U; }
        }
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
