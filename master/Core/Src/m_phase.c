/**
  ******************************************************************************
  * @file    m_phase.c
  * @brief   The phase sequence and the angles between the phases. See m_phase.h. No hardware, no maths library.
  ******************************************************************************
  */
#include "m_phase.h"

static const int32_t s_cos[91] = {   /* cos(0..90 degrees) in Q15 */
    32768, 32763, 32748, 32723, 32688, 32643, 32588, 32524, 32449, 32365,
    32270, 32166, 32052, 31928, 31795, 31651, 31499, 31336, 31164, 30983,
    30792, 30592, 30382, 30163, 29935, 29698, 29452, 29197, 28932, 28660,
    28378, 28088, 27789, 27482, 27166, 26842, 26510, 26170, 25822, 25466,
    25102, 24730, 24351, 23965, 23571, 23170, 22763, 22348, 21926, 21498,
    21063, 20622, 20174, 19720, 19261, 18795, 18324, 17847, 17364, 16877,
    16384, 15886, 15384, 14876, 14365, 13848, 13328, 12803, 12275, 11743,
    11207, 10668, 10126, 9580, 9032, 8481, 7927, 7371, 6813, 6252,
    5690, 5126, 4560, 3993, 3425, 2856, 2286, 1715, 1144, 572,
    0
};

int32_t MPhase_CosQ15(uint32_t deg_dd)
{
    uint32_t x = deg_dd % 3600U, i, f; int32_t neg = 0, a, b, v;
    if (x > 1800U) x = 3600U - x;                          /* cos is even */
    if (x > 900U) { x = 1800U - x; neg = 1; }              /* cos(180 - x) = -cos(x) */
    i = x / 10U; f = x % 10U;
    a = s_cos[i]; b = s_cos[(i < 90U) ? (i + 1U) : 90U];
    v = a + (((b - a) * (int32_t)f) / 10);
    return (neg != 0) ? -v : v;
}

static uint32_t isqrt64(uint64_t v)
{
    uint64_t r = 0U, bit = (uint64_t)1U << 62;
    while (bit > v) bit >>= 2;
    while (bit != 0U) { if (v >= (r + bit)) { v -= r + bit; r = (r >> 1) + bit; } else r >>= 1; bit >>= 2; }
    return (uint32_t)r;
}

void MPhase_Init(MPhase_t *p)
{
    uint8_t i;
    p->state = MPH_UNKNOWN; p->cand = MPH_UNKNOWN; p->cand_n = 0U; p->have_ang = 0U; p->ang_cls = MPH_UNKNOWN; p->t_last_ms = 0U;
    for (i = 0U; i < 3U; i++) p->ang_dd[i] = 0U;
}

static uint32_t dist_pm(uint32_t a, uint32_t b) { uint32_t d = (a > b) ? (a - b) : (b - a); return (d > 500U) ? (1000U - d) : d; }       /* circular distance on 0..999 */

uint8_t MPhase_Classify(const MPhRound_t *r, uint16_t *fy_pm, uint16_t *fb_pm)
{
    uint32_t sum = 0U, n = 0U, T, fy, fb, i; int32_t x;
    for (i = 0U; i < 3U; i++) { if (r->valid[i] == 0U) return MPH_UNKNOWN; if (r->delay_us[i] == MPH_NONE_US) return MPH_UNKNOWN; }
    for (i = 0U; i < 3U; i++) { if (r->period_us[i] != 0U) { sum += r->period_us[i]; n++; } }
    if (n == 0U) return MPH_UNKNOWN;                                    /* no slave knew the period yet */
    T = sum / n;
    if (T == 0U) return MPH_UNKNOWN;
    for (i = 0U; i < 3U; i++) { if ((uint32_t)r->delay_us[i] > T + (T / 10U)) return MPH_UNKNOWN; }          /* a delay longer than a period: an edge was missed */
    x = (int32_t)r->delay_us[1] - (int32_t)r->delay_us[0]; x %= (int32_t)T; if (x < 0) x += (int32_t)T; fy = ((uint32_t)x * 1000U) / T;
    x = (int32_t)r->delay_us[2] - (int32_t)r->delay_us[0]; x %= (int32_t)T; if (x < 0) x += (int32_t)T; fb = ((uint32_t)x * 1000U) / T;
    if (fy_pm != 0) *fy_pm = (uint16_t)fy;
    if (fb_pm != 0) *fb_pm = (uint16_t)fb;
    if ((dist_pm(fy, 0U) < MPH_TOL_PM) || (dist_pm(fb, 0U) < MPH_TOL_PM) || (dist_pm(fy, fb) < MPH_TOL_PM)) return MPH_SAME;
    if ((dist_pm(fy, 333U) < MPH_TOL_PM) && (dist_pm(fb, 667U) < MPH_TOL_PM)) return MPH_OK;
    if ((dist_pm(fy, 667U) < MPH_TOL_PM) && (dist_pm(fb, 333U) < MPH_TOL_PM)) return MPH_REVERSE;
    return MPH_UNKNOWN;
}

void MPhase_Round(MPhase_t *p, const MPhRound_t *r, uint32_t now_ms)
{
    uint16_t fy = 0U, fb = 0U; uint8_t cls = MPhase_Classify(r, &fy, &fb);
    if (cls == MPH_UNKNOWN) { p->cand = MPH_UNKNOWN; p->cand_n = 0U; return; }               /* says nothing: the decision stays, and ages */
    if (cls == p->cand) { if (p->cand_n < 255U) p->cand_n++; } else { p->cand = cls; p->cand_n = 1U; }
    p->t_last_ms = now_ms;
    if (p->cand_n >= MPH_CONFIRM) p->state = cls;
    if ((cls == MPH_OK) || (cls == MPH_REVERSE))
    {   /* the angles follow the latest rounds (a quarter of the way each time: the jitter of one round is a degree or two) */
        uint32_t ty = ((uint32_t)fy * 36U) / 10U, tb = ((uint32_t)fb * 36U) / 10U;
        if ((p->have_ang == 0U) || (p->ang_cls != cls)) { p->ang_dd[1] = (uint16_t)ty; p->ang_dd[2] = (uint16_t)tb; p->have_ang = 1U; p->ang_cls = cls; }      /* the first round, or a change of class (the sequence turned over): the new angles at once, not slid towards */
        else { p->ang_dd[1] = (uint16_t)((int32_t)p->ang_dd[1] + (((int32_t)ty - (int32_t)p->ang_dd[1]) / 4)); p->ang_dd[2] = (uint16_t)((int32_t)p->ang_dd[2] + (((int32_t)tb - (int32_t)p->ang_dd[2]) / 4)); }
        p->ang_dd[0] = 0U;
    }
    if (p->state == MPH_SAME) p->have_ang = 0U;
}

void MPhase_Age(MPhase_t *p, uint32_t now_ms)
{
    if ((p->state != MPH_UNKNOWN) && ((uint32_t)(now_ms - p->t_last_ms) >= MPH_STALE_MS)) MPhase_Lost(p);
}

void MPhase_Lost(MPhase_t *p) { p->state = MPH_UNKNOWN; p->cand = MPH_UNKNOWN; p->cand_n = 0U; p->have_ang = 0U; }

uint16_t MPhase_LineDv(const MPhase_t *p, uint8_t a, uint8_t b, uint16_t va_dv, uint16_t vb_dv)
{
    uint32_t th; int64_t v2, ab; int32_t c;
    if ((p->have_ang == 0U) || (a > 2U) || (b > 2U) || (a == b) || (va_dv == 0U) || (vb_dv == 0U)) return 0U;
    th = ((uint32_t)p->ang_dd[b] + 3600U - (uint32_t)p->ang_dd[a]) % 3600U;
    c = MPhase_CosQ15(th);
    ab = (int64_t)va_dv * (int64_t)vb_dv;
    v2 = (int64_t)va_dv * (int64_t)va_dv + (int64_t)vb_dv * (int64_t)vb_dv - ((2 * ab * (int64_t)c) / 32768);
    if (v2 < 0) v2 = 0;
    { uint32_t r = isqrt64((uint64_t)v2); return (r > 65535U) ? 65535U : (uint16_t)r; }
}
