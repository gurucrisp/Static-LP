/** @file slave_tunechk.c  @brief The start-up check that the same transformer is fitted. See slave_tunechk.h. */
#include "slave_tunechk.h"

#define ST_BASE  0U
#define ST_WATCH 1U
#define ST_DONE  2U

static float absf(float x) { return (x < 0.0f) ? -x : x; }

static void window_reset(TuneChk_t *c) { c->wn = 0U; c->w_dv = 0.0f; c->w_d = 0.0f; c->w_vin = 0.0f; c->w_d0 = c->d_f; }

void TuneChk_Init(TuneChk_t *c, const TuneProfile_t *p, uint32_t now_ms)
{
    c->have = (uint8_t)((p != 0) && (Tune_ProfileValid(p) != 0U));
    if (c->have != 0U) c->p = *p;
    c->state = (c->have != 0U) ? ST_BASE : ST_DONE; c->strikes = 0U; c->t0_ms = now_ms; c->last_ms = now_ms;
    c->base_sum = 0.0f; c->base = 0.0f; c->d_f = 0.0f; c->vin_prev = 0.0f; c->base_n = 0U; window_reset(c);
}

void TuneChk_Stop(TuneChk_t *c) { c->state = ST_DONE; }

uint8_t TuneChk_Update(TuneChk_t *c, uint32_t now_ms, float d, uint8_t boost, float vin, float vout)
{
    float dt = (float)(uint32_t)(now_ms - c->last_ms), k;
    if (c->state == ST_DONE) return TUNECHK_CONT;
    if ((uint32_t)(now_ms - c->t0_ms) > TUNECHK_WATCH_MS) { c->state = ST_DONE; return TUNECHK_CONT; }     /* it never ran steady in boost: no verdict, the other checks still run */
    c->last_ms = now_ms;
    k = dt / TUNECHK_TAU_MS; if (k > 1.0f) k = 1.0f; if (k < 0.0f) k = 0.0f;
    c->d_f += (d - c->d_f) * k;
    if (c->state == ST_BASE)
    {   /* the sensors' own offset, with (almost) no correction applied */
        if ((d < TUNECHK_BASE_TICKS) && (c->d_f < TUNECHK_BASE_TICKS)) { c->base_sum += (vout - vin); if (c->base_n < 1000U) c->base_n++; c->vin_prev = vin; if (c->base_n < TUNECHK_BASE_SAMPLES) return TUNECHK_CONT; }
        if (c->base_n >= 3U) c->base = c->base_sum / (float)c->base_n;                                 /* too few samples (a fast start): offset 0 */
        c->state = ST_WATCH; c->vin_prev = vin; window_reset(c);
    }
    {   /* WATCH */
        const float jump = absf(vin - c->vin_prev);
        c->vin_prev = vin;
        if ((boost == 0U) || (c->d_f < TUNECHK_MIN_TICKS) || (jump > TUNECHK_JUMP_V)) { window_reset(c); if (jump > TUNECHK_JUMP_V) c->strikes = 0U; return TUNECHK_CONT; }
        if ((c->wn > 0U) && (absf(c->d_f - c->w_d0) > ((c->w_d0 * 0.10f > TUNECHK_STEADY_ABS) ? (c->w_d0 * 0.10f) : TUNECHK_STEADY_ABS))) { window_reset(c); return TUNECHK_CONT; }   /* still ramping: start again */
        if (c->wn == 0U) c->w_d0 = c->d_f;
        c->w_dv += (vout - vin) - c->base; c->w_d += c->d_f; c->w_vin += vin; c->wn++;
        if (c->wn >= TUNECHK_WINDOW)
        {
            const float n = (float)c->wn, dv = c->w_dv / n, dd = c->w_d / n, vv = c->w_vin / n;
            uint8_t v;
            window_reset(c);
            v = Tune_ModelCheck(&c->p, dd, vv, dv);
            if (v == TUNE_CHECK_SKIP) return TUNECHK_CONT;                                              /* too little predicted injection to judge */
            if (v == TUNE_CHECK_OK) { c->state = ST_DONE; return TUNECHK_OK; }
            if (v == TUNE_CHECK_FLIP) { c->state = ST_DONE; return TUNECHK_FLIP; }
            if (++c->strikes >= TUNECHK_STRIKES) { c->state = ST_DONE; return TUNECHK_MISMATCH; }       /* a different gain, twice in a row */
        }
    }
    return TUNECHK_CONT;
}
