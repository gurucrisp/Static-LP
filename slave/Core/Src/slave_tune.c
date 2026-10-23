/** @file slave_tune.c  @brief Transformer autotune, profile and model check. See slave_tune.h. */
#include "slave_tune.h"

typedef enum { TS_IDLE = 0, TS_BASE, TS_STEP, TS_DONE } TuneSt_t;

static const uint16_t S_TICKS[TUNE_N_STEPS] = { 16U, 32U, 64U, 128U, 256U, 512U, 1024U, 1600U, 2000U, 2752U };

static struct
{
    TuneSt_t st;
    uint32_t t0, wait_t0, t_valid;
    float    maxc, base, vin_ref;
    float    sum, vin_sum, vin_min, vin_max; uint32_t n;
    uint8_t  step_i, last_step;               /* index in S_TICKS; the current step is the last one (shortened to the limit) */
    float    cur_ticks;
    uint16_t pt_t[TUNE_MAX_POINTS]; float pt_dv[TUNE_MAX_POINTS]; uint8_t npts;
    TuneFrame_t q[4]; uint8_t qh, qn;
    TuneProfile_t prof; uint8_t prof_ok, done_pending;
    float    last_vin, last_dv;
    uint16_t hw_cap;
} s;

static float fabs_(float x) { return (x < 0.0f) ? -x : x; }

float Tune_RefG(float vin)
{   /* the bench curve (volts per effective tick against the input voltage): the shape of the injection against the input */
    if (vin <= 0.0f) return 0.0f;
    if (vin <= 110.0f) return 0.011f * vin / 110.0f;
    if (vin <= 200.0f) return 0.011f + (0.0297f - 0.011f) * (vin - 110.0f) / 90.0f;
    if (vin <= 250.0f) return 0.0297f + (0.0328f - 0.0297f) * (vin - 200.0f) / 50.0f;
    return 0.0328f;
}

static void push_frame(uint8_t step, uint16_t ticks, float vin, float dv, uint8_t result)
{
    TuneFrame_t *f;
    if (s.qn >= 4U) { s.qh = (uint8_t)((s.qh + 1U) & 3U); s.qn--; }                 /* the oldest is dropped: the master can only be slower than 4 frames */
    f = &s.q[(s.qh + s.qn) & 3U]; s.qn++;
    f->step = step; f->ticks = ticks; f->vin_dv = (uint16_t)((vin < 0.0f) ? 0.0f : (vin * 10.0f + 0.5f));
    { float d = dv * 10.0f; if (d > 999.0f) d = 999.0f; if (d < -999.0f) d = -999.0f; f->dv_dv = (int16_t)((d < 0.0f) ? (d - 0.5f) : (d + 0.5f)); }
    f->result = result;
}

static void reset_accum(uint32_t now) { s.t0 = now; s.sum = 0.0f; s.vin_sum = 0.0f; s.n = 0U; s.vin_min = 1.0e9f; s.vin_max = -1.0e9f; }

static void build_profile(uint8_t knee, uint8_t cap_idx)
{
    uint8_t i, a = 0xFFU; float g, gref, ratio, scale; TuneProfile_t *p = &s.prof;
    p->version = TUNE_VERSION; p->n = s.npts; p->knee = knee;
    p->vin_ref_dv = (uint16_t)(s.vin_ref * 10.0f + 0.5f); p->maxc_dv = (uint16_t)(s.maxc * 10.0f + 0.5f);
    for (i = 0U; i < s.npts; i++) { p->ticks[i] = s.pt_t[i]; p->dv_dv[i] = (int16_t)((s.pt_dv[i] < 0.0f) ? (s.pt_dv[i] * 10.0f - 0.5f) : (s.pt_dv[i] * 10.0f + 0.5f)); }
    p->cap_ticks = (s.pt_t[cap_idx] > s.hw_cap) ? s.hw_cap : s.pt_t[cap_idx];
    for (i = 0U; i <= cap_idx; i++) if (s.pt_dv[i] >= 1.0f) { a = i; break; }       /* the linear region: from the first point with a real injection to the cap point */
    if ((a != 0xFFU) && (cap_idx > a)) g = (s.pt_dv[cap_idx] - s.pt_dv[a]) / (float)(s.pt_t[cap_idx] - s.pt_t[a]);
    else g = s.pt_dv[cap_idx] / (float)s.pt_t[cap_idx];
    gref = Tune_RefG(s.vin_ref); ratio = (gref > 0.0f) ? (g / gref) : 1.0f;
    scale = (ratio > 0.0001f) ? (1.0f / ratio) : TUNE_SCALE_MAX;
    if (scale < TUNE_SCALE_MIN) scale = TUNE_SCALE_MIN;
    if (scale > TUNE_SCALE_MAX) scale = TUNE_SCALE_MAX;
    p->kib_m = (uint16_t)(scale * 1000.0f + 0.5f); p->kiu_m = p->kib_m;
    s.prof_ok = 1U;
}

static void finish(uint8_t result, uint8_t knee, uint8_t cap_idx)
{
    if ((result == TUNE_R_FINISHED) && (s.npts >= 3U)) build_profile(knee, cap_idx);
    else if (result == TUNE_R_FINISHED) result = TUNE_R_NO_RESPONSE;               /* fewer than 3 points: nothing to build a curve from */
    push_frame((uint8_t)(s.step_i + 1U), (uint16_t)s.cur_ticks, s.last_vin, s.last_dv, result);
    s.st = TS_DONE; s.done_pending = 1U; s.cur_ticks = 0.0f;
}

uint8_t Tune_Start(float maxc_v, uint32_t now_ms)
{
    if (maxc_v <= 0.0f) return 1U;
    if ((s.st == TS_BASE) || (s.st == TS_STEP)) return 2U;
    s.st = TS_BASE; s.maxc = maxc_v; s.npts = 0U; s.step_i = 0U; s.last_step = 0U; s.cur_ticks = 0.0f; s.prof_ok = 0U; s.done_pending = 0U; s.qh = 0U; s.qn = 0U;
    s.wait_t0 = now_ms; s.t_valid = now_ms; s.last_vin = 0.0f; s.last_dv = 0.0f; s.hw_cap = 2752U;
    reset_accum(now_ms);
    return 0U;
}

void Tune_Abort(void) { if ((s.st == TS_BASE) || (s.st == TS_STEP)) finish(TUNE_R_BY_MASTER, 0U, 0U); }
uint8_t Tune_Running(void) { return (uint8_t)((s.st == TS_BASE) || (s.st == TS_STEP)); }
uint8_t Tune_TakeFrame(TuneFrame_t *f) { if (s.qn == 0U) return 0U; *f = s.q[s.qh]; s.qh = (uint8_t)((s.qh + 1U) & 3U); s.qn--; return 1U; }
uint8_t Tune_GetProfile(TuneProfile_t *p) { if (s.prof_ok == 0U) return 0U; *p = s.prof; return 1U; }

/* the decision at the end of a step of s.cur_ticks with the measured injection dv (the baseline already taken off) */
static void step_done(float dv, float vin)
{
    uint8_t i = s.npts; float t = s.cur_ticks, slope; uint8_t last = s.last_step;
    s.last_vin = vin; s.last_dv = dv;
    if (i < TUNE_MAX_POINTS) { s.pt_t[i] = (uint16_t)(t + 0.5f); s.pt_dv[i] = dv; s.npts = (uint8_t)(i + 1U); }
    push_frame((uint8_t)(s.step_i + 1U), (uint16_t)(t + 0.5f), vin, dv, TUNE_R_STEP);
    i = s.npts;                                                                     /* the number of points now */
    if (((t >= 128.0f) && (dv < -TUNE_FLIP_V)) || ((t >= 64.0f) && (dv < -2.0f))) { finish(TUNE_R_FLIPPED, 0U, 0U); return; }
    if ((t >= 256.0f) && (dv < TUNE_NO_RESP_V)) { finish(TUNE_R_NO_RESPONSE, 0U, 0U); return; }
    if (i >= 3U)
    {   /* a saturation knee: the slope falls below 40 % of the previous one (judged only when the previous increment was a real one, not noise) */
        float inc_prev = s.pt_dv[i - 2U] - s.pt_dv[i - 3U], sp = inc_prev / (float)(s.pt_t[i - 2U] - s.pt_t[i - 3U]);
        float sc = (s.pt_dv[i - 1U] - s.pt_dv[i - 2U]) / (float)(s.pt_t[i - 1U] - s.pt_t[i - 2U]);
        if ((inc_prev >= 1.0f) && (sc < TUNE_KNEE_FRAC * sp)) { finish(TUNE_R_FINISHED, 1U, (uint8_t)(i - 2U)); return; }      /* the cap is the step BEFORE the knee */
    }
    if (last != 0U) { finish(TUNE_R_FINISHED, 0U, (uint8_t)(i - 1U)); return; }          /* that was the shortened step at the limit */
    /* the next step: predicted from the last slope, shortened so that it stays at or below 80 % of Max Correction */
    if (dv >= TUNE_LIMIT_FRAC * s.maxc) { finish(TUNE_R_FINISHED, 0U, (uint8_t)(i - 1U)); return; }
    if (i >= 2U) slope = (s.pt_dv[i - 1U] - s.pt_dv[i - 2U]) / (float)(s.pt_t[i - 1U] - s.pt_t[i - 2U]); else slope = (t > 0.0f) ? (dv / t) : 0.0f;
    if (slope < 0.0f) slope = 0.0f;
    s.step_i++;
    if ((s.step_i >= TUNE_N_STEPS) || (S_TICKS[s.step_i] > s.hw_cap)) { finish(TUNE_R_FINISHED, 0U, (uint8_t)(i - 1U)); return; }
    {
        float nt = (float)S_TICKS[s.step_i], pred = dv + slope * (nt - t), lim = TUNE_LIMIT_FRAC * s.maxc;
        if (pred > lim)
        {
            if (slope <= 0.0f) { finish(TUNE_R_FINISHED, 0U, (uint8_t)(i - 1U)); return; }
            nt = t + (lim - dv) / slope;
            if (nt < t + 8.0f) { finish(TUNE_R_FINISHED, 0U, (uint8_t)(i - 1U)); return; }
            s.last_step = 1U;
        }
        if (nt > (float)s.hw_cap) nt = (float)s.hw_cap;
        s.cur_ticks = nt; s.st = TS_STEP;
    }
}

void Tune_Service(uint32_t now, const TuneIn_t *in, TuneOut_t *out)
{
    out->active = (uint8_t)((s.st == TS_BASE) || (s.st == TS_STEP)); out->done = 0U;
    if ((s.st == TS_BASE) || (s.st == TS_STEP))
    {
        float d;
        if (in->hw_cap_ticks != 0U) s.hw_cap = (in->hw_cap_ticks > 2752U) ? 2752U : in->hw_cap_ticks;
        if (in->meas_valid != 0U) s.t_valid = now;
        if ((uint32_t)(now - s.t_valid) >= TUNE_STALE_MS) { finish(TUNE_R_ABORTED, 0U, 0U); goto out; }
        if (in->meas_valid == 0U) goto out;
        if ((in->fault != 0U) || (in->maxc_v <= 0.0f)) { finish(TUNE_R_ABORTED, 0U, 0U); goto out; }
        if (in->vout > in->vin + s.maxc + TUNE_OVERSHOOT_V) { finish(TUNE_R_ABORTED, 0U, 0U); goto out; }     /* more injection than the installer's rating: stop at once */
        if (in->vin_in_window == 0U) { finish(TUNE_R_UNSTEADY, 0U, 0U); goto out; }
        d = in->vout - in->vin;
        if (s.st == TS_BASE)
        {
            s.sum += d; s.vin_sum += in->vin; s.n++; if (in->vin < s.vin_min) s.vin_min = in->vin; if (in->vin > s.vin_max) s.vin_max = in->vin;
            if ((s.vin_max - s.vin_min) > TUNE_STEADY_PP_V)
            {   /* the input moved: start the window again, for at most TUNE_WAIT_STEADY_MS in all */
                if ((uint32_t)(now - s.wait_t0) >= TUNE_WAIT_STEADY_MS) { finish(TUNE_R_UNSTEADY, 0U, 0U); goto out; }
                reset_accum(now);
            }
            else if (((uint32_t)(now - s.t0) >= TUNE_BASE_MS) && (s.n > 0U))
            {
                s.base = s.sum / (float)s.n; s.vin_ref = s.vin_sum / (float)s.n;
                s.step_i = 0U; s.cur_ticks = (float)S_TICKS[0]; s.last_step = 0U; s.st = TS_STEP; reset_accum(now);
            }
        }
        else
        {   /* TS_STEP: the first TUNE_SETTLE_MS are not averaged */
            uint32_t el = (uint32_t)(now - s.t0);
            if (el >= TUNE_SETTLE_MS)
            {
                s.sum += d; s.vin_sum += in->vin; s.n++; if (in->vin < s.vin_min) s.vin_min = in->vin; if (in->vin > s.vin_max) s.vin_max = in->vin;
                if ((s.vin_max - s.vin_min) > TUNE_STEADY_PP_V) { finish(TUNE_R_UNSTEADY, 0U, 0U); goto out; }
            }
            if ((el >= TUNE_STEP_MS) && (s.n > 0U))
            {
                float dv = (s.sum / (float)s.n) - s.base, vin = s.vin_sum / (float)s.n;
                step_done(dv, vin);
                if (s.st == TS_STEP) reset_accum(now);
            }
        }
    }
out:
    out->active = (uint8_t)((s.st == TS_BASE) || (s.st == TS_STEP));
    out->request_ticks = (s.st == TS_STEP) ? (uint16_t)(s.cur_ticks + 0.5f) : 0U;
    out->frame_ready = (uint8_t)(s.qn != 0U);
    if (s.done_pending != 0U) { out->done = 1U; s.done_pending = 0U; }
}

/* ------------------------------------------------------------------ the profile ------------------------------------------------------------------ */
float Tune_PredictDv(const TuneProfile_t *p, float ticks, float vin)
{
    float t0 = 0.0f, d0 = 0.0f, v = 0.0f, scale; uint8_t i;
    if (p->n == 0U) return 0.0f;
    if (ticks <= 0.0f) return 0.0f;
    for (i = 0U; i < p->n; i++)
    {
        float t1 = (float)p->ticks[i], d1 = (float)p->dv_dv[i] / 10.0f;
        if (ticks <= t1 || i == (uint8_t)(p->n - 1U)) { v = (t1 > t0) ? (d0 + (d1 - d0) * (ticks - t0) / (t1 - t0)) : d1; break; }       /* beyond the last point: the last segment goes on */
        t0 = t1; d0 = d1;
    }
    { float rv = Tune_RefG((float)p->vin_ref_dv / 10.0f); scale = (rv > 0.0f) ? (Tune_RefG(vin) / rv) : 1.0f; }
    return v * scale;
}

uint8_t Tune_ProfileValid(const TuneProfile_t *p)
{
    uint8_t i;
    if ((p->version != TUNE_VERSION) || (p->n < 3U) || (p->n > TUNE_MAX_POINTS)) return 0U;
    if ((p->cap_ticks < 16U) || (p->cap_ticks > 2752U)) return 0U;
    if ((p->kib_m < 100U) || (p->kib_m > 1500U) || (p->kiu_m < 100U) || (p->kiu_m > 1500U)) return 0U;
    if ((p->vin_ref_dv < 500U) || (p->vin_ref_dv > 3500U) || (p->maxc_dv == 0U)) return 0U;
    for (i = 1U; i < p->n; i++) if (p->ticks[i] <= p->ticks[i - 1U]) return 0U;
    if (p->ticks[0] == 0U) return 0U;
    return 1U;
}

static void put16(uint8_t *b, uint16_t v) { b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); }
static uint16_t get16(const uint8_t *b) { return (uint16_t)(b[0] | (b[1] << 8)); }

uint32_t Tune_Serialize(const TuneProfile_t *p, uint8_t *buf, uint32_t max)
{
    uint32_t need = TUNE_BYTES(p->n), k = 0U; uint8_t i;
    if ((need > max) || (p->n > TUNE_MAX_POINTS)) return 0U;
    buf[k++] = p->version; buf[k++] = p->n; buf[k++] = p->knee;
    put16(&buf[k], p->vin_ref_dv); k += 2U; put16(&buf[k], p->maxc_dv); k += 2U; put16(&buf[k], p->cap_ticks); k += 2U; put16(&buf[k], p->kib_m); k += 2U; put16(&buf[k], p->kiu_m); k += 2U;
    for (i = 0U; i < p->n; i++) { put16(&buf[k], p->ticks[i]); k += 2U; put16(&buf[k], (uint16_t)p->dv_dv[i]); k += 2U; }
    return k;
}

uint8_t Tune_Deserialize(const uint8_t *buf, uint32_t len, TuneProfile_t *p)
{
    uint32_t k = 0U; uint8_t i, n;
    if (len < 13U) return 0U;
    n = buf[1]; if ((n > TUNE_MAX_POINTS) || (len < TUNE_BYTES(n))) return 0U;
    p->version = buf[0]; p->n = n; p->knee = buf[2]; k = 3U;
    p->vin_ref_dv = get16(&buf[k]); k += 2U; p->maxc_dv = get16(&buf[k]); k += 2U; p->cap_ticks = get16(&buf[k]); k += 2U; p->kib_m = get16(&buf[k]); k += 2U; p->kiu_m = get16(&buf[k]); k += 2U;
    for (i = 0U; i < TUNE_MAX_POINTS; i++) { p->ticks[i] = 0U; p->dv_dv[i] = 0; }
    for (i = 0U; i < n; i++) { p->ticks[i] = get16(&buf[k]); k += 2U; p->dv_dv[i] = (int16_t)get16(&buf[k]); k += 2U; }
    return Tune_ProfileValid(p);
}

/* the lowest input from which setv can be reached at the cap: vin + injection(cap, vin) >= setv; the injection grows with the input, so bisection works */
uint16_t Tune_ReachLoDv(const TuneProfile_t *p, float setv)
{
    float lo = 50.0f, hi = setv; uint8_t it;
    if (Tune_ProfileValid(p) == 0U) return 0U;
    if (hi + Tune_PredictDv(p, (float)p->cap_ticks, hi) < setv) return 0U;
    if (lo + Tune_PredictDv(p, (float)p->cap_ticks, lo) >= setv) return (uint16_t)(lo * 10.0f);
    for (it = 0U; it < 40U; it++) { float mid = 0.5f * (lo + hi); if (mid + Tune_PredictDv(p, (float)p->cap_ticks, mid) >= setv) hi = mid; else lo = mid; }
    return (uint16_t)(hi * 10.0f + 0.5f);
}

uint16_t Tune_ReachHiDv(const TuneProfile_t *p, float setv)
{
    float lo = setv, hi = 400.0f; uint8_t it;
    if (Tune_ProfileValid(p) == 0U) return 0U;
    for (it = 0U; it < 40U; it++) { float mid = 0.5f * (lo + hi), inj = TUNE_BUCK_RATIO * Tune_PredictDv(p, (float)p->cap_ticks, mid); if (mid - inj <= setv) lo = mid; else hi = mid; }
    return (uint16_t)(lo * 10.0f + 0.5f);
}

uint8_t Tune_ModelCheck(const TuneProfile_t *p, float ticks, float vin, float dv_measured)
{
    float pred = Tune_PredictDv(p, ticks, vin), err, tol;
    if (pred < 3.0f) return TUNE_CHECK_SKIP;
    if ((dv_measured < -TUNE_FLIP_V) && (pred > 2.0f)) return TUNE_CHECK_FLIP;
    err = fabs_(dv_measured - pred); tol = TUNE_CHECK_TOL_REL * pred; if (tol < TUNE_CHECK_TOL_ABS_V) tol = TUNE_CHECK_TOL_ABS_V;
    return (err > tol) ? TUNE_CHECK_MISMATCH : TUNE_CHECK_OK;
}
