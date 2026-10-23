/** @file m_app.c  @brief The application: settings, log, UI and keys together. See m_app.h. */
#include "m_app.h"



/* The display value of a reading (tenths): a first-order filter (about 0.6 s) that jumps at once on a step of 5 V or more, then a hysteresis on what is SHOWN so the last digit does not flicker
 * between two values: the shown value changes only when the filtered reading is `band` tenths away from it. `step` is the size of one displayed unit in tenths (10 = whole volts, 1 = tenths). */
static uint16_t disp_filter(MDispFilt_t *f, uint16_t raw, uint8_t have, int32_t step, int32_t band, uint32_t now)
{
    int32_t r = (int32_t)raw * 256, e, diff; uint8_t jumped = 0U;
    if (have == 0U) { f->init = 0U; return raw; }
    if (f->init == 0U) { f->ema = r; f->shown = (((int32_t)raw + (step / 2)) / step) * step; f->init = 1U; f->t_chg = now; return (uint16_t)f->shown; }
    diff = r - f->ema;
    if ((diff > (50 * 256)) || (diff < -(50 * 256))) { f->ema = r; jumped = 1U; }          /* a real step: follow it at once */
    else f->ema += diff / MAPP_DISP_EMA_DIV;
    e = (f->ema + 128) / 256;
    if (((e - f->shown) >= band) || ((f->shown - e) >= band))
    {   /* the shown value changes only when the filtered reading is `band` away from it, and (except for a real step) not more often than MAPP_DISP_HOLD_MS */
        if ((jumped != 0U) || ((uint32_t)(now - f->t_chg) >= MAPP_DISP_HOLD_MS)) { f->shown = ((e + (step / 2)) / step) * step; f->t_chg = now; }
    }
    return (uint16_t)f->shown;
}


/* the load current in tenths of an amp from the CT count (times 16) and the calibration */
static uint16_t ct_to_da(const MCal_t *c, uint16_t x16)
{
    int32_t d = (int32_t)x16 - (int32_t)c->zero_x16; uint64_t v;
    if (d < 0) d = 0;
    v = ((uint64_t)(uint32_t)d * MAPP_CT_DA_PER_COUNT_X1000 * (uint64_t)c->gain_x1000 + 8000000ULL) / 16000000ULL;
    return (v > 6000ULL) ? 6000U : (uint16_t)v;
}
static uint32_t ct_nominal_da(const MCal_t *c, uint16_t x16)      /* the same with gain 1: what the span calibration divides by */
{
    int32_t d = (int32_t)x16 - (int32_t)c->zero_x16; if (d < 0) d = 0;
    return (uint32_t)((((uint64_t)(uint32_t)d * MAPP_CT_DA_PER_COUNT_X1000) + 8000ULL) / 16000ULL);
}
static void cal_defaults(MCal_t *c) { c->zero_x16 = (uint16_t)MAPP_CT_ZERO_DEFAULT_X16; c->gain_x1000 = (uint16_t)MAPP_CT_GAIN_DEFAULT; c->from_flash = 0U; }
static int cal_save(MApp_t *a)
{
    uint8_t b[4]; b[0] = (uint8_t)a->cal.zero_x16; b[1] = (uint8_t)(a->cal.zero_x16 >> 8); b[2] = (uint8_t)a->cal.gain_x1000; b[3] = (uint8_t)(a->cal.gain_x1000 >> 8);
    if (a->cstore.fl == 0) return -1;
    return MStore_Save(&a->cstore, b);
}
void MApp_InitCal(MApp_t *a, uint32_t pa, uint32_t pb)
{
    uint8_t b[4];
    MStore_Init(&a->cstore, a->fl, pa, pb, 1U, 4U);
    cal_defaults(&a->cal);
    if (MStore_Load(&a->cstore, b) != 0U)
    {
        uint16_t z = (uint16_t)(b[0] | (b[1] << 8)), g = (uint16_t)(b[2] | (b[3] << 8));
        if ((g >= 200U) && (g <= 5000U) && (z <= (uint16_t)(4095U * 16U))) { a->cal.zero_x16 = z; a->cal.gain_x1000 = g; a->cal.from_flash = 1U; }
    }
    MUI_SetFeatures(&a->ui, MUI_FEAT_CALIBRATE);
}


/* the slave's start-up wait reasons as letters: Z zero-cross pattern, V input window, M measurement, F FLT low, B break input low (or floating), O output too high, R relays not settled */
static uint32_t wait_letters(uint8_t ctx, char *o)
{
    static const char L[7] = { 'Z', 'V', 'M', 'F', 'B', 'O', 'R' }; uint32_t k, n = 0U;
    for (k = 0U; k < 7U; k++) if (((ctx >> k) & 1U) != 0U) o[n++] = L[k];
    o[n] = '\0'; return n;
}

static uint32_t age_ms(uint32_t now, uint32_t t) { return (uint32_t)(now - t); }
static void put2(char *o, uint32_t v) { o[0] = (char)('0' + ((v / 10U) % 10U)); o[1] = (char)('0' + (v % 10U)); }
static void copy(char *o, const char *s) { uint32_t i = 0U; while ((s[i] != '\0') && (i < 20U)) { o[i] = s[i]; i++; } o[i] = '\0'; }

/* what the unit is waiting for: shown on the Run screen's status row while the output is off, so it is never a mystery why nothing is starting */
static void make_status(const MApp_t *a, const MAppData_t *d, uint32_t now, char *o)
{
    const MUnit_t *u = &a->eng.u[0]; const uint32_t rd = (uint32_t)a->settings.restart_time * 1000UL; uint32_t rem;
    o[0] = '\0';
    if ((d->have == 0U) || (MUI_SetModeOpen(&a->ui) != 0U)) return;
    switch (u->st)
    {
    case U_OFF:      copy(o, "Press START"); break;
    case U_ONDELAY:  rem = (age_ms(now, u->t_enter) >= rd) ? 0U : ((rd - age_ms(now, u->t_enter) + 999UL) / 1000UL); copy(o, "Start in 00 s"); put2(&o[9], rem); break;
    case U_STARTING:
        if ((d->wait_valid != 0U) && ((d->wait_ctx & 0x7FU) != 0U))
        {   /* the slave was told to run but is waiting for something: SAY WHAT (letters, see wait_letters) */
            char w[10]; copy(o, "Slave Waits:"); (void)wait_letters((uint8_t)(d->wait_ctx & 0x7FU), w); { uint32_t i = 0U; while ((w[i] != '\0') && (12U + i < 20U)) { o[12U + i] = w[i]; i++; } o[12U + i] = '\0'; }
        }
        else if (u->inband_run == 0U)
        {   /* rising to the set voltage: "To 240 V (230)" = the set voltage and what the output is now */
            uint32_t sv = a->settings.set_v, ov = (uint32_t)(d->vout_dv + 5U) / 10U; copy(o, "To 000 V (000)");
            o[3] = (char)('0' + ((sv / 100U) % 10U)); o[4] = (char)('0' + ((sv / 10U) % 10U)); o[5] = (char)('0' + (sv % 10U));
            o[10] = (char)('0' + ((ov / 100U) % 10U)); o[11] = (char)('0' + ((ov / 10U) % 10U)); o[12] = (char)('0' + (ov % 10U));
        }
        else copy(o, "At Setpoint...");
        break;
    default: break;
    }
}

void MApp_Init(MApp_t *a, const MFlash_t *fl, uint32_t sa, uint32_t sb, uint32_t la, uint32_t lb, uint32_t now)
{
    uint8_t *p = (uint8_t *)a; uint32_t i;
    for (i = 0U; i < (uint32_t)sizeof(*a); i++) p[i] = 0U;
    a->fl = fl;
    MStore_Init(&a->sstore, fl, sa, sb, MS_VERSION, MS_PACK_LEN);
    a->settings_from_flash = MS_Load(&a->sstore, &a->settings);          /* damaged, blank or breaking the rules: the defaults */
    MLog_Init(&a->log, fl, la, lb);
    MK_Init(&a->keys);
    MF_Init(&a->eng, &a->settings, now);
    a->retries_cfg = (uint8_t)MF_RETRIES;
    cal_defaults(&a->cal);                                                  /* until MApp_InitCal: the original firmware's scale and 3 A offset */
    a->k_id = 1U;                                    /* power-up: Auto waits the Restart Time, Manual waits for START */
    MUI_Init(&a->ui, &a->settings, &a->log, now);
    a->ui.features |= MUI_FEAT_RETUNE;                                              /* the Re-tune item starts a transformer sweep (behind the Set-Mode password); a slave that cannot answers "Not Supported" */
    MUI_SetLimits(&a->ui, 0);                                              /* the slave's window and output guard come with its $H frame (stage 7c) */
    MUI_SetFeatures(&a->ui, 0U);                                           /* calibration and re-tune are not available yet */
}

/* The response timer: while the unit runs with the contactor closed, an output more than 5 V from the set voltage starts the clock; back within 1 V stops it. The last result is shown on the diagnostics page
 * (row 0: "3.4s/41V" = 3.4 s to come back from a 41 V error). A tool to measure the correction speed with a variac without a scope. An excursion that has not recovered after 30 s is dropped. */
static void rec_update(MApp_t *a, const MAppData_t *d, uint32_t now)
{
    int32_t err; uint32_t ae;
    if ((d->have == 0U) || (d->state != 2U) || (a->eout.contactor[0] == 0U)) { a->rec_active = 0U; return; }
    err = (int32_t)d->vout_dv - (int32_t)(a->settings.set_v * 10U); ae = (uint32_t)((err < 0) ? -err : err);
    if (a->rec_active == 0U) { if (ae > 50U) { a->rec_active = 1U; a->rec_t0 = now; a->rec_peak = (uint16_t)ae; } return; }
    if (ae > a->rec_peak) a->rec_peak = (uint16_t)ae;
    if (ae <= 10U)
    {
        uint32_t t10 = (uint32_t)(now - a->rec_t0) / 100U;
        a->rec_s10 = (uint16_t)((t10 > 999U) ? 999U : t10); a->rec_pk_v = (uint16_t)((a->rec_peak + 5U) / 10U); a->rec_have = 1U; a->rec_active = 0U;
    }
    else if ((uint32_t)(now - a->rec_t0) >= 30000UL) a->rec_active = 0U;
}

/* The CT peak backup (see m_app.h): above 3 x the Overload Current, or at the ADC full scale, continuously for 20 ms. */
static void ct_peak_update(MApp_t *a, const MAppData_t *d, uint32_t now)
{
    uint8_t hi = 0U;
    if (d->ct_max_x16 != 0U)
    {
        uint32_t lim_da = (uint32_t)a->settings.ovl_current * 10U * MAPP_CT_PEAK_FACTOR;
        if ((d->ct_max_x16 >= MAPP_CT_FULL_X16) || ((uint32_t)ct_to_da(&a->cal, d->ct_max_x16) > lim_da)) hi = 1U;
    }
    if (hi == 0U) { a->ct_hi = 0U; a->ct_peak_on = 0U; return; }
    if (a->ct_hi == 0U) { a->ct_hi = 1U; a->ct_hi_t0 = now; }
    a->ct_peak_on = (uint8_t)(((uint32_t)(now - a->ct_hi_t0) >= MAPP_CT_PEAK_MS) ? 1U : 0U);
}

/* ---------------- the transformer autotune (master side) ----------------
 * Re-tune (Set-Mode item 19, behind the password) -> $K TUNE (3 sends, same id) -> R -> the slave sends $T frames (one per step, the result repeated) -> a screen over everything. The unit is held off the whole time (the engine's setmode),
 * every key but STOP is swallowed, STOP sends TABORT. A refusal or a failure is explained on the screen. States: 0 idle, 1 asked (waiting for R), 2 running, 3 result on screen. */
#define TS_IDLE 0U
#define TS_ASK  1U
#define TS_RUN  2U
#define TS_DONE 3U
#define TR_LOST 7U      /* our own result code: the $T frames stopped coming */

static void tune_refuse(MApp_t *a, const char *l1, const char *l2, uint32_t now) { a->tune_state = TS_IDLE; MUI_Notify(&a->ui, l1, l2, now); }

static void tune_request(MApp_t *a, const MAppData_t *d, uint32_t now)
{
    if (a->tune_state != TS_IDLE) return;
    if (d->have == 0U) { MUI_Notify(&a->ui, "Slave Link Lost", "", now); return; }
    if ((a->eout.run[0] != 0U) || (d->state != 0U)) { MUI_Notify(&a->ui, "Stop The Unit", "First", now); return; }
    a->tune_state = TS_ASK; a->tune_id = MApp_NextKId(a); a->tune_cmd = (uint8_t)MAPP_KCMD_TUNE; a->tune_due = 1U; a->tune_tx_n = 0U; a->tune_t0 = now; a->tune_t_tx = now;
    a->tune_seen = d->tune_n; a->tune_step = 0U; a->tune_dv_dv = 0; a->tune_ticks = 0U; a->tune_result = 0U; a->tune_abort_sent = 0U;           /* frames already counted belong to an older sweep */
}

static void tune_abort(MApp_t *a, uint32_t now)
{
    if ((a->tune_abort_sent != 0U) || (a->tune_state == TS_IDLE) || (a->tune_state == TS_DONE)) return;
    a->tune_abort_sent = 1U; a->tune_id = MApp_NextKId(a); a->tune_cmd = (uint8_t)MAPP_KCMD_TABORT; a->tune_due = 1U; a->tune_tx_n = 0U; a->tune_t_tx = now; a->tune_t0 = now;
}

static void tune_key(MApp_t *a, uint8_t ev, uint32_t now)
{
    if (a->tune_state == TS_DONE) { if (ev != (uint8_t)MK_NONE) a->tune_state = TS_IDLE; return; }                     /* any key dismisses the result */
    if (ev == (uint8_t)MK_STOP) tune_abort(a, now);
}

static void tune_service(MApp_t *a, const MAppData_t *d, uint32_t now)
{
    if (a->tune_state == TS_IDLE) return;
    if (a->tune_state == TS_DONE) { if ((uint32_t)(now - a->tune_t_done) >= MAPP_TUNE_RESULT_MS) a->tune_state = TS_IDLE; return; }
    /* a command is repeated with the same id until it is answered (3 sends, 200 ms apart) */
    if ((a->tune_due == 0U) && (a->tune_tx_n > 0U) && (a->tune_tx_n < 3U) && ((uint32_t)(now - a->tune_t_tx) >= MAPP_VCAL_RETRY_MS) && ((a->tune_state == TS_ASK) || (a->tune_abort_sent != 0U))) a->tune_due = 1U;
    if ((a->tune_state == TS_ASK) && ((uint32_t)(now - a->tune_t0) >= MAPP_VCAL_TIMEOUT_MS)) { tune_refuse(a, "No Answer", "From The Slave", now); return; }
    if (a->tune_state == TS_RUN)
    {
        if (d->tune_n != a->tune_seen)
        {   /* a new $T frame */
            a->tune_seen = d->tune_n; a->tune_t_frame = now;
            if (d->tune_result == 0U) { a->tune_step = d->tune_step; a->tune_ticks = d->tune_ticks; a->tune_dv_dv = d->tune_dv_dv; }
            else { a->tune_result = d->tune_result; a->tune_step = d->tune_step; a->tune_dv_dv = d->tune_dv_dv; a->tune_state = TS_DONE; a->tune_t_done = now; if (d->tune_result == 1U) MF_RetuneDone(&a->eng); return; }
        }
        if ((a->tune_abort_sent != 0U) && ((uint32_t)(now - a->tune_t0) >= 5000UL)) { a->tune_result = 6U; a->tune_state = TS_DONE; a->tune_t_done = now; return; }       /* the abort was never confirmed: say so */
        if ((uint32_t)(now - a->tune_t_frame) >= MAPP_TUNE_FRAME_TIMEOUT_MS) { tune_abort(a, now); a->tune_result = TR_LOST; a->tune_state = TS_DONE; a->tune_t_done = now; }
    }
}

static void tune_on_result(MApp_t *a, uint8_t status, int32_t value, uint32_t now)
{
    if (a->tune_cmd == (uint8_t)MAPP_KCMD_TABORT) { a->tune_due = 0U; return; }                    /* the abort is confirmed by the final $T frame, not by this answer */
    if (a->tune_state != TS_ASK) return;
    a->tune_due = 0U;
    if (status == 0U) { a->tune_state = TS_RUN; a->tune_t_frame = now; return; }
    if (status == 4U) { tune_refuse(a, "Stop The Unit", "First", now); return; }
    if (status == 6U) { tune_refuse(a, "Not Supported", "Update The Slave", now); return; }
    if (status == 7U)
    {
        if (value == 1) tune_refuse(a, "Tune Refused", "No Tune Mode Yet", now);
        else if (value == 2) tune_refuse(a, "Tune Refused", "Set Max Correction", now);
        else if (value == 3) tune_refuse(a, "Tune Refused", "Slave Busy", now);
        else if (value == 5) tune_refuse(a, "Tune Refused", "Slave Cannot Start", now);
        else if (value == 6) tune_refuse(a, "Input Too High", "Lower The Mains", now);
        else if (value == 4) { a->tune_state = TS_RUN; a->tune_t_frame = now; }                      /* it is already running: follow it */
        else tune_refuse(a, "Tune Refused", "Not Allowed", now);
        return;
    }
    tune_refuse(a, "Tune Refused", "Try Again", now);
}

uint8_t MApp_ShowsTune(const MApp_t *a) { return (uint8_t)(a->tune_state != TS_IDLE); }

static void tune_dv_text(char *o, int16_t dv)
{   /* "+12.4 V" */
    int32_t v = dv; uint32_t av = (uint32_t)((v < 0) ? -v : v), k = 0U;
    o[k++] = (v < 0) ? '-' : '+';
    if ((av / 100U) > 0U) o[k++] = (char)('0' + ((av / 100U) % 10U));
    o[k++] = (char)('0' + ((av / 10U) % 10U)); o[k++] = '.'; o[k++] = (char)('0' + (av % 10U)); o[k++] = ' '; o[k++] = 'V'; o[k] = '\0';
}

void MApp_TuneScreen(const MApp_t *a, uint32_t now, char frame[MUI_ROWS][MUI_COLS + 1U])
{
    uint32_t r, c, n; char t[24]; (void)now;
    for (r = 0U; r < MUI_ROWS; r++) { for (c = 0U; c < MUI_COLS; c++) frame[r][c] = ' '; frame[r][MUI_COLS] = '\0'; }
    if (a->tune_state == TS_DONE)
    {
        const char *l0, *l1, *l2;
        switch (a->tune_result)
        {
        case 1U: l0 = "Tuning Complete"; l1 = "Transformer OK"; l2 = "Profile Stored"; break;
        case 2U: l0 = "Tuning Failed"; l1 = "Reversed Transformer"; l2 = "Check Connections"; break;
        case 3U: l0 = "Tuning Failed"; l1 = "No Response"; l2 = "Transformer Missing?"; break;
        case 4U: l0 = "Tuning Failed"; l1 = "Aborted By Slave"; l2 = "Fault Or Flash Error"; break;
        case 5U: l0 = "Tuning Failed"; l1 = "Input Not Steady"; l2 = "Try Again Later"; break;
        case 6U: l0 = "Tuning Stopped"; l1 = "By Operator"; l2 = "Nothing Stored"; break;
        default: l0 = "Tuning Failed"; l1 = "Slave Not Answering"; l2 = "Check The Link"; break;
        }
        copy(t, l0); for (n = 0U; t[n] != '\0'; n++) frame[0][n] = t[n];
        copy(t, l1); for (n = 0U; t[n] != '\0'; n++) frame[1][n] = t[n];
        copy(t, l2); for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n];
        copy(t, "Press any key"); for (n = 0U; t[n] != '\0'; n++) frame[3][n] = t[n];
        return;
    }
    copy(t, "Transformer Tuning"); for (n = 0U; t[n] != '\0'; n++) frame[0][1U + n] = t[n];
    if ((a->tune_state == TS_ASK) || (a->tune_step == 0U)) { copy(t, (a->tune_state == TS_ASK) ? "Starting..." : "Measuring baseline"); for (n = 0U; t[n] != '\0'; n++) frame[1][n] = t[n]; copy(t, "Please wait"); for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n]; }
    else
    {
        copy(t, "Step 00 of 10"); t[5] = (char)('0' + ((a->tune_step / 10U) % 10U)); t[6] = (char)('0' + (a->tune_step % 10U)); if (t[5] == '0') t[5] = ' ';
        for (n = 0U; t[n] != '\0'; n++) frame[1][n] = t[n];
        copy(t, "Injected "); n = 0U; while (t[n] != '\0') n++; tune_dv_text(&t[n], a->tune_dv_dv);
        for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n];
    }
    copy(t, "STOP = abort"); for (n = 0U; t[n] != '\0'; n++) frame[3][n] = t[n];
}

static void vcal_request(MApp_t *a, const MAppData_t *d, uint32_t now);
static void vcal_service(MApp_t *a, uint32_t now);

void MApp_Step(MApp_t *a, uint8_t menu, uint8_t start, uint8_t stop, uint8_t log, const MAppData_t *d, uint32_t now, char frame[MUI_ROWS][MUI_COLS + 1U])
{
    MUIRun_t r; uint8_t *p = (uint8_t *)&r; uint32_t i; uint8_t ph; uint16_t ld; uint8_t lok;
    {   uint8_t ev = MK_Update(&a->keys, menu, start, stop, log, now);
        if (a->tune_state != 0U) tune_key(a, ev, now); else MUI_Key(&a->ui, ev, now);      /* while the transformer is being tuned STOP aborts and no other key does anything */
        if (a->tune_state != 0U) a->ui.t_key = now;                                          /* the Set-Mode idle timeout must not run out in the middle of a sweep */
    }

    for (i = 0U; i < (uint32_t)sizeof(r); i++) p[i] = 0U;
    for (ph = 0U; ph < 3U; ph++) { r.have[ph] = 0U; }
    r.have[0] = d->have; r.freq_chz = d->freq_chz;
    r.vin_dv[0]  = disp_filter(&a->f_vin,  d->vin_dv,  d->have, 10, 7, now);      /* whole volts, changes only when the filtered value is 0.7 V away */
    r.vout_dv[0] = disp_filter(&a->f_vout, d->vout_dv, d->have, 10, 7, now);
    ld = d->load_da; lok = d->load_ok;
    if (d->ct_raw != 0U) { ld = ct_to_da(&a->cal, d->ct_x16); lok = 1U; }                      /* from the CT count and the calibration */
    r.load_ok[0] = lok; r.load_da[0] = disp_filter(&a->f_load, ld, (uint8_t)(d->have && lok), 1, 3, now);   /* tenths of an amp, 0.3 A band */

    /* the slave's own limits (its $H frame): Set-Mode checks the I/P cut-offs against its window and O/P HV against its output guard (D7, D26) */
    if ((d->h_valid != 0U) && (a->lim_set == 0U))
    {
        a->lim.win_lo_v = (uint16_t)(d->win_lo_dv / 10U); a->lim.win_hi_v = (uint16_t)(d->win_hi_dv / 10U); a->lim.out_guard_v = (uint16_t)(d->vout_max_dv / 10U);
        MUI_SetLimits(&a->ui, &a->lim); a->lim_set = 1U;
    }
    /* the clear counter starts from the slave's own, so a master restart never looks like a "clear" */
    if ((d->e_valid != 0U) && (a->clr_synced == 0U)) { a->clr_cnt = d->clr_echo; a->clr_synced = 1U; }

    /* the engine decides everything: run, the clear pulses, the contactor, the buzzer, what to log */
    {
        MFIn_t in; uint8_t *q = (uint8_t *)&in; uint32_t k; uint8_t k_start = MUI_Take(&a->ui, MUI_REQ_START), k_stop = MUI_Take(&a->ui, MUI_REQ_STOP);
        for (k = 0U; k < (uint32_t)sizeof(in); k++) q[k] = 0U;
        in.ph[0].link_ok = d->link_ok; in.ph[0].vin_dv = d->vin_dv; in.ph[0].vout_dv = d->vout_dv;
        in.ph[0].load_da = (lok != 0U) ? ld : 0U;
        in.ph[0].slave_fault = (uint8_t)((d->have != 0U) ? d->fault : 0U);
        {   /* IN BAND = the output has reached the set voltage: the slave says so ($E IN_BAND), OR the master's own reading of the output is within 2 V of it while the slave is running */
            int32_t dv = (int32_t)d->vout_dv - (int32_t)(a->settings.set_v * 10U);
            uint8_t meas = (uint8_t)((d->have != 0U) && (d->state == 2U) && (dv <= (int32_t)MAPP_BAND_DV) && (dv >= -(int32_t)MAPP_BAND_DV));
            in.ph[0].in_band = (uint8_t)(((d->e_valid != 0U) && (d->in_band != 0U)) || (meas != 0U));
        }
        in.ph[0].heat = d->heat; in.ph[0].ct_peak = (uint8_t)(d->ct_peak | a->ct_peak_on);
        in.ph[0].slave_state = d->state; in.ph[0].state_valid = d->have;                    /* PWM off = load off */
        /* the Y and B slaves have no link yet: in Industrial and Domestic they count as missing, so those modes do not start */
        in.setmode = (uint8_t)((MUI_SetModeOpen(&a->ui) != 0U) || (a->tune_state != 0U)); in.key_start = k_start; in.key_stop = k_stop; in.run_min = (d->clock_ok != 0U) ? MC_Stamp(&d->clock) : a->run_min_total;       /* the log time: the real-time clock when it is set, else the running minutes */
        a->eng.max_retries = a->retries_cfg;                                           /* MF_Init (a mode change) resets it: apply it every step */
        {   MSettings_t es = a->settings;                                                /* what the engine judges by */
            if (a->no_cuts != 0U) { es.ip_lv = 1U; es.ip_hv = 999U; es.op_lv = 1U; es.op_hv = 999U; }   /* TEST: no voltage limits */
            MF_Tick(&a->eng, &es, &in, now, &a->eout);
        }
        for (k = 0U; k < a->eout.n_log; k++) (void)MLog_Append(&a->log, &a->eout.log[k].rec);
        if (a->eout.clr_pulse[0] != 0U) a->clr_cnt++;
        a->ctl_out.run = a->eout.run[0]; a->ctl_out.clr = a->clr_cnt;
        a->ctl_out.setv_dv = (uint16_t)(a->settings.set_v * 10U); a->ctl_out.maxc_dv = (uint16_t)(a->settings.maxcorr_v * 10U);
    }

    /* $M is sent once the slave was heard ($E, the clear counter then in step with its own) OR its $S frames have been arriving for 3 s: a slave whose $E is not understood is still started */
    if (d->have != 0U) { if (a->have_seen == 0U) { a->have_seen = 1U; a->t_first_have = now; } } else a->have_seen = 0U;
    a->ctl_valid = (uint8_t)((a->clr_synced != 0U) || ((a->have_seen != 0U) && ((uint32_t)(now - a->t_first_have) >= 3000UL)));
    a->last = *d;
    make_status(a, d, now, a->status);
    r.status = a->status;

    /* the screen: what the engine shows (a tripped fault, else a limit being violated), else what the slave reports */
    r.out_on = a->eout.contactor[0];
    if (a->eout.fault[0] != 0U) r.fault[0] = a->eout.fault[0];
    else if (a->eout.appear[0] != 0U) r.fault[0] = a->eout.appear[0];
    else if ((d->have != 0U) && (d->fault != 0U)) r.fault[0] = MF_SlaveFaultType(d->fault, d->vin_dv, a->settings.set_v);
    r.fcode[0] = (uint8_t)((d->have != 0U) ? d->fault : 0U);
    if (d->have == 0U) r.fault[0] = (uint8_t)(a->eout.fault[0] != 0U ? a->eout.fault[0] : 0U);
    r.vin_raw_dv = d->vin_dv; r.vout_raw_dv = d->vout_dv; r.active = (uint8_t)((a->eout.run[0] != 0U) || (d->state != 0U));       /* for the Voltage Calibration pages */
    r.wait_s[0] = (a->eout.fault[0] != 0U) ? MF_RestartLeftS(&a->eng, 0U) : 0U;                                      /* the automatic restart countdown for the status row */
    r.fcode[0] = (a->eout.fault[0] != 0U) ? a->eng.u[0].fcode : (((a->eout.appear[0] == 0U) && (d->have != 0U)) ? d->fault : 0U);      /* the slave code behind the fault shown */
    MUI_Tick(&a->ui, &r, now, frame);

    if (MUI_Take(&a->ui, MUI_REQ_SAVE))
    {
        if (MS_Save(&a->sstore, &a->settings) != 0) { a->save_failures++; MUI_Notify(&a->ui, "Save Failed", "Flash Error", now); }
        else a->settings_from_flash = 1U;
    }
    if (MUI_Take(&a->ui, MUI_REQ_CLEAR_LOG)) (void)MLog_Clear(&a->log);
    MUI_SetClock(&a->ui, &d->clock, d->clock_ok);                                       /* the Date and Time items start from the clock */
    a->t_now = now;
    {   /* the running time: whole minutes while the contactor is closed (a step longer than 1 s, a stalled loop, adds only 1 s) */
        uint32_t dt = (uint32_t)(now - a->t_run_last); a->t_run_last = now; if (dt > 1000U) dt = 1000U;
        if (a->eout.contactor[0] != 0U) { a->run_acc_ms += dt; while (a->run_acc_ms >= 60000UL) { a->run_acc_ms -= 60000UL; a->run_min_total++; a->run_dirty = 1U; } }
    }
    rec_update(a, d, now);
    ct_peak_update(a, d, now);
    if (MUI_Take(&a->ui, MUI_REQ_VCAL_SAVE)) vcal_request(a, d, now);
    vcal_service(a, now);
    if (MUI_Take(&a->ui, MUI_REQ_SET_CLOCK)) { a->clk_set = *MUI_ClockToSet(&a->ui); a->clk_set_pending = 1U; }
    if (MUI_Take(&a->ui, MUI_REQ_CALIBRATE))
    {   /* ZERO: Set-Mode "Calibration" = Yes. The output is off in Set-Mode, so there is no load: the reading now IS the zero. */
        if ((d->ct_raw == 0U) || (a->cstore.fl == 0)) MUI_Notify(&a->ui, "Calibration", "Not Available", now);
        else if ((a->eout.contactor[0] != 0U) || (d->ct_x16 > (uint16_t)(600U * 16U))) MUI_Notify(&a->ui, "Load Not Zero", "Open The Load", now);
        else { uint16_t keep = a->cal.zero_x16; a->cal.zero_x16 = d->ct_x16; if (cal_save(a) == 0) { a->cal.from_flash = 1U; MUI_Notify(&a->ui, "Zero Set", "Now Span: MENU x3", now); } else { a->cal.zero_x16 = keep; MUI_Notify(&a->ui, "Save Failed", "Flash Error", now); } }
    }
    if (MUI_Take(&a->ui, MUI_REQ_CAL_SAVE))
    {   /* SPAN: the Load Calibration page, LOG. The meter value typed in, against what the CT reads now. */
        uint32_t entered = MUI_CalValue(&a->ui), nom = (d->ct_raw != 0U) ? ct_nominal_da(&a->cal, d->ct_x16) : 0U;
        if ((d->ct_raw == 0U) || (a->cstore.fl == 0)) MUI_Notify(&a->ui, "Calibration", "Not Available", now);
        else if (nom < MAPP_CT_MIN_CAL_DA) MUI_Notify(&a->ui, "Load Too Low", "Min 2.0 A", now);
        else if ((entered < MAPP_CT_MIN_CAL_DA) || (entered > 1800U)) MUI_Notify(&a->ui, "Enter 2.0-180.0 A", "", now);
        else
        {
            uint32_t g = (entered * 1000U + (nom / 2U)) / nom;
            if ((g < 200U) || (g > 5000U)) MUI_Notify(&a->ui, "Out Of Range", "Check The CT", now);
            else { uint16_t keep = a->cal.gain_x1000; a->cal.gain_x1000 = (uint16_t)g; if (cal_save(a) == 0) { a->cal.from_flash = 1U; MUI_Notify(&a->ui, "Calibrated", "Saved", now); } else { a->cal.gain_x1000 = keep; MUI_Notify(&a->ui, "Save Failed", "Flash Error", now); } }
        }
    }
    if (MUI_Take(&a->ui, MUI_REQ_RETUNE) != 0U) tune_request(a, d, now);
    tune_service(a, d, now);
}

uint8_t MApp_SetModeOpen(const MApp_t *a) { return MUI_SetModeOpen(&a->ui); }
const MCtlOut_t *MApp_Control(const MApp_t *a) { return &a->ctl_out; }
uint8_t MApp_ControlValid(const MApp_t *a) { return a->ctl_valid; }
void MApp_NoteFrameSent(MApp_t *a) { a->m_sent++; }
uint8_t MApp_TakeClockSet(MApp_t *a, MDateTime_t *out) { if (a->clk_set_pending == 0U) return 0U; *out = a->clk_set; a->clk_set_pending = 0U; return 1U; }
uint8_t MApp_HelloDue(MApp_t *a, uint32_t now)
{
    if ((a->last.e_valid == 0U) || (a->last.h_valid != 0U)) { a->hello_n = 0U; return 0U; }
    if ((a->hello_n != 0U) && ((uint32_t)(now - a->t_hello) < 2000UL)) return 0U;
    a->t_hello = now; a->hello_n = 1U; return 1U;
}
void MApp_SetInrushRetries(MApp_t *a, uint8_t n) { a->retries_cfg = n; a->eng.max_retries = n; }
void MApp_SetTestNoCuts(MApp_t *a, uint8_t on) { a->no_cuts = (uint8_t)(on != 0U); }
uint8_t MApp_ShowsDiag(const MApp_t *a) { return MUI_ShowsDiag(&a->ui); }

/* the control diagnostics (MENU, third page): why the unit is, or is not, running */
void MApp_Diag(const MApp_t *a, uint32_t now, char frame[MUI_ROWS][MUI_COLS + 1U])
{
    static const char *const US[6] = { "OFF", "ONDELAY", "STARTING", "ON", "FAULT", "LATCHED" };
    const MAppData_t *d = &a->last; char t[24]; uint32_t r, c, n; const char *s;
    for (r = 0U; r < MUI_ROWS; r++) { for (c = 0U; c < MUI_COLS; c++) frame[r][c] = ' '; frame[r][MUI_COLS] = '\0'; }
    (void)now;
    /* row 0: what has been heard from the slave, and how many $M frames went out */
    copy(t, "S:  E:  H:  M:00000"); t[2] = (d->have != 0U) ? 'Y' : 'N'; t[6] = (d->e_valid != 0U) ? 'Y' : 'N'; t[10] = (a->lim_set != 0U) ? 'Y' : 'N';
    { uint32_t m = a->m_sent % 100000UL; t[14] = (char)('0' + (m / 10000U)); t[15] = (char)('0' + ((m / 1000U) % 10U)); t[16] = (char)('0' + ((m / 100U) % 10U)); t[17] = (char)('0' + ((m / 10U) % 10U)); t[18] = (char)('0' + (m % 10U)); }
    if (a->rec_have != 0U)
    {   /* the last response time replaces the frame count, columns 12 to 19: "3.4s/41V" (a recovery of 10 s or more shows whole seconds: "12s/41V"; an error over 99 V shows 99) */
        char u[12]; uint32_t k = 0U, sec = a->rec_s10 / 10U, tn = a->rec_s10 % 10U, pv = (a->rec_pk_v > 99U) ? 99U : a->rec_pk_v;
        if (sec >= 10U) { u[k++] = (char)('0' + ((sec / 10U) % 10U)); u[k++] = (char)('0' + (sec % 10U)); }
        else { u[k++] = (char)('0' + sec); u[k++] = '.'; u[k++] = (char)('0' + tn); }
        u[k++] = 's'; u[k++] = '/'; u[k++] = (char)('0' + (pv / 10U)); u[k++] = (char)('0' + (pv % 10U)); u[k++] = 'V'; u[k] = '\0';
        for (c = 0U; c < 8U; c++) t[12U + c] = ' ';
        for (c = 0U; (u[c] != '\0') && (c < 8U); c++) t[12U + c] = u[c];
        t[20] = '\0';
    }
    for (n = 0U; t[n] != '\0'; n++) frame[0][n] = t[n];
    /* row 1: the unit's state, the run level it sends, the clear counter */
    s = US[(a->eng.u[0].st < 6U) ? a->eng.u[0].st : 0U];
    copy(t, "U:"); n = 2U; while ((*s != '\0') && (n < 12U)) t[n++] = *s++;
    t[n++] = ' '; t[n++] = 'r'; t[n++] = 'u'; t[n++] = 'n'; t[n++] = ':'; t[n++] = (char)('0' + (a->ctl_out.run & 1U)); t[n++] = ' '; t[n++] = 'c'; t[n++] = ':'; put2(&t[n], a->ctl_out.clr); n += 2U; t[n] = '\0';
    for (n = 0U; (t[n] != '\0') && (n < MUI_COLS); n++) frame[1][n] = t[n];
    /* row 2: the input voltage and the limits it is judged by */
    copy(t, "Vin 000 000-000"); { uint32_t v = (uint32_t)(d->vin_dv + 5U) / 10U; t[4] = (char)('0' + ((v / 100U) % 10U)); t[5] = (char)('0' + ((v / 10U) % 10U)); t[6] = (char)('0' + (v % 10U)); }
    t[8] = (char)('0' + ((a->settings.ip_lv / 100U) % 10U)); t[9] = (char)('0' + ((a->settings.ip_lv / 10U) % 10U)); t[10] = (char)('0' + (a->settings.ip_lv % 10U));
    t[12] = (char)('0' + ((a->settings.ip_hv / 100U) % 10U)); t[13] = (char)('0' + ((a->settings.ip_hv / 10U) % 10U)); t[14] = (char)('0' + (a->settings.ip_hv % 10U));
    if (a->no_cuts != 0U) copy(&t[15], " NOCUT");
    else
    {   /* T = the set voltage the SLAVE is using (its $E echo); "!" in place of the T when it differs from the Set Voltage in Set-Mode */
        uint32_t ev = (uint32_t)(d->e_setv_dv + 5U) / 10U; copy(&t[15], " T---");
        if ((d->e_valid != 0U) && (d->e_setv_dv != 0U))
        {
            t[17] = (char)('0' + ((ev / 100U) % 10U)); t[18] = (char)('0' + ((ev / 10U) % 10U)); t[19] = (char)('0' + (ev % 10U));
            if (ev != (uint32_t)a->settings.set_v) t[16] = '!';
        }
    }
    for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n];
    /* row 3: the slave's own state, fault code and IN_BAND */
    copy(t, "Slv st:0 F:00 inb:0"); t[7] = (char)('0' + (d->state % 10U)); put2(&t[11], d->fault); t[18] = (char)('0' + (d->in_band & 1U));
    if ((d->wait_valid != 0U) && ((d->wait_ctx & 0x7FU) != 0U)) { char w[10]; copy(&t[14], "W:"); (void)wait_letters((uint8_t)(d->wait_ctx & 0x7FU), w); { uint32_t i = 0U; while ((w[i] != '\0') && (16U + i < 20U)) { t[16U + i] = w[i]; i++; } t[16U + i] = '\0'; } }
    if (d->slave_wdg != 0U) { n = 0U; while (t[n] != '\0') n++; while (n < 19U) t[n++] = ' '; t[19] = 'w'; t[20] = '\0'; }      /* the slave's last reset was its watchdog */
    for (n = 0U; t[n] != '\0'; n++) frame[3][n] = t[n];
}

/* ---------------- voltage calibration over the link ---------------- */
void MApp_BootReset(MApp_t *a, uint8_t wdg)
{
    MLogRec_t r;
    if (wdg == 0U) return;
    r.type = (uint8_t)MLOG_T_WDG_RESET; r.phase = 3U; r.slave_code = 0U; r.vin_dv = 0U; r.vout_dv = 0U; r.load_da = 0U; r.run_min = a->run_min_total;
    (void)MLog_Append(&a->log, &r);
}

void MApp_SetRunMinutes(MApp_t *a, uint32_t minutes) { a->run_min_total = minutes & 0x7FFFFFFFUL; a->run_acc_ms = 0U; a->run_dirty = 0U; }      /* the top bit of a log time marks a real date and time */
uint32_t MApp_RunMinutes(const MApp_t *a) { return a->run_min_total; }
uint8_t MApp_TakeRunDirty(MApp_t *a) { uint8_t d = a->run_dirty; a->run_dirty = 0U; return d; }

uint8_t MApp_NextKId(MApp_t *a) { uint8_t id = a->k_id; a->k_id = (uint8_t)((a->k_id >= 255U) ? 1U : (a->k_id + 1U)); return id; }

static void vcal_message(MApp_t *a, uint8_t status, int32_t value)
{
    char l2[21]; uint32_t n = 0U; const uint32_t now = a->t_now; const char *ch = (a->vcal_ch == 0U) ? "I/P" : "O/P";
    switch (status)
    {
    case 0U:
        if (a->vcal_cmd == MAPP_KCMD_CALR) { l2[0] = '\0'; MUI_Notify(&a->ui, "Reset To Nominal", ch, now); break; }
        {   /* the total correction against nominal, 10000 = 0 %: "I/P +1.52 %" */
            int32_t d = value - 10000; uint32_t ad = (uint32_t)((d < 0) ? -d : d);
            l2[n++] = ch[0]; l2[n++] = ch[1]; l2[n++] = ch[2]; l2[n++] = ' '; l2[n++] = (d < 0) ? '-' : '+';
            if ((ad / 100U) >= 10U) l2[n++] = (char)('0' + ((ad / 1000U) % 10U));
            l2[n++] = (char)('0' + ((ad / 100U) % 10U)); l2[n++] = '.'; l2[n++] = (char)('0' + ((ad / 10U) % 10U)); l2[n++] = (char)('0' + (ad % 10U)); l2[n++] = ' '; l2[n++] = '%'; l2[n] = '\0';
            MUI_Notify(&a->ui, "Calibrated", l2, now);
        }
        break;
    case 1U: MUI_Notify(&a->ui, "Bad Value", "50.0 - 400.0 V", now); break;
    case 2U: MUI_Notify(&a->ui, "No Steady Signal", "Min 50 V", now); break;
    case 3U: MUI_Notify(&a->ui, "Out Of Range", "Max +/-10 %", now); break;
    case 4U: MUI_Notify(&a->ui, "Stop The Unit", "First", now); break;
    case 5U:
    {   /* FLASH_FAIL: the value says where: stage x 10,000,000 + reason x 1,000,000 + dual-bank flag x 100,000 + 16 bits of HAL error flags: "S2 R3 D1 E0080" */
        static const char HX[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
        if (value > 0)
        {
            uint32_t v = (uint32_t)value, e = v % 100000UL;
            l2[n++] = 'S'; l2[n++] = (char)('0' + ((v / 10000000UL) % 10U)); l2[n++] = ' '; l2[n++] = 'R'; l2[n++] = (char)('0' + ((v / 1000000UL) % 10U)); l2[n++] = ' ';
            l2[n++] = 'D'; l2[n++] = (char)('0' + ((v / 100000UL) % 10U)); l2[n++] = ' '; l2[n++] = 'E'; l2[n++] = HX[(e >> 12) & 15U]; l2[n++] = HX[(e >> 8) & 15U]; l2[n++] = HX[(e >> 4) & 15U]; l2[n++] = HX[e & 15U]; l2[n] = '\0';
            MUI_Notify(&a->ui, "Save Failed", l2, now);
        }
        else MUI_Notify(&a->ui, "Save Failed", "Flash Error", now);
        break;
    }
    case 6U: MUI_Notify(&a->ui, "Not Supported", "Update The Slave", now); break;
    case 7U: MUI_Notify(&a->ui, "Slave Busy", "Try Again", now); break;
    default: MUI_Notify(&a->ui, "Calibration", "Failed", now); break;
    }
}

/* LOG on a Voltage Calibration page */
static void vcal_request(MApp_t *a, const MAppData_t *d, uint32_t now)
{
    uint16_t entered = MUI_CalValue(&a->ui); uint8_t ch = (a->ui.page == 6U) ? 1U : 0U;
    if (a->vcal_wait != 0U) { MUI_Notify(&a->ui, "Busy", "Wait For The Answer", now); return; }
    if (d->have == 0U) { MUI_Notify(&a->ui, "Slave Link Lost", "", now); return; }
    if ((a->eout.run[0] != 0U) || (d->state != 0U)) { MUI_Notify(&a->ui, "Stop The Unit", "First", now); return; }          /* never while it regulates: a gain change would make the output jump */
    a->vcal_ch = ch;
    if (entered == 0U) { a->vcal_cmd = MAPP_KCMD_CALR; a->vcal_letter = (ch == 0U) ? 'V' : 'O'; a->vcal_num = 0; }          /* typing 000.0 resets the channel to nominal */
    else if ((entered < 500U) || (entered > 4000U)) { MUI_Notify(&a->ui, "Enter 50.0-400.0 V", "Or 000.0 To Reset", now); return; }
    else { a->vcal_cmd = (ch == 0U) ? MAPP_KCMD_CALV : MAPP_KCMD_CALO; a->vcal_letter = '\0'; a->vcal_num = (int32_t)entered; }
    a->vcal_id = MApp_NextKId(a); a->vcal_tx_n = 0U; a->vcal_t0 = now; a->vcal_t_tx = now; a->vcal_due = 1U; a->vcal_wait = 1U;
    MUI_Notify(&a->ui, "Calibrating", "Wait...", now);
}

static void vcal_service(MApp_t *a, uint32_t now)
{
    if (a->vcal_wait == 0U) return;
    if ((a->vcal_due == 0U) && (a->vcal_tx_n > 0U) && (a->vcal_tx_n < 3U) && ((uint32_t)(now - a->vcal_t_tx) >= MAPP_VCAL_RETRY_MS)) a->vcal_due = 1U;       /* the same id again */
    if ((uint32_t)(now - a->vcal_t0) >= MAPP_VCAL_TIMEOUT_MS) { a->vcal_wait = 0U; a->vcal_due = 0U; MUI_Notify(&a->ui, "No Answer", "From The Slave", now); }
}

uint8_t MApp_TakeKCommand(MApp_t *a, MCalK_t *out, uint32_t now)
{
    if ((a->tune_due != 0U) && (a->tune_state != TS_IDLE))
    {   /* a TUNE or TABORT (re)send is due */
        out->id = a->tune_id; out->cmd = a->tune_cmd; out->num = 0; out->letter = '\0'; a->tune_due = 0U; a->tune_tx_n++; a->tune_t_tx = now; return 1U;
    }
    if ((a->vcal_wait == 0U) || (a->vcal_due == 0U)) return 0U;
    out->id = a->vcal_id; out->cmd = a->vcal_cmd; out->num = a->vcal_num; out->letter = a->vcal_letter;
    a->vcal_due = 0U; a->vcal_tx_n++; a->vcal_t_tx = now;
    return 1U;
}

void MApp_OnKResult(MApp_t *a, uint8_t id, uint8_t status, int32_t value)
{
    if ((a->tune_state != TS_IDLE) && (id == a->tune_id)) { tune_on_result(a, status, value, a->t_now); return; }       /* the answer to TUNE or TABORT */
    if ((a->vcal_wait == 0U) || (id != a->vcal_id)) return;                       /* not the answer to a calibration (a HELLO's, or a late one) */
    a->vcal_wait = 0U; a->vcal_due = 0U;
    vcal_message(a, status, value);
}

uint8_t MApp_ShowsIout(const MApp_t *a) { return MUI_ShowsIout(&a->ui); }

/* The converter-side current page (LOG, then MENU x7): what the slave's CT2500 chain (net I-OUT) measures, to check the scaling against a clamp meter on the same conductor. It is the CT PRIMARY current. */
void MApp_IoutPage(const MApp_t *a, uint32_t now, char frame[MUI_ROWS][MUI_COLS + 1U])
{
    const MAppData_t *d = &a->last; uint32_t r, c, n; char t[24]; (void)now;
    for (r = 0U; r < MUI_ROWS; r++) { for (c = 0U; c < MUI_COLS; c++) frame[r][c] = ' '; frame[r][MUI_COLS] = '\0'; }
    copy(t, "Converter Current"); for (n = 0U; t[n] != '\0'; n++) frame[0][1U + n] = t[n];
    if (d->iout_valid != 0U)
    {
        uint32_t rms = (d->iout_rms_ca > 9999U) ? 9999U : d->iout_rms_ca, pk = (d->iout_pk_ca > 9999U) ? 9999U : d->iout_pk_ca;
        copy(t, "RMS   00.00 A"); t[6] = (char)('0' + ((rms / 1000U) % 10U)); t[7] = (char)('0' + ((rms / 100U) % 10U)); t[9] = (char)('0' + ((rms / 10U) % 10U)); t[10] = (char)('0' + (rms % 10U)); if (t[6] == '0') t[6] = ' ';
        for (n = 0U; t[n] != '\0'; n++) frame[1][n] = t[n];
        copy(t, "Peak  00.00 A"); t[6] = (char)('0' + ((pk / 1000U) % 10U)); t[7] = (char)('0' + ((pk / 100U) % 10U)); t[9] = (char)('0' + ((pk / 10U) % 10U)); t[10] = (char)('0' + (pk % 10U)); if (t[6] == '0') t[6] = ' ';
        for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n];
        copy(t, (d->iout_clip != 0U) ? "OVER RANGE (clipped)" : "CT primary current");
    }
    else { copy(t, "RMS    --.-- A"); for (n = 0U; t[n] != '\0'; n++) frame[1][n] = t[n]; copy(t, "Peak   --.-- A"); for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n]; copy(t, "Not measured"); }
    for (n = 0U; (t[n] != '\0') && (n < MUI_COLS); n++) frame[3][n] = t[n];
}

uint8_t MApp_ShowsClock(const MApp_t *a) { return MUI_ShowsClock(&a->ui); }

/* the Clock service page (LOG, then MENU x4): what the real-time clock is doing, to find out why a fault carries no time */
void MApp_ClockPage(const MApp_t *a, uint32_t now, char frame[MUI_ROWS][MUI_COLS + 1U])
{
    const MAppData_t *d = &a->last; char t[24]; uint32_t r, c, n; static const char HX[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
    for (r = 0U; r < MUI_ROWS; r++) { for (c = 0U; c < MUI_COLS; c++) frame[r][c] = ' '; frame[r][MUI_COLS] = '\0'; }
    (void)now;
    if (d->clock_ok != 0U)
    {   /* row 0: the date, row 1: the time */
        copy(t, "Date 00/00/0000"); put2(&t[5], d->clock.day); put2(&t[8], d->clock.month); t[11] = (char)('0' + ((d->clock.year / 1000U) % 10U)); t[12] = (char)('0' + ((d->clock.year / 100U) % 10U)); t[13] = (char)('0' + ((d->clock.year / 10U) % 10U)); t[14] = (char)('0' + (d->clock.year % 10U));
        for (n = 0U; t[n] != '\0'; n++) frame[0][n] = t[n];
        copy(t, "Time 00:00"); put2(&t[5], d->clock.hour); put2(&t[8], d->clock.minute);
        for (n = 0U; t[n] != '\0'; n++) frame[1][n] = t[n];
    }
    else { copy(t, "Clock NOT SET"); for (n = 0U; t[n] != '\0'; n++) frame[0][n] = t[n]; copy(t, "(Set-Mode: Date, Time)"); for (n = 0U; (t[n] != '\0') && (n < MUI_COLS); n++) frame[1][n] = t[n]; }
    /* row 2: the clock source the backup domain reports, and whether the reading counts as set */
    copy(t, "Src:");
    { const char *sn = (d->rtc_src == 1U) ? "Crystal" : ((d->rtc_src == 2U) ? "LSI !!!" : "-------"); for (n = 0U; n < 7U; n++) t[4U + n] = sn[n]; }
    t[11] = ' '; t[12] = 'S'; t[13] = 'e'; t[14] = 't'; t[15] = ':'; t[16] = (d->clock_ok != 0U) ? 'Y' : 'N'; t[17] = '\0';
    for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n];
    /* row 3: the raw registers, for me: RCC BDCR and RTC ICSR (low 16 bits each) */
    copy(t, "BD:0000 IC:0000");
    t[3] = HX[(d->rtc_bdcr >> 12) & 15U]; t[4] = HX[(d->rtc_bdcr >> 8) & 15U]; t[5] = HX[(d->rtc_bdcr >> 4) & 15U]; t[6] = HX[d->rtc_bdcr & 15U];
    t[11] = HX[(d->rtc_icsr >> 12) & 15U]; t[12] = HX[(d->rtc_icsr >> 8) & 15U]; t[13] = HX[(d->rtc_icsr >> 4) & 15U]; t[14] = HX[d->rtc_icsr & 15U];
    for (n = 0U; t[n] != '\0'; n++) frame[3][n] = t[n];
}
const MFOut_t *MApp_Engine(const MApp_t *a) { return &a->eout; }
uint8_t MApp_ShowsMonitor(const MApp_t *a) { return MUI_ShowsMonitor(&a->ui); }
