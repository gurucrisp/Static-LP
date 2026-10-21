/** @file m_app.c  @brief The application: settings, log, UI and keys together. See m_app.h. */
#include "m_app.h"



/* The display value of a reading (tenths): a first-order filter (about 0.6 s) that jumps at once on a step of 5 V or more, then a hysteresis on what is SHOWN so the last digit does not flicker
 * between two values: the shown value changes only when the filtered reading is `band` tenths away from it. `step` is the size of one displayed unit in tenths (10 = whole volts, 1 = tenths). */
static uint16_t disp_filter(void *fp, uint16_t raw, uint8_t have, int32_t step, int32_t band)
{
    struct { int32_t ema; int32_t shown; uint8_t init; } *f = fp;
    int32_t r = (int32_t)raw * 256, e, diff;
    if (have == 0U) { f->init = 0U; return raw; }
    if (f->init == 0U) { f->ema = r; f->shown = (((int32_t)raw + (step / 2)) / step) * step; f->init = 1U; return (uint16_t)f->shown; }
    diff = r - f->ema;
    if ((diff > (50 * 256)) || (diff < -(50 * 256))) f->ema = r;                 /* a real step: follow it at once */
    else f->ema += diff / 64;
    e = (f->ema + 128) / 256;
    if (((e - f->shown) >= band) || ((f->shown - e) >= band)) f->shown = ((e + (step / 2)) / step) * step;
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
    cal_defaults(&a->cal);                                                  /* until MApp_InitCal: the original firmware's scale and 3 A offset */                                    /* power-up: Auto waits the Restart Time, Manual waits for START */
    MUI_Init(&a->ui, &a->settings, &a->log, now);
    MUI_SetLimits(&a->ui, 0);                                              /* the slave's window and output guard come with its $H frame (stage 7c) */
    MUI_SetFeatures(&a->ui, 0U);                                           /* calibration and re-tune are not available yet */
}

void MApp_Step(MApp_t *a, uint8_t menu, uint8_t start, uint8_t stop, uint8_t log, const MAppData_t *d, uint32_t now, char frame[MUI_ROWS][MUI_COLS + 1U])
{
    MUIRun_t r; uint8_t *p = (uint8_t *)&r; uint32_t i; uint8_t ph; uint16_t ld; uint8_t lok;
    MUI_Key(&a->ui, MK_Update(&a->keys, menu, start, stop, log, now), now);

    for (i = 0U; i < (uint32_t)sizeof(r); i++) p[i] = 0U;
    for (ph = 0U; ph < 3U; ph++) { r.have[ph] = 0U; }
    r.have[0] = d->have; r.freq_chz = d->freq_chz;
    r.vin_dv[0]  = disp_filter(&a->f_vin,  d->vin_dv,  d->have, 10, 7);      /* whole volts, changes only when the filtered value is 0.7 V away */
    r.vout_dv[0] = disp_filter(&a->f_vout, d->vout_dv, d->have, 10, 7);
    ld = d->load_da; lok = d->load_ok;
    if (d->ct_raw != 0U) { ld = ct_to_da(&a->cal, d->ct_x16); lok = 1U; }                      /* from the CT count and the calibration */
    r.load_ok[0] = lok; r.load_da[0] = disp_filter(&a->f_load, ld, (uint8_t)(d->have && lok), 1, 3);   /* tenths of an amp, 0.3 A band */

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
        in.ph[0].heat = d->heat; in.ph[0].ct_peak = d->ct_peak;
        in.ph[0].slave_state = d->state; in.ph[0].state_valid = d->have;                    /* PWM off = load off */
        /* the Y and B slaves have no link yet: in Industrial and Domestic they count as missing, so those modes do not start */
        in.setmode = MUI_SetModeOpen(&a->ui); in.key_start = k_start; in.key_stop = k_stop; in.run_min = now / 60000UL;
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
    MUI_Tick(&a->ui, &r, now, frame);

    if (MUI_Take(&a->ui, MUI_REQ_SAVE))
    {
        if (MS_Save(&a->sstore, &a->settings) != 0) { a->save_failures++; MUI_Notify(&a->ui, "Save Failed", "Flash Error", now); }
        else a->settings_from_flash = 1U;
    }
    if (MUI_Take(&a->ui, MUI_REQ_CLEAR_LOG)) (void)MLog_Clear(&a->log);
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
    (void)MUI_Take(&a->ui, MUI_REQ_RETUNE);
}

uint8_t MApp_SetModeOpen(const MApp_t *a) { return MUI_SetModeOpen(&a->ui); }
const MCtlOut_t *MApp_Control(const MApp_t *a) { return &a->ctl_out; }
uint8_t MApp_ControlValid(const MApp_t *a) { return a->ctl_valid; }
void MApp_NoteFrameSent(MApp_t *a) { a->m_sent++; }
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
    for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n];
    /* row 3: the slave's own state, fault code and IN_BAND */
    copy(t, "Slv st:0 F:00 inb:0"); t[7] = (char)('0' + (d->state % 10U)); put2(&t[11], d->fault); t[18] = (char)('0' + (d->in_band & 1U));
    if ((d->wait_valid != 0U) && ((d->wait_ctx & 0x7FU) != 0U)) { char w[10]; copy(&t[14], "W:"); (void)wait_letters((uint8_t)(d->wait_ctx & 0x7FU), w); { uint32_t i = 0U; while ((w[i] != '\0') && (16U + i < 20U)) { t[16U + i] = w[i]; i++; } t[16U + i] = '\0'; } }
    for (n = 0U; t[n] != '\0'; n++) frame[3][n] = t[n];
}
const MFOut_t *MApp_Engine(const MApp_t *a) { return &a->eout; }
uint8_t MApp_ShowsMonitor(const MApp_t *a) { return MUI_ShowsMonitor(&a->ui); }
