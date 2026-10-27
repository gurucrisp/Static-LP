/** @file m_app.c  @brief The application: settings, log, UI and keys together. See m_app.h. */
#include "m_app.h"
#include "m_ntc.h"



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
/* the load current in tenths of an amp from the slave's CONVERTER current: load = (peak - zero) x gain. The PEAK is used: the converter's RMS hardly moves with the load (0.9 A RMS at 10 A of load, and as much at no load), the peak does (7.8 A at 10 A).
 * The zero is the peak at no load WITH THE UNIT RUNNING (the converter's own magnetizing current), the gain is load / (peak - zero) at a known load. */
static uint16_t sl_to_da(uint16_t zero_ca, uint16_t gain, uint16_t pk_ca)
{
    int32_t x = (int32_t)pk_ca - (int32_t)zero_ca; uint64_t v;
    if (x < 0) x = 0;
    v = (((uint64_t)(uint32_t)x * (uint64_t)gain) + 5000ULL) / 10000ULL;                         /* hundredths of an amp x thousandths / 10000 = tenths of an amp */
    return (v > 6000ULL) ? 6000U : (uint16_t)v;
}
/* v0.9.55: the load of the Y (k 0) or B (k 1) phase: they have no master CT, so it is always their slave's converter current with THEIR calibration. 0 = no load reading (not calibrated, not measured, clipped): shown as --.-A, no overload trip */
static uint8_t yb_load(const MApp_t *a, const MAppSlave_t *s, uint8_t k, uint16_t *ld)
{
    *ld = 0U;
    if ((a->settings.sl_gain_yb[k] == 0U) || (s->have == 0U) || (s->iout_valid == 0U) || (s->iout_clip != 0U)) return 0U;
    *ld = sl_to_da(a->settings.sl_zero_yb[k], a->settings.sl_gain_yb[k], s->iout_pk_ca); return 1U;
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
    MUI_SetFeatures(&a->ui, (uint8_t)(MUI_FEAT_CALIBRATE | MUI_FEAT_RETUNE | ((a->no_ct != 0U) ? MUI_FEAT_NO_CT : 0U)));            /* calibration is available now; Re-tune stays available (MUI_SetFeatures ASSIGNS: it must not drop the flag MApp_Init set) */
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
    if ((a->seq_wait != 0U) && ((u->st == U_OFF) || (u->st == U_ONDELAY))) { copy(o, "Checking Phase Seq"); return; }      /* Industrial with Phase Sequence on: the sequence is measured before a start */
    if ((a->settings.mode == (uint8_t)M_MODE_INDUSTRIAL) && (a->settings.phase_seq != 0U) && ((a->phase.state == MPH_REVERSE) || (a->phase.state == MPH_SAME)) && ((u->st == U_OFF) || (u->st == U_ONDELAY)))
    {   /* v0.9.54: a wrong sequence measured BEFORE the start holds it: say so (it used to show a frozen "Start in 05 s") */
        copy(o, (a->phase.state == MPH_SAME) ? "Phase Seq Same Ph" : "Phase Seq Reverse"); return;
    }
    switch (u->st)
    {
    case U_OFF:      copy(o, "Press START"); break;
    case U_ONDELAY:  rem = (age_ms(now, u->t_enter) >= rd) ? 0U : ((rd - age_ms(now, u->t_enter) + 999UL) / 1000UL); copy(o, "Start in 00 s"); put2(&o[9], rem); break;
    case U_STARTING:
        if ((a->settings.mode == (uint8_t)M_MODE_SINGLE) && (d->wait_valid != 0U) && ((d->wait_ctx & 0x7FU) != 0U))
        {   /* the slave was told to run but is waiting for something: SAY WHAT (letters, see wait_letters). v0.9.57: Single only (three phases: it only showed R's, and its start-up "R") */
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
    a->retries_cfg = (uint8_t)MF_RETRIES; a->ntc_heat = 1U;                       /* the heat-sink temperature trips the unit by default (MApp_SetNtcHeat) */
    cal_defaults(&a->cal);                                                  /* until MApp_InitCal: the original firmware's scale and 3 A offset */
    a->k_id = 1U;                                    /* power-up: Auto waits the Restart Time, Manual waits for START */
    MUI_Init(&a->ui, &a->settings, &a->log, now);
    MUI_SetLimits(&a->ui, 0);                                              /* the slave's window and output guard come with its $H frame (stage 7c) */
    MUI_SetFeatures(&a->ui, (uint8_t)(MUI_FEAT_RETUNE | ((a->no_ct != 0U) ? MUI_FEAT_NO_CT : 0U)));                              /* Re-tune starts a transformer sweep (behind the Set-Mode password; a slave that cannot answers "Not Supported"); calibration is added by MApp_InitCal */
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
    if ((d->ct_max_x16 != 0U) && (a->settings.load_src != MS_LOADSRC_SLAVE_NOCT) && (a->no_ct == 0U))          /* v0.9.59: no master CT fitted: its input is noise, never a peak */
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

static void tune_refuse(MApp_t *a, const char *l1, const char *l2, uint32_t now) { a->tune_state = TS_IDLE; a->tune_all = 0U; MUI_Notify(&a->ui, l1, l2, now); }

/* The phase sequence rounds (v0.9.51), three-phase modes with all three slaves heard: ARM ($K SEQ <round>) on the three links, 20 ms later one SYNC byte on the three links at once, then the three slaves'
 * results come back in their $E frames (sq_id = the round, the time from the SYNC byte to their rising zero-cross, their period). MPhase decides after three identical rounds. */
static void seq_step(MApp_t *a, const MAppData_t *d, uint32_t now)
{
    uint8_t three = (uint8_t)(a->settings.mode != (uint8_t)M_MODE_SINGLE);
    uint8_t heard = (uint8_t)((d->have != 0U) && (d->yb[0].have != 0U) && (d->yb[1].have != 0U));
    if ((three == 0U) || (heard == 0U))
    {   /* a single-phase unit, or a slave is not heard: nothing to measure, and what was decided is gone */
        a->seq_fsm = 0U; a->seq_act = 0U; MPhase_Lost(&a->phase); a->t_seq_next = now + MAPP_SEQ_FAST_MS; return;
    }
    MPhase_Age(&a->phase, now);
    if (a->seq_fsm == 0U)
    {
        if ((int32_t)(now - a->t_seq_next) >= 0) { a->seq_round = (uint8_t)((a->seq_round % 255U) + 1U); a->seq_act = 1U; a->t_seq = now; a->seq_fsm = 1U; }
    }
    else if (a->seq_fsm == 1U)
    {
        if (age_ms(now, a->t_seq) >= MAPP_SEQ_SYNC_DELAY_MS) { a->seq_act = 2U; a->t_seq = now; a->seq_fsm = 2U; }
    }
    else
    {
        uint16_t id[3], dl[3], pd[3]; MPhRound_t r; uint32_t i, got = 0U;
        id[0] = d->sq_id; dl[0] = d->sq_delay_us; pd[0] = d->sq_period_us;
        id[1] = d->yb[0].sq_id; dl[1] = d->yb[0].sq_delay_us; pd[1] = d->yb[0].sq_period_us;
        id[2] = d->yb[1].sq_id; dl[2] = d->yb[1].sq_delay_us; pd[2] = d->yb[1].sq_period_us;
        for (i = 0U; i < 3U; i++)
        {
            uint8_t mine = (uint8_t)(id[i] == (uint16_t)a->seq_round);
            if (mine != 0U) got++;
            r.valid[i] = (uint8_t)((mine != 0U) && (dl[i] != (uint16_t)MPH_NONE_US)); r.delay_us[i] = dl[i]; r.period_us[i] = pd[i];
        }
        if ((got == 3U) || (age_ms(now, a->t_seq) >= MAPP_SEQ_TIMEOUT_MS))
        {
            MPhase_Round(&a->phase, &r, now);
            a->t_seq_next = now + ((got == 0U) ? MAPP_SEQ_SILENT_MS : ((a->phase.state == MPH_UNKNOWN) ? MAPP_SEQ_FAST_MS : MAPP_SEQ_SLOW_MS));
            a->seq_fsm = 0U;
        }
    }
}
uint8_t MApp_TakeSeqAction(MApp_t *a, uint8_t *round) { uint8_t act = a->seq_act; a->seq_act = 0U; *round = a->seq_round; return act; }
const MPhase_t *MApp_Phase(const MApp_t *a) { return &a->phase; }

/* The heat-sink temperature of one phase (v0.9.49): the slave's raw reading is filtered (1/16 per step), converted, and turned into the overheat signal the engine takes
 * (its Overheat Trip Delay, the restart when cooled and the three-trips rule are the engine's): asserted at the Overheat Temp, released at the Overheat Restart (a setting, 35 to Overheat Temp - 5, default 10 degC below).
 * A SHORT reading counts as overheat (fail-safe); an OPEN one (no NTC fitted, a broken wire) is shown but never trips. No fresh reading: no signal. */
static void ntc_update(MApp_t *a, uint8_t ph, uint8_t valid, uint16_t raw)
{
    int16_t t; int32_t trip = (int32_t)a->settings.heat_temp * 10, rst = (int32_t)a->settings.heat_reset * 10;
    if (valid == 0U) { a->ntc_have[ph] = 0U; a->heat_on[ph] = 0U; a->temp_st[ph] = 255U; return; }
    if (a->ntc_have[ph] == 0U) { a->ntc_f16[ph] = (uint32_t)raw * 16U; a->ntc_have[ph] = 1U; }
    else a->ntc_f16[ph] = (uint32_t)((int32_t)a->ntc_f16[ph] + (((int32_t)raw * 16 - (int32_t)a->ntc_f16[ph]) / 16));
    a->temp_st[ph] = MNtc_RawToDeci((uint16_t)((a->ntc_f16[ph] + 8U) / 16U), &t); a->temp_dc[ph] = t;
    if (a->ntc_heat == 0U) { a->heat_on[ph] = 0U; return; }
    if (a->temp_st[ph] == MNTC_SHORT) a->heat_on[ph] = 1U;
    else if (a->temp_st[ph] == MNTC_OPEN) a->heat_on[ph] = 0U;
    else if ((int32_t)t >= trip) a->heat_on[ph] = 1U;
    else if ((int32_t)t <= rst) a->heat_on[ph] = 0U;                                              /* released at the Overheat Restart (Set-Mode, at most the Overheat Temp - 5) */
}
void MApp_SetNtcHeat(MApp_t *a, uint8_t on) { a->ntc_heat = on; }
uint8_t MApp_TempDeci(const MApp_t *a, uint8_t ph, int16_t *t_dc)
{
    if ((ph > 2U) || (a->ntc_have[ph] == 0U)) { *t_dc = 0; return 255U; }
    *t_dc = a->temp_dc[ph]; return a->temp_st[ph];
}

/* The Slave Load Cal page, LOG (v0.9.45). With the unit RUNNING: a typed 00.0 = "no load now": the converter PEAK now is the zero; a typed load = "this is the load now": the gain. Both saved with the settings. */
static void slcal_request(MApp_t *a, const MAppData_t *d, uint32_t now)
{
    uint32_t entered = MUI_CalValue(&a->ui);                                           /* tenths of an amp */
    uint8_t cp = MUI_CalPhase(&a->ui);                                                 /* v0.9.55: R, Y or B: its slave and its own zero and gain */
    uint16_t *zp = (cp == 0U) ? &a->settings.sl_zero_ca : &a->settings.sl_zero_yb[cp - 1U], *gp = (cp == 0U) ? &a->settings.sl_gain : &a->settings.sl_gain_yb[cp - 1U];
    uint16_t keep_z = *zp, keep_g = *gp, pk; uint8_t have, valid, clip, st;
    if (cp == 0U) { have = d->have; valid = d->iout_valid; clip = d->iout_clip; st = d->state; pk = d->iout_pk_ca; }
    else { const MAppSlave_t *s = &d->yb[cp - 1U]; have = s->have; valid = s->iout_valid; clip = s->iout_clip; st = s->state; pk = s->iout_pk_ca; }
    if ((have == 0U) || (valid == 0U)) { MUI_Notify(&a->ui, "No Converter", "Current Reading", now); return; }
    if (st != 2U) { MUI_Notify(&a->ui, "Run The Unit", "First", now); return; }
    if (clip != 0U) { MUI_Notify(&a->ui, "Reading Clipped", "Reduce The Load", now); return; }
    if (a->sstore.fl == 0) { MUI_Notify(&a->ui, "Calibration", "Not Available", now); return; }
    if (entered == 0U)
    {   /* THE ZERO: the peak at no load */
        if (pk > 4000U) { MUI_Notify(&a->ui, "Out Of Range", "Open The Load", now); return; }
        *zp = pk;
        if (MS_Save(&a->sstore, &a->settings) == 0) MUI_Notify(&a->ui, "Zero Set", "Now The Gain", now);
        else { *zp = keep_z; MUI_Notify(&a->ui, "Save Failed", "Flash Error", now); }
        return;
    }
    {   /* THE GAIN: the typed load against the peak above the zero */
        uint32_t delta = (pk > *zp) ? (uint32_t)(pk - *zp) : 0U, g;
        if (delta < MAPP_SL_MIN_DELTA_CA)
        {   /* v0.9.58: say the two numbers, so a zero taken with the load on, a load on another phase or a converter that hardly works at this input can be told apart: "Pk 2.80 Zero 2.50" */
            char t[21]; uint32_t n = 0U; uint16_t v[2]; uint32_t k; v[0] = (pk > 9999U) ? 9999U : pk; v[1] = (*zp > 9999U) ? 9999U : *zp;
            for (k = 0U; k < 2U; k++)
            {   const char *lb = (k == 0U) ? "Pk " : " Zero "; uint32_t i = 0U, w = v[k] / 100U;
                while (lb[i] != '\0') t[n++] = lb[i++];
                if (w >= 10U) t[n++] = (char)('0' + (w / 10U));
                t[n++] = (char)('0' + (w % 10U)); t[n++] = '.'; t[n++] = (char)('0' + ((v[k] / 10U) % 10U)); t[n++] = (char)('0' + (v[k] % 10U));
            }
            t[n] = '\0'; MUI_Notify(&a->ui, "Load Too Low", t, now); return;
        }
        if ((entered < MAPP_CT_MIN_CAL_DA) || (entered > 1800U)) { MUI_Notify(&a->ui, "Enter 2.0-180.0 A", "", now); return; }
        g = (entered * 10000U + (delta / 2U)) / delta;
        if ((g < 200U) || (g > 5000U)) { MUI_Notify(&a->ui, "Out Of Range", "Check The Load", now); return; }
        *gp = (uint16_t)g;
        if (MS_Save(&a->sstore, &a->settings) == 0) MUI_Notify(&a->ui, "Calibrated", "Saved", now);
        else { *gp = keep_g; MUI_Notify(&a->ui, "Save Failed", "Flash Error", now); }
    }
}

/* a $K id for the Y or B link (ph 1 or 2): 128..255 (master_links uses 1..127 for its hello and SEQ), and never the id of the application's last command on that link (the slave would take it for a retry) */
static uint8_t yb_kid(MApp_t *a, uint8_t ph)
{
    uint8_t id = (uint8_t)(MApp_NextKId(a) | 0x80U);
    if (id == a->tune_id_yb[ph - 1U]) id = (uint8_t)(0x80U | (uint8_t)(id + 1U));
    a->tune_id_yb[ph - 1U] = id; return id;
}

/* v0.9.54: what the slave of phase ph says about its Re-tune (the R slave's fields are the top level of MAppData_t, the Y and B slaves' are in yb[]) */
typedef struct { uint8_t have, state, fault, wait_valid, wait_ctx, tune_n, tune_step, tune_result; uint16_t tune_ticks; int16_t tune_dv_dv; } TuneSrc_t;
static void tune_src(const MAppData_t *d, uint8_t ph, TuneSrc_t *t)
{
    if ((ph == 1U) || (ph == 2U))
    {   const MAppSlave_t *s = &d->yb[ph - 1U];
        t->have = s->have; t->state = s->state; t->fault = s->fault; t->wait_valid = s->wait_valid; t->wait_ctx = s->wait_ctx;
        t->tune_n = s->tune_n; t->tune_step = s->tune_step; t->tune_result = s->tune_result; t->tune_ticks = s->tune_ticks; t->tune_dv_dv = s->tune_dv_dv; return; }
    t->have = d->have; t->state = d->state; t->fault = d->fault; t->wait_valid = d->wait_valid; t->wait_ctx = d->wait_ctx;
    t->tune_n = d->tune_n; t->tune_step = d->tune_step; t->tune_result = d->tune_result; t->tune_ticks = d->tune_ticks; t->tune_dv_dv = d->tune_dv_dv;
}

/* ph: 0 R, 1 Y, 2 B (Single: always 0). ONE phase at a time: the units are held off while a sweep runs (the engine's setmode) */
static void tune_request(MApp_t *a, const MAppData_t *d, uint8_t ph, uint32_t now)
{
    TuneSrc_t t;
    if (a->tune_state != TS_IDLE) return;
    if ((ph > 2U) || ((ph != 0U) && (a->settings.mode == (uint8_t)M_MODE_SINGLE))) ph = 0U;
    tune_src(d, ph, &t); a->tune_ph = ph;
    if (t.have == 0U) { a->tune_all = 0U; MUI_Notify(&a->ui, "Slave Link Lost", (ph == 1U) ? "Y Phase" : ((ph == 2U) ? "B Phase" : ""), now); return; }
    if ((a->eout.run[ph] != 0U) || ((t.state != 0U) && !((t.state == 3U) && ((t.fault == 18U) || (t.fault == 19U)))))      /* the slave latched "Re-tune Required" (18, 19): that is exactly what a tune is for */
    { a->tune_all = 0U; MUI_Notify(&a->ui, "Stop The Unit", "First", now); return; }
    a->tune_state = TS_ASK; a->tune_id = MApp_NextKId(a); a->tune_cmd = (uint8_t)MAPP_KCMD_TUNE; a->tune_due = 1U; a->tune_tx_n = 0U; a->tune_t0 = now; a->tune_t_tx = now;
    if (ph != 0U) a->tune_id = yb_kid(a, ph);
    a->tune_seen = t.tune_n; a->tune_step = 0U; a->tune_dv_dv = 0; a->tune_ticks = 0U; a->tune_result = 0U; a->tune_abort_sent = 0U;           /* frames already counted belong to an older sweep */
}

static void tune_abort(MApp_t *a, uint32_t now)
{
    if ((a->tune_abort_sent != 0U) || (a->tune_state == TS_IDLE) || (a->tune_state == TS_DONE)) return;
    a->tune_abort_sent = 1U; a->tune_id = (a->tune_ph != 0U) ? yb_kid(a, a->tune_ph) : MApp_NextKId(a); a->tune_cmd = (uint8_t)MAPP_KCMD_TABORT; a->tune_due = 1U; a->tune_tx_n = 0U; a->tune_t_tx = now; a->tune_t0 = now;
}

static void tune_key(MApp_t *a, uint8_t ev, uint32_t now)
{
    if (a->tune_state == TS_DONE) { if (ev != (uint8_t)MK_NONE) { a->tune_state = TS_IDLE; a->tune_all = 0U; } return; }      /* any key dismisses the result (and ends All, one by one) */
    if (ev == (uint8_t)MK_STOP) tune_abort(a, now);
}

static void tune_service(MApp_t *a, const MAppData_t *d, uint32_t now)
{
    TuneSrc_t t; tune_src(d, a->tune_ph, &t);
    a->tune_sl_state = t.state; a->tune_sl_wv = t.wait_valid; a->tune_sl_wait = (uint8_t)(t.wait_ctx & 0x7FU); a->tune_sl_fault = t.fault;
    if (a->tune_state == TS_IDLE) return;
    if (a->tune_state == TS_DONE)
    {
        if ((a->tune_all != 0U) && (a->tune_result == 1U) && (a->tune_ph < 2U))
        {   /* All, one by one: this phase is done, the next one starts after MAPP_TUNE_NEXT_MS (never two at once) */
            if ((uint32_t)(now - a->tune_t_done) >= MAPP_TUNE_NEXT_MS) { a->tune_state = TS_IDLE; tune_request(a, d, (uint8_t)(a->tune_ph + 1U), now); }
            return;
        }
        a->tune_all = 0U;
        if ((uint32_t)(now - a->tune_t_done) >= MAPP_TUNE_RESULT_MS) a->tune_state = TS_IDLE;
        return;
    }
    /* a command is repeated with the same id until it is answered (3 sends, 200 ms apart) */
    if ((a->tune_due == 0U) && (a->tune_tx_n > 0U) && (a->tune_tx_n < 3U) && ((uint32_t)(now - a->tune_t_tx) >= MAPP_VCAL_RETRY_MS) && ((a->tune_state == TS_ASK) || (a->tune_abort_sent != 0U))) a->tune_due = 1U;
    if ((a->tune_state == TS_ASK) && ((uint32_t)(now - a->tune_t0) >= MAPP_VCAL_TIMEOUT_MS)) { tune_refuse(a, "No Answer", "From The Slave", now); return; }
    if (a->tune_state == TS_RUN)
    {
        if (t.tune_n != a->tune_seen)
        {   /* a new $T frame */
            a->tune_seen = t.tune_n; a->tune_t_frame = now;
            if (t.tune_result == 0U)
            {   /* a step frame. Step 0 is the slave's HEARTBEAT ("still here"): it only keeps the timeout away, it must NOT send a sweep that is under way back to "Measuring baseline" */
                if (t.tune_step != 0U) { a->tune_step = t.tune_step; a->tune_ticks = t.tune_ticks; a->tune_dv_dv = t.tune_dv_dv; }
            }
            else
            {
                a->tune_result = t.tune_result; a->tune_step = t.tune_step; a->tune_dv_dv = t.tune_dv_dv; a->tune_state = TS_DONE; a->tune_t_done = now;
                if (t.tune_result == 1U) { MF_RetuneDone(&a->eng, a->tune_ph); if (a->tune_ph == 0U) a->hello_force = 1U; else a->hello_yb[a->tune_ph - 1U] = 1U; }      /* that phase's latch only, and ITS slave is asked for the new profile */
                return;
            }
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
    uint32_t r, c, n; char t[24]; static const char PH[3] = { 'R', 'Y', 'B' }; const uint8_t three = (uint8_t)(a->settings.mode != (uint8_t)M_MODE_SINGLE); (void)now;
    for (r = 0U; r < MUI_ROWS; r++) { for (c = 0U; c < MUI_COLS; c++) frame[r][c] = ' '; frame[r][MUI_COLS] = '\0'; }
    if (a->tune_state == TS_DONE)
    {
        const char *l0, *l1, *l2; char det[24];
        switch (a->tune_result)
        {
        case 1U: l0 = "Tuning Complete"; l1 = "Transformer OK"; l2 = "Profile Stored"; break;
        case 2U: l0 = "Tuning Failed"; l1 = "Reversed Transformer"; l2 = "Check Connections"; break;
        case 3U: l0 = "Tuning Failed"; l1 = "No Response"; l2 = "Transformer Missing?"; break;
        case 4U:
        {   /* the slave's final frame carries the reason in its injection field: > 0 the converter current rose (tenths of an ampere, 999 = the amplifier clipped), -1 there is no converter-current reading, 0 a fault or a flash error */
            int32_t v = (int32_t)a->tune_dv_dv; l0 = "Tuning Failed";
            if (v > 0)
            {
                uint32_t av = (uint32_t)v, q = 0U; const char *pre = "Converter ";
                l1 = "Current Too High";
                if (v >= 999) { l2 = "Converter at limit"; }
                else
                {
                    while (pre[q] != '\0') { det[q] = pre[q]; q++; }
                    if ((av / 100U) > 0U) { det[q++] = (char)('0' + ((av / 100U) % 10U)); }
                    if ((av / 10U) > 0U) { det[q++] = (char)('0' + ((av / 10U) % 10U)); } else { det[q++] = '0'; }
                    det[q++] = '.'; det[q++] = (char)('0' + (av % 10U)); det[q++] = ' '; det[q++] = 'A'; det[q] = '\0';
                    l2 = det;
                }
            }
            else if (v < 0) { l1 = "No Current Reading"; l2 = "Needs SLAVE_IOUT 1"; }
            else { l1 = "Aborted By Slave"; l2 = "Fault Or Flash Error"; }
            break;
        }
        case 5U:
        {   /* why: the slave's final frame carries it in the injection field: > 0 the input moved that much (tenths of a volt, peak to peak), < 0 the reading itself was noisy or had not settled (its spread), 0 the input left the slave's window */
            int32_t v = (int32_t)a->tune_dv_dv; uint32_t av = (uint32_t)((v < 0) ? -v : v), q = 0U, whole = av / 10U, frac = av % 10U; const char *pre = (v > 0) ? "Input moved " : "Reading spread ";
            l0 = "Tuning Failed"; l1 = "Input Not Steady";
            if (v == 0) { l2 = "Input out of window"; }
            else
            {
                while ((pre[q] != '\0') && (q < 15U)) { det[q] = pre[q]; q++; }
                if (whole >= 10U) { det[q++] = (char)('0' + ((whole / 10U) % 10U)); }
                det[q++] = (char)('0' + (whole % 10U));
                if (whole < 10U) { det[q++] = '.'; det[q++] = (char)('0' + frac); }
                det[q++] = ' '; det[q++] = 'V'; det[q] = '\0'; l2 = det;
            }
            break;
        }
        case 6U: l0 = "Tuning Stopped"; l1 = "By Operator"; l2 = "Nothing Stored"; break;
        default: l0 = "Tuning Failed"; l1 = "Slave Not Answering"; l2 = "Check The Link"; break;
        }
        copy(t, l0); for (n = 0U; t[n] != '\0'; n++) frame[0][n] = t[n];
        copy(t, l1); for (n = 0U; t[n] != '\0'; n++) frame[1][n] = t[n];
        copy(t, l2); for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n];
        copy(t, ((a->tune_all != 0U) && (a->tune_result == 1U) && (a->tune_ph < 2U)) ? ((a->tune_ph == 0U) ? "Next: Y Phase..." : "Next: B Phase...") : "Press any key"); for (n = 0U; t[n] != '\0'; n++) frame[3][n] = t[n];
        if (three != 0U) frame[0][19] = PH[a->tune_ph % 3U];                                   /* v0.9.54: which phase */
        return;
    }
    copy(t, "Transformer Tuning"); for (n = 0U; t[n] != '\0'; n++) frame[0][(three != 0U) ? n : (1U + n)] = t[n];
    if (three != 0U) frame[0][19] = PH[a->tune_ph % 3U];
    if ((a->tune_state == TS_ASK) || (a->tune_step == 0U))
    {
        copy(t, (a->tune_state == TS_ASK) ? "Starting..." : "Measuring baseline"); for (n = 0U; t[n] != '\0'; n++) frame[1][n] = t[n];
        /* WHAT THE SLAVE IS DOING: so that a start-up that does not progress says why (a normal start shows the same on the Run screen: "Slave Waits: Z") */
        if (a->tune_state == TS_ASK) copy(t, "Please wait");
        else if (a->tune_sl_state == 3U) { copy(t, "Slave Fault F00"); t[13] = (char)('0' + ((a->tune_sl_fault / 10U) % 10U)); t[14] = (char)('0' + (a->tune_sl_fault % 10U)); }
        else if (a->tune_sl_state == 2U) copy(t, "Slave In RUN");
        else if (a->tune_sl_state == 1U)
        {
            if ((a->tune_sl_wv != 0U) && (a->tune_sl_wait != 0U)) { char w[10]; copy(t, "Slave Waits: "); (void)wait_letters(a->tune_sl_wait, w); { uint32_t i = 0U; while ((w[i] != '\0') && (13U + i < 20U)) { t[13U + i] = w[i]; i++; } t[13U + i] = '\0'; } }
            else copy(t, "Slave Starting...");
        }
        else copy(t, "Slave Not Started");                   /* the slave was told to start but is still OFF: its converter and inverter pulses are not on */
        for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n];
    }
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
    if ((a->settings.load_src == MS_LOADSRC_SLAVE_NOCT) || (a->no_ct != 0U)) { ld = 0U; lok = 0U; }                  /* v0.9.59: no master CT: the slave's reading below, or none (--.-A, no overload trip) */
    if (((a->settings.load_src != MS_LOADSRC_CT) || (a->no_ct != 0U)) && (a->settings.sl_gain != 0U) && (d->have != 0U) && (d->iout_valid != 0U) && (d->iout_clip == 0U))
    {   /* "Load Source = Slave": the slave's converter current, calibrated. NOT calibrated, no reading or a clipped one: the master CT stays in charge (the overload trip must never depend on a reading that is not there). */
        ld = sl_to_da(a->settings.sl_zero_ca, a->settings.sl_gain, d->iout_pk_ca); lok = 1U;
    }
    r.load_ok[0] = lok; r.load_da[0] = disp_filter(&a->f_load, ld, (uint8_t)(d->have && lok), 1, 3, now);   /* tenths of an amp, 0.3 A band */
    for (ph = 0U; ph < 3U; ph++)
    {   /* the heat-sink temperature of each phase for the Run screens (v0.9.50) */
        int16_t tdc; uint8_t ts = MApp_TempDeci(a, ph, &tdc);
        r.temp_dc[ph] = tdc; r.temp_st[ph] = (uint8_t)((ts == 255U) ? 0U : ((ts == MNTC_OK) ? 1U : ((ts == MNTC_OPEN) ? 2U : 3U)));
        if ((r.temp_st[ph] != 0U) && (a->heat_on[ph] != 0U)) r.temp_st[ph] |= 0x10U;
    }
    for (ph = 0U; ph < 2U; ph++)
    {   /* the Y and B slaves (v0.9.48): the same readings and display filters as the R slave's */
        const MAppSlave_t *s = &d->yb[ph];
        r.have[1U + ph] = s->have;
        r.vin_dv[1U + ph]  = disp_filter(&a->f_vin_yb[ph],  s->vin_dv,  s->have, 10, 7, now);
        r.vout_dv[1U + ph] = disp_filter(&a->f_vout_yb[ph], s->vout_dv, s->have, 10, 7, now);
        { uint16_t yl; uint8_t yok = yb_load(a, s, ph, &yl); r.load_ok[1U + ph] = yok; r.load_da[1U + ph] = disp_filter(&a->f_load_yb[ph], yl, (uint8_t)(s->have && yok), 1, 3, now); }      /* v0.9.55: its slave, calibrated */
    }
    r.seq_state = (uint8_t)((a->phase.state == MPH_REVERSE) ? 1U : ((a->phase.state == MPH_SAME) ? 2U : 0U));
    r.vll_dv[0] = ((r.have[0] != 0U) && (r.have[1] != 0U)) ? MPhase_LineDv(&a->phase, 0U, 1U, r.vout_dv[0], r.vout_dv[1]) : 0U;      /* the phase-to-phase output voltages R-Y, Y-B, B-R from the three voltages and the angles the slaves measured */
    r.vll_dv[1] = ((r.have[1] != 0U) && (r.have[2] != 0U)) ? MPhase_LineDv(&a->phase, 1U, 2U, r.vout_dv[1], r.vout_dv[2]) : 0U;
    r.vll_dv[2] = ((r.have[2] != 0U) && (r.have[0] != 0U)) ? MPhase_LineDv(&a->phase, 2U, 0U, r.vout_dv[2], r.vout_dv[0]) : 0U;

    /* the slave's own limits (its $H frame): Set-Mode checks the I/P cut-offs against its window and O/P HV against its output guard (D7, D26) */
    if ((d->h_valid != 0U) && (a->lim_set == 0U))
    {
        a->lim.win_lo_v = (uint16_t)(d->win_lo_dv / 10U); a->lim.win_hi_v = (uint16_t)(d->win_hi_dv / 10U); a->lim.out_guard_v = (uint16_t)(d->vout_max_dv / 10U);
        MUI_SetLimits(&a->ui, &a->lim); a->lim_set = 1U;
    }
    /* the clear counter starts from the slave's own, so a master restart never looks like a "clear" */
    if ((d->e_valid != 0U) && (a->clr_synced == 0U)) { a->clr_cnt = d->clr_echo; a->clr_synced = 1U; }
    for (ph = 0U; ph < 2U; ph++) { const MAppSlave_t *s = &d->yb[ph]; if ((s->e_valid != 0U) && (a->clr_synced_yb[ph] == 0U)) { a->clr_cnt_yb[ph] = s->clr_echo; a->clr_synced_yb[ph] = 1U; } }

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
        seq_step(a, d, now);
        a->seq_wait = (uint8_t)((a->settings.mode == (uint8_t)M_MODE_INDUSTRIAL) && (a->settings.phase_seq != 0U) && (a->phase.state == MPH_UNKNOWN));
        in.seq_state = (uint8_t)((a->phase.state == MPH_REVERSE) ? 1U : ((a->phase.state == MPH_SAME) ? 2U : 0U)); in.seq_unknown = a->seq_wait;
        ntc_update(a, 0U, d->ntc_valid, d->ntc_raw);
        for (k = 0U; k < 2U; k++) ntc_update(a, (uint8_t)(1U + k), d->yb[k].ntc_valid, d->yb[k].ntc_raw);
        in.ph[0].heat = (uint8_t)((d->heat != 0U) || (a->heat_on[0] != 0U)); in.ph[0].ct_peak = (uint8_t)(d->ct_peak | a->ct_peak_on);
        in.ph[0].slave_state = d->state; in.ph[0].state_valid = d->have;                    /* PWM off = load off */
        for (k = 0U; k < 2U; k++)
        {   /* the Y and B slaves (v0.9.48): what the engine needs from each. A slave that is not heard counts as missing, as before. */
            const MAppSlave_t *s = &d->yb[k]; MPhaseIn_t *pi = &in.ph[1U + k];
            int32_t dvb = (int32_t)s->vout_dv - (int32_t)(a->settings.set_v * 10U);
            uint8_t meas = (uint8_t)((s->have != 0U) && (s->state == 2U) && (dvb <= (int32_t)MAPP_BAND_DV) && (dvb >= -(int32_t)MAPP_BAND_DV));
            pi->link_ok = s->link_ok; pi->vin_dv = s->vin_dv; pi->vout_dv = s->vout_dv; { uint16_t yl; pi->load_da = (yb_load(a, s, k, &yl) != 0U) ? yl : 0U; }      /* v0.9.55: the overload trip of Y and B, once calibrated */
            pi->slave_fault = (uint8_t)((s->have != 0U) ? s->fault : 0U);
            pi->in_band = (uint8_t)(((s->e_valid != 0U) && (s->in_band != 0U)) || (meas != 0U));
            pi->heat = (uint8_t)((s->heat != 0U) || (a->heat_on[1U + k] != 0U)); pi->ct_peak = 0U; pi->slave_state = s->state; pi->state_valid = s->have;
        }
        in.setmode = (uint8_t)((MUI_SetModeOpen(&a->ui) != 0U) || (a->tune_state != 0U)); in.key_start = k_start; in.key_stop = k_stop; in.run_min = (d->clock_ok != 0U) ? MC_Stamp(&d->clock) : a->run_min_total;       /* the log time: the real-time clock when it is set, else the running minutes */
        a->eng.max_retries = a->retries_cfg;                                           /* MF_Init (a mode change) resets it: apply it every step */
        {   MSettings_t es = a->settings;                                                /* what the engine judges by */
            if (a->no_cuts != 0U) { es.ip_lv = 1U; es.ip_hv = 999U; es.op_lv = 1U; es.op_hv = 999U; }   /* TEST: no voltage limits */
            MF_Tick(&a->eng, &es, &in, now, &a->eout);
        }
        for (k = 0U; k < a->eout.n_log; k++) (void)MLog_Append(&a->log, &a->eout.log[k].rec);
        if (a->eout.clr_pulse[0] != 0U) a->clr_cnt++;
        a->ctl_out.run = a->eout.run[0]; a->ctl_out.clr = a->clr_cnt; a->ctl_out.hold = a->eout.hold[0];       /* v0.9.61: the load-connect hold */
        a->ctl_out.setv_dv = (uint16_t)(a->settings.set_v * 10U); a->ctl_out.maxc_dv = (uint16_t)(a->settings.maxcorr_v * 10U);
        for (k = 0U; k < 2U; k++)
        {   /* the $M content for the Y and B slaves: its own run level and clear counter, the same set voltage and Max Correction */
            if (a->eout.clr_pulse[1U + k] != 0U) a->clr_cnt_yb[k]++;
            a->ctl_out_yb[k].run = a->eout.run[1U + k]; a->ctl_out_yb[k].clr = a->clr_cnt_yb[k]; a->ctl_out_yb[k].hold = a->eout.hold[1U + k];
            a->ctl_out_yb[k].setv_dv = a->ctl_out.setv_dv; a->ctl_out_yb[k].maxc_dv = a->ctl_out.maxc_dv;
        }
    }

    /* $M is sent once the slave was heard ($E, the clear counter then in step with its own) OR its $S frames have been arriving for 3 s: a slave whose $E is not understood is still started */
    if (d->have != 0U) { if (a->have_seen == 0U) { a->have_seen = 1U; a->t_first_have = now; } } else a->have_seen = 0U;
    a->ctl_valid = (uint8_t)((a->clr_synced != 0U) || ((a->have_seen != 0U) && ((uint32_t)(now - a->t_first_have) >= 3000UL)));
    for (ph = 0U; ph < 2U; ph++)
    {   const MAppSlave_t *s = &d->yb[ph];
        if (s->have != 0U) { if (a->have_seen_yb[ph] == 0U) { a->have_seen_yb[ph] = 1U; a->t_first_have_yb[ph] = now; } } else a->have_seen_yb[ph] = 0U;
        a->ctl_valid_yb[ph] = (uint8_t)((a->clr_synced_yb[ph] != 0U) || ((a->have_seen_yb[ph] != 0U) && ((uint32_t)(now - a->t_first_have_yb[ph]) >= 3000UL)));
    }
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
    r.sl_pk_ca = d->iout_pk_ca; r.sl_rms_ca = d->iout_rms_ca; r.sl_valid = (uint8_t)((d->have != 0U) && (d->iout_valid != 0U)); r.sl_run = (uint8_t)(d->state == 2U);
    {   /* v0.9.55: the calibration pages of the Y or B phase show ITS slave */
        uint8_t cp = MUI_CalPhase(&a->ui);
        if (cp != 0U)
        {   const MAppSlave_t *s = &d->yb[cp - 1U];
            r.sl_pk_ca = s->iout_pk_ca; r.sl_rms_ca = s->iout_rms_ca; r.sl_valid = (uint8_t)((s->have != 0U) && (s->iout_valid != 0U)); r.sl_run = (uint8_t)(s->state == 2U);
        }
    }      /* for the Slave Load Cal page */
    r.vin_raw_dv = d->vin_dv; r.vout_raw_dv = d->vout_dv; r.active = (uint8_t)((a->eout.run[0] != 0U) || (d->state != 0U));
    { uint8_t cp = MUI_CalPhase(&a->ui); if (cp != 0U) { const MAppSlave_t *s = &d->yb[cp - 1U]; r.vin_raw_dv = s->have ? s->vin_dv : 0U; r.vout_raw_dv = s->have ? s->vout_dv : 0U; r.active = (uint8_t)((a->eout.run[cp] != 0U) || (s->state != 0U)); } }       /* for the Voltage Calibration pages */
    r.wait_s[0] = (a->eout.fault[0] != 0U) ? MF_RestartLeftS(&a->eng, 0U) : 0U;                                      /* the automatic restart countdown for the status row */
    r.fcode[0] = (a->eout.fault[0] != 0U) ? a->eng.u[0].fcode : (((a->eout.appear[0] == 0U) && (d->have != 0U)) ? d->fault : 0U);      /* the slave code behind the fault shown */
    if ((d->have == 0U) && (r.fault[0] == 0U) && (a->eout.appear[0] == (uint8_t)MLOG_T_PHASE_MISS)) r.fault[0] = (uint8_t)MLOG_T_PHASE_MISS;      /* v0.9.57: R missing before a start */
    for (ph = 0U; ph < 2U; ph++)
    {   /* v0.9.57: the Y and B rows of the status page show THEIR fault (they always said OK): the engine's, else what is violated now, else what their slave reports */
        const MAppSlave_t *s = &d->yb[ph]; uint8_t q = (uint8_t)(1U + ph), u = (a->settings.mode == (uint8_t)M_MODE_DOMESTIC) ? q : 0U;
        if (a->eout.fault[q] != 0U) { r.fault[q] = a->eout.fault[q]; r.fcode[q] = a->eng.u[u].fcode; }
        else if ((a->eout.appear[q] != 0U) && ((s->have != 0U) || (a->eout.appear[q] == (uint8_t)MLOG_T_PHASE_MISS))) r.fault[q] = a->eout.appear[q];
        else if ((s->have != 0U) && (s->fault != 0U)) { r.fault[q] = MF_SlaveFaultType(s->fault, s->vin_dv, a->settings.set_v); r.fcode[q] = s->fault; }
    }
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
    if (MUI_Take(&a->ui, MUI_REQ_SLCAL_SAVE)) slcal_request(a, d, now);
    if (MUI_Take(&a->ui, MUI_REQ_RETUNE) != 0U) { uint8_t sel = MUI_RetuneSel(&a->ui); a->tune_all = (uint8_t)((sel == 4U) && (a->settings.mode != (uint8_t)M_MODE_SINGLE)); tune_request(a, d, (uint8_t)((sel >= 1U) && (sel <= 3U) ? (sel - 1U) : 0U), now); }      /* v0.9.54: R, Y, B, or all three one by one */
    tune_service(a, d, now);
}

uint8_t MApp_SetModeOpen(const MApp_t *a) { return MUI_SetModeOpen(&a->ui); }
const MCtlOut_t *MApp_Control(const MApp_t *a) { return &a->ctl_out; }
const MCtlOut_t *MApp_ControlPh(const MApp_t *a, uint8_t ph) { return (ph == 1U) ? &a->ctl_out_yb[0] : ((ph == 2U) ? &a->ctl_out_yb[1] : &a->ctl_out); }
uint8_t MApp_ControlValidPh(const MApp_t *a, uint8_t ph) { return (ph == 1U) ? a->ctl_valid_yb[0] : ((ph == 2U) ? a->ctl_valid_yb[1] : a->ctl_valid); }
uint8_t MApp_ControlValid(const MApp_t *a) { return a->ctl_valid; }
void MApp_NoteFrameSent(MApp_t *a) { a->m_sent++; }
uint8_t MApp_TakeClockSet(MApp_t *a, MDateTime_t *out) { if (a->clk_set_pending == 0U) return 0U; *out = a->clk_set; a->clk_set_pending = 0U; return 1U; }
uint8_t MApp_HelloDue(MApp_t *a, uint32_t now)
{
    if (a->last.e_valid == 0U) { a->hello_n = 0U; return 0U; }
    if (a->hello_force != 0U) { a->hello_force = 0U; a->t_hello = now; a->hello_n = 1U; return 1U; }      /* a tune has just stored a profile: ask for the slave's hello AGAIN, its limits and the profile numbers (the Transformer Profile page) */
    if (a->last.h_valid != 0U) { a->hello_n = 0U; return 0U; }
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
    uint16_t entered = MUI_CalValue(&a->ui); uint8_t ch = ((((a->ui.live->mode == (uint8_t)M_MODE_SINGLE) ? a->ui.page : a->ui.svc)) == 6U) ? 1U : 0U;      /* the output page (v0.9.54: the service pages exist in every mode) */
    uint8_t cp = MUI_CalPhase(&a->ui), have = d->have, st = d->state;                      /* v0.9.55: the chosen phase's slave, over its own link */
    if (cp != 0U) { have = d->yb[cp - 1U].have; st = d->yb[cp - 1U].state; }
    if (a->vcal_wait != 0U) { MUI_Notify(&a->ui, "Busy", "Wait For The Answer", now); return; }
    if (have == 0U) { MUI_Notify(&a->ui, "Slave Link Lost", "", now); return; }
    if ((a->eout.run[cp] != 0U) || (st != 0U)) { MUI_Notify(&a->ui, "Stop The Unit", "First", now); return; }          /* never while it regulates: a gain change would make the output jump */
    a->vcal_ch = ch;
    if (entered == 0U) { a->vcal_cmd = MAPP_KCMD_CALR; a->vcal_letter = (ch == 0U) ? 'V' : 'O'; a->vcal_num = 0; }          /* typing 000.0 resets the channel to nominal */
    else if ((entered < 500U) || (entered > 4000U)) { MUI_Notify(&a->ui, "Enter 50.0-400.0 V", "Or 000.0 To Reset", now); return; }
    else { a->vcal_cmd = (ch == 0U) ? MAPP_KCMD_CALV : MAPP_KCMD_CALO; a->vcal_letter = '\0'; a->vcal_num = (int32_t)entered; }
    a->vcal_ph = cp; a->vcal_id = (cp == 0U) ? MApp_NextKId(a) : yb_kid(a, cp); a->vcal_tx_n = 0U; a->vcal_t0 = now; a->vcal_t_tx = now; a->vcal_due = 1U; a->vcal_wait = 1U;
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
        out->id = a->tune_id; out->cmd = a->tune_cmd; out->num = 0; out->letter = '\0'; out->ph = a->tune_ph; a->tune_due = 0U; a->tune_tx_n++; a->tune_t_tx = now; return 1U;
    }
    if ((a->vcal_wait == 0U) || (a->vcal_due == 0U)) return 0U;
    out->id = a->vcal_id; out->cmd = a->vcal_cmd; out->num = a->vcal_num; out->letter = a->vcal_letter; out->ph = a->vcal_ph;
    a->vcal_due = 0U; a->vcal_tx_n++; a->vcal_t_tx = now;
    return 1U;
}

void MApp_OnKResult(MApp_t *a, uint8_t id, uint8_t status, int32_t value)
{
    if ((a->tune_state != TS_IDLE) && (a->tune_ph == 0U) && (id == a->tune_id)) { tune_on_result(a, status, value, a->t_now); return; }       /* the answer to TUNE or TABORT (of the R phase: a Y or B Re-tune is answered on its own link) */
    if ((a->vcal_wait == 0U) || (a->vcal_ph != 0U) || (id != a->vcal_id)) return;   /* not the answer to a calibration of R (a HELLO's, a late one, or one for Y or B) */
    a->vcal_wait = 0U; a->vcal_due = 0U;
    vcal_message(a, status, value);
}

void MApp_OnKResultPh(MApp_t *a, uint8_t ph, uint8_t id, uint8_t status, int32_t value)
{
    if (ph == 0U) { MApp_OnKResult(a, id, status, value); return; }
    if ((a->tune_state != TS_IDLE) && (a->tune_ph == ph) && (id == a->tune_id)) { tune_on_result(a, status, value, a->t_now); return; }      /* the Y and B links: only the answer to that phase's TUNE or TABORT */
    if ((a->vcal_wait != 0U) && (a->vcal_ph == ph) && (id == a->vcal_id)) { a->vcal_wait = 0U; a->vcal_due = 0U; vcal_message(a, status, value); }      /* v0.9.55: or to its voltage calibration */
}
void MApp_SetCtFitted(MApp_t *a, uint8_t fitted)
{
    a->no_ct = (uint8_t)(fitted == 0U);
    MUI_SetFeatures(&a->ui, (uint8_t)((a->ui.features & (uint8_t)~MUI_FEAT_NO_CT) | ((a->no_ct != 0U) ? MUI_FEAT_NO_CT : 0U)));
}
uint8_t MApp_TakeYbHello(MApp_t *a, uint8_t i) { if ((i > 1U) || (a->hello_yb[i] == 0U)) return 0U; a->hello_yb[i] = 0U; return 1U; }

uint8_t MApp_ShowsProfile(const MApp_t *a) { return MUI_ShowsProfile(&a->ui); }

/* The transformer profile page (LOG, then MENU x8): what the slave's last autotune stored (from the optional fields of its $H frame). Three numbers decide what it does: the duty CAP (with SLAVE_TUNE_APPLY the regulator never asks for more),
 * the GAIN SCALE for boost and buck (the regulator's integral gain is multiplied by it: below 1 for a transformer with more gain than the reference), and the input voltage and Max Correction it was MADE at (a cap made with a low Max Correction is not applied). */
void MApp_ProfilePage(const MApp_t *a, uint32_t now, char frame[MUI_ROWS][MUI_COLS + 1U])
{
    const MAppData_t *dd = &a->last; uint32_t r, c, n; char t[24];
    struct { uint8_t prof_n, prof_knee; uint16_t prof_cap, prof_kib, prof_kiu, prof_vref_dv, prof_maxc_dv; } pp, *d = &pp;
    uint8_t ph = (uint8_t)((a->settings.mode == (uint8_t)M_MODE_SINGLE) ? 0U : ((now / 3000U) % 3U));      /* v0.9.54: Industrial and Domestic show R, Y and B in turn, 3 s each */
    if (ph == 0U) { pp.prof_n = dd->prof_n; pp.prof_knee = dd->prof_knee; pp.prof_cap = dd->prof_cap; pp.prof_kib = dd->prof_kib; pp.prof_kiu = dd->prof_kiu; pp.prof_vref_dv = dd->prof_vref_dv; pp.prof_maxc_dv = dd->prof_maxc_dv; }
    else { const MAppSlave_t *s = &dd->yb[ph - 1U]; pp.prof_n = s->prof_n; pp.prof_knee = s->prof_knee; pp.prof_cap = s->prof_cap; pp.prof_kib = s->prof_kib; pp.prof_kiu = s->prof_kiu; pp.prof_vref_dv = s->prof_vref_dv; pp.prof_maxc_dv = s->prof_maxc_dv; }
    for (r = 0U; r < MUI_ROWS; r++) { for (c = 0U; c < MUI_COLS; c++) frame[r][c] = ' '; frame[r][MUI_COLS] = '\0'; }
    copy(t, (a->settings.mode == (uint8_t)M_MODE_SINGLE) ? "Transformer Profile" : ((ph == 0U) ? "Transformer Prof. R" : ((ph == 1U) ? "Transformer Prof. Y" : "Transformer Prof. B"))); for (n = 0U; t[n] != '\0'; n++) frame[0][n] = t[n];
    if (d->prof_cap != 0U)
    {
        uint32_t cap = (d->prof_cap > 9999U) ? 9999U : d->prof_cap, kb = (d->prof_kib > 9999U) ? 9999U : d->prof_kib, ku = (d->prof_kiu > 9999U) ? 9999U : d->prof_kiu;
        uint32_t vr = (uint32_t)(d->prof_vref_dv + 5U) / 10U, mc = (uint32_t)(d->prof_maxc_dv + 5U) / 10U, pn = (d->prof_n > 99U) ? 99U : d->prof_n;
        if (vr > 999U) { vr = 999U; }
        if (mc > 999U) { mc = 999U; }
        copy(t, "Cap 0000 ticks"); t[4] = (char)('0' + ((cap / 1000U) % 10U)); t[5] = (char)('0' + ((cap / 100U) % 10U)); t[6] = (char)('0' + ((cap / 10U) % 10U)); t[7] = (char)('0' + (cap % 10U));
        if (d->prof_knee != 0U) { t[14] = ' '; t[15] = 'K'; t[16] = 'n'; t[17] = 'e'; t[18] = 'e'; t[19] = '\0'; }
        for (n = 0U; t[n] != '\0'; n++) frame[1][n] = t[n];
        copy(t, "Gain B0.000 U0.000"); t[6] = (char)('0' + ((kb / 1000U) % 10U)); t[8] = (char)('0' + ((kb / 100U) % 10U)); t[9] = (char)('0' + ((kb / 10U) % 10U)); t[10] = (char)('0' + (kb % 10U));
        t[13] = (char)('0' + ((ku / 1000U) % 10U)); t[15] = (char)('0' + ((ku / 100U) % 10U)); t[16] = (char)('0' + ((ku / 10U) % 10U)); t[17] = (char)('0' + (ku % 10U));
        for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n];
        copy(t, "At 000V Max 000V n00"); t[3] = (char)('0' + ((vr / 100U) % 10U)); t[4] = (char)('0' + ((vr / 10U) % 10U)); t[5] = (char)('0' + (vr % 10U));
        t[12] = (char)('0' + ((mc / 100U) % 10U)); t[13] = (char)('0' + ((mc / 10U) % 10U)); t[14] = (char)('0' + (mc % 10U)); t[18] = (char)('0' + ((pn / 10U) % 10U)); t[19] = (char)('0' + (pn % 10U));
        for (n = 0U; t[n] != '\0'; n++) frame[3][n] = t[n];
    }
    else
    {
        copy(t, "No profile stored"); for (n = 0U; t[n] != '\0'; n++) frame[1][n] = t[n];
        copy(t, "(or an older slave)"); for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n];
        copy(t, "Set-Mode: Re-tune"); for (n = 0U; t[n] != '\0'; n++) frame[3][n] = t[n];
    }
}

uint8_t MApp_ShowsIout(const MApp_t *a) { return MUI_ShowsIout(&a->ui); }

/* The converter-side current page (LOG, then MENU x7): what the slave's CT2500 chain (net I-OUT) measures, to check the scaling against a clamp meter on the same conductor. It is the CT PRIMARY current. */
/* the last row of the Converter Current page: the R slave's heat-sink temperature (v0.9.49) */
static void heat_row(const MApp_t *a, uint8_t ph, char *t)
{
    int16_t dc; uint8_t st = MApp_TempDeci(a, ph, &dc); uint32_t v;
    if (st == 255U)       { copy(t, "Heat Sink  ---"); return; }
    if (st == MNTC_OPEN)  { copy(t, "Heat Sink: no NTC"); return; }
    if (st == MNTC_SHORT) { copy(t, "OVERHEAT: NTC short"); return; }
    copy(t, (a->heat_on[ph] != 0U) ? "OVERHEAT  000.0 C" : "Heat Sink 000.0 C");
    v = (dc < 0) ? (uint32_t)(-(int32_t)dc) : (uint32_t)dc;
    t[10] = (dc < 0) ? '-' : (char)('0' + ((v / 1000U) % 10U)); t[11] = (char)('0' + ((v / 100U) % 10U)); t[12] = (char)('0' + ((v / 10U) % 10U)); t[14] = (char)('0' + (v % 10U));
}
void MApp_IoutPage(const MApp_t *a, uint32_t now, char frame[MUI_ROWS][MUI_COLS + 1U])
{
    const MAppData_t *dd = &a->last; uint32_t r, c, n; char t[24];
    struct { uint8_t iout_valid, iout_clip; uint16_t iout_rms_ca, iout_pk_ca; } cv, *d = &cv;
    uint8_t ph = (uint8_t)((a->settings.mode == (uint8_t)M_MODE_SINGLE) ? 0U : ((now / 3000U) % 3U));      /* v0.9.60: Industrial and Domestic show R, Y and B in turn, 3 s each */
    if (ph == 0U) { cv.iout_valid = dd->iout_valid; cv.iout_clip = dd->iout_clip; cv.iout_rms_ca = dd->iout_rms_ca; cv.iout_pk_ca = dd->iout_pk_ca; }
    else { const MAppSlave_t *s = &dd->yb[ph - 1U]; cv.iout_valid = (uint8_t)(s->have && s->iout_valid); cv.iout_clip = s->iout_clip; cv.iout_rms_ca = s->iout_rms_ca; cv.iout_pk_ca = s->iout_pk_ca; }
    for (r = 0U; r < MUI_ROWS; r++) { for (c = 0U; c < MUI_COLS; c++) frame[r][c] = ' '; frame[r][MUI_COLS] = '\0'; }
    if (a->settings.mode == (uint8_t)M_MODE_SINGLE) { copy(t, "Converter Current"); for (n = 0U; t[n] != '\0'; n++) frame[0][1U + n] = t[n]; }
    else { copy(t, (ph == 0U) ? "Converter Current R" : ((ph == 1U) ? "Converter Current Y" : "Converter Current B")); for (n = 0U; t[n] != '\0'; n++) frame[0][n] = t[n]; }
    if (d->iout_valid != 0U)
    {
        uint32_t rms = (d->iout_rms_ca > 9999U) ? 9999U : d->iout_rms_ca, pk = (d->iout_pk_ca > 9999U) ? 9999U : d->iout_pk_ca;
        copy(t, "RMS   00.00 A"); t[6] = (char)('0' + ((rms / 1000U) % 10U)); t[7] = (char)('0' + ((rms / 100U) % 10U)); t[9] = (char)('0' + ((rms / 10U) % 10U)); t[10] = (char)('0' + (rms % 10U)); if (t[6] == '0') t[6] = ' ';
        for (n = 0U; t[n] != '\0'; n++) frame[1][n] = t[n];
        copy(t, "Peak  00.00 A"); t[6] = (char)('0' + ((pk / 1000U) % 10U)); t[7] = (char)('0' + ((pk / 100U) % 10U)); t[9] = (char)('0' + ((pk / 10U) % 10U)); t[10] = (char)('0' + (pk % 10U)); if (t[6] == '0') t[6] = ' ';
        for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n];
        if (d->iout_clip != 0U) copy(t, "OVER RANGE (clipped)"); else heat_row(a, ph, t);
    }
    else { copy(t, "RMS    --.-- A"); for (n = 0U; t[n] != '\0'; n++) frame[1][n] = t[n]; copy(t, "Peak   --.-- A"); for (n = 0U; t[n] != '\0'; n++) frame[2][n] = t[n]; heat_row(a, ph, t); }
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
