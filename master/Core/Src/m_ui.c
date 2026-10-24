/** @file m_ui.c  @brief Screens and key logic. See m_ui.h. All texts are in this file, in tables, so they can be adjusted in one place. */
#include "m_ui.h"

typedef char Frame_t[MUI_ROWS][MUI_COLS + 1U];

/* ---------------------------------------------------------------------------------------------------- text helpers */
static void f_clear(Frame_t f) { uint32_t r, c; for (r = 0U; r < MUI_ROWS; r++) { for (c = 0U; c < MUI_COLS; c++) f[r][c] = ' '; f[r][MUI_COLS] = '\0'; } }
static void f_put(Frame_t f, uint32_t row, uint32_t col, const char *s) { while ((*s != '\0') && (col < MUI_COLS)) f[row][col++] = *s++; }
static uint32_t slen(const char *s) { uint32_t n = 0U; while (s[n] != '\0') n++; return n; }
static void f_center(Frame_t f, uint32_t row, const char *s) { uint32_t n = slen(s); if (n > MUI_COLS) n = MUI_COLS; f_put(f, row, (MUI_COLS - n) / 2U, s); }
static void f_put_right(Frame_t f, uint32_t row, const char *s) { uint32_t n = slen(s); if (n < MUI_COLS) f_put(f, row, MUI_COLS - n, s); else f_put(f, row, 0U, s); }

/* a number with `digits` digits, zero padded, into out; returns the length */
static uint32_t fmt_zero(char *out, uint32_t v, uint32_t digits)
{
    uint32_t i; for (i = 0U; i < digits; i++) { out[digits - 1U - i] = (char)('0' + (v % 10U)); v /= 10U; }
    out[digits] = '\0'; return digits;
}
/* whole volts: a right-aligned integer of width w, "---" when there is no reading */
static void fmt_int(char *out, uint32_t v, uint32_t w, uint8_t ok)
{
    char tmp[8]; uint32_t n = 0U, i, pad; v = (v + 5U) / 10U;
    if (ok == 0U) { for (i = 0U; i < w; i++) out[i] = ' '; for (i = 0U; (i < 3U) && (i < w); i++) out[w - 1U - i] = '-'; out[w] = '\0'; return; }
    if (v == 0U) { tmp[n++] = '0'; }
    while ((v > 0U) && (n < 7U)) { tmp[n++] = (char)('0' + (v % 10U)); v /= 10U; }
    pad = (n < w) ? (w - n) : 0U;
    for (i = 0U; i < pad; i++) out[i] = ' ';
    for (i = 0U; (i < n) && ((pad + i) < w); i++) out[pad + i] = tmp[n - 1U - i];
    out[pad + ((n < w) ? n : w)] = '\0';
}
static void int_put(Frame_t f, uint32_t row, uint32_t col, uint32_t v, uint32_t w, uint8_t ok) { char b[10]; fmt_int(b, v, w, ok); f_put(f, row, col, b); }

/* tenths: "230.0", width w, right aligned with spaces; v == 0 and !ok gives dashes */
static void fmt_tenths(char *out, uint32_t v, uint32_t w, uint8_t ok)
{
    char tmp[12]; uint32_t n = 0U, i, ip = v / 10U, fp = v % 10U, pad;
    if (ok == 0U)
    {   /* no reading: "--.-" right aligned */
        static const char D[] = "--.-"; uint32_t dl = 4U;
        for (i = 0U; i < w; i++) out[i] = ' ';
        for (i = 0U; i < dl && i < w; i++) out[w - dl + i] = D[i];
        out[w] = '\0'; return;
    }
    { char d[8]; uint32_t k = 0U; if (ip == 0U) d[k++] = '0'; while (ip > 0U) { d[k++] = (char)('0' + (ip % 10U)); ip /= 10U; } while (k > 0U) tmp[n++] = d[--k]; }
    tmp[n++] = '.'; tmp[n++] = (char)('0' + fp); tmp[n] = '\0';
    pad = (n < w) ? (w - n) : 0U;
    for (i = 0U; i < pad; i++) out[i] = ' ';
    for (i = 0U; i < n && (pad + i) < w; i++) out[pad + i] = tmp[i];
    out[pad + n] = '\0';
}
static void tenths_put(Frame_t f, uint32_t row, uint32_t col, uint32_t v, uint32_t w, uint8_t ok) { char b[12]; fmt_tenths(b, v, w, ok); f_put(f, row, col, b); }

static void msg_set(MUI_t *u, const char *l1, const char *l2, uint32_t now, uint8_t to_run)
{
    uint32_t i;
    for (i = 0U; i < MUI_COLS; i++) { u->msg[0][i] = (i < slen(l1)) ? l1[i] : '\0'; u->msg[1][i] = (i < slen(l2)) ? l2[i] : '\0'; }
    u->msg[0][MUI_COLS] = '\0'; u->msg[1][MUI_COLS] = '\0';
    u->msg_on = 1U; u->t_msg = now; u->return_to_run = to_run;
}

/* ---------------------------------------------------------------------------------------------------- fault texts */
static const char *full_text(uint8_t type)
{
    switch (type)
    {
    case MLOG_T_IP_LV: return "Input LV";            case MLOG_T_IP_HV: return "Input HV";
    case MLOG_T_OP_LV: return "Output LV";           case MLOG_T_OP_HV: return "Output HV";
    case MLOG_T_OVERLOAD: return "Overload";         case MLOG_T_HEAT: return "Heat";
    case MLOG_T_FLIP: return "Transformer Flip";     case MLOG_T_INRUSH: return "Inrush";
    case MLOG_T_PHASE_SEQ: return "Phase Sequence";  case MLOG_T_MAINS: return "Mains Unstable";
    case MLOG_T_POWER_STAGE: return "Power Stage Fault"; case MLOG_T_RETUNE: return "Re-tune Required";
    case MLOG_T_LINK_LOST: return "Slave Link Lost"; case MLOG_T_WDG_RESET: return "Watchdog Reset"; default: return "";
    }
}
/* the short text after "R : " in three-phase: at most 16 characters */
static void short_text(char *o, uint8_t type, uint8_t code, uint8_t seq)
{
    const char *s = "";
    switch (type)
    {
    case MLOG_T_IP_LV: s = "IP LV Appear"; break;   case MLOG_T_IP_HV: s = "IP HV Appear"; break;
    case MLOG_T_OP_LV: s = "OP LV Appear"; break;   case MLOG_T_OP_HV: s = "OP HV Appear"; break;
    case MLOG_T_OVERLOAD: s = "OL Appear"; break;   case MLOG_T_HEAT: s = "Heat Appear"; break;
    case MLOG_T_FLIP: s = "TRF Flip"; break;        case MLOG_T_INRUSH: s = "INRUSH"; break;
    case MLOG_T_PHASE_SEQ: s = (seq == 2U) ? "PH SEQ SAME" : "PH SEQ REVERSE"; break;
    case MLOG_T_RETUNE: s = "RETUNE REQ"; break;    case MLOG_T_LINK_LOST: s = "LINK LOST"; break;
    case MLOG_T_WDG_RESET: s = "WDG RESET"; break;
    case MLOG_T_MAINS: case MLOG_T_POWER_STAGE:
    {   uint32_t n = 0U; const char *p = (type == MLOG_T_MAINS) ? "MAINS F" : "PS F"; while (p[n] != '\0') { o[n] = p[n]; n++; }
        o[n++] = (char)('0' + ((code / 10U) % 10U)); o[n++] = (char)('0' + (code % 10U)); o[n] = '\0'; return; }
    default: s = "OK"; break;
    }
    { uint32_t n = 0U; while (s[n] != '\0') { o[n] = s[n]; n++; } o[n] = '\0'; }
}
static const char *mode_text(uint8_t m) { return (m == (uint8_t)M_MODE_INDUSTRIAL) ? "Industrial" : ((m == (uint8_t)M_MODE_DOMESTIC) ? "Domestic" : "Single Phase"); }

/* ---------------------------------------------------------------------------------------------------- Set-Mode items */
typedef enum { K_NUM = 0, K_MODE, K_YN, K_AUTOMAN, K_DATE, K_TIME, K_LOADSRC } MKind_t;
typedef struct { uint8_t id; const char *name; uint8_t kind, digits; char unit; } MItem_t;
#define N_ITEMS 25U
static const MItem_t ITEMS[N_ITEMS] =
{
    {  1, "Mode",                K_MODE,    0U, ' ' }, {  2, "Calibration",         K_YN,   0U, ' ' }, {  3, "Set Voltage",         K_NUM,  3U, 'V' },
    {  4, "Max Correction",      K_NUM,     3U, 'V' }, {  5, "I/P LV",              K_NUM,  3U, 'V' }, {  6, "I/P HV",              K_NUM,  3U, 'V' },
    {  7, "O/P LV",              K_NUM,     3U, 'V' }, {  8, "O/P HV",              K_NUM,  3U, 'V' }, {  9, "Volt Cut Delay",      K_NUM,  2U, 's' },
    { 10, "Overload Current",    K_NUM,     3U, 'A' }, { 25, "Load Source",         K_LOADSRC, 0U, ' ' }, { 11, "Overload Cut Delay",  K_NUM,  2U, 's' }, { 12, "Overheat Trip Delay", K_NUM,  2U, 's' },
    { 13, "Phase Sequence",      K_YN,      0U, ' ' }, { 14, "Display Scroll",      K_YN,   0U, ' ' }, { 15, "Scroll Time",         K_NUM,  2U, 's' },
    { 16, "Password",            K_NUM,     3U, ' ' }, { 17, "Restart Mode",        K_AUTOMAN, 0U, ' ' }, { 18, "Restart Time",     K_NUM,  3U, 's' },
    { 23, "Date",                K_DATE,    8U, ' ' }, { 24, "Time",                K_TIME, 4U, ' ' },
    { 19, "Re-tune",             K_YN,      0U, ' ' }, { 20, "Clear Fault Log",     K_YN,   0U, ' ' }, { 21, "Discard Changes",     K_YN,   0U, ' ' },
    { 22, "Factory Reset",       K_YN,      0U, ' ' }
};

static uint8_t is_action(uint8_t id) { return (id == 2U) || (id == 19U) || (id == 20U) || (id == 21U) || (id == 22U); }
static uint8_t visible(const MSettings_t *w, uint8_t id)
{
    switch (id)
    {
    case 13: return (uint8_t)(w->mode == (uint8_t)M_MODE_INDUSTRIAL);                         /* Phase Sequence: Industrial only */
    case 14: return (uint8_t)(w->mode != (uint8_t)M_MODE_SINGLE);                             /* Display Scroll: three-phase only */
    case 15: return (uint8_t)((w->mode != (uint8_t)M_MODE_SINGLE) && (w->scroll != 0U));      /* Scroll Time: only if scrolling */
    case 18: return (uint8_t)(w->restart_auto != 0U);                                         /* Restart Time: Auto only */
    default: return 1U;
    }
}
static const MItem_t *item_of(uint8_t id) { uint32_t i; for (i = 0U; i < N_ITEMS; i++) if (ITEMS[i].id == id) return &ITEMS[i]; return &ITEMS[0]; }

static uint16_t item_get(const MSettings_t *s, uint8_t id)
{
    switch (id)
    {
    case 1: return s->mode;   case 3: return s->set_v;  case 4: return s->maxcorr_v; case 5: return s->ip_lv; case 6: return s->ip_hv; case 7: return s->op_lv; case 8: return s->op_hv;
    case 9: return s->volt_cut_delay; case 10: return s->ovl_current; case 11: return s->ovl_cut_delay; case 12: return s->heat_delay; case 13: return s->phase_seq; case 14: return s->scroll;
    case 15: return s->scroll_time; case 16: return s->password; case 17: return s->restart_auto; case 18: return s->restart_time; case 25: return s->load_src; default: return 0U;
    }
}
static void item_set(MSettings_t *s, uint8_t id, uint16_t v)
{
    switch (id)
    {
    case 1: s->mode = (uint8_t)v; break;   case 3: s->set_v = v; break;  case 4: s->maxcorr_v = v; break; case 5: s->ip_lv = v; break; case 6: s->ip_hv = v; break; case 7: s->op_lv = v; break; case 8: s->op_hv = v; break;
    case 9: s->volt_cut_delay = (uint8_t)v; break; case 10: s->ovl_current = v; break; case 11: s->ovl_cut_delay = (uint8_t)v; break; case 12: s->heat_delay = (uint8_t)v; break;
    case 13: s->phase_seq = (uint8_t)v; break; case 14: s->scroll = (uint8_t)v; break; case 15: s->scroll_time = (uint8_t)v; break; case 16: s->password = v; break;
    case 17: s->restart_auto = (uint8_t)v; break; case 18: s->restart_time = v; break; case 25: s->load_src = (uint8_t)v; break; default: break;
    }
}
static uint8_t item_of_error(MSErr_t e)
{
    switch (e)
    {
    case MS_BAD_MODE: return 1U; case MS_BAD_SET_V: return 3U; case MS_BAD_MAXCORR: return 4U; case MS_BAD_IP_LV: return 5U; case MS_BAD_IP_HV: return 6U; case MS_BAD_OP_LV: return 7U; case MS_BAD_OP_HV: return 8U;
    case MS_BAD_VOLT_DELAY: return 9U; case MS_BAD_OVL_CURRENT: return 10U; case MS_BAD_OVL_DELAY: return 11U; case MS_BAD_HEAT_DELAY: return 12U; case MS_BAD_SCROLL_TIME: return 15U;
    case MS_BAD_PASSWORD: return 16U; case MS_BAD_RESTART_TIME: return 18U; default: return 1U;
    }
}

static uint8_t first_item(const MUI_t *u) { uint8_t i; for (i = 1U; i <= N_ITEMS; i++) if (visible(&u->work, i)) return i; return 1U; }
static uint8_t item_index(uint8_t id) { uint32_t i; for (i = 0U; i < N_ITEMS; i++) if (ITEMS[i].id == id) return (uint8_t)i; return 0U; }
/* the next VISIBLE item in the order of the table (the ids are fixed, the order is not: Date and Time sit after Restart Time) */
static uint8_t next_item(const MUI_t *u, uint8_t from) { uint8_t i = item_index(from), n; for (n = 0U; n < N_ITEMS; n++) { i = (uint8_t)((i + 1U) % N_ITEMS); if (visible(&u->work, ITEMS[i].id)) return ITEMS[i].id; } return from; }

static void enter_item(MUI_t *u, uint8_t id)
{
    u->item = id; u->cursor = 0U; u->yn = 0U; u->clk_dirty = 0U;
    u->ed_backup = item_get(&u->work, id);
    if ((id == 23U) || (id == 24U))
    {   /* Date and Time are edited on one working copy that starts from the clock (or a placeholder if it was never set); it is written to the clock only when Set-Mode is SAVED */
        if (u->clk_active == 0U)
        {
            if (u->clk_now_valid != 0U) u->clk_work = u->clk_now;
            else { u->clk_work.year = 2026U; u->clk_work.month = 1U; u->clk_work.day = 1U; u->clk_work.hour = 0U; u->clk_work.minute = 0U; }
            u->clk_active = 1U; u->clk_pending = 0U;
        }
        u->clk_backup = u->clk_work;
    }
}
/* the part of the working Date or Time that item `id` edits is valid */
static uint8_t clk_part_valid(const MUI_t *u, uint8_t id)
{
    const MDateTime_t *t = &u->clk_work;
    if (id == 24U) return (uint8_t)((t->hour <= 23U) && (t->minute <= 59U));
    return (uint8_t)((t->year >= MC_YEAR_MIN) && (t->year <= MC_YEAR_MAX) && (t->month >= 1U) && (t->month <= 12U) && (t->day >= 1U) && (t->day <= MC_DaysInMonth(t->year, t->month)));
}
/* one digit up (9 wraps to 0, no carry), the digits of dd/mm/yyyy or HH:MM */
static void clk_digit_inc(MUI_t *u, uint8_t id)
{
    MDateTime_t *t = &u->clk_work; uint8_t dg[8]; uint16_t y;
    if (id == 23U)
    {
        y = t->year; dg[0] = (uint8_t)((t->day / 10U) % 10U); dg[1] = (uint8_t)(t->day % 10U); dg[2] = (uint8_t)((t->month / 10U) % 10U); dg[3] = (uint8_t)(t->month % 10U);
        dg[4] = (uint8_t)((y / 1000U) % 10U); dg[5] = (uint8_t)((y / 100U) % 10U); dg[6] = (uint8_t)((y / 10U) % 10U); dg[7] = (uint8_t)(y % 10U);
        dg[u->cursor & 7U] = (uint8_t)((dg[u->cursor & 7U] + 1U) % 10U);
        t->day = (uint8_t)(dg[0] * 10U + dg[1]); t->month = (uint8_t)(dg[2] * 10U + dg[3]); t->year = (uint16_t)(dg[4] * 1000U + dg[5] * 100U + dg[6] * 10U + dg[7]);
    }
    else
    {
        dg[0] = (uint8_t)((t->hour / 10U) % 10U); dg[1] = (uint8_t)(t->hour % 10U); dg[2] = (uint8_t)((t->minute / 10U) % 10U); dg[3] = (uint8_t)(t->minute % 10U);
        dg[u->cursor & 3U] = (uint8_t)((dg[u->cursor & 3U] + 1U) % 10U);
        t->hour = (uint8_t)(dg[0] * 10U + dg[1]); t->minute = (uint8_t)(dg[2] * 10U + dg[3]);
    }
    u->clk_dirty = 1U;
}

static void open_set(MUI_t *u, uint32_t now)
{
    u->work = *u->live; u->screen = (uint8_t)MUI_SET; u->t_key = now; u->msg_on = 0U; u->clk_active = 0U; u->clk_pending = 0U; u->clk_dirty = 0U;
    enter_item(u, first_item(u));
}

/* ---------------------------------------------------------------------------------------------------- init, keys */
void MUI_Init(MUI_t *u, MSettings_t *live, MLog_t *log, uint32_t now)
{
    uint8_t *p = (uint8_t *)u; uint32_t i;
    for (i = 0U; i < (uint32_t)sizeof(*u); i++) p[i] = 0U;
    u->live = live; u->log = log; u->screen = (uint8_t)MUI_SPLASH; u->t0 = now; u->t_key = now; u->t_page = now;
}
void MUI_SetLimits(MUI_t *u, const MLimits_t *lim) { u->lim = lim; }
void MUI_SetFeatures(MUI_t *u, uint8_t feat) { u->features = feat; }
uint8_t MUI_SetModeOpen(const MUI_t *u) { return (uint8_t)((u->screen == (uint8_t)MUI_SET) || (u->screen == (uint8_t)MUI_PW)); }
uint8_t MUI_Take(MUI_t *u, uint16_t req) { if ((u->req & req) != 0U) { u->req = (uint16_t)(u->req & ~req); return 1U; } return 0U; }
void MUI_Notify(MUI_t *u, const char *l1, const char *l2, uint32_t now) { msg_set(u, l1, l2, now, 0U); }
void MUI_SetClock(MUI_t *u, const MDateTime_t *now, uint8_t valid) { u->clk_now = *now; u->clk_now_valid = (uint8_t)(valid != 0U); }
const MDateTime_t *MUI_ClockToSet(const MUI_t *u) { return &u->clk_set; }
uint16_t MUI_CalValue(const MUI_t *u) { return u->cal_val; }
uint8_t MUI_ShowsCal(const MUI_t *u) { return (uint8_t)((u->screen == (uint8_t)MUI_RUN) && (u->live->mode == (uint8_t)M_MODE_SINGLE) && (u->page == 3U) && (u->msg_on == 0U)); }
uint8_t MUI_ShowsVCal(const MUI_t *u) { return (uint8_t)(((u->screen == (uint8_t)MUI_RUN) && (u->live->mode == (uint8_t)M_MODE_SINGLE) && (u->msg_on == 0U) && ((u->page == 5U) || (u->page == 6U))) ? (u->page - 4U) : 0U); }
uint8_t MUI_ShowsProfile(const MUI_t *u) { return (uint8_t)((u->screen == (uint8_t)MUI_RUN) && (u->live->mode == (uint8_t)M_MODE_SINGLE) && (u->page == 8U) && (u->msg_on == 0U)); }
uint8_t MUI_ShowsIout(const MUI_t *u) { return (uint8_t)((u->screen == (uint8_t)MUI_RUN) && (u->live->mode == (uint8_t)M_MODE_SINGLE) && (u->page == 7U) && (u->msg_on == 0U)); }
uint8_t MUI_ShowsClock(const MUI_t *u) { return (uint8_t)((u->screen == (uint8_t)MUI_RUN) && (u->live->mode == (uint8_t)M_MODE_SINGLE) && (u->page == 4U) && (u->msg_on == 0U)); }
uint8_t MUI_ShowsDiag(const MUI_t *u) { return (uint8_t)((u->screen == (uint8_t)MUI_RUN) && (u->live->mode == (uint8_t)M_MODE_SINGLE) && (u->page == 2U) && (u->msg_on == 0U)); }
uint8_t MUI_ShowsMonitor(const MUI_t *u) { return (uint8_t)((u->screen == (uint8_t)MUI_RUN) && (u->live->mode == (uint8_t)M_MODE_SINGLE) && (u->page == 1U) && (u->msg_on == 0U)); }

static void leave_set_saved(MUI_t *u, uint32_t now)
{
    MSErr_t e = MS_Validate(&u->work, u->lim);
    if (e != MS_OK)
    {
        enter_item(u, item_of_error(e));
        msg_set(u, "Invalid Value", item_of(u->item)->name, now, 0U);
        return;
    }
    *u->live = u->work; u->req |= MUI_REQ_SAVE;
    if ((u->clk_pending != 0U) && (MC_Valid(&u->clk_work) != 0U)) { u->clk_set = u->clk_work; u->req |= MUI_REQ_SET_CLOCK; }      /* a changed Date or Time goes to the clock now, with the settings */
    u->clk_active = 0U; u->clk_pending = 0U; u->clk_dirty = 0U;
    u->screen = (uint8_t)MUI_RUN; u->page = 0U; u->t_page = now;
    msg_set(u, "Settings", "Saved", now, 1U);
}
static void leave_set_discard(MUI_t *u, uint32_t now, const char *l1, const char *l2)
{
    u->clk_active = 0U; u->clk_pending = 0U; u->clk_dirty = 0U;                          /* a Date or Time edit is discarded with everything else */
    u->screen = (uint8_t)MUI_RUN; u->page = 0U; u->t_page = now;
    msg_set(u, l1, l2, now, 1U);
}

static void digit_inc(MUI_t *u, const MItem_t *it)
{
    uint16_t v = item_get(&u->work, it->id), place = 1U, d; uint32_t k;
    for (k = 0U; k + 1U + u->cursor < it->digits; k++) place = (uint16_t)(place * 10U);
    d = (uint16_t)((v / place) % 10U);
    if (d == 9U) v = (uint16_t)(v - 9U * place); else v = (uint16_t)(v + place);
    item_set(&u->work, it->id, v);
}

static uint8_t is_last_item(const MUI_t *u) { return (uint8_t)(item_index(next_item(u, u->item)) <= item_index(u->item)); }     /* the next visible item would wrap round to the first */

static void set_commit(MUI_t *u, uint32_t now)
{
    const MItem_t *it = item_of(u->item);
    if (is_action(it->id))
    {
        uint8_t yes = u->yn;
        if (yes != 0U)
        {
            if (it->id == 2U) { if ((u->features & MUI_FEAT_CALIBRATE) != 0U) { u->req |= MUI_REQ_CALIBRATE; msg_set(u, "Calibration", "Requested", now, 0U); } else msg_set(u, "Calibration", "Not Available", now, 0U); }
            else if (it->id == 19U) { if ((u->features & MUI_FEAT_RETUNE) != 0U) { u->req |= MUI_REQ_RETUNE; msg_set(u, "Re-tune", "Requested", now, 0U); } else msg_set(u, "Re-tune", "Not Available", now, 0U); }
            else if (it->id == 20U) { u->req |= MUI_REQ_CLEAR_LOG; msg_set(u, "Fault Log", "Cleared", now, 0U); }
            else if (it->id == 21U) { leave_set_discard(u, now, "Changes", "Discarded"); return; }
            else if (it->id == 22U) { uint16_t kz = u->work.sl_zero_ca, kg = u->work.sl_gain; MS_Defaults(&u->work); u->work.sl_zero_ca = kz; u->work.sl_gain = kg; MS_ApplyLim(&u->work, u->lim); }      /* the load calibrations are not settings: they stay */
        }
        if (is_last_item(u)) { leave_set_saved(u, now); return; }          /* one round is complete: leave Set-Mode (saving), do not start again */
        enter_item(u, next_item(u, u->item));
        return;
    }
    if ((it->id == 23U) || (it->id == 24U))
    {
        if (u->clk_dirty != 0U)
        {
            if (clk_part_valid(u, it->id) == 0U) { u->clk_work = u->clk_backup; u->clk_dirty = 0U; msg_set(u, "Invalid Value", it->name, now, 0U); return; }      /* e.g. 31/02: put back, stay */
            u->clk_pending = 1U;
        }
        if (is_last_item(u)) { leave_set_saved(u, now); return; }
        enter_item(u, next_item(u, u->item));
        return;
    }
    {   /* Refuse an edit only if THE EDIT caused the violation: with the item put back the set must have been valid. A violation that was already there (the slave's window or guard is
         * tighter than a default) must not trap the user on this item: the check when leaving Set-Mode sends them to it. */
        MSettings_t before = u->work; MSErr_t after, was;
        item_set(&before, it->id, u->ed_backup);
        if ((it->id == 3U) || (it->id == 4U)) MS_Rederive(&u->work, &before, u->lim);      /* Set Voltage or Max Correction changed: the limits that were at their defaults follow, the others are clipped into the new window */
        after = MS_Validate(&u->work, u->lim); was = MS_Validate(&before, u->lim);
        if ((after != MS_OK) && (was == MS_OK)) { u->work = before; msg_set(u, "Invalid Value", it->name, now, 0U); return; }      /* the WHOLE set goes back: a Set Voltage or Max Correction edit may have moved the limits */     /* put it back, stay on the item */
        if (is_last_item(u)) { leave_set_saved(u, now); return; }
        enter_item(u, next_item(u, u->item));
    }
}

void MUI_Key(MUI_t *u, MKEvent_t ev, uint32_t now)
{
    if (ev == MK_NONE) return;
    if (u->screen == (uint8_t)MUI_SPLASH) return;
    u->t_key = now; u->t_blink = now;
    if (u->msg_on != 0U)
    {   /* a key dismisses a message (the combination is still honoured below) */
        if (ev != MK_COMBO) { u->msg_on = 0U; if (u->return_to_run != 0U) u->screen = (uint8_t)MUI_RUN; return; }
        u->msg_on = 0U;
    }
    switch (u->screen)
    {
    case MUI_RUN:
        if (ev == MK_COMBO)
        {
            if (u->live->password != 0U) { u->screen = (uint8_t)MUI_PW; u->pw[0] = u->pw[1] = u->pw[2] = 0U; u->cursor = 0U; }
            else open_set(u, now);
        }
        else if ((ev == MK_LOG_SHORT) && (u->live->mode == (uint8_t)M_MODE_SINGLE) && (u->page == 3U)) u->req |= MUI_REQ_CAL_SAVE;
        else if ((ev == MK_LOG_SHORT) && (u->live->mode == (uint8_t)M_MODE_SINGLE) && (u->page == 9U)) u->req |= MUI_REQ_SLCAL_SAVE;               /* the Slave Load Cal page: LOG saves the zero (00.0) or the gain */
        else if ((ev == MK_LOG_SHORT) && (u->live->mode == (uint8_t)M_MODE_SINGLE) && ((u->page == 5U) || (u->page == 6U))) u->req |= MUI_REQ_VCAL_SAVE;      /* on the calibration page LOG saves */
        else if (ev == MK_LOG_SHORT) { u->screen = (uint8_t)MUI_LOG; u->log_idx = 0U; }
        else if (ev == MK_MENU_SHORT)
        {
            if (u->live->mode == (uint8_t)M_MODE_SINGLE)
            {   /* ONE Run screen: MENU does nothing on it. The service pages (monitor, diagnostics, load calibration) are reached through the log (LOG, then MENU) and MENU cycles them. */
                if (u->page != 0U) { u->page = (uint8_t)((u->page + 1U) % 10U); if ((u->page == 3U) || (u->page == 5U) || (u->page == 6U) || (u->page == 9U)) { u->cal_init = 1U; u->cal_cursor = 2U; } }     /* the cursor starts on the units of amps */
            }  /* the Run screen, the monitor diagnostics, the control diagnostics, the load calibration */
            else if (u->live->scroll == 0U) { u->page = (uint8_t)((u->page + 1U) % 3U); u->t_page = now; }
        }
        else if ((ev == MK_START) && (u->live->mode == (uint8_t)M_MODE_SINGLE) && ((u->page == 3U) || (u->page == 5U) || (u->page == 6U) || (u->page == 9U)))      /* on the calibration page START edits the digit and STOP moves the cursor: they never start or stop the unit */
        {
            static const uint16_t PLACE[4] = { 1000U, 100U, 10U, 1U };
            uint16_t p = PLACE[u->cal_cursor & 3U], d = (uint16_t)((u->cal_val / p) % 10U);
            u->cal_val = (uint16_t)((d == 9U) ? (u->cal_val - 9U * p) : (u->cal_val + p));
        }
        else if ((ev == MK_STOP) && (u->live->mode == (uint8_t)M_MODE_SINGLE) && ((u->page == 5U) || (u->page == 6U)) && (u->unit_active != 0U)) u->req |= MUI_REQ_STOP;     /* "STOP the unit first": the STOP key does it */
        else if ((ev == MK_STOP) && (u->live->mode == (uint8_t)M_MODE_SINGLE) && ((u->page == 3U) || (u->page == 5U) || (u->page == 6U) || (u->page == 9U))) u->cal_cursor = (uint8_t)((u->cal_cursor + 1U) & 3U);
        else if (ev == MK_START) u->req |= MUI_REQ_START;
        else if (ev == MK_STOP) u->req |= MUI_REQ_STOP;
        break;
    case MUI_LOG:
        if (ev == MK_START) { if ((uint16_t)(u->log_idx + 1U) < MLog_Count(u->log)) u->log_idx++; }               /* older */
        else if (ev == MK_STOP) { if (u->log_idx > 0U) u->log_idx--; }                                           /* newer */
        else if ((ev == MK_MENU_SHORT) && (u->live->mode == (uint8_t)M_MODE_SINGLE)) { u->screen = (uint8_t)MUI_RUN; u->page = 1U; u->t_page = now; }     /* the way to the service pages */
        else if ((ev == MK_LOG_SHORT) || (ev == MK_MENU_SHORT) || (ev == MK_COMBO)) u->screen = (uint8_t)MUI_RUN;
        break;
    case MUI_PW:
        if (ev == MK_START) u->pw[u->cursor] = (uint8_t)((u->pw[u->cursor] + 1U) % 10U);
        else if (ev == MK_STOP) u->cursor = (uint8_t)((u->cursor + 1U) % 3U);
        else if (ev == MK_COMBO) u->screen = (uint8_t)MUI_RUN;
        else if (ev == MK_MENU_SHORT)
        {
            uint16_t typed = (uint16_t)(u->pw[0] * 100U + u->pw[1] * 10U + u->pw[2]);
            if (typed == u->live->password) open_set(u, now);
            else { u->screen = (uint8_t)MUI_RUN; msg_set(u, "Wrong Password", "", now, 1U); }
        }
        break;
    case MUI_SET:
    {
        const MItem_t *it = item_of(u->item);
        if (ev == MK_COMBO)
        {   /* MENU+LOG saves the live edit without a MENU commit: a Set Voltage or Max Correction typed on the item must still move the limits that follow their defaults, exactly as a MENU would */
            if ((it->id == 3U) || (it->id == 4U)) { MSettings_t before = u->work; item_set(&before, it->id, u->ed_backup); MS_Rederive(&u->work, &before, u->lim); }
            if (((it->id == 23U) || (it->id == 24U)) && (u->clk_dirty != 0U))
            {   /* the same as a MENU on the item: a typed Date or Time that does not exist is refused, a good one waits for the save */
                if (clk_part_valid(u, it->id) == 0U) { u->clk_work = u->clk_backup; u->clk_dirty = 0U; msg_set(u, "Invalid Value", it->name, now, 0U); break; }
                u->clk_pending = 1U;
            }
            leave_set_saved(u, now);
        }
        else if (ev == MK_MENU_SHORT) set_commit(u, now);
        else if (ev == MK_START)
        {
            if (is_action(it->id)) u->yn = (uint8_t)(u->yn ^ 1U);
            else if (it->kind == (uint8_t)K_MODE) { item_set(&u->work, 1U, (uint16_t)((u->work.mode + 1U) % 3U)); }
            else if ((it->kind == (uint8_t)K_YN) || (it->kind == (uint8_t)K_AUTOMAN) || (it->kind == (uint8_t)K_LOADSRC)) item_set(&u->work, it->id, (uint16_t)(item_get(&u->work, it->id) ^ 1U));
            else if ((it->kind == (uint8_t)K_DATE) || (it->kind == (uint8_t)K_TIME)) clk_digit_inc(u, it->id);
            else digit_inc(u, it);
        }
        else if (ev == MK_STOP) { if ((it->kind == (uint8_t)K_NUM) || (it->kind == (uint8_t)K_DATE) || (it->kind == (uint8_t)K_TIME)) u->cursor = (uint8_t)((u->cursor + 1U) % it->digits); }
        break;
    }
    default: break;
    }
}

/* ---------------------------------------------------------------------------------------------------- the frame */
static uint8_t blink_on(const MUI_t *u, uint32_t now);
static void render_run(const MUI_t *u, const MUIRun_t *d, Frame_t f, uint32_t now)
{
    uint8_t p; (void)now;
    if (u->live->mode == (uint8_t)M_MODE_SINGLE)
    {
        uint8_t ok = d->have[0];
        if (u->page == 9U)
        {   /* SLAVE LOAD CAL (v0.9.45): the converter current the slave measures, and the meter value being typed. With the unit RUNNING and NO load, type 00.0 and press LOG: the peak now is the zero. With a known load, type the meter's value and press LOG: the gain.
             * START edits the digit, STOP moves the cursor, LOG saves, MENU goes on. The conversion is: load = (peak - zero) x gain. */
            char v[8]; uint32_t c; uint16_t pk = (uint16_t)((d->sl_pk_ca + 5U) / 10U), rm = (uint16_t)((d->sl_rms_ca + 5U) / 10U);
            if (u->cal_init != 0U) { ((MUI_t *)u)->cal_val = (pk > 9999U) ? 9999U : pk; ((MUI_t *)u)->cal_init = 0U; }
            f_center(f, 0U, "Slave Load Cal");
            f_put(f, 1U, 0U, "Peak"); tenths_put(f, 1U, 4U, pk, 5U, d->sl_valid); f_put(f, 1U, 9U, "A"); f_put(f, 1U, 11U, "RMS"); tenths_put(f, 1U, 14U, rm, 5U, d->sl_valid); f_put(f, 1U, 19U, "A");
            f_put(f, 2U, 0U, "Actual");
            v[0] = (char)('0' + ((u->cal_val / 1000U) % 10U)); v[1] = (char)('0' + ((u->cal_val / 100U) % 10U)); v[2] = (char)('0' + ((u->cal_val / 10U) % 10U)); v[3] = '.'; v[4] = (char)('0' + (u->cal_val % 10U)); v[5] = '\0';
            c = (u->cal_cursor == 3U) ? 4U : u->cal_cursor;
            if (blink_on(u, now) == 0U) v[c] = '_';
            f_put(f, 2U, 11U, v); f_put(f, 2U, 18U, "A");
            f_put(f, 3U, 0U, (d->sl_valid == 0U) ? "No converter reading" : ((d->sl_run == 0U) ? "Run the unit first" : "LOG=Save  MENU=Next"));
            return;
        }
        if ((u->page == 3U) || (u->page == 5U) || (u->page == 6U))
        {   /* LOAD CALIBRATION (page 3) and the two VOLTAGE CALIBRATION pages (5 = input, 6 = output): the reading, and the meter value being typed. START edits the digit, STOP moves the cursor, LOG saves, MENU leaves. */
            char v[8], un[2]; uint32_t c; uint16_t rd = (u->page == 3U) ? d->load_da[0] : ((u->page == 5U) ? d->vin_raw_dv : d->vout_raw_dv);
            un[0] = (u->page == 3U) ? 'A' : 'V'; un[1] = '\0';
            if (u->cal_init != 0U) { ((MUI_t *)u)->cal_val = (rd > 9999U) ? 9999U : rd; ((MUI_t *)u)->cal_init = 0U; }
            f_center(f, 0U, (u->page == 3U) ? "Load Calibration" : ((u->page == 5U) ? "I/P Calibration" : "O/P Calibration"));
            f_put(f, 1U, 0U, "Reading"); tenths_put(f, 1U, 11U, rd, 6U, (uint8_t)(ok && ((u->page != 3U) || (d->load_ok[0] != 0U)))); f_put(f, 1U, 18U, un);
            f_put(f, 2U, 0U, "Actual");
            v[0] = (char)('0' + ((u->cal_val / 1000U) % 10U)); v[1] = (char)('0' + ((u->cal_val / 100U) % 10U)); v[2] = (char)('0' + ((u->cal_val / 10U) % 10U)); v[3] = '.'; v[4] = (char)('0' + (u->cal_val % 10U)); v[5] = '\0';
            c = (u->cal_cursor == 3U) ? 4U : u->cal_cursor;                           /* the digit's place in the text: 0 1 2 . 4 */
            if (blink_on(u, now) == 0U) v[c] = '_';
            f_put(f, 2U, 11U, v); f_put(f, 2U, 18U, un);
            f_put(f, 3U, 0U, ((u->page != 3U) && (d->active != 0U)) ? "Press STOP first" : "LOG=Save  MENU=Next");
            return;
        }
        if (u->page != 0U) return;                                                  /* the monitor and diagnostics pages: the app draws them */
        f_put(f, 0U, 0U, "O/P Voltage"); int_put(f, 0U, 12U, d->vout_dv[0], 6U, ok); f_put(f, 0U, 19U, "V");
        f_put(f, 1U, 0U, "I/P Voltage"); int_put(f, 1U, 12U, d->vin_dv[0], 6U, ok); f_put(f, 1U, 19U, "V");
        f_put(f, 2U, 0U, "Load"); tenths_put(f, 2U, 5U, d->load_da[0], 5U, (uint8_t)(ok && d->load_ok[0])); f_put(f, 2U, 10U, "A");
        tenths_put(f, 2U, 12U, d->freq_chz / 10U, 6U, (uint8_t)(ok && d->freq_chz != 0U)); f_put(f, 2U, 18U, "Hz");
        f_put(f, 3U, 0U, (d->fault[0] != 0U) ? full_text(d->fault[0]) : ((ok == 0U) ? "Slave Link Lost" : (d->out_on ? "Output ON" : (((d->status != 0) && (d->status[0] != '\0')) ? d->status : "Output OFF"))));
        if ((d->wait_s[0] != 0U) && ((d->fault[0] == (uint8_t)MLOG_T_INRUSH) || (d->fault[0] == (uint8_t)MLOG_T_POWER_STAGE) || (d->fault[0] == (uint8_t)MLOG_T_HEAT)))
        {   /* an automatic restart is counting down: the short name at the left, "Rst m:ss" at the right (the fault code is in the log) */
            char w[10]; uint32_t m = d->wait_s[0] / 60U, sec = d->wait_s[0] % 60U, k = 0U, i;
            f_put(f, 3U, 0U, "                    ");
            f_put(f, 3U, 0U, (d->fault[0] == (uint8_t)MLOG_T_INRUSH) ? "Inrush" : ((d->fault[0] == (uint8_t)MLOG_T_POWER_STAGE) ? "Power Stage" : "Heat"));
            w[k++] = 'R'; w[k++] = 's'; w[k++] = 't'; w[k++] = ' ';
            if (m >= 10U) w[k++] = (char)('0' + ((m / 10U) % 10U));
            w[k++] = (char)('0' + (m % 10U)); w[k++] = ':'; w[k++] = (char)('0' + (sec / 10U)); w[k++] = (char)('0' + (sec % 10U)); w[k] = '\0';
            (void)i; f_put_right(f, 3U, w);
        }
        else if ((d->fault[0] == (uint8_t)MLOG_T_MAINS) || (d->fault[0] == (uint8_t)MLOG_T_POWER_STAGE) || (d->fault[0] == (uint8_t)MLOG_T_INRUSH))
        {   /* the slave's fault code next to the text: "Mains Unstable   F16" */
            if (d->fcode[0] != 0U) { char cd[4]; cd[0] = 'F'; cd[1] = (char)('0' + ((d->fcode[0] / 10U) % 10U)); cd[2] = (char)('0' + (d->fcode[0] % 10U)); cd[3] = '\0'; f_put_right(f, 3U, cd); }
        }
        return;
    }
    if (u->page == 0U)
    {   /* per-phase output voltage and load */
        static const char N[3] = { 'R', 'Y', 'B' };
        for (p = 0U; p < 3U; p++)
        {
            char c[3] = { N[p], ':', '\0' }; f_put(f, p, 0U, c);
            int_put(f, p, 3U, d->vout_dv[p], 4U, d->have[p]); f_put(f, p, 7U, "V");
            tenths_put(f, p, 12U, d->load_da[p], 5U, (uint8_t)(d->have[p] && d->load_ok[p])); f_put(f, p, 17U, "A");
        }
        f_put(f, 3U, 0U, d->out_on ? "Output ON" : (((d->status != 0) && (d->status[0] != '\0')) ? d->status : "Output OFF"));
    }
    else if (u->page == 1U)
    {   /* phase-to-phase and frequency */
        static const char *L[3] = { "R-Y", "Y-B", "B-R" };
        for (p = 0U; p < 3U; p++) { f_put(f, p, 0U, L[p]); f_put(f, p, 4U, ":"); int_put(f, p, 6U, d->vll_dv[p], 5U, (uint8_t)(d->vll_dv[p] != 0U)); f_put(f, p, 12U, "V"); }
        f_put(f, 3U, 0U, "Freq"); tenths_put(f, 3U, 6U, d->freq_chz / 10U, 6U, (uint8_t)(d->freq_chz != 0U)); f_put(f, 3U, 13U, "Hz");
    }
    else
    {   /* status */
        static const char N[3] = { 'R', 'Y', 'B' };
        for (p = 0U; p < 3U; p++)
        {
            char c[5] = { N[p], ' ', ':', ' ', '\0' }; char s[20]; f_put(f, p, 0U, c);
            if (d->have[p] == 0U && d->fault[p] == 0U) f_put(f, p, 4U, "---");
            else { short_text(s, d->fault[p], d->fcode[p], d->seq_state); f_put(f, p, 4U, s); }
        }
        f_put(f, 3U, 0U, mode_text(u->live->mode)); f_put_right(f, 3U, d->out_on ? "ON" : "OFF");
    }
}

static void render_log(const MUI_t *u, Frame_t f)
{
    MLogRec_t r; uint16_t n = MLog_Count(u->log);
    if (n == 0U) { f_put(f, 0U, 0U, "Fault Log"); f_put(f, 1U, 0U, "No Faults"); return; }
    if (MLog_Get(u->log, u->log_idx, &r) != 0) { f_put(f, 0U, 0U, "Fault Log"); f_put(f, 1U, 0U, "Unreadable"); return; }
    {   char b[12]; uint32_t c = 0U; static const char P[4] = { 'R', 'Y', 'B', '*' };
        f_put(f, 0U, 0U, "Fault Log "); fmt_zero(b, u->log_idx, 2U); f_put(f, 0U, 10U, b); f_put(f, 0U, 12U, "/"); fmt_zero(b, (uint32_t)(n - 1U), 2U); f_put(f, 0U, 13U, b);
        { char pc[3] = { P[r.phase & 3U], ' ', '\0' }; f_put(f, 1U, 0U, pc);
          if ((r.type == MLOG_T_MAINS) || (r.type == MLOG_T_POWER_STAGE))
          {   char cd[6]; f_put(f, 1U, 2U, (r.type == MLOG_T_MAINS) ? "Mains Unstable" : "Power Stage"); cd[0] = 'F'; cd[1] = (char)('0' + ((r.slave_code / 10U) % 10U)); cd[2] = (char)('0' + (r.slave_code % 10U)); cd[3] = '\0'; f_put_right(f, 1U, cd); }
          else f_put(f, 1U, 2U, full_text(r.type)); }
        f_put(f, 2U, 0U, "I"); int_put(f, 2U, 1U, r.vin_dv, 4U, 1U); f_put(f, 2U, 6U, "O"); int_put(f, 2U, 7U, r.vout_dv, 4U, 1U);
        f_put(f, 2U, 12U, "L"); tenths_put(f, 2U, 13U, r.load_da, 5U, 1U); f_put(f, 2U, 18U, "A");                  /* the load at the trip */
        if (MC_IsStamp(r.run_min) != 0U)
        {   /* the real-time clock stamp: "21/10/2026 14:32" */
            MDateTime_t dt; char t[20]; MC_FromMinutes(MC_StampMinutes(r.run_min), &dt);
            t[0] = (char)('0' + (dt.day / 10U)); t[1] = (char)('0' + (dt.day % 10U)); t[2] = '/'; t[3] = (char)('0' + (dt.month / 10U)); t[4] = (char)('0' + (dt.month % 10U)); t[5] = '/';
            t[6] = (char)('0' + ((dt.year / 1000U) % 10U)); t[7] = (char)('0' + ((dt.year / 100U) % 10U)); t[8] = (char)('0' + ((dt.year / 10U) % 10U)); t[9] = (char)('0' + (dt.year % 10U)); t[10] = ' ';
            t[11] = (char)('0' + (dt.hour / 10U)); t[12] = (char)('0' + (dt.hour % 10U)); t[13] = ':'; t[14] = (char)('0' + (dt.minute / 10U)); t[15] = (char)('0' + (dt.minute % 10U)); t[16] = '\0';
            f_put(f, 3U, 0U, t);
        }
        else
        { uint32_t h = r.run_min / 60U, m = r.run_min % 60U; uint32_t k, n2 = 0U; char t[16];
          f_put(f, 3U, 0U, "Run");                                                                                       /* no clock time: how long the unit had been running */
          { char d[12]; uint32_t q = h; if (q == 0U) d[n2++] = '0'; while (q > 0U) { d[n2++] = (char)('0' + (q % 10U)); q /= 10U; } for (k = 0U; k < n2; k++) t[k] = d[n2 - 1U - k]; }
          t[n2++] = 'h'; t[n2++] = (char)('0' + (m / 10U)); t[n2++] = (char)('0' + (m % 10U)); t[n2++] = 'm'; t[n2] = '\0'; f_put_right(f, 3U, t); }
        (void)c; }
}

static uint8_t blink_on(const MUI_t *u, uint32_t now) { return (uint8_t)((((uint32_t)(now - u->t_blink)) / MUI_BLINK_MS) % 2U == 0U); }

static void render_set(const MUI_t *u, Frame_t f, uint32_t now)
{
    const MItem_t *it = item_of(u->item); char v[24]; uint32_t n = 0U, i;
    f_center(f, 0U, "Set Mode");
    f_center(f, 1U, it->name);
    if (is_action(it->id)) { const char *y = (u->yn != 0U) ? "Yes" : "No"; for (i = 0U; y[i] != '\0'; i++) v[n++] = y[i]; v[n] = '\0'; f_center(f, 2U, v); return; }
    if (it->kind == (uint8_t)K_MODE) { f_center(f, 2U, mode_text(u->work.mode)); return; }
    if (it->kind == (uint8_t)K_YN) { f_center(f, 2U, (item_get(&u->work, it->id) != 0U) ? "Yes" : "No"); return; }
    if (it->kind == (uint8_t)K_AUTOMAN) { f_center(f, 2U, (u->work.restart_auto != 0U) ? "Auto" : "Manual"); return; }
    if (it->kind == (uint8_t)K_LOADSRC) { f_center(f, 2U, (u->work.load_src != 0U) ? "Slave" : "Master CT"); return; }
    if ((it->kind == (uint8_t)K_DATE) || (it->kind == (uint8_t)K_TIME))
    {   /* dd/mm/yyyy or HH:MM, centred, the blinking underscore under the digit being edited */
        const MDateTime_t *t = &u->clk_work; char s[12]; uint32_t len, pos, col0;
        if (it->kind == (uint8_t)K_DATE)
        {
            s[0] = (char)('0' + ((t->day / 10U) % 10U)); s[1] = (char)('0' + (t->day % 10U)); s[2] = '/'; s[3] = (char)('0' + ((t->month / 10U) % 10U)); s[4] = (char)('0' + (t->month % 10U)); s[5] = '/';
            s[6] = (char)('0' + ((t->year / 1000U) % 10U)); s[7] = (char)('0' + ((t->year / 100U) % 10U)); s[8] = (char)('0' + ((t->year / 10U) % 10U)); s[9] = (char)('0' + (t->year % 10U)); s[10] = '\0'; len = 10U;
            pos = (u->cursor < 2U) ? u->cursor : ((u->cursor < 4U) ? (u->cursor + 1U) : (u->cursor + 2U));
        }
        else
        {
            s[0] = (char)('0' + ((t->hour / 10U) % 10U)); s[1] = (char)('0' + (t->hour % 10U)); s[2] = ':'; s[3] = (char)('0' + ((t->minute / 10U) % 10U)); s[4] = (char)('0' + (t->minute % 10U)); s[5] = '\0'; len = 5U;
            pos = (u->cursor < 2U) ? u->cursor : (u->cursor + 1U);
        }
        f_center(f, 2U, s); col0 = (MUI_COLS - len) / 2U;
        if (blink_on(u, now) != 0U) f_put(f, 3U, col0 + pos, "_");
        return;
    }
    {   char z[8]; fmt_zero(z, item_get(&u->work, it->id), it->digits);
        for (i = 0U; z[i] != '\0'; i++) v[n++] = z[i];
        if (it->unit != ' ') { v[n++] = ' '; v[n++] = it->unit; }
        if ((it->id == 4U) && (u->work.maxcorr_v == 0U)) { const char *ns = " Not Set"; for (i = 0U; ns[i] != '\0'; i++) v[n++] = ns[i]; }
        v[n] = '\0'; f_center(f, 2U, v);
        if (blink_on(u, now) != 0U) f_put(f, 3U, ((MUI_COLS - n) / 2U) + u->cursor, "_");        /* a blinking underscore under the digit being edited */
    }
}

void MUI_Tick(MUI_t *u, const MUIRun_t *d, uint32_t now, Frame_t out)
{
    u->unit_active = d->active;                       /* the Voltage Calibration pages: STOP stops a running unit */
    /* timers */
    if ((u->screen == (uint8_t)MUI_SPLASH) && ((uint32_t)(now - u->t0) >= MUI_SPLASH_MS)) { u->screen = (uint8_t)MUI_RUN; u->t_page = now; u->t_key = now; }
    if ((u->msg_on != 0U) && ((uint32_t)(now - u->t_msg) >= MUI_MSG_MS)) { u->msg_on = 0U; if (u->return_to_run != 0U) u->screen = (uint8_t)MUI_RUN; }
    if (((u->screen == (uint8_t)MUI_SET) || (u->screen == (uint8_t)MUI_PW)) && ((uint32_t)(now - u->t_key) >= MUI_SET_IDLE_MS))
        leave_set_discard(u, now, "Timeout -", "Not Saved");
    if ((u->screen == (uint8_t)MUI_LOG) && ((uint32_t)(now - u->t_key) >= MUI_SET_IDLE_MS)) u->screen = (uint8_t)MUI_RUN;
    if (u->screen == (uint8_t)MUI_RUN)
    {
        uint8_t shown = 0U, p;
        for (p = 0U; p < 3U; p++) if (d->fault[p] != 0U && MF_Rank(d->fault[p]) >= MF_Rank(shown)) shown = d->fault[p];
        if ((shown != 0U) && (shown != u->prev_shown) && (u->live->mode != (uint8_t)M_MODE_SINGLE)) { u->page = 2U; u->t_page = now; }    /* a new fault jumps to the status page */
        if (u->live->mode == (uint8_t)M_MODE_SINGLE)
        {   /* a new fault, or a minute without a key, takes a service page back to THE Run screen, where the parameters and the fault are */
            if (u->page != 0U && (((shown != 0U) && (shown != u->prev_shown)) || ((uint32_t)(now - u->t_key) >= MUI_SERVICE_IDLE_MS))) u->page = 0U;
        }
        u->prev_shown = shown;
        if ((u->live->mode != (uint8_t)M_MODE_SINGLE) && (u->live->scroll != 0U) && ((uint32_t)(now - u->t_page) >= ((uint32_t)u->live->scroll_time * 1000UL)))
        { u->page = (uint8_t)((u->page + 1U) % 3U); u->t_page = now; }
    }
    /* the frame */
    f_clear(out);
    if (u->msg_on != 0U) { f_center(out, 1U, u->msg[0]); f_center(out, 2U, u->msg[1]); return; }
    switch (u->screen)
    {
    case MUI_SPLASH: f_put(out, 0U, 0U, "Welcome"); f_put(out, 1U, 0U, "Static Voltage"); f_put(out, 2U, 0U, mode_text(u->live->mode)); f_put(out, 3U, 0U, "Wait..."); break;
    case MUI_RUN:    render_run(u, d, out, now); break;
    case MUI_LOG:    render_log(u, out); break;
    case MUI_PW:
    {   char b[4]; f_center(out, 0U, "Enter Password"); b[0] = (char)('0' + u->pw[0]); b[1] = (char)('0' + u->pw[1]); b[2] = (char)('0' + u->pw[2]); b[3] = '\0'; f_center(out, 2U, b);
        if (blink_on(u, now) != 0U) f_put(out, 3U, 8U + u->cursor, "_");
        break; }
    case MUI_SET:    render_set(u, out, now); break;
    default: break;
    }
}
