/* Tests of the master's key handling (m_keys) and screens (m_ui): exact screen text, Set-Mode editing and validation, password, timeouts, the fault-log viewer, and a fuzz run. */
#include <stdio.h>
/* the limits the tests were written for (valid in the new limits model with Max Correction not set); the NEW defaults have their own tests */
#define LEGACY_LIMITS(s) do { (s).ip_lv = 180U; (s).ip_hv = 280U; (s).op_lv = 200U; (s).op_hv = 250U; } while (0)
#include <string.h>
#include <stdint.h>
#include "m_ui.h"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* ================================================================================================ keys */
static MKeys_t K; static uint32_t kt; static MKEvent_t kev[64]; static int nkev;
static void kreset(void) { MK_Init(&K); kt = 1000U; nkev = 0; }
static void khold(uint8_t m, uint8_t s, uint8_t st, uint8_t l, uint32_t ms)
{
    uint32_t e = kt + ms;
    while (kt < e) { MKEvent_t ev = MK_Update(&K, m, s, st, l, kt); if (ev != MK_NONE && nkev < 64) kev[nkev++] = ev; kt += 5U; }
}
static int kcount(MKEvent_t t) { int i, c = 0; for (i = 0; i < nkev; i++) if (kev[i] == t) c++; return c; }

static void t_keys(void)
{
    uint32_t i;
    kreset(); khold(0, 1, 0, 0, 5000U); khold(0, 0, 0, 0, 200U);
    CHECK(nkev == 1 && kev[0] == MK_START, "START held 5 s gives ONE event, on the press (%d events)", nkev);
    kreset(); khold(0, 0, 1, 0, 300U); khold(0, 0, 0, 0, 200U);
    CHECK(nkev == 1 && kev[0] == MK_STOP, "STOP gives one event on the press");
    kreset(); khold(1, 0, 0, 0, 120U);
    CHECK(nkev == 0, "MENU alone gives nothing while it is down");
    khold(0, 0, 0, 0, 100U);
    CHECK(nkev == 1 && kev[0] == MK_MENU_SHORT, "MENU gives a short press on RELEASE");
    kreset(); khold(0, 0, 0, 1, 120U); khold(0, 0, 0, 0, 100U);
    CHECK(nkev == 1 && kev[0] == MK_LOG_SHORT, "LOG gives a short press on release");
    /* contact bounce shorter than the debounce time */
    kreset(); for (i = 0U; i < 6U; i++) { khold(0, 1, 0, 0, 10U); khold(0, 0, 0, 0, 10U); } khold(0, 0, 0, 0, 200U);
    CHECK(nkev == 0, "chatter shorter than the debounce time is not a press");
    /* MENU + LOG */
    kreset(); khold(1, 0, 0, 1, 1900U); khold(0, 0, 0, 0, 300U);
    CHECK(nkev == 0, "MENU+LOG for 1.9 s then released: no combo AND no short press from either key");
    kreset(); khold(1, 0, 0, 1, 2200U);
    CHECK(kcount(MK_COMBO) == 1 && nkev == 1, "MENU+LOG held: exactly ONE combo event (%d events)", nkev);
    khold(1, 0, 0, 1, 5000U);
    CHECK(kcount(MK_COMBO) == 1, "holding on does not repeat it");
    khold(1, 0, 0, 0, 200U); khold(0, 0, 0, 0, 200U);
    CHECK(nkev == 1, "releasing LOG then MENU after the combo gives no short press");
    khold(0, 1, 0, 0, 200U);
    CHECK(kcount(MK_START) == 1, "after both are released the keys work again");
    kreset(); khold(1, 0, 0, 0, 300U); khold(1, 0, 0, 1, 2300U);
    CHECK(kcount(MK_COMBO) == 1 && kcount(MK_MENU_SHORT) == 0, "MENU first, then LOG: the 2 s count starts when BOTH are down");
    /* START / STOP are ignored while MENU or LOG is down: forming the combo cannot start or stop the output */
    kreset(); khold(1, 0, 0, 0, 100U); khold(1, 1, 0, 0, 200U); khold(1, 0, 1, 0, 200U); khold(0, 0, 0, 0, 200U);
    CHECK(kcount(MK_START) == 0 && kcount(MK_STOP) == 0, "START and STOP pressed while MENU is down do nothing");
    kreset(); khold(0, 0, 0, 1, 100U); khold(0, 1, 0, 1, 200U); khold(0, 0, 0, 0, 200U);
    CHECK(kcount(MK_START) == 0, "nor while LOG is down");
    /* the combo twice in a row */
    kreset(); khold(1, 0, 0, 1, 2200U); khold(0, 0, 0, 0, 300U); khold(1, 0, 0, 1, 2200U);
    CHECK(kcount(MK_COMBO) == 2, "the combination can be used again after a full release");
    printf("  keys: START/STOP on press, MENU/LOG short on release, MENU+LOG 2 s once, no short or START/STOP leaks, bounce rejected\n");
}

static uint32_t rs = 31337U;
static uint32_t rnd(void) { rs = (rs * 1664525U) + 1013904223U; return rs >> 8; }

static void t_keys_fuzz(void)
{
    uint32_t i, bad_combo = 0U, bad_leak = 0U, ev_n = 0U; uint8_t lvl[4] = { 0, 0, 0, 0 }; int combos_since_release = 0;
    kreset();
    for (i = 0U; i < 3000000U; i++)
    {
        MKEvent_t e;
        if ((rnd() % 40U) == 0U) lvl[rnd() % 4U] ^= 1U;
        if ((rnd() % 4000U) == 0U) { lvl[0] = lvl[3] = 1U; }                    /* often form a combination */
        e = MK_Update(&K, lvl[0], lvl[1], lvl[2], lvl[3], kt); kt += 5U;
        if (e == MK_COMBO) { combos_since_release++; if (combos_since_release > 1) bad_combo++; ev_n++; }
        if ((e == MK_START || e == MK_STOP) && (K.stable[0] || K.stable[3])) bad_leak++;
        if (!K.stable[0] && !K.stable[3] && !K.raw[0] && !K.raw[3]) combos_since_release = 0;
    }
    CHECK(bad_combo == 0U && ev_n > 50U, "3,000,000 random key steps: never two combos without both keys released between them (%u), %u combos seen", (unsigned)bad_combo, (unsigned)ev_n);
    CHECK(bad_leak == 0U, "and START/STOP never fire while MENU or LOG is down (%u)", (unsigned)bad_leak);
    printf("  keys fuzz: 3,000,000 random steps, %u combos: invariants hold\n", (unsigned)ev_n);
}

/* ================================================================================================ the UI */
static uint8_t fm[4096];
static int f_read(uint32_t a, uint8_t *b, uint32_t n) { memcpy(b, &fm[a], n); return 0; }
static int f_prog(uint32_t a, const uint8_t *b, uint32_t n) { uint32_t i; for (i = 0U; i < n; i++) fm[a + i] &= b[i]; return 0; }
static int f_erase(uint32_t a) { memset(&fm[a], 0xFF, MFLASH_PAGE); return 0; }
static const MFlash_t FL = { f_read, f_prog, f_erase };

static MSettings_t S; static MLog_t LG; static MUI_t U; static MUIRun_t D; static uint32_t T; static char F[4][21];
static MLimits_t LIM;

static void ui_reset(uint8_t mode)
{
    int p;
    memset(fm, 0xFF, sizeof fm); MLog_Init(&LG, &FL, 0U, MFLASH_PAGE);
    MS_Defaults(&S); LEGACY_LIMITS(S); S.mode = mode;
    memset(&D, 0, sizeof D);
    for (p = 0; p < 3; p++) { D.have[p] = 1U; D.load_ok[p] = 1U; D.vin_dv[p] = 2015U; D.vout_dv[p] = 2300U; D.load_da[p] = 123U; D.vll_dv[p] = 4001U; }
    D.freq_chz = 5000U;
    T = 10000U; MUI_Init(&U, &S, &LG, T); MUI_SetFeatures(&U, 0U);
    LIM.win_lo_v = 80U; LIM.win_hi_v = 310U; LIM.out_guard_v = 265U; MUI_SetLimits(&U, &LIM);
}
static void tick(uint32_t ms) { uint32_t e = T + ms; while (T < e) { T += 50U; MUI_Tick(&U, &D, T, F); } }
static void key(MKEvent_t e) { MUI_Key(&U, e, T); MUI_Tick(&U, &D, T, F); }
static void to_run(void) { tick(MUI_SPLASH_MS + 100U); }
static const char *row(int r) { static char b[4][22]; int n; strcpy(b[r], F[r]); n = (int)strlen(b[r]); while (n > 0 && b[r][n - 1] == ' ') b[r][--n] = '\0'; return b[r]; }
#define ROW(r, txt) CHECK(strcmp(row(r), txt) == 0, "row %d is '%s' (expected '%s')", r, row(r), txt)
static void enter_set(void) { key(MK_COMBO); tick(100U); }
/* Set-Mode text is centred: the expected row is the text with (20 - length) / 2 spaces in front */
static int crow_ok(int r, const char *txt) { char b[24]; const char *g = row(r); int n = (int)strlen(txt), i, c = (20 - n) / 2; for (i = 0; i < c; i++) b[i] = ' '; strcpy(&b[c], txt);
    if (strlen(g) != strlen(b)) return 0;
    for (i = 0; b[i] != '\0'; i++) if ((g[i] != b[i]) && !((r == 2) && (g[i] == '_') && (b[i] >= '0') && (b[i] <= '9'))) return 0;      /* v0.9.57: the digit being edited blinks in place: an underscore stands for it half the time */
    return 1; }
#define CROW(r, txt) CHECK(crow_ok(r, txt), "row %d is '%s' (expected '%s' centred)", r, row(r), txt)
/* the cursor (v0.9.57): digit `idx` of a value `len` characters long, centred on row 2, BLINKS IN PLACE: shown for 0.5 s after a key, then an underscore for 0.5 s; row 3 stays empty */
#define CUR(len, idx) do { int cu_c = ((20 - (len)) / 2) + (idx); char cu_d = F[2][cu_c]; tick(520U); CHECK(F[2][cu_c] == '_' && cu_d != '_' && strchr(F[3], '_') == NULL && strchr(F[2], '_') == &F[2][cu_c], "cursor: the digit at column %d blinks in place (row 2 '%s', row 3 '%s')", cu_c, F[2], F[3]); tick(500U); } while (0)
/* walk the items with MENU until Set-Mode is left; returns how many were visited, their numbers in ids[] */
static int walk(uint8_t *ids) { int n = 0; while (MUI_SetModeOpen(&U) && n < 30) { ids[n++] = U.item; key(MK_MENU_SHORT); tick(1600U); } return n; }

static void t_screens(void)
{
    int r, c, ok = 1;
    ui_reset(M_MODE_SINGLE); MUI_Tick(&U, &D, T, F);
    ROW(0, "Welcome"); ROW(1, "Static Voltage"); ROW(2, "Single Phase"); ROW(3, "Wait...");
    tick(2800U); ROW(0, "Welcome"); tick(300U);
    ROW(0, "O/P Voltage    230 V"); ROW(1, "I/P Voltage    202 V"); ROW(2, "Load  12.3A   50.0Hz"); ROW(3, "Output OFF");
    D.out_on = 1U; D.vout_dv[0] = 2304U; tick(100U); ROW(0, "O/P Voltage    230 V"); ROW(3, "Output ON");
    D.have[0] = 0U; tick(100U); ROW(0, "O/P Voltage    --- V"); ROW(3, "Slave Link Lost");
    D.have[0] = 1U; D.fault[0] = MLOG_T_IP_LV; tick(100U); ROW(3, "Input LV");
    D.fault[0] = MLOG_T_MAINS; D.fcode[0] = 8U; tick(100U); ROW(3, "Mains Unstable   F08");
    D.fault[0] = MLOG_T_POWER_STAGE; D.fcode[0] = 12U; tick(100U); ROW(3, "Power Stage FaultF12");
    D.fault[0] = MLOG_T_FLIP; tick(100U); ROW(3, "Transformer Flip");
    D.fault[0] = MLOG_T_RETUNE; tick(100U); ROW(3, "Re-tune Required");
    D.fault[0] = MLOG_T_LINK_LOST; tick(100U); ROW(3, "Slave Link Lost");
    /* single-phase: ONE Run screen. MENU does nothing on it; the service pages (the app draws them) are reached through the log: LOG, then MENU */
    D.fault[0] = 0U; tick(100U); CHECK(!MUI_ShowsMonitor(&U), "the Run screen is shown");
    key(MK_MENU_SHORT); tick(100U); CHECK(!MUI_ShowsMonitor(&U) && !MUI_ShowsDiag(&U) && !MUI_ShowsCal(&U), "MENU does nothing on the Run screen"); ROW(0, "O/P Voltage    230 V");
    key(MK_LOG_SHORT); key(MK_MENU_SHORT); CHECK(MUI_ShowsMonitor(&U) && !MUI_ShowsDiag(&U), "LOG then MENU: the monitor page");
    key(MK_MENU_SHORT); CHECK(!MUI_ShowsMonitor(&U) && MUI_ShowsDiag(&U), "MENU again: the control diagnostics page");
    key(MK_MENU_SHORT); CHECK(MUI_ShowsCal(&U) && !MUI_ShowsDiag(&U), "a third press: the Load Calibration page");
    key(MK_MENU_SHORT); CHECK(MUI_ShowsClock(&U) && !MUI_ShowsCal(&U), "a fourth press: the Clock page");
    key(MK_MENU_SHORT); CHECK(MUI_ShowsVCal(&U) == 1U, "a fifth press: the I/P Calibration page");
    key(MK_MENU_SHORT); CHECK(MUI_ShowsVCal(&U) == 2U, "a sixth: the O/P Calibration page");
    key(MK_MENU_SHORT); CHECK(MUI_ShowsIout(&U) && MUI_ShowsVCal(&U) == 0U, "a seventh: the Converter Current page");
    key(MK_MENU_SHORT); CHECK(MUI_ShowsProfile(&U) && !MUI_ShowsIout(&U) && MUI_ShowsVCal(&U) == 0U, "an eighth: the Transformer Profile page");
    key(MK_MENU_SHORT); CHECK(strstr(row(0), "Slave Load Cal") != NULL && U.page == 9U, "a ninth: the Slave Load Cal page (drawn by the UI)");
    key(MK_MENU_SHORT); CHECK(!MUI_ShowsMonitor(&U) && !MUI_ShowsDiag(&U) && !MUI_ShowsCal(&U) && !MUI_ShowsClock(&U) && MUI_ShowsVCal(&U) == 0U && !MUI_ShowsIout(&U) && !MUI_ShowsProfile(&U) && U.page == 0U, "and a tenth: back to the Run screen"); ROW(0, "O/P Voltage    230 V");
    /* a service page returns to the Run screen by itself after a minute, and at once when a fault appears */
    key(MK_LOG_SHORT); key(MK_MENU_SHORT); CHECK(MUI_ShowsMonitor(&U), "on the monitor page again");
    tick(59000U); CHECK(MUI_ShowsMonitor(&U), "59 s without a key: still there"); tick(2000U); CHECK(!MUI_ShowsMonitor(&U), "61 s without a key: back on the Run screen");
    key(MK_LOG_SHORT); key(MK_MENU_SHORT); key(MK_MENU_SHORT); CHECK(MUI_ShowsDiag(&U), "the diagnostics page");
    D.fault[0] = MLOG_T_MAINS; D.fcode[0] = 16U; tick(100U); CHECK(!MUI_ShowsDiag(&U), "a new fault takes it back to the Run screen"); ROW(3, "Mains Unstable   F16");
    D.fault[0] = 0U; D.fcode[0] = 0U; tick(100U);
    D.load_ok[0] = 0U; tick(100U); ROW(2, "Load  --.-A   50.0Hz"); D.load_ok[0] = 1U; tick(100U);
    /* every frame is 4 rows of 20 printable characters */
    for (r = 0; r < 4; r++) { if (strlen(F[r]) != 20U) ok = 0; for (c = 0; c < 20; c++) if (F[r][c] < 32 || F[r][c] > 126) ok = 0; }
    CHECK(ok, "every row is exactly 20 printable characters");
    /* three-phase */
    ui_reset(M_MODE_INDUSTRIAL); S.scroll = 0U; MUI_Tick(&U, &D, T, F); ROW(2, "Industrial"); to_run();
    ROW(0, "R: 230V       12.3A"); ROW(1, "Y: 230V       12.3A"); ROW(2, "B: 230V       12.3A"); ROW(3, "Output OFF");
    key(MK_MENU_SHORT);
    CHECK(strcmp(row(0), "R I/P :   202 V") == 0 && strcmp(row(1), "Y I/P :   202 V") == 0 && strcmp(row(2), "B I/P :   202 V") == 0 && strcmp(row(3), "Input Voltage") == 0, "v0.9.57, page 2: the input voltages ('%s' / '%s')", row(0), row(3));
    key(MK_MENU_SHORT);
    CHECK(strstr(row(0), "R-Y") != NULL && strstr(row(0), "400") != NULL && strstr(row(3), "Freq") != NULL && strstr(row(3), "50.0") != NULL, "page 3: phase-to-phase and frequency ('%s' / '%s')", row(0), row(3));
    D.vll_dv[1] = 0U; tick(100U); CHECK(strstr(row(1), "Y-B") != NULL && strstr(row(1), "400") == NULL && strstr(row(1), "-") != NULL, "a missing phase shows dashes: '%s'", row(1));
    key(MK_MENU_SHORT);
    ROW(0, "R : OK"); ROW(3, "Industrial       OFF");
    key(MK_MENU_SHORT); ROW(0, "R: 230V       12.3A");
    CHECK(1, "the pages wrap");
    D.have[1] = 0U; key(MK_MENU_SHORT); CHECK(strstr(row(1), "Y I/P") != NULL && strstr(row(1), "202") == NULL && strstr(row(1), "-") != NULL, "a missing phase: dashes on the input page ('%s')", row(1)); key(MK_MENU_SHORT); key(MK_MENU_SHORT); ROW(1, "Y : ---");
    printf("  screens: splash, single-phase page and every fault text, Industrial pages, a missing phase or link\n");
}


/* ---- Date and Time in Set-Mode, and the time stamp in the fault log ---- */
static void goto_item(uint8_t id) { int n; for (n = 0; n < 40 && U.item != id; n++) key(MK_MENU_SHORT); }
static void with_clock(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi, uint8_t valid) { MDateTime_t c; c.year = y; c.month = mo; c.day = d; c.hour = h; c.minute = mi; MUI_SetClock(&U, &c, valid); }

static void t_clock_items(void)
{
    int i, n = 0; uint8_t ids[40];
    /* the order of the items: ... Restart Mode (17), Restart Time (18), Date (23), Time (24), Re-tune (19) ... Factory Reset (22) last. The ids stay what they were. */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2026U, 10U, 21U, 14U, 32U, 1U); enter_set();
    do { ids[n++] = U.item; key(MK_MENU_SHORT); } while (U.item != ids[0] && n < 40 && MUI_SetModeOpen(&U));
    { int p18 = -1, p23 = -1, p24 = -1, p19 = -1; for (i = 0; i < n; i++) { if (ids[i] == 18U) p18 = i; if (ids[i] == 23U) p23 = i; if (ids[i] == 24U) p24 = i; if (ids[i] == 19U) p19 = i; }
      CHECK(p18 >= 0 && p23 == p18 + 1 && p24 == p23 + 1 && p19 == p24 + 1 && ids[n - 1] == 22U, "the order is ... Restart Time, Date, Time, Re-tune ... Factory Reset (last)"); }
    /* the items start from the clock and are drawn centred */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2026U, 10U, 21U, 14U, 32U, 1U); enter_set(); goto_item(23U);
    CROW(1, "Date"); CROW(2, "21/10/2026"); key(MK_STOP); tick(520U);
    CHECK(F[2][6] == '_' && strchr(F[3], '_') == NULL, "v0.9.57: the Date digit being edited blinks in place ('%s' / '%s')", F[2], F[3]); tick(500U); CHECK(F[2][6] == '1', "and comes back ('%s')", F[2]);
    key(MK_STOP); key(MK_STOP); key(MK_STOP); key(MK_STOP); key(MK_STOP); key(MK_STOP); key(MK_STOP);      /* round the eight digits back to the first */
    key(MK_MENU_SHORT); CROW(1, "Time"); CROW(2, "14:32");
    /* a clock that was never set: the entry starts from a placeholder, 01/01/2026 00:00 */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2000U, 1U, 1U, 0U, 0U, 0U); enter_set(); goto_item(23U); CROW(2, "01/01/2026"); key(MK_MENU_SHORT); CROW(2, "00:00");
    /* editing: START adds one to the digit under the cursor, STOP moves the cursor over the digits (the separators are skipped), 9 wraps to 0 */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2026U, 10U, 21U, 14U, 32U, 1U); enter_set(); goto_item(23U);
    key(MK_START); CROW(2, "31/10/2026");                                                             /* the day's tens: 2 -> 3 */
    key(MK_STOP);                                                                                      /* the cursor moves to the day's units */
    for (i = 0; i < 9; i++) { key(MK_START); } CROW(2, "30/10/2026");                                       /* the day's units: 1 + 9 presses wraps to 0 */
    key(MK_STOP); key(MK_START); CROW(2, "30/20/2026");                                                 /* the cursor moves to the month's tens: 1 -> 2 */
    key(MK_MENU_SHORT); CROW(1, "Invalid Value"); tick(1600U); CROW(1, "Date"); CROW(2, "21/10/2026");
    CHECK(!MUI_Take(&U, MUI_REQ_SET_CLOCK), "month 20 does not exist: refused, the whole date put back, still on the item");
    /* a day that does not exist in that month: 30/04 + 1 = 31/04 */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2026U, 4U, 30U, 9U, 0U, 1U); enter_set(); goto_item(23U);
    key(MK_STOP); key(MK_START); CROW(2, "31/04/2026"); key(MK_MENU_SHORT); CROW(1, "Invalid Value"); tick(1600U); CROW(2, "30/04/2026");
    CHECK(1, "31 April is refused");
    /* leap years: 29/02/2024 is good, 29/02/2025 is not */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2024U, 2U, 28U, 9U, 0U, 1U); enter_set(); goto_item(23U);
    key(MK_STOP); key(MK_START); CROW(2, "29/02/2024"); key(MK_MENU_SHORT); CROW(1, "Time");
    CHECK(1, "29 February 2024 is accepted: it moves on to Time");
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2025U, 2U, 28U, 9U, 0U, 1U); enter_set(); goto_item(23U);
    key(MK_STOP); key(MK_START); CROW(2, "29/02/2025"); key(MK_MENU_SHORT); CROW(1, "Invalid Value");
    CHECK(1, "29 February 2025 is refused");
    /* a time: 14:32 -> 24:32 refused, 14:32 -> 15:32 accepted */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2026U, 10U, 21U, 14U, 32U, 1U); enter_set(); goto_item(24U);
    key(MK_START); CROW(2, "24:32"); key(MK_MENU_SHORT); CROW(1, "Invalid Value"); tick(1600U); CROW(1, "Time"); CROW(2, "14:32");
    key(MK_STOP); key(MK_START); CROW(2, "15:32");                                                       /* the hour's units: 4 -> 5 */
    key(MK_MENU_SHORT); CROW(1, "Re-tune");
    CHECK(1, "15:32 is accepted: it moves on to Re-tune");
    printf("  Date and Time items: order, display, a clock never set, digit editing with wrap, impossible dates, months, leap years and times refused and put back\n");
}


static void add_log(uint8_t type, uint8_t ph, uint8_t code, uint16_t vin, uint16_t vout, uint16_t load, uint32_t min);
static int clk_is(const MDateTime_t *c, uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi) { return c->year == y && c->month == mo && c->day == d && c->hour == h && c->minute == mi; }
static void t_clock_save(void)
{
    /* passing through Date and Time without typing changes nothing: the settings are saved, the clock is NOT touched */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2026U, 10U, 21U, 14U, 32U, 1U); enter_set(); goto_item(23U); key(MK_MENU_SHORT); key(MK_MENU_SHORT);
    key(MK_COMBO); tick(100U);
    CHECK(!MUI_SetModeOpen(&U) && !MUI_Take(&U, MUI_REQ_SET_CLOCK) && MUI_Take(&U, MUI_REQ_SAVE), "visiting Date and Time without typing: the settings are saved, the clock is NOT touched");
    /* a new date, committed with MENU, then saved with MENU+LOG: ONE request, with the new date and the OLD time */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2026U, 10U, 21U, 14U, 32U, 1U); enter_set(); goto_item(23U);
    key(MK_START); key(MK_MENU_SHORT); CROW(1, "Time"); key(MK_COMBO); tick(100U);
    CHECK(!MUI_SetModeOpen(&U) && MUI_Take(&U, MUI_REQ_SET_CLOCK) && clk_is(MUI_ClockToSet(&U), 2026U, 10U, 31U, 14U, 32U) && !MUI_Take(&U, MUI_REQ_SET_CLOCK), "a new date saved: one request, 31/10/2026 14:32 (the time untouched)");
    CHECK(MUI_Take(&U, MUI_REQ_SAVE), "and the settings are saved with it");
    /* a new time */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2026U, 10U, 21U, 14U, 32U, 1U); enter_set(); goto_item(24U);
    key(MK_STOP); key(MK_START); key(MK_MENU_SHORT); key(MK_COMBO); tick(100U);
    CHECK(MUI_Take(&U, MUI_REQ_SET_CLOCK) && clk_is(MUI_ClockToSet(&U), 2026U, 10U, 21U, 15U, 32U), "a new time saved: 21/10/2026 15:32 (the date untouched)");
    /* typed but not committed with MENU, saved straight away with MENU+LOG: the typed value counts, as for every other item */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2026U, 10U, 21U, 14U, 32U, 1U); enter_set(); goto_item(23U);
    key(MK_START); key(MK_COMBO); tick(100U);
    CHECK(!MUI_SetModeOpen(&U) && MUI_Take(&U, MUI_REQ_SET_CLOCK) && clk_is(MUI_ClockToSet(&U), 2026U, 10U, 31U, 14U, 32U), "a date typed and saved with MENU+LOG at once: it is used");
    /* an impossible one typed, then MENU+LOG: refused, still in Set-Mode, nothing requested */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2026U, 10U, 21U, 14U, 32U, 1U); enter_set(); goto_item(23U);
    key(MK_STOP); key(MK_STOP); key(MK_START); CROW(2, "21/20/2026"); key(MK_COMBO);
    CHECK(MUI_SetModeOpen(&U) && strstr(row(1), "Invalid Value") != NULL && !MUI_Take(&U, MUI_REQ_SET_CLOCK) && !MUI_Take(&U, MUI_REQ_SAVE), "month 20 typed, MENU+LOG: refused ('%s'), nothing saved, nothing requested", row(1));
    /* Discard Changes: the typed date is NOT written */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2026U, 10U, 21U, 14U, 32U, 1U); enter_set(); goto_item(23U);
    key(MK_START); key(MK_MENU_SHORT); goto_item(21U); CROW(1, "Discard Changes"); key(MK_START); key(MK_MENU_SHORT); tick(100U);
    CHECK(!MUI_SetModeOpen(&U) && !MUI_Take(&U, MUI_REQ_SET_CLOCK) && !MUI_Take(&U, MUI_REQ_SAVE), "Discard Changes: the typed date is not written to the clock");
    /* the 30 s idle timeout: not written either */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2026U, 10U, 21U, 14U, 32U, 1U); enter_set(); goto_item(23U);
    key(MK_START); key(MK_MENU_SHORT); tick(31000U);
    CHECK(!MUI_SetModeOpen(&U) && !MUI_Take(&U, MUI_REQ_SET_CLOCK), "the 30 s idle timeout: not written");
    /* the next Set-Mode session starts from the clock again, not from the earlier edit */
    ui_reset(M_MODE_SINGLE); to_run(); with_clock(2026U, 10U, 21U, 14U, 32U, 1U); enter_set(); goto_item(23U); key(MK_START); key(MK_MENU_SHORT); key(MK_COMBO); tick(100U); (void)MUI_Take(&U, MUI_REQ_SET_CLOCK); (void)MUI_Take(&U, MUI_REQ_SAVE);
    with_clock(2026U, 10U, 31U, 14U, 32U, 1U); enter_set(); goto_item(23U); CROW(2, "31/10/2026"); key(MK_MENU_SHORT); key(MK_MENU_SHORT); key(MK_COMBO); tick(100U);
    CHECK(!MUI_Take(&U, MUI_REQ_SET_CLOCK), "a later session that types nothing sends no request");
    printf("  clock save: only a typed change reaches the clock, only on save (MENU or MENU+LOG), never after Discard or a timeout, an impossible one is refused\n");
}

static void t_log_stamp(void)
{
    MDateTime_t dt; dt.year = 2026U; dt.month = 10U; dt.day = 21U; dt.hour = 14U; dt.minute = 32U;
    ui_reset(M_MODE_SINGLE); to_run();
    add_log(MLOG_T_HEAT, 0U, 0U, 2250U, 2300U, 123U, 7425U);                       /* an old record: its running time, no stamp */
    add_log(MLOG_T_IP_LV, 0U, 0U, 1700U, 1700U, 66U, MC_Stamp(&dt));               /* a record with the real-time clock */
    key(MK_LOG_SHORT); ROW(0, "Fault Log 00/01");
    ROW(3, "21/10/2026 14:32"); CHECK(strstr(row(2), "L  6.6A") != NULL || strstr(row(2), "6.6") != NULL, "the newest has its date and time, and the load at the trip is on row 2 ('%s' / '%s')", row(2), row(3));
    key(MK_START); ROW(0, "Fault Log 01/01");
    CHECK(strncmp(row(3), "Run", 3) == 0 && strstr(row(3), "123h45m") != NULL && strstr(row(2), "12.3") != NULL, "the older one has no clock time: it shows its running time ('%s')", row(3));
    { MDateTime_t late; late.year = 2099U; late.month = 12U; late.day = 31U; late.hour = 23U; late.minute = 59U; add_log(MLOG_T_HEAT, 0U, 0U, 2250U, 2300U, 123U, MC_Stamp(&late)); }
    key(MK_STOP); key(MK_STOP); ROW(3, "31/12/2099 23:59");
    CHECK(1, "the last minute of the range is shown in full");
    printf("  fault log time: a record with a clock stamp shows dd/mm/yyyy HH:MM, an older one its running time\n");
}


static void t_wait_row(void)
{
    /* an automatic restart is counting down: the short name at the left, "Rst m:ss" at the right; the code is dropped while it counts */
    ui_reset(M_MODE_SINGLE); to_run();
    D.fault[0] = MLOG_T_INRUSH; D.fcode[0] = 2U; D.wait_s[0] = 272U; tick(100U); ROW(3, "Inrush      Rst 4:32");
    D.fault[0] = MLOG_T_POWER_STAGE; D.fcode[0] = 7U; D.wait_s[0] = 30U; tick(100U); ROW(3, "Power Stage Rst 0:30");
    D.fault[0] = MLOG_T_HEAT; D.wait_s[0] = 5U; tick(100U); ROW(3, "Heat        Rst 0:05");
    D.fault[0] = MLOG_T_INRUSH; D.fcode[0] = 2U; D.wait_s[0] = 300U; tick(100U); ROW(3, "Inrush      Rst 5:00");
    D.wait_s[0] = 0U; tick(100U); ROW(3, "Inrush           F02");                     /* nothing counting (latched): the text and the code, as before */
    D.fault[0] = MLOG_T_OVERLOAD; D.wait_s[0] = 12U; tick(100U); ROW(3, "Overload");                    /* only Inrush, Power Stage and Heat count down */
    printf("  restart countdown on the status row: Inrush, Power Stage and Heat show Rst m:ss, a latch shows its code\n");
}


static void t_vcal_pages(void)
{
    ui_reset(M_MODE_SINGLE); to_run(); D.vin_raw_dv = 2268U; D.vout_raw_dv = 2275U; D.active = 0U; tick(100U);
    key(MK_LOG_SHORT); { int q; for (q = 0; q < 5; q++) key(MK_MENU_SHORT); } tick(100U);
    CHECK(MUI_ShowsVCal(&U) == 1U, "LOG, then MENU x5 opens the I/P Calibration page");
    ROW(0, "  I/P Calibration"); CHECK(strncmp(row(1), "Reading", 7) == 0 && strstr(row(1), "226.8") != NULL && row(1)[18] == 'V', "it shows the slave's reading, 226.8 V ('%s')", row(1));
    CHECK(MUI_CalValue(&U) == 2268U, "the entry starts from the reading");
    { uint32_t k; int seen = 0; for (k = 0U; k < 1200U; k += 50U) { tick(50U); if (strstr(row(2), "226.8") != NULL) seen = 1; } CHECK(seen, "the Actual row reads 226.8 ('%s')", row(2)); }
    ROW(3, "LOG=Save  MENU=Next");
    /* the cursor starts on the units of volts: START adds a volt, STOP moves to the tenths */
    key(MK_START); CHECK(MUI_CalValue(&U) == 2278U && !MUI_Take(&U, MUI_REQ_START), "START adds 1 V (227.8) and does not start the unit");
    key(MK_STOP); key(MK_START); CHECK(MUI_CalValue(&U) == 2279U, "STOP moves to the tenths, START adds 0.1 V (227.9)");
    key(MK_LOG_SHORT); CHECK(MUI_Take(&U, MUI_REQ_VCAL_SAVE) && !MUI_Take(&U, MUI_REQ_CAL_SAVE) && MUI_ShowsVCal(&U) == 1U, "LOG makes one VOLTAGE save request (not the load one) and stays on the page");
    /* the unit is running: the page asks for it to be stopped first */
    D.active = 1U; tick(100U); ROW(3, "Press STOP first");
    { uint32_t c0 = U.cal_cursor; key(MK_STOP); CHECK(MUI_Take(&U, MUI_REQ_STOP) && U.cal_cursor == c0, "while the unit runs, STOP on the page STOPS the unit and does not move the cursor"); }
    D.active = 0U; tick(100U); { uint32_t c0 = U.cal_cursor; key(MK_STOP); CHECK(!MUI_Take(&U, MUI_REQ_STOP) && U.cal_cursor != c0, "once it is stopped, STOP moves the cursor again"); key(MK_STOP); key(MK_STOP); key(MK_STOP); }
    D.active = 1U; tick(100U);
    /* the output page */
    key(MK_MENU_SHORT); CHECK(MUI_ShowsVCal(&U) == 2U, "MENU: the O/P Calibration page"); D.active = 0U; tick(100U);
    ROW(0, "  O/P Calibration"); CHECK(strstr(row(1), "227.5") != NULL && MUI_CalValue(&U) == 2275U, "its own reading, 227.5 V ('%s')", row(1));
    key(MK_LOG_SHORT); CHECK(MUI_Take(&U, MUI_REQ_VCAL_SAVE), "LOG saves the output value");
    key(MK_MENU_SHORT); CHECK(MUI_ShowsVCal(&U) == 0U, "MENU leaves the pages");
    /* a fault takes it back to the Run screen as for the other pages */
    key(MK_LOG_SHORT); { int q; for (q = 0; q < 5; q++) key(MK_MENU_SHORT); } D.fault[0] = MLOG_T_MAINS; D.fcode[0] = 16U; tick(100U); CHECK(MUI_ShowsVCal(&U) == 0U, "a new fault takes it back to the Run screen"); D.fault[0] = 0U; D.fcode[0] = 0U; tick(100U);
    printf("  voltage calibration pages: I/P and O/P, the live reading, digit editing, LOG = one voltage save request, a hint to stop the unit first\n");
}

static void t_cal_page(void)
{
    ui_reset(M_MODE_SINGLE); to_run(); D.load_da[0] = 66U; D.load_ok[0] = 1U; tick(100U);
    key(MK_LOG_SHORT); key(MK_MENU_SHORT); key(MK_MENU_SHORT); key(MK_MENU_SHORT); tick(100U);
    CHECK(MUI_ShowsCal(&U), "LOG, then three MENU presses open the Load Calibration page");
    ROW(0, "  Load Calibration"); CHECK(strncmp(row(1), "Reading", 7) == 0 && strstr(row(1), "6.6") != NULL, "it shows the reading ('%s')", row(1));
    CHECK(MUI_CalValue(&U) == 66U, "and the entry starts from it (6.6 A = 66)");
    ROW(3, "LOG=Save  MENU=Next");
    /* the cursor starts on the units of amps; START adds an amp, STOP moves left to right and wraps */
    { uint32_t t; int seen = 0; for (t = 0U; t < 1200U; t += 50U) { tick(50U); if (strstr(row(2), "006.6") != NULL) seen = 1; } CHECK(seen, "the entry reads 006.6 on the Actual row ('%s')", row(2)); }
    key(MK_START); CHECK(MUI_CalValue(&U) == 76U && !MUI_Take(&U, MUI_REQ_START), "START adds 1 A (7.6 A) and does NOT start the unit");
    key(MK_STOP); key(MK_START); CHECK(MUI_CalValue(&U) == 77U && !MUI_Take(&U, MUI_REQ_STOP), "STOP moves to the tenths; START adds 0.1 A (7.7 A) and does NOT stop the unit");
    key(MK_STOP); key(MK_STOP); key(MK_STOP); key(MK_START); CHECK(MUI_CalValue(&U) == 87U, "STOP wraps around the four digits: back on the units, 8.7 A");
    { int i; for (i = 0; i < 9; i++) key(MK_START); CHECK(MUI_CalValue(&U) == 77U, "a digit wraps 9 to 0 without carrying: ten presses are a full circle"); }
    key(MK_LOG_SHORT); CHECK(MUI_Take(&U, MUI_REQ_CAL_SAVE) && !MUI_Take(&U, MUI_REQ_CAL_SAVE) && MUI_ShowsCal(&U), "LOG makes ONE save request and stays on the page (it does not open the fault log)");
    key(MK_MENU_SHORT); CHECK(!MUI_ShowsCal(&U), "MENU leaves");
    /* Run-Mode keys are untouched elsewhere */
    key(MK_START); CHECK(MUI_Take(&U, MUI_REQ_START), "on the Run screen START is START again");
    printf("  load calibration page: opens with MENU x3, starts from the reading, START/STOP edit and never start or stop the unit, LOG saves once\n");
}

static void t_pages(void)
{
    ui_reset(M_MODE_DOMESTIC); S.scroll = 1U; S.scroll_time = 5U; to_run();
    CHECK(strstr(row(0), "R:") != NULL, "starts on page 1");
    tick(4800U); CHECK(strstr(row(0), "R:") != NULL && strstr(row(0), "R-Y") == NULL, "Scroll Yes: still page 1 before the Scroll Time");
    tick(400U); CHECK(strstr(row(0), "R I/P") != NULL, "after 5 s it moves on by itself: the input voltages");
    tick(5000U); CHECK(strstr(row(0), "R-Y") != NULL, "after 5 s more: phase to phase");
    tick(5000U); CHECK(strstr(row(0), "R : ") != NULL, "and again after 5 s: the status page");
    key(MK_MENU_SHORT); tick(100U); CHECK(strstr(row(0), "R : ") != NULL, "Scroll Yes: a MENU press does not turn the page (the timer does)");
    /* Scroll No: MENU turns the page, the timer does not */
    ui_reset(M_MODE_DOMESTIC); S.scroll = 0U; to_run(); tick(60000U); CHECK(strstr(row(0), "R:") != NULL && strstr(row(0), "R-Y") == NULL, "Scroll No: it stays on the page for a minute");
    key(MK_MENU_SHORT); CHECK(strstr(row(0), "R I/P") != NULL, "a MENU press turns it");
    /* a new fault jumps to the status page, once */
    ui_reset(M_MODE_INDUSTRIAL); S.scroll = 0U; to_run(); CHECK(strstr(row(0), "R:") != NULL, "on page 1");
    D.fault[1] = MLOG_T_IP_LV; tick(100U);
    CHECK(strstr(row(0), "R : ") != NULL && strstr(row(1), "Y : IP LV Appear") != NULL, "a new fault jumps to the status page: '%s'", row(1));
    key(MK_MENU_SHORT); CHECK(strstr(row(0), "R:") != NULL, "and the user can leave it: it does not jump again for the SAME fault");
    tick(500U); CHECK(strstr(row(0), "R:") != NULL, "still on the page the user chose");
    D.fault[1] = 0U; tick(100U); D.fault[2] = MLOG_T_OVERLOAD; tick(100U); CHECK(strstr(row(2), "B : OL Appear") != NULL, "a different fault jumps again: '%s'", row(2));
    D.fault[0] = MLOG_T_MAINS; D.fcode[0] = 8U; D.fault[2] = 0U; key(MK_MENU_SHORT); tick(100U); D.fault[0] = 0U; tick(50U);
    printf("  pages: Scroll Yes rotates by the Scroll Time, Scroll No by MENU, a new fault jumps to the status page\n");
}

static void add_log(uint8_t type, uint8_t ph, uint8_t code, uint16_t vin, uint16_t vout, uint16_t load, uint32_t min)
{ MLogRec_t r; r.type = type; r.phase = ph; r.slave_code = code; r.vin_dv = vin; r.vout_dv = vout; r.load_da = load; r.run_min = min; MLog_Append(&LG, &r); }

static void t_log_viewer(void)
{
    ui_reset(M_MODE_SINGLE); to_run();
    key(MK_LOG_SHORT); ROW(0, "Fault Log"); ROW(1, "No Faults");
    key(MK_LOG_SHORT); CHECK(strstr(row(0), "O/P") != NULL, "an empty log: LOG leaves it");
    add_log(MLOG_T_IP_LV, 0U, 0U, 1700U, 1700U, 0U, 7425U); add_log(MLOG_T_OVERLOAD, 1U, 0U, 2300U, 2290U, 1200U, 7500U); add_log(MLOG_T_MAINS, 2U, 8U, 2250U, 0U, 50U, 9000U);
    key(MK_LOG_SHORT);
    ROW(0, "Fault Log 00/02"); ROW(1, "B Mains Unstable F08"); CHECK(strstr(row(2), "225") != NULL, "00 is the newest: B, Mains Unstable with the slave's code F08, 225 V ('%s' / '%s')", row(1), row(2));
    CHECK(strstr(row(3), "150h00m") != NULL, "the running time is shown in hours and minutes: 9000 min = 150h00m ('%s')", row(3));
    key(MK_START); ROW(0, "Fault Log 01/02"); CHECK(strstr(row(1), "Y Overload") != NULL && strstr(row(3), "125h00m") != NULL && strstr(row(2), "120.0") != NULL, "START goes to an older one: Y, Overload, 120.0 A (the load is on row 2 now) ('%s' / '%s')", row(1), row(3));
    key(MK_START); ROW(0, "Fault Log 02/02"); CHECK(strstr(row(1), "R Input LV") != NULL && strstr(row(3), "123h45m") != NULL, "the oldest: R, Input LV, 7425 min = 123h45m ('%s' / '%s')", row(1), row(3));
    key(MK_START); ROW(0, "Fault Log 02/02");
    CHECK(1, "START at the oldest stays");
    key(MK_STOP); key(MK_STOP); key(MK_STOP); ROW(0, "Fault Log 00/02");
    key(MK_MENU_SHORT); CHECK(MUI_ShowsMonitor(&U), "MENU in the log opens the service pages (single-phase)");
    { int q; for (q = 0; q < 7; q++) key(MK_MENU_SHORT); } CHECK(MUI_ShowsProfile(&U) && !MUI_ShowsIout(&U), "seven more MENU presses reach the Transformer Profile page (the last service page)");
    key(MK_MENU_SHORT); CHECK(strstr(row(0), "Slave Load Cal") != NULL, "one more reaches the Slave Load Cal page");
    key(MK_MENU_SHORT); CHECK(strstr(row(0), "O/P") != NULL, "and one more cycles back to the Run screen (ten pages in all: Run + nine service pages)");
    key(MK_LOG_SHORT); key(MK_LOG_SHORT); CHECK(strstr(row(0), "O/P") != NULL, "LOG leaves the log");
    key(MK_LOG_SHORT); tick(30100U); CHECK(strstr(row(0), "O/P") != NULL, "30 s with no key leaves the log by itself");
    { int i; for (i = 0; i < 130; i++) add_log(MLOG_T_HEAT, 0U, 0U, 2300U, 2300U, 100U, (uint32_t)i); key(MK_LOG_SHORT); ROW(0, "Fault Log 00/99"); CHECK(1, "with a long log the numbers run 00 to 99"); }
    printf("  fault log viewer: newest first, START older, STOP newer, time as hours and minutes, 00 to 99, idle exit\n");
}


/* ---- v0.9.45: the Slave Load Cal page (the ninth service page) and the Load Source item ---- */
static void t_slave_cal_page(void)
{
    ui_reset(M_MODE_SINGLE); to_run();
    D.sl_pk_ca = 780U; D.sl_rms_ca = 90U; D.sl_valid = 1U; D.sl_run = 1U; tick(100U);
    key(MK_LOG_SHORT); { int q; for (q = 0; q < 9; q++) key(MK_MENU_SHORT); } tick(100U);
    CHECK(U.page == 9U && strstr(row(0), "Slave Load Cal") != NULL, "LOG, then nine MENU presses open the Slave Load Cal page ('%s')", row(0));
    CHECK(strstr(row(1), "Peak") != NULL && strstr(row(1), "7.8") != NULL && strstr(row(1), "RMS") != NULL && strstr(row(1), "0.9") != NULL && strlen(row(1)) <= 20U, "it shows the converter PEAK and RMS ('%s')", row(1));
    CHECK(MUI_CalValue(&U) == 78U, "and the entry starts from the peak (7.8 A = 78)");
    CHECK(strstr(row(3), "LOG=Save  MENU=Next") != NULL, "with the unit running the last row says LOG=Save ('%s')", row(3));
    { uint32_t t; int seen = 0; for (t = 0U; t < 1200U; t += 50U) { tick(50U); if (strstr(row(2), "007.8") != NULL) seen = 1; } CHECK(seen, "the entry reads 007.8 on the Actual row ('%s')", row(2)); }
    key(MK_START); CHECK(MUI_CalValue(&U) == 88U && !MUI_Take(&U, MUI_REQ_START), "START adds 1 A and does NOT start the unit");
    key(MK_STOP); key(MK_START); CHECK(MUI_CalValue(&U) == 89U && !MUI_Take(&U, MUI_REQ_STOP), "STOP moves to the tenths, START adds 0.1 A and does NOT stop the unit");
    key(MK_LOG_SHORT); CHECK(MUI_Take(&U, MUI_REQ_SLCAL_SAVE) && !MUI_Take(&U, MUI_REQ_SLCAL_SAVE) && U.page == 9U && U.screen == (uint8_t)MUI_RUN, "LOG makes ONE save request and stays on the page (it does not open the fault log)");
    D.sl_run = 0U; tick(100U); CHECK(strstr(row(3), "Run the unit first") != NULL, "unit not running: 'Run the unit first' ('%s')", row(3));
    D.sl_valid = 0U; tick(100U); CHECK(strstr(row(3), "No converter reading") != NULL && strstr(row(1), "--") != NULL, "no converter reading: dashes and 'No converter reading' ('%s' / '%s')", row(1), row(3));
    key(MK_MENU_SHORT); CHECK(U.page == 0U && strstr(row(0), "O/P") != NULL, "MENU leaves, back to the Run screen");
    /* a very large reading never overflows the row */
    D.sl_pk_ca = 4000U; D.sl_rms_ca = 4000U; D.sl_valid = 1U; D.sl_run = 1U; key(MK_LOG_SHORT); { int q; for (q = 0; q < 9; q++) key(MK_MENU_SHORT); } tick(100U);
    { int w = 1, r; for (r = 0; r < 4; r++) if (strlen(F[r]) != 20U) w = 0; CHECK(w, "40 A of peak still fits the 20 columns"); }
    printf("  slave load cal page: LOG + MENU x9, shows peak and RMS, the entry starts from the peak, START/STOP edit and never start or stop the unit, LOG saves once\n");
}

static void t_load_source_item(void)
{
    uint8_t ids[32]; int n, i, p10 = -1, p25 = -1, p11 = -1;
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); n = walk(ids);
    for (i = 0; i < n; i++) { if (ids[i] == 10U) p10 = i; if (ids[i] == 25U) p25 = i; if (ids[i] == 11U) p11 = i; }
    CHECK(p10 >= 0 && p25 == p10 + 1 && p11 == p25 + 1, "Load Source (25) sits right after Overload Current (10) and before Overload Cut Delay (11)");
    ui_reset(M_MODE_SINGLE); to_run(); S.load_src = 0U; enter_set(); goto_item(25U);
    CROW(1, "Load Source"); CROW(2, "Master CT");
    key(MK_START); CROW(2, "Slave"); key(MK_START); CROW(2, "Master CT"); key(MK_START); CROW(2, "Slave");
    { int g; for (g = 0; g < 40 && MUI_SetModeOpen(&U); g++) { key(MK_MENU_SHORT); tick(100U); } }
    CHECK(!MUI_SetModeOpen(&U) && S.load_src == 1U && MUI_Take(&U, MUI_REQ_SAVE), "START toggles Master CT / Slave; leaving Set-Mode saves it (load_src = %u)", (unsigned)S.load_src);
    /* the slave load calibration is not a setting that Factory Reset throws away */
    ui_reset(M_MODE_SINGLE); to_run(); S.sl_zero_ca = 123U; S.sl_gain = 1280U; S.load_src = 1U; enter_set(); goto_item(22U); CROW(1, "Factory Reset"); key(MK_START); key(MK_MENU_SHORT); tick(100U);
    CHECK(S.set_v == 230U && S.load_src == 0U && S.sl_zero_ca == 123U && S.sl_gain == 1280U && MUI_Take(&U, MUI_REQ_SAVE), "Factory Reset resets the settings (Load Source back to Master CT) but KEEPS the slave load calibration (zero %u, gain %u)", (unsigned)S.sl_zero_ca, (unsigned)S.sl_gain);
    ui_reset(M_MODE_DOMESTIC); to_run(); S.sl_zero_yb[0] = 210U; S.sl_gain_yb[0] = 990U; S.sl_zero_yb[1] = 220U; S.sl_gain_yb[1] = 1010U; S.ovl_current = 60U; enter_set(); goto_item(22U); key(MK_START); key(MK_MENU_SHORT); tick(100U);
    CHECK(S.ovl_current == 25U && S.sl_zero_yb[0] == 210U && S.sl_gain_yb[0] == 990U && S.sl_zero_yb[1] == 220U && S.sl_gain_yb[1] == 1010U, "v0.9.55: Factory Reset keeps the Y and B load calibrations too, and the Overload Current goes back to 25 A (%u)", (unsigned)S.ovl_current);
    printf("  load source item: after Overload Current, toggles Master CT / Slave, saved; Factory Reset keeps the slave load calibration\n");
}

/* ---- v0.9.49: the Overheat Temp item ---- */
static void t_overheat_temp_item(void)
{
    uint8_t ids[32]; int n, i, p12 = -1, p26 = -1;
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); n = walk(ids);
    for (i = 0; i < n; i++) { if (ids[i] == 12U) p12 = i; if (ids[i] == 26U) p26 = i; }
    CHECK(p12 >= 0 && p26 == p12 + 1, "Overheat Temp (26) sits right after Overheat Trip Delay (12)");
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); goto_item(26U);
    CROW(1, "Overheat Temp"); CHECK(strstr(row(2), "075") != NULL && strstr(row(2), "C") != NULL, "it shows 075 C by default ('%s')", row(2));
    { static const uint8_t bad[] = { 0U, 49U, 121U, 255U }; unsigned b;
      for (b = 0U; b < sizeof bad; b++)
      {
          int g; ui_reset(M_MODE_SINGLE); to_run(); enter_set(); goto_item(26U); U.work.heat_temp = bad[b];
          for (g = 0; g < 40 && MUI_SetModeOpen(&U) && strstr(row(1), "Invalid Value") == NULL; g++) { key(MK_MENU_SHORT); tick(100U); }
          CHECK(strstr(row(1), "Invalid Value") != NULL, "%u degC is refused when Set-Mode is left", (unsigned)bad[b]);
          tick(1600U); CHECK(MUI_SetModeOpen(&U) && U.item == 26U, "and Set-Mode stays open on the Overheat Temp item (item %u)", (unsigned)U.item);
      }
      { static const uint8_t good[] = { 50U, 75U, 90U, 120U }; for (b = 0U; b < sizeof good; b++)
        { int g; ui_reset(M_MODE_SINGLE); to_run(); enter_set(); goto_item(26U); U.work.heat_temp = good[b]; U.work.heat_reset = MS_HeatResetDefault(good[b]); for (g = 0; g < 40 && MUI_SetModeOpen(&U); g++) { key(MK_MENU_SHORT); tick(100U); }
          CHECK(!MUI_SetModeOpen(&U) && S.heat_temp == good[b], "%u degC is accepted and saved (%u)", (unsigned)good[b], (unsigned)S.heat_temp); } } }
    /* Factory Reset puts it back to 75 */
    ui_reset(M_MODE_SINGLE); to_run(); S.heat_temp = 95U; enter_set(); goto_item(22U); key(MK_START); key(MK_MENU_SHORT); tick(100U);
    CHECK(S.heat_temp == 75U, "Factory Reset puts the Overheat Temp back to 75 degC (%u)", (unsigned)S.heat_temp);
    printf("  overheat temperature item: after the trip delay, shows 075 C, 50 to 120 accepted, the rest refused, Factory Reset gives 75\n");
}

/* ---- v0.9.50: the heat-sink temperature on the RUN screens ---- */
static void t_run_temperature(void)
{
    int m;
    /* Single phase: at the right of the status row */
    ui_reset(M_MODE_SINGLE); to_run(); D.have[0] = 1U; D.temp_dc[0] = 473; D.temp_st[0] = 1U; tick(100U);
    CHECK(strstr(row(3), "Temp 47C") != NULL && strlen(row(3)) <= 20U, "Single mode: the status row ends with 'Temp 47C' ('%s')", row(3));
    CHECK(strncmp(row(3), "Output", 6) == 0, "and still starts with the status ('%s')", row(3));
    D.temp_dc[0] = 824; D.temp_st[0] = 0x11U; tick(100U); CHECK(strstr(row(3), "Temp 82C!") != NULL, "while the overheat signal is asserted: 'Temp 82C!' ('%s')", row(3));
    D.temp_dc[0] = 1054; D.temp_st[0] = 0x11U; tick(100U); CHECK(strstr(row(3), "Temp 105C") != NULL && strlen(row(3)) <= 20U, "a three digit temperature fits ('%s')", row(3));
    D.temp_dc[0] = 0; D.temp_st[0] = 2U; tick(100U); CHECK(strstr(row(3), "Temp ---") != NULL, "no NTC fitted: 'Temp ---' ('%s')", row(3));
    D.temp_st[0] = 3U; tick(100U); CHECK(strstr(row(3), "Temp SHT") != NULL, "a shorted NTC: 'Temp SHT' ('%s')", row(3));
    D.temp_st[0] = 0U; tick(100U); CHECK(strstr(row(3), "Temp") == NULL, "no reading: nothing is shown ('%s')", row(3));
    /* a long status text keeps the whole row; a text that just leaves room (11 characters + a gap + 'Temp 47C' = 20) shares it */
    D.temp_dc[0] = 473; D.temp_st[0] = 1U; D.out_on = 0U; D.status = "Waiting Mains 12s"; tick(100U);
    CHECK(strncmp(row(3), "Waiting Mains 12s", 17) == 0 && strstr(row(3), "Temp") == NULL, "a long status ('Waiting Mains 12s') keeps the row to itself ('%s')", row(3));
    D.status = "Mains Check"; tick(100U);
    CHECK(strncmp(row(3), "Mains Check", 11) == 0 && strstr(row(3), "Temp 47C") != NULL && strlen(row(3)) == 20U, "an 11 character status and the temperature share the row exactly ('%s')", row(3));
    D.status = "Mains Checks"; tick(100U);
    CHECK(strncmp(row(3), "Mains Checks", 12) == 0 && strstr(row(3), "Temp") == NULL, "one character more and the temperature steps aside ('%s')", row(3));
    D.status = 0; D.out_on = 0U; tick(100U);
    D.temp_dc[0] = 473; D.temp_st[0] = 1U; D.fault[0] = (uint8_t)MLOG_T_OVERLOAD; tick(100U); CHECK(strstr(row(3), "Temp") == NULL, "a fault text owns the row: no temperature ('%s')", row(3));
    D.fault[0] = 0U; D.have[0] = 0U; tick(100U); CHECK(strstr(row(3), "Temp") == NULL && strstr(row(3), "Link Lost") != NULL, "'Slave Link Lost' owns the row too ('%s')", row(3));
    D.have[0] = 1U; D.wait_s[0] = 20U; D.fault[0] = (uint8_t)MLOG_T_HEAT; tick(100U); CHECK(strstr(row(3), "Rst") != NULL && strstr(row(3), "Temp") == NULL, "the restart countdown owns the row: 'Heat   Rst 0:20' ('%s')", row(3));
    D.wait_s[0] = 0U; D.fault[0] = 0U;
    /* three phases: per phase, in the gap between the voltage and the load */
    for (m = 0; m < 2; m++)
    {
        ui_reset(m == 0 ? M_MODE_DOMESTIC : M_MODE_INDUSTRIAL); to_run(); { int p; for (p = 0; p < 3; p++) { D.have[p] = 1U; D.vout_dv[p] = 2300U; D.load_da[p] = 123U; D.load_ok[p] = 1U; } }
        D.temp_dc[0] = 473; D.temp_st[0] = 1U; D.temp_dc[1] = 824; D.temp_st[1] = 0x11U; D.temp_dc[2] = 0; D.temp_st[2] = 2U; tick(100U);
        CHECK(strstr(row(0), "R:") != NULL && strstr(row(0), "47C") != NULL && strstr(row(1), "82C!") != NULL && strstr(row(2), "---") != NULL, "%s: the page shows R 47C, Y 82C!, B --- ('%s' / '%s' / '%s')", m == 0 ? "Domestic" : "Industrial", row(0), row(1), row(2));
        CHECK(strstr(row(0), "230V") != NULL && strstr(row(0), "12.3A") != NULL && strlen(row(0)) <= 20U && strlen(row(1)) <= 20U, "and the voltage and load are still there, nothing overlaps ('%s')", row(0));
        ROW(0, "R: 230V 47C   12.3A"); ROW(1, "Y: 230V 82C!  12.3A"); ROW(2, "B: 230V ---   12.3A");      /* v0.9.54: a space between the voltage and the temperature (it read '230V47C') */
        D.load_da[1] = 1234U; tick(100U); ROW(1, "Y: 230V 82C! 123.4A"); D.load_da[1] = 123U;      /* a hot reading and a three digit load share the row without touching */
        {   /* v0.9.54: the status page carries each phase's heat sink at the right, when the fault text leaves room */
            int k; uint8_t sc = S.scroll; S.scroll = 0U; for (k = 0; k < 4 && strstr(row(0), "R : ") == NULL; k++) { key(MK_MENU_SHORT); tick(100U); }
            ROW(0, "R : OK           47C"); ROW(1, "Y : OK          82C!"); ROW(2, "B : OK           ---");
            D.fault[1] = (uint8_t)MLOG_T_PHASE_SEQ; D.seq_state = 1U; tick(100U);
            if (m == 0) CHECK(strcmp(row(1), "Y : PH SEQ REVERSE") == 0, "a long fault text keeps its row: no temperature ('%s')", row(1));
            else CHECK(row(0)[0] == '\0' && crow_ok(1, "PHASE REVERSE") && crow_ok(2, "Check R-Y-B Order") && row(3)[0] == '\0', "v0.9.57, Industrial: a wrong sequence is said alone, in the middle ('%s' / '%s')", row(1), row(2));
            D.fault[1] = (uint8_t)MLOG_T_IP_LV; D.fault[0] = (uint8_t)MLOG_T_IP_LV; tick(100U); ROW(0, "R : IP LV Appear 47C"); CHECK(strcmp(row(1), "Y : IP LV Appear") == 0, "exactly 20 fits ('47C'), 21 does not ('82C!': '%s')", row(1));
            D.fault[1] = 0U; D.fault[0] = 0U; D.seq_state = 0U; tick(100U);
            for (k = 0; k < 4 && strstr(row(0), "R: ") == NULL; k++) { key(MK_MENU_SHORT); tick(100U); }
            S.scroll = sc;
        }
        D.temp_dc[0] = 1054; D.temp_st[0] = 0x11U; tick(100U); CHECK(strstr(row(0), "105C") != NULL && strstr(row(0), "12.3A") != NULL, "a three digit temperature does not touch the load ('%s')", row(0));
        D.temp_st[0] = 0U; D.temp_st[1] = 0U; D.temp_st[2] = 0U; tick(100U); CHECK(strstr(row(0), "C") == NULL && strstr(row(1), "C") == NULL, "with no readings the page is as it was ('%s')", row(0));
    }
    printf("  run screens: Single status row 'Temp 47C' (not over a fault text or a countdown), Domestic and Industrial per phase\n");
}

/* ---- v0.9.50: the Overheat Restart item: at least 35, at most the Overheat Temp - 5 ---- */
static void leave_set_checking(void) { int g; for (g = 0; g < 40 && MUI_SetModeOpen(&U) && strstr(row(1), "Invalid Value") == NULL; g++) { key(MK_MENU_SHORT); tick(100U); } }
static void t_overheat_restart_item(void)
{
    uint8_t ids[32]; int n, i, p26 = -1, p27 = -1;
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); n = walk(ids);
    for (i = 0; i < n; i++) { if (ids[i] == 26U) p26 = i; if (ids[i] == 27U) p27 = i; }
    CHECK(p26 >= 0 && p27 == p26 + 1, "Overheat Restart (27) sits right after Overheat Temp (26)");
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); goto_item(27U);
    CROW(1, "Overheat Restart"); CHECK(strstr(row(2), "065") != NULL && strstr(row(2), "C") != NULL, "it shows 065 C by default (10 below the 75) ('%s')", row(2));
    { static const struct { uint8_t temp, reset; int ok; } TT[] = { {75U, 34U, 0}, {75U, 35U, 1}, {75U, 65U, 1}, {75U, 70U, 1}, {75U, 71U, 0}, {75U, 75U, 0}, {75U, 0U, 0}, {50U, 45U, 1}, {50U, 46U, 0}, {120U, 115U, 1}, {120U, 116U, 0} }; unsigned q;
      for (q = 0U; q < sizeof TT / sizeof TT[0]; q++)
      {
          ui_reset(M_MODE_SINGLE); to_run(); enter_set(); goto_item(27U); U.work.heat_temp = TT[q].temp; U.work.heat_reset = TT[q].reset; leave_set_checking();
          if (TT[q].ok) CHECK(!MUI_SetModeOpen(&U) && S.heat_temp == TT[q].temp && S.heat_reset == TT[q].reset, "Overheat Temp %u with Overheat Restart %u is accepted and saved (%u / %u)", (unsigned)TT[q].temp, (unsigned)TT[q].reset, (unsigned)S.heat_temp, (unsigned)S.heat_reset);
          else { CHECK(strstr(row(1), "Invalid Value") != NULL, "Overheat Temp %u with Overheat Restart %u is refused", (unsigned)TT[q].temp, (unsigned)TT[q].reset); tick(1600U); CHECK(MUI_SetModeOpen(&U) && U.item == 27U, "and Set-Mode stays open on the Overheat Restart item (item %u)", (unsigned)U.item); }
      } }
    /* the cross rule: lowering the Overheat Temp so that the restart would be too close takes the restart down with it (never up) */
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); goto_item(26U); U.work.heat_temp = 60U; { int g; for (g = 0; g < 40 && MUI_SetModeOpen(&U); g++) { key(MK_MENU_SHORT); tick(100U); } }
    CHECK(!MUI_SetModeOpen(&U) && S.heat_temp == 60U && S.heat_reset == 50U && MS_Validate(&S, 0) == MS_OK, "Overheat Temp 60 with the restart at 65: saved, and the restart followed down to 50 (%u / %u)", (unsigned)S.heat_temp, (unsigned)S.heat_reset);
    ui_reset(M_MODE_SINGLE); to_run(); S.heat_temp = 90U; S.heat_reset = 40U; enter_set(); goto_item(26U); U.work.heat_temp = 60U; { int g; for (g = 0; g < 40 && MUI_SetModeOpen(&U); g++) { key(MK_MENU_SHORT); tick(100U); } }
    CHECK(S.heat_temp == 60U && S.heat_reset == 40U, "a restart that still fits (40 with the temperature lowered to 60) is left alone (%u / %u)", (unsigned)S.heat_temp, (unsigned)S.heat_reset);
    ui_reset(M_MODE_SINGLE); to_run(); S.heat_temp = 60U; S.heat_reset = 50U; enter_set(); goto_item(26U); U.work.heat_temp = 100U; { int g; for (g = 0; g < 40 && MUI_SetModeOpen(&U); g++) { key(MK_MENU_SHORT); tick(100U); } }
    CHECK(S.heat_temp == 100U && S.heat_reset == 50U, "raising the temperature never moves the restart (%u / %u)", (unsigned)S.heat_temp, (unsigned)S.heat_reset);
    /* Factory Reset */
    ui_reset(M_MODE_SINGLE); to_run(); S.heat_temp = 95U; S.heat_reset = 50U; enter_set(); goto_item(22U); key(MK_START); key(MK_MENU_SHORT); tick(100U);
    CHECK(S.heat_temp == 75U && S.heat_reset == 65U, "Factory Reset gives 75 and 65 (%u / %u)", (unsigned)S.heat_temp, (unsigned)S.heat_reset);
    printf("  overheat restart item: after the Overheat Temp, 065 C by default, 35 to Overheat Temp - 5 accepted, the rest refused, the cross rule points at it, Factory Reset gives 65\n");
}

static void t_setmode_nav(void)
{
    uint8_t ids[32]; int n, m; MSettings_t *w;
    ui_reset(M_MODE_SINGLE); to_run();
    enter_set();
    CHECK(MUI_SetModeOpen(&U), "Set-Mode is open: the engine must hold the output off");
    CROW(0, "Set Mode"); CROW(1, "Mode"); CROW(2, "Single Phase");
    CHECK(F[0][18] == ' ' && F[0][19] == ' ', "no item number in the 19th and 20th positions of the first row");
    /* walk every visible item and record its number */
    n = walk(ids);
    CHECK(ids[0] == 1U && ids[1] == 2U && ids[2] == 3U && ids[3] == 4U && ids[4] == 5U, "the first items are Mode, Calibration, Set Voltage, Max Correction, I/P LV");
    for (m = 0; m < n; m++) if (ids[m] == 13U || ids[m] == 14U || ids[m] == 15U) break;
    CHECK(m == n, "Single mode: Phase Sequence, Display Scroll and Scroll Time are not offered");
    CHECK(n == 24, "Single mode with Auto restart offers 24 items: the 19 before, Load Source, Overheat Temp, Overheat Restart, Date and Time (%d)", n);
    CHECK(ids[n - 1] == 22U, "the last is Factory Reset");
    CHECK(!MUI_SetModeOpen(&U) && MUI_Take(&U, MUI_REQ_SAVE), "one round of the items is complete: the MENU press on the last one LEAVES Set-Mode (saving), it does not start again");
    CHECK(strstr(row(0), "O/P") != NULL, "and Run-Mode is shown");
    (void)w;
    /* Industrial adds Phase Sequence, Display Scroll, Scroll Time (scroll Yes) */
    ui_reset(M_MODE_INDUSTRIAL); to_run(); enter_set();
    n = walk(ids);
    { int has13 = 0, has14 = 0, has15 = 0; for (m = 0; m < n; m++) { if (ids[m] == 13U) has13 = 1; if (ids[m] == 14U) has14 = 1; if (ids[m] == 15U) has15 = 1; }
      CHECK(has13 && has14 && has15 && n == 27, "Industrial offers Phase Sequence, Display Scroll and Scroll Time: 27 items (%d)", n); }
    /* Domestic: no Phase Sequence */
    ui_reset(M_MODE_DOMESTIC); to_run(); enter_set();
    n = walk(ids);
    { int has13 = 0; for (m = 0; m < n; m++) if (ids[m] == 13U) has13 = 1; CHECK(!has13 && n == 26, "Domestic: Display Scroll and Scroll Time but no Phase Sequence: 26 items (%d)", n); }
    /* Manual restart hides Restart Time; Scroll No hides Scroll Time */
    ui_reset(M_MODE_INDUSTRIAL); S.restart_auto = 0U; S.scroll = 0U; to_run(); enter_set();
    n = walk(ids);
    { int has15 = 0, has18 = 0; for (m = 0; m < n; m++) { if (ids[m] == 15U) has15 = 1; if (ids[m] == 18U) has18 = 1; } CHECK(!has15 && !has18, "Manual restart hides Restart Time, Scroll No hides Scroll Time"); }
    printf("  Set-Mode navigation: the items offered in each mode, wrapping, the output held off\n");
}

static void t_setmode_edit(void)
{
    int i;
    ui_reset(M_MODE_SINGLE); to_run(); enter_set();
    key(MK_MENU_SHORT); key(MK_MENU_SHORT);                        /* Mode -> Calibration -> Set Voltage */
    CROW(1, "Set Voltage"); CROW(2, "230 V"); CUR(5, 0);
    key(MK_START); CROW(2, "330 V");
    key(MK_MENU_SHORT);
    CROW(1, "Invalid Value"); CROW(2, "Set Voltage");
    tick(1600U); CROW(1, "Set Voltage"); CROW(2, "230 V");
    CHECK(1, "330 V is above 300: refused with 'Invalid Value', the value put back, still on the item");
    key(MK_STOP); CUR(5, 1); key(MK_START); CROW(2, "240 V");
    key(MK_STOP); key(MK_STOP); CUR(5, 0);
    CHECK(1, "STOP moves the cursor through the digits and wraps");
    key(MK_MENU_SHORT); CROW(1, "Max Correction"); CROW(2, "000 V Not Set");
    key(MK_STOP); key(MK_START); key(MK_START); key(MK_START); CROW(2, "030 V");                         /* Max Correction 30 V: with Set 240 V the input window is 210..270 V */
    key(MK_MENU_SHORT); CROW(1, "I/P LV");
    key(MK_STOP); key(MK_STOP); for (i = 0; i < 10; i++) key(MK_START); CROW(2, "210 V");                  /* the old 180 V was clipped into the new window 210..270 */
    CHECK(1, "ten presses are a full circle of a digit");
    for (i = 0; i < 9; i++) { key(MK_START); } CROW(2, "219 V"); CUR(5, 2);                              /* the cursor is still on the units digit */
    key(MK_STOP); key(MK_START); CROW(2, "319 V"); CUR(5, 0);                                             /* cursor wraps to the hundreds: 2 -> 3 */
    key(MK_MENU_SHORT); CROW(1, "Invalid Value");
    tick(1600U); CROW(1, "I/P LV"); CROW(2, "210 V");
    CHECK(1, "an I/P LV of 319 V is above the Set Voltage 240: refused, put back");
    printf("  Set-Mode editing: digit by digit, wrap, range refusal with the value put back, cursor\n");
}

static void t_cursor_and_layout(void)
{
    int i, on = 0, off = 0, ok_pos = 1, id; uint32_t t0;
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); key(MK_MENU_SHORT); key(MK_MENU_SHORT);           /* Set Voltage 230 V */
    CHECK(strchr(F[3], '^') == NULL, "no ^ anywhere");
    t0 = T;
    for (i = 0; i < 40; i++)
    {   /* 2 seconds in 50 ms steps with no key: the underscore is there for 0.5 s, gone for 0.5 s, and never moves */
        MUI_Tick(&U, &D, T, F); T += 50U;
        if (F[2][7] == '_') off++; else if (F[2][7] == '2') on++; else ok_pos = 0;                          /* v0.9.57: the first digit of "230 V" blinks in place */
        if ((strchr(F[3], '_') != NULL) || ((strchr(F[2], '_') != NULL) && (strchr(F[2], '_') != &F[2][7]))) ok_pos = 0;
    }
    CHECK(on >= 15 && off >= 15 && ok_pos, "the digit BLINKS in place: shown %d, underscore %d of 40 samples, always the first digit (column 7), nothing on row 3", on, off);
    CHECK(strstr(F[3], "^") == NULL, "never a ^");
    (void)t0;
    key(MK_STOP); CHECK(F[2][8] == '3', "right after a key the digit is shown ('%s')", F[2]); tick(520U); CHECK(F[2][8] == '_', "then it blinks ('%s')", F[2]); key(MK_STOP); CUR(5, 2);
    CHECK(1, "and a key press shows it again at once");
    /* every item: centred, nothing in the last two columns of the first row, the cursor only on numbers */
    ui_reset(M_MODE_INDUSTRIAL); to_run(); enter_set();
    for (id = 0; id < 30 && MUI_SetModeOpen(&U); id++)
    {
        int r, c, bad = 0; char l1[22]; int n1, lead;
        MUI_Tick(&U, &D, T, F);
        CHECK(crow_ok(0, "Set Mode"), "item %d: the first row is 'Set Mode' centred", (int)U.item);
        if (F[0][18] != ' ' || F[0][19] != ' ') bad = 1;
        for (r = 1; r <= 2; r++) { strcpy(l1, row(r)); n1 = (int)strlen(l1); lead = 0; while (l1[lead] == ' ') lead++; if (n1 > 0 && (lead != (20 - (n1 - lead)) / 2)) bad = 1; }
        for (r = 0; r < 4; r++) for (c = 0; c < 20; c++) if (F[r][c] < 32 || F[r][c] > 126) bad = 1;
        CHECK(!bad, "item %d: the name and the value are centred, no number in columns 19 and 20, only printable characters", (int)U.item);
        key(MK_MENU_SHORT); tick(1600U);
    }
    printf("  cursor and layout: a blinking underscore, every Set-Mode screen centred, no item number\n");
}

static void t_setmode_rules(void)
{
    int i;
    /* a Set Voltage below the I/P LV: the EDIT breaks the cross rule, so it is refused */
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); key(MK_MENU_SHORT); key(MK_MENU_SHORT);
    for (i = 0; i < 9; i++) key(MK_START);                                                                    /* hundreds 2 -> 1: 130 */
    key(MK_STOP); for (i = 0; i < 4; i++) key(MK_START); CROW(2, "170 V");
    key(MK_MENU_SHORT); CROW(1, "Max Correction");
    CHECK(U.work.set_v == 170U && U.work.ip_lv == 160U && U.work.op_lv == 160U && U.work.ip_hv == 280U && U.work.op_hv == 250U,
          "Set Voltage 170 V is ACCEPTED and the limits follow: I/P LV 180 -> 160 V, O/P LV 200 -> 160 V, the high limits stay (%u %u %u %u)", U.work.ip_lv, U.work.op_lv, U.work.ip_hv, U.work.op_hv);
    /* a Set Voltage that equals O/P HV is refused too (strict) */
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); key(MK_MENU_SHORT); key(MK_MENU_SHORT); key(MK_STOP); for (i = 0; i < 2; i++) key(MK_START); CROW(2, "250 V");
    key(MK_MENU_SHORT); CROW(1, "Max Correction");
    CHECK(U.work.set_v == 250U && U.work.op_hv == 260U && U.work.ip_hv == 280U && U.work.ip_lv == 180U,
          "Set Voltage 250 V is accepted and O/P HV (250 V, no longer 10 V above Set) moves up to 260 V (%u)", U.work.op_hv);
    /* slave window and guard (D7, D26): an O/P HV above the guard must not trap the user on unrelated items */
    ui_reset(M_MODE_SINGLE); LIM.out_guard_v = 240U; to_run(); enter_set();
    key(MK_MENU_SHORT); CROW(1, "Calibration"); key(MK_MENU_SHORT); CROW(1, "Set Voltage");
    CHECK(1, "a violation that was already there does not stop the user moving between unrelated items");
    key(MK_COMBO);
    CHECK(strstr(row(1), "Invalid Value") != NULL && strstr(row(2), "O/P HV") != NULL, "but leaving Set-Mode is refused and names the item ('%s' / '%s')", row(1), row(2));
    tick(1600U); CROW(0, "Set Mode"); CROW(1, "O/P HV");
    CHECK(MUI_SetModeOpen(&U) && !MUI_Take(&U, MUI_REQ_SAVE), "the user is taken to O/P HV, still in Set-Mode, nothing to save");
    printf("  Set-Mode rules: the edit that causes a violation is refused; an existing violation is caught on leaving, not by trapping\n");
}

static void t_setmode_actions(void)
{
    int i; MSettings_t before;
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); before = S;
    key(MK_MENU_SHORT); CROW(1, "Calibration"); key(MK_START); CROW(2, "Yes"); key(MK_MENU_SHORT);
    CHECK(strstr(row(1), "Calibration") != NULL && strstr(row(2), "Not Available") != NULL && !MUI_Take(&U, MUI_REQ_CALIBRATE), "Calibration Yes without the feature: 'Not Available', no request ('%s')", row(2));
    ui_reset(M_MODE_SINGLE); MUI_SetFeatures(&U, MUI_FEAT_CALIBRATE | MUI_FEAT_RETUNE); to_run(); enter_set();
    key(MK_MENU_SHORT); key(MK_START); key(MK_MENU_SHORT); CHECK(MUI_Take(&U, MUI_REQ_CALIBRATE) && !MUI_Take(&U, MUI_REQ_CALIBRATE), "with the feature, Calibration Yes makes ONE request");
    /* walk to Re-tune (19) and Clear Fault Log (20) */
    tick(1600U); for (i = 0; i < 40 && U.item != 19; i++) { key(MK_MENU_SHORT); tick(1600U); }
    CROW(1, "Re-tune"); key(MK_START); key(MK_MENU_SHORT); CHECK(MUI_Take(&U, MUI_REQ_RETUNE), "Re-tune Yes with the feature makes the request");
    tick(1600U); CROW(1, "Clear Fault Log"); key(MK_START); key(MK_MENU_SHORT); CHECK(MUI_Take(&U, MUI_REQ_CLEAR_LOG), "Clear Fault Log Yes makes the request");
    tick(1600U); CROW(1, "Discard Changes");
    key(MK_START); CROW(2, "Yes"); key(MK_START); CROW(2, "No");
    CHECK(1, "START toggles Yes and No on an action item");
    /* edit something, then Discard Changes = Yes: the live settings are untouched */
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); before = S;
    key(MK_MENU_SHORT); key(MK_MENU_SHORT); key(MK_STOP); key(MK_START); key(MK_MENU_SHORT);                                   /* Set Voltage 230 -> 240 */
    for (i = 0; i < 40 && U.item != 21; i++) { key(MK_MENU_SHORT); tick(1600U); }
    CROW(1, "Discard Changes"); key(MK_START); key(MK_MENU_SHORT); tick(100U);
    CHECK(!MUI_SetModeOpen(&U) && S.set_v == before.set_v && !MUI_Take(&U, MUI_REQ_SAVE), "Discard Changes: out of Set-Mode, nothing saved, set voltage still %u", (unsigned)S.set_v);
    CROW(1, "Changes"); CROW(2, "Discarded"); tick(1600U); CHECK(strstr(row(0), "O/P") != NULL, "the message goes and Run-Mode returns");
    /* a real edit saved by the combination */
    ui_reset(M_MODE_SINGLE); to_run(); enter_set();
    key(MK_MENU_SHORT); key(MK_MENU_SHORT); key(MK_STOP); key(MK_START); key(MK_MENU_SHORT);                                   /* 240 */
    key(MK_COMBO); tick(100U);
    CHECK(!MUI_SetModeOpen(&U) && S.set_v == 240U && MUI_Take(&U, MUI_REQ_SAVE) && !MUI_Take(&U, MUI_REQ_SAVE), "MENU+LOG saves: set voltage 240 in the live settings, ONE save request");
    CROW(1, "Settings"); CROW(2, "Saved"); tick(1600U); CHECK(strstr(row(0), "O/P") != NULL, "then Run-Mode");
    CHECK(MS_Validate(&S, &LIM) == MS_OK, "the saved settings pass the rules");
    /* Factory Reset keeps nothing of the edits, only settings are reset, and it takes effect on saving */
    ui_reset(M_MODE_SINGLE); S.set_v = 250U; S.ip_hv = 290U; S.password = 0U; to_run(); enter_set();
    for (i = 0; i < 40 && U.item != 22; i++) { key(MK_MENU_SHORT); tick(1600U); }
    CROW(1, "Factory Reset"); key(MK_START); key(MK_MENU_SHORT); tick(100U);
    CHECK(S.set_v == 230U && S.ip_hv == 300U && S.ip_lv == 90U && S.op_lv == 110U && S.op_hv == LIM.out_guard_v && MUI_Take(&U, MUI_REQ_SAVE), "Factory Reset (the last item): Yes + MENU resets the settings to the new default limits, O/P HV kept under the slave's output guard, and saves them, leaving Set-Mode (set %u, I/P %u-%u, O/P %u-%u)", S.set_v, S.ip_lv, S.ip_hv, S.op_lv, S.op_hv);
    /* idle timeout */
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); key(MK_MENU_SHORT); key(MK_MENU_SHORT); key(MK_STOP); key(MK_START);
    tick(29000U); CHECK(MUI_SetModeOpen(&U), "29 s with no key: still in Set-Mode");
    tick(1500U);
    CHECK(!MUI_SetModeOpen(&U) && S.set_v == 230U && !MUI_Take(&U, MUI_REQ_SAVE), "30 s with no key: out, NOT saved, the edit thrown away");
    CHECK(strstr(row(1), "Timeout") != NULL && strstr(row(2), "Not Saved") != NULL, "with the message 'Timeout - Not Saved' ('%s' / '%s')", row(1), row(2));
    /* each key restarts the idle time */
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); for (i = 0; i < 5; i++) { tick(20000U); key(MK_STOP); }
    CHECK(MUI_SetModeOpen(&U), "a key every 20 s keeps Set-Mode open for 100 s");
    printf("  Set-Mode actions: Calibration/Re-tune/Clear Log requests (and 'Not Available'), Discard, save by MENU+LOG, Factory Reset, the 30 s idle timeout\n");
}

static void t_password(void)
{
    ui_reset(M_MODE_SINGLE); S.password = 123U; to_run();
    key(MK_COMBO); CROW(0, "Enter Password"); CROW(2, "000"); CHECK(MUI_SetModeOpen(&U), "with a password set, the combination asks for it; the output must already be off");
    key(MK_START); CROW(2, "100"); key(MK_STOP); key(MK_START); key(MK_START); CROW(2, "120"); key(MK_STOP); key(MK_START); key(MK_START); key(MK_START); CROW(2, "123");
    key(MK_MENU_SHORT); CROW(0, "Set Mode");
    CHECK(1, "the right password opens Set-Mode");
    key(MK_COMBO); tick(1600U);
    ui_reset(M_MODE_SINGLE); S.password = 123U; to_run(); key(MK_COMBO); key(MK_START); key(MK_MENU_SHORT);
    CROW(1, "Wrong Password"); tick(1600U); CHECK(!MUI_SetModeOpen(&U) && strstr(row(0), "O/P") != NULL, "a wrong password goes back to Run-Mode");
    ui_reset(M_MODE_SINGLE); S.password = 123U; to_run(); key(MK_COMBO); key(MK_COMBO); CHECK(!MUI_SetModeOpen(&U), "the combination again cancels the prompt");
    ui_reset(M_MODE_SINGLE); S.password = 123U; to_run(); key(MK_COMBO); tick(30100U); CHECK(!MUI_SetModeOpen(&U), "an unanswered prompt times out after 30 s");
    ui_reset(M_MODE_SINGLE); S.password = 0U; to_run(); key(MK_COMBO); CROW(0, "Set Mode");
    CHECK(1, "password 000 means no prompt");
    printf("  password: asked for when set, typed digit by digit, wrong/cancel/timeout go back, 000 = none\n");
}

static void t_run_keys(void)
{
    ui_reset(M_MODE_SINGLE); to_run();
    key(MK_START); CHECK(MUI_Take(&U, MUI_REQ_START) && !MUI_Take(&U, MUI_REQ_START), "START in Run-Mode is passed to the app, once");
    key(MK_STOP); CHECK(MUI_Take(&U, MUI_REQ_STOP), "and STOP");
    ui_reset(M_MODE_SINGLE); MUI_Key(&U, MK_COMBO, T); MUI_Key(&U, MK_START, T); CHECK(!MUI_SetModeOpen(&U) && !MUI_Take(&U, MUI_REQ_START), "keys during the splash screen do nothing");
    ui_reset(M_MODE_SINGLE); to_run(); enter_set(); key(MK_START); CHECK(!MUI_Take(&U, MUI_REQ_START), "START inside Set-Mode edits, it never reaches the app as a START");
    printf("  Run-Mode keys: START and STOP reach the app; ignored during the splash and inside Set-Mode\n");
}

static void t_ui_fuzz(void)
{
    uint32_t i, bad_frame = 0U, bad_live = 0U, bad_open = 0U; int p;
    static const MKEvent_t EV[6] = { MK_NONE, MK_MENU_SHORT, MK_START, MK_STOP, MK_LOG_SHORT, MK_COMBO };
    for (i = 0U; i < 40U; i++) { /* 40 rounds in random modes */
        uint32_t k;
        ui_reset((uint8_t)(rnd() % 3U)); S.password = (uint16_t)((rnd() % 3U == 0U) ? (rnd() % 1000U) : 0U); S.scroll = (uint8_t)(rnd() & 1U);
        if (rnd() % 2U) { LIM.win_lo_v = 150U; LIM.out_guard_v = 240U; }
        MUI_SetFeatures(&U, (uint8_t)(rnd() & 3U));
        for (k = 0U; k < 60000U; k++)
        {
            uint32_t r = rnd() % 100U; int q;
            if (r < 55U) MUI_Key(&U, EV[rnd() % 6U], T);
            if (r == 99U) T += 31000U;
            if (r > 90U) { for (p = 0; p < 3; p++) { D.have[p] = (uint8_t)(rnd() & 1U); D.fault[p] = (uint8_t)(rnd() % 14U); D.fcode[p] = (uint8_t)(rnd() % 40U); D.vout_dv[p] = (uint16_t)(rnd() % 5000U); D.load_da[p] = (uint16_t)(rnd() % 3000U); } }
            T += 50U + (rnd() % 400U);
            MUI_Tick(&U, &D, T, F);
            for (q = 0; q < 4; q++) { uint32_t c; if (strlen(F[q]) != 20U) bad_frame++; for (c = 0U; c < 20U; c++) if (F[q][c] < 32 || F[q][c] > 126) bad_frame++; }
            if (MS_Validate(&S, 0) != MS_OK) bad_live++;                                  /* the live settings are only ever changed by a validated save */
            if ((U.screen == (uint8_t)MUI_SET || U.screen == (uint8_t)MUI_PW) != (MUI_SetModeOpen(&U) != 0U)) bad_open++;
            while (MUI_Take(&U, MUI_REQ_SAVE) || MUI_Take(&U, MUI_REQ_CLEAR_LOG) || MUI_Take(&U, MUI_REQ_CALIBRATE) || MUI_Take(&U, MUI_REQ_RETUNE) || MUI_Take(&U, MUI_REQ_START) || MUI_Take(&U, MUI_REQ_STOP)) {}
        }
    }
    CHECK(bad_frame == 0U, "2,400,000 random key and time steps in random modes: every frame is 4 rows of 20 printable characters (%u bad)", (unsigned)bad_frame);
    CHECK(bad_live == 0U, "the live settings always pass the rules: only a validated save changes them (%u bad)", (unsigned)bad_live);
    CHECK(bad_open == 0U, "MUI_SetModeOpen always agrees with the screen (%u bad)", (unsigned)bad_open);
    printf("  UI fuzz: 2,400,000 random steps: frames well formed, live settings always valid\n");
}


/* v0.9.54: the service pages (monitor, diagnostics, calibrations, clock, converter current, transformer profile) in Industrial and Domestic too, and the per-phase Re-tune choice */
static void t_three_service(void)
{
    int m, i; static const char *const RT[5] = { "No", "R Phase", "Y Phase", "B Phase", "All, One By One" };
    for (m = 0; m < 2; m++)
    {
        uint8_t mode = (m == 0) ? (uint8_t)M_MODE_DOMESTIC : (uint8_t)M_MODE_INDUSTRIAL; const char *mn = (m == 0) ? "Domestic" : "Industrial";
        ui_reset(mode); S.scroll = 0U; to_run();
        key(MK_LOG_SHORT); key(MK_MENU_SHORT); CHECK(MUI_ShowsMonitor(&U) && U.page == 0U, "%s: LOG then MENU opens the monitor page (it went straight back to the Run page before)", mn);
        key(MK_MENU_SHORT); CHECK(MUI_ShowsDiag(&U) && !MUI_ShowsMonitor(&U), "%s: MENU: the diagnostics page", mn);
        key(MK_MENU_SHORT); CHECK(MUI_ShowsCal(&U), "%s: MENU: the load calibration page", mn);
        key(MK_MENU_SHORT); CHECK(MUI_ShowsClock(&U), "%s: the clock page", mn);
        {   /* v0.9.55: the I/P, O/P and Slave Load Cal pages come for R, then Y, then B */
            static const char PH[3] = { 'R', 'Y', 'B' }; int q;
            for (q = 0; q < 3; q++) { char t[24]; key(MK_MENU_SHORT); snprintf(t, sizeof t, "I/P Calibration %c", PH[q]); CHECK(MUI_ShowsVCal(&U) == 1U && MUI_CalPhase(&U) == (uint8_t)q && strstr(row(0), t) != NULL, "%s: input calibration %c ('%s')", mn, PH[q], row(0)); }
            for (q = 0; q < 3; q++) { char t[24]; key(MK_MENU_SHORT); snprintf(t, sizeof t, "O/P Calibration %c", PH[q]); CHECK(MUI_ShowsVCal(&U) == 2U && MUI_CalPhase(&U) == (uint8_t)q && strstr(row(0), t) != NULL, "%s: output calibration %c ('%s')", mn, PH[q], row(0)); }
            key(MK_MENU_SHORT); CHECK(MUI_ShowsIout(&U) && MUI_CalPhase(&U) == 0U, "%s: converter current", mn);
            key(MK_MENU_SHORT); CHECK(MUI_ShowsProfile(&U), "%s: the transformer profile page", mn);
            for (q = 0; q < 3; q++) { char t[24]; key(MK_MENU_SHORT); snprintf(t, sizeof t, "Slave Load Cal %c", PH[q]); CHECK(U.svc == 9U && MUI_CalPhase(&U) == (uint8_t)q && strstr(row(0), t) != NULL, "%s: the slave load calibration %c ('%s')", mn, PH[q], row(0)); }
        }
        key(MK_MENU_SHORT); CHECK(U.svc == 0U && U.page == 0U && strstr(row(0), "R: ") != NULL, "%s: after the last service page MENU is back on the Run page ('%s')", mn, row(0));
        key(MK_MENU_SHORT); CHECK(U.page == 1U && U.svc == 0U && !MUI_ShowsDiag(&U), "%s: and MENU turns the Run pages as before (page %u, not the diagnostics)", mn, (unsigned)U.page);
        /* a minute without a key, or a new fault, leaves a service page; the scroll timer does not turn it */
        key(MK_MENU_SHORT); key(MK_MENU_SHORT); key(MK_LOG_SHORT); key(MK_MENU_SHORT); key(MK_MENU_SHORT); S.scroll = 1U; S.scroll_time = 5U; tick(20000U);
        CHECK(MUI_ShowsDiag(&U), "%s: Display Scroll does not turn a service page away", mn);
        tick(MUI_SERVICE_IDLE_MS); CHECK(U.svc == 0U && !MUI_ShowsDiag(&U), "%s: a minute without a key: back to the Run pages", mn);
        S.scroll = 0U; key(MK_LOG_SHORT); key(MK_MENU_SHORT); CHECK(MUI_ShowsMonitor(&U), "(monitor again)");
        D.fault[1] = (uint8_t)MLOG_T_IP_LV; tick(100U); CHECK(U.svc == 0U && U.page == MUI_PG_STATUS && strstr(row(1), "Y : IP LV") != NULL, "%s: a new fault takes a service page to the status page ('%s')", mn, row(1));
        D.fault[1] = 0U; tick(100U);
        /* the Re-tune item: START walks No, R, Y, B, All */
        ui_reset(mode); MUI_SetFeatures(&U, MUI_FEAT_RETUNE); to_run(); enter_set();
        tick(1600U); for (i = 0; i < 40 && U.item != 19; i++) { key(MK_MENU_SHORT); tick(1600U); }
        CROW(1, "Re-tune"); CROW(2, RT[0]);
        for (i = 1; i <= 5; i++) { key(MK_START); CHECK(crow_ok(2, RT[i % 5]), "%s: START %d: '%s' ('%s')", mn, i, RT[i % 5], row(2)); }
        key(MK_START); key(MK_START); CROW(2, "Y Phase"); key(MK_MENU_SHORT);
        CHECK(MUI_Take(&U, MUI_REQ_RETUNE) && MUI_RetuneSel(&U) == 2U, "%s: Re-tune = Y Phase asks for the Y phase (%u)", mn, (unsigned)MUI_RetuneSel(&U));
        ui_reset(mode); MUI_SetFeatures(&U, MUI_FEAT_RETUNE); to_run(); enter_set(); tick(1600U); for (i = 0; i < 40 && U.item != 19; i++) { key(MK_MENU_SHORT); tick(1600U); }
        for (i = 0; i < 4; i++) key(MK_START);
        key(MK_MENU_SHORT); CHECK(MUI_Take(&U, MUI_REQ_RETUNE) && MUI_RetuneSel(&U) == 4U, "%s: 'All, One By One' asks for all three (%u)", mn, (unsigned)MUI_RetuneSel(&U));
        ui_reset(mode); MUI_SetFeatures(&U, MUI_FEAT_RETUNE); to_run(); enter_set(); tick(1600U); for (i = 0; i < 40 && U.item != 19; i++) { key(MK_MENU_SHORT); tick(1600U); }
        key(MK_MENU_SHORT); CHECK(!MUI_Take(&U, MUI_REQ_RETUNE), "%s: left on No: no request", mn);
    }
    /* Single: Yes / No as before, and the request is the R phase */
    ui_reset(M_MODE_SINGLE); MUI_SetFeatures(&U, MUI_FEAT_RETUNE); to_run(); enter_set(); tick(1600U); for (i = 0; i < 40 && U.item != 19; i++) { key(MK_MENU_SHORT); tick(1600U); }
    key(MK_START); CROW(2, "Yes"); key(MK_START); CROW(2, "No"); key(MK_START); key(MK_MENU_SHORT);
    CHECK(MUI_Take(&U, MUI_REQ_RETUNE) && MUI_RetuneSel(&U) == 1U, "Single: Re-tune Yes asks for the (only) R phase");
    printf("  three-phase service pages (LOG, MENU: monitor ... slave load cal, back to Run), idle and fault exits, Re-tune R / Y / B / All\n");
}

/* v0.9.57: Industrial, a missing phase on the status page: alone, in the middle; two phases named together; Domestic keeps the rows */
static void t_missing_screen(void)
{
    ui_reset(M_MODE_INDUSTRIAL); S.scroll = 0U; to_run(); { int p; for (p = 0; p < 3; p++) D.have[p] = 1U; }
    D.fault[2] = (uint8_t)MLOG_T_PHASE_MISS; tick(100U);
    CHECK(U.page == MUI_PG_STATUS && row(0)[0] == '\0' && crow_ok(1, "PHASE MISSING") && crow_ok(2, "B Phase") && row(3)[0] == '\0', "Industrial, B missing: 'PHASE MISSING' / 'B Phase' alone ('%s' / '%s')", row(1), row(2));
    D.fault[0] = (uint8_t)MLOG_T_PHASE_MISS; tick(100U); CHECK(crow_ok(2, "R B Phase"), "R and B: 'R B Phase' ('%s')", row(2));
    D.fault[0] = 0U; D.fault[2] = 0U; tick(100U); CHECK(strstr(row(0), "R : OK") != NULL, "gone: the rows are back ('%s')", row(0));
    ui_reset(M_MODE_DOMESTIC); S.scroll = 0U; to_run(); { int p; for (p = 0; p < 3; p++) D.have[p] = 1U; }
    D.fault[1] = (uint8_t)MLOG_T_PHASE_MISS; tick(100U);
    CHECK(strcmp(row(0), "R : OK") == 0 && strcmp(row(1), "Y : PH MISSING") == 0, "Domestic: the rows stay, 'Y : PH MISSING' ('%s')", row(1));
    D.fault[1] = 0U; tick(100U);
    printf("  missing phase screen: Industrial alone in the middle (one or two phases), Domestic on its row\n");
}
int main(void)
{
    t_missing_screen();
    t_three_service();
    t_keys();
    t_keys_fuzz();
    t_screens();
    t_pages();
    t_wait_row();
    t_clock_items();
    t_clock_save();
    t_log_stamp();
    t_cal_page();
    t_slave_cal_page();
    t_load_source_item();
    t_overheat_temp_item();
    t_overheat_restart_item();
    t_run_temperature();
    t_vcal_pages();
    t_log_viewer();
    t_setmode_nav();
    t_cursor_and_layout();
    t_setmode_edit();
    t_setmode_rules();
    t_setmode_actions();
    t_password();
    t_run_keys();
    t_ui_fuzz();
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
