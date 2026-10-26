/* End-to-end tests of the master application (m_app): raw key levels with real timing, settings that survive a restart, a failed save, a damaged flash, and the slave's data on the screen. */
#include <stdio.h>
#include <math.h>
/* the limits the tests were written for (valid in the new limits model with Max Correction not set); the NEW defaults have their own tests */
#define LEGACY_LIMITS(s) do { (s).ip_lv = 180U; (s).ip_hv = 280U; (s).op_lv = 200U; (s).op_hv = 250U; } while (0)
#include <string.h>
#include <stdint.h>
#include "m_app.h"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint8_t fm[8 * MFLASH_PAGE]; static int fail_prog;
static int f_read(uint32_t a, uint8_t *b, uint32_t n) { memcpy(b, &fm[a], n); return 0; }
static int f_prog(uint32_t a, const uint8_t *b, uint32_t n) { uint32_t i; if (fail_prog) return 9; for (i = 0U; i < n; i++) fm[a + i] &= b[i]; return 0; }
static int f_erase(uint32_t a) { if (fail_prog) return 9; memset(&fm[a], 0xFF, MFLASH_PAGE); return 0; }
static const MFlash_t FL = { f_read, f_prog, f_erase };
/* the four pages used here: settings at 0 and 1, log at 2 and 3 */
#define SA 0U
#define SB MFLASH_PAGE
#define LA (2U * MFLASH_PAGE)
#define LB (3U * MFLASH_PAGE)

static uint8_t last_clr; static uint32_t clr_at;   /* the slave model: it clears its fault 200 ms after the clear counter changes */
static MApp_t A; static MAppData_t D; static uint32_t T; static char F[4][21]; static uint8_t K[4];
static void boot(void) { MApp_Init(&A, &FL, SA, SB, LA, LB, T); memset(K, 0, sizeof K); }
static void fresh(void) { memset(fm, 0xFF, sizeof fm); fail_prog = 0; memset(&D, 0, sizeof D); D.have = 1U; D.link_ok = 1U; D.vin_dv = 2015U; D.vout_dv = 2300U; D.freq_chz = 5000U; D.state = 2U; D.e_valid = 1U; D.in_band = 1U; D.load_ok = 1U; D.load_da = 123U; D.clr_echo = 0U; T = 50000U; last_clr = 0U; clr_at = 0U; boot();  LEGACY_LIMITS(A.settings); }
static void run(uint32_t ms)
{
    uint32_t e = T + ms;
    while (T < e)
    {
        MApp_Step(&A, K[0], K[1], K[2], K[3], &D, T, F);
        if (MApp_Control(&A)->clr != last_clr) { last_clr = MApp_Control(&A)->clr; clr_at = T + 200U; }
        if ((clr_at != 0U) && ((int32_t)(T - clr_at) >= 0)) { clr_at = 0U; D.fault = 0U; if (D.state == 3U) D.state = 2U; D.clr_echo = last_clr; }
        T += 5U;
    }
}
static void tap(int key, uint32_t down_ms) { K[key] = 1U; run(down_ms); K[key] = 0U; run(120U); }      /* 0 MENU, 1 START, 2 STOP, 3 LOG */
static void combo(void) { K[0] = K[3] = 1U; run(2150U); K[0] = K[3] = 0U; run(200U); }
static const char *row(int r) { static char b[4][22]; int n; strcpy(b[r], F[r]); n = (int)strlen(b[r]); while (n > 0 && b[r][n - 1] == ' ') b[r][--n] = '\0'; return b[r]; }
static int crow_ok(int r, const char *txt) { char b[24]; int n = (int)strlen(txt), i, c = (20 - n) / 2; for (i = 0; i < c; i++) b[i] = ' '; strcpy(&b[c], txt); return strcmp(row(r), b) == 0; }
#define CROW(r, txt) CHECK(crow_ok(r, txt), "row %d is '%s' (expected '%s' centred)", r, row(r), txt)
#define ROW(r, txt) CHECK(strcmp(row(r), txt) == 0, "row %d is '%s' (expected '%s')", r, row(r), txt)
static void to_run(void) { run(MUI_SPLASH_MS + 200U); }

static void t_boot_and_set(void)
{
    fresh();
    CHECK(!A.settings_from_flash && A.settings.set_v == 230U, "a blank flash: the defaults are in use");
    run(100U); ROW(0, "Welcome"); ROW(3, "Wait...");
    to_run(); ROW(0, "O/P Voltage    230 V"); ROW(3, "Start in 02 s");
    CHECK(!MApp_SetModeOpen(&A), "Run-Mode: Set-Mode is closed");
    combo(); CROW(0, "Set Mode"); CROW(1, "Mode");
    CHECK(MApp_SetModeOpen(&A), "MENU+LOG held for 2 s opens Set-Mode with real key timing: the output must be held off");
    tap(0, 100U); tap(0, 100U); CROW(1, "Set Voltage");
    tap(1, 100U); tap(1, 100U); CROW(2, "430 V"); tap(2, 100U); tap(2, 100U); tap(2, 100U);
    /* cursor is back on the hundreds: 430 is out of range; step it back to 2 and use the tens */
    tap(1, 100U); tap(1, 100U); tap(1, 100U); tap(1, 100U); tap(1, 100U); tap(1, 100U); tap(1, 100U); tap(1, 100U);       /* hundreds 4 -> 2: 8 presses wrap past 9, 0, 1 */
    CROW(2, "230 V");
    tap(2, 100U); tap(1, 100U); CROW(2, "240 V");
    tap(0, 100U); CROW(1, "Max Correction");
    combo(); CROW(1, "Settings"); CROW(2, "Saved");
    CHECK(A.settings.set_v == 240U && A.settings_from_flash && !MApp_SetModeOpen(&A), "MENU+LOG again saves: set voltage 240 is live and in flash");
    run(1700U); ROW(0, "O/P Voltage    230 V");
    /* a restart: the settings are still there */
    boot(); CHECK(A.settings_from_flash && A.settings.set_v == 240U, "after a restart the set voltage is 240 V again, read from flash");
    printf("  boot and Set-Mode with raw key timing: open, edit, save, restart, still there\n");
}

static void t_persist(void)
{
    int i; MSettings_t want;
    fresh();
    for (i = 0; i < 5; i++)
    {
        to_run(); combo(); tap(0, 100U); tap(0, 100U); tap(2, 100U); tap(1, 100U); tap(0, 100U); combo();
        want = A.settings;
        boot(); CHECK(A.settings_from_flash && A.settings.set_v == want.set_v && A.settings.set_v == (uint16_t)(230U + 10U * (uint16_t)(i + 1)) , "save %d: after a restart the set voltage is %u (read %u)", i + 1, (unsigned)(230U + 10U * (unsigned)(i + 1)), (unsigned)A.settings.set_v);
        /* keep it valid: the limits must stay above it; relax O/P HV and I/P HV as it rises */
        A.settings.op_hv = 295U; A.settings.ip_hv = 299U; (void)MS_Save(&A.sstore, &A.settings);
    }
    printf("  five saves in a row, each read back after a restart (the two slots alternate)\n");
}

static void t_failures(void)
{
    uint8_t keep[8 * MFLASH_PAGE]; uint32_t b;
    fresh(); to_run(); combo(); tap(0, 100U); tap(0, 100U); tap(2, 100U); tap(1, 100U); tap(0, 100U); combo();
    CHECK(A.settings.set_v == 240U && A.settings_from_flash, "a first good save: 240 V");
    memcpy(keep, fm, sizeof fm);
    /* a save that fails */
    run(1700U); combo(); tap(0, 100U); tap(0, 100U); tap(2, 100U); tap(1, 100U); tap(0, 100U);
    fail_prog = 1; combo(); CROW(1, "Save Failed"); CROW(2, "Flash Error");
    CHECK(A.save_failures == 1U, "a failed save is reported on the screen and counted");
    fail_prog = 0; run(1700U); boot();
    CHECK(A.settings_from_flash && A.settings.set_v == 240U, "and after a restart the previous good settings are intact (240 V, not the 250 V that failed)");
    /* damaged flash: flip every single bit of the newest record in turn: the defaults or the older record, never a wrong value */
    { uint32_t wrong = 0U, tot = 0U; MApp_t t;
      for (b = 0U; b < 64U * 8U; b++)
      {
          memcpy(fm, keep, sizeof fm); fm[SA + b / 8U] ^= (uint8_t)(1U << (b % 8U)); fm[SB + b / 8U] ^= 0U;
          MApp_Init(&t, &FL, SA, SB, LA, LB, T); tot++;
          if (t.settings_from_flash && t.settings.set_v != 240U) wrong++;
          if (MS_Validate(&t.settings, 0) != MS_OK) wrong++;
      }
      CHECK(wrong == 0U, "%u single-bit corruptions of the stored settings: never a wrong value, never an invalid set (%u wrong)", (unsigned)tot, (unsigned)wrong); }
    memcpy(fm, keep, sizeof fm); memset(&fm[SA], 0x00, 64U); memset(&fm[SB], 0x00, 64U);
    boot(); CHECK(!A.settings_from_flash && A.settings.set_v == 230U, "both slots destroyed: the defaults are used and the unit still works");
    to_run(); ROW(0, "O/P Voltage    230 V");
    combo(); tap(0, 100U); tap(0, 100U); tap(2, 100U); tap(1, 100U); tap(0, 100U); fail_prog = 0; combo();
    CHECK(A.settings_from_flash && A.save_failures == 0U, "and the next save works (the page is erased first)");
    printf("  failures: a failed save leaves the old settings, 512 bit flips never give a wrong value, a destroyed flash gives defaults and recovers\n");
}

static void t_data(void)
{
    static const struct { uint8_t code, state; const char *text; } F_[] = { { 8, 3, "Mains Unstable" }, { 2, 3, "Inrush" }, { 7, 3, "Power Stage Fault" }, { 17, 3, "Transformer Flip" }, { 6, 3, "Output HV" }, { 19, 3, "Re-tune Required" } };
    unsigned i;
    fresh(); to_run();
    ROW(0, "O/P Voltage    230 V"); ROW(1, "I/P Voltage    202 V"); ROW(2, "Load  12.3A   50.0Hz"); ROW(3, "Start in 02 s");
    CHECK(1, "the output is OFF until the engine has started the slave and IN_BAND has held for the Restart Time");
    run(9000U); ROW(3, "Output ON");
    CHECK(MApp_Engine(&A)->contactor[0] == 1U, "then the contactor is closed: 'Output ON' now means the contactor");
    D.load_ok = 0U; run(200U); ROW(2, "Load  --.-A   50.0Hz"); D.load_ok = 1U; run(200U);
    D.have = 0U; D.link_ok = 0U; run(200U); ROW(0, "O/P Voltage    --- V"); ROW(3, "Slave Link Lost");
    CHECK(MApp_Engine(&A)->contactor[0] == 0U, "a lost link opens the contactor at once");
    /* every slave fault code, shown while the unit is not yet running (the display never hides what the slave reports) */
    for (i = 0U; i < sizeof F_ / sizeof F_[0]; i++)
    {
        char ex[21]; size_t n = strlen(F_[i].text);
        fresh(); to_run(); D.fault = F_[i].code; D.state = F_[i].state; run(200U);
        memset(ex, ' ', 20); ex[20] = '\0'; memcpy(ex, F_[i].text, n);
        if ((F_[i].code == 8U) || (F_[i].code == 2U) || (F_[i].code == 7U)) { ex[17] = 'F'; ex[18] = (char)('0' + (F_[i].code / 10U)); ex[19] = (char)('0' + (F_[i].code % 10U)); }   /* Mains / Inrush / Power stage show the slave's code at the right */
        { int e = 19; while ((e >= 0) && (ex[e] == ' ')) { ex[e] = '\0'; e--; } }
        ROW(3, ex);
    }
    fresh(); to_run(); D.fault = 9U; D.vin_dv = 1700U; D.state = 3U; run(200U); ROW(3, "Input LV");
    fresh(); to_run(); D.fault = 9U; D.vin_dv = 2900U; D.state = 3U; run(200U); ROW(3, "Input HV");
    fresh(); to_run(); run(200U);
    /* MENU toggles the monitor diagnostics */
    tap(0, 100U); CHECK(!MApp_ShowsMonitor(&A), "MENU does nothing on the single Run screen");
    tap(3, 100U); tap(0, 100U); CHECK(MApp_ShowsMonitor(&A), "LOG then MENU opens the monitor diagnostics");
    tap(0, 100U); tap(0, 100U); tap(0, 100U); CHECK(!MApp_ShowsMonitor(&A), "and three more MENU presses cycle back to the Run screen");
    printf("  slave data on the screen: voltages, load, the contactor state, every fault code, no link\n");
}

static void t_log(void)
{
    MLogRec_t r; int i;
    fresh(); to_run();
    for (i = 0; i < 3; i++) { r.type = (uint8_t)(1 + i); r.phase = 0U; r.slave_code = 0U; r.vin_dv = 2000U; r.vout_dv = 2300U; r.load_da = 100U; r.run_min = (uint32_t)(60 * (i + 1)); MLog_Append(&A.log, &r); }
    tap(3, 100U); ROW(0, "Fault Log 00/02"); ROW(1, "R Output LV");
    CHECK(strstr(row(3), "3h00m") != NULL, "the log opens with LOG: the newest is 00 (%s)", row(3));
    tap(3, 100U); CHECK(strstr(row(0), "O/P") != NULL, "LOG closes it");
    /* Clear Fault Log through Set-Mode */
    combo(); { int n; for (n = 0; n < 40 && A.ui.item != 20U; n++) { tap(0, 100U); run(1600U); } }
    CROW(1, "Clear Fault Log"); tap(1, 100U); CROW(2, "Yes"); tap(0, 100U);
    CHECK(MLog_Count(&A.log) == 0U, "Clear Fault Log = Yes empties the log (%u left)", (unsigned)MLog_Count(&A.log));
    boot(); CHECK(MLog_Count(&A.log) == 0U, "and it stays empty after a restart");
    printf("  fault log: opened by LOG, newest first, cleared from Set-Mode, stays cleared\n");
}

static void t_setmode_isolation(void)
{
    fresh(); to_run(); combo(); CHECK(MApp_SetModeOpen(&A), "open");
    /* while Set-Mode is open, START and STOP must not leak anywhere: there is no output here, so the only observable is that they edit and the screen stays in Set-Mode */
    tap(1, 100U); tap(2, 100U); CHECK(MApp_SetModeOpen(&A), "START and STOP in Set-Mode stay in Set-Mode");
    run(30500U); CHECK(!MApp_SetModeOpen(&A), "30 s with no key: closed, nothing saved");
    boot(); CHECK(!A.settings_from_flash, "nothing was written to flash");
    printf("  Set-Mode isolation: idle exit writes nothing\n");
}

static void t_slave_control(void)
{
    const MCtlOut_t *c; uint32_t k; int opened_in_step = 0;
    /* Auto: the unit is told to run after the Restart Time (5 s) */
    fresh(); c = MApp_Control(&A); to_run(); CHECK(c->run == 0U, "Auto: not yet at the end of the splash (3.2 s)"); run(2000U);
    CHECK(c->run == 1U && c->setv_dv == 2300U && c->maxc_dv == 0U, "Auto: after the splash and the 5 s on-delay the slave is told run = 1, Set Voltage 2300, Max Correction 0 (run %u, setv %u)", (unsigned)c->run, (unsigned)c->setv_dv);
    /* Set-Mode: run = 0 in the SAME step that opens it */
    fresh(); to_run(); run(2100U); CHECK(MApp_Control(&A)->run == 1U, "running");
    K[0] = K[3] = 1U;
    for (k = 0U; k < 600U && !MApp_SetModeOpen(&A); k++) { MApp_Step(&A, K[0], K[1], K[2], K[3], &D, T, F); T += 5U; if (MApp_SetModeOpen(&A)) opened_in_step = (MApp_Control(&A)->run == 0U); }
    CHECK(MApp_SetModeOpen(&A) && opened_in_step, "the very step that opens Set-Mode already has run = 0: the slave stops its PWM and trips the output");
    K[0] = K[3] = 0U; run(300U);
    run(20000U); CHECK(MApp_Control(&A)->run == 0U, "and run stays 0 while Set-Mode is open (20 s later)");
    /* a new Set Voltage reaches the slave after saving, not while it is only being edited */
    tap(0, 100U); tap(0, 100U); tap(2, 100U); tap(1, 100U);                                   /* Set Voltage 230 -> 240 in the working copy */
    CHECK(MApp_Control(&A)->setv_dv == 2300U, "while editing, the slave still gets the OLD set voltage (2300)");
    combo(); run(300U);
    CHECK(!MApp_SetModeOpen(&A) && A.settings.set_v == 240U && MApp_Control(&A)->setv_dv == 2400U, "after saving, the slave is told 2400 (the Set Voltage 240 V)");
    CHECK(MApp_Control(&A)->run == 0U, "and run is still 0: leaving Set-Mode (Auto) waits the Restart Time");
    run(3500U); CHECK(MApp_Control(&A)->run == 0U, "about 4 s after leaving: not yet");
    run(1600U); CHECK(MApp_Control(&A)->run == 1U && MApp_Control(&A)->setv_dv == 2400U, "after the Restart Time: run = 1 with the new set voltage");
    /* Max Correction */
    combo(); tap(0, 100U); tap(0, 100U); tap(0, 100U);                                         /* Max Correction */
    tap(2, 100U); tap(1, 100U); tap(1, 100U); tap(1, 100U); tap(1, 100U);   /* 040 V: the tens digit, 4 presses */
    combo(); run(300U); CHECK(A.settings.maxcorr_v == 40U && MApp_Control(&A)->maxc_dv == 400U, "Max Correction 40 V is sent as 400 (setting %u, sent %u, limits %u-%u / %u-%u)", (unsigned)A.settings.maxcorr_v, (unsigned)MApp_Control(&A)->maxc_dv, (unsigned)A.settings.ip_lv, (unsigned)A.settings.ip_hv, (unsigned)A.settings.op_lv, (unsigned)A.settings.op_hv);
    /* a discarded edit never reaches the slave */
    fresh(); to_run(); combo(); tap(0, 100U); tap(0, 100U); tap(2, 100U); tap(1, 100U); run(31000U);
    CHECK(!MApp_SetModeOpen(&A) && MApp_Control(&A)->setv_dv == 2300U, "an edit abandoned by the 30 s timeout is never sent (still 2300)");
    /* START and STOP in Run-Mode */
    fresh(); to_run(); run(2100U); CHECK(MApp_Control(&A)->run == 1U, "running"); tap(2, 100U); CHECK(MApp_Control(&A)->run == 0U, "STOP in Run-Mode: run = 0");
    run(30000U); CHECK(MApp_Control(&A)->run == 0U, "and it stays 0");
    tap(1, 100U); CHECK(MApp_Control(&A)->run == 1U, "START: run = 1");
    /* a slave fault: Auto changes the counter every Restart Time */
    { uint8_t c0; run(600U); c0 = MApp_Control(&A)->clr; D.state = 3U; D.fault = 8U; run(7000U); CHECK(MApp_Control(&A)->clr != c0 && MApp_Control(&A)->run == 1U, "a slave fault in Auto: the clear counter changes after the Restart Time, run stays 1"); D.state = 2U; D.fault = 0U; }
    /* Manual */
    fresh(); A.settings.restart_auto = 0U; MF_Init(&A.eng, &A.settings, T); run(MUI_SPLASH_MS + 10000U);
    CHECK(MApp_Control(&A)->run == 0U, "Manual: the slave is not started by itself");
    tap(1, 100U); CHECK(MApp_Control(&A)->run == 1U, "START in Manual starts it");
    printf("  telling the slave: run at once and stop at once, Set-Mode = run 0 in the same step, settings sent after saving, a discarded edit never sent, START/STOP, fault clearing\n");
}

static void t_engine_inputs(void)
{
    MLogRec_t r;
    /* the clear counter starts from the slave's own: nothing is sent before the slave has been heard, and a master restart never looks like a "clear" */
    fresh(); D.e_valid = 0U; run(2000U);
    CHECK(MApp_ControlValid(&A) == 0U, "before any $E from the slave, no $M may be sent (the counter is not in step yet)");
    D.e_valid = 1U; D.clr_echo = 7U; run(100U);
    CHECK(MApp_ControlValid(&A) == 1U && MApp_Control(&A)->clr == 7U, "after the first $E the counter is the slave's own (7), not 0 (got %u)", (unsigned)MApp_Control(&A)->clr);
    /* the slave's own limits (its $H): Set-Mode refuses what the slave could never reach */
    fresh(); D.h_valid = 1U; D.win_lo_dv = 1500U; D.win_hi_dv = 2500U; D.vout_max_dv = 2400U; run(200U);
    to_run(); combo(); CHECK(MApp_SetModeOpen(&A), "Set-Mode open");
    combo(); run(100U);
    CHECK(MApp_SetModeOpen(&A) && strstr(row(1), "Invalid Value") != NULL, "the default I/P HV 280 V is above the slave's window (250 V) and O/P HV 250 V is above its guard (240 V): leaving is refused (%s / %s)", row(1), row(2));
    /* the load from the master CT: 120 A against 99 A for the 10 s Overload Cut Delay trips the output; START only */
    fresh(); to_run(); run(9000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "running with the contactor closed");
    D.load_da = 1200U; run(9000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "9 s of overload is not yet a trip");
    run(1500U);
    CHECK(MApp_Engine(&A)->contactor[0] == 0U && strcmp(row(3), "Overload") == 0, "after the 10 s Overload Cut Delay the contactor opens and the screen says Overload (%s)", row(3));
    CHECK(MLog_Count(&A.log) == 1U && MLog_Get(&A.log, 0U, &r) == 0 && r.type == MLOG_T_OVERLOAD && r.load_da == 1200U, "it is logged with the load: 120.0 A");
    D.load_da = 100U; run(60000U); CHECK(MApp_Engine(&A)->contactor[0] == 0U && MApp_Control(&A)->run == 0U, "overload is START only: a minute later nothing has restarted it, even in Auto");
    tap(1, 100U); run(12000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "START restarts it");
    printf("  engine inputs: the slave's clear counter, the slave's limits in Set-Mode, the CT load causing an Overload trip that waits for START\n");
}

static void t_status_and_diag(void)
{
    /* the status row says what the unit is waiting for, so it is never a mystery why nothing starts */
    fresh(); run(MUI_SPLASH_MS + 200U); ROW(3, "Start in 02 s");
    run(1000U); ROW(3, "Start in 01 s");
    run(1900U); CHECK(strcmp(row(3), "At Setpoint...") == 0 || strcmp(row(3), "Output ON") == 0, "after the on-delay the output is at the set voltage and the contactor follows 1 s later (%s)", row(3));
    run(2000U); ROW(3, "Output ON");
    fresh(); A.settings.restart_auto = 0U; MF_Init(&A.eng, &A.settings, T); run(MUI_SPLASH_MS + 5000U); ROW(3, "Press START");
    tap(1, 100U); run(300U); CHECK(strcmp(row(3), "At Setpoint...") == 0, "after START the output is at the set voltage: the contactor closes within a second (%s)", row(3));
    run(1500U); ROW(3, "Output ON");
    /* rising: the screen shows the set voltage and where the output is */
    fresh(); D.vout_dv = 2000U; D.state = 1U; D.in_band = 0U; run(MUI_SPLASH_MS + 6000U); ROW(3, "To 230 V (200)");
    CHECK(MApp_Engine(&A)->contactor[0] == 0U, "while the output is 30 V below the set voltage the contactor stays open");
    D.vout_dv = 2250U; D.state = 2U; run(500U); ROW(3, "To 230 V (225)");
    D.vout_dv = 2290U; D.in_band = 1U; run(500U); CHECK(strcmp(row(3), "At Setpoint...") == 0, "within 2 V of the set voltage: at setpoint (%s)", row(3));
    run(1200U); CHECK(MApp_Engine(&A)->contactor[0] == 1U && strcmp(row(3), "Output ON") == 0, "and 1 s later the contactor closes (%s)", row(3));
    /* a slave whose $E is not understood: $M still goes out after 3 s of $S, and the screen says so */
    fresh(); D.e_valid = 0U; D.in_band = 0U; run(2800U);
    CHECK(MApp_ControlValid(&A) == 0U, "no $E and only 2.8 s of $S: $M is not sent yet");
    run(500U); CHECK(MApp_ControlValid(&A) == 1U, "after 3 s of $S frames $M may be sent even without an $E");
    run(5000U); CHECK(MApp_Control(&A)->run == 1U, "the unit is started anyway");
    CHECK(MApp_Engine(&A)->contactor[0] == 1U, "and the contactor closes without any $E: the master's own reading of the output (230 V = the set voltage) is enough");
    fresh(); D.have = 0U; D.link_ok = 0U; D.e_valid = 0U; run(10000U); CHECK(MApp_ControlValid(&A) == 0U, "never heard from the slave at all: nothing is sent");
    /* the diagnostics page */
    fresh(); run(MUI_SPLASH_MS + 6000U); tap(3, 100U); tap(0, 100U); tap(0, 100U);
    CHECK(MApp_ShowsDiag(&A), "two MENU presses show the control diagnostics");
    MApp_NoteFrameSent(&A); MApp_NoteFrameSent(&A); MApp_Diag(&A, T, F);
    CHECK(strncmp(F[0], "S:Y E:Y H:N M:00002", 19) == 0, "row 0: heard from the slave and the frames sent ('%s')", row(0));
    CHECK(strncmp(F[1], "U:STARTING run:1 c:", 19) == 0 || strncmp(F[1], "U:ON run:1 c:", 13) == 0, "row 1: the unit state and run level ('%s')", row(1));
    CHECK(strncmp(F[2], "Vin 202 180-280", 15) == 0, "row 2: the input voltage and the limits ('%s')", row(2));
    CHECK(strncmp(F[3], "Slv st:2 F:00 inb:1", 19) == 0, "row 3: the slave's state, fault code and IN_BAND ('%s')", row(3));
    printf("  status text and diagnostics: the countdown, Press START, No $E, the 3 s rule, the diagnostics page\n");
}

static void t_display_stability(void)
{
    uint32_t k, changes = 0U; char prev[21], prev1[21];
    /* a reading that straddles a rounding boundary (230.4 / 230.7 V, alternating every 20 ms) must not make the last digit flicker */
    fresh(); to_run(); run(3000U); strcpy(prev, row(0)); strcpy(prev1, row(1));
    for (k = 0U; k < 3000U; k++) { D.vout_dv = (uint16_t)((k & 1U) ? 2307U : 2304U); D.vin_dv = (uint16_t)((k & 1U) ? 2154U : 2157U); run(20U); if (strcmp(row(0), prev) != 0) { changes++; strcpy(prev, row(0)); } }
    CHECK(changes <= 1U, "a reading alternating across the rounding boundary for 60 s changes the shown O/P voltage at most once (%u changes)", (unsigned)changes);
    { uint32_t ch1 = 0U; for (k = 0U; k < 3000U; k++) { D.vin_dv = (uint16_t)((k & 1U) ? 2154U : 2157U); run(20U); if (strcmp(row(1), prev1) != 0) { ch1++; strcpy(prev1, row(1)); } } CHECK(ch1 <= 1U, "and the I/P voltage the same (%u changes)", (unsigned)ch1); }
    /* random noise of +-0.6 V around 230.5: still steady */
    { uint32_t seed = 7U, ch = 0U; fresh(); to_run(); run(3000U); strcpy(prev, row(0));
      for (k = 0U; k < 6000U; k++) { seed = seed * 1664525U + 1013904223U; D.vout_dv = (uint16_t)(2305 + (int)((seed >> 12) % 13U) - 6); run(10U); if (strcmp(row(0), prev) != 0) { ch++; strcpy(prev, row(0)); } }
      CHECK(ch <= 2U, "random noise of +-0.6 V around 230.5 V for 60 s: the shown value changes at most twice (%u)", (unsigned)ch); }
    /* a real step is followed at once; a slow drift is followed too */
    fresh(); to_run(); run(3000U); D.vout_dv = 2400U; run(300U); ROW(0, "O/P Voltage    240 V");
    D.vout_dv = 2312U; run(10000U); ROW(0, "O/P Voltage    231 V");
    D.vout_dv = 2380U; run(100U); ROW(0, "O/P Voltage    238 V");
    CHECK(1, "a step of 5 V or more shows at once; a 1.2 V change shows within seconds");
    /* the load current too: +-0.2 A of noise on 12.3 A */
    { uint32_t seed = 9U, ch = 0U; fresh(); to_run(); run(3000U); strcpy(prev, row(2));
      for (k = 0U; k < 6000U; k++) { seed = seed * 1664525U + 1013904223U; D.load_da = (uint16_t)(123 + (int)((seed >> 12) % 5U) - 2); run(10U); if (strcmp(row(2), prev) != 0) { ch++; strcpy(prev, row(2)); } }
      CHECK(ch <= 2U, "the load shows steady with +-0.2 A of noise (%u changes in 60 s)", (unsigned)ch); }
    /* no reading: dashes, and the filter restarts from the next reading */
    fresh(); to_run(); run(1000U); D.have = 0U; D.link_ok = 0U; run(300U); ROW(0, "O/P Voltage    --- V");
    D.have = 1U; D.link_ok = 1U; D.vout_dv = 2500U; run(100U); ROW(0, "O/P Voltage    250 V");
    printf("  display stability: no flicker across a rounding boundary or with noise, steps followed at once, whole volts, dashes\n");
}

static void t_no_cuts(void)
{
    /* TEST switch: the master applies no voltage limits */
    fresh(); MApp_SetTestNoCuts(&A, 0U); D.vin_dv = 1200U; run(MUI_SPLASH_MS + 20000U);
    CHECK(MApp_Engine(&A)->contactor[0] == 0U && MApp_Control(&A)->run == 0U, "standard: 120 V in (the I/P LV is 180 V): the unit does not start");
    fresh(); MApp_SetTestNoCuts(&A, 1U); D.vin_dv = 1200U; run(MUI_SPLASH_MS + 14000U);
    CHECK(MApp_Engine(&A)->contactor[0] == 1U, "test switch on: it starts and the contactor closes at 120 V");
    D.vin_dv = 3500U; D.vout_dv = 3200U; run(30000U);
    CHECK(MApp_Engine(&A)->contactor[0] == 1U && MLog_Count(&A.log) == 0U, "350 V in and 320 V out for 30 s: no trip, nothing logged");
    D.vin_dv = 500U; D.vout_dv = 500U; run(30000U);
    CHECK(MApp_Engine(&A)->contactor[0] == 1U && MLog_Count(&A.log) == 0U, "50 V in and out for 30 s: no trip either (no I/P LV, no O/P LV)");
    D.load_da = 1200U; run(11000U);
    CHECK(MApp_Engine(&A)->contactor[0] == 0U && MLog_Count(&A.log) == 1U, "but an overload still trips: only the VOLTAGE cuts are off");
    fresh(); MApp_SetTestNoCuts(&A, 1U); run(MUI_SPLASH_MS + 7000U); MApp_Diag(&A, T, F); CHECK(strstr(F[2], "NOCUT") != NULL, "the diagnostics page says NOCUT ('%s')", F[2]);
    printf("  test switch: no voltage cuts in the master (start, high and low input and output), overload still trips\n");
}

#define CALA (4U * MFLASH_PAGE)
#define CALB (5U * MFLASH_PAGE)
static void fresh_cal(void) { fresh(); MApp_InitCal(&A, CALA, CALB); D.ct_raw = 1U; D.ct_x16 = (uint16_t)(141U * 16U); }
static int load_shows(const char *s) { return strstr(row(2), s) != NULL; }

static void t_load_calibration(void)
{
    uint32_t C;
    /* the DEFAULTS reproduce the original firmware: counts -> amps = (counts / 4) * 65.25 / 766, minus 3 A */
    fresh_cal(); to_run();
    for (C = 160U; C <= 3000U; C += 140U)
    {
        double old = ((double)(C >> 2) * 65.25 / 766.0 - 3.0) * 10.0, got;
        if (old < 0.0) old = 0.0;
        D.ct_x16 = (uint16_t)(C * 16U); run(300U);                 /* what the application computes from this CT count, through its screen's filter */
        got = (double)A.last.ct_x16 / 16.0;                         /* the count it was given */
        got = (got - 141.0) * 0.213; if (got < 0.0) got = 0.0; if (got > 600.0) got = 600.0;
        CHECK(got - old < 3.0 && old - got < 3.0, "defaults: %u counts: the original firmware %.1f dA, the new formula %.1f dA (within 0.3 A)", (unsigned)C, old, got);
    }
    /* YOUR CASE: the old scale shows 3.0 A while the clamp meter says 6.6 A (282 counts) */
    fresh_cal(); to_run(); D.ct_x16 = (uint16_t)(282U * 16U); run(3000U);
    CHECK(load_shows("3.0") || load_shows("2.9") || load_shows("3.1"), "uncalibrated: the screen shows 3.0 A, within its 0.3 A hysteresis ('%s')", row(2));
    /* ZERO: Set-Mode, Calibration = Yes, with no load (the CT reads 141 counts, as the original firmware assumed) */
    D.ct_x16 = (uint16_t)(141U * 16U); combo(); tap(0, 100U); tap(1, 100U); ROW(1, "    Calibration"); tap(0, 100U);
    CHECK(A.cal.zero_x16 == 141U * 16U && A.cal.from_flash, "Set-Mode, Calibration = Yes: the zero is taken from the CT now (141 counts)");
    ROW(1, "      Zero Set");
    run(2000U); combo(); run(400U);                                    /* leave Set-Mode */
    /* SPAN: the Load Calibration page, MENU x3; the load runs at 6.6 A (the CT reads 282 counts) */
    D.ct_x16 = (uint16_t)(282U * 16U); run(15000U);
    tap(3, 100U); tap(0, 100U); tap(0, 100U); tap(0, 100U); CHECK(MUI_ShowsCal(&A.ui), "LOG, then MENU x3 opens the Load Calibration page");
    run(1500U); CHECK(strstr(row(1), "3.0") != NULL || strstr(row(1), "2.9") != NULL || strstr(row(1), "3.1") != NULL, "it shows the reading, 3.0 A within the hysteresis (%s)", row(1));
    tap(1, 100U); { uint32_t v0 = MUI_CalValue(&A.ui); CHECK(MUI_CalValue(&A.ui) >= 29U && MUI_CalValue(&A.ui) <= 41U, "START edits the entry that started from the shown reading (%u)", (unsigned)v0); }
    A.ui.cal_val = 66U;                                                /* 6.6 A typed in (the digit editing itself is tested in the UI tests; the display may show 2.9 or 3.0 for the same reading, so the entry is set directly) */
    CHECK(MUI_CalValue(&A.ui) == 66U, "6.6 A typed in");
    tap(3, 100U);                                                      /* LOG: save */
    ROW(1, "     Calibrated");
    CHECK(A.cal.gain_x1000 >= 2190U && A.cal.gain_x1000 <= 2206U, "the gain is 2.198: 6.6 A / 3.003 A (stored %u)", (unsigned)A.cal.gain_x1000);
    tap(0, 100U); run(6000U);
    CHECK(load_shows("6.6") || load_shows("6.5") || load_shows("6.7"), "calibrated: the screen shows 6.5 to 6.7 A at the same CT reading ('%s')", row(2));
    /* it survives a restart */
    { uint16_t z = A.cal.zero_x16, g = A.cal.gain_x1000; boot(); MApp_InitCal(&A, CALA, CALB); D.ct_raw = 1U; D.ct_x16 = (uint16_t)(282U * 16U);
      CHECK(A.cal.from_flash && A.cal.zero_x16 == z && A.cal.gain_x1000 == g, "after a restart the calibration is read back from flash (zero %u, gain %u)", (unsigned)A.cal.zero_x16, (unsigned)A.cal.gain_x1000);
      to_run(); run(4000U); CHECK(load_shows("6.6") || load_shows("6.5") || load_shows("6.7"), "and the display is right straight away ('%s')", row(2)); }
    /* the checks of the spec: a load below 2.0 A is refused, so is a value outside 2.0 to 180.0 A, so is a gain that no CT could give */
    fresh_cal(); to_run(); D.ct_x16 = (uint16_t)(141U * 16U + 40U * 16U); run(1500U);        /* 40 counts above the zero = 0.85 A nominal */
    tap(3, 100U); tap(0, 100U); tap(0, 100U); tap(0, 100U); tap(3, 100U); ROW(1, "    Load Too Low"); CHECK(A.cal.gain_x1000 == 1000U, "a load below 2.0 A: 'Load Too Low', nothing changed");
    run(2000U); D.ct_x16 = (uint16_t)(141U * 16U + 300U * 16U); run(1500U);                    /* 6.4 A nominal */
    A.ui.cal_val = 15U; tap(3, 100U); CHECK(strstr(row(1), "Enter") != NULL && A.cal.gain_x1000 == 1000U, "a typed value below 2.0 A is refused (%s)", row(1));
    run(2000U); A.ui.cal_val = 1900U; tap(3, 100U); CHECK(strstr(row(1), "Enter") != NULL && A.cal.gain_x1000 == 1000U, "and one above 180.0 A (%s)", row(1));
    run(2000U); A.ui.cal_val = 1800U; tap(3, 100U); CHECK(strstr(row(1), "Out Of Range") != NULL && A.cal.gain_x1000 == 1000U, "a 180 A entry against 6.4 A on the CT (a gain of 28) is 'Out Of Range' (%s)", row(1));
    /* the zero is refused while there is a load: the CT reads far above any zero */
    fresh_cal(); to_run(); D.ct_x16 = (uint16_t)(900U * 16U); run(500U); combo(); tap(0, 100U); tap(1, 100U); tap(0, 100U);
    ROW(1, "   Load Not Zero"); CHECK(A.cal.zero_x16 == 141U * 16U, "Set-Mode Calibration with the CT reading 900 counts: 'Load Not Zero', the zero is unchanged");
    /* a failed save leaves the old calibration */
    fresh_cal(); to_run(); D.ct_x16 = (uint16_t)(282U * 16U); run(1500U); tap(3, 100U); tap(0, 100U); tap(0, 100U); tap(0, 100U); A.ui.cal_val = 66U; fail_prog = 1; tap(3, 100U); fail_prog = 0;
    ROW(1, "    Save Failed"); CHECK(A.cal.gain_x1000 == 1000U, "a failed flash write: the gain is back to the old value");
    /* Factory Reset keeps the calibration (a separate record) */
    printf("  load calibration: defaults equal the original firmware, your 3.0 A -> 6.6 A case, zero and span, flash, the 2.0 A and range checks\n");
}

static void t_slave_wait(void)
{
    /* YOUR VIDEO: the master commands run, the slave answers "starting" (state 1) with no fault, never reaches RUN, the output stays at the input. The slave says WHY in its status frame. */
    fresh(); D.state = 1U; D.in_band = 0U; D.vout_dv = 2160U; D.vin_dv = 2160U; run(MUI_SPLASH_MS + 7000U);
    CHECK(MApp_Control(&A)->run == 1U && strcmp(row(3), "To 230 V (216)") == 0, "no wait reason sent: the screen shows the rise to the set voltage (%s)", row(3));
    D.wait_valid = 1U; D.wait_ctx = 0x10U; run(300U); ROW(3, "Slave Waits:B");
    D.wait_ctx = 0x08U; run(300U); ROW(3, "Slave Waits:F");
    D.wait_ctx = 0x01U | 0x04U | 0x40U; run(300U); ROW(3, "Slave Waits:ZMR");
    D.wait_ctx = 0x7FU; run(300U); ROW(3, "Slave Waits:ZVMFBOR");
    D.wait_valid = 1U; D.wait_ctx = 0U; run(300U); CHECK(strcmp(row(3), "To 230 V (216)") == 0, "'nothing is missing' shows the normal rise text (%s)", row(3));
    D.wait_ctx = 0x10U; D.wait_valid = 0U; run(300U); CHECK(strncmp(row(3), "Slave Waits", 11) != 0, "a stale wait value without the valid flag is not shown (%s)", row(3));
    D.wait_valid = 1U; D.wait_ctx = 0x18U; run(300U); MApp_Diag(&A, T, F);
    CHECK(strncmp(F[3], "Slv st:1 F:00 W:FB", 18) == 0, "diagnostics row 3 shows W:FB ('%s')", F[3]);
    D.wait_valid = 0U; D.state = 2U; run(300U); MApp_Diag(&A, T, F);
    CHECK(strncmp(F[3], "Slv st:2 F:00 inb:0", 19) == 0 || strncmp(F[3], "Slv st:2 F:00 inb:1", 19) == 0, "without a wait the page is as before ('%s')", F[3]);
    printf("  slave start-up wait shown: Z V M F B O R letters on the status row and the diagnostics page\n");
}

static void t_inrush_retries_app(void)
{
    MLogRec_t r;
    fresh(); MApp_SetInrushRetries(&A, 0U); to_run(); run(12000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "running");
    D.state = 3U; D.fault = 2U; run(300U);
    CHECK(MApp_Engine(&A)->contactor[0] == 0U && strcmp(row(3), "Inrush           F02") == 0, "the slave reports FLT (code 2): the contactor opens and the screen says Inrush with its code (%s)", row(3));
    D.fault = 0U; D.state = 0U; run(120000U);
    CHECK(MApp_Control(&A)->run == 0U && A.eng.u[0].st == (uint8_t)U_LATCHED && MLog_Count(&A.log) == 1U && MLog_Get(&A.log, 0U, &r) == 0 && r.type == MLOG_T_INRUSH,
          "with 0 retries it is latched: two minutes later the slave is still not told to run, one entry in the log");
    fresh(); MApp_SetInrushRetries(&A, 3U); to_run(); run(12000U); D.state = 3U; D.fault = 2U; run(300U); D.fault = 0U; D.state = 2U; run(40000U);
    CHECK(MApp_Control(&A)->run == 1U, "with 3 retries it tries again after 30 s");
    printf("  inrush retries in the application: 0 latches at once, 3 retries\n");
}

static void t_inverter_off(void)
{
    MLogRec_t r;
    /* the slave stops WITHOUT any fault code (mains unstable, its own link timeout ...): PWM off means the load must be off too */
    fresh(); to_run(); run(12000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "running with the contactor closed");
    D.state = 0U; run(150U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "the slave has been off for 0.15 s: not yet (a glitch is tolerated)");
    run(150U); CHECK(MApp_Engine(&A)->contactor[0] == 0U && MApp_Control(&A)->run == 0U, "off for 0.3 s: the contactor is OPEN and the master stops commanding run");
    CHECK(strcmp(row(3), "Mains Unstable") == 0, "the screen says Mains Unstable (%s)", row(3));
    CHECK(MLog_Count(&A.log) == 1U && MLog_Get(&A.log, 0U, &r) == 0 && r.type == MLOG_T_MAINS && r.slave_code == 0U, "logged once, as Mains, with no slave code");
    /* Auto: it restarts by itself once the Restart Time has passed and the slave is running again */
    D.state = 2U; run(9000U); CHECK(MApp_Control(&A)->run == 1U, "Auto: the master tries again after the Restart Time"); run(4000U);
    CHECK(MApp_Engine(&A)->contactor[0] == 1U, "and the contactor closes again when the output is at the set voltage");
    /* a flicker of the slave's state shorter than 0.2 s changes nothing */
    D.state = 1U; run(100U); D.state = 2U; run(3000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "a 0.1 s glitch of the state does not open it");
    /* fault state 3 without a code, and state 1 (unwinding) held: both open it */
    D.state = 3U; D.fault = 0U; run(400U); CHECK(MApp_Engine(&A)->contactor[0] == 0U, "state 3 without a code opens it");
    /* STARTING is not affected: the slave is allowed to be 'starting' before the contactor closes */
    fresh(); D.state = 1U; D.in_band = 0U; D.vout_dv = 2160U; run(MUI_SPLASH_MS + 9000U);
    CHECK(MApp_Control(&A)->run == 1U && MLog_Count(&A.log) == 0U && MApp_Engine(&A)->contactor[0] == 0U, "while STARTING the slave may be in state 1: nothing is tripped, nothing logged");
    /* a slave that is not being heard at all is the link-lost rule, not this one */
    fresh(); to_run(); run(12000U); D.have = 0U; D.link_ok = 0U; run(300U); CHECK(MApp_Engine(&A)->contactor[0] == 0U && strcmp(row(3), "Slave Link Lost") == 0, "no frames from the slave: Slave Link Lost, contactor open");
    printf("  inverter off = load off: the contactor opens 0.2 s after the slave leaves RUNNING, whatever the reason; STARTING and glitches are tolerated\n");
}

static void t_hello_and_echo(void)
{
    /* the master asks for the slave's $H when it missed it */
    fresh(); D.e_valid = 1U; D.h_valid = 0U; run(MUI_SPLASH_MS + 500U);
    CHECK(MApp_HelloDue(&A, T) == 1U, "$E heard, no $H: the master asks for the hello");
    CHECK(MApp_HelloDue(&A, T) == 0U, "and not twice at once");
    run(1900U); CHECK(MApp_HelloDue(&A, T) == 0U, "1.9 s later: still waiting for the answer");
    run(300U); CHECK(MApp_HelloDue(&A, T) == 1U, "2.2 s later with no answer: it asks again");
    D.h_valid = 1U; run(300U); run(2500U); CHECK(MApp_HelloDue(&A, T) == 0U, "once the $H has arrived it never asks");
    fresh(); D.e_valid = 0U; D.h_valid = 0U; run(MUI_SPLASH_MS + 4000U); CHECK(MApp_HelloDue(&A, T) == 0U, "no $E (a slave without the v2 link): it does not ask");
    /* the diagnostics page shows the set voltage the slave is using */
    fresh(); D.e_valid = 1U; D.h_valid = 1U; D.e_setv_dv = 2300U; run(MUI_SPLASH_MS + 3000U); MApp_Diag(&A, T, F);
    CHECK(strcmp(&F[2][15], " T230") == 0, "row 2 ends with the slave's set voltage: ' T230' ('%s')", F[2]);
    D.e_setv_dv = 2200U; run(300U); MApp_Diag(&A, T, F); CHECK(strcmp(&F[2][15], " !220") == 0, "a slave using 220 V while Set-Mode says 230 V: the T becomes ! ('%s')", F[2]);
    D.e_setv_dv = 0U; run(300U); MApp_Diag(&A, T, F); CHECK(strcmp(&F[2][15], " T---") == 0, "not known yet: ' T---' ('%s')", F[2]);
    D.e_valid = 0U; D.e_setv_dv = 2300U; run(300U); MApp_Diag(&A, T, F); CHECK(strcmp(&F[2][15], " T---") == 0, "no $E: ' T---' ('%s')", F[2]);
    printf("  hello request and set-voltage echo: asked every 2 s until $H arrives, never without $E; T230 on the diagnostics page, ! when the slave disagrees\n");
}


static void set_clock(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi, uint8_t ok) { D.clock.year = y; D.clock.month = mo; D.clock.day = d; D.clock.hour = h; D.clock.minute = mi; D.clock_ok = ok; }
static void t_clock_app(void)
{
    MLogRec_t r; MDateTime_t back, nt; int n;
    /* a fault is logged with the REAL-TIME CLOCK when it is set */
    fresh(); set_clock(2026U, 10U, 21U, 14U, 32U, 1U); to_run(); run(12000U);
    D.state = 3U; D.fault = 8U; run(300U);
    CHECK(MLog_Count(&A.log) == 1U && MLog_Get(&A.log, 0U, &r) == 0 && MC_IsStamp(r.run_min), "a trip with the clock set is logged with a clock stamp");
    MC_FromMinutes(MC_StampMinutes(r.run_min), &back);
    CHECK(back.year == 2026U && back.month == 10U && back.day == 21U && back.hour == 14U && back.minute == 32U, "and it reads 21/10/2026 14:32 (%u/%u/%u %u:%u)", back.day, back.month, back.year, back.hour, back.minute);
    /* the time of the trip is the time AT the trip: a later trip gets the later time */
    D.fault = 0U; D.state = 2U; set_clock(2026U, 10U, 21U, 15U, 5U, 1U); run(12000U); D.state = 3U; D.fault = 2U; run(300U);
    CHECK(MLog_Count(&A.log) >= 2U && MLog_Get(&A.log, 0U, &r) == 0 && MC_IsStamp(r.run_min), "a second trip is stamped too");
    MC_FromMinutes(MC_StampMinutes(r.run_min), &back); CHECK(back.hour == 15U && back.minute == 5U, "with ITS time, 15:05 (%u:%u)", back.hour, back.minute);
    /* the clock was never set (or the battery was lost): the log keeps using the running minutes, as before */
    fresh(); set_clock(2000U, 1U, 1U, 0U, 0U, 0U); to_run(); run(12000U); D.state = 3U; D.fault = 8U; run(300U);
    CHECK(MLog_Count(&A.log) == 1U && MLog_Get(&A.log, 0U, &r) == 0 && !MC_IsStamp(r.run_min), "a trip with the clock NOT set is logged with its running time, no stamp (%lu)", (unsigned long)r.run_min);
    /* Date and Time in Set-Mode start from the application's clock; typing a date and saving gives ONE write request to the driver */
    fresh(); set_clock(2026U, 10U, 21U, 14U, 32U, 1U); to_run(); run(500U);
    combo(); for (n = 0; n < 40 && A.ui.item != 23U; n++) { tap(0, 100U); run(1600U); }
    CROW(1, "Date"); CROW(2, "21/10/2026");
    tap(1, 100U); CROW(2, "31/10/2026"); tap(0, 100U); run(100U); CROW(1, "Time");
    combo(); run(300U);
    CHECK(MApp_TakeClockSet(&A, &nt) == 1U && nt.year == 2026U && nt.month == 10U && nt.day == 31U && nt.hour == 14U && nt.minute == 32U, "saving: the driver is asked to set 31/10/2026 14:32");
    CHECK(MApp_TakeClockSet(&A, &nt) == 0U, "and only once");
    /* passing through without typing: no request */
    fresh(); set_clock(2026U, 10U, 21U, 14U, 32U, 1U); to_run(); run(500U);
    combo(); for (n = 0; n < 40 && A.ui.item != 23U; n++) { tap(0, 100U); run(1600U); } tap(0, 100U); run(100U); tap(0, 100U); run(100U); combo(); run(300U);
    CHECK(MApp_TakeClockSet(&A, &nt) == 0U, "passing through Date and Time without typing: the clock is not touched");
    /* a clock that is not set: the entry starts from a placeholder, and saving the placeholder as typed is a normal write */
    fresh(); set_clock(2000U, 1U, 1U, 0U, 0U, 0U); to_run(); run(500U);
    combo(); for (n = 0; n < 40 && A.ui.item != 23U; n++) { tap(0, 100U); run(1600U); } CROW(2, "01/01/2026");
    printf("  clock in the application: faults stamped with the real-time clock (or the running time if it is not set), Date/Time start from the clock, one write request on save\n");
}


static void t_clock_page(void)
{
    char fr[MUI_ROWS][MUI_COLS + 1U];
    /* the Clock page: LOG, then MENU x4 */
    fresh(); set_clock(2026U, 10U, 21U, 14U, 32U, 1U); D.rtc_src = 1U; D.rtc_bdcr = 0x8303U; D.rtc_icsr = 0x00F0U; to_run(); run(500U);
    tap(3, 100U); tap(0, 100U); tap(0, 100U); tap(0, 100U); tap(0, 100U); run(200U);
    CHECK(MApp_ShowsClock(&A), "LOG, then MENU x4 opens the Clock page");
    MApp_ClockPage(&A, T, fr);
    CHECK(strncmp(fr[0], "Date 21/10/2026", 15) == 0, "row 0: the date ('%s')", fr[0]);
    CHECK(strncmp(fr[1], "Time 14:32", 10) == 0, "row 1: the time ('%s')", fr[1]);
    CHECK(strncmp(fr[2], "Src:Crystal Set:Y", 17) == 0, "row 2: the clock source and that it counts as set ('%s')", fr[2]);
    CHECK(strncmp(fr[3], "BD:8303 IC:00F0", 15) == 0, "row 3: the raw registers ('%s')", fr[3]);
    /* never set: the page says so, with the way to set it */
    fresh(); set_clock(2000U, 1U, 1U, 0U, 0U, 0U); D.rtc_src = 2U; D.rtc_bdcr = 0x8101U; D.rtc_icsr = 0x0000U; to_run(); run(500U);
    tap(3, 100U); tap(0, 100U); tap(0, 100U); tap(0, 100U); tap(0, 100U); run(200U); MApp_ClockPage(&A, T, fr);
    CHECK(strncmp(fr[0], "Clock NOT SET", 13) == 0 && strncmp(fr[2], "Src:LSI !!! Set:N", 17) == 0 && strncmp(fr[3], "BD:8101 IC:0000", 15) == 0, "not set, running from the LSI: 'Clock NOT SET', 'Src:LSI !!! Set:N' and the raw bits ('%s' / '%s' / '%s')", fr[0], fr[2], fr[3]);
    fresh(); set_clock(2000U, 1U, 1U, 0U, 0U, 0U); D.rtc_src = 0U; to_run(); run(500U); tap(3, 100U); tap(0, 100U); tap(0, 100U); tap(0, 100U); tap(0, 100U); run(200U); MApp_ClockPage(&A, T, fr);
    CHECK(strncmp(fr[2], "Src:------- Set:N", 17) == 0, "no clock source: 'Src:-------' ('%s')", fr[2]);
    tap(0, 100U); run(200U); CHECK(!MApp_ShowsClock(&A), "the next MENU leaves the Clock page");
    printf("  clock page: the date and time, the source (crystal / LSI / none), whether it counts as set, the raw registers\n");
}


static void t_heat_inrush_app(void)
{
    /* overheat through the application: the pin must stay low for the Overheat Trip Delay (10 s), then "Heat"; the countdown only runs once the pin is high again */
    fresh(); to_run(); run(12000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "running");
    D.heat = 1U; run(8000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "the pin low for 8 s: nothing yet");
    run(2500U); CHECK(MApp_Engine(&A)->contactor[0] == 0U && strncmp(row(3), "Heat", 4) == 0 && strstr(row(3), "Rst") == NULL, "after the 10 s delay: tripped, 'Heat', no countdown while the pin is still low (%s)", row(3));
    D.heat = 0U; run(1500U); CHECK(strncmp(row(3), "Heat", 4) == 0 && strstr(row(3), "Rst 0:0") != NULL, "the pin high: the Restart Time counts down on the screen (%s)", row(3));
    run(5000U); CHECK(MApp_Control(&A)->run == 1U, "and it restarts by itself when it reaches zero");
    /* inrush: the retry is 30 s away, counted on the screen; the first inrush of a commissioning build (0 retries) latches and shows its code */
    fresh(); to_run(); run(12000U); D.state = 3U; D.fault = 2U; run(300U);
    CHECK(strncmp(row(3), "Inrush", 6) == 0 && (strstr(row(3), "Rst 0:2") != NULL || strstr(row(3), "Rst 0:30") != NULL), "an inrush trip with retries allowed: 'Inrush ... Rst 0:30' counting down (%s)", row(3));
    fresh(); MApp_SetInrushRetries(&A, 0U); to_run(); run(12000U); D.state = 3U; D.fault = 2U; run(300U);
    ROW(3, "Inrush           F02");
    D.fault = 0U; D.state = 0U; run(120000U); ROW(3, "Inrush           F02");
    CHECK(MApp_Control(&A)->run == 0U, "commissioning (0 retries): latched, no countdown, no restart (the code stays on the screen)");
    printf("  heat and inrush in the application: the delay, the countdown, the restart, and the latch of a commissioning build\n");
}


static void vcal_open(uint8_t page_menus) { int q; tap(3, 100U); for (q = 0; q < (int)page_menus; q++) tap(0, 100U); run(200U); }
static void stop_unit(void) { tap(2, 100U); D.state = 0U; run(500U); }
static void t_vcal_app(void)
{
    MCalK_t k; int n; uint8_t id1;
    /* the unit is RUNNING: refused on the master, nothing is sent */
    fresh(); D.vin_dv = 2268U; D.vout_dv = 2275U; to_run(); run(12000U); vcal_open(5);
    CHECK(MUI_ShowsVCal(&A.ui) == 1U, "the I/P Calibration page is open");
    tap(3, 100U); run(100U);
    CHECK(MApp_TakeKCommand(&A, &k, T) == 0U && strstr(row(1), "Stop The Unit") != NULL, "unit running: 'Stop The Unit First', no command is sent ('%s')", row(1));
    /* stopped: LOG sends CALV with the typed value, then retries with the SAME id every 200 ms, three times in all */
    run(2000U); stop_unit(); CHECK(A.eout.run[0] == 0U, "STOP stopped the unit");
    tap(1, 100U); run(100U);                                                                         /* still on the I/P page: 226.8 -> 227.8 */
    tap(3, 100U); run(50U);
    CHECK(MApp_TakeKCommand(&A, &k, T) == 1U && k.cmd == MAPP_KCMD_CALV && k.num == 2278, "stopped: LOG sends CALV with 2278 (227.8 V) (cmd %u, num %d)", k.cmd, (int)k.num);
    id1 = k.id; CHECK(MApp_TakeKCommand(&A, &k, T) == 0U, "and not again at once");
    n = 1; { uint32_t g; uint8_t same = 1U; for (g = 0U; g < 2900U; g += 10U) { run(10U); if (MApp_TakeKCommand(&A, &k, T)) { n++; if (k.id != id1) same = 0U; } } CHECK(n == 3 && same, "it is sent 3 times in all, 200 ms apart, always with the same id (%d times)", n); }
    run(200U); CHECK(strstr(row(1), "No Answer") != NULL, "after 3 s without an answer: 'No Answer From The Slave' ('%s')", row(1));
    CHECK(MApp_TakeKCommand(&A, &k, T) == 0U, "and nothing more is sent");
    /* the answer */
    run(2000U); tap(3, 100U); run(50U); (void)MApp_TakeKCommand(&A, &k, T); id1 = k.id;                  /* still on the I/P page */
    MApp_OnKResult(&A, (uint8_t)(id1 + 1U), 0U, 10152); run(100U); CHECK(MApp_TakeKCommand(&A, &k, T) == 0U && strstr(row(1), "Calibrated") == NULL, "an answer with another id is ignored");
    MApp_OnKResult(&A, id1, 0U, 10152); run(100U);
    CHECK(strstr(row(1), "Calibrated") != NULL && strstr(row(2), "I/P +1.52 %") != NULL, "R,id,0,10152: 'Calibrated', 'I/P +1.52 %%' ('%s' / '%s')", row(1), row(2));
    run(2000U); tap(3, 100U); run(50U); (void)MApp_TakeKCommand(&A, &k, T); MApp_OnKResult(&A, k.id, 0U, 9920U); run(100U);
    CHECK(strstr(row(2), "I/P -0.80 %") != NULL, "a correction below nominal: 'I/P -0.80 %%' ('%s')", row(2));
    run(2000U); tap(3, 100U); run(50U); (void)MApp_TakeKCommand(&A, &k, T); MApp_OnKResult(&A, k.id, 0U, 11000U); run(100U);
    CHECK(strstr(row(2), "I/P +10.00 %") != NULL, "+10.00 %% fits ('%s')", row(2));
    /* every refusal has its own words */
    { static const struct { uint8_t st; const char *l1; } E[] = { {1U, "Bad Value"}, {2U, "No Steady Signal"}, {3U, "Out Of Range"}, {4U, "Stop The Unit"}, {5U, "Save Failed"}, {6U, "Not Supported"}, {7U, "Slave Busy"}, {99U, "Calibration"} };
      unsigned q; for (q = 0U; q < sizeof E / sizeof E[0]; q++) { run(2000U); tap(3, 100U); run(50U); (void)MApp_TakeKCommand(&A, &k, T); MApp_OnKResult(&A, k.id, E[q].st, 0); run(100U); CHECK(strstr(row(1), E[q].l1) != NULL, "status %u: '%s' ('%s')", E[q].st, E[q].l1, row(1)); } }
    /* a flash failure carries the reason: stage 2, reason 3 (HAL error), dual bank, flags 0x0080 */
    run(2000U); tap(3, 100U); run(50U); (void)MApp_TakeKCommand(&A, &k, T); MApp_OnKResult(&A, k.id, 5U, 23100128); run(100U);
    CHECK(strstr(row(1), "Save Failed") != NULL && strstr(row(2), "S2 R3 D1 E0080") != NULL, "FLASH_FAIL with a diagnostic: 'Save Failed' / 'S2 R3 D1 E0080' ('%s' / '%s')", row(1), row(2));
    run(2000U); tap(3, 100U); run(50U); (void)MApp_TakeKCommand(&A, &k, T); MApp_OnKResult(&A, k.id, 5U, 0); run(100U);
    CHECK(strstr(row(2), "Flash Error") != NULL, "FLASH_FAIL without one: 'Flash Error'");
    /* only one at a time */
    run(2000U); tap(3, 100U); run(1700U); tap(3, 100U); run(50U); CHECK(strstr(row(1), "Busy") != NULL, "a second LOG while one is out: 'Busy' ('%s')", row(1)); run(3500U);
    /* typing 000.0 resets the channel */
    run(500U); A.ui.cal_val = 0U; tap(3, 100U); run(50U);
    CHECK(MApp_TakeKCommand(&A, &k, T) == 1U && k.cmd == MAPP_KCMD_CALR && k.letter == 'V', "000.0 on the input page sends CALR V");
    MApp_OnKResult(&A, k.id, 0U, 10000); run(100U); CHECK(strstr(row(1), "Reset To Nominal") != NULL, "and the answer says 'Reset To Nominal' ('%s')", row(1));
    /* a value outside 50.0 to 400.0 V */
    run(2000U); A.ui.cal_val = 400U; tap(3, 100U); run(50U); CHECK(MApp_TakeKCommand(&A, &k, T) == 0U && strstr(row(1), "Enter 50.0-400.0 V") != NULL, "40.0 V: 'Enter 50.0-400.0 V', nothing sent ('%s')", row(1));
    run(2000U); A.ui.cal_val = 4001U; tap(3, 100U); run(50U); CHECK(MApp_TakeKCommand(&A, &k, T) == 0U, "400.1 V: nothing sent");
    /* the output page uses CALO and its own letter */
    run(2000U); tap(0, 100U); run(200U); CHECK(MUI_ShowsVCal(&A.ui) == 2U, "the O/P Calibration page"); A.ui.cal_val = 2250U; tap(3, 100U); run(50U);
    CHECK(MApp_TakeKCommand(&A, &k, T) == 1U && k.cmd == MAPP_KCMD_CALO && k.num == 2250, "LOG on the output page sends CALO 2250");
    MApp_OnKResult(&A, k.id, 0U, 9920U); run(100U); CHECK(strstr(row(2), "O/P -0.80 %") != NULL, "answered 'O/P -0.80 %%' ('%s')", row(2));
    run(2000U); A.ui.cal_val = 0U; tap(3, 100U); run(50U); CHECK(MApp_TakeKCommand(&A, &k, T) == 1U && k.cmd == MAPP_KCMD_CALR && k.letter == 'O', "000.0 on the output page sends CALR O");
    /* no slave link */
    run(4000U); D.have = 0U; D.link_ok = 0U; run(300U); A.ui.cal_val = 2250U; tap(3, 100U); run(50U); CHECK(MApp_TakeKCommand(&A, &k, T) == 0U, "no slave link: nothing is sent");
    /* one id counter for every $K: they never repeat in a row and wrap from 255 to 1 */
    fresh(); { uint8_t a1 = MApp_NextKId(&A), a2 = MApp_NextKId(&A); A.k_id = 255U; { uint8_t b1 = MApp_NextKId(&A), b2 = MApp_NextKId(&A); CHECK(a1 == 1U && a2 == 2U && b1 == 255U && b2 == 1U, "ids 1, 2 ... 255, then 1 again, never 0 (%u %u %u %u)", a1, a2, b1, b2); } }
    printf("  voltage calibration in the application: refused while running, CALV / CALO / CALR with the typed value, 3 sends with one id, the timeout, every answer in words, one at a time\n");
}


static void t_response_timer(void)
{
    char fr[MUI_ROWS][MUI_COLS + 1U]; uint32_t g;
    /* the unit runs at 230 V: nothing timed yet, the diagnostics row shows the frame count as before */
    fresh(); D.vin_dv = 2300U; D.vout_dv = 2300U; to_run(); run(12000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "running with the contactor closed");
    tap(3, 100U); tap(0, 100U); tap(0, 100U); run(100U); CHECK(MApp_ShowsDiag(&A), "the diagnostics page is open"); MApp_Diag(&A, T, fr);
    CHECK(strstr(fr[0], "M:") != NULL && strstr(fr[0], "s/") == NULL, "nothing timed yet: the frame count is shown ('%s')", fr[0]);
    /* a 40 V excursion that comes back after 1.5 s: "1.5s/40V" */
    D.vout_dv = 1900U; run(1500U); D.vout_dv = 2295U; run(300U); MApp_Diag(&A, T, fr);
    CHECK(strncmp(fr[0], "S:Y E:", 6) == 0 && (strstr(fr[0], " 1.5s/40V") != NULL || strstr(fr[0], " 1.6s/40V") != NULL || strstr(fr[0], " 1.4s/40V") != NULL), "a 40 V error that lasted 1.5 s: 'x.xs/40V' on row 0 ('%s')", fr[0]);
    /* an excursion under 5 V is not timed */
    D.vout_dv = 2260U; run(3000U); D.vout_dv = 2300U; run(300U); MApp_Diag(&A, T, fr); CHECK(strstr(fr[0], "/40V") != NULL, "a 4 V error does not start the clock: the last result is still 1.5 s / 40 V ('%s')", fr[0]);
    /* a longer one (inside the output limits, so the unit keeps running): 12 s shows whole seconds; the largest error counts, not the first */
    D.vout_dv = 2200U; run(2000U); D.vout_dv = 2150U; run(10000U); D.vout_dv = 2300U; run(300U); MApp_Diag(&A, T, fr);
    CHECK(strstr(fr[0], " 12s/15V") != NULL || strstr(fr[0], " 13s/15V") != NULL, "12 s with the largest error 15 V: '12s/15V' ('%s')", fr[0]);
    /* the output never comes back within 30 s: not a recovery time, the old result stays */
    D.vout_dv = 2150U; run(32000U); D.vout_dv = 2300U; run(300U); MApp_Diag(&A, T, fr); CHECK(strstr(fr[0], "s/15V") != NULL, "an excursion that lasts over 30 s is dropped, the last result stays ('%s')", fr[0]);
    /* the unit is stopped: the clock stops with it */
    D.vout_dv = 2150U; run(500U); stop_unit(); D.vout_dv = 2300U; run(2000U); MApp_Diag(&A, T, fr); CHECK(strstr(fr[0], "s/15V") != NULL, "a stopped unit is not timed ('%s')", fr[0]);
    (void)g;
    printf("  response timer: the time from a 5 V error back to within 1 V, and its largest error, on the diagnostics page\n");
}


static void t_watchdog_events(void)
{
    char fr[MUI_ROWS][MUI_COLS + 1U]; int r, seen = 0;
    /* a restart by the watchdog is logged once; an ordinary start is not */
    fresh(); MApp_BootReset(&A, 0U); CHECK(MLog_Count(&A.log) == 0U, "an ordinary start logs nothing");
    MApp_BootReset(&A, 1U); CHECK(MLog_Count(&A.log) == 1U, "a restart by the watchdog logs one entry (%u)", (unsigned)MLog_Count(&A.log));
    to_run(); run(1000U); tap(3, 100U); run(200U); for (r = 0; r < 4; r++) if (strstr(row(r), "Watchdog Reset") != NULL) seen = 1;
    CHECK(seen, "the log viewer names it 'Watchdog Reset' ('%s' / '%s' / '%s' / '%s')", row(0), row(1), row(2), row(3));
    /* the slave's last reset was its watchdog: a 'w' in the corner of the diagnostics page, not otherwise */
    fresh(); to_run(); run(1000U); D.slave_wdg = 0U; tap(3, 100U); tap(0, 100U); tap(0, 100U); run(100U); MApp_Diag(&A, T, fr);
    CHECK(fr[3][19] != 'w', "no marker when the slave's last reset was not its watchdog ('%s')", fr[3]);
    D.slave_wdg = 1U; run(100U); MApp_Diag(&A, T, fr); CHECK(fr[3][19] == 'w' && strncmp(fr[3], "Slv st:", 7) == 0, "a 'w' in column 20 of row 3 when it was ('%s')", fr[3]);
    printf("  watchdog events: a restart by the watchdog is logged once, the slave's watchdog reset shows as a 'w' on the diagnostics page\n");
}


static void t_run_minutes(void)
{
    MLogRec_t rec; uint32_t g;
    /* a power-up: the counter starts from what the glue loaded, and counts only while the contactor is closed */
    fresh(); MApp_SetRunMinutes(&A, 5000U); to_run(); run(12000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "running with the contactor closed");
    (void)MApp_TakeRunDirty(&A); { uint32_t m0 = MApp_RunMinutes(&A); run(125000U); CHECK(MApp_RunMinutes(&A) == m0 + 2U, "125 s with the contactor closed: 2 more minutes (%u -> %u)", (unsigned)m0, (unsigned)MApp_RunMinutes(&A)); }
    CHECK(MApp_TakeRunDirty(&A) == 1U && MApp_TakeRunDirty(&A) == 0U, "a whole minute makes the counter dirty, once, so the glue saves it");
    /* stopped: nothing counts */
    stop_unit(); run(500U); { uint32_t m1 = MApp_RunMinutes(&A); (void)MApp_TakeRunDirty(&A); run(180000U); CHECK(MApp_RunMinutes(&A) == m1 && MApp_TakeRunDirty(&A) == 0U, "3 minutes with the unit stopped: the counter stays (%u)", (unsigned)m1); }
    /* a stalled loop (one step of 20 s) adds at most one second */
    { uint32_t m2; fresh(); MApp_SetRunMinutes(&A, 100U); to_run(); run(12000U); m2 = MApp_RunMinutes(&A); (void)m2; }
    /* the fault log carries the running minutes as its time when no clock is set, and the real date and time when it is */
    fresh(); MApp_SetRunMinutes(&A, 7777U); to_run(); run(12000U); D.state = 3U; D.fault = 2U; run(500U);
    CHECK(MLog_Count(&A.log) >= 1U && MLog_Get(&A.log, 0, &rec) == 0 && rec.run_min >= 7777U && rec.run_min < 7780U && (rec.run_min & 0x80000000UL) == 0U, "a fault without a clock: the log time is the running minutes, not minutes since power-up (%u)", (unsigned)rec.run_min);
    fresh(); set_clock(2026U, 10U, 21U, 14U, 32U, 1U); MApp_SetRunMinutes(&A, 7777U); to_run(); run(12000U); D.state = 3U; D.fault = 2U; run(500U);
    CHECK(MLog_Get(&A.log, 0, &rec) == 0 && (rec.run_min & 0x80000000UL) != 0U, "with the clock set the log time is the real date and time (top bit set)");
    /* a watchdog restart is logged with the running minutes */
    fresh(); MApp_SetRunMinutes(&A, 4242U); MApp_BootReset(&A, 1U); CHECK(MLog_Get(&A.log, 0, &rec) == 0 && rec.type == MLOG_T_WDG_RESET && rec.run_min == 4242U, "the watchdog-reset entry carries the running minutes (%u)", (unsigned)rec.run_min);
    /* the top bit of a loaded value is never kept: it would look like a date */
    fresh(); MApp_SetRunMinutes(&A, 0xFFFFFFFFUL); CHECK(MApp_RunMinutes(&A) == 0x7FFFFFFFUL, "a loaded value is cut to 31 bits");
    (void)g;
    printf("  running hours: minutes of output ON counted from the loaded value, dirty once a minute, nothing while stopped, the log time uses them when no clock is set\n");
}


/* the CT reading (x16) that the default calibration turns into this many tenths of an amp */
static uint16_t x16_for_da(uint32_t da) { return (uint16_t)(MAPP_CT_ZERO_DEFAULT_X16 + (da * 16000000ULL + (MAPP_CT_DA_PER_COUNT_X1000 * (uint64_t)MAPP_CT_GAIN_DEFAULT) / 2U) / (MAPP_CT_DA_PER_COUNT_X1000 * (uint64_t)MAPP_CT_GAIN_DEFAULT) + 1U); }
static void t_ct_peak(void)
{
    MLogRec_t rec; uint32_t g;
    /* Overload Current 20 A (above the 12.3 A the test load shows, so Overload itself stays quiet): the peak limit is 3 x 20 = 60 A (600 tenths) */
    fresh(); A.settings.ovl_current = 20U; to_run(); run(12000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "running with the contactor closed");
    D.ct_max_x16 = x16_for_da(550U); run(500U); CHECK(MApp_Engine(&A)->contactor[0] == 1U && MLog_Count(&A.log) == 0U, "55 A (under 3 x 20 A): nothing happens");
    D.ct_max_x16 = x16_for_da(700U); run(10U); D.ct_max_x16 = x16_for_da(550U); run(500U);
    CHECK(MApp_Engine(&A)->contactor[0] == 1U && MLog_Count(&A.log) == 0U, "70 A for 10 ms only: not enough (20 ms in a row are needed)");
    D.ct_max_x16 = x16_for_da(700U); run(40U);
    CHECK(MApp_Engine(&A)->contactor[0] == 0U && MLog_Count(&A.log) >= 1U && MLog_Get(&A.log, 0, &rec) == 0 && rec.type == MLOG_T_INRUSH, "70 A for 40 ms (the limit is 60 A): the contactor opens and an Inrush is logged");
    D.ct_max_x16 = 0U; run(500U);
    /* the same reading with the default 99 A overload setting: 70 A is nothing */
    fresh(); to_run(); run(12000U); D.ct_max_x16 = x16_for_da(700U); run(2000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U && MLog_Count(&A.log) == 0U, "Overload Current 99 A: 70 A is no peak");
    /* the full scale of the ADC (4000 counts and more) is a peak whatever the setting is */
    D.ct_max_x16 = (uint16_t)(MAPP_CT_FULL_X16 - 16U); run(500U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "just under the full scale (3999 counts): no trip");
    D.ct_max_x16 = (uint16_t)MAPP_CT_FULL_X16; run(60U); CHECK(MApp_Engine(&A)->contactor[0] == 0U && MLog_Get(&A.log, 0, &rec) == 0 && rec.type == MLOG_T_INRUSH, "the ADC at its full scale: the contactor opens, an Inrush is logged");
    /* not sampled (the glue gives 0): the backup is idle even when the averaged CT reading is high (3 s is shorter than the Overload delay) */
    fresh(); A.settings.ovl_current = 20U; to_run(); run(12000U); D.ct_max_x16 = 0U; D.ct_x16 = x16_for_da(900U); D.ct_raw = 1U; run(3000U);
    CHECK(MApp_Engine(&A)->contactor[0] == 1U && MLog_Count(&A.log) == 0U, "no fast samples (0): the backup does nothing");
    /* the condition has to be continuous: a reading that dips below the limit every other 10 ms never trips */
    fresh(); A.settings.ovl_current = 20U; to_run(); run(12000U); for (g = 0U; g < 100U; g++) { D.ct_max_x16 = x16_for_da(700U); run(10U); D.ct_max_x16 = x16_for_da(100U); run(10U); }
    CHECK(MApp_Engine(&A)->contactor[0] == 1U && MLog_Count(&A.log) == 0U, "70 A and 10 A on alternate 10 ms for 2 s: it never trips");
    printf("  CT peak backup: above 3 x the Overload Current or at the ADC full scale for 20 ms in a row is an Inrush; a shorter excursion, alternating readings, no samples and smaller readings are not\n");
}


static int shown_vout(void) { const char *r = row(0); int v = 0, seen = 0; while (*r) { if (*r >= '0' && *r <= '9') { v = v * 10 + (*r - '0'); seen = 1; } else if (seen) break; r++; } return seen ? v : -1; }
static void t_display_speed(void)
{
    uint32_t k, t_last = 0U, min_gap = 100000U, changes = 0U; int prev, now_v;
    /* a fast drift of the output, 5 V/s for 3 s (230 -> 215 V, each change below the 5 V that counts as a real step): the hysteresis alone would change the number every 0.14 s; the hold makes it at most every 300 ms */
    fresh(); D.vin_dv = 2300U; D.vout_dv = 2300U; to_run(); run(12000U); prev = shown_vout(); CHECK(prev == 230, "steady 230 V shows 230 (%d)", prev);
    for (k = 0U; k < 300U; k++)
    {
        D.vout_dv = (uint16_t)(2300U - k / 2U);                                  /* half a tenth per 10 ms = 5 V per second */
        run(10U); now_v = shown_vout();
        if (now_v != prev) { uint32_t tt = k * 10U; if (changes > 0U && tt - t_last < min_gap) min_gap = tt - t_last; t_last = tt; changes++; prev = now_v; }
    }
    CHECK(changes >= 4U && changes <= 11U, "a 15 V drift over 3 s: the number changes %u times (a few, not on every 0.7 V)", (unsigned)changes);
    CHECK(min_gap >= 290U, "and never twice within 300 ms (the shortest gap was %u ms)", (unsigned)min_gap);
    run(5000U); CHECK(shown_vout() == 215 || shown_vout() == 216, "it ends on the right value 5 s later (%d)", shown_vout());
    /* REAL steps (8 V each, 150 ms apart) are shown at once, even inside the 300 ms hold */
    D.vout_dv = 2230U; run(40U); CHECK(shown_vout() == 223, "an 8 V step up (215 -> 223 V) is shown within 40 ms (%d)", shown_vout());
    run(110U); D.vout_dv = 2310U; run(40U); CHECK(shown_vout() == 231, "the next 8 V step 150 ms later (-> 231 V) is shown within 40 ms as well: a real step is never held (%d)", shown_vout());
    /* the filter is slower than before. NOTE: this test calls the application every 5 ms, the real firmware every 10 ms, so here the filter runs twice as fast (time constant 0.48 s; 0.96 s in the unit, was 0.64 s).
     * After a 3 V step the number is still on its way at 0.9 s (the old divisor 64 would be there by 0.74 s) and has arrived by 4 s. */
    fresh(); D.vin_dv = 2300U; D.vout_dv = 2300U; to_run(); run(12000U); D.vout_dv = 2270U; run(900U); { int s09 = shown_vout(); run(3100U); CHECK(s09 >= 228 && s09 <= 229 && shown_vout() == 227, "a 3 V step: at 0.9 s %d (still on its way), at 4 s %d", s09, shown_vout()); }
    printf("  display speed: the shown voltage changes at most every 300 ms, a real step (5 V or more) at once, a small step is smoothed over about two seconds\n");
}


static void t_iout_page(void)
{
    char fr[MUI_ROWS][MUI_COLS + 1U]; int q;
    /* the page: LOG, then MENU x7 */
    fresh(); to_run(); run(1000U); tap(3, 100U); for (q = 0; q < 7; q++) tap(0, 100U); run(100U);
    CHECK(MApp_ShowsIout(&A), "LOG, then MENU x7 opens the Converter Current page");
    /* the slave does not measure it (an older slave, or SLAVE_IOUT 0) */
    D.iout_valid = 0U; D.iout_rms_ca = 0U; D.iout_pk_ca = 0U; run(100U); MApp_IoutPage(&A, T, fr);
    CHECK(strstr(fr[0], "Converter Current") != NULL && strstr(fr[1], "--.--") != NULL && strstr(fr[2], "--.--") != NULL && strstr(fr[3], "Not measured") != NULL, "not measured: dashes and 'Not measured' ('%s' / '%s' / '%s' / '%s')", fr[0], fr[1], fr[2], fr[3]);
    /* the full load of the original board: 5.89 A RMS, 8.31 A peak */
    D.iout_valid = 1U; D.iout_rms_ca = 589U; D.iout_pk_ca = 831U; D.iout_clip = 0U; run(100U); MApp_IoutPage(&A, T, fr);
    CHECK(strncmp(fr[1], "RMS    5.89 A", 13) == 0 && strncmp(fr[2], "Peak   8.31 A", 13) == 0 && strstr(fr[3], "CT primary current") != NULL, "5.89 A and 8.31 A: '%s' / '%s' / '%s'", fr[1], fr[2], fr[3]);
    D.iout_rms_ca = 7U; D.iout_pk_ca = 12U; run(100U); MApp_IoutPage(&A, T, fr); CHECK(strncmp(fr[1], "RMS    0.07 A", 13) == 0 && strncmp(fr[2], "Peak   0.12 A", 13) == 0, "0.07 A and 0.12 A keep their zeros ('%s' / '%s')", fr[1], fr[2]);
    D.iout_rms_ca = 0U; D.iout_pk_ca = 0U; run(100U); MApp_IoutPage(&A, T, fr); CHECK(strncmp(fr[1], "RMS    0.00 A", 13) == 0 && strstr(fr[3], "Not measured") == NULL, "a measured zero is shown as 0.00 A, not as 'not measured'");
    D.iout_rms_ca = 1234U; D.iout_pk_ca = 3999U; run(100U); MApp_IoutPage(&A, T, fr); CHECK(strncmp(fr[1], "RMS   12.34 A", 13) == 0 && strncmp(fr[2], "Peak  39.99 A", 13) == 0, "12.34 A and 39.99 A fit the line ('%s' / '%s')", fr[1], fr[2]);
    /* the amplifier is at a rail: the page says so */
    D.iout_rms_ca = 700U; D.iout_pk_ca = 1011U; D.iout_clip = 1U; run(100U); MApp_IoutPage(&A, T, fr); CHECK(strstr(fr[3], "OVER RANGE") != NULL, "at the limit of the range: 'OVER RANGE' ('%s')", fr[3]);
    /* the next MENU leaves the page */
    tap(0, 100U); run(100U); CHECK(!MApp_ShowsIout(&A), "the next MENU leaves the page");
    printf("  converter current page: not measured, the values with their zeros, a measured zero, over range\n");
}


static void send_t(int step, int result, int ticks, int vin_dv, int dv) { D.tune_step = (uint8_t)step; D.tune_result = (uint8_t)result; D.tune_ticks = (uint16_t)ticks; D.tune_vin_dv = (uint16_t)vin_dv; D.tune_dv_dv = (int16_t)dv; D.tune_n++; }
static void tune_screen(char fr[MUI_ROWS][MUI_COLS + 1U]) { MApp_TuneScreen(&A, T, fr); }
static int fr_has(char fr[MUI_ROWS][MUI_COLS + 1U], const char *s) { int r; for (r = 0; r < 4; r++) if (strstr(fr[r], s) != NULL) return 1; return 0; }
static void ask_tune(MCalK_t *k) { A.ui.req |= MUI_REQ_RETUNE; run(30U); (void)MApp_TakeKCommand(&A, k, T); }
static void t_tune_app(void)
{
    MCalK_t k, k2; char fr[MUI_ROWS][MUI_COLS + 1U]; int q;
    /* the unit is running: refused, nothing is sent, no screen */
    fresh(); to_run(); run(12000U); A.ui.req |= MUI_REQ_RETUNE; run(60U);
    CHECK(MApp_TakeKCommand(&A, &k, T) == 0U && !MApp_ShowsTune(&A) && strstr(row(1), "Stop The Unit") != NULL, "the unit is running: 'Stop The Unit First', nothing sent ('%s')", row(1));
    /* the slave has latched "Re-tune Required" (fault 18 or 19): Re-tune is allowed; any other fault still blocks it */
    { int fc; for (fc = 0; fc < 3; fc++) { static const uint8_t RTF[3] = { 18U, 19U, 6U }; fresh(); to_run(); run(2000U); stop_unit(); D.state = 3U; D.fault = RTF[fc]; run(100U); A.ui.req |= MUI_REQ_RETUNE; run(60U);
        if (fc < 2) CHECK(MApp_TakeKCommand(&A, &k, T) == 1U && k.cmd == MAPP_KCMD_TUNE && MApp_ShowsTune(&A), "slave fault %u (re-tune required): Re-tune is accepted and TUNE is sent", (unsigned)RTF[fc]);
        else        CHECK(MApp_TakeKCommand(&A, &k, T) == 0U && !MApp_ShowsTune(&A), "slave fault %u (not a re-tune fault): refused, nothing sent", (unsigned)RTF[fc]); } }
    /* no link */
    fresh(); to_run(); run(2000U); stop_unit(); D.have = 0U; A.ui.req |= MUI_REQ_RETUNE; run(60U); CHECK(!MApp_ShowsTune(&A) && MApp_TakeKCommand(&A, &k, T) == 0U, "no slave link: refused");
    /* a whole sweep */
    fresh(); to_run(); run(12000U); stop_unit(); D.tune_n = 5U; D.tune_result = 1U;                              /* an older sweep's frame is already counted */
    ask_tune(&k); CHECK(k.cmd == MAPP_KCMD_TUNE && MApp_ShowsTune(&A), "Re-tune: a TUNE command is due and the tuning screen is up");
    tune_screen(fr); CHECK(fr_has(fr, "Transformer Tuning") && fr_has(fr, "Starting...") && fr_has(fr, "STOP = abort"), "'Starting...' while it waits for the answer");
    { int n = 1; uint32_t g; uint8_t same = 1U; for (g = 0U; g < 1500U; g += 10U) { run(10U); if (MApp_TakeKCommand(&A, &k2, T)) { n++; if (k2.id != k.id || k2.cmd != k.cmd) same = 0U; } } CHECK(n == 3 && same, "TUNE is sent 3 times with the same id (%d)", n); }
    MApp_OnKResult(&A, k.id, 0U, 0); run(50U); tune_screen(fr); CHECK(fr_has(fr, "Measuring baseline"), "R status 0: 'Measuring baseline'");
    run(2000U); tune_screen(fr); CHECK(fr_has(fr, "Measuring baseline"), "the stale frame counted before the request is NOT taken as progress");
    /* what the slave is doing is shown while the sweep has not started: a start-up that does not progress says why */
    D.state = 0U; run(30U); tune_screen(fr); CHECK(fr_has(fr, "Slave Not Started"), "the slave is still OFF: 'Slave Not Started' ('%s')", fr[2]);
    D.state = 1U; D.wait_valid = 1U; D.wait_ctx = 0x01U; run(30U); tune_screen(fr); CHECK(fr_has(fr, "Slave Waits: Z"), "waiting for the zero-cross: 'Slave Waits: Z' ('%s')", fr[2]);
    D.wait_ctx = 0x03U; run(30U); tune_screen(fr); CHECK(fr_has(fr, "Slave Waits: ZV"), "waiting for the zero-cross and the input window: 'Slave Waits: ZV' ('%s')", fr[2]);
    D.wait_ctx = 0x7FU; run(30U); tune_screen(fr); CHECK(fr_has(fr, "Slave Waits: ZVMFBOR") && strlen(fr[2]) == 20U, "every reason at once still fits the 20 columns ('%s')", fr[2]);
    D.wait_valid = 0U; D.wait_ctx = 0U; run(30U); tune_screen(fr); CHECK(fr_has(fr, "Slave Starting..."), "starting, nothing to wait for: 'Slave Starting...' ('%s')", fr[2]);
    D.state = 3U; D.fault = 6U; run(30U); tune_screen(fr); CHECK(fr_has(fr, "Slave Fault F06"), "a fault in the slave: 'Slave Fault F06' ('%s')", fr[2]);
    D.state = 2U; D.fault = 0U; run(30U); tune_screen(fr); CHECK(fr_has(fr, "Slave In RUN"), "in RUN, measuring the baseline: 'Slave In RUN' ('%s')", fr[2]);
    D.state = 0U;
    send_t(1, 0, 16, 2300, 3); run(30U); tune_screen(fr); CHECK(fr_has(fr, "Step  1 of 10") && fr_has(fr, "Injected +0.3 V"), "step 1: 'Step  1 of 10', 'Injected +0.3 V' ('%s' / '%s')", fr[1], fr[2]);
    send_t(6, 0, 512, 2300, 124); run(30U); tune_screen(fr); CHECK(fr_has(fr, "Step  6 of 10") && fr_has(fr, "Injected +12.4 V"), "step 6: +12.4 V");
    send_t(8, 0, 1600, 2300, 485); run(30U); tune_screen(fr); CHECK(fr_has(fr, "+48.5 V"), "a two-digit injection: +48.5 V ('%s')", fr[2]);
    /* the slave's heartbeat (step 0) arrives between the steps: the screen must stay on the step, and the timeout must stay away */
    send_t(0, 0, 0, 2300, 0); run(30U); tune_screen(fr); CHECK(fr_has(fr, "Step  8 of 10") && fr_has(fr, "+48.5 V"), "a heartbeat (step 0) does NOT send the screen back to the baseline ('%s' / '%s')", fr[1], fr[2]);
    { int alive = 1; for (q = 0; q < 12; q++) { run(2000U); send_t(0, 0, 0, 2300, 0); if (A.tune_state != 2U) alive = 0; } tune_screen(fr); CHECK(alive && fr_has(fr, "Step  8 of 10"), "24 s with only heartbeats every 2 s (a long wait for a quiet input): still running, still on step 8"); }
    /* the unit stays off for the whole sweep, in Auto mode, and the keys do nothing */
    { int ok = 1; for (q = 0; q < 30; q++) { D.state = 0U; run(1000U); send_t(6, 0, 512, 2300, 124); if (A.eout.run[0] != 0U || A.eout.contactor[0] != 0U) ok = 0; } CHECK(ok, "30 s of a sweep in Auto mode: the engine never starts the unit"); }
    { uint8_t scr0 = A.ui.screen; tap(0, 100U); tap(3, 100U); CHECK(A.ui.screen == scr0, "one LOG press while tuning does not open the log viewer"); tap(1, 100U); tap(0, 100U);
      CHECK(MApp_ShowsTune(&A) && !MApp_ShowsDiag(&A) && !MApp_ShowsMonitor(&A) && A.eout.run[0] == 0U && A.ui.screen == scr0 && A.ui.page == 0U, "MENU, LOG and START do nothing while tuning: the UI is exactly where it was (screen %u, page %u)", (unsigned)A.ui.screen, (unsigned)A.ui.page); }
    send_t(10, 1, 2752, 2300, 856); run(30U); tune_screen(fr); CHECK(MApp_HelloDue(&A, T) == 1U && MApp_HelloDue(&A, T) == 0U, "a finished tune makes the master ask for the slave's hello ONCE (the profile numbers travel in it)"); CHECK(fr_has(fr, "Tuning Complete") && fr_has(fr, "Profile Stored"), "the final FINISHED frame: 'Tuning Complete' ('%s')", fr[0]);
    send_t(10, 1, 2752, 2300, 856); run(30U); tune_screen(fr); CHECK(fr_has(fr, "Tuning Complete"), "the repeats of the final frame change nothing");
    run(7000U); CHECK(!MApp_ShowsTune(&A), "the result goes after 6 s");
    run(9000U); CHECK(A.eout.run[0] == 1U || A.eout.contactor[0] == 1U || (A.eng.u[0].st != 0U), "afterwards the engine may start the unit again");
    /* a key dismisses the result at once */
    fresh(); to_run(); run(2000U); stop_unit(); ask_tune(&k); MApp_OnKResult(&A, k.id, 0U, 0); send_t(2, 1, 100, 2300, 100); run(30U); CHECK(MApp_ShowsTune(&A), "a result is shown"); tap(1, 100U); CHECK(!MApp_ShowsTune(&A), "any key dismisses it");
    /* every refusal in words */
    { static const struct { uint8_t st; int32_t v; const char *l1, *l2; } E[] = { {7U, 1, "Tune Refused", "No Tune Mode Yet"}, {7U, 2, "Tune Refused", "Set Max Correction"}, {7U, 3, "Tune Refused", "Slave Busy"}, {4U, 0, "Stop The Unit", "First"}, {6U, 0, "Not Supported", "Update The Slave"}, {5U, 0, "Tune Refused", "Try Again"}, {7U, 5, "Tune Refused", "Slave Cannot Start"}, {7U, 6, "Input Too High", "Lower The Mains"} };
      unsigned e; for (e = 0U; e < sizeof E / sizeof E[0]; e++) { fresh(); to_run(); run(2000U); stop_unit(); ask_tune(&k); MApp_OnKResult(&A, k.id, E[e].st, E[e].v); run(80U); CHECK(!MApp_ShowsTune(&A) && strstr(row(1), E[e].l1) != NULL && strstr(row(2), E[e].l2) != NULL, "status %u reason %d: '%s' / '%s' ('%s' / '%s')", E[e].st, (int)E[e].v, E[e].l1, E[e].l2, row(1), row(2)); } }
    /* no answer at all */
    fresh(); to_run(); run(2000U); stop_unit(); ask_tune(&k); run(3200U); CHECK(!MApp_ShowsTune(&A) && strstr(row(1), "No Answer") != NULL, "no answer in 3 s: 'No Answer' ('%s')", row(1));
    /* every way a sweep can end */
    { static const struct { int r; const char *l1; } FAILS[] = { {2, "Reversed Transformer"}, {3, "No Response"}, {4, "Aborted By Slave"}, {5, "Input Not Steady"}, {6, "By Operator"} };
      unsigned e; for (e = 0U; e < sizeof FAILS / sizeof FAILS[0]; e++) { fresh(); to_run(); run(2000U); stop_unit(); ask_tune(&k); MApp_OnKResult(&A, k.id, 0U, 0); send_t(4, FAILS[e].r, 128, 2300, (FAILS[e].r == 4) ? 0 : -30); run(40U); tune_screen(fr); CHECK(fr_has(fr, FAILS[e].l1), "result %d: '%s' ('%s' / '%s')", FAILS[e].r, FAILS[e].l1, fr[0], fr[1]); } }
    /* why an ABORTED sweep stopped: the slave's final frame carries it */
    { static const struct { int dv; const char *l1, *l2; } AB[] = { {52, "Current Too High", "Converter 5.2 A"}, {7, "Current Too High", "Converter 0.7 A"}, {152, "Current Too High", "Converter 15.2 A"}, {999, "Current Too High", "Converter at limit"}, {-1, "No Current Reading", "Needs SLAVE_IOUT 1"}, {0, "Aborted By Slave", "Fault Or Flash Error"} };
      unsigned e; for (e = 0U; e < sizeof AB / sizeof AB[0]; e++) { fresh(); to_run(); run(2000U); stop_unit(); ask_tune(&k); MApp_OnKResult(&A, k.id, 0U, 0); send_t(7, 4, 1024, 2060, AB[e].dv); run(40U); tune_screen(fr);
        CHECK(fr_has(fr, AB[e].l1) && fr_has(fr, AB[e].l2) && strlen(fr[2]) <= 20U, "Aborted with a detail of %d: '%s' / '%s' ('%s' / '%s')", AB[e].dv, AB[e].l1, AB[e].l2, fr[1], fr[2]); } }
    /* the reason an Input Not Steady sweep gave up, from the final frame's injection field */
    { static const struct { int dv; const char *txt; } UNS[] = { {123, "Input moved 12 V"}, {85, "Input moved 8.5 V"}, {-13, "Reading spread 1.3 V"}, {-120, "Reading spread 12 V"}, {0, "Input out of window"} };
      unsigned e; for (e = 0U; e < sizeof UNS / sizeof UNS[0]; e++) { fresh(); to_run(); run(2000U); stop_unit(); ask_tune(&k); MApp_OnKResult(&A, k.id, 0U, 0); send_t(6, 5, 512, 2340, UNS[e].dv); run(40U); tune_screen(fr);
        CHECK(fr_has(fr, "Input Not Steady") && fr_has(fr, UNS[e].txt) && strlen(fr[2]) <= 20U, "Input Not Steady with a detail of %d tenths: '%s' ('%s' / '%s')", UNS[e].dv, UNS[e].txt, fr[1], fr[2]); } }
    /* STOP aborts: TABORT with a new id, repeated; the final frame (6) ends it */
    fresh(); to_run(); run(2000U); stop_unit(); ask_tune(&k); MApp_OnKResult(&A, k.id, 0U, 0); send_t(2, 0, 32, 2300, 4); run(30U);
    tap(2, 100U); run(30U); CHECK(MApp_TakeKCommand(&A, &k2, T) == 1U && k2.cmd == MAPP_KCMD_TABORT && k2.id != k.id, "STOP sends TABORT with a new id");
    send_t(3, 6, 64, 2300, 8); run(30U); tune_screen(fr); CHECK(fr_has(fr, "Tuning Stopped") && fr_has(fr, "Nothing Stored"), "the slave's result 6: 'Tuning Stopped, Nothing Stored'");
    fresh(); to_run(); run(2000U); stop_unit(); ask_tune(&k); MApp_OnKResult(&A, k.id, 0U, 0); tap(2, 100U); run(5200U); tune_screen(fr); CHECK(fr_has(fr, "Tuning Stopped"), "an abort the slave never confirms is shown as stopped after 5 s");
    /* the frames stop coming */
    fresh(); to_run(); run(2000U); stop_unit(); ask_tune(&k); MApp_OnKResult(&A, k.id, 0U, 0); send_t(3, 0, 64, 2300, 8); run(30U); run(8200U); tune_screen(fr);
    CHECK(fr_has(fr, "Slave Not Answering") && MApp_TakeKCommand(&A, &k2, T) == 1U && k2.cmd == MAPP_KCMD_TABORT, "no $T frame for 8 s: the master aborts and says 'Slave Not Answering'");
    /* the Set-Mode idle timeout does not run out in the middle of a sweep */
    fresh(); to_run(); run(2000U); stop_unit(); combo(); run(500U); CHECK(MApp_SetModeOpen(&A), "Set-Mode is open");
    ask_tune(&k); MApp_OnKResult(&A, k.id, 0U, 0); for (q = 0; q < 40; q++) { run(1000U); send_t(3, 0, 64, 2300, 8); } CHECK(MApp_SetModeOpen(&A) && MApp_ShowsTune(&A), "40 s of tuning with no key: Set-Mode is still open behind the screen");
    send_t(10, 1, 2752, 2300, 856); run(100U); tap(1, 100U); run(100U); CHECK(MApp_SetModeOpen(&A) && !MApp_ShowsTune(&A), "afterwards the operator is back in Set-Mode");
    printf("  autotune, master side: the request, the answers in words, the progress screens, the result screens, STOP, timeouts, stale frames, the unit held off, Set-Mode kept open\n");
}


/* the Re-tune ITEM reached with the REAL keys after the REAL start-up order (MApp_Init, then MApp_InitCal): v0.9.33 to v0.9.35 lost the flag in exactly this order and every test raised the request directly */
static void t_retune_item_keys(void)
{
    int q, found = 0; MCalK_t k; int still_cal = 0;
    fresh_cal(); to_run(); run(2000U); stop_unit(); run(500U);
    combo(); run(500U); CHECK(MApp_SetModeOpen(&A), "Set-Mode is open");
    for (q = 0; q < 40 && !found; q++) { if (strstr(row(0), "Re-tune") != NULL || strstr(row(1), "Re-tune") != NULL || strstr(row(2), "Re-tune") != NULL) found = 1; else tap(0, 100U); }
    CHECK(found, "MENU reaches the Re-tune item (%d presses; rows '%s' / '%s' / '%s')", q, row(0), row(1), row(2));
    tap(1, 100U); CHECK(strstr(row(2), "Yes") != NULL || strstr(row(1), "Yes") != NULL || strstr(row(3), "Yes") != NULL, "START makes the value Yes ('%s' / '%s' / '%s')", row(1), row(2), row(3));
    tap(0, 100U); run(100U);
    CHECK(strstr(row(1), "Not Available") == NULL && strstr(row(2), "Not Available") == NULL, "MENU on Re-tune = Yes does NOT say 'Not Available' ('%s' / '%s')", row(1), row(2));
    CHECK(MApp_ShowsTune(&A) && MApp_TakeKCommand(&A, &k, T) == 1U && k.cmd == MAPP_KCMD_TUNE, "it starts a sweep: the tuning screen is up and a TUNE command is due");
    /* Calibration (item 2) must still be available in the same start-up order */
    fresh_cal(); to_run(); run(2000U); stop_unit(); run(500U); combo(); run(500U);
    for (q = 0; q < 6 && !still_cal; q++) { if (strstr(row(0), "Calibration") != NULL || strstr(row(1), "Calibration") != NULL || strstr(row(2), "Calibration") != NULL) still_cal = 1; else tap(0, 100U); }
    tap(1, 100U); tap(0, 100U); run(100U);
    CHECK(still_cal && strstr(row(1), "Not Available") == NULL && strstr(row(2), "Not Available") == NULL, "Calibration is still available too ('%s' / '%s')", row(1), row(2));
    printf("  Re-tune item with the real keys after the real start-up order: reachable, Yes + MENU starts a sweep, no 'Not Available', Calibration unchanged\n");
}


/* the Transformer Profile page (LOG, then MENU x8): the numbers of the slave's stored profile */
static void t_profile_page(void)
{
    char fr[MUI_ROWS][MUI_COLS + 1U]; int q;
    fresh(); to_run(); run(1000U); tap(3, 100U); for (q = 0; q < 8; q++) tap(0, 100U); run(100U);
    CHECK(MApp_ShowsProfile(&A) && !MApp_ShowsIout(&A), "LOG, then MENU x8 opens the Transformer Profile page");
    D.prof_tuned = 0U; D.prof_cap = 0U; run(100U); MApp_ProfilePage(&A, T, fr);
    CHECK(strstr(fr[0], "Transformer Profile") != NULL && strstr(fr[1], "No profile stored") != NULL && strstr(fr[3], "Re-tune") != NULL, "no profile: 'No profile stored' ('%s' / '%s')", fr[1], fr[3]);
    D.prof_tuned = 1U; D.prof_cap = 2752U; D.prof_kib = 523U; D.prof_kiu = 1000U; D.prof_vref_dv = 1600U; D.prof_maxc_dv = 1100U; D.prof_n = 10U; D.prof_knee = 0U; run(100U); MApp_ProfilePage(&A, T, fr);
    CHECK(strncmp(fr[1], "Cap 2752 ticks", 14) == 0 && fr[1][14] == ' ' && strncmp(fr[2], "Gain B0.523 U1.000", 18) == 0 && strncmp(fr[3], "At 160V Max 110V n10", 20) == 0, "a profile: '%s' / '%s' / '%s'", fr[1], fr[2], fr[3]);
    D.prof_knee = 1U; D.prof_cap = 1024U; D.prof_kib = 1198U; D.prof_kiu = 80U; D.prof_vref_dv = 2304U; D.prof_maxc_dv = 600U; D.prof_n = 7U; run(100U); MApp_ProfilePage(&A, T, fr);
    CHECK(strncmp(fr[1], "Cap 1024 ticks Knee", 19) == 0 && strncmp(fr[2], "Gain B1.198 U0.080", 18) == 0 && strncmp(fr[3], "At 230V Max 060V n07", 20) == 0, "a knee, small scales, 7 points: '%s' / '%s' / '%s'", fr[1], fr[2], fr[3]);
    D.prof_cap = 9999U; D.prof_kib = 65535U; D.prof_kiu = 65535U; D.prof_vref_dv = 65535U; D.prof_maxc_dv = 65535U; D.prof_n = 15U; D.prof_knee = 0U; run(100U); MApp_ProfilePage(&A, T, fr);
    { int w = 1; for (q = 0; q < 4; q++) if (strlen(fr[q]) != 20U) w = 0; CHECK(w, "extreme values never overflow the 20 columns"); }
    tap(0, 100U); run(100U); CHECK(!MApp_ShowsProfile(&A) && A.ui.page == 9U, "MENU after the profile page goes to the Slave Load Cal page (page %u)", (unsigned)A.ui.page);
    tap(0, 100U); run(100U); CHECK(!MApp_ShowsProfile(&A) && !MApp_ShowsIout(&A) && A.ui.page == 0U, "and MENU on the last page goes back to the Run screen (page %u)", (unsigned)A.ui.page);
    printf("  transformer profile page: the numbers, no profile, a knee, extreme values\n");
}


/* ---- v0.9.45: the load current from the slave's converter current: the conversion, the source choice with its fallbacks, the overload trip, the Slave Load Cal page (zero, gain, errors, persistence) ---- */
static void slave_load_setup(uint16_t pk_ca, uint16_t zero_ca, uint16_t gain)
{
    D.iout_valid = 1U; D.iout_clip = 0U; D.iout_pk_ca = pk_ca; D.iout_rms_ca = 90U; A.settings.sl_zero_ca = zero_ca; A.settings.sl_gain = gain;
}
static float load_shown(void) { float v = -1.0f; if (sscanf(row(2), "Load %f", &v) != 1) v = -1.0f; return v; }      /* the Load number on the Run screen: it has a 0.3 A hysteresis band, so it is compared with a tolerance */
static void t_slave_load(void)
{
    int q;
    /* the conversion: load = (peak - zero) x gain: (7.80 - 1.00) A x 1.280 = 8.7 A, against the master CT's 5.5 A */
    fresh(); to_run(); run(9000U); D.load_da = 55U; D.load_ok = 1U; slave_load_setup(780U, 100U, 1280U); A.settings.load_src = 0U; run(4000U);
    CHECK(fabsf(load_shown() - 5.5f) <= 0.35f, "Load Source = Master CT: the Run screen shows the master CT's 5.5 A ('%s')", row(2));
    A.settings.load_src = 1U; run(4000U);
    CHECK(fabsf(load_shown() - 8.7f) <= 0.35f, "Load Source = Slave: (7.80 - 1.00) A x 1.280 = 8.7 A ('%s')", row(2));
    /* every case in which the slave's reading cannot be trusted falls back to the master CT */
    A.settings.sl_gain = 0U; run(4000U); CHECK(fabsf(load_shown() - 5.5f) <= 0.35f, "Slave source but NOT calibrated (gain 0): the master CT stays in charge ('%s')", row(2));
    A.settings.sl_gain = 1280U; D.iout_clip = 1U; run(4000U); CHECK(fabsf(load_shown() - 5.5f) <= 0.35f, "a CLIPPED converter reading (the current is larger than shown): the master CT ('%s')", row(2));
    D.iout_clip = 0U; D.iout_valid = 0U; run(4000U); CHECK(fabsf(load_shown() - 5.5f) <= 0.35f, "no converter reading (an older slave, SLAVE_IOUT 0): the master CT ('%s')", row(2));
    D.iout_valid = 1U; D.have = 0U; run(1000U); D.have = 1U; run(4000U); CHECK(fabsf(load_shown() - 8.7f) <= 0.35f, "and the slave source is back when the reading is ('%s')", row(2));
    A.settings.sl_zero_ca = 900U; run(4000U); CHECK(load_shown() >= 0.0f && load_shown() <= 0.35f, "a peak below the zero never gives a negative load ('%s')", row(2));
    /* the OVERLOAD trip follows the chosen source: 120 A from the slave's converter (24 A above zero x gain 5.000) against 0 A from the master CT */
    fresh(); to_run(); run(9000U); D.load_da = 0U; D.load_ok = 1U; slave_load_setup(2500U, 100U, 5000U); A.settings.load_src = 0U; run(15000U);
    CHECK(MApp_Engine(&A)->contactor[0] == 1U, "Master CT source: the slave's converter current has no say: no overload after 15 s");
    A.settings.load_src = 1U; run(9000U); CHECK(MApp_Engine(&A)->contactor[0] == 1U, "Slave source: 9 s of the slave-derived 120 A is not yet a trip");
    run(2000U); CHECK(MApp_Engine(&A)->contactor[0] == 0U && strcmp(row(3), "Overload") == 0, "after the 10 s Overload Cut Delay the contactor opens: the trip follows the slave's current (%s)", row(3));
    /* the Slave Load Cal page: the ZERO, then the GAIN, saved with the settings and still there after a restart */
    fresh(); to_run(); run(2000U); D.iout_valid = 1U; D.iout_clip = 0U; D.iout_pk_ca = 250U; D.iout_rms_ca = 100U; run(300U);
    tap(3, 100U); for (q = 0; q < 9; q++) tap(0, 100U); run(100U);
    CHECK(A.ui.page == 9U && strstr(row(0), "Slave Load Cal") != NULL && strstr(row(1), "2.5") != NULL, "LOG, MENU x9: the page shows the no-load peak, 2.5 A ('%s' / '%s')", row(0), row(1));
    A.ui.cal_val = 0U; tap(3, 100U);
    CHECK(A.settings.sl_zero_ca == 250U && strstr(row(1), "Zero Set") != NULL, "00.0 and LOG at no load: the peak (2.50 A) is the zero (%u) ('%s')", (unsigned)A.settings.sl_zero_ca, row(1));
    run(3000U); D.iout_pk_ca = 1030U; run(300U); A.ui.cal_val = 100U; tap(3, 100U);
    CHECK(A.settings.sl_gain == 1282U && strstr(row(1), "Calibrated") != NULL, "10.0 A typed with a peak of 10.30 A: gain = 10.0 / (10.30 - 2.50) = 1.282 (%u) ('%s')", (unsigned)A.settings.sl_gain, row(1));
    boot(); CHECK(A.settings.sl_zero_ca == 250U && A.settings.sl_gain == 1282U && A.settings_from_flash, "after a restart the calibration is still there (zero %u, gain %u)", (unsigned)A.settings.sl_zero_ca, (unsigned)A.settings.sl_gain);
    /* the refusals, each with its words, and nothing changes */
    { static const struct { int what; const char *l1; } E[] = { {0, "Run The Unit"}, {1, "No Converter"}, {2, "Reading Clipped"}, {3, "Load Too Low"}, {4, "Enter 2.0-180.0 A"}, {5, "Out Of Range"}, {6, "Save Failed"} };
      unsigned e; for (e = 0U; e < sizeof E / sizeof E[0]; e++)
      {
          uint16_t z0, g0; fresh(); to_run(); run(2000U); D.iout_valid = 1U; D.iout_pk_ca = 1030U; D.iout_rms_ca = 100U; A.settings.sl_zero_ca = 250U; A.settings.sl_gain = 1000U; z0 = 250U; g0 = 1000U; run(300U);
          if (E[e].what == 0) stop_unit();                                                  /* a unit that is stopped (STOP key, no fault) */
          tap(3, 100U); for (q = 0; q < 9; q++) tap(0, 100U); run(100U); A.ui.cal_val = 100U;
          switch (E[e].what)
          {
          case 0: break;                                                       /* the unit was stopped above */
          case 1: D.iout_valid = 0U; break;
          case 2: D.iout_clip = 1U; break;
          case 3: D.iout_pk_ca = 280U; break;                                  /* only 0.30 A above the zero */
          case 4: A.ui.cal_val = 10U; break;                                   /* 1.0 A: below 2.0 A */
          case 5: D.iout_pk_ca = 400U; A.ui.cal_val = 1800U; break;           /* 180 A against 1.50 A above the zero: gain 120 */
          default: fail_prog = 1; break;
          }
          run(100U); tap(3, 100U);
          CHECK(strstr(row(1), E[e].l1) != NULL && A.settings.sl_zero_ca == z0 && A.settings.sl_gain == g0, "refused with '%s': nothing changed ('%s', zero %u, gain %u)", E[e].l1, row(1), (unsigned)A.settings.sl_zero_ca, (unsigned)A.settings.sl_gain);
      } }
    printf("  slave load: the conversion, the fallbacks to the master CT, the overload trip following the source, the zero and gain on the page, saved and restored, every refusal\n");
}


/* ---- v0.9.48: THREE PHASES in the application: the Y and B slaves feed the engine, each phase has its own $M content and clear counter ---- */
static uint8_t last_clr3[3]; static uint32_t clr_at3[3];
static void yb_set(int i, uint8_t state, uint16_t v_dv)
{
    memset(&D.yb[i], 0, sizeof D.yb[i]); D.yb[i].have = 1U; D.yb[i].link_ok = 1U; D.yb[i].e_valid = 1U; D.yb[i].h_valid = 1U; D.yb[i].state = state; D.yb[i].in_band = (uint8_t)(state == 2U);
    D.yb[i].vin_dv = v_dv; D.yb[i].vout_dv = v_dv; D.yb[i].load_ok = 1U;
}
static void yb_gone(int i) { memset(&D.yb[i], 0, sizeof D.yb[i]); }
/* run with three slaves: a changed clear counter makes that slave drop its fault 200 ms later and echo the counter, as the real slaves do */
static void run3(uint32_t ms)
{
    uint32_t e = T + ms;
    while (T < e)
    {
        int ph;
        MApp_Step(&A, K[0], K[1], K[2], K[3], &D, T, F);
        for (ph = 0; ph < 3; ph++)
        {
            uint8_t c = MApp_ControlPh(&A, (uint8_t)ph)->clr;
            if (c != last_clr3[ph]) { last_clr3[ph] = c; clr_at3[ph] = T + 200U; }
            if ((clr_at3[ph] != 0U) && ((int32_t)(T - clr_at3[ph]) >= 0))
            {
                clr_at3[ph] = 0U;
                if (ph == 0) { D.fault = 0U; if (D.state == 3U) D.state = 2U; D.clr_echo = c; }
                else { D.yb[ph - 1].fault = 0U; if (D.yb[ph - 1].state == 3U) D.yb[ph - 1].state = 2U; D.yb[ph - 1].clr_echo = c; }
            }
        }
        T += 5U;
    }
}
static void tap3(int key) { K[key] = 1U; run3(100U); K[key] = 0U; run3(120U); }
static void three_fresh(uint8_t mode)
{
    fresh(); memset(last_clr3, 0, sizeof last_clr3); memset(clr_at3, 0, sizeof clr_at3); A.settings.mode = mode; MF_Init(&A.eng, &A.settings, T);       /* a power-up with this mode saved: the engine builds its units for it */
    yb_set(0, 2U, 2300U); yb_set(1, 2U, 2300U); run3(MUI_SPLASH_MS + 200U);
}
static void t_three_phase_app(void)
{
    const MFOut_t *o;
    /* DOMESTIC: three independent units */
    three_fresh((uint8_t)M_MODE_DOMESTIC); run3(12000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && o->contactor[1] == 1U && o->contactor[2] == 1U, "Domestic with the three slaves heard and running: the three contactors close (%u %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2]);
    CHECK(MApp_ControlPh(&A, 0)->run == 1U && MApp_ControlPh(&A, 1)->run == 1U && MApp_ControlPh(&A, 2)->run == 1U && MApp_ControlValidPh(&A, 1) == 1U && MApp_ControlValidPh(&A, 2) == 1U, "each phase gets its own $M content: run = 1 for R, Y and B, and Y and B are ready to be sent");
    CHECK(MApp_ControlPh(&A, 1)->setv_dv == 2300U && MApp_ControlPh(&A, 2)->maxc_dv == MApp_Control(&A)->maxc_dv, "with the same set voltage (230 V) and Max Correction");
    D.yb[0].link_ok = 0U; D.yb[0].have = 0U; D.yb[0].state = 0U; run3(3000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && o->contactor[1] == 0U && o->contactor[2] == 1U && o->fault[1] != 0U && o->fault[0] == 0U && o->fault[2] == 0U, "the Y link is lost: ONLY the Y unit trips (contactors %u %u %u, faults %u %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2], (unsigned)o->fault[0], (unsigned)o->fault[1], (unsigned)o->fault[2]);
    CHECK(MApp_ControlPh(&A, 0)->run == 1U && MApp_ControlPh(&A, 1)->run == 0U && MApp_ControlPh(&A, 2)->run == 1U, "R and B keep their run level, Y's own run level is 0 (%u %u %u)", (unsigned)MApp_ControlPh(&A, 0)->run, (unsigned)MApp_ControlPh(&A, 1)->run, (unsigned)MApp_ControlPh(&A, 2)->run);
    /* a slave that STOPS without a fault code while its contactor is closed (its inverter is off): only that unit trips */
    three_fresh((uint8_t)M_MODE_DOMESTIC); run3(12000U); D.yb[1].state = 0U; D.yb[1].in_band = 0U; run3(4000U); o = MApp_Engine(&A);
    CHECK(o->contactor[2] == 0U && o->contactor[0] == 1U && o->contactor[1] == 1U && o->fault[2] != 0U, "the B slave stops without a fault code (its inverter is off): only the B unit trips (contactors %u %u %u, faults %u %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2], (unsigned)o->fault[0], (unsigned)o->fault[1], (unsigned)o->fault[2]);
    three_fresh((uint8_t)M_MODE_DOMESTIC); yb_gone(1); run3(12000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && o->contactor[1] == 1U && o->contactor[2] == 0U, "the B slave is never heard: R and Y run, B does not (%u %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2]);
    /* the clear counters are per phase: an inrush trip on Y, START retries only Y */
    three_fresh((uint8_t)M_MODE_DOMESTIC); run3(12000U);
    { uint8_t r0 = MApp_ControlPh(&A, 0)->clr, y0 = MApp_ControlPh(&A, 1)->clr, b0 = MApp_ControlPh(&A, 2)->clr;
      D.yb[0].fault = 1U; D.yb[0].state = 3U; run3(2000U); o = MApp_Engine(&A);
      CHECK(o->fault[1] == MLOG_T_INRUSH && o->contactor[1] == 0U && o->contactor[0] == 1U && o->contactor[2] == 1U, "an inrush trip of the Y slave: only the Y unit is off (contactors %u %u %u, faults %u %u %u)", (unsigned)o->contactor[0], (unsigned)o->contactor[1], (unsigned)o->contactor[2], (unsigned)o->fault[0], (unsigned)o->fault[1], (unsigned)o->fault[2]);
      tap3(1); run3(1500U);
      CHECK(MApp_ControlPh(&A, 1)->clr == (uint8_t)(y0 + 1U) && MApp_ControlPh(&A, 0)->clr == r0 && MApp_ControlPh(&A, 2)->clr == b0, "START retries only Y: its clear counter %u -> %u, R stays %u, B stays %u", (unsigned)y0, (unsigned)MApp_ControlPh(&A, 1)->clr, (unsigned)MApp_ControlPh(&A, 0)->clr, (unsigned)MApp_ControlPh(&A, 2)->clr); }
    /* INDUSTRIAL: one unit, one contactor, all three phases must be there, any trip trips all */
    three_fresh((uint8_t)M_MODE_INDUSTRIAL); run3(12000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && MApp_ControlPh(&A, 0)->run == 1U && MApp_ControlPh(&A, 1)->run == 1U && MApp_ControlPh(&A, 2)->run == 1U, "Industrial with the three slaves: the one contactor closes and all three get run = 1");
    D.yb[1].link_ok = 0U; D.yb[1].have = 0U; D.yb[1].state = 0U; run3(3000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 0U && MApp_ControlPh(&A, 0)->run == 0U && MApp_ControlPh(&A, 1)->run == 0U && MApp_ControlPh(&A, 2)->run == 0U, "the B link is lost: ALL THREE are stopped and the contactor opens");
    three_fresh((uint8_t)M_MODE_INDUSTRIAL); yb_gone(1); run3(12000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 0U && MApp_ControlPh(&A, 0)->run == 0U && MApp_ControlPh(&A, 1)->run == 0U, "Industrial with the B slave never heard: the unit does not start");
    /* SINGLE: the Y and B data are ignored */
    fresh(); memset(last_clr3, 0, sizeof last_clr3); yb_set(0, 3U, 0U); D.yb[0].fault = 17U; yb_gone(1); run3(MUI_SPLASH_MS + 200U + 12000U); o = MApp_Engine(&A);
    CHECK(o->contactor[0] == 1U && o->fault[0] == 0U, "Single mode: a fault on a Y slave that is not even used changes nothing (contactor %u)", (unsigned)o->contactor[0]);
    printf("  three phases in the application: Domestic units independent, a lost or faulty phase touches only itself, Industrial all or nothing, per-phase control frames and clear counters, Single unchanged\n");
}

static uint32_t rs = 5150U;
static uint32_t rnd(void) { rs = (rs * 1664525U) + 1013904223U; return rs >> 8; }

static void t_fuzz(void)
{
    uint32_t round, k, bad_frame = 0U, bad_set = 0U, bad_boot = 0U;
    for (round = 0U; round < 30U; round++)
    {
        fresh(); to_run();
        for (k = 0U; k < 120000U; k++)
        {
            uint32_t r = rnd() % 1000U; int q, c;
            if (r < 6U) K[rnd() % 4U] ^= 1U;
            if (r == 7U) { K[0] = K[3] = 1U; }
            if (r == 8U) { K[0] = K[3] = 0U; }
            if (r == 9U) { D.have = (uint8_t)(rnd() & 1U); D.link_ok = D.have; D.state = (uint8_t)(rnd() % 4U); D.fault = (uint8_t)(rnd() % 25U); D.vin_dv = (uint16_t)(rnd() % 4000U); D.vout_dv = (uint16_t)(rnd() % 4000U); D.freq_chz = (uint16_t)(rnd() % 6500U); }
            if (r == 10U) fail_prog = (int)(rnd() % 6U == 0U);
            if (r == 11U) { uint32_t pos = rnd() % (4U * MFLASH_PAGE); fm[pos] ^= (uint8_t)(1U << (rnd() % 8U)); }
            if (r == 12U && (rnd() % 50U) == 0U) { boot(); if (MS_Validate(&A.settings, 0) != MS_OK) bad_boot++; }
            MApp_Step(&A, K[0], K[1], K[2], K[3], &D, T, F); T += 5U + (rnd() % 20U);
            for (q = 0; q < 4; q++) { if (strlen(F[q]) != 20U) bad_frame++; for (c = 0; c < 20; c++) if (F[q][c] < 32 || F[q][c] > 126) bad_frame++; }
            if (MS_Validate(&A.settings, 0) != MS_OK) bad_set++;
        }
    }
    CHECK(bad_frame == 0U, "3.6 million random steps with random keys, data, failed saves and flash bit flips: every frame is well formed (%u bad)", (unsigned)bad_frame);
    CHECK(bad_set == 0U && bad_boot == 0U, "the live settings are always valid, also after a restart from a damaged flash (%u / %u bad)", (unsigned)bad_set, (unsigned)bad_boot);
    printf("  app fuzz: 3.6 million random steps with failures and bit flips: frames well formed, settings always valid\n");
}

int main(void)
{
    t_boot_and_set();
    t_persist();
    t_failures();
    t_data();
    t_log();
    t_setmode_isolation();
    t_slave_control();
    t_engine_inputs();
    t_status_and_diag();
    t_inverter_off();
    t_hello_and_echo();
    t_inrush_retries_app();
    t_clock_app();
    t_clock_page();
    t_heat_inrush_app();
    t_vcal_app();
    t_response_timer();
    t_watchdog_events();
    t_run_minutes();
    t_ct_peak();
    t_display_speed();
    t_iout_page();
    t_tune_app();
    t_retune_item_keys();
    t_profile_page();
    t_slave_load();
    t_three_phase_app();
    t_slave_wait();
    t_display_stability();
    t_load_calibration();
    t_no_cuts();
    t_fuzz();
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
