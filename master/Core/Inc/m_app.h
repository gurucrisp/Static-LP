/**
  * @file    m_app.h
  * @brief   Master core, part 7: the application. Owns the settings, the fault log, the UI and the keys, and connects them to the world through plain data: raw key levels and the slave's
  *          latest status in, a 4 x 20 text frame out. PURE LOGIC (the flash is MFlash_t), so the whole thing runs on the PC; master_monitor.c supplies the hardware.
  *          Release 0.9.4: it displays, configures AND now tells the slave what to do through the $M frame (m_ctl.h): run / stop, the Set Voltage, the Max Correction and the clear counter. It still never drives a contactor.
  */
#ifndef M_APP_H
#define M_APP_H
#include <stdint.h>
#include "m_ui.h"
#include "m_ntc.h"
#include "m_phase.h"
#include "m_ctl.h"

/* one display filter: the filtered reading, the number shown, whether it is initialised, when the shown number last changed */
typedef struct { int32_t ema; int32_t shown; uint8_t init; uint32_t t_chg; } MDispFilt_t;    /* one display filter */
#ifdef __cplusplus
extern "C" {
#endif

/* flash pages (STM32G070, 128 KB, 2 KB pages). The old firmware keeps its settings in the last page, 0x0801F800: these are below it and never touch it. */
#define MAPP_PAGE_SETTINGS_A   0x0801F000UL
#define MAPP_PAGE_SETTINGS_B   0x0801E800UL
#define MAPP_PAGE_CALIB_A      0x0801E000UL      /* the load-current calibration */
#define MAPP_PAGE_CALIB_B      0x0801D800UL
#define MAPP_PAGE_LOG_A        0x0801D000UL
#define MAPP_PAGE_LOG_B        0x0801C800UL

/* LOAD CURRENT, from the master CT. The glue gives the average ADC count (12 bit, times 16). current = gain * (counts - zero) * 0.213 dA per count. The defaults reproduce the original firmware
 * (65.25 / 766, minus 3 A), so nothing changes until a calibration is made: ZERO from Set-Mode "Calibration" = Yes (output off, no load), SPAN from the Load Calibration page (MENU, MENU, MENU in
 * Run-Mode) with the clamp-meter reading typed in while the load runs. Stored in flash; Factory Reset keeps it. */
#define MAPP_CT_DA_PER_COUNT_X1000   213U      /* nominal tenths of an amp per ADC count: 65.25 / 766 / 4 A */
#define MAPP_CT_ZERO_DEFAULT_X16     (141U * 16U)   /* the original firmware's 3 A offset, in counts */
#define MAPP_CT_GAIN_DEFAULT         1000U     /* x1000 */
#define MAPP_CT_MIN_CAL_DA           20U       /* the span calibration needs at least 2.0 A of load (spec) */
typedef struct { uint16_t zero_x16, gain_x1000; uint8_t from_flash; } MCal_t;

/* the output counts as "at the set voltage" within this many tenths of a volt of it (2.0 V) */
#define MAPP_BAND_DV   20

/* What the master knows about ONE of the Y and B slaves (v0.9.48). The R slave's data is the top-level fields of MAppData_t, as it always was. */
typedef struct
{
    uint8_t  have, link_ok;            /* valid $S frames from this slave / a valid $S within the last 500 ms                         */
    uint8_t  e_valid, h_valid;         /* an $E / a $H from this slave has been heard                                                  */
    uint8_t  state, fault;             /* its state ($S: 0 stopped, 1 starting, 2 RUNNING, 3 fault) and fault code                      */
    uint8_t  in_band, heat;            /* $E IN_BAND / OVERHEAT                                                                        */
    uint8_t  clr_echo;                 /* its clear counter, to start the master's from                                                */
    uint16_t vin_dv, vout_dv;          /* its input and output, tenths of a volt                                                       */
    uint16_t load_da; uint8_t load_ok; /* its load, tenths of an ampere (from its converter current, a later step)                     */
    uint16_t ntc_raw; uint8_t ntc_valid; /* its heat-sink NTC: the 12 bit reading of PA5 / it is fresh (v0.9.49)                         */
    uint16_t sq_id, sq_delay_us, sq_period_us;   /* its last phase sequence round (v0.9.51): the round id (0 = none), delay, period                       */
    uint8_t  wait_valid, wait_ctx;     /* v0.9.54, for its Re-tune: its start-up wait and what for (as the R slave's)                                         */
    uint8_t  tune_n, tune_step, tune_result; uint16_t tune_ticks; int16_t tune_dv_dv;   /* its $T frames: counter, step, result, duty, injection             */
    uint8_t  prof_tuned, prof_n, prof_knee; uint16_t prof_cap, prof_kib, prof_kiu, prof_vref_dv, prof_maxc_dv;   /* its stored profile ($H), for the Transformer Profile page */
    uint16_t iout_rms_ca, iout_pk_ca; uint8_t iout_valid, iout_clip;      /* v0.9.55: its converter current ($E), for its load (its Slave Load Cal) */
} MAppSlave_t;

typedef struct
{
    uint8_t  have;               /* a valid $S frame within the link timeout                         */
    uint16_t vin_dv, vout_dv;    /* tenths of a volt                                                  */
    uint16_t freq_chz;           /* hundredths of a hertz, 0 = unknown                                */
    uint8_t  state;              /* the slave's state: 0 stopped, 1 starting, 2 running, 3 fault       */
    uint8_t  fault;              /* the slave's fault code, 0 = none                                  */
    uint8_t  link_ok;            /* the slave's frames are arriving                                    */
    uint16_t load_da;            /* the master CT, tenths of an amp                                    */
    uint8_t  load_ok;            /* the CT was read (otherwise the load is shown as --.-)              */
    uint16_t ct_x16;             /* the CT: average ADC count times 16. With ct_raw = 1 the application computes the load from this and its calibration, and load_da / load_ok are ignored */
    uint8_t  ct_raw;
    uint8_t  e_valid;            /* $E frames are arriving (the slave runs the master-control config)  */
    uint8_t  in_band;            /* $E IN_BAND, from a fresh $E                                        */
    uint8_t  heat;               /* the slave's OVERHEAT flag (not available yet: 0)                   */
    uint8_t  ct_peak;            /* the CT reached full scale (not measured yet: 0)                    */
    uint8_t  clr_echo;           /* $E: the clear counter the slave last saw                           */
    MDateTime_t clock;           /* the real-time clock, read by the glue about once a second            */
    uint8_t  clock_ok;           /* the clock has been set (the RTC never returns 2000, and the battery kept it) */
    uint8_t  rtc_src;            /* the RTC clock source: 0 none, 1 crystal (LSE), 2 internal LSI */
    uint8_t  slave_wdg;          /* the slave's last reset (from its $H) was its watchdog: a "w" on the diagnostics page */
    uint16_t ct_max_x16;         /* the largest CT reading (12 bit x 16) of the fast samples since the last step; 0 = not sampled: the CT peak backup is idle */
    uint16_t rtc_bdcr, rtc_icsr; /* raw status bits, for the Clock page */
    uint16_t e_setv_dv;          /* $E: the set voltage the slave is actually using, tenths of a volt (0 = not known) */
    uint16_t iout_rms_ca, iout_pk_ca;   /* $E: the slave's converter-side current (CT primary), AC RMS and peak, hundredths of an ampere */
    uint8_t  prof_tuned, prof_n, prof_knee;                                   /* $H: the slave's stored profile (all 0 = none, or an older slave): tuned flag, points, saturation knee */
    uint16_t prof_cap, prof_kib, prof_kiu, prof_vref_dv, prof_maxc_dv;         /* its duty cap (ticks), gain scale boost / buck (thousandths), the input it was made at and the Max Correction it was made with (tenths of a volt) */
    uint8_t  tune_n;                    /* $T: counts every progress frame received (the glue's counter, wraps) */
    uint8_t  tune_step, tune_result;    /* the newest $T frame: its step (1..) and result (0 = a step, 1..6 = the end of the sweep) */
    uint16_t tune_ticks, tune_vin_dv;   /* its duty ticks and the input voltage (tenths) */
    int16_t  tune_dv_dv;                /* the injection it measured (tenths of a volt) */
    uint8_t  iout_valid, iout_clip;     /* the slave measures it / the measurement is at the limit of its range */
    uint8_t  wait_valid;         /* the slave is in its start-up wait and says what for (its $S flags)  */
    uint8_t  wait_ctx;           /* what it waits for: bit 0 Z zero-cross, 1 V input window, 2 M measurement, 3 F FLT line low, 4 B break input low (also a floating one), 5 O output too high, 6 R relays */
    uint8_t  h_valid;            /* a $H frame was received: the three below are the slave's own limits */
    uint16_t win_lo_dv, win_hi_dv, vout_max_dv;   /* the slave's input window and output guard, tenths of a volt */
    MAppSlave_t yb[2];                  /* [0] = the Y slave, [1] = the B slave: the Industrial and Domestic modes use them (v0.9.48) */
    uint16_t sq_id, sq_delay_us, sq_period_us;   /* the R slave's last phase sequence round (v0.9.51)                                           */
    uint16_t ntc_raw; uint8_t ntc_valid;   /* the R slave's heat-sink NTC: the 12 bit reading of PA5 (the $S frame) / it is fresh (v0.9.49) */
} MAppData_t;

typedef struct
{
    const MFlash_t *fl;
    MStore_t     sstore;
    MSettings_t  settings;       /* the live settings                                                 */
    MLog_t       log;
    MUI_t        ui;
    MKeys_t      keys;
    MFaults_t    eng;                  /* the fault and restart engine: it decides run, the clear pulses, the contactor and the buzzer */
    MFOut_t      eout;
    MCtlOut_t    ctl_out;
    MPhase_t     phase;                            /* the phase sequence decision and the angles between the phases (v0.9.51) */
    uint8_t      seq_fsm, seq_round, seq_act, seq_wait;   /* the round: 0 idle, 1 armed, 2 synced; its id; the action for the glue (0 none, 1 ARM, 2 SYNC); Industrial + Phase Sequence + not measured yet */
    uint32_t     t_seq, t_seq_next;
    uint8_t      ntc_heat;                         /* 1 = the heat-sink temperature trips the unit (the default); 0 = it is only shown */
    uint32_t     ntc_f16[3];                       /* the filtered reading x 16, per phase (R, Y, B) */
    uint8_t      ntc_have[3], heat_on[3], temp_st[3]; int16_t temp_dc[3];
    MCtlOut_t    ctl_out_yb[2];                    /* the $M content for the Y and B slaves */
    uint8_t      clr_cnt_yb[2], clr_synced_yb[2], ctl_valid_yb[2], have_seen_yb[2];
    uint32_t     t_first_have_yb[2];
    MDispFilt_t  f_vin_yb[2], f_vout_yb[2], f_load_yb[2];   /* the display filters of the Y and B readings */              /* the content of the next $M frame                                    */
    MLimits_t    lim;                  /* the slave's limits, from its $H frame                               */
    MStore_t     cstore;               /* the calibration record                                              */
    MCal_t       cal;
    uint8_t      clr_cnt, clr_synced;  /* the clear counter sent to the slave; synced to the slave's own on its first $E */
    uint8_t      lim_set;
    uint8_t      retries_cfg;
    uint8_t      k_id;                 /* the next $K id, 1..255 */
    uint8_t      tune_state, tune_id, tune_cmd, tune_due, tune_tx_n, tune_seen, tune_step, tune_result, tune_abort_sent;   /* the autotune: see m_app.c */
    uint8_t      tune_sl_state, tune_sl_wv, tune_sl_wait, tune_sl_fault;      /* what the slave is doing (its $S state, the start-up wait, its fault), shown on the tuning screen */
    uint8_t      tune_ph, tune_all, tune_id_yb[2], hello_yb[2];               /* v0.9.54: the phase being tuned (0 R, 1 Y, 2 B); 1 = then the next phase (All, one by one); the application's last $K id on the Y / B link (TUNE, TABORT, calibration); ask Y / B for a hello */
    int16_t      tune_dv_dv;
    uint16_t     tune_ticks;
    uint32_t     tune_t0, tune_t_tx, tune_t_frame, tune_t_done;
    uint8_t      ct_hi, ct_peak_on;    /* CT peak backup: the reading is above the limit now; the debounced condition that the engine sees as an Inrush */
    uint32_t     ct_hi_t0;             /* when the reading went above the limit */
    uint32_t     run_min_total, run_acc_ms, t_run_last;   /* running time: minutes the output has been ON in all (loaded at start, saved by the glue), the milliseconds towards the next minute, the last step */
    uint8_t      run_dirty;                               /* a whole minute was added: the glue saves the counter */
    uint8_t      rec_active, rec_have; /* response timer: an excursion (output more than 5 V from the set voltage) is being timed / one has been timed */
    uint16_t     rec_peak, rec_s10, rec_pk_v;   /* the largest error so far (tenths of a volt); the last recovery time (tenths of a second); its largest error (volts) */
    uint32_t     rec_t0;
    uint8_t      vcal_ph;                          /* v0.9.55: the phase whose slave the calibration command goes to (0 R, 1 Y, 2 B) */
    uint8_t      vcal_wait, vcal_due, vcal_id, vcal_cmd, vcal_ch, vcal_tx_n;   /* a calibration command is out: waiting for its $R; a (re)send is due; its id, command, channel, transmissions so far */
    int32_t      vcal_num; char vcal_letter; uint32_t vcal_t0, vcal_t_tx, t_now;
    uint8_t      clk_set_pending;      /* a new Date/Time was saved in Set-Mode and waits for the glue to write it to the RTC */
    MDateTime_t  clk_set;
    uint8_t      hello_n, hello_force;   /* hello_force: ask the slave for a fresh hello once (a new profile was stored: its numbers travel in the hello) */
    uint32_t     t_hello;
    uint8_t      ctl_valid;            /* $M may be sent: the slave was heard ($E), or its $S frames have been arriving for 3 s */
    uint8_t      have_seen;
    uint32_t     t_first_have, m_sent;
    char         status[21];           /* what the unit is waiting for, for the Run screen */
    MAppData_t   last;                 /* the latest data, for the diagnostics page */
    uint8_t      no_cuts;              /* TEST: the master applies no voltage limits (see MApp_SetTestNoCuts)  */
    MDispFilt_t  f_vin, f_vout, f_load;    /* the display filters */
    uint8_t      settings_from_flash;   /* 1 = the settings were read from flash, 0 = the defaults are in use */
    uint32_t     save_failures;
} MApp_t;

void    MApp_Init(MApp_t *a, const MFlash_t *fl, uint32_t page_set_a, uint32_t page_set_b, uint32_t page_log_a, uint32_t page_log_b, uint32_t now_ms);
/* call every few ms: raw key levels (1 = pressed) and the slave's latest data; fills the frame to draw */
void    MApp_Step(MApp_t *a, uint8_t menu, uint8_t start, uint8_t stop, uint8_t log, const MAppData_t *d, uint32_t now_ms, char frame[MUI_ROWS][MUI_COLS + 1U]);
uint8_t MApp_SetModeOpen(const MApp_t *a);      /* the output must be off while this is 1 */
#define MAPP_TUNE_NEXT_MS       4000U   /* v0.9.54, Re-tune All: the result of one phase is shown this long before the next phase starts */
#define MAPP_SEQ_SYNC_DELAY_MS  20U     /* from the $K SEQ (ARM) to the SYNC byte: the slaves act on the $K in their main loop */
#define MAPP_SEQ_TIMEOUT_MS     500U    /* from the SYNC byte to the three results ($E every 100 ms) */
#define MAPP_SEQ_FAST_MS        300U    /* between rounds while the sequence is not known */
#define MAPP_SEQ_SLOW_MS        5000U   /* between rounds once it is (the monitoring) */
#define MAPP_SEQ_SILENT_MS      2000U   /* between rounds when NO slave answered the last one (a build without the measurement) */
/* The glue asks this every pass: 0 = nothing to do, 1 = send $K SEQ <*round> on all three links, 2 = send the SYNC byte (LINKV2_SYNC_BYTE) on all three links at once. Each is returned once. */
uint8_t MApp_TakeSeqAction(MApp_t *a, uint8_t *round);
const MPhase_t *MApp_Phase(const MApp_t *a);
void    MApp_SetNtcHeat(MApp_t *a, uint8_t on);                       /* MASTER_NTC_HEAT: 1 = the temperature trips the unit */
/* the heat-sink temperature of phase ph (0 R, 1 Y, 2 B), tenths of a degC: returns MNTC_OK / MNTC_OPEN / MNTC_SHORT, or 255 = no reading */
uint8_t MApp_TempDeci(const MApp_t *a, uint8_t ph, int16_t *t_dc);
const MCtlOut_t *MApp_ControlPh(const MApp_t *a, uint8_t ph);   /* the $M content for phase ph: 0 = R (MApp_Control), 1 = Y, 2 = B */
uint8_t MApp_ControlValidPh(const MApp_t *a, uint8_t ph);       /* as MApp_ControlValid, per phase */
const MCtlOut_t *MApp_Control(const MApp_t *a); /* the content of the $M frame to send to the slave (the caller sends it every 100 ms, and at once when it changes) */
/* TEST ONLY: 1 = the master's own voltage faults (I/P LV, I/P HV, O/P LV, O/P HV) and the input check before a start are OFF: the engine is given limits of 1 to 999 V instead of the Set-Mode values.
 * The overload, heat, inrush, power-stage, mains, link and flip rules are unchanged. Never ship with this on. */
void    MApp_SetTestNoCuts(MApp_t *a, uint8_t on);
/* the load-current calibration lives in two flash pages of its own; call once after MApp_Init to enable it (it also turns on Set-Mode "Calibration") */
void    MApp_InitCal(MApp_t *a, uint32_t page_a, uint32_t page_b);
/* inrush / power-stage retries: 3 = the approved rule (retry after 30 s, up to 3 times), 0 = latch at the FIRST trip. Use 0 while commissioning a repaired power stage. */
void    MApp_SetInrushRetries(MApp_t *a, uint8_t n);
/* The slave announces its limits ($H) only at its own power-up. If the master started later it asks: returns 1 (at most every 2 s) while the slave's $E is heard but no $H has arrived; the glue then sends a $K HELLO. */
uint8_t MApp_HelloDue(MApp_t *a, uint32_t now);

/* ---- voltage calibration over the link ($K CALV / CALO / CALR, answered by $R) ---- */
typedef struct { uint8_t id, cmd; int32_t num; char letter; uint8_t ph; } MCalK_t;        /* cmd: MAPP_KCMD_*; ph: the link it goes on (0 R, 1 Y, 2 B: v0.9.54, a Re-tune of the Y or B phase) */
#define MAPP_KCMD_CALV  1U            /* num = the true input voltage, tenths of a volt   */
#define MAPP_KCMD_CALO  2U            /* num = the true output voltage, tenths of a volt  */
#define MAPP_KCMD_CALR  3U            /* letter = 'V' or 'O': back to nominal             */
#define MAPP_SL_MIN_DELTA_CA 100U   /* the slave load calibration needs the peak at least 1 A above the zero (hundredths of an ampere) */
#define MAPP_KCMD_TUNE  4U            /* start the transformer autotune                    */
#define MAPP_KCMD_TABORT 5U           /* stop it                                           */
#define MAPP_TUNE_FRAME_TIMEOUT_MS 8000UL   /* no $T frame for this long while tuning: the slave is lost, the master aborts          */
#define MAPP_TUNE_RESULT_MS        6000UL   /* the result screen stays this long, or until a key */
/* CT PEAK BACKUP (approved: the slave's IGBT over-current is primary, the master's CT the backup): a CT reading above 3 x the Overload Current, or at the full scale of the ADC, for 20 ms in a row is an Inrush
 * (retry rules as for the slave's own). The CT here is an averaged level, not the instantaneous current: the glue samples it every pass of the main loop and passes the largest reading of each step. */
/* DISPLAY SPEED of the voltages and the load current: the shown value follows the reading through a first-order filter (time constant about MAPP_DISP_EMA_DIV x 10 ms) and may change at most once every MAPP_DISP_HOLD_MS;
 * a real step (5 V or more) is shown at once. Larger = calmer display. (v0.9.29: 96 and 300 ms; before 64 and no hold, which was about 0.6 s.) */
#define MAPP_DISP_EMA_DIV      96
#define MAPP_DISP_HOLD_MS      300UL

#define MAPP_CT_PEAK_FACTOR    3U
#define MAPP_CT_FULL_X16       (4000U * 16U)
#define MAPP_CT_PEAK_MS        20UL    /* the condition must hold continuously for this long (in time, not in calls: the glue calls the application every 10 ms, the tests every 5 ms) */
#define MAPP_VCAL_TIMEOUT_MS   3000UL /* the slave averages for 1 s: it has this long to answer                                   */
#define MAPP_VCAL_RETRY_MS     200UL  /* a $K is repeated (same id) every 200 ms, three times in all                               */
void    MApp_SetRunMinutes(MApp_t *a, uint32_t minutes);                      /* once at start-up: the counter loaded from the battery-backed registers */
uint32_t MApp_RunMinutes(const MApp_t *a);                                   /* the minutes the output has been ON in all */
uint8_t MApp_TakeRunDirty(MApp_t *a);                                        /* 1 once per minute while the output is on: save MApp_RunMinutes() */
void    MApp_BootReset(MApp_t *a, uint8_t wdg);
uint8_t MApp_ShowsTune(const MApp_t *a);                                      /* the tuning screen is up (the glue draws it over everything) */
void    MApp_TuneScreen(const MApp_t *a, uint32_t now, char out[4][21]);                               /* once at start-up, after the log is open: a restart by the watchdog is logged ("Watchdog Reset") */
uint8_t MApp_NextKId(MApp_t *a);                                              /* ONE id counter for every $K (a repeated id is a retry for the slave): HELLO and calibration use it */
uint8_t MApp_TakeKCommand(MApp_t *a, MCalK_t *out, uint32_t now);             /* 1 = a $K frame is due now (first send or a retry): build and send it */
void    MApp_OnKResult(MApp_t *a, uint8_t id, uint8_t status, int32_t value); /* a $R arrived on the R link */
void    MApp_OnKResultPh(MApp_t *a, uint8_t ph, uint8_t id, uint8_t status, int32_t value);   /* v0.9.54: a $R on link ph (0 R, 1 Y, 2 B): only the Re-tune of that phase takes it from the Y and B links */
uint8_t MApp_TakeYbHello(MApp_t *a, uint8_t i);                               /* v0.9.54: 1 once when the Y (0) or B (1) slave must be asked for its hello again (a Re-tune stored a new profile) */
/* a Date or Time saved in Set-Mode: returns 1 once, with the value to write to the real-time clock (seconds are set to 0 by the glue) */
uint8_t MApp_TakeClockSet(MApp_t *a, MDateTime_t *out);
void    MApp_NoteFrameSent(MApp_t *a);          /* the glue calls this for every $M it sends (the diagnostics count them) */
uint8_t MApp_ShowsDiag(const MApp_t *a);
uint8_t MApp_ShowsProfile(const MApp_t *a);                     /* the transformer profile page is shown (LOG, then MENU x8) */
void    MApp_ProfilePage(const MApp_t *a, uint32_t now, char out[4][21]);   /* draws it */
uint8_t MApp_ShowsIout(const MApp_t *a);                        /* the converter-side current page is shown (LOG, then MENU x7) */
void    MApp_IoutPage(const MApp_t *a, uint32_t now, char out[4][21]);   /* draws it */
uint8_t MApp_ShowsClock(const MApp_t *a);                       /* the Clock service page is shown (LOG, then MENU x4) */
void    MApp_ClockPage(const MApp_t *a, uint32_t now, char out[4][21]);   /* draws it: the date and time, whether it counts as set, the clock source, the raw bits */
void    MApp_Diag(const MApp_t *a, uint32_t now_ms, char frame[MUI_ROWS][MUI_COLS + 1U]);   /* the control diagnostics page */
uint8_t MApp_ControlValid(const MApp_t *a);     /* 1 once the slave has been heard ($E) and the clear counter synced: only then is $M sent */
const MFOut_t *MApp_Engine(const MApp_t *a);    /* contactors, buzzer, faults: what the glue must drive */
uint8_t MApp_ShowsMonitor(const MApp_t *a);     /* the app wants the monitor diagnostics drawn instead of the frame */

#ifdef __cplusplus
}
#endif
#endif
