/**
  * @file    m_ui.h
  * @brief   Master core, part 6: the screens and the key logic: splash, Run-Mode pages, the fault-log viewer, the password prompt and Set-Mode. PURE LOGIC: key events and the measured data in,
  *          a 4 x 20 text frame and one-shot requests out. The caller (the app) draws the frame on the LCD and acts on the requests.
  *
  *  Run-Mode: MENU+LOG held 2 s opens Set-Mode (asking the password first if one is set); LOG short opens the fault log; MENU short turns the 3-phase page when Display Scroll is No.
  *  Set-Mode: MENU = next item (the value just edited is checked first: "Invalid Value" and it is put back if it breaks a rule); START = increment / toggle; STOP = move the cursor to the next digit;
  *            MENU+LOG held 2 s = leave and SAVE; "Discard Changes" = Yes + MENU leaves without saving; 30 s without a key leaves without saving ("Timeout - Not Saved").
  *            The output must be OFF while Set-Mode is open: MUI_SetModeOpen() tells the engine.
  */
#ifndef M_UI_H
#define M_UI_H
#include <stdint.h>
#include "m_keys.h"
#include "m_settings.h"
#include "m_log.h"
#include "m_clock.h"
#include "m_faults.h"
#ifdef __cplusplus
extern "C" {
#endif

#define MUI_ROWS 4U
#define MUI_COLS 20U
#define MUI_SPLASH_MS     3000UL
#define MUI_SET_IDLE_MS   30000UL
#define MUI_MSG_MS        1500UL
#define MUI_BLINK_MS      500UL

/* one-shot requests for the app */
#define MUI_REQ_SAVE        0x01U   /* the settings changed: write them (MS_Save of ui->live) */
#define MUI_REQ_CLEAR_LOG   0x02U
#define MUI_REQ_CALIBRATE   0x04U
#define MUI_REQ_RETUNE      0x08U
#define MUI_REQ_START       0x10U   /* the START key in Run-Mode */
#define MUI_SERVICE_IDLE_MS 60000U   /* a service page (monitor, diagnostics, load calibration) returns to the Run screen after this long without a key */
#define MUI_REQ_STOP        0x20U
#define MUI_REQ_VCAL_SAVE   0x0100U /* LOG on a Voltage Calibration page: the entered meter value is MUI_CalValue(), the channel is page 5 (input) or 6 (output) */
#define MUI_REQ_SET_CLOCK   0x80U   /* Set-Mode was saved with a new Date or Time: the value is MUI_ClockToSet() */   /* the STOP key in Run-Mode  */
#define MUI_REQ_CAL_SAVE    0x40U   /* LOG on the Load Calibration page: the entered meter value is MUI_CalValue() */
/* what the app can do: a request the app cannot serve is answered "Not Available" on the screen */
#define MUI_FEAT_CALIBRATE  0x01U
#define MUI_FEAT_RETUNE     0x02U

typedef struct
{
    uint8_t  have[3];                 /* valid readings from that phase's slave                                  */
    uint8_t  load_ok[3];              /* the load current is measured (master CT): otherwise it is shown as --.-  */
    uint16_t vin_dv[3], vout_dv[3], load_da[3];
    uint16_t freq_chz;                /* 0 = unknown                                                             */
    uint16_t vll_dv[3];               /* R-Y, Y-B, B-R phase-to-phase; 0 = a phase is missing ("---")            */
    uint8_t  out_on;                  /* the output is on                                                        */
    uint16_t vin_raw_dv, vout_raw_dv; /* the slave's calibrated voltage readings, unfiltered, tenths of a volt (the Voltage Calibration pages show them) */
    uint8_t  active;                  /* the unit is running or starting (the Voltage Calibration pages ask for it to be stopped first) */
    uint16_t wait_s[3];               /* per phase: seconds until the next AUTOMATIC restart attempt after an Inrush / Power-stage / Heat trip, 0 = none or not counting */
    uint8_t  fault[3];                /* per phase: MLOG_T_* fault or "Appear" shown for it, 0 = none            */
    uint8_t  fcode[3];                /* the slave's code behind a Mains / Power-stage text                      */
    uint8_t  seq_state;               /* 0 ok, 1 reverse, 2 same phase                                           */
    const char *status;               /* what the unit is waiting for ("Start in 03 s", ...), shown on the status row while the output is off; NULL or "" = "Output OFF" */
} MUIRun_t;

typedef enum { MUI_SPLASH = 0, MUI_RUN, MUI_LOG, MUI_PW, MUI_SET } MUIScreen_t;

typedef struct
{
    MSettings_t *live;                /* the settings in use                                                     */
    MSettings_t  work;                /* the copy being edited in Set-Mode                                       */
    const MLimits_t *lim;             /* what the slave announced (NULL until it has)                            */
    MLog_t      *log;                 /* the fault log                                                           */
    uint8_t      screen, item, cursor, page, features;
    uint16_t     req;                 /* pending requests (MUI_REQ_*): 16 bits, the 8 original ones are used up */
    MDateTime_t  clk_now, clk_work, clk_backup, clk_set;   /* the real-time clock now; the Date/Time being edited, its backup at item entry; what is to be written to the clock on save */
    uint8_t      clk_now_valid, clk_active, clk_dirty, clk_pending;    /* the clock is set; an edit session is open; a digit was typed on this item; a changed value waits for the save */
    uint8_t      unit_active;           /* the unit is running or starting (from the last tick): on a Voltage Calibration page STOP then stops it instead of moving the cursor */
    uint8_t      cal_cursor, cal_init;  /* the Load Calibration page: which digit is being edited; the entry starts from the reading */
    uint16_t     cal_val;               /* the entered meter value, tenths of an amp */
    uint8_t      pw[3], yn;           /* the password being typed; the Yes/No of an action item                  */
    uint16_t     ed_backup;           /* the value of the item when it was entered                               */
    uint16_t     log_idx;
    uint8_t      prev_shown;
    uint32_t     t0, t_key, t_page, t_msg, t_blink;   /* t_blink: the last key press; the cursor is shown for 0.5 s from it, then hidden 0.5 s, and so on */
    char         msg[2][MUI_COLS + 1U];            /* the message lines as typed, centred when drawn */
    uint8_t      msg_on;
    uint8_t      return_to_run;       /* after a message, go to Run-Mode                                         */
} MUI_t;

void    MUI_Init(MUI_t *u, MSettings_t *live, MLog_t *log, uint32_t now_ms);
void    MUI_SetLimits(MUI_t *u, const MLimits_t *lim);
void    MUI_SetFeatures(MUI_t *u, uint8_t feat);
void    MUI_Key(MUI_t *u, MKEvent_t ev, uint32_t now_ms);
/* timers (splash, scroll, message and idle timeouts) and the frame to draw */
void    MUI_Tick(MUI_t *u, const MUIRun_t *d, uint32_t now_ms, char out[MUI_ROWS][MUI_COLS + 1U]);
uint8_t MUI_SetModeOpen(const MUI_t *u);        /* Set-Mode or its password prompt is open: the output must be off */
uint8_t MUI_Take(MUI_t *u, uint16_t req);         /* 1 once if that request is pending */
void    MUI_Notify(MUI_t *u, const char *line1, const char *line2, uint32_t now_ms);   /* a message for 1.5 s (for example "Save Failed") */
/* single-phase Run-Mode: MENU toggles between the Run screen and the monitor diagnostics (the app draws those) */
uint8_t MUI_ShowsMonitor(const MUI_t *u);
void     MUI_SetClock(MUI_t *u, const MDateTime_t *now, uint8_t valid);   /* the application tells the UI the real-time clock every step (valid = the RTC has been set) */
const MDateTime_t *MUI_ClockToSet(const MUI_t *u);                        /* after MUI_Take(MUI_REQ_SET_CLOCK): the date and time to write to the clock */
uint16_t MUI_CalValue(const MUI_t *u);      /* the value typed on the Load Calibration page, tenths of an amp */
uint8_t MUI_ShowsCal(const MUI_t *u);         /* single-phase, the fourth MENU page: Load Calibration (the UI draws it) */
uint8_t MUI_ShowsVCal(const MUI_t *u);         /* 1 = the input, 2 = the output Voltage Calibration page (the UI draws them), else 0 */
uint8_t MUI_ShowsIout(const MUI_t *u);        /* the converter-side current page (the app draws it) */
uint8_t MUI_ShowsClock(const MUI_t *u);       /* single-phase, the fifth service page (LOG, then MENU x4): the Clock page (the app draws it) */
uint8_t MUI_ShowsDiag(const MUI_t *u);        /* single-phase, the third MENU page: the control diagnostics (the app draws them) */

#ifdef __cplusplus
}
#endif
#endif
