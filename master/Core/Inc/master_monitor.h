/**
  ******************************************************************************
  * @file    master_monitor.h
  * @brief   MASTER BRING-UP MONITOR: receives the new slave status frame
  *          (link_proto.h) on USART1 (the R link, PC4 TX / PC5 RX) and shows
  *          input voltage, output voltage, mains frequency and link health on
  *          the 20x4 LCD.
  *
  * WHY A SEPARATE MODE: the existing master firmware waits at start-up for the
  * OLD slave protocol ("[V]" / "[R]..." frames and the "[P]/[D]/[K]" handshake)
  * and shows "LCD Power Failure / Restart the System" until it arrives. The new
  * slave speaks the new protocol, so the old start-up could never get past that
  * screen. In monitor mode main() hands control to MasterMonitor_Run() right
  * after the LCD is initialised, BEFORE any of the old logic or its timers
  * start, so nothing in the old firmware can interfere. The contactor outputs
  * are driven to their OFF level and never touched.
  *
  * MASTER_MONITOR_MODE
  *   1 = monitor (this file owns USART1 RX and the LCD; baud = LINK_BAUDRATE)
  *   0 = the original master firmware, unchanged (USART1 at 115200 for the old
  *       slave protocol)
  *
  * RECEIVE PATH: USART1 RX -> circular DMA (256 bytes, set up here) -> polled
  * by the main loop -> LinkRx_Feed(). The old idle-line scheme (normal-mode DMA
  * restarted on every idle event) is NOT used: at 500000 baud a byte arrives
  * every 20 us and a restart can lose one.
  *
  * LCD: the old LCD driver waits ~1 ms per enable pulse, so refreshing the
  * screen with it would block the receiver for tens of milliseconds. The
  * monitor writes the LCD with its own fast routine, ONE character per loop
  * pass, and only characters that changed.
  ******************************************************************************
  */

#ifndef MASTER_MONITOR_H
#define MASTER_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "link_proto.h"

#define MASTER_MONITOR_MODE     1

/* Within the monitor build (MASTER_MONITOR_MODE 1): 1 = the APPLICATION of release 0.9.2 (Run-Mode screens, Set-Mode, settings in flash, the fault log), with the monitor diagnostics one MENU press away
 * in single-phase mode. 0 = the plain monitor screens exactly as before. The contactors stay OFF in both: the app displays and configures, it does not drive the output (stage 7c). */
#define MASTER_APP_MODE         1

/* The application drives the OUTPUT CONTACTOR(S) and the BUZZER from the fault engine (v0.9.5). 1 = drive them. 0 = never (they stay at their OFF level), for a first test with the load disconnected.
 * A contactor closes only after the slave reports IN_BAND for the Restart Time, and opens at once on any trip, in Set-Mode, on STOP and when the link is lost. */
#define MASTER_DRIVE_CONTACTORS 1

/* TEST ONLY: 1 = the master applies NO voltage limits: I/P LV, I/P HV, O/P LV, O/P HV and the input check before a start are off (the Set-Mode values for them are ignored). The slave has its own switch
 * (SLAVE_TEST_NO_VOLTAGE_CUTS, ready-made file slave_config_STAGE3_cap2752_full_range_MASTER_TEST_NOCUTS.h). Overload, heat, inrush, power-stage, mains and link-lost rules stay. The diagnostics page shows NOCUT.
 * Set to 0 for any unit that is not on the test bench. */
#define MASTER_TEST_NO_VOLTAGE_CUTS 0     /* 0 = the cuts are ON (the safe default since v0.9.11, after a unit lost its IGBTs on a fast variac test) */

/* Inrush and power-stage trips (the slave's hardware break / FLT, its power-stage codes, the CT peak): how many times the master RETRIES, 30 s apart, before it latches until power off and on.
 * 3 = the approved rule (D27). 0 = latch at the FIRST trip: use 0 while a repaired power stage is being commissioned, so that no retry ever re-energises a stage that may still be faulty. */
#define MASTER_INRUSH_RETRIES   3        /* v0.9.47: 3 (the approved rule): an inrush or power-stage trip is retried after the Restart Time (at least 30 s) up to 3 times, then a 5 minute wait and ONE more attempt, then it latches. 0 = latch at the first trip (commissioning, a stage under repair). */

/* The real-time clock (STM32G070 RTC, 32.768 kHz crystal, backup battery on VBAT) for the fault-log time stamps and the Date / Time items in Set-Mode. 1 = on. The driver (master_rtc.c) uses direct register access: it does NOT need the HAL RTC
 * module or a CubeMX RTC configuration, but the board must have the crystal on PC14/PC15 and a battery on VBAT. If the crystal does not start within 3 s the internal LSI (inaccurate) is used. 0 = no clock: the log shows the running time. */
#define MASTER_RTC_ENABLED      1

/* The OVERHEAT input on PA5 (m_board.h MHEAT_PORT / MHEAT_PIN): active LOW, external pull-up on the board. 1 = read it (the master configures the pin as a plain input without a pull). The pin must stay low for the "Overheat Trip Delay"
 * (Set-Mode item 12) before the unit trips; after it has been high for the Restart Time the unit restarts by itself (Auto restart mode), or on START (Manual); three trips within 30 minutes need START. 0 = no overheat input (the flag is never set). */
/* 1 = the independent watchdog (IWDG) runs: the main loop kicks it once per pass; if it stops for MASTER_WATCHDOG_MS the CPU is reset: the contactor outputs go low (open) and the unit starts again. A reset by the watchdog is
 * logged ("Watchdog Reset") at the next start. 0 = no watchdog. */
#define MASTER_WATCHDOG         1
#define MASTER_WATCHDOG_MS      1000U    /* the longest a main-loop pass may take: a flash erase takes tens of ms, the RTC start-up wait (up to 3 s) is BEFORE the watchdog starts */

/* 1 = the CT peak backup: the CT channel is also sampled once per pass of the main loop and the largest reading of each 10 ms step goes to the application, which treats more than 3 x the Overload Current (or the ADC full scale) for 20 ms
 * as an Inrush. 0 = off (the slave's own over-current protection stays the only one). */
#define MASTER_CT_PEAK          1

#define MASTER_HEAT_INPUT       0        /* WARNING: PA5 is ALSO the R-phase input (R_Phase_Pin, main.h) of the original board, and PA6 and PA7 are the Y and B phase inputs: do NOT switch this on without the schematic. OFF since v0.9.21: with the pin read as low (floating, no pull-up fitted, or another function) the master saw permanent overheat and would not start. Set 1 only after checking PA5 with a meter: it must read HIGH (3.3 V) when cool. */

/* The load current (the master CT): the average of MASTER_CT_SAMPLES ADC readings. The application converts it with its own CALIBRATION (zero and gain, in flash; the defaults reproduce the original firmware's
 * scale and 3 A offset). Zero: Set-Mode "Calibration" = Yes (no load). Span: the Load Calibration page (MENU x3 in Run-Mode), typing the clamp-meter reading while the load runs. */
#define MASTER_CT_SAMPLES       200U
#define MON_LINK_TIMEOUT_MS     500U      /* no valid frame for this long = NO LINK */
#define MON_RXBUF_SIZE          256U      /* DMA ring, power of two (12.8 ms at 500000 baud) */
#define MON_LCD_COLS            20U
#define MON_LCD_ROWS            4U
#define MON_COMPOSE_PERIOD_MS   100U

typedef struct
{
    LinkStatus_t last;               /* the latest valid status frame                 */
    uint8_t      have_frame;         /* at least one valid frame has arrived          */
    uint8_t      have_seq;
    uint8_t      last_seq;
    uint32_t     t_last_ms;          /* HAL_GetTick() when the last valid frame came  */

    uint32_t     frames_ok;          /* valid frames                                  */
    uint32_t     frames_bad;         /* complete lines rejected (checksum/format)     */
    uint32_t     overflows;          /* over-long lines                               */
    uint32_t     lost;               /* frames missing, from gaps in the seq number   */
    uint32_t     bytes_rx;           /* every byte the DMA delivered                  */
    uint32_t     uart_errors;        /* overrun / framing / noise flags cleared       */
    uint32_t     fps;                /* valid frames in the last whole second         */
    uint32_t     max_poll_gap_us;    /* longest time the loop went without polling    */

    /* link version 2 frames from the slave (only a slave built with SLAVE_LINK_V2 sends them) */
    uint8_t      e_valid;            /* at least one $E has arrived                   */
    uint16_t     e_flags2;           /* the latest $E flags (LINKV2_FLAG_*)           */
    uint8_t      e_clr;              /* the clear counter the slave last saw          */
    uint32_t     e_t_ms;             /* when it arrived                               */
    uint8_t      h_valid;            /* a $H has arrived                              */
    uint16_t     h_win_lo_dv, h_win_hi_dv, h_vout_max_dv;
    uint16_t     e_setv_dv;          /* the set voltage the slave reports it is using ($E)  */
    uint16_t     e_iout_rms_ca, e_iout_pk_ca;   /* the slave's converter-side current ($E, optional fields), hundredths of an ampere */
    uint8_t      r_id, r_status;     /* the last $R (the answer to a calibration command) */
    uint8_t      t_n, t_step, t_result;           /* the last $T (the autotune's progress): a counter, its step and result */
    uint16_t     h_prof_cap, h_prof_kib, h_prof_kiu, h_prof_vref_dv, h_prof_maxc_dv; uint8_t h_prof_n, h_prof_knee, h_tuned;   /* the slave's stored profile, from the optional fields of $H */
    uint16_t     t_ticks, t_vin_dv; int16_t t_dv_dv;
    uint8_t      rst_wdg;            /* this master was restarted by its watchdog (RCC reset flags, read at start) */
    uint8_t      h_rst;              /* the slave's reset cause from its last $H: 1 = its watchdog */
    int32_t      r_value;
    uint32_t     r_n;                /* how many $R frames arrived */
} MasterMon_t;

extern MasterMon_t g_mon;

/* Never returns. Call right after LCD_Init() in main(). */
void MasterMonitor_Run(void) __attribute__((noreturn));
void MasterApp_Run(void) __attribute__((noreturn));          /* the application (MASTER_APP_MODE 1); never returns */

/* ---- Hardware-independent parts (also used by the PC-side tests) ----------- */

void Mon_Reset(void);

/* Feeds the bytes the DMA has written since *rd, up to write index wr, into the
 * frame receiver and updates g_mon. ring_size must be a power of two. */
void Mon_ProcessBytes(const volatile uint8_t *ring, uint32_t ring_size,
                      uint32_t *rd, uint32_t wr, uint32_t now_ms);

/* Rolls the one-second frame counter. Call with the current time often. */
void Mon_Tick(uint32_t now_ms);

/* Builds the four LCD rows (each exactly MON_LCD_COLS characters + NUL). */
void Mon_Compose(const MasterMon_t *m, uint32_t now_ms,
                 char rows[MON_LCD_ROWS][MON_LCD_COLS + 1U]);

/* Changed-characters-only LCD writer. */
typedef void (*MonLcdWriteFn)(uint8_t rs, uint8_t value);   /* rs 0 = command, 1 = data */
typedef struct
{
    char     have[MON_LCD_ROWS][MON_LCD_COLS + 1U];   /* what is on the display        */
    int32_t  cur_row, cur_col;                        /* where the LCD cursor is, -1 = unknown */
    uint32_t cursor;                                  /* next cell to look at          */
} MonLcd_t;

void     MonLcd_Init(MonLcd_t *l);
/* Writes at most one changed character (plus a cursor command if needed).
 * Returns the number of LCD bytes written (0 = display already up to date). */
uint32_t MonLcd_Step(MonLcd_t *l, const char want[MON_LCD_ROWS][MON_LCD_COLS + 1U],
                     MonLcdWriteFn write);

#ifdef __cplusplus
}
#endif

#endif /* MASTER_MONITOR_H */
