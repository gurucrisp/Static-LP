/**
  ******************************************************************************
  * @file    slave_ctrl.h
  * @brief   Converter + inverter control for the slave: zero-cross
  *          synchronised converter commutation, inverter leg selection, the
  *          start-up sequence, the protection/fault state machine and the
  *          regulator hook.
  *
  * DESIGN RULES (each one exists because a wrong converter polarity or a stale
  * synchronisation puts the DC link across the mains):
  *
  *  1. POLARITY IS MEASURED, NOT ASSUMED. At every zero-cross event the sign of
  *     the mains just before the event is read from the ADC. The next half-
  *     cycle has the opposite sign, and that decides the converter diagonal
  *     (positive -> 3H+4L, negative -> 3L+4H). No assumption is made about
  *     which ZCD edge means which half-cycle.
  *  2. THE ZCD PATTERN IS VALIDATED, NOT TRUSTED. Two hypotheses are tracked in
  *     parallel: "every edge is a zero-cross" (square wave) and "every falling
  *     edge is a zero-cross" (pulse train). The converter only ever runs on a
  *     hypothesis that has been seen to be self-consistent (interval about half
  *     a mains period AND the measured sign alternating) for 8 events in a row.
  *     Anything else - one pulse per cycle, noise, a wrong-way sense - never
  *     starts, or stops at once, with a fault.
  *  3. AT EVERY ZERO-CROSS EVENT the energised diagonal is dropped FIRST, before
  *     any bookkeeping; the new one is switched on only after the software dead
  *     time (CTRL_DEADTIME_US). Both diagonals are never on together.
  *  4. THE CONVERTER TURNS OFF BY PREDICTION. At each event the next one is
  *     expected one measured half-cycle later; a TIM2 compare switches the
  *     diagonal off CTRL_PREDICT_LEAD_US before that, event or no event. A second
  *     compare (watchdog) raises ZC_LOST if the event is more than
  *     CTRL_WD_MARGIN_US late. A lost pulse thus leaves the converter OFF.
  *  4b. A POLARITY GUARD runs on the 20 kHz ADC tick: if the line sign is
  *     opposite to the energised diagonal (a phase jump, a swapped sense,
  *     anything the zero-cross timing cannot see) everything goes off within
  *     about 0.2 ms.
  *  5. ANY FAULT turns everything off from the interrupt that sees it. A restart
  *     always starts from a fresh synchronisation (trackers and mode cleared).
  *  5b. THE PRIMARY IS NEVER LEFT OPEN AND UNCONTROLLED (slave_relay.h, rules R1-R5). The relays are
 *     BYPASS relays across the series transformer's primary. Start: relays shorted -> converter ->
 *     inverter at zero duty (low sides on) -> shunts opened -> duty ramps. Stop: duty zero -> shunts
 *     closed -> inverter disabled. A fault that is not an overcurrent follows the same unwinding;
 *     break / FLT / a relay fault switch everything off at once and close the shunts immediately.
 *  6. BREAK / FLT escalate: retry after 10, 20, 30, 40, 50 s, latched on the
  *     6th; the counter resets after 5 minutes of fault-free running.
  *
  * WHAT IS NOT HERE YET: the output CONTACTOR (CONT1/CONT2) and the load soft-start
  * limiter (they depend on the master), and the master's run/stop and set-point
  * commands.
  ******************************************************************************
  */
#ifndef SLAVE_CTRL_H
#define SLAVE_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "slave_config.h"
#include "slave_features.h"

/* ---- Tunables (override with -D if needed) -------------------------------- */
#ifndef CTRL_TARGET_V
#define CTRL_TARGET_V            230.0f     /* regulated output (master set-point later) */
#endif
#ifndef CTRL_VIN_MIN_V
#define CTRL_VIN_MIN_V           140.0f     /* firmware's outer input envelope (spec 2.1) */
#endif
#ifndef CTRL_VIN_MAX_V
#define CTRL_VIN_MAX_V           300.0f
#endif
#ifndef CTRL_VIN_START_HYST_V
#define CTRL_VIN_START_HYST_V    10.0f      /* HYSTERESIS: the converter STARTS only inside [MIN + this, MAX - this] but only TRIPS
                                               outside [MIN, MAX]. Starting the converter loads the supply: without this gap a start at
                                               exactly MIN trips (F09) as soon as the input sags. */
#endif
/* Transformer-connection check (SLAVE_TF_CHECK, slave_config.h): the result rides in the optional diag2 field with bit 15 = 0, bit 14 = 0, bit 13 = 1:
 *   bits 12..11  verdict: 1 = OK (Vout above Vin), 2 = FLIPPED (Vout below Vin), 3 = inconclusive (inside the dead band)
 *   bits 10..0   (Vout - Vin) in 0.1 V, 11-bit two's complement (-102.4 ... +102.3 V) */
#ifndef CTRL_TF_SETTLE_MS
#define CTRL_TF_SETTLE_MS        1500UL     /* after the converter has really been running */
#endif
#ifndef CTRL_TF_WINDOW_MS
#define CTRL_TF_WINDOW_MS        2000UL     /* averaging window */
#endif
#ifndef CTRL_TF_MIN_SAMPLES
#define CTRL_TF_MIN_SAMPLES      20U        /* fewer RMS results than this in the window = no verdict */
#endif
#ifndef CTRL_TF_DEADBAND_V
#define CTRL_TF_DEADBAND_V       0.5f       /* |Vout - Vin| below this is "inconclusive" (the two sensors differ by a few tenths of a volt at rest) */
#endif

#ifndef CTRL_VOUT_MAX_V
#define CTRL_VOUT_MAX_V          280.0f     /* hardware protection, independent of the master */
#endif

#ifndef CTRL_DEADTIME_US
#define CTRL_DEADTIME_US         40U        /* converter software dead time (old firmware: 40 us) */
#endif
#ifndef CTRL_EVENT_MIN_US
#define CTRL_EVENT_MIN_US        7500U      /* valid spacing of zero-cross events: a half-cycle */
#endif
#ifndef CTRL_EVENT_MAX_US
#define CTRL_EVENT_MAX_US        12500U     /*   of 40...66 Hz mains                           */
#endif
#ifndef CTRL_WATCHDOG_US
#define CTRL_WATCHDOG_US         14500U     /* nominal watchdog before the first interval is known */
#endif
#ifndef CTRL_PREDICT_LEAD_US
#define CTRL_PREDICT_LEAD_US     300U       /* the converter is switched OFF this long BEFORE the next
                                               expected zero-cross, by timer, whether or not the event
                                               arrives. A missing event therefore leaves it OFF, never
                                               conducting against the line. */
#endif
/* SLAVE_GLITCH_SKIP: at most CTRL_GLITCH_MAX skipped half-cycles in CTRL_GLITCH_WINDOW_US; the next one is the fault */
/* SLAVE_TF_RUN_CHECK: the flip check (see slave_features.h) */
#ifndef CTRL_TFRUN_BASE_TICKS
#define CTRL_TFRUN_BASE_TICKS    15U        /* the sensor offset is recorded while the correction is below this many effective ticks */
#endif
#ifndef CTRL_TFRUN_MIN_TICKS
#define CTRL_TFRUN_MIN_TICKS     40U        /* the output is judged only above this correction */
#endif
#ifndef CTRL_TFRUN_WINDOW
#define CTRL_TFRUN_WINDOW        5U         /* measurement updates per window (100 ms at 50 Hz) */
#endif
#ifndef CTRL_TFRUN_VERDICT_V
#define CTRL_TFRUN_VERDICT_V     1.0f       /* the signed change in the commanded direction: >= +1.0 V = correct, <= -1.0 V = backwards */
#endif
#ifndef CTRL_TFRUN_CLEAR_V
#define CTRL_TFRUN_CLEAR_V       2.5f       /* a window this far AGAINST the command decides at once: it is not noise */
#endif
#ifndef CTRL_TFRUN_STRIKES
#define CTRL_TFRUN_STRIKES       2U         /* windows in a row that say "backwards" */
#endif
#ifndef CTRL_TFRUN_JUMP_V
#define CTRL_TFRUN_JUMP_V        3.0f       /* an input step bigger than this between two updates restarts the window */
#endif
#ifndef CTRL_TFRUN_WATCH_MS
#define CTRL_TFRUN_WATCH_MS      60000UL    /* stop watching this long after the start */
#endif
#ifndef CTRL_GLITCH_WINDOW_US
#define CTRL_GLITCH_WINDOW_US    2000000UL
#endif
#ifndef CTRL_GLITCH_MAX
#define CTRL_GLITCH_MAX          2U
#endif
uint32_t SlaveCtrl_GlitchSkips(void);       /* how many half-cycles have been skipped since power-up (0 when SLAVE_GLITCH_SKIP is 0) */
#ifndef CTRL_GUARD_BLANK_US
#define CTRL_GUARD_BLANK_US      250U       /* polarity guard ignores this long after a commutation (the
                                               newest ADC samples still belong to the previous half-cycle) */
#endif
#ifndef CTRL_GUARD_CONFIRM_TICKS
#define CTRL_GUARD_CONFIRM_TICKS 2U         /* the polarity guard needs this many CONSECUTIVE 50 us ticks showing the
                                               wrong sign (so one noisy ADC sample cannot trip it) */
#endif
#ifndef CTRL_RESHUNT_TIMEOUT_MS
#define CTRL_RESHUNT_TIMEOUT_MS  (SLAVE_RELAY_SETTLE_MS + 400U)   /* gives up waiting for the shunts to close */
#endif
#ifndef CTRL_WD_MARGIN_US
#define CTRL_WD_MARGIN_US        2000U      /* ZC_LOST fault if the event is this late (converter is
                                               already off by then) */
#endif
#ifndef CTRL_SYNC_GOOD_EVENTS
#define CTRL_SYNC_GOOD_EVENTS    8U         /* consecutive valid events required to start */
#endif
#ifndef CTRL_SYNC_FRESH_US
#define CTRL_SYNC_FRESH_US       25000U     /* the latest event must be this recent */
#endif

#ifndef CTRL_RECT_TO_INV_MS
#define CTRL_RECT_TO_INV_MS      3000U      /* converter running before the inverter starts (old: 3000) */
#endif
#ifndef CTRL_INV_ZERO_DUTY_MS
#define CTRL_INV_ZERO_DUTY_MS    100U       /* zero duty right after the inverter is enabled (old: 100) */
#endif
#ifndef CTRL_DUTY_MAX_PCT
#define CTRL_DUTY_MAX_PCT        90U        /* cap: the gate drivers are BOOTSTRAPPED, a high side held
                                               on for ever starves its supply (see slave_power.h) */
#endif
#ifndef CTRL_STAGE2_DUTY_COUNTS
#define CTRL_STAGE2_DUTY_COUNTS  64U        /* stage 2 injection test: the EFFECTIVE high-side on-time in timer ticks (64 = 1.0 us = 2 % of the 50 us period; 96 = 3 %, 160 = 5 %).
                                               The dead time (POWER_DEADTIME_TICKS, 128 = 2 us) is ADDED to it: a pulse shorter than the dead time is not generated at all. */
#endif
#if (CTRL_STAGE2_DUTY_COUNTS > 160U)
#error "The stage-2 injection test is limited to 5 % effective duty (160 ticks)"
#endif
#if (CTRL_STAGE2_DUTY_COUNTS < 1U)
#error "CTRL_STAGE2_DUTY_COUNTS must be at least 1 tick"
#endif

/* Stage 2 = OPEN-LOOP INJECTION TEST, one shot per start, run in CTRL_RUN with the primary free:
 *   BASE  zero duty, the baseline (Vout - Vin) is averaged (the two sensors differ by a few tenths of a volt at rest)
 *   RAMP  the duty ramps linearly 0 -> CTRL_STAGE2_DUTY_COUNTS
 *   HOLD  the duty is held; after the settle time (Vout - Vin) is averaged
 *   DOWN  the duty ramps back to 0;  DONE: zero duty until a stop / fault / restart.
 * Verdict: the injected change = (mean hold - baseline), counted positive in the EXPECTED direction (boost: Vout up, buck: Vout down). OK if it is above the
 * dead band, FLIPPED if it is below minus the dead band (the transformer, or a leg, is reversed), else inconclusive. It is sent in the same diag2 word as the
 * converter-only check (bit 13, verdict bits 12..11, value in 0.1 V in bits 10..0) and the master shows it as  TF OK +1.2V / TF FLIP -1.1V / TF ? +0.1V. */
/* Stage 3 (closed loop): the CAP on the regulator's EFFECTIVE high-side on-time in timer ticks (the dead time is added on top, so the timer CCR is at most
 * dead time + this). The regulator's integrator is clamped to it (no wind-up). WHAT A CAP IS WORTH, measured on the bench with no load (the correction is NOT
 * proportional to the ticks: steep over the first 64, then about 0.009 to 0.014 V per tick):  640 ticks = about 7.5 V at 200 V / 9.6 V at 250 V (reaches 230 V from
 * about 222 to 239 V);  1600 = 17 / 22 V (212 to 252 V);  2000 = 20.5 / 27.6 V (208 to 259 V);  2752 (the duty ceiling) = 27 / 38 V (202 to 270 V, the upper figure
 * extrapolated). Default 640: the FIRST closed-loop run is deliberately small; widen it step by step. */
#ifndef CTRL_STAGE3_MAX_EFF_TICKS
#define CTRL_STAGE3_MAX_EFF_TICKS  640U
#endif
#if (SLAVE_POWER_STAGE >= 3) && (CTRL_STAGE3_MAX_EFF_TICKS < 1U)
#error "CTRL_STAGE3_MAX_EFF_TICKS must be at least 1"
#endif

/* Soft start of the closed loop: the regulator's setpoint starts at the present input and slides to 230 V. Defaults = the OLD firmware's values (proven on its hardware):
 * 3 V/s for the first 2 s, then 10 V/s until within 3 V of the target, then 5 V/s. From 205 V that is about 4 s (3.6 s from 210 V, 2.5 s from 220 V). Faster is possible (see
 * slave_config_STAGE3_cap2752_full_range_fast_start.h: 10 / 30 / 10 V/s, 0.5 s: about 1.4 s from 205 V, no overshoot in simulation) but the old values were chosen for a LOADED unit:
 * a faster ramp means a faster rise of the load voltage. The 3 s the converter runs alone before the inverter starts (CTRL_RECT_TO_INV_MS) is a separate delay and is NOT changed. */
#ifndef CTRL_SOFTSTART_SLOW_SLOPE_VPS
#define CTRL_SOFTSTART_SLOW_SLOPE_VPS   3.0f
#endif
#ifndef CTRL_SOFTSTART_FAST_SLOPE_VPS
#define CTRL_SOFTSTART_FAST_SLOPE_VPS   10.0f
#endif
#ifndef CTRL_SOFTSTART_FINAL_SLOPE_VPS
#define CTRL_SOFTSTART_FINAL_SLOPE_VPS  5.0f
#endif
#ifndef CTRL_SOFTSTART_SLOW_TIME_MS
#define CTRL_SOFTSTART_SLOW_TIME_MS     2000UL
#endif

#ifndef CTRL_INJ_BASE_MS
#define CTRL_INJ_BASE_MS         1500UL
#endif
#ifndef CTRL_INJ_RAMP_MS
#define CTRL_INJ_RAMP_MS         2000UL
#endif
#ifndef CTRL_INJ_HOLD_MS
#define CTRL_INJ_HOLD_MS         5000UL     /* includes the settle time */
#endif
#ifndef CTRL_INJ_SETTLE_MS
#define CTRL_INJ_SETTLE_MS       1500UL     /* at the start of HOLD: not averaged */
#endif
#ifndef CTRL_INJ_DOWN_MS
#define CTRL_INJ_DOWN_MS         1000UL
#endif
#ifndef CTRL_INJ_DEADBAND_V
#define CTRL_INJ_DEADBAND_V      0.3f       /* |injected change| below this = inconclusive */
#endif
#ifndef CTRL_INJ_ABORT_V
#define CTRL_INJ_ABORT_V         15.0f      /* |Vout - Vin - baseline| beyond this at any moment: duty to zero at once, verdict inconclusive */
#endif

#ifndef CTRL_MEAS_STALE_MS
#define CTRL_MEAS_STALE_MS       100U
#endif
#ifndef CTRL_RANGE_DEBOUNCE
#define CTRL_RANGE_DEBOUNCE      3U         /* consecutive RMS results out of range before a trip */
#endif

#ifndef CTRL_RESTART_STEP_MS
#define CTRL_RESTART_STEP_MS     10000UL
#endif
#ifndef CTRL_RESTART_MAX_MS
#define CTRL_RESTART_MAX_MS      60000UL
#endif
#ifndef CTRL_LATCH_AFTER
#define CTRL_LATCH_AFTER         6U
#endif
#ifndef CTRL_COUNTER_RESET_MS
#define CTRL_COUNTER_RESET_MS    300000UL
#endif
#ifndef CTRL_CONDITION_RETRY_MS
#define CTRL_CONDITION_RETRY_MS  2000UL     /* for faults that are conditions, not hardware events */
#endif

/* ---- States, fault codes ---------------------------------------------------- */
typedef enum
{
    CTRL_OFF = 0,        /* nothing energised; zero-cross pattern is still monitored                      */
    CTRL_SYNC,           /* waiting for a validated zero-cross pattern, sane inputs, relays SHORTED        */
    CTRL_RECT,           /* converter commutating; relays SHORTED, inverter off (stage 1 stops here)       */
    CTRL_INV_START,      /* inverter enabled, ZERO duty (low sides on), relays still SHORTED               */
    CTRL_UNSHUNT,        /* shunts opening (rule R4); duty zero; waiting for the relays to settle          */
    CTRL_RUN,            /* relays FREE, regulating (stage 2: fixed duty)                                  */
    CTRL_RESHUNT,        /* duty zero, shunts closing; the inverter is disabled only when they have closed  */
    CTRL_FAULT,          /* everything off, relays shorted, waiting for the retry time                     */
    CTRL_LATCHED         /* too many escalating trips: needs SlaveCtrl_ClearLatch()                        */
} CtrlState_t;

typedef enum
{
    CF_NONE            = 0,
    CF_BREAK           = 1,    /* TIM1/TIM15 hardware break (BKIN)                  - escalates */
    CF_FLT             = 2,    /* FLT input (PD2)                                   - escalates */
    CF_ZC_PATTERN      = 3,    /* a zero-cross event arrived at the WRONG TIME (spacing outside 7.5..12.5 ms) - escalates */
    CF_POLARITY        = 4,    /* mains too small to read a sign at an event        - escalates */
    CF_SENSE_MISMATCH  = 5,    /* AC_IN and AC_OUT disagree on the sign             - escalates */
    CF_VOUT_HIGH       = 6,    /* output above CTRL_VOUT_MAX_V                      - escalates */
    CF_INV_ENABLE      = 7,    /* inverter would not leave break                    - escalates */
    CF_LINE_REVERSED   = 11,   /* line polarity opposite to the energised diagonal  - escalates */
    CF_RELAY           = 12,   /* relays not in the state the controller requires    - escalates (hard) */
    CF_ZC_STALE        = 13,   /* main-loop check: latest zero-cross event is too old - condition */
    CF_NO_COMMUTATION  = 14,   /* converter running state, events arrive, but no diagonal ever energised - escalates */
    CF_ZC_SEQUENCE     = 15,   /* an event arrived on time but the measured sign did NOT flip / the edge type did not alternate - escalates */
    CF_PRE_ENERGISE    = 16,   /* last-moment check: the newest samples show the OPPOSITE polarity to the one about to be energised - escalates */
    CF_ZC_LOST         = 8,    /* WATCHDOG COMPARE: next zero-cross event overdue    - condition */
    CF_VIN_RANGE       = 9,    /* input outside the permitted window                - condition */
    CF_MEAS_STALE      = 10,   /* measurement stopped updating                      - condition */
    CF_NOT_TUNED       = 18,   /* no valid transformer profile (SLAVE_TUNE_ENFORCE): the master shows "Re-tune Required"; Re-tune clears it */
    CF_RETUNE_MISMATCH = 19,   /* the injection measured at start does not match the stored profile: a different transformer (SLAVE_TUNE_APPLY); "Re-tune Required" */
    CF_TRANSFORMER_FLIP = 17   /* the series transformer is connected backwards (SLAVE_TF_RUN_CHECK) - latches at once, in every mode */
} CtrlFault_t;

#define CTRL_ZC_UNKNOWN   0U   /* pattern not (yet) recognised          */
#define CTRL_ZC_BOTH      1U   /* every edge is a zero-cross (square)   */
#define CTRL_ZC_FALL      2U   /* every falling edge is a zero-cross    */

/* Live state: read it in the debugger's Live Expressions. */
typedef struct
{
    volatile uint8_t   state;           /* CtrlState_t                                   */
    volatile uint8_t   mode;            /* CTRL_ZC_*                                     */
    volatile uint16_t  fault;           /* code of the current / latest fault            */
    volatile uint8_t   fault_count;     /* escalating trips since the last reset         */
    volatile uint8_t   good_all;        /* consecutive valid events, "every edge"        */
    volatile uint8_t   good_fall;       /* consecutive valid events, "falling edges"     */
    volatile uint8_t   run_request;     /* 1 = the master/bench wants the stage running  */
    volatile uint8_t   run_allowed;     /* 1 = the interrupts may energise the converter */
    volatile uint8_t   inv_active;      /* 1 = the inverter is enabled and the interrupts may write its duty */
    volatile uint8_t   duty_ok;         /* 1 = the shunts are open and settled: NONZERO duty is allowed (rule R1) */
    volatile int8_t    conv_pol;        /* polarity currently energised: +1, -1, 0 none  */
    volatile int8_t    pending_pol;     /* polarity to energise after the dead time      */
    volatile uint8_t   boost;           /* inverter direction request                    */
    volatile uint16_t  duty_counts;     /* inverter duty request, timer counts           */
    volatile uint16_t  isr_fault;       /* fault raised in an interrupt                  */
    volatile uint32_t  events;          /* accepted zero-cross events while running      */
    volatile uint32_t  commutations;    /* converter diagonals energised                 */
    volatile uint32_t  t_event_us;      /* time of the latest zero-cross event           */
    volatile uint32_t  interval_us;     /* spacing of the last two valid events (half-cycle) */
    volatile int8_t    last_sign_in;    /* sign read at the latest event                 */
    volatile int8_t    last_sign_out;
    volatile int32_t   last_dev_in;     /* deviation sums behind those signs             */
    volatile int32_t   last_dev_out;
    volatile uint8_t   break_flags;     /* POWER_BREAK_* seen                            */
    volatile uint8_t   fault_ctx;       /* context of the latest fault, CTRL_CTX_* below  */
    volatile uint8_t   wait_ctx;        /* what the SYNC state is waiting for, CTRL_WAIT_* (| CTRL_CTX_VALID) */
    volatile uint8_t   flt_idle, flt_run;   /* FLT (PD2) falling edges seen while NOT running / running (saturate at 31) */
    volatile uint8_t   brk_idle, brk_run;   /* break (BKIN) events seen while NOT running / running (saturate at 7)      */
    volatile uint8_t   first_pending;             /* 1 = no diagonal energised yet since the start (SLAVE_CONV_FIRST_POL) */
    volatile uint8_t   map_agree, map_disagree;   /* edge-direction mapping check, see SlaveCtrl_MapWord() */
    volatile uint16_t  trip_cap;            /* where in the mains half-cycle the latest running FLT / break event hit (see SlaveCtrl_TripCapture) */
} SlaveCtrl_t;

/* Fault context byte, sent to the master in bits 8..15 of the link "flags" field and shown on row 3 of
 * the master display while the slave is in FAULT:
 *   bits 0-2 : controller state when the fault was handled (CtrlState_t)
 *   bits 3-4 : zero-cross mode (CTRL_ZC_*)
 *   bit  5   : FLT (PD2) read LOW when the fault was handled   -> still low = a real, lasting assertion
 *   bit  6   : a BKIN input (PB12 / PA9) read LOW              -> high again = it was only a glitch
 *   bit  7   : context valid                                                                           */
#define CTRL_CTX_STATE_MASK  0x07U
#define CTRL_CTX_MODE_SHIFT  3U
#define CTRL_CTX_FLT_LOW     0x20U
#define CTRL_CTX_BKIN_LOW    0x40U
#define CTRL_CTX_VALID       0x80U

/* While the slave is waiting in SYNC ("START" on the master, nothing energised) the same byte says WHAT IT IS
 * WAITING FOR: every bit set is a condition that is not met yet. */
#define CTRL_WAIT_ZC         0x01U   /* no validated zero-cross pattern yet (or its events are stale)   */
#define CTRL_WAIT_VIN        0x02U   /* input not yet inside MIN+hysteresis .. MAX-hysteresis (140+10 .. 300-10 V) */
#define CTRL_WAIT_MEAS       0x04U   /* no valid voltage measurement yet                                */
#define CTRL_WAIT_FLT        0x08U   /* FLT (PD2) reads LOW                                             */
#define CTRL_WAIT_BKIN       0x10U   /* a break input (PB12 / PA9) reads LOW - this includes a FLOATING one */
#define CTRL_WAIT_VOUT       0x20U   /* output voltage above the protection limit                       */
#define CTRL_WAIT_RELAY      0x40U   /* relays not yet settled in the SHORTED (bypass) state             */

extern SlaveCtrl_t g_ctrl;

/* ---- Main-loop API ---------------------------------------------------------- */
void     SlaveCtrl_Init(void);          /* after SlavePower_Init(): state OFF, all off */
void     SlaveCtrl_Service(void);       /* call on EVERY main-loop pass                */
void     SlaveCtrl_SetRunRequest(uint8_t on);
void     SlaveCtrl_ClearLatch(void);    /* manual reset after CTRL_LATCHED             */

/* The injection guard (SLAVE_INJ_GUARD): see slave_features.h */
#ifndef CTRL_DIR_FLIP_MAX_TICKS
#define CTRL_DIR_FLIP_MAX_TICKS 96U     /* the direction may change at once only while the correction is this small (about 1 V)        */
#endif
#ifndef CTRL_VIN_STEP_V
#define CTRL_VIN_STEP_V         15.0f   /* an input step this big between two RMS results holds the correction at zero                  */
#endif
/* After a guard event the regulator restarts from zero with a FAST ramp and no slow first phase (the output is already live, the 3 V/s phase is only for a cold start). V/s. */
#ifndef CTRL_GUARD_RESTART_SLOPE_VPS
#define CTRL_GUARD_RESTART_SLOPE_VPS  30.0f
#endif
#ifndef CTRL_GUARD_RESTART_FINAL_VPS
#define CTRL_GUARD_RESTART_FINAL_VPS  10.0f
#endif
#ifndef CTRL_GUARD_HOLD_MS
#define CTRL_GUARD_HOLD_MS      100UL   /* ... until the input has been steady this long, then the regulator restarts from zero        */
#endif
uint32_t SlaveCtrl_GuardCount(void);    /* how many times the guard has taken the correction to zero (diagnostics) */

/* ---- Master control (roadmap step 3; decisions D1, D5, D9). Without SLAVE_MASTER_CONTROL: SlaveCtrl_OnControl does nothing and the getters describe the bench behaviour. ---- */
#ifndef CTRL_LINK_TIMEOUT_MS
#define CTRL_LINK_TIMEOUT_MS   1000UL   /* no valid $M for this long: the slave stops (D9)                          */
#endif
#ifndef CTRL_IN_BAND_V
#define CTRL_IN_BAND_V         2.0f     /* IN_BAND: running and |Vout - set voltage| within this (my choice, D1) */
#endif
/* A valid $M arrived (main-loop context). run is a LEVEL; the set voltage is adopted only while the unit is NOT energised (the master repeats it every 100 ms, so
 * it is picked up the moment the unit stops); the clear counter is adopted by the first $M and a CHANGE of it clears a latched fault (the fault's cause is checked again
 * by the normal start); maxc is the Max Correction (used by the autotune, step 5). */
void     SlaveCtrl_OnControl(uint8_t run, uint16_t setv_dv, uint8_t clr, uint16_t maxc_dv, uint32_t now_ms);
/* v0.9.61, the load-connect hold ($M hold, a LEVEL): while it is 1 and the unit is in RUN the regulator's duty is FROZEN (no integration, no ramp) so the load arriving through the
 * contactor (and its bounce) meets a steady converter instead of a loop chasing the dip. Every protection and the input-step guard still run. A hold ends when the master sends 0,
 * when the link is lost, or by itself CTRL_LOAD_HOLD_MAX_MS after it began. */
#ifndef CTRL_LOAD_HOLD_MAX_MS
#define CTRL_LOAD_HOLD_MAX_MS  1000UL
#endif
void     SlaveCtrl_OnLoadHold(uint8_t hold, uint32_t now_ms);
uint8_t  SlaveCtrl_LoadHoldActive(uint32_t now_ms);   /* 1 = the duty is frozen now (RUN and a hold in force) */
uint16_t SlaveCtrl_Flags2(void);        /* LINKV2_FLAG_* for the $E frame                                     */
uint16_t SlaveCtrl_SetvDv(void);        /* the set voltage in use, tenths of a volt; 0 = none accepted yet (master control) */
uint8_t  SlaveCtrl_ClrEcho(void);       /* the last clear counter value seen in $M                            */
uint16_t SlaveCtrl_MaxCorrDv(void);

/* ---- TUNE MODE (SLAVE_TUNE_MODE): the transformer autotune drives the stage open loop ---- */
#ifndef CTRL_TUNE_SLEW_TPS
#define CTRL_TUNE_SLEW_TPS      1500.0f   /* the open-loop duty may rise by this many effective ticks per second (0 to the 2752 cap in 1.8 s); it may fall at once */
#endif
#ifndef CTRL_TUNE_HARD_OVER_V
#define CTRL_TUNE_HARD_OVER_V   8.0f      /* the controller's own limit: the output more than Max Correction + this above the input stops the unit (fault 6), whatever the sweep thinks */
#endif
uint8_t SlaveCtrl_TuneBegin(void);                    /* 1 = accepted (the unit is OFF, nothing latched, the master is there, Max Correction set): the normal start sequence begins, in tune mode */
void    SlaveCtrl_TuneRequest(uint16_t eff_ticks);    /* the open-loop BOOST duty wanted now (effective ticks); applied in RUN, slew-limited and capped */
void    SlaveCtrl_TuneEnd(void);                      /* stop: duty to zero at once, the shunts close, OFF */
uint8_t SlaveCtrl_TuneRunning(void);                  /* tune mode is on (any phase of the start sequence or RUN) */
uint8_t SlaveCtrl_TuneStopped(void);                  /* 1 = the unit is stopped for a tune: OFF, or latched with "Re-tune Required" (fault 18 or 19), which a tune is exactly the cure for */
uint8_t SlaveCtrl_TuneReady(void);                    /* in RUN for at least 1 s: the sweep may start */     /* the Max Correction last sent in $M, tenths of a volt (0 = not set)   */

uint8_t  SlaveCtrl_LinkState(void);     /* link_proto state: 0 stopped 1 starting 2 running 3 fault */
uint16_t SlaveCtrl_FaultCode(void);     /* CF_* while in FAULT/LATCHED, else 0         */
uint8_t  SlaveCtrl_FaultContext(void);  /* CTRL_CTX_* while in FAULT/LATCHED, else 0   */
uint8_t  SlaveCtrl_WaitContext(void);   /* CTRL_WAIT_* | CTRL_CTX_VALID while in SYNC, else 0 */
/* Bench diagnostics for the link "diag" field: how often the MCU_FLT net went low while the converter was NOT running and
 * while it WAS. Events while idle cannot come from the converter's own current: they are noise or supply disturbance.
 * bits 0-4 FLT idle | 5-9 FLT running | 10-12 break idle | 13-15 break running. Counts since power-up. */
uint16_t SlaveCtrl_DiagWord(void);
/* Latest FLT / break event that hit WHILE THE CONVERTER WAS RUNNING (0 = none yet), for the link "diag2" field:
 *   bits 0-9  time since the last valid zero-cross event, in 16 us units (0..1023 = 0..16.4 ms)
 *   bit  10   the POSITIVE-half diagonal was energised at that moment
 *   bit  11   the NEGATIVE-half diagonal was energised at that moment
 *   bit  15   valid
 * The converter turns on about 40 us after the zero-cross event and off about 0.3 ms before the next one, so the time tells
 * whether a trip hits at turn-on, in the middle of the half-cycle, at turn-off, or at random (noise). */
uint16_t SlaveCtrl_TripCapture(void);

/* PASSIVE converter-mapping check (nothing needs to run: it counts in OFF / SYNC too).
 * The OLD firmware, which ran this hardware, picks the converter diagonal from the ZCD EDGE DIRECTION:
 *     falling edge -> PB3 + PB5 (3H + 4L)      rising edge -> PC4 + PC5 (3L + 4H)
 * This firmware picks it from the measured AC_IN SIGN:  positive next half -> 3H + 4L,  negative -> 3L + 4H.
 * For every valid zero-cross event this counts whether the two choices AGREE or DISAGREE:
 *   all AGREE    : this firmware energises the same diagonal as the old one in every half-cycle;
 *   all DISAGREE : this firmware energises the OPPOSITE diagonal (set SLAVE_CONV_SWAP only after a scope check);
 *   mixed        : the ZCD edge direction does not identify the half-cycle (e.g. one pulse per half-cycle).
 * Word for the link "diag2" field (when no trip is being reported): bit 14 valid, bits 0-6 agree, bits 7-13 disagree.
 * Both counts are halved when one reaches 127, so the ratio stays current. */
uint16_t SlaveCtrl_MapWord(void);
uint16_t SlaveCtrl_TfWord(void);            /* the transformer-connection result (0 = none yet), see CTRL_TF_* */
uint16_t SlaveCtrl_RegWord(void);           /* stage 3, in RUN: the regulator's state for diag2 (0 otherwise). bit 12 = 1 (bits 15..13 = 0), bit 11 = boost, bit 10 = AT THE CAP,
                                               bits 9..0 = the EFFECTIVE on-time in ticks / 4 (rounded; 0 = frozen / no correction). The master shows it as BOOST 462t / BUCK 310t / FREEZE. */
uint16_t SlaveCtrl_Diag2(void);             /* what the link sends in its optional diag2 field: the trip capture while in FAULT, the transformer result while
                                               the converter runs, else the passive mapping check (see the policy in slave_ctrl.c) */

/* ---- Interrupt entry points (priority 0) ------------------------------------ */
void     SlaveCtrl_OnZcEdge(uint32_t now_us, uint8_t rising);   /* PB0 edge  - CALL FIRST */
void     SlaveCtrl_OnDeadtimeIrq(void);                         /* TIM2 CC2: dead time over */
void     SlaveCtrl_OnPredictOffIrq(void);                       /* TIM2 CC3: predicted zero */
void     SlaveCtrl_OnWatchdogIrq(void);                         /* TIM2 CC4: event missing  */
void     SlaveCtrl_OnSampleTick(void);                          /* TIM3 20 kHz: polarity guard */
void     SlaveCtrl_OnBreakIrq(uint32_t power_break_flags);      /* TIM1 / TIM15 break     */
void     SlaveCtrl_OnFltIrq(void);                              /* PD2 asserting edge     */
void     SlaveCtrl_OnFltCbcIrq(void);                           /* v0.9.66: PD2 edge with the cycle-by-cycle limit (SLAVE_FLT_CBC); the PWM is already forced off */
void     SlaveCtrl_OnPwmPeriodIrq(void);                        /* v0.9.66: TIM1 update while the limit is armed */
void     SlaveCtrl_OnBreakCbcIrq(uint32_t power_break_flags);   /* v0.9.67: TIM1 / TIM15 break: limited in RUN with SLAVE_FLT_CBC, else the converter off + SlaveCtrl_OnBreakIrq */
uint32_t SlaveCtrl_CbcCount(void);                              /* v0.9.66: limited FLT edges since power-up (0 without SLAVE_FLT_CBC) */

/* Pure helper, exposed for the host tests: which timer gets the duty. */
void     SlaveCtrl_LegsFor(int8_t polarity, uint8_t boost, uint16_t duty,
                           uint16_t *leg1_tim15, uint16_t *leg2_tim1);

#ifdef __cplusplus
}
#endif

#endif /* SLAVE_CTRL_H */
