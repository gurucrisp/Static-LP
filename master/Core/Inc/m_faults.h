/**
  * @file    m_faults.h
  * @brief   Master core, part 4: the fault and restart engine. PURE LOGIC: inputs are the slave readings, the load, the keys and the clock; outputs are what to command (run level to each
  *          slave, clear-counter pulses, contactors, buzzer) and what to log and show. Called about every 100 ms. All rules come from the approved spec and decisions:
  *   Units:   Single = one unit (R). Industrial = ONE unit made of all three phases: one contactor, any phase fault trips all. Domestic = three independent units, per-phase contactors.
  *   Voltage faults (I/P LV/HV after Volt Cut Delay of continuous violation, O/P LV/HV the same while the output is live): clear 5 V back inside the limit; restart Auto after the Restart Time or Manual (START).
  *   Overload: START only. Overheat: the pin must stay low for the Overheat Trip Delay; after it has been high for the Restart Time the unit restarts by itself (Auto), or on START (Manual); 3 trips within 30 min need START.
  *   Inrush (slave FLT/break codes or the master CT peak) and Power stage (slave codes 7, 12, 14): trip at once, retry 3 times 30 s apart, then latch until power off and on (D6, D27). Every attempt is logged.
  *   Mains group (slave codes 3, 4, 5, 8, 10, 11, 13, 15, 16): trip at once, restart like the voltage faults (D6). Slave code 9 shows as I/P LV or HV (D7); code 6 as O/P HV.
  *   Transformer flip (slave code 17): latched until power off and on, buzzer sounds in EVERY mode including Domestic (D28). Re-tune required (codes 18, 19): latched until MF_RetuneDone().
  *   Slave link lost (no valid frame for 500 ms, D29): trip at once, restart like the Mains group.
  *   The contactor closes once the output has reached the set voltage and stayed there for MF_CLOSE_HOLD_MS (1 s); it used to wait the Restart Time (D27). Set-Mode open = output off (D10). Power-up: Auto waits the Restart Time (the on-delay), Manual waits for START.
  *   Buzzer: sounds on a trip (not in Domestic, except for the flip); STOP silences it.
  *   Display priority: Flip > Inrush > Power stage > Re-tune > Heat > Overload > Input V > Phase sequence/Mains > Link lost > Output V.
  */
#ifndef M_FAULTS_H
#define M_FAULTS_H
#include <stdint.h>
#include "m_settings.h"
#include "m_log.h"
#ifdef __cplusplus
extern "C" {
#endif

#define MF_CLOSE_HOLD_MS    1000UL    /* the contactor closes once the output has been IN BAND (at the set voltage) for this long: a short debounce, no longer the Restart Time */
#define MF_OP_HV_FAST_V     30U       /* an output this many volts ABOVE the O/P HV setting is cut without waiting for the Volt Cut Delay ... (30 since v0.9.26; it was 15: a variac step up passes straight through the series transformer and tripped it) */
#define MF_OP_HV_FAST_MS    200UL     /* ... once it has lasted this long: the load must not sit on a gross over-voltage for the whole delay */
#define MF_INV_OFF_MS       200UL     /* the contactor opens when the slave has not been RUNNING (state 2) for this long while the output is live: PWM off means the load must be off too */
#define MF_RETRY_MS         30000UL   /* the SHORTEST interval between inrush / power-stage retries (D27): the Restart Time is used if it is longer */
#define MF_FINAL_WAIT_MS    300000UL  /* after the retries are used up: wait this long, then ONE more attempt; if that trips again the unit latches */
#define MF_HEAT_WINDOW_MS   1800000UL /* overheat: this many trips (MF_HEAT_MAX_TRIPS) within this window need a manual START */
#define MF_HEAT_MAX_TRIPS   3U
#define MF_RETRIES          3U
#define MF_HYST_V           5U        /* a voltage fault clears this far back inside its limit */
#define MF_FORGIVE_MS       300000UL  /* the retry count is forgotten after the unit has run this long without tripping (my choice: to confirm) */

typedef struct
{
    uint8_t  link_ok;          /* a valid $S frame within the last 500 ms (the link layer decides, D29)     */
    uint16_t vin_dv, vout_dv;  /* the slave's readings, tenths of a volt                                     */
    uint16_t load_da;          /* the master CT, tenths of an amp                                            */
    uint8_t  slave_fault;      /* the slave's fault code, 0 = none                                           */
    uint8_t  in_band;          /* $E IN_BAND                                                                 */
    uint8_t  heat;             /* $E OVERHEAT (the digital limit signal)                                     */
    uint8_t  ct_peak;          /* the master CT reached its ADC full scale (inrush backup, D27)              */
    uint8_t  slave_state;      /* the slave's state from $S: 0 stopped, 1 starting / unwinding, 2 RUNNING, 3 fault */
    uint8_t  state_valid;      /* slave_state is real (the slave is being heard)                              */
} MPhaseIn_t;

typedef struct
{
    MPhaseIn_t ph[3];
    uint8_t  seq_state;        /* phase sequence: 0 ok or not checked, 1 reverse, 2 same phase               */
    uint8_t  seq_unknown;      /* 1 = the sequence has not been measured yet (v0.9.51): in Industrial mode with Phase Sequence on, a START waits for it (a running unit is not affected) */
    uint8_t  key_start, key_stop;   /* one tick each per key press                                            */
    uint8_t  setmode;          /* Set-Mode is open                                                           */
    uint32_t run_min;          /* running time in minutes, for the log                                       */
} MFIn_t;

typedef struct { MLogRec_t rec; } MFLogEvt_t;

typedef struct
{
    uint8_t  run[3];           /* the run LEVEL to send each slave in $M                                     */
    uint8_t  clr_pulse[3];     /* 1 for one tick: increment that slave's clear counter                       */
    uint8_t  contactor[3];     /* 1 = closed                                                                 */
    uint8_t  buzzer;
    uint8_t  fault[3];         /* per phase: the fault shown for it (MLOG_T_*), 0 = none                     */
    uint8_t  appear[3];        /* per phase: a limit is violated right now ("Appear"), 0 = none             */
    uint8_t  shown;            /* the highest-priority fault overall, 0 = none                               */
    uint8_t  shown_phase;
    uint8_t  unit_state[3];    /* MUState_t of each unit                                                     */
    uint8_t  n_log;
    MFLogEvt_t log[6];         /* what to append to the fault log this tick                                  */
} MFOut_t;

typedef enum { U_OFF = 0, U_ONDELAY, U_STARTING, U_ON, U_FAULT, U_LATCHED } MUState_t;
typedef enum { FC_NONE = 0, FC_VOLT, FC_START, FC_RETRY, FC_LATCH, FC_RETUNE, FC_HEAT } MFClass_t;

typedef struct
{
    uint8_t  st, ftype, fclass, fphase, fcode, retries, clr_owed, inband_run;
    uint8_t  long_wait, final_try;     /* inrush: the retries are used up and the long wait runs / the one extra attempt after it is under way */
    uint8_t  heat_manual, heat_n;      /* overheat: 3 trips within 30 min: only START restarts / how many trips so far (ring index) */
    uint16_t left_s;                   /* seconds until the next automatic restart attempt (0 = none or not counting), for the screen */
    uint32_t t_enter, t_inband, t_on;
    uint32_t t_heat_trip[3];           /* when the last overheat trips happened */
} MUnit_t;

typedef struct
{
    MUnit_t  u[3];
    uint8_t  nunits;
    uint8_t  mode;
    uint8_t  setmode_prev;
    uint8_t  ip_lv[3], ip_hv[3], op_lv[3], op_hv[3];        /* hysteresis memory: 1 = the limit is violated and has not yet cleared */
    uint32_t t_ip_lv[3], t_ip_hv[3], t_op_lv[3], t_op_hv[3], t_ovl[3], t_heat[3];   /* when the violation began */
    uint8_t  a_ovl[3], a_heat[3];
    uint8_t  hv_fast[3];                   /* the output is more than MF_OP_HV_FAST_V above the O/P HV setting          */
    uint32_t t_hv_fast[3];
    uint8_t  st_bad[3];                    /* the slave is not RUNNING while the output is live                      */
    uint32_t t_st_bad[3];                  /* since when                                                             */
    uint8_t  buzzer;
    uint8_t  inited;
    uint8_t  max_retries;      /* inrush / power-stage retries before the latch: MF_RETRIES (3, the approved rule) or 0 = latch at the FIRST trip (commissioning a repaired power stage) */
} MFaults_t;

/* power-up: the units start per the mode and restart mode */
void MF_Init(MFaults_t *f, const MSettings_t *s, uint32_t now_ms);
void MF_Tick(MFaults_t *f, const MSettings_t *s, const MFIn_t *in, uint32_t now_ms, MFOut_t *out);
uint16_t MF_RestartLeftS(const MFaults_t *f, uint8_t unit);      /* seconds until the next automatic restart attempt of this unit (inrush / power stage / overheat), 0 = none */
void MF_SetMaxRetries(MFaults_t *f, uint8_t n);   /* survives nothing: MF_Init sets it back to MF_RETRIES */
void MF_RetuneDone(MFaults_t *f);     /* the user re-tuned: clears the Re-tune latch */
uint8_t MF_Rank(uint8_t type);        /* display priority, higher = shown first */
/* the fault type (MLOG_T_*) a slave fault code stands for; code 9 needs the input voltage and the Set Voltage to decide I/P LV or HV. Used by the engine AND the screens, so they cannot disagree. */
uint8_t MF_SlaveFaultType(uint8_t code, uint16_t vin_dv, uint16_t set_v);

#ifdef __cplusplus
}
#endif
#endif
