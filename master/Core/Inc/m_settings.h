/**
  * @file    m_settings.h
  * @brief   Master core, part 2: the Set-Mode settings, their defaults, their validation ("Invalid Value") and their packed flash record. PURE LOGIC.
  *          Spec: the agreed Set-Mode list, with Max Correction (D17) and the 001-180 A Overload Current (D22). Factory reset restores THESE settings only: the calibration,
  *          the fault log and the tuning profile are separate records and stay (D20).
  */
#ifndef M_SETTINGS_H
#define M_SETTINGS_H
#include <stdint.h>
#include "m_store.h"
#ifdef __cplusplus
extern "C" {
#endif

/* the factory default Overload Current is the RATED current of the variant: a build constant, so a default unit never trips below its rated load (D22) */
#ifndef MASTER_RATED_CURRENT_A
#define MASTER_RATED_CURRENT_A   99U
#endif

/* THE LIMITS MODEL (v0.9.17). The unit works from MS_VIN_MIN_V to MS_VIN_MAX_V of input. The WINDOW is Set Voltage -/+ Max Correction, never outside 90..300 V (Max Correction 0 = not set: the whole 90..300 V).
 * I/P LV and I/P HV default to the window's edges and can be moved INSIDE the window, but never closer than MS_LIM_MARGIN_V to the Set Voltage. O/P LV and O/P HV default MS_OUT_OFFSET_V inside the input limits and can be moved
 * between the input limit and MS_LIM_MARGIN_V from the Set Voltage. So: window_lo <= I/P LV <= O/P LV <= Set - 10 and Set + 10 <= O/P HV <= I/P HV <= window_hi. */
#define MS_VIN_MIN_V       90U
#define MS_VIN_MAX_V       300U
#define MS_LIM_MARGIN_V    10U
#define MS_OUT_OFFSET_V    20U

typedef enum { M_MODE_SINGLE = 0, M_MODE_INDUSTRIAL = 1, M_MODE_DOMESTIC = 2 } MMode_t;

typedef struct
{
    uint8_t  mode;             /* MMode_t                                                          */
    uint16_t set_v;            /* volts, 100..300 (D11: the slave accepts 100.0 to 300.0 V)        */
    uint16_t maxcorr_v;        /* Max Correction, volts 0..199; 0 = "not set" (D17)                 */
    uint16_t ip_lv, ip_hv;     /* input cut-offs, volts                                            */
    uint16_t op_lv, op_hv;     /* output cut-offs, volts                                           */
    uint8_t  volt_cut_delay;   /* seconds 1..99                                                    */
    uint16_t ovl_current;      /* amps 1..180 (D22)                                                */
    uint8_t  ovl_cut_delay;    /* seconds 1..99                                                    */
    uint8_t  heat_delay;       /* Overheat Trip Delay, seconds 1..99                               */
    uint8_t  phase_seq;        /* 1 = check the phase sequence (Industrial only)                   */
    uint8_t  scroll;           /* 1 = auto-rotate the 3-phase pages                                */
    uint8_t  scroll_time;      /* seconds 1..99                                                    */
    uint16_t password;         /* 000..999                                                         */
    uint8_t  restart_auto;     /* 1 = Auto, 0 = Manual                                             */
    uint16_t restart_time;     /* seconds 1..999: also the power-up on-delay and the IN_BAND hold before the contactor closes */
    uint8_t  load_src;         /* 0 = the load current is the master CT's, 1 = the slave's converter current, converted by the slave load calibration (v0.9.45)  */
    uint16_t sl_zero_ca;       /* the slave load calibration's zero: the converter-current PEAK at no load with the unit running, hundredths of an ampere 0..4000 */
    uint16_t sl_gain;          /* its gain, thousandths: load = (peak - zero) x gain / 1000; 0 = not calibrated, else 200..5000 */
    uint8_t  heat_reset;       /* Overheat Restart: the unit may restart when the heat sink is at or below this, degC, MS_HEAT_RESET_MIN..(Overheat Temp - 5) (v0.9.50). Byte 32 of the record; 0 there means the default */
    uint8_t  heat_temp;        /* Overheat Temp: the heat-sink temperature that trips the unit, degC 50..120 (v0.9.49). Stored in byte 27 of the record; 0 there (an older record) means the default */
} MSettings_t;
#define MS_HEAT_TEMP_DEFAULT  75U
#define MS_HEAT_TEMP_MIN      50U
#define MS_HEAT_TEMP_MAX     120U
#define MS_HEAT_RESET_MIN     35U     /* Overheat Restart: the lowest the restart temperature may be set */
#define MS_HEAT_RESET_GAP      5U     /* ... and it must stay at least this far below the Overheat Temp (restart <= Overheat Temp - 5) */
uint8_t MS_HeatResetDefault(uint8_t heat_temp);
void    MS_HeatResetFollow(MSettings_t *s);               /* an Overheat Temp edit that leaves the restart above Overheat Temp - 5 pulls the restart down to the default (10 degC below) */   /* 10 degC below the Overheat Temp, never below the minimum: what an older record gets */

/* what the slave announced in $H (D7, D26) and what the master knows; used to reject cut-offs the slave would never let be reached */
typedef struct { uint16_t win_lo_v, win_hi_v; uint16_t out_guard_v; } MLimits_t;

typedef enum
{
    MS_OK = 0, MS_BAD_MODE, MS_BAD_SET_V, MS_BAD_MAXCORR, MS_BAD_IP_LV, MS_BAD_IP_HV, MS_BAD_OP_LV, MS_BAD_OP_HV, MS_BAD_VOLT_DELAY, MS_BAD_OVL_CURRENT,
    MS_BAD_OVL_DELAY, MS_BAD_HEAT_DELAY, MS_BAD_SCROLL_TIME, MS_BAD_PASSWORD, MS_BAD_RESTART_TIME, MS_BAD_HEAT_TEMP, MS_BAD_HEAT_RESET, MS_BAD_FLAG
} MSErr_t;

#define MS_PACK_LEN  48U              /* v0.9.50: 48 bytes (it was 32): bytes 0 to 31 are laid out exactly as before, byte 32 is the Overheat Restart */
#define MS_PACK_LEN_V1  32U           /* the record of firmware up to v0.9.49: MS_Load reads it and carries the settings over */
#define MS_VERSION_V1   1U
#define MS_VERSION   2U       /* the record version: MStore_Init(&st, fl, page_a, page_b, MS_VERSION, MS_PACK_LEN) */

void     MS_Defaults(MSettings_t *s);
uint16_t MS_WinLo(const MSettings_t *s);      /* the lowest input limit allowed: Set - Max Correction, not below 90 V */
uint16_t MS_WinHi(const MSettings_t *s);      /* the highest input limit allowed: Set + Max Correction, not above 300 V */
/* the four defaults for this Set Voltage and Max Correction; with lim (the slave's $H) they are kept inside what the slave announced: I/P inside its window, O/P HV not above its output guard. lim may be NULL. */
void     MS_DefaultLimits(const MSettings_t *s, const MLimits_t *lim, uint16_t *ip_lv, uint16_t *ip_hv, uint16_t *op_lv, uint16_t *op_hv);
/* after Set Voltage or Max Correction was changed (s = the new settings, old = before): a limit that was at the OLD default follows to the NEW default, any other (a limit the user moved) is kept, clipped into the new window */
void     MS_Rederive(MSettings_t *s, const MSettings_t *old, const MLimits_t *lim);
void     MS_ApplyLim(MSettings_t *s, const MLimits_t *lim);     /* put the four limits inside the slave's announced window and output guard (used after a Factory Reset) */
/* checks every range and the cross rules: I/P LV < Set V < I/P HV, O/P LV < Set V < O/P HV, I/P cut-offs inside the slave window, O/P HV not above the slave's output guard.
 * lim may be NULL (the slave has not announced yet): then only the plain ranges and the cross rules are checked. Returns MS_OK or the FIRST offending item. */
MSErr_t  MS_Validate(const MSettings_t *s, const MLimits_t *lim);
void     MS_Pack(const MSettings_t *s, uint8_t out[MS_PACK_LEN]);
void     MS_Unpack(MSettings_t *s, const uint8_t in[MS_PACK_LEN]);

/* storage: Load returns 1 if the stored record was valid AND passes MS_Validate(NULL); otherwise the defaults are put in *s and 0 is returned. */
uint8_t  MS_Load(MStore_t *st, MSettings_t *s);
int      MS_Save(MStore_t *st, const MSettings_t *s);

#ifdef __cplusplus
}
#endif
#endif
