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
} MSettings_t;

/* what the slave announced in $H (D7, D26) and what the master knows; used to reject cut-offs the slave would never let be reached */
typedef struct { uint16_t win_lo_v, win_hi_v; uint16_t out_guard_v; } MLimits_t;

typedef enum
{
    MS_OK = 0, MS_BAD_MODE, MS_BAD_SET_V, MS_BAD_MAXCORR, MS_BAD_IP_LV, MS_BAD_IP_HV, MS_BAD_OP_LV, MS_BAD_OP_HV, MS_BAD_VOLT_DELAY, MS_BAD_OVL_CURRENT,
    MS_BAD_OVL_DELAY, MS_BAD_HEAT_DELAY, MS_BAD_SCROLL_TIME, MS_BAD_PASSWORD, MS_BAD_RESTART_TIME, MS_BAD_FLAG
} MSErr_t;

#define MS_PACK_LEN  32U
#define MS_VERSION   1U       /* the record version: MStore_Init(&st, fl, page_a, page_b, MS_VERSION, MS_PACK_LEN) */

void     MS_Defaults(MSettings_t *s);
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
