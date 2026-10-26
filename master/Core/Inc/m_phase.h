/**
  ******************************************************************************
  * @file    m_phase.h
  * @brief   The phase sequence and the angles between the phases, from what the three slaves measured (v0.9.51). No hardware, no maths library.
  *
  * A round: the master gave the three slaves one common instant (SYNC byte); each measured the time to its next RISING zero-cross (delay) and its mains period T.
  * Phase Y lags R by T/3 and B by 2T/3, so (delay_Y - delay_R) mod T = T/3 and (delay_B - delay_R) mod T = 2T/3 for the right sequence R-Y-B:
  *   OK        both within 10 % of a period (36 degrees) of 1/3 and 2/3
  *   REVERSE   both within 10 % of 2/3 and 1/3 (R-B-Y)
  *   SAME      any two phases within 10 % of a period of each other (the same phase twice, or a phase missing and replaced by another)
  *   otherwise the round says nothing (an unbalanced or noisy measurement).
  * The decision changes only after MPH_CONFIRM identical rounds in a row; no classifying round for MPH_STALE_MS makes it UNKNOWN again.
  * With the sequence OK or REVERSE the angles are known and the phase-to-phase voltage is V = sqrt(Va^2 + Vb^2 - 2 Va Vb cos(theta)).
  ******************************************************************************
  */
#ifndef M_PHASE_H
#define M_PHASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MPH_UNKNOWN   255U
#define MPH_OK        0U       /* the values the engine takes for seq_state */
#define MPH_REVERSE   1U
#define MPH_SAME      2U
#define MPH_CONFIRM   3U       /* identical rounds in a row to change the decision */
#define MPH_TOL_PM    100U     /* the tolerance, per mille of a period (100 = 36 degrees) */
#define MPH_STALE_MS  20000UL  /* no classifying round for this long: unknown again */
#define MPH_NONE_US   65535U   /* a delay of LINKV2_SQ_NONE: the slave's round failed */

typedef struct
{
    uint8_t  valid[3];         /* phase R, Y, B answered this round with a delay (not LINKV2_SQ_NONE) */
    uint16_t delay_us[3];
    uint16_t period_us[3];     /* 0 = not known */
} MPhRound_t;

typedef struct
{
    uint8_t  state;            /* MPH_UNKNOWN until MPH_CONFIRM identical rounds, then OK / REVERSE / SAME */
    uint8_t  cand, cand_n;     /* the class of the last rounds and how many in a row */
    uint8_t  have_ang, ang_cls;   /* the angles are known (the sequence is OK or REVERSE) / the class they belong to */
    uint16_t ang_dd[3];        /* each phase's angle behind R, tenths of a degree, 0..3599 (R = 0) */
    uint32_t t_last_ms;        /* the last round that classified */
} MPhase_t;

void     MPhase_Init(MPhase_t *p);
/* one round: MPH_OK / MPH_REVERSE / MPH_SAME, or MPH_UNKNOWN (missing data, or says nothing). fy_pm and fb_pm (if not NULL) = Y's and B's delay behind R as per mille of the period. */
uint8_t  MPhase_Classify(const MPhRound_t *r, uint16_t *fy_pm, uint16_t *fb_pm);
void     MPhase_Round(MPhase_t *p, const MPhRound_t *r, uint32_t now_ms);
void     MPhase_Age(MPhase_t *p, uint32_t now_ms);
void     MPhase_Lost(MPhase_t *p);                      /* a slave is not heard: unknown at once */
/* the phase-to-phase voltage between phases a and b (0 R, 1 Y, 2 B) from their voltages, tenths of a volt; 0 = not known (no angles, or a voltage of 0) */
uint16_t MPhase_LineDv(const MPhase_t *p, uint8_t a, uint8_t b, uint16_t va_dv, uint16_t vb_dv);
int32_t  MPhase_CosQ15(uint32_t deg_dd);                /* cos of an angle in tenths of a degree, Q15 (exported for the tests) */

#ifdef __cplusplus
}
#endif
#endif /* M_PHASE_H */
