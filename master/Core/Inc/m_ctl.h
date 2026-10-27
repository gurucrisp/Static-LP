/**
  * @file    m_ctl.h
  * @brief   Master core, part 8: what the master tells the slave. PURE LOGIC: the keys, Set-Mode, the settings and the slave's reported state in; the content of the $M control frame out.
  *          The slave (built with SLAVE_MASTER_CONTROL 1) does exactly what the latest $M says, and stops by itself if no valid $M arrives for 1 s.
  *
  *   run      a LEVEL. Set-Mode open (or its password prompt) = 0 at once: the slave stops its PWM and trips the output (D10). STOP = 0 until START. Power-up and leaving Set-Mode:
  *            Auto = 1 after the Restart Time (the on-delay), Manual = 0 until START.
  *   setv     the Set Voltage setting, tenths of a volt. The slave takes it while it is stopped, so a change made in Set-Mode is in force at the next start.
  *   maxc     the Max Correction setting, tenths of a volt (0 = not set).
  *   clr      the clear counter. The slave latches every fault until it changes. Auto: while the slave reports a fault, the counter is changed every Restart Time (the slave then tries again);
  *            Manual: the unit is stopped at the fault and START changes the counter.
  */
#ifndef M_CTL_H
#define M_CTL_H
#include <stdint.h>
#include "m_settings.h"
#ifdef __cplusplus
extern "C" {
#endif

#define MCTL_SETTLE_MS  1500UL     /* after the counter changes the slave still reports its OLD fault for a moment: it is ignored this long */

typedef struct
{
    uint8_t  user_on;           /* the unit is wanted ON */
    uint8_t  waiting;           /* the on-delay (Auto) is running */
    uint8_t  first;             /* power-up not yet handled */
    uint8_t  setmode_prev;
    uint8_t  fault_seen;
    uint8_t  clr;
    uint32_t t_ref, t_fault, t_clr;     /* t_clr: when the clear counter last changed */
} MCtl_t;

typedef struct
{
    uint8_t  run;
    uint8_t  clr;
    uint16_t setv_dv;
    uint16_t maxc_dv;
    uint8_t  hold;      /* v0.9.61: $M hold, the duty frozen while the contactor connects the load (m_faults.c MF_HOLD_*) */
} MCtlOut_t;

void MCtl_Init(MCtl_t *c);
/* slave_fault: 1 if the slave reports a fault (state 3 or a fault code) */
void MCtl_Step(MCtl_t *c, const MSettings_t *s, uint8_t setmode, uint8_t key_start, uint8_t key_stop, uint8_t link_ok, uint8_t slave_fault, uint32_t now_ms, MCtlOut_t *out);

#ifdef __cplusplus
}
#endif
#endif
