/**
  ******************************************************************************
  * @file    slave_ctrl_hw.h
  * @brief   The few hardware services slave_ctrl.c needs, behind a tiny
  *          interface. On the target they are implemented in slave_ctrl_hw.c
  *          (TIM2 compares + the ADC ring); the host tests provide scripted
  *          versions, so the sequencing logic itself is tested unmodified.
  ******************************************************************************
  */
#ifndef SLAVE_CTRL_HW_H
#define SLAVE_CTRL_HW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float    vin;             /* calibrated AC_IN  RMS, volts             */
    float    vout;            /* calibrated AC_OUT RMS, volts             */
    uint32_t update_count;    /* increments with every new RMS result      */
    uint8_t  valid;           /* 1 once a first RMS result exists          */
} CtrlMeas_t;

/* One-time hardware setup (TIM2 compare interrupts, NVIC). */
void     CtrlHw_Init(void);

/* Free-running microsecond clock (32-bit, wraps) and millisecond tick. */
uint32_t CtrlHw_NowUs(void);
uint32_t CtrlHw_NowMs(void);

/* Arm / disarm the three one-shot compare interrupts, at an ABSOLUTE microsecond
 * time. If that time has already passed when armed, the interrupt must fire
 * immediately (never wait for the 32-bit timer to wrap).
 *   dead time   : -> SlaveCtrl_OnDeadtimeIrq()    (TIM2 CC2)
 *   predict-off : -> SlaveCtrl_OnPredictOffIrq()  (TIM2 CC3)
 *   watchdog    : -> SlaveCtrl_OnWatchdogIrq()    (TIM2 CC4)                  */
void     CtrlHw_ArmDeadtime(uint32_t at_us);
void     CtrlHw_DisarmDeadtime(void);
void     CtrlHw_ArmPredictOff(uint32_t at_us);
void     CtrlHw_DisarmPredictOff(void);
void     CtrlHw_ArmWatchdog(uint32_t at_us);
void     CtrlHw_DisarmWatchdog(void);

/* ISR-SAFE, integer only. Sign of the mains just before "now" for AC_IN
 * (out_channel = 0) or AC_OUT (1): +1, -1, or 0 = too small / no data.
 * dev_sum receives the signed deviation sum (see slave_meas.h). */
int8_t   CtrlHw_Sign(uint8_t out_channel, int32_t *dev_sum);

/* ISR-SAFE, integer only. Sign of AC_IN from the NEWEST two ADC frames, with a
 * larger dead band than CtrlHw_Sign (used by the 20 kHz polarity guard). */
int8_t   CtrlHw_SignLatest(uint8_t strong);     /* strong = 1: wide dead band (~30 V) instead of ~10 V */

/* ISR-SAFE, integer only. v0.9.69: the same newest two AC_IN frames with a dead band of min_counts per sample (the voltage gate, CTRL_VGATE_COUNTS). */
int8_t   CtrlHw_SignAbove(int32_t min_counts);

/* Snapshot of the latest RMS results. */
void     CtrlHw_GetMeas(CtrlMeas_t *m);

/* Critical section around state shared with the interrupts. */
uint32_t CtrlHw_Lock(void);
void     CtrlHw_Unlock(uint32_t previous);

#ifdef __cplusplus
}
#endif

#endif /* SLAVE_CTRL_HW_H */
