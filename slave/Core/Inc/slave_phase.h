/**
  ******************************************************************************
  * @file    slave_phase.h
  * @brief   The slave's half of the phase sequence measurement (v0.9.51). No hardware.
  *
  * The three slaves have no common clock, so the master gives them one instant: it sends $K SEQ <round> (ARM), then one SYNC byte (LINKV2_SYNC_BYTE) on all three links at
  * the same moment. This module measures, on THIS slave, the time from that SYNC byte to its own next RISING zero-cross: delay_us, together with its own mains period.
  * The master takes the three delays and turns their differences into the angles between the phases (m_phase.c).
  *
  * Who calls what:  SlavePhase_Arm      main loop     the master's $K SEQ arrived
  *                  SlavePhase_OnSync   RX interrupt  the SYNC byte was received (time-stamped there, with the 1 microsecond timer of the zero-cross interrupt)
  *                  SlavePhase_OnZc     ZC interrupt  every zero-cross edge, with the same timer
  *                  SlavePhase_Service  main loop     timeouts
  *                  SlavePhase_Result   main loop     the last result, for the $E frame (sq_id, sq_delay_us, sq_period_us)
  * The interrupts only ever move the state forward; the two main-loop calls take the lock the glue gives to SlavePhase_Init (NULL on the PC).
  ******************************************************************************
  */
#ifndef SLAVE_PHASE_H
#define SLAVE_PHASE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define SPH_ARM_WINDOW_MS     100U       /* a SYNC byte is taken for real only this long after the $K SEQ                               */
#define SPH_FAIL_AFTER_MS     300U       /* no rising zero-cross this long after the $K SEQ: the round is reported as failed             */
#define SPH_MAX_DELAY_US      45000U     /* the longest sync-to-zero-cross time that can be real (a bit over one period at 22 Hz)        */
#define SPH_PERIOD_MIN_US     12000U     /* a mains period outside 12 ms to 45 ms (83 Hz to 22 Hz) is not trusted                       */
#define SPH_PERIOD_MAX_US     45000U
#define SPH_NONE              65535U     /* = LINKV2_SQ_NONE: the round failed                                                           */

void SlavePhase_Init(void (*lock)(void), void (*unlock)(void));
void SlavePhase_Arm(uint8_t round, uint32_t now_ms);
void SlavePhase_OnSync(uint32_t now_us);
void SlavePhase_OnZc(uint32_t now_us, uint8_t rising);
void SlavePhase_Service(uint32_t now_ms);
/* id 0 = no round has finished yet; delay SPH_NONE = it failed; period 0 = not known yet */
void SlavePhase_Result(uint16_t *id, uint16_t *delay_us, uint16_t *period_us);

#ifdef __cplusplus
}
#endif
#endif /* SLAVE_PHASE_H */
