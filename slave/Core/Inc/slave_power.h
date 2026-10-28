/**
  ******************************************************************************
  * @file    slave_power.h
  * @brief   Power-stage hardware layer for the slave: inverter PWM timers,
  *          converter (zero-cross commutated) GPIO diagonals, ISR-safe
  *          "everything off", break-flag readout.
  *
  * POWER BOARD (from schematic_of_static.pdf, page 1)
  *
  *   Four half-bridges of FGL60N100BNTD IGBTs on a DC link (V+ / PE):
  *
  *      leg 1 = PWM1 H/L  (TIM15 CH1/CH1N, PB14/PB13)  mid-point  PRI-PHASE
  *      leg 2 = PWM2 H/L  (TIM1  CH1/CH1N, PA8 /PA7 )  mid-point  PRI-NTRL
  *                 -> legs 1+2 drive the series-injection transformer primary
  *                    ("inverter", 20 kHz PWM, hardware dead time, hardware BKIN)
  *
  *      leg 3 = PWM3 H/L  (PB3 /PC4 , plain GPIO)      mid-point  PHASE-OUT
  *      leg 4 = PWM4 H/L  (PC5 /PB5 , plain GPIO)      mid-point  NTRL
  *                 -> legs 3+4 are the "converter": it connects the line to the
  *                    DC link and is commutated ONCE PER HALF-CYCLE.
  *
  *   Converter diagonals (derived from the schematic and checked against the
  *   old firmware's pin writes):
  *
  *      positive half (PHASE-OUT > NTRL):  3H (PB3) + 4L (PB5)  ON,  3L, 4H OFF
  *      negative half (PHASE-OUT < NTRL):  3L (PC4) + 4H (PC5)  ON,  3H, 4L OFF
  *
  *   Turning on the WRONG diagonal for the present line polarity puts the DC
  *   link across the mains. That is why the sequencer (slave_ctrl.c) takes the
  *   polarity from the measured waveform and why EVERY function here that
  *   energises a diagonal first switches the opposite one off.
  *
  *   The converter pins have NO hardware break. SlavePower_AllOffNow() is the
  *   one routine that kills everything; it is safe to call from any interrupt.
  ******************************************************************************
  */
#ifndef SLAVE_POWER_H
#define SLAVE_POWER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32g0xx_hal.h"

/* ---- Inverter timing (unchanged from the board-verified bring-up) --------- */
#define POWER_TIMER_CLK_HZ     64000000UL
#define POWER_SWITCHING_HZ     20000UL
#define POWER_PERIOD_TICKS     (POWER_TIMER_CLK_HZ / POWER_SWITCHING_HZ)   /* 3200 -> ARR = 3199 */

/* Raw BDTR.DTG (timer clock 64 MHz): 64 -> 1.0 us, 128 -> 2.0 us (DTG 128..191 = (64 + DTG[5:0]) x 2 / 64 MHz).
 * The OLD firmware runs the inverter with 2.0 us (TIM1_TIM15_UpdateParameters(0, 128) at start-up), proven on these IGBTs; this firmware used 1.0 us until v0.6.15.
 * SCOPE-VERIFY at the IGBT gate-driver inputs with the DC link UNPOWERED (BRINGUP_PWM_MODE 1, see docs/POWER_STAGE_BRINGUP.md). */
#ifndef POWER_DEADTIME_DTG
#define POWER_DEADTIME_DTG     128U
#endif

/* The same dead time in timer TICKS (the PWM counter and the dead-time generator both run at POWER_TIMER_CLK_HZ; period = POWER_PERIOD_TICKS).
 * A high-side pulse SHORTER than the dead time is NOT GENERATED AT ALL (RM0444, dead-time insertion): with CCR <= POWER_DEADTIME_TICKS a leg outputs NOTHING,
 * and the real high-side on-time is  CCR - POWER_DEADTIME_TICKS. (v0.7.0 asked for 32 / 64 / 96 ticks against a 128-tick dead time: no output at all.) */
#if (POWER_DEADTIME_DTG < 128U)
#define POWER_DEADTIME_TICKS   ((uint16_t)(POWER_DEADTIME_DTG))
#elif (POWER_DEADTIME_DTG < 192U)
#define POWER_DEADTIME_TICKS   ((uint16_t)((64U + (POWER_DEADTIME_DTG & 63U)) * 2U))
#else
#error "POWER_DEADTIME_DTG above 191 is not supported (the tick conversion is only defined up to 191)"
#endif

/* Break-input digital filter (TIMx_BDTR.BKF, 0..15). The old firmware uses 4: a break pulse must last 6 samples at fDTS/2 (about 0.19 us at 64 MHz). */
#ifndef POWER_BREAK_FILTER
#define POWER_BREAK_FILTER     4U
#endif

/* Bits returned by SlavePower_TakeBreakFlags() */
#define POWER_BREAK_TIM1       0x01U
#define POWER_BREAK_TIM15      0x02U

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim15;

/* Configures TIM1 + TIM15 (zero duty, outputs disabled, break armed, pins to
 * their alternate functions). Used by the control firmware AND by the 50 %
 * bring-up test, so there is only one copy of this verified setup. */
void SlavePower_TimersInit(void);

/* Everything the control firmware needs: timers, break interrupts, converter
 * pins low, FLT (PD2) falling-edge interrupt. All power outputs stay OFF.
 * The four INVERTER pins stay PLAIN GPIO OUTPUTS DRIVEN LOW (not handed to the
 * timers) until SlavePower_InverterEnable(), and return to that state in
 * SlavePower_InverterDisable() / SlavePower_AllOffNow(). */
void SlavePower_Init(void);

/* Arms ONLY the FLT interrupt (PD2, EXTI line 2, priority IRQ_PRIO_FLT). SlavePower_Init() calls it. The stage-0 bench build (BRINGUP_FLT_TEST) calls it on its own,
 * so a FLT edge switches the debug PWM off and the delay can be measured with NO MAINS. */
void SlavePower_FltInit(void);

/* ISR-SAFE. Converter pins low, inverter duty 0, inverter outputs disabled
 * (MOE off -> all four inverter gate inputs low). Idempotent. */
void SlavePower_AllOffNow(void);

/* ISR-SAFE. v0.9.65, the FLT response (the FIRST statement of the FLT interrupt). SLAVE_FLT_KEEP_LOWS 1: both inverter channels FORCED INACTIVE (high sides off at
 * once, low sides on after the dead time: the primary stays shorted), then the converter off; the inverter stays enabled until the shunts have closed and
 * SlavePower_InverterEnable() restores the PWM mode at the next start. SLAVE_FLT_KEEP_LOWS 0: SlavePower_AllOffNow(). */
void SlavePower_FltOffNow(void);

/* ISR-SAFE. Both converter diagonals off (all four converter pins low). */
void SlavePower_ConverterOffNow(void);

/* ISR-SAFE. Switch the opposite diagonal off first, then the requested one on. */
void SlavePower_ConverterPositive(void);   /* 3H (PB3) + 4L (PB5) */
void SlavePower_ConverterNegative(void);   /* 3L (PC4) + 4H (PC5) */

/* Main loop. Duty to zero, counters reset, shadow registers loaded, outputs
 * enabled; only after MOE is really set are the four pins handed to the timers.
 * Returns 1 if both timers came out of break, 0 if a break input is still
 * asserted (the pins then stay low). */
uint8_t SlavePower_InverterEnable(void);

/* ISR-SAFE. Both legs' duty to 0 and outputs disabled. */
void SlavePower_InverterDisable(void);

/* ISR-SAFE. Duty of leg 1 (TIM15) and leg 2 (TIM1) in timer counts. Values
 * above ARR are clamped. A leg at 0 duty keeps its LOW side on (bootstrap
 * charging) - same as the old firmware. */
void SlavePower_SetLegDuty(uint16_t leg1_tim15, uint16_t leg2_tim1);

/* ISR-SAFE. Restart both PWM counters (old firmware did this at every
 * commutation so the carrier restarts with the half-cycle). */
void SlavePower_SyncCounters(void);

uint8_t  SlavePower_InverterIsEnabled(void);

/* ISR-SAFE. Returns the break flags seen (POWER_BREAK_*) and clears them.
 * It also MASKS the break interrupt of every timer that fired: while a BKIN
 * input is held low its flag sets itself again as fast as it is cleared, so an
 * unmasked priority-0 interrupt would re-enter for ever and starve the main
 * loop. The main loop re-enables it with SlavePower_BreakIrqRearm(). */
uint32_t SlavePower_TakeBreakFlags(void);

/* Main loop: re-enable the break interrupts, but only while BOTH break inputs
 * are released (otherwise it stays masked). Cheap; call it every pass. */
void     SlavePower_BreakIrqRearm(void);

/* 1 while TIM1_BKIN (PB12) or TIM15_BKIN (PA9) is held at its active (low) level. */
uint8_t  SlavePower_BreakInputActive(void);

/* Relay coils (PC9 = SH-RLY, PC8 = W-RLY): 1 = coil energised. Main loop and the
 * relay close interrupt only; sequencing lives in slave_relay.c. */
void    SlavePower_RelaySh(uint8_t on);
void    SlavePower_RelayW(uint8_t on);

/* FLT pin (PD2, active LOW = fault). 1 = fault asserted. */
uint8_t SlavePower_FltAsserted(void);

#ifdef __cplusplus
}
#endif

#endif /* SLAVE_POWER_H */
