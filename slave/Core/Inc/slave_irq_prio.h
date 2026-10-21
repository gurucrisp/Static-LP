/* Interrupt priorities of the slave: ONE place, with a compile-time check (decision D25).
 *
 * The CT over-current comparators reach the CPU on the FLT pin (PD2, EXTI line 2): a SOFTWARE input, not the timer's hardware break. So the time from a FLT edge to the
 * gate drivers going low is the interrupt latency plus the handler, and nothing may be allowed to delay it. On this Cortex-M0+ a lower number is a higher priority and an
 * interrupt can pre-empt only one of STRICTLY lower priority (a higher number): FLT is therefore the ONLY interrupt at the top level.
 *
 *   level 0  FLT (EXTI2_3)                      the over-current trip: outputs off as the very first instruction of its handler
 *   level 1  control timer (TIM2), zero-cross (EXTI0_1), break handlers (TIM1_BRK, TIM15)   timing-critical but not protection
 *   level 2  measurement timer (TIM3_TIM4)
 *
 * tools/check_irq.py (part of the test runner) fails if any HAL_NVIC_SetPriority call uses a number instead of these names. */
#ifndef SLAVE_IRQ_PRIO_H
#define SLAVE_IRQ_PRIO_H

#define IRQ_PRIO_FLT     0U
#define IRQ_PRIO_TIMING  1U
#define IRQ_PRIO_MEAS    2U
#define IRQ_PRIO_LINK    3U      /* the link receive interrupt (USART1, step 3): the LOWEST level, so it can never delay the protection path or the timing */

_Static_assert(IRQ_PRIO_FLT < IRQ_PRIO_TIMING, "FLT must be strictly above every other interrupt");
_Static_assert(IRQ_PRIO_TIMING < IRQ_PRIO_MEAS, "the timing interrupts stay above the measurement one");
_Static_assert(IRQ_PRIO_MEAS < IRQ_PRIO_LINK, "the link receive interrupt stays below the measurement one");
_Static_assert(IRQ_PRIO_LINK <= 3U, "the Cortex-M0+ has only four priority levels");

#endif
