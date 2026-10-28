# The over-current protection path (decision D25, v0.8.4)

## The path, from the schematic you sent

CT2500 (2500 turns) on the series transformer's primary -> shunt (68 ohm x 3 in parallel) -> window comparators (two MCP6567, around VCC/2, FIXED resistor-divider thresholds,
open-drain outputs wired together) -> **FLT pin PD2** (internal pull-up, asserts LOW) -> **EXTI line 2** -> `EXTI2_3_IRQHandler` -> `SlavePower_AllOffNow()`.

FLT is a *software* input, not the timer's hardware break (BKIN). So the time from the comparator trip to the gate drivers going low is the interrupt latency plus whatever the
handler does first. The CPU has NO analog primary-current signal (you confirmed the slave current is a digital output), so this trip is the only over-current protection the
CPU sees.

## What v0.8.4 changes

1. **The shutdown is the first statement of `EXTI2_3_IRQHandler`**, before the HAL's EXTI handler and before any bookkeeping. `SlaveCtrl_OnFltIrq()` also starts with it (the PC tests call that
   function directly; on the target the second call is harmless). The counters, the trip record (`CaptureTrip`: it reads only a timer count and two controller fields, nothing the
   output stage touches) and the fault flag follow, unchanged.
2. **FLT is the only interrupt at the top priority** (`slave_irq_prio.h`): FLT 0; control timer, zero-cross, both break handlers 1; measurement timer 2. Before, five interrupts shared
   level 0, so a FLT edge arriving while a zero-cross or control-timer interrupt was running had to wait for it to finish.
3. **A bench build to measure it with NO MAINS**: `slave_config_SCOPE_flt_latency_NO_MAINS.h` (stage 0, the 50 % debug PWM, `BRINGUP_FLT_TEST 1`). A FLT edge now switches the debug PWM off.
   `BRINGUP_FLT_TEST` is rejected by `#error` with any real power stage. Found while building it: the FLT handler did not exist at all in a stage-0 build, so arming the
   interrupt there would have landed in the default handler (an endless loop). It is now defined whenever FLT is armed.

## Enforced by the test runner, not by good intentions

* `tests/test_ctrl.c`, group "FLT: the shutdown comes first": a probe in the harness stub records the controller's counters and trip record at the instant the shutdown is called; the
  shutdown must come before any of them is touched, and every one of them must still happen afterwards. It fails 3 checks if the old order is restored.
* `tools/check_irq.py`: every `HAL_NVIC_SetPriority` uses an `IRQ_PRIO_*` name (no bare number), `IRQ_PRIO_FLT` belongs to EXTI2_3 alone, the levels are strictly ordered, and the first statement
  of both FLT functions is `SlavePower_AllOffNow();`. Checked against five deliberate breakages.
* `slave_irq_prio.h` has `_Static_assert`s for the ordering at compile time.

## What is an ESTIMATE and must be MEASURED

Nothing about the TIMING can be shown on a PC. The workbook `igbt_selection_30kva.xlsx` (sheet SlaveCT) uses these figures, all placeholders until you measure:

| | before | after | basis |
|---|---|---|---|
| comparator and filter | 2.0 us | 2.0 us | assumption: the input filters (1 nF with 0.7 to 2.2 kohm) |
| firmware, FLT edge to outputs off | about 3.5 us (plus any priority-0 interrupt already running) | about 0.6 us | my estimate from the code at 64 MHz: NOT measured, there is no ARM compiler here |
| gate driver and IGBT turn-off | 1.0 us | 1.0 us | assumption |
| primary current rise at a hard short | 5 A/us | 5 A/us | PLACEHOLDER: measure it |

### Measuring the firmware part, no mains, about 15 minutes

1. Flash `slave_config_SCOPE_flt_latency_NO_MAINS.h` (stage 0, debug PWM, gate supply only). The PWM runs at 20 kHz, 50 %.
2. Scope channel 1 on PD2 (FLT). Channel 2 on a gate-driver input (the PWM pin) or on the gate itself.
3. Drive PD2 from a function generator, 0 to 3.3 V, a single falling edge (or briefly short PD2 to ground through a button with the scope triggered on it).
4. Read the time from the FLT edge at the pin to the gate-driver input going low. Do it with v0.8.3 and with v0.8.4, a few dozen times each, and note the worst one.
   The firmware part should be around 1 us or less with v0.8.4; the PWM edge that is in progress when FLT arrives adds up to one switching period of 50 us in the worst case for a
   gate that was already high, so trigger the scope on the FLT edge and look at the first gate edge that follows, not at the PWM period.
5. After a trip only a RESET restarts the PWM.

Then measure the primary current rise (A/us) in a deliberate overload test and replace the placeholders in the workbook.

## Incident report: inverter IGBTs lost during a fast variac test (v0.9.9, the no-voltage-cuts build)
What was running: the master-control slave, probably the TEST_NOCUTS config (no input-range trip, no output-high trip) with the master's own cuts also off, and an input turned back and forth quickly 5 or 6 times.
**What I found by simulation (not proof of the cause):** in the stage-3 run state the boost/buck direction came straight from `Reg_IsBoost(vin)` and was applied in the same pass as the input crossed the set point, together with whatever correction the regulator held.
A snap from 205 V to 258 V reversed the injection at 768 counts of duty (the cap) in a single step; eight snaps gave eight such reversals. In addition, the output (RMS) measurement is stale for a cycle or two after a step, and the integral regulator wound up to the cap on it within 20 ms and injected 21 V the wrong way.
There was no input-step detection at all; the only input protection was the range trip, which that build had removed.
**What v0.9.11 changes (SLAVE_INJ_GUARD, on by default; `slave_features.h`):** a step of the input of 15 V or more between two RMS results, or a wish to reverse the direction while the correction is above 96 ticks, takes the correction to ZERO at once and holds it for 100 ms after the input is steady;
then the regulator restarts from zero through its soft-start ramp. In the simulation, eight snaps of 205/258 V every 150 ms now give no reversal at a large correction, the output follows the input (no correction stacked on it), the unit stays in RUN and returns to 230 V when the variac stops.
With `SLAVE_INJ_GUARD 0` the same test fails all eight reversals. The master's voltage cuts are ON again by default. **This is a mitigation of one cause I could reproduce. It is not a proof that this destroyed the IGBTs.**
**Not yet known, and the reason to measure:** the FLT shutdown latency (`slave_config_SCOPE_flt_latency_NO_MAINS.h`, procedure above) and the primary current at a fault have never been measured on your hardware; an IGBT with an 8 us short-circuit withstand is lost if the shutdown is slower than that. The turn-off of an inductive current
(the shutdown at a high primary current) is the other thing a scope should show. **Before the next test:** do not use the TEST_NOCUTS files; do not swing the input quickly until the FLT latency is measured; and on the repaired unit check the gate drivers, the gate resistors and the gate-emitter protection of BOTH devices of the leg that failed, not only the one that looks burnt.
**What would tell me the cause:** the master's fault log (LOG key) and the last fault code on the slave's display before it failed; which config was flashed; the input range of the swing and how fast; which IGBTs failed (one leg, both legs, high side or low side); and whether the FLT / over-current indication latched.


## v0.9.62: the FLT glitch filter (`SLAVE_FLT_FILTER_US`, 1 us in the production config)
`EXTI2_3_IRQHandler` -> `SlaveZC_FltIrqFiltered()`: wait the window on SysTick (iteration cap), clear the EXTI flags, read FLT twice; asserted -> `SlavePower_AllOffNow()` then `SlaveCtrl_OnFltIrq()`; released -> ignored (more than 4 in one millisecond -> trip). 0 = the plain path. Above 2 us refused. It adds the window to the shutdown time of a real short circuit: measure the total with the SCOPE config above. Reason: F02 from noise (a nearby load switched) with 1k / 1 nF on FLT.

v0.9.63: the main loop's FLT LEVEL check (`RunningChecks`) is filtered too when the filter is on: FLT asserted on consecutive passes for `CTRL_FLT_POLL_CONFIRM_US` (100 us; readings more than 10 ms apart do not add up). v0.9.62 had missed it: one noisy reading there was still F02.


## v0.9.65: the FLT response keeps the low sides on (`SLAVE_FLT_KEEP_LOWS`, 1 by default)
The first statement of the FLT interrupt is `SlavePower_FltOffNow()`: TIM1 and TIM15 CCMR1 OC1M = 0100 (forced inactive): the high sides off at once, the low sides on after the dead time, so the primary stays shorted; then the converter off. The controller unwinds (shunts closed, then the inverter disabled), reporting F02 at once; `SlavePower_InverterEnable()` restores PWM mode 1. Before, everything went off at once and the primary was open under load for the relays' travel time. BKIN is unchanged (silicon all-off).


## v0.9.66: the cycle-by-cycle limit (`SLAVE_FLT_CBC`, 0 by default)
EXTI2_3 -> `SlaveZC_FltIrqCbc()`: first `SlavePower_InverterForceOffNow()` (TIM1/TIM15 forced inactive, the converter untouched), then `SlaveCtrl_OnFltCbcIrq()`: in RUN a limit (TIM1 update armed), else the full FLT shutdown. TIM1 update -> `SlaveZC_Tim1UpdateIrq()` -> `SlaveCtrl_OnPwmPeriodIrq()`: FLT released = PWM mode 1 again; asserted 20 period starts (1 ms) or more than 40 edges in 10 ms = the full shutdown (F02, low sides on). The main loop's level check waits 2 ms. Not with the glitch filter.
