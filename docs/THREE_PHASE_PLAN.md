# Three-phase: the hardware map, what exists, what is missing, and the order of work

## 1. The three slave links on the master (checked against the code and the STM32G070 datasheet)
| Phase | USART | Master TX pin | Master RX pin | AF | DMA RX / TX (DMA1) | Baud now | To do |
|---|---|---|---|---|---|---|---|
| R | USART1 | PC4 | PC5 | AF1 | ch1 / ch2 | 500000 (LINK_BAUDRATE) | works today |
| Y | USART4 | PC10 | PC11 | AF1 | ch7 / ch3 | 500000 since v0.9.44 | the RX ring and the transmitter |
| B | USART3 | PB2 | PB0 | AF4 | ch5 / ch6 | 500000 since v0.9.44 | the RX ring and the transmitter |
(In the original-application build, `MASTER_MONITOR_MODE 0`, USART3 and USART4 stay at 115200 as before.) The master runs from the 16 MHz HSI with no divider, so 500000 baud is exact on all three (BRR = 32).
* The master's `stm32g0xx_hal_msp.c` already sets exactly these pins, alternate functions and DMA channels; the three serial ports are initialised in `main.c`. USART3 and USART4 share one interrupt vector (`USART3_4_IRQHandler`).
* **The Y wiring is confirmed (v0.9.44): TX-Y on PC10 and RX-Y on PC11,** which is what the master's setup configures. (The datasheet gives PC10 only TX functions (USART3_TX, USART4_TX) and PC11 only RX functions (USART3_RX, USART4_RX). The master's TX-Y must therefore be on **PC10** and RX-Y on **PC11**. Had the board been wired the other way round, the USART's SWAP bit would cross them in software; it is not needed.)
* The slave link has NO addressing, so each slave needs its own serial line: this is why three ports are used.

## 2. What exists
* The fault engine `m_faults` is already three-phase: three phase inputs (`ph[3]`), three run levels, three clear pulses, three contactor outputs, three units in Domestic mode and one in Industrial (one contactor, any phase trips all, a phase-sequence check).
* The Set-Mode items, the three-phase Run pages (per-phase V and A, phase-to-phase V and sequence, Display Scroll) and the settings (mode Single / Industrial / Domestic).
* The slave is the same single-phase slave in all three phases: three identical boards, each regulating its own phase.

## 3. What is missing
1. The master application feeds the engine from ONE slave. It needs per-phase data (the three `$S`, `$E`, `$H` streams), per-phase `$M` frames out, per-phase link-lost handling, and the per-phase calibration, Re-tune, converter-current and profile pages.
2. The HAL glue for three links: three receive rings, three transmitters, 500 kbaud on USART3 and USART4 (they are at 115200 now), the receive error recovery of USART1 repeated for each.
3. **Phase-to-phase voltage and the phase sequence.** Three RMS values are not enough: they need the timing between phases, which the slaves will measure (section 5).
4. Per-phase autotune (three profiles, one per slave).
5. The mains rules of the three-phase modes that need a second phase: one phase missing, phase imbalance, and what Industrial does with a lost link.

## 4. The order of work (each step tested on the PC first, with mutation checks, like the single-phase work)
* **S1. The data model:** per-phase data in the master application, the engine fed for R, Y and B, per-phase `$M` frames. Domestic first (three independent units), then Industrial (one unit, all trip).
* **S2. The three links in the glue:** the three receive paths and transmitters, link-lost per slave. (HAL code: not compilable here, as for the single-phase glue; reviewed line by line.)
* **S3. The three-phase screens fed from the three slaves,** and the per-phase service pages.
* **S4. Phase sequence and phase-to-phase voltage from the slaves** (section 5): the ARM frame, the SYNC byte, the slave timestamp, the classification.
* **S5. Industrial rules:** any phase trips all, the sequence check, a lost slave.
* **S6. Per-phase autotune,** only after the single-phase sweeps are proven safe on the bench.

## 5. Phase sequence and phase angles FROM THE SLAVES (decided: not from the master's PA5, PA6 and PA7)
**Decision (v0.9.45): the sequence is measured by the slaves, not by the master's three phase inputs.** (The old firmware used PA5, PA6 and PA7; PA5 is also the pin that `MASTER_HEAT_INPUT` assumed: those pins stay unused by the new master code.)
**Principle.** The three slaves have no common clock, so the master gives them one instant. Each slave measures the time from that instant to its own next RISING zero-cross and reports it. Phase Y lags R by a third of a period and B by two thirds, so the DIFFERENCES of the three delays, as fractions of the period, are 1/3 and 2/3.
**The exchange (per measurement round):**
1. The master sends an ARM frame on each of the three links: `$K,id,SEQ` ("the next SYNC byte within 50 ms is real"). The slave answers with the usual `$R`.
2. The master sends ONE SYNC byte (0x16) on the three links, back to back (about 20 microseconds apart: under 1 degree at 50 Hz).
3. Each slave timestamps that byte IN THE SERIAL INTERRUPT (the character-match interrupt of its USART, with the 1 microsecond timer it already uses for the zero-cross timestamps) and, at its next rising zero-cross, stores `delay = t_zero_cross - t_sync` (0 up to one period) together with its measured period.
4. The slave reports `delay` and the round id in OPTIONAL trailing fields of `$E` (the same backward-compatible mechanism as the converter current and the profile fields).
5. The master takes the three delays of one round id and classifies:
   * a_Y = ((delay_Y - delay_R) mod T) / T, a_B likewise;
   * **OK (R-Y-B)**: a_Y near 1/3 and a_B near 2/3, each within 0.10 of a period (36 degrees);
   * **REVERSE (R-B-Y)**: a_Y near 2/3 and a_B near 1/3;
   * **SAME PHASE**: any two phases within 0.10 of a period of each other (mod 1);
   * anything else: not decided, measure again.
   Three rounds, the median decides; a phase whose slave is silent is "phase missing", not a sequence fault.
**Why it is robust:** the ARM frame (checksummed) means a noise byte on the line is not taken as a SYNC; the ISR timestamp removes the main-loop jitter (about 1 ms, 18 degrees at 50 Hz); each slave's zero-cross circuit has its own fixed delay (a few degrees at most), far inside the 36 degree margin; the classification needs no absolute angle.
**Accuracy of the angles** (for the phase-to-phase voltage from the three RMS values and the angles): about 1 to 5 degrees, which moves a 400 V phase-to-phase reading by well under 1 %.
**What it needs:** the three links alive (steps S1 and S2); a character-match interrupt and a timestamp in the slave's serial interrupt (slave change); two optional fields in `$E` and one new `$K` command (the link library, both copies); the classification and the engine input `seq_state` in the master (the engine already takes it). All of it is testable on the PC with three simulated slaves at chosen angles, with jitter, a lost link and a reversed sequence.
**What it replaces:** the old firmware's blocking `detect_phase_sequence()` (it waited for edges on the master's pins and delayed the main loop).

## 6. Before any three-phase power-up
* The single-phase stage must be proven safe on the bench first: the over-current trip lowered (about 60 A peak with the 22.67 ohm burden), the gate drivers' desaturation checked, a current-limited source for the first run (see AUTOTUNE_BENCH.md, the first section).
* Three stages on one source: use a current-limited source for the first three-phase run too.
* Industrial mode trips all three phases together by design; Domestic does not: test Domestic first.

## 7. Decisions needed
1. (Done) TX-Y is on PC10.
2. Domestic first (recommended: three independent units, the simplest) or Industrial first?
3. (Decided) Phase timing from the slaves (section 5): no schematic page is needed for it.
