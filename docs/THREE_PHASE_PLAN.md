# Three-phase: the hardware map, what exists, what is missing, and the order of work

## 1. The three slave links on the master (checked against the code and the STM32G070 datasheet)
| Phase | USART | Master TX pin | Master RX pin | AF | DMA RX / TX (DMA1) | Baud now | To do |
|---|---|---|---|---|---|---|---|
| R | USART1 | PC4 | PC5 | AF1 | ch1 / ch2 | 500000 (LINK_BAUDRATE) | works today |
| Y | USART4 | PC10 | PC11 | AF1 | ch7 / ch3 | 115200 | 500000, RX ring, TX |
| B | USART3 | PB2 | PB0 | AF4 | ch5 / ch6 | 115200 | 500000, RX ring, TX |
* The master's `stm32g0xx_hal_msp.c` already sets exactly these pins, alternate functions and DMA channels; the three serial ports are initialised in `main.c`. USART3 and USART4 share one interrupt vector (`USART3_4_IRQHandler`).
* **Check the Y wiring.** The datasheet gives PC10 only TX functions (USART3_TX, USART4_TX) and PC11 only RX functions (USART3_RX, USART4_RX). The master's TX-Y must therefore be on **PC10** and RX-Y on **PC11**. If the board has them the other way round (TX-Y on PC11), the USART's SWAP bit crosses them in software (`AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT`, `Swap = UART_ADVFEATURE_SWAP_ENABLE`); say so and it is one setting.
* The slave link has NO addressing, so each slave needs its own serial line: this is why three ports are used.

## 2. What exists
* The fault engine `m_faults` is already three-phase: three phase inputs (`ph[3]`), three run levels, three clear pulses, three contactor outputs, three units in Domestic mode and one in Industrial (one contactor, any phase trips all, a phase-sequence check).
* The Set-Mode items, the three-phase Run pages (per-phase V and A, phase-to-phase V and sequence, Display Scroll) and the settings (mode Single / Industrial / Domestic).
* The slave is the same single-phase slave in all three phases: three identical boards, each regulating its own phase.

## 3. What is missing
1. The master application feeds the engine from ONE slave. It needs per-phase data (the three `$S`, `$E`, `$H` streams), per-phase `$M` frames out, per-phase link-lost handling, and the per-phase calibration, Re-tune, converter-current and profile pages.
2. The HAL glue for three links: three receive rings, three transmitters, 500 kbaud on USART3 and USART4 (they are at 115200 now), the receive error recovery of USART1 repeated for each.
3. **Phase-to-phase voltage and the phase sequence.** Three RMS values are not enough: they need the timing between phases (see section 5).
4. Per-phase autotune (three profiles, one per slave).
5. The mains rules of the three-phase modes that need a second phase: one phase missing, phase imbalance, and what Industrial does with a lost link.

## 4. The order of work (each step tested on the PC first, with mutation checks, like the single-phase work)
* **S1. The data model:** per-phase data in the master application, the engine fed for R, Y and B, per-phase `$M` frames. Domestic first (three independent units), then Industrial (one unit, all trip).
* **S2. The three links in the glue:** the three receive paths and transmitters, link-lost per slave. (HAL code: not compilable here, as for the single-phase glue; reviewed line by line.)
* **S3. The three-phase screens fed from the three slaves,** and the per-phase service pages.
* **S4. Phase timing** (section 5): the phase sequence and the phase-to-phase voltage.
* **S5. Industrial rules:** any phase trips all, the sequence check, a lost slave.
* **S6. Per-phase autotune,** only after the single-phase sweeps are proven safe on the bench.

## 5. Phase timing: how the master learns the angle between R, Y and B
Option A (needs no new master hardware): the master sends one SYNC frame on the three links at the same moment; each slave measures the time from receiving it to its next rising zero-cross and reports it in an optional field of `$E`. The master turns the three delays into the angles: the sequence and, with the three RMS values, the phase-to-phase voltages.
Accuracy: each slave must timestamp the received SYNC in the serial interrupt (the main-loop polling jitter of about 1 ms would be 18 degrees at 50 Hz); with an interrupt timestamp it is about 50 microseconds, about 1 degree.
Option B (if the master board has R, Y and B zero-cross or voltage inputs): the master measures the timing itself. I need the master's schematic page(s) for any R, Y and B voltage or zero-cross inputs to know whether B is possible.

## 6. Before any three-phase power-up
* The single-phase stage must be proven safe on the bench first: the over-current trip lowered (about 60 A peak with the 22.67 ohm burden), the gate drivers' desaturation checked, a current-limited source for the first run (see AUTOTUNE_BENCH.md, the first section).
* Three stages on one source: use a current-limited source for the first three-phase run too.
* Industrial mode trips all three phases together by design; Domestic does not: test Domestic first.

## 7. Decisions needed
1. Confirm TX-Y is on PC10 (section 1).
2. Domestic first (recommended: three independent units, the simplest) or Industrial first?
3. Phase timing: option A, or does the master board have R, Y and B timing inputs (option B)?
