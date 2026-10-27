# Phase sequence and phase-to-phase voltage, measured by the slaves (v0.9.51)

## What it does
In Industrial and Domestic mode, with all three slaves heard, the master measures the angles between R, Y and B and from them
* the **phase sequence**: OK (R-Y-B), REVERSE (R-B-Y) or SAME (two phases the same). In Industrial mode with **Phase Sequence = Yes** (Set-Mode) a REVERSE or SAME result blocks the start and trips a running unit ("Phase Sequence"), as the engine always specified. Domestic and Single mode never check it;
* the **phase-to-phase voltages** R-Y, Y-B, B-R on the second Run page (`V = sqrt(Va^2 + Vb^2 - 2 Va Vb cos(angle))` from the three OUTPUT voltages and the measured angles; dashes while the angles are not known).
The master's own phase inputs (PA5, PA6, PA7) are not used.

## How (a round, about 0.3 s)
1. The master sends `$K,id,SEQ,<round>` (ARM, round 1 to 255) on the R, Y and B links.
2. 20 ms later it sends ONE SYNC byte (0x16) on the three links at the same moment (the three data registers written back to back: within about a microsecond).
3. Each slave time-stamps that byte IN ITS RECEIVE INTERRUPT with the microsecond timer of its zero-cross interrupt, and at its next RISING zero-cross computes `delay = time of the edge - time of the SYNC byte` (0 up to one period) and its mains period. The result (round, delay, period) rides in the optional trailing fields of every `$E` frame until the next round replaces it.
4. The master takes the three results of the round and classifies: Y lags R by a third of a period and B by two thirds, so `(delay_Y - delay_R) mod T` is T/3 and `(delay_B - delay_R) mod T` is 2T/3 for R-Y-B (swapped for R-B-Y; near 0 for the same phase). **Each must be within 10 % of a period (36 degrees).** The decision changes only after **three identical rounds in a row**; no classifying round for 20 s makes it unknown again; a slave that is not heard makes it unknown at once.
Rounds: every 300 ms until the sequence is known (about 1.5 s after power-up), then one every 5 s (the monitoring); every 2 s if NO slave answers (a build without the measurement), so the link is not spammed.
The ARM frame is checksummed, so a noise byte is never taken for a SYNC (a SYNC is only accepted for 100 ms after an ARM). Common delays (the SYNC byte's reception, each board's zero-cross circuit) cancel in the differences; what is left is a few tens of microseconds, about 1 degree.

## What you see
* **Industrial, Phase Sequence = Yes:** until the first measurement the status row says `Checking Phase Seq` and the unit does not start (a unit that is already running is never touched by "not known yet"). Then it starts, or stays off on REVERSE / SAME. **v0.9.54:** a REVERSE or SAME measured BEFORE the start is shown: `Phase Seq Reverse` / `Phase Seq Same Ph` on the status row, `R : PH SEQ REVERSE` / `PH SEQ SAME` on the status page, "Phase Sequence" as the fault shown (before, the start delay was held and the screen stayed at `Start in 05 s` with no reason). When the sequence is right again the countdown runs and the unit starts by itself.
* **The second Run page:** `R-Y :  398 V`, `Y-B :  398 V`, `B-R :  398 V` for three 230 V phases. Dashes mean the angles are not known: a slave is missing or the measurement failed. **This page is the quickest bench check that the measurement works.**
* Build switch: `SLAVE_SEQ_SYNC` (slave_features.h), on by default wherever a master runs the slave over link v2; 0 = the slave answers `$K SEQ` with UNKNOWN_CMD and the master sees no result.

## Limits and what to check on the bench
* **The reference is the zero-cross circuit's RISING edge.** A board whose zero-cross signal gives falling edges only (a pulse-train mode) never reports: its round fails, the sequence stays unknown, and Industrial with Phase Sequence = Yes does not start. Check that the second Run page shows voltages on every unit.
* **The time stamp is taken in the receive interrupt.** A long higher-priority interrupt delays it; one or two per cent of a period of error is harmless (the tolerance is 10 %). If the line voltages jump around, tell me the numbers.
* **A rising edge on every board must be the same edge of the mains** (the three boards are identical, each with its own phase on its own zero-cross input). If one board's zero-cross circuit is wired with the opposite polarity, its angle is 180 degrees out and the sequence reads wrong.
* Not measured: the INPUT phase-to-phase voltage (the output angles are reused, as in the original specification); the master's three-phase protection of a missing phase is the engine's link/voltage rules.
