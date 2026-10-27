# Load current from the slave's converter current (v0.9.45)

## What it is
The slave measures the current of the converter (the primary of the series transformer, the I-OUT channel). The master can use it as the **load current** instead of its own CT: Set-Mode item **Load Source** = `Master CT` (the default) or `Slave`. The Run screen's Load, the Overload trip, the fault log's load and the three-phase pages all follow the chosen source.
A conversion is needed, and it is calibrated on the new service page **Slave Load Cal** (LOG, then MENU x9; one more MENU returns to the Run screen).

## Why the PEAK, not the RMS
At 10 A of load the converter RMS read 0.9 A, no different from no load (0.9 to 1.2 A): the converter carries narrow pulses, so its RMS hardly moves with the load. Its PEAK moved: 7.8 A at 10 A of load. The conversion therefore uses the peak:
`load = (peak - zero) x gain`
* **zero** = the converter's peak at NO load with the unit RUNNING (its own magnetizing current).
* **gain** = the true load / (peak - zero) at a known load.
The page shows the peak and the RMS, so you can see for yourself which one follows your load.

## Calibration, step by step (single-phase, one slave)
1. The slave needs `SLAVE_IOUT 1` (the converter current) in its config. The unit must be RUNNING.
2. **Zero.** Run the unit with NOTHING connected to the output. Open the page (LOG, MENU x9). The entry starts from the peak; bring it to `000.0` with START (adds 1 to the digit under the cursor, a digit wraps 9 to 0) and STOP (moves the cursor), then press LOG. The screen says `Zero Set`.
3. **Gain.** Connect a known load (a resistive load of at least about 5 A is best) and read it on a meter you trust (a clamp meter or the calibrated master CT). On the page type the meter's value (for example `010.0`) and press LOG: `Calibrated`. The peak must be at least 1 A above the zero, the load 2.0 to 180.0 A, and the gain between 0.200 and 5.000, otherwise the screen says why (`Load Too Low`, `Enter 2.0-180.0 A`, `Out Of Range`) and nothing changes.
   Example: zero 2.50 A, peak 10.30 A at a 10.0 A load: gain = 10.0 / (10.30 - 2.50) = 1.282.
4. **Check at other loads.** Leave the page, set Load Source = Slave, apply three or four other loads and compare the Run screen's Load with your meter. If the error is more than about 10 %, the peak is not a good measure of the load for this unit or this kind of load: keep `Master CT`.
5. The zero and gain are saved with the settings (they survive a restart) and are NOT cleared by Factory Reset.

## What happens when the slave's reading cannot be trusted
The master falls back to its own CT, so the Overload trip never depends on a reading that is not there:
* Load Source = Slave but the page was never calibrated (gain 0);
* the slave sends no converter current (an older slave, or `SLAVE_IOUT 0`), or the link is lost;
* the converter amplifier is clipping (the current is larger than shown: above 26.8 A peak with the 68 ohm burden).

## Limits to keep in mind
* The peak follows the load's PEAK current. A resistive load calibrates well; a non-sinusoidal load (a rectifier, a motor start) has a different peak to RMS ratio than the load you calibrated with, and the number will be off. The master CT measures RMS directly.
* The reading is the largest sample of about 100 ms. It is steady for steady loads and noisy for fast-changing ones.
* The channel clips at +-26.8 A of converter peak, which sets how large a load it can show.
* This version has ONE calibration, for the single slave on the R link. Per phase calibrations come with the three-phase work.
* The master CT is still used for the inrush backup trip (it is a hardware ADC full-scale check), whatever the source.

## Three phases: Y and B (v0.9.55)
The master's CT serves one phase (R). The Y and B phases get their load ONLY from their own slave's converter current, with THEIR OWN zero and gain:
* LOG, then MENU to **Slave Load Cal**: in Industrial and Domestic the page comes three times, `Slave Load Cal R`, `Slave Load Cal Y`, `Slave Load Cal B` (MENU steps R, Y, B, then the next page). Calibrate each the same way: unit RUNNING, no load, type 00.0, LOG (the zero); a known load, type it, LOG (the gain).
* Until a phase is calibrated, or while its slave sends no current reading, or the reading is clipped, that phase shows `--.-A` and has **no overload protection**. Once calibrated, its load is on the Run page and its own overload trip works (Domestic: only that phase's unit; Industrial: the whole unit).
* R keeps the Load Source choice (Master CT or Slave) as before.
* Stored in the settings record, bytes 33 to 40 (Y zero, Y gain, B zero, B gain). An older record reads as "not calibrated". Factory Reset keeps them.

## v0.9.58
* The gain needs the converter peak at least **0.5 A** above the zero (it was 1 A). When it is less, the message says both numbers: `Load Too Low` / `Pk 2.90 Zero 2.50` (the peak now and the stored zero, amperes). A zero taken with the load already on, a load on another phase's output, or a converter that hardly works at this input voltage (its current follows the CORRECTION it makes, so near 230 V in it barely moves) all show up there. Calibrate with the input well away from the set voltage (a variac) and the load on THAT phase's output.
