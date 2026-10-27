# The converter-side current: net I-OUT (v0.9.30)

## What the schematic shows (22122016-EB, page 1, bottom right; read from the drawing: please confirm on the board)
* CT2500 (T1): 2500 turns. Its secondary works into R42, R43, R44 = 180 ohm x 3 in parallel = **60 ohm** (the note on the drawing: "Shunt = 180E x 3").
* A bias divider R40 680 / R39 1k / R38 1k / R37 680 between ground and 3.3 V gives the midpoint **VCC/2 = 1.65 V** (and, at TP32 and TP33, the references 0.67 V and 2.63 V of the short-circuit comparators).
  The burden's lower end does NOT join that divider (the line only crosses it): the burden voltage is the difference between the VCC/2 node and the burden's lower node.
* U8C (MCP6004), R45 / R46 1k, R47 6k8: a differential amplifier of gain 6.8, + input from VCC/2, 1 nF across R47 (about 23 kHz). Its output is **I-OUT** (TP20, connector pin 32, PA2 of the piggyback).
* The comparators U10 and U14 (MCP6567) watch the burden voltage through R41 1k and make the FLT line; they trip at about +-0.98 V across the burden = about +-41 A (peak) of CT primary current (the note says 41.7 A).

## The burden of THIS board: 68 ohm x 3, not the drawing's 180 ohm x 3
The drawing has R42, R43, R44 = 180 ohm each. This board has **68 ohm each = 22.67 ohm in parallel**. `IOUT_BURDEN_R_EACH_OHM` (slave_iout.h, default 68, settable in slave_config.h) is the one number to change if the resistors change.
| | 68 ohm x 3 (this board) | 180 ohm x 3 (the drawing) |
|---|---|---|
| volts of I-OUT per ampere of CT primary current | **0.0617** | 0.1632 |
| ADC count | **13.1 mA** | 4.9 mA |
| the channel clips at | **+-26.8 A peak (18.9 A RMS)** | +-10.1 A peak (7.1 A RMS) |
| the hardware short-circuit comparators trip at | **about +-108 A peak** | about +-41 A peak |
V(I-OUT) = 1.65 V +- 6.8 x R_burden x I_primary / 2500. The conversion is tested for both burdens.

### Check this: the short-circuit trip moved
The comparators U10 and U14 watch the burden voltage against FIXED references (about +-0.98 V across the burden, from the R37 .. R40 divider). A smaller burden means a larger current for the same voltage: the hardware FLT now trips at about **108 A peak of CT primary current instead of 41 A**.
That is 2.6 times higher. If it is intended (a larger unit) fine, but please check it against the IGBT's pulsed current rating and the protection you want (FGL60N100BNTD is a 60 A class part: look at its pulsed rating in the datasheet). To keep a lower trip with the 22.67 ohm burden the comparator references would have to change (R37 .. R40).

## Scaling for the original burden (the drawing)
* Full load of the original board: 5.89 A RMS = 8.33 A peak, a swing of +-1.36 V with 60 ohm (+-0.51 V with 22.67 ohm: a lower swing means fewer counts, 13 mA each, so the resolution at small currents is coarser).
* Tolerance before any calibration: about +-3 to 4 % (1 % resistors, the CT ratio, the amplifier gain). `IOUT_CAL` (slave_iout.h) is a factor for a field calibration.
* The output cannot leave 0 .. 3.3 V: above the clip limit the channel shows `OVER RANGE (clipped)` and the reading is too low. The comparators still trip at their own current: they are separate.

## What it measures
The current in the conductor that passes through the CT. The drawing does not show which one. From the net names (PRI-PHASE, PRI-NTRL) and the 5.89 A full load it looks like the **converter (primary) side of the series transformer**, not the load line. If so:
**load current = this current x the transformer's turns ratio** (secondary : primary). The ratio differs per transformer and is not known to the firmware yet.

## How to use it on the bench
1. Slave config `slave_config_STAGE3_cap2752_full_range_MASTER_fast_start_iout.h` (or `SLAVE_IOUT 1`), master v0.9.30. FIRST check that the input and output voltages read exactly as before (the ADC scan went from 3 to 4 channels).
2. Master: LOG, then MENU x7: **Converter Current**. No load: close to 0 (the transformer's magnetizing current). With a load: RMS and peak in amperes.
3. Put a clamp meter on the SAME conductor that goes through the CT and compare. If you cannot reach it, compare with the master's load current (Run screen): load / this = the turns ratio.
4. Send me: which conductor, the readings at three loads, and the ratio you found.

## What it will be used for next
* The ratio gives a load current from the slave, an independent cross-check of the master's CT (overload, inrush).
* The autotune sweep has no load: the current is then the magnetizing current, which spikes when the core saturates. That is an earlier and safer stop than the slope of the voltage, and a limit far below the 41 A hardware trip.

## v0.9.58: 180 ohm x 3 is the default again
The boards change to 180 ohm x 3 (60 ohm, the drawing's value). `IOUT_BURDEN_R_EACH_OHM` is 180.0f in the default `slave_config.h`, the production config, the other I-OUT configs and `slave_iout.h`. A board that still has 68 ohm resistors needs `#define IOUT_BURDEN_R_EACH_OHM 68.0f` in its `slave_config.h`, or every converter-current reading is 2.65 times too low.
* Resolution 4.9 mA per count (was 13.1); the channel clips at +-10.1 A peak (was 26.8); the hardware comparators trip at about +-41 A peak (was 108).
* At the converter peaks measured on the bench (7.8 A of peak at 10 A of load) the load reading clips from about 13 A of load. Above that the slave's load reading is "clipped": R falls back to the master CT, **Y and B have no load reading and no overload trip**. Check the clip and the 41 A trip against your largest normal load and inrush.
* Redo the Slave Load Cal (zero and gain) of every phase after the resistors change.


## v0.9.61: F02 when the contactor closes onto a load
Reported: every phase tripped F02 (these comparators) when the contactor closed onto a load, never at no load or when the load was added later. v0.9.61 freezes the regulator's duty from 150 ms before the contactor closes to 500 ms after (the load-connect hold, docs/LINK_V2.md). If F02 remains, the peak is the load step itself: measure I-OUT at the closing with a scope, and change the trip in hardware (burden or R37 .. R40), not above about 75 A.
