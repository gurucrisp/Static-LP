# The converter-side current: net I-OUT (v0.9.30)

## What the schematic shows (22122016-EB, page 1, bottom right; read from the drawing: please confirm on the board)
* CT2500 (T1): 2500 turns. Its secondary works into R42, R43, R44 = 180 ohm x 3 in parallel = **60 ohm** (the note on the drawing: "Shunt = 180E x 3").
* A bias divider R40 680 / R39 1k / R38 1k / R37 680 between ground and 3.3 V gives the midpoint **VCC/2 = 1.65 V** (and, at TP32 and TP33, the references 0.67 V and 2.63 V of the short-circuit comparators).
  The burden's lower end does NOT join that divider (the line only crosses it): the burden voltage is the difference between the VCC/2 node and the burden's lower node.
* U8C (MCP6004), R45 / R46 1k, R47 6k8: a differential amplifier of gain 6.8, + input from VCC/2, 1 nF across R47 (about 23 kHz). Its output is **I-OUT** (TP20, connector pin 32, PA2 of the piggyback).
* The comparators U10 and U14 (MCP6567) watch the burden voltage through R41 1k and make the FLT line; they trip at about +-0.98 V across the burden = about +-41 A (peak) of CT primary current (the note says 41.7 A).

## The scaling
V(I-OUT) = 1.65 V +- 6.8 x 60 ohm x I_primary / 2500 = **1.65 V +- 0.1632 V per ampere** of CT primary current.
* 1 ADC count (3.3 V / 4095) = 4.94 mA of primary current.
* The output cannot leave 0 .. 3.3 V: it clips at about **+-10.1 A peak (7.1 A RMS)**. Above that the channel shows `OVER RANGE` and the reading is too low. The comparators still trip at 41 A: they are separate.
* Full load of the original board, 5.89 A RMS = 8.33 A peak = a swing of +-1.36 V (0.29 .. 3.01 V).
* Tolerance before any calibration: about +-3 to 4 % (1 % resistors, the CT ratio, the amplifier gain). `IOUT_CAL` (slave_iout.h) is a factor for a field calibration.

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
