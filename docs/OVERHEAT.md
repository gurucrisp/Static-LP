# Overheat from the heat-sink NTC (v0.9.49)

## The circuit and where the decision is made
`3.3 V -- 1.5 kohm -- PA5 -- 10 kohm NTC -- GND`, read by the SLAVE's ADC (PA5, 12 bit, the same 3.3 V reference: the reading is ratiometric, so the supply does not matter). The slave sends the raw reading (0 to 4095) in every status frame; the MASTER converts it to a temperature, compares it with the **Overheat Temp** setting and gives the engine an overheat signal per phase. The engine's own rules then apply, unchanged: the **Overheat Trip Delay** (Set-Mode item 12, default 10 s) must pass with the signal asserted, the contactor opens, the unit restarts by itself when cooled (after the Restart Time), and three trips within 30 minutes need START.
A divider with 1.5 kohm puts the middle of the scale (reading 2048) at an NTC of 1.5 kohm, about 75 degC, so the reading changes by about 33 counts per degree exactly where the trip is.

## The setting
Set-Mode item **Overheat Temp** (after Overheat Trip Delay): 50 to 120 degC, default **75**. Saved with the settings (byte 27 of the record; an older record, where that byte was always zero, loads as 75). Factory Reset puts it back to 75.
* The unit is tripped when the heat sink reaches the Overheat Temp and the delay has passed. It may restart when the heat sink is **10 degC below** it (65 degC with the default): the hysteresis is fixed.
* The reading is filtered (about 160 ms) and converted with a table (tools/gen_ntc_table.py) that is within 1 degC of the beta equation.
* **In three-phase modes each slave's own NTC** is used: Domestic trips only the unit whose heat sink is hot, Industrial stops all three.

## Seeing the temperature
LOG, then MENU x7: the **Converter Current** page's last row is `Heat Sink 047.3 C` (R slave). `OVERHEAT 082.0 C` while the overheat signal is asserted; `Heat Sink: no NTC` if the reading is at the top of the scale; `Heat Sink  ---` with no fresh reading from the slave. (The Y and B temperatures are used for the protection but not displayed yet.)

## Fail-safe rules
* **NTC shorted** (reading below 150): counted as OVERHEAT.
* **NTC open or not fitted** (reading colder than -10 degC, in practice 4095): shown as `no NTC`, NEVER a trip. A board without an NTC fitted therefore runs, with no overheat protection: check that the page shows a temperature on every unit.
* No fresh reading (the link is not heard): no signal and no value; the unit has its link-lost rules.
* The pin option `MASTER_HEAT_INPUT` stays 0 (PA5 of the master is the R phase input). `MASTER_NTC_HEAT` (master_monitor.h, default 1) switches the trip off (0): the temperature is then only shown.

## The default of 75 degC
It is a conservative heat-sink alarm: far below the IGBT's own limit (175 degC junction), and where the sensor resolves best. The risk is a nuisance trip in a hot cabinet (an ambient of 45 degC and a small heat sink at full load can come close). **Measure it:** run the unit at full load until the temperature settles (watch the page), then set the Overheat Temp 10 to 15 degC above that; do not set it above about 100 degC at this sensor.

## Check the NTC's beta value
The table assumes beta = 3950 (the most common for a 10 kohm part). A part with beta 3435 reads about 7 degC COOLER than it is at 75 degC (so it would trip at about 82 degC for a setting of 75). Check one point against a thermometer on the heat sink, or give the part's beta: `python3 tools/gen_ntc_table.py <beta> [R0] [R_fixed] > master/Core/Src/m_ntc_table.h` regenerates the table.
