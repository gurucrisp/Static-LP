# Voltage calibration (v0.9.20)

## What it does
The slave measures the input and the output voltage with one gain each (volts per ADC count, `g_meas_cal` in slave_meas.c). The calibration changes that gain from ONE meter reading per channel: total correction = meter reading / what the NOMINAL chain reads. It is always against nominal, so doing it twice gives the same result.
Accepted only if: the unit is stopped, the nominal reading is at least 50 V, the signal is steady (max - min within 2 % over the 1 s average), and the correction is within +-10 % (0.90 to 1.10). Otherwise nothing changes. The ratio is stored in 0.01 % units (10000 = nominal) in two CRC-protected flash pages.

## Why only while stopped
A gain change while the unit regulates would make the regulator see a different output voltage at once: the output would jump by the same percentage. Stopped (bypass), nothing is being regulated.

## Bench procedure
1. Slave config with `SLAVE_VOLTAGE_CAL 1` (all master-control configs have it), master `MASTER_APP_MODE 1`. The program must end below 0x0807F000 (the driver refuses to write otherwise and the master shows `Save Failed`).
2. Mains steady at about 230 V. Stop the unit (STOP on the Run screen, or on the calibration page). The contactor is open when stopped: measure where the SLAVE's sensors are connected. With the unit in bypass both sensors should see the same voltage as the input terminals (if the output sensor is on the far side of the transformer winding, check that the two readings agree before trusting the output calibration).
3. LOG, MENU x5: I/P Calibration. Read the meter at the input, type it (START / STOP), LOG. Wait for `Calibrated`. Then MENU: O/P Calibration, type the meter reading at the output sensor's node, LOG.
4. Power off and on: the readings must be the same as before (the stored calibration is applied at start). A programmer's mass erase removes it; a normal re-flash of the program does not touch the two pages.
5. To undo: type 000.0 on the page and LOG.

## What is NOT verified
* The flash driver (slave_flash.c) was tested only against a model of the HAL flash API (bank/page arithmetic for dual- and single-bank mode, double-word programming, the address guard). Check on the board that a calibration survives a power cycle.
* slave_cal_hw.c (the binding to g_meas / g_ctrl) and its call in main.c could not be compiled here (no slave HAL headers). Send me any build error.
* The 1 s average assumes the main loop calls the link service every few milliseconds (it does in the current main loop).
* The output sensor topology in bypass (step 2).
