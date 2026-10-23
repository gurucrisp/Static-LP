# Phase 1: robustness (v0.9.27)

## 1. Independent watchdogs
| | slave | master |
|---|---|---|
| switch | `SLAVE_WATCHDOG 1` (slave_features.h) | `MASTER_WATCHDOG 1` (master_monitor.h) |
| timeout | `SLAVE_WATCHDOG_MS` 500 ms | `MASTER_WATCHDOG_MS` 1000 ms |
| starts | just before the main loop | just before the main loop (after the slow RTC and flash start-up) |
| kicked | once per main-loop pass | once per main-loop pass |
| on a reset | timers and pins back to the reset state (pulses off), waits for `$M` again, reports `rst = 1` in `$H` | contactor outputs go low, starts again, logs "Watchdog Reset" |
The IWDG runs from the LSI (nominally 32 kHz, in the worst case 25 to 35 kHz): the real timeout is between about 0.75 and 1.3 times the nominal one. It cannot be stopped once started. A debugger halt does not reset the unit (DBGMCU freeze, where the chip's header has it).
**Why not shorter:** a main-loop pass includes a flash erase (tens of ms) when settings or a calibration are saved. If the unit ever resets by itself during a save, the timeout is too short: raise it, and tell me.
**Seen on the unit:** the master's log shows "Watchdog Reset" (LOG key); the slave's last reset cause shows as a `w` at the right end of row 3 of the diagnostics page (LOG, MENU x2).

### Bench checks (not done yet)
1. With a debugger or a deliberately added `while(1){}` after the loop starts, the slave must reset after about half a second and the pulses must stay OFF during and after it (scope a gate drive signal). This is the check that matters.
2. The same for the master: the contactors must open when it resets.
3. Save a calibration and a setting a few times: the unit must not reset.

## 2. CT peak backup (master)
A second layer behind the slave's own over-current protection: the master samples the CT channel every pass of its main loop and takes the largest reading of each 10 ms. A reading above **3 x the Overload Current** (peak limit = 3 x the setting, in amps) **or at the ADC full scale** (4000 of 4095 counts), held continuously for **20 ms**, is an Inrush: the contactor opens, it is logged, and the retry rules apply (`MASTER_INRUSH_RETRIES`).
* The CT circuit gives an averaged level, not the instantaneous current, so this catches a gross over-current, not a brief spike.
* With the default calibration the ADC full scale is far above 3 x the usual Overload settings, so for a high Overload setting the full scale is the real limit. A calibrated gain (Load Calibration) moves both.
* A load with a big switch-on current (a motor, a transformer) can exceed 3 x the Overload setting for more than 20 ms: if that trips the unit, raise the Overload Current setting, or switch the backup off with `MASTER_CT_PEAK 0`.

## 3. Running hours
The minutes the output has been ON are kept in the RTC's battery-backed registers (TAMP BKP0R..BKP4R) as two redundant copies, saved once a minute while the output is on. At a power cut at most about a minute is lost. They are the "Run 123h45m" time of a log entry while the clock is not set (when it is set the entry carries the real date and time).
**Needs the RTC battery.** Without it the counter restarts from 0 at every power-up. A new board (or a lost battery) starts at 0. There is no menu item to clear it.
