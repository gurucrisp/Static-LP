# Master screens, keys and Set-Mode (v0.9.2 to v0.9.4)

This release makes the master a **flashable application that displays and configures**. `MASTER_APP_MODE 1` (in `master_monitor.h`, the default) runs it inside the monitor build, which already owns the
USART and the LCD. **It does not drive the contactors and does not tell the slave to run: the three contactor pins stay at their OFF level, as in the monitor. That is stage 7c.**
`MASTER_APP_MODE 0` gives the previous plain monitor screens back; `MASTER_MONITOR_MODE 0` gives the original firmware.

## Keys (pins in `m_board.h`, active low)
MENU = PB10, START = PB11, STOP = PB12, LOG = PB13. **If MENU and LOG are swapped on your board** (a short LOG press turns the page; a short MENU press opens the log), swap the two defines in `m_board.h`.
MENU + LOG held 2 s opens Set-Mode (and, in Set-Mode, saves and leaves). The keys are debounced (30 ms); START and STOP act on the press and are ignored while MENU or LOG is down; MENU and LOG give a
short press on RELEASE, never for a touch that was part of the combination.

## Run-Mode
Splash (3 s): Welcome / Static Voltage / mode / Wait... Single phase: one screen (O/P and I/P voltage, load, frequency, status). Three-phase (Industrial, Domestic): three pages (per-phase V and A; phase-to-phase V and
frequency; per-phase status), rotating every Scroll Time or, with Display Scroll = No, one page per MENU press; a new fault jumps to the status page. LOG opens the fault log (START older, STOP newer, LOG or MENU leaves, 30 s idle leaves).
**In single-phase mode MENU toggles to your monitor diagnostics** (the screens you had) and back.
What is shown comes from the slave's `$S` frame. "Output ON" means the slave is regulating; the contactor is not driven yet. The load shows `--.-` (the master CT is stage 7c). Faults: the slave's fault code is mapped by the
same function the fault engine uses (`MF_SlaveFaultType`); the engine itself is not running yet, so there are no master-side limit faults, restarts or log entries from faults yet. The fault log viewer works and can be cleared.

## Set-Mode
Items (the number shows top right): 1 Mode, 2 Calibration, 3 Set Voltage, 4 Max Correction, 5 I/P LV, 6 I/P HV, 7 O/P LV, 8 O/P HV, 9 Volt Cut Delay, 10 Overload Current, 11 Overload Cut Delay, 12 Overheat Trip Delay,
13 Phase Sequence (Industrial only), 14 Display Scroll (three-phase), 15 Scroll Time (if scrolling), 16 Password, 17 Restart Mode, 18 Restart Time (Auto only), 19 Re-tune, 20 Clear Fault Log, 21 Discard Changes, 22 Factory Reset.
MENU = next item (the edit is checked first), START = increment / toggle, STOP = next digit (the `^` marks it). An edit that breaks a rule ("Invalid Value": ranges, I/P LV < Set V < I/P HV, O/P LV < Set V < O/P HV) is put back.
A violation that was already there never traps you on an item; leaving Set-Mode checks everything and takes you to the offending item. MENU + LOG saves and leaves ("Settings Saved"); Discard Changes = Yes leaves without saving;
30 s without a key leaves without saving ("Timeout - Not Saved"). A password (000 = none) is asked for on entry. Calibration and Re-tune say "Not Available" until their stages exist. Factory Reset resets the settings only and takes effect on saving.

## Storage
Settings: two flash pages at 0x0801F000 and 0x0801E800 (power-cut safe, CRC, version); the fault log: 0x0801D000 and 0x0801C800 (reserved: calibration at 0x0801E000 / 0x0801D800). The old firmware's settings page, 0x0801F800, is never touched.
The flash driver only erases or programs those pages AND only above the end of your program image: if the program ever grows into them, saving fails with "Save Failed" instead of damaging code.
**Please check on the bench that your firmware image ends below 0x0801C800 (112 KB):** in CubeIDE look at the build size; the main loop code of the old firmware is still compiled in.

## Choices I made for the screens (please check on the LCD; all texts are in the tables at the top of `m_ui.c`)
The exact layout of the Run-Mode screens, the header of Set-Mode (name and item number), the cursor line, the fault-log layout and the message texts are mine: the manuals' exact screens were not available to me. Tell me what to change.
Items 13 to 15 and 18 are hidden when they do not apply; the item numbers are fixed, so numbers are skipped.

## Tests
`test_master_ui.c` (185 checks: keys with 3,000,000 random steps; exact screen text; every Set-Mode rule; the log viewer; passwords; timeouts; 2.4 million random UI steps) and `test_master_app.c` (67 checks: raw key timing,
settings surviving restarts, a failed save, a destroyed flash, 512 bit flips, every fault code on screen, 3.6 million random steps with failures and flash damage). 35 deliberate breakages of these modules are caught.


## Changes in v0.9.4
* **Set-Mode layout:** every Set-Mode screen is centred (title, item name, value) and the item number is gone from the first row. The cursor is a blinking underscore `_` under the digit being edited (shown for 0.5 s from each key press, then hidden 0.5 s, and so on). Messages are centred too.
* **One round:** MENU on the LAST item (Factory Reset) leaves Set-Mode and saves, like MENU+LOG; it no longer starts a second round. "Discard Changes" still leaves without saving. Factory Reset = Yes on the last item resets the settings and leaves, saved.
* **Max Correction** is not a tolerance: it is the largest voltage the series transformer is expected to ADD or SUBTRACT, in volts (0 = not set). It is used by the slave's transformer autotune (a later stage) to limit the test injection and to set the correction cap.
  It is not the accuracy band around the Set Voltage. The default stays 000 (Not Set).
* **The master now TELLS the slave** (`m_ctl`, `$M` frame every 100 ms and at once on a change): run / stop, the Set Voltage (tenths of a volt), the Max Correction and the clear counter.
  - Set-Mode (or its password prompt) opening = run 0 in the very same step: the slave stops its PWM pulses and trips the output. It stays stopped while Set-Mode is open.
  - A Set Voltage changed and saved in Set-Mode is sent to the slave; the slave takes it while it is stopped, so it is in force at the next start. An edit you abandon (timeout, Discard) is never sent.
  - Run: Auto = the master starts the slave by itself after the Restart Time (also at power-up and after leaving Set-Mode); Manual = it waits for START. STOP stops it until START.
  - A slave fault: the slave latches it until the clear counter changes. Auto: the master changes the counter every Restart Time while the fault is reported (the slave then tries again); Manual: the master stops, and START changes the counter.
    After each change the slave's old fault report is ignored for 1.5 s (it needs a moment to answer).
* **THE SLAVE MUST RUN THE MASTER-CONTROL BUILD.** A slave built with the normal config ignores `$M` (it autoruns at its compile-time 230 V and cannot be stopped by the master). Flash the slave with `slave_config_STAGE3_cap2752_full_range_MASTER.h`
  (or your own config with `SLAVE_LINK_V2 1`, `SLAVE_MASTER_CONTROL 1`, `SLAVE_BENCH_AUTORUN 0`). With that build the slave does NOT start by itself: it needs this master.
* The contactors are still not driven by the master (a later stage): "trip the output" here means the slave stops its PWM and opens its own outputs.


## Changes in v0.9.5: the master now runs the whole single-phase unit
The fault engine (`m_faults`) is now in charge, replacing the simple controller of v0.9.4. For a single-phase unit with one slave, the master now does everything of the agreed spec that does not need a second or third slave:
* **Contactor:** `CONT_1` (PA10) is driven by the engine. It closes only after the slave has been IN_BAND (from its `$E` frame) for the Restart Time, and opens at once on any trip, in Set-Mode, on STOP and when the slave's frames stop. "Output ON" on the screen now means the contactor is closed.
  **First test with the load disconnected, or set `MASTER_DRIVE_CONTACTORS 0` in `master_monitor.h` to keep the contactors open while you check everything else.**
* **Buzzer:** PB14 (high = on, as in the original firmware) sounds on a trip; STOP silences it.
* **Load current:** the master CT on ADC channel 0, every 200 ms, with the original firmware's averaging, scaling (65.25 / 766) and its 3 A zero offset (`MASTER_CT_OFFSET_DA`); it feeds the Overload fault and the Load line. The per-phase calibration (Set-Mode item 2) is still to come.
* **Faults, restarts and the log:** every rule of the approved spec (docs/MASTER_CORE.md): voltage faults with the 5 V hysteresis, Overload and Heat START-only, Inrush and Power-stage 3 retries 30 s apart then latched, the Mains group, link lost, Auto and Manual restart, the Restart Time as the on-delay. Every trip is written to the fault log (viewer: LOG key).
* **The slave's limits:** the master reads the slave's `$H` frame (input window and output guard) and Set-Mode refuses cut-offs the slave could never reach. The clear counter starts from the slave's own, and no `$M` is sent until the slave has been heard.
* **Needs:** the slave built with `SLAVE_MASTER_CONTROL 1` (slave_config_STAGE3_cap2752_full_range_MASTER.h).
**Still not in this release:** the Y and B slaves (Industrial and Domestic do not start: only one slave link exists), phase-to-phase voltage and phase sequence (they need the Y and B inputs), the overheat input, calibration, the autotune and the transformer-flip check (slave side), the master CT peak (inrush backup) and the running-hours counter (the log time restarts at each power-up).


## Flash size (v0.9.6): "region FLASH overflowed"
The STM32G070RB has 128 KB of flash. The original application (about 5,900 lines of `main.c`) plus the new code did not fit in a Debug build (-O0, debug info, float printf): the linker reported the region overflowed by 1,884 bytes.
**v0.9.6 removes the old application from the image in monitor/app mode** (`MASTER_MONITOR_MODE 1`): the old start-up and main loop in `main()` and the old 1 ms timer callback are inside `#if (MASTER_MONITOR_MODE == 0)`, and
`MasterApp_Run` / `MasterMonitor_Run` are `noreturn`. In that mode they can never run, so the linker (`--gc-sections`) drops everything only they used. With `MASTER_MONITOR_MODE 0` the original firmware is built exactly as before.
If you keep your own `main.c`, make just these changes in it: (1) put `#if (MASTER_MONITOR_MODE == 0)` before `Load_Settings_From_Flash();` in `main()` and `#endif` before the closing brace of `main()`; (2) put the same `#if` / `#endif` around the whole `HAL_TIM_PeriodElapsedCallback` function.
**Where the settings live:** the pages at 0x0801C800 to 0x0801F7FF. The program image must end BELOW 0x0801C800, that is `text + data` of the build must be under 116,736 bytes. The build prints the size at the end ("Master.elf  text  data  bss"); if the program is too large the app
shows "Flash Full - Settings Not Saved" at start-up and refuses to write (it never damages code). If it is still too big after this release, in this order:
  1. Optimise for size: Project > Properties > C/C++ Build > Settings > MCU GCC Compiler > Optimization > "Optimize for size (-Os)" (or build the Release configuration). This usually saves 30 to 40 percent.
  2. Remove float printf if nothing uses it: Project > Properties > C/C++ Build > Settings > MCU Settings > untick "Use float with printf from newlib-nano". (Saves roughly 8 to 10 KB. The new code does not need it; the original application, built only with MASTER_MONITOR_MODE 0, does.)
  3. Tell me the "text / data / bss" numbers (or send Master.map) and I will remove what is left.


## "It does not start": read the screen (v0.9.7)
The status row of the Run screen now says what the master is waiting for while the output is off, and a third MENU page shows the control diagnostics (MENU once = your monitor page, twice = the diagnostics, three times = back).
| status row | meaning |
|---|---|
| `Slave Link Lost` | no `$S` frames arrive: the slave is off, or the wire slave TX to master RX, or the baud rate |
| `Input LV` / `Input HV` | the input is outside the I/P limits (180 to 280 V by default; it must be 5 V inside to start) |
| `Start in 04 s` | the power-up on-delay (the Restart Time) is running (Auto) |
| `Press START` | Manual mode, or after STOP: START runs it |
| `Starting...` | the slave was told to run; waiting for it to reach the set voltage |
| `In Band: 04 s` | the slave is in band; the contactor closes when this reaches 0 |
| `No $E From Slave` | the slave was started but its `$E` frames are not arriving: it is not on the master-control config, or they are not understood. The contactor cannot close without them |
| `Output ON` | the contactor is closed |
Diagnostics page: row 1 `S:Y E:Y H:N M:00012` = status frames heard, `$E` heard, `$H` heard (the slave's limits), `$M` frames sent. Row 2 `U:STARTING run:1 c:00` = the unit state, the run level sent, the clear counter.
Row 3 `Vin 201.5 180-280` = the input voltage and the limits. Row 4 `Slv st:2 F:00 inb:1` = the slave's state (0 stopped, 1 starting, 2 running, 3 fault), its fault code, IN_BAND.
**If `run:1` and `M` keeps counting up but the slave does not run:** the `$M` frames are not reaching it (the wire master TX to slave RX) or the slave is not on the master-control config. **If `M` stays at 0:** the master is not sending (nothing heard from the slave yet).
Also new: `$M` is sent as soon as `$E` has been heard, OR after 3 s of `$S` frames even without an `$E`, so a slave whose `$E` is not understood still starts.


## Test switches: no voltage cuts, whole volts, a steady display (v0.9.8)
**For bench testing only.** The standard slave has its own voltage limits besides the master's settings: an input envelope (trip below 140 V or above 300 V, and a narrower start window of 150 to 290 V) and an output-high cut (`CTRL_VOUT_MAX_V`, 250 to 280 V depending on the config; the slave has no output-low cut).
* **Slave:** use `slave_config_STAGE3_cap2752_full_range_MASTER_TEST_NOCUTS.h` (in slave_config_ready_made.zip), which sets `SLAVE_TEST_NO_VOLTAGE_CUTS 1`: no input-range trip, no input start window, no output-high trip, and `$H` announces a 0 to 650 V window so Set-Mode accepts any cut-off.
  Kept: the hardware over-current protection (FLT, break input), the sensor and commutation checks, the relays, the zero-cross checks. The standard `_MASTER` file keeps every limit; the runner refuses the switch in any standard config.
* **Master:** `MASTER_TEST_NO_VOLTAGE_CUTS 1` in `master_monitor.h` (the default in this release): I/P LV, I/P HV, O/P LV, O/P HV and the input check before a start are off, and the Set-Mode values for them are ignored. Overload, heat, inrush, power-stage, mains and link-lost rules stay. The diagnostics page shows `NOCUT`.
  **Set it to 0 and use the standard slave file for any unit that is not on the test bench.**
* **Display:** every voltage is now whole volts (`O/P Voltage    230 V`, `R:  230V`, `R-Y :   400 V`, the log's `I 225 O 230`); the load current keeps one decimal. The display is filtered so the last digit does not flicker: a first-order filter of about 0.6 s that jumps at once on a step of 5 V or more,
  and the shown value changes only when the filtered reading is 0.7 V away from it (the load: 0.3 A). The limits and faults use the raw readings, not the filtered ones.


## The contactor closes when the output reaches the set voltage (v0.9.9)
Before: it waited for the slave's IN_BAND flag and then for the whole Restart Time (5 s), and needed the slave's `$E` frame. Now: the contactor closes once the output has been AT THE SET VOLTAGE (within 2.0 V, `MAPP_BAND_DV`) for 1 s (`MF_CLOSE_HOLD_MS` in `m_faults.h`; a debounce so a passing value does not close it).
"At the set voltage" is true when the slave says IN_BAND ($E) OR the master's own reading of the output is within 2 V of the Set Voltage while the slave is running, so it also works without any `$E`. While the output is rising, the status row shows `To 240 V (231)` (the set voltage and the output now), then `At Setpoint...`, then `Output ON`.
Everything that OPENS the contactor is unchanged (any trip, Set-Mode, STOP, a lost link). The Restart Time is now only the power-up on-delay and the wait before an automatic restart. To close with no wait at all, set `MF_CLOSE_HOLD_MS` to 0.


## Load-current calibration (v0.9.10)
The load shown (and used for the Overload fault) came from the original firmware's scale (65.25 / 766) and a fixed 3 A offset, which belong to another CT: with a 150 A / 30 mA CT it showed 3 A for a real 6.6 A. Now the load is
`current = gain x (CT count - zero) x 0.213 dA per count`, with a ZERO and a GAIN that you calibrate and that are stored in flash (pages 0x0801E000 / 0x0801D800; Factory Reset keeps them). The defaults reproduce the old behaviour, so nothing changes until you calibrate.
**Do it in this order:**
1. **Zero (no load):** open the output load (or make sure nothing draws current), then MENU + LOG for 2 s (Set-Mode opens the output anyway), MENU once to item "Calibration", START for Yes, MENU. The screen says `Zero Set`. It is refused (`Load Not Zero`) if the CT reads far above any zero.
2. **Span (with the load running):** let the unit run until `Output ON`. In Run-Mode press MENU three times: the Load Calibration page. It shows the Reading and an Actual value that starts from it. Type the CLAMP METER reading: START adds one to the digit under the blinking cursor (it starts on the units of amps),
   STOP moves the cursor. Press LOG: `Calibrated`. It is refused below 2.0 A of load (`Load Too Low`, as in the spec), for a value outside 2.0 to 180.0 A, and for a gain that no CT could give (`Out Of Range`: check the CT). MENU leaves the page (a fourth press returns to the Run screen).
   START and STOP on this page only edit the value; they never start or stop the unit.
The raw ADC count averaging is unchanged (200 samples every 200 ms). The ADC reads the average of the rectified CT signal, so a distorted (non-sinusoidal) load reads differently from a true-RMS clamp meter: calibrate with the typical load.
