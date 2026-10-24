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
30 s without a key leaves without saving ("Timeout - Not Saved"). A password (000 = none) is asked for on entry. Calibration works (Load Calibration page, Voltage Calibration pages); Re-tune starts a transformer sweep (see below). Factory Reset resets the settings only and takes effect on saving.

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


## "Slave Waits:..." (v0.9.12)
When the master has told the slave to run (`run:1` on the diagnostics page) and the slave answers "starting" (`Slv st:1`) but never reaches RUN, the slave is waiting in its start-up check. It says what for in its status frame, and the status row now shows it
as letters, for example `Slave Waits:FB` (the diagnostics page shows `W:FB`; your old monitor page, one MENU press, shows the same as `WAIT: F B`):
| letter | the slave is waiting for |
|---|---|
| `Z` | a validated zero-cross pattern (no or unstable mains zero-cross signal) |
| `V` | the input to be inside its start window (off in the NOCUTS build) |
| `M` | a valid voltage measurement |
| `F` | the FLT line (PD2, the over-current comparators) to go HIGH: it reads LOW |
| `B` | the break input (PB12 / PA9) to go HIGH: it reads LOW, **which includes a FLOATING input** |
| `O` | the output voltage to fall below its limit |
| `R` | the relays to settle in the shorted (bypass) state |
No letters (`To 230 V (216)`) means nothing is missing and the slave is moving through its start sequence. **`F` and `B` are protection: the slave refuses to start while an over-current / driver-fault line reads LOW.** After a power-stage repair a latched
gate-driver fault output, a floating break input or a comparator without its CT connected will do exactly that. Find out why the line is low; do not bypass it.


## "Inrush" on the screen, and the retries (v0.9.13)
The master shows **Inrush** when the slave reports its hardware break (code 1) or its FLT input (code 2): the over-current comparator chain fired. It does NOT necessarily mean a mains inrush; in a unit with a repaired power stage it can be a real over-current or a spurious trip.
Read the diagnostics page (MENU twice): `Slv F:02` = FLT, `F:01` = the hardware break; the fault log (LOG key) has the entry with the voltages and the load at the moment.
**Retries:** the approved rule retries an Inrush or power-stage trip after 30 s, up to 3 times, then latches until power off and on. Each retry re-energises the power stage, which is NOT safe while a repaired stage is being commissioned.
`MASTER_INRUSH_RETRIES` in `master_monitor.h` sets it: **0 (this release's default) = latch at the FIRST trip**, 3 = the approved rule. Set it back to 3 for a unit in service.


## Faster buck and boost, and "inverter off = load off" (v0.9.14)
**1. The speed problem was my injection guard (v0.9.11), and it had two defects.** After an input step the guard takes the correction to zero and restarts the regulator. (a) The restart used the compile-time slow ramp (2 s at 3 V/s first) and did not apply the stage-3 correction cap, so every recovery was slow
whatever the config said. (b) A direction mismatch re-armed a hold that was already running, while the regulator wound its output up to the cap during the hold: the guard never released, and after a 200 to 250 V step the output stayed at the input for ever (found in simulation).
Now: the restart uses a fast ramp (`CTRL_GUARD_RESTART_SLOPE_VPS` 30 V/s, final 10 V/s, no slow phase), applies the same cap as a normal start, and a mismatch cannot re-arm a running hold. Simulation (full-range cap): a 230 to 200 V step recovers in about 2 s (was 5 s), a 200 to 250 V step in about 1 s (it never recovered), eight fast swings never stick.
**2. The start-up ramp** (from the run command to the set voltage) is the configurable soft start: the standard 3 / 10 / 5 V/s takes 12 s from 175 V, 9 s from 205 V; the fast profile 10 / 30 / 10 V/s with 0.5 s takes 7.5 s and 6.4 s. About 5 s of that is fixed (the 3 s the converter runs alone, the sync and the relays).
Use `slave_config_STAGE3_cap2752_full_range_MASTER_fast_start.h`, or add the four `CTRL_SOFTSTART_*` lines of it to your own `slave_config.h`. A faster ramp is a faster rise of the LOAD voltage.
**3. The load contactor now opens when the inverter is off.** Until now the master opened it only on a slave FAULT CODE; a slave that stopped without one (mains unstable, its own link timeout ...) left the contactor closed. Now, while the output is live, the master opens the contactor
when the slave has not been RUNNING (state 2) for 0.2 s (`MF_INV_OFF_MS`), whatever the reason; it is logged as a Mains trip (no code) and restarts like the Mains group. STARTING, a 0.1 s glitch of the state, and a slave not being heard (that is the link-lost rule) are not affected.


## The master owns the voltage limits (v0.9.15)
**What you saw:** the unit tripped at once at "I/P LV / I/P HV / O/P HV" and seemed to ignore the Set-Mode limits and the Volt Cut Delay. The `full_range` slave configs narrow the slave's own envelope to 195 V in (low), 275 V in (high) and 265 V out (an old-firmware imitation).
Those limits trip immediately and are independent of the master, so with a master I/P LV of 180 V (below 195 V) or I/P HV of 280 V (above 275 V) the slave always acted first. The Set Voltage itself WAS taken: tested with the real master and real slave at 200 to 250 V, the output reaches each value.
**Now (option A):** the master-control configs (`..._MASTER.h`, `..._MASTER_fast_start.h`, `..._MASTER_TEST_NOCUTS.h`) no longer narrow the slave. It keeps its built-in outer envelope: input trips below 140 V and above 300 V (it STARTS only from 150 to 290 V), output trips above 280 V.
The master's four limits (I/P LV, I/P HV, O/P LV, O/P HV) with the Volt Cut Delay decide inside that envelope. The runner refuses a master-control config that narrows it again.
* **Set-Mode checks against the slave's envelope:** I/P LV below 140 V, I/P HV above 300 V or O/P HV above 280 V is refused (Invalid Value), once the slave's `$H` frame is known. If the master started after the slave it missed `$H`: the master now ASKS for it (`$K HELLO`, every 2 s while `$E` is heard and `$H` is not).
* **A fast cut for a gross output over-voltage:** an output more than **30 V** above the O/P HV setting for 0.2 s is cut without waiting for the Volt Cut Delay (v0.9.26; it was 15 V, which a variac step up could trip: the output follows the input through the series transformer until the correction brings it back, 0.5 to 1.5 s). Anything smaller waits for the Volt Cut Delay. The slave also stops at once above its own 280 V output limit (F06) or outside its 80 to 300 V input window (F09): those are not delayed.
  Between O/P HV and O/P HV + 15 V the normal delay applies. Low output voltage always waits the delay.
* **Diagnostics page (MENU twice), row 2:** `Vin 179 166-305 T230` is the input, the I/P limits and `T` = the set voltage the SLAVE is using (its `$E` echo). `T!230` means it differs from the Set Voltage in Set-Mode; `T---` means not known.
**What you must do:** copy `slave_config_STAGE3_cap2752_full_range_MASTER_fast_start.h` (the new one) over your `slave_config.h` and reflash the SLAVE; flash the MASTER too. The trade: the slave's own immediate limits are wider now, so the master's delayed limits and the fast output cut are what protect the load. Keep the O/P HV setting sensible (for 230 V: 250 V).


## "Mains Unstable" F16 / F04 in stable conditions: a rare glitch no longer opens the load (v0.9.16)
**What was happening (bench: 6 times in 10 minutes, mostly F16, once F04):** at every mains zero-cross the slave switches the converter's next diagonal on 40 us later. **F16** = the newest voltage sample then shows the OPPOSITE polarity (an early or noisy event), **F04** = the sample at the event is too small to read a sign.
Both used to latch the unit at the FIRST occurrence; in master-control mode the master then opens the load contactor ("Mains Unstable") and restarts after the Restart Time. Six events in about 60,000 half-cycles is a very rare noise spike at the zero crossing, not a systematic fault.
**Now (`SLAVE_GLITCH_SKIP 1`, set in the three master-control configs; 0 = the old behaviour, still the default of every other build):** while RUNNING the slave SKIPS that one half-cycle (nothing is energised: the same safe state the check enforced), counts it, and carries on. At most 2 per 2 s are tolerated; the third is the fault exactly as before,
so a real polarity problem (it fails on EVERY half-cycle) still stops the unit within about 30 ms. During a start the checks stay strict. For F04 the zero-cross tracker infers the alternating sign across the one unreadable event so the next event is not reported as F15.
Tested: one and two glitches tolerated, the third within 2 s is F16, a persistent tiny signal is F04 within 0.1 s, a glitch every 100 s for 20 simulated minutes never stops the unit, and with the real master six glitches in ten minutes leave the contactor closed with nothing logged while three in a row open it and log F16.
**This hides a symptom, not the cause:** something still disturbs the zero-cross sample about once every 100 s (noise on the AC_IN sensing or the zero-cross detector, a mains disturbance, ringing near the crossing). The skipped half-cycle costs one half-cycle of injection (10 ms). Look at the zero-cross detector output and AC_IN with a scope on a trigger of the slave's PB0 events to find it.
Flash the SLAVE (and use the updated config) for this change; the master is unchanged.


## First release of the settings round (v0.9.17): limits model, one Run screen, works from 90 V
**1. The limits model (Set-Mode).** The unit works from 90 V to 300 V of input. The WINDOW is **Set Voltage minus/plus Max Correction**, never outside 90..300 V (Max Correction 0 = not set: the whole 90..300 V; otherwise at least 10 V).
* **I/P LV** and **I/P HV** default to the window's edges. You can move them INSIDE the window, but never closer than **10 V** to the Set Voltage.
* **O/P LV** and **O/P HV** default **20 V inside** the input limits (O/P LV = I/P LV + 20, O/P HV = I/P HV - 20, but never closer than 10 V to the Set Voltage). You can move them between the input limit and 10 V from the Set Voltage.
* So: `window low <= I/P LV <= O/P LV <= Set - 10` and `Set + 10 <= O/P HV <= I/P HV <= window high`. Example, Set 230 V, Max Correction 50 V: window 180..280 V, defaults I/P 180-280, O/P 200-260. Max Correction 40 V: I/P 190-270, O/P 210-250.
* **Changing the Set Voltage or the Max Correction** moves the limits that were at their old defaults to the new defaults; a limit you moved yourself stays, clipped into the new window. The same happens if you leave Set-Mode with MENU+LOG straight after typing the value.
* **Set Voltage is now 100..290 V** (the limits must fit 10 V either side inside 90..300 V). With the slave's 280 V output guard the highest usable Set Voltage is about 270 V (O/P HV must be at least 10 V above it and not above the guard). Move the guard first if you need more.
* The default limits and Factory Reset are also kept inside what the slave announced in `$H` (window, output guard).
* **Max Correction must be set to the transformer's range** for the window to mean anything: left at 0 the window is 90..300 V, the O/P limits 110..280 V.
**2. The slave works from 90 V.** The three master-control configs now set `CTRL_VIN_MIN_V 80`: the slave's input trip is 80 V and it starts from 90 V (+10 V hysteresis); it announces 80..300 V in `$H`. The master's I/P LV can therefore be 90 V (it can still only START 5 V inside after a violation, so from 95 V).
**HARDWARE IS NOT CONFIRMED AT 90 V** (see the comment in the config): the zero-cross IC only worked above about 70 V on the bench, the power board's 12 V / 3.3 V rails and the relay coils at 90 V are untested, and the transformer's correction range does not reach the Set Voltage from 90 V: the unit then simply cannot reach its set voltage there.
The runner now refuses a master-control config whose input trip is above 80 V or that narrows the 300 V / 280 V limits.
**3. One Run screen (single-phase).** The Run screen shows O/P voltage, I/P voltage, load, frequency and, in the bottom row, the status or the fault; for Mains Unstable, Power Stage and Inrush the slave's code is at the right end (`Mains Unstable   F16`). MENU does nothing on it.
The service pages (monitor, diagnostics with the slave's echoed set voltage, Load Calibration) are reached through the log: **LOG, then MENU**; MENU cycles them (Load Calibration is the third: LOG, MENU, MENU, MENU), and they return to the Run screen by themselves after 60 s without a key or when a fault appears.
(Older sections of this document say "MENU x2 / MENU x3": read that as "LOG, then MENU x2 / x3".)
**4. Set-Mode stops the pulses.** Entering Set-Mode sends run = 0: measured in simulation the slave stops its inverter pulses in about 110 ms and the contactor opens at once.


## Transformer-connection (flip) check at every start (v0.9.18)
**What it was:** the old flip check existed only for the converter-only stage 1 (it refuses to compile on stage 3) and as a slow open-loop injection test in stage 2 that only left a verdict word. In the stage-3 / master-control firmware there was NO flip check, and the slave never raised fault 17 (the master already knew how to show and latch it).
**What it does now** (`SLAVE_TF_RUN_CHECK 1`, set in the three master-control configs): while the regulator starts up, the output must move in the direction it is told to. Before any correction the slave records the offset between its two voltage sensors (Vout - Vin). Then it watches the signed change in the commanded direction in 100 ms windows:
about +1 V = connected correctly (the check ends); about -1 V in two windows in a row, or -2.5 V in one = the series transformer is connected BACKWARDS: the slave stops with **fault 17, latched**. The master shows **Transformer Flip**, sounds the buzzer in every mode, logs it with code F17, and does not retry (latched until power off and on).
In simulation the output moves only 0.7 V (boost) or 1.5 V (buck) the wrong way before the stop.
**Limits to know:** (1) it judges only once the correction is large enough to see; a unit that needs no correction (input at the set voltage) is not tested, and is not at risk either, because a backwards transformer does no harm at zero correction. The check ends 60 s after the start. (2) The two-windows-in-a-row rule is a robustness setting
that the tests do not prove. (3) The sensor offset is recorded at every start; an offset larger than a few volts is tolerated, but a sensor that is badly wrong is not diagnosed here. (4) It was developed and tested against a simulated plant; the volts-per-tick of your transformer is not assumed (only the direction and a 1 V step are used), but it has not been run on your hardware.
**To test it on the bench:** swap the two leads of ONE transformer winding (power off, DC link discharged), start with an input that needs at least 3 V of correction: the unit must stop within a few seconds with "Transformer Flip" and F17.


## Real-time clock, Date and Time in Set-Mode, time in the fault log (v0.9.18)
**Hardware:** the master uses the STM32G070's RTC with the 32.768 kHz crystal (PC14 / PC15) and the backup battery on VBAT, so the time survives power-off. The driver (`master_rtc.c`) uses direct register access: it needs neither the HAL RTC module nor a CubeMX RTC configuration. `MASTER_RTC_ENABLED` in `master_monitor.h` (1 = on).
First start (or after the battery was lost): the crystal is started and chosen as the RTC clock; if it does not start within 3 s the internal LSI (about 32 kHz, NOT accurate) is used instead. The clock reads "not set" until you enter the date and time once; a clock that is already running is left alone at every later start.
**Setting it:** Set-Mode has two new items after **Restart Time**: **Date** (`dd/mm/yyyy`) and **Time** (`HH:MM`), before Re-tune. They start from the clock (or from 01/01/2026 00:00 if it was never set). START adds one to the digit under the blinking cursor (9 wraps to 0), STOP moves the cursor, MENU accepts.
A date or time that does not exist (month 20, 31 April, 29 February 2025, hour 24) is refused with "Invalid Value" and put back. Years 2001 to 2099 are valid. **The clock is written only when Set-Mode is SAVED** (MENU+LOG, or the end of the round), and only if you typed a change: passing through the items changes nothing, Discard Changes and the 30 s idle timeout do not write it. Seconds are set to 0.
**The log:** every new fault is stored with its date and time. In the log viewer (LOG key) row 3 shows `21/10/2026 14:32`; the load at the trip moved to row 2 (`L 12.3A`). An entry made while the clock was not set, or an older entry, shows `Run` and its running time as before. (The time is stored as minutes since 2000 with the top bit of the old run-minutes field set: the record format and its version are unchanged, so existing logs stay readable.)
**Set-Mode now has 21 items** in single-phase with Auto restart (24 in Industrial). Navigation follows the table order; the item numbers are unchanged (Date 23, Time 24).
**Not done:** no time zone or daylight saving, no drift calibration of the crystal. **Not proven on hardware:** the register driver is tested against a software model of the RTC, not on the real part: if the log shows no times after you have set them, or the time is lost at power-off, check the crystal and the battery, and `MRtc_UsingLse()`.


## The Clock page: "the time does not show in the log" (v0.9.19)
**First, the common reason:** only faults that happen AFTER the clock was set carry a time. Entries made earlier keep showing `Run` and their running time; nothing can re-stamp them. Set the clock, then make a NEW fault (or wait for one) and look at that entry (LOG key, entry 00 is the newest).
**If a new fault still shows no time,** the master is not getting a valid clock reading from the RTC. The new **Clock page** shows why: LOG, then MENU x4 (the fifth MENU press returns to the Run screen; it returns by itself after 60 s):
* row 0, 1: `Date 21/10/2026`, `Time 14:32` as the master reads them, or `Clock NOT SET` if it reads the RTC as never set (or lost);
* row 2: `Src:Crystal Set:Y`: the clock source the backup domain reports (`Crystal` = the 32.768 kHz crystal, `LSI !!!` = the internal oscillator, the crystal did not start: the time then drifts, `-------` = the RTC is not running) and whether the reading counts as set;
* row 3: `BD:8303 IC:00F0`: the raw RCC backup-domain register and RTC status register (low 16 bits), for diagnosis.
Read it right after you set the date and time and again a minute later, and send me the four rows if the clock is not set or the time does not advance.
The service pages are now five: monitor, diagnostics, Load Calibration, Clock (LOG, then MENU x1 to x4).


## Overheat, inrush restarts and the countdown (v0.9.20)
**Overheat (PA5, active low):** the pin must stay low for the *Overheat Trip Delay* (Set-Mode item 12) before the unit trips ("Heat", logged, buzzer per mode). While it is low nothing counts down. When it has been high for the *Restart Time* the unit restarts by itself (Restart Mode Auto);
in Manual only START restarts it. The third trip within 30 minutes needs START even in Auto (a failing fan or a blocked vent would otherwise heat the unit up again every few minutes). If your thermostat releases right at its trip point it will trip again after every restart: use a longer Restart Time or a thermostat with hysteresis.
**Inrush / power stage:** the first trip is retried after the Restart Time (never less than 30 s), up to the retry count (3 by default; `MASTER_INRUSH_RETRIES`). Then a 5-minute wait and ONE more attempt; if that trips too, the unit latches. START restarts at once. With `MASTER_INRUSH_RETRIES 0` (commissioning) the first trip latches until power off and on.
**The status row** shows `Inrush      Rst 4:32` (Power Stage, Heat likewise) while the automatic restart counts down; a latched fault shows its code.

## Voltage calibration (v0.9.20)
LOG, then MENU x5 = **I/P Calibration**; MENU = **O/P Calibration**; MENU again = the Run screen after the next page... (the seven service pages are: monitor, diagnostics, Load Calibration, Clock, I/P Calibration, O/P Calibration, back to the Run screen).
Each page shows the slave's reading (`Reading 226.8 V`) and the value you type (`Actual 230.2 V`): START changes the digit, STOP moves the cursor, LOG sends it. **The unit must be stopped**: while it runs the page says `Press STOP first` and STOP stops it.
Typing **000.0** and LOG resets that channel to nominal. Answers: `Calibrated  I/P +1.50 %` (the total correction against nominal), `No Steady Signal Min 50 V`, `Out Of Range Max +/-10 %`, `Stop The Unit First`, `Save Failed Flash Error`, `Not Supported Update The Slave`, `No Answer From The Slave` (after 3 s).
See docs/VOLTAGE_CALIBRATION.md for the bench procedure.


## How fast the numbers change (v0.9.29)
The input voltage, the output voltage and the load current are shown through a filter: the shown number follows the reading with a time constant of about 1 s, changes at most once every 300 ms, and moves only when the filtered reading is 0.7 V (0.3 A for the load) away from it. A step of 5 V or more (5 A for the load) is shown at once.
To change it: `MAPP_DISP_EMA_DIV` (96: larger = slower filter) and `MAPP_DISP_HOLD_MS` (300) in `m_app.h`. The load display can show 0.1 A below a true value (the hysteresis); that is by design.


## Converter Current page (v0.9.30)
LOG, then MENU x7: `Converter Current`, RMS and peak in amperes of the slave's CT2500 chain (net I-OUT), and a status line: `CT primary current`, `OVER RANGE (clipped)` or `Not measured` (an older slave, or `SLAVE_IOUT 0`). There are eight service pages now:
monitor, diagnostics, Load Calibration, Clock, I/P Calibration, O/P Calibration, Converter Current, then the Run screen. See docs/IOUT.md.


## Re-tune: the transformer autotune from Set-Mode (v0.9.33)
Set-Mode (hold MENU and LOG 2 s, the password if one is set), item 19 **Re-tune**, Yes. The unit must be stopped. The screen shows the sweep (`Step n of 10`, the injection of the last step); STOP aborts it; every other key does nothing until it ends. The result screen goes after 6 s or on any key.
A slave without the autotune answers `Not Supported`; one with `SLAVE_AUTOTUNE 1` but without `SLAVE_TUNE_MODE 1` answers `Tune Refused / No Tune Mode Yet`. With both, the unit starts itself (about 6 s), then the sweep runs (about 40 s in all). Max Correction (item 4) must be set before a sweep, and the input must leave at least 20 V of headroom under the output limit (`Input Too High / Lower The Mains`). See docs/AUTOTUNE_BENCH.md before the first run.


## The Transformer Profile page (v0.9.39)
LOG, then MENU x8: the numbers of the profile the slave's last autotune stored (cap, gain scale boost / buck, the input and Max Correction it was made with, the points), from optional fields of the slave's `$H` frame. MENU once more returns to the Run screen (nine pages: Run + eight service pages). See docs/AUTOTUNE_BENCH.md for what each number does.
