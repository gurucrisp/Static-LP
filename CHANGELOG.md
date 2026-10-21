# Changelog — Crisp static stabilizer firmware (slave STM32G0B1 + master STM32G070)

Each entry is a git tag. One tag = one delivered package. `python tools/which_version.py slave <your Core folder>` tells you which tag a folder on your PC matches.

## v0.9.13 — no automatic inrush retries while commissioning (current)
* The master showed "Inrush" (the slave's hardware break or FLT, codes 1 and 2) on a unit with a repaired power stage. The approved rule retries such a trip after 30 s, up to 3 times: every retry re-energises a stage that may still be faulty. NEW `MASTER_INRUSH_RETRIES` (default 0 = latch at the FIRST trip; 3 = the approved rule), engine `max_retries`,
  `MApp_SetInrushRetries`. The slave and the engine's default (3) are unchanged. Tests: engine 117 checks, app 190 (0, 1 and 3 retries; codes 1, 2 and 7).

## v0.9.12 — the screen names what the slave is waiting for
* From a video of the unit: the master commanded run (`run:1`, `$M` counting up), the slave answered "starting" with no fault and never started its converter, and nothing on the screen said why. The slave already reports its start-up wait reason (Z zero-cross, V input window, M measurement, F FLT low, B break input low or floating,
  O output too high, R relays) in its status frame. The status row now shows it (`Slave Waits:FB`) and the diagnostics page shows `W:FB`. See `docs/MASTER_UI.md`. Tests: app suite 186 checks.
* NO change to the slave's protection. `F` and `B` mean an over-current / driver-fault line reads LOW (or floats): the slave is right to refuse to start.
* The development repository was rebuilt from the delivered v0.9.11 package after a sandbox reset: the source is identical, the git history before v0.9.11 is not available.

## v0.9.11 — injection guard after the IGBT failure in a fast variac test
* A unit lost its inverter IGBTs when the input was swung back and forth quickly (v0.9.9, voltage cuts removed). By simulation: the boost/buck direction was reversed at the regulator's full correction in a single pass as the input crossed the set point (8 of 8 snaps), and the integral
  regulator wound up on a stale output measurement within 20 ms of a step. NEW `SLAVE_INJ_GUARD` (on by default): an input step of 15 V or more between RMS results, or a direction reversal above 96 ticks of correction, takes the correction to zero, holds it until the input has been steady 100 ms, and restarts the regulator from zero.
  `MASTER_TEST_NO_VOLTAGE_CUTS` is 0 (cuts ON) by default again, a build with it on says "TEST: NO V CUTS" at start, and the runner refuses unsafe defaults. See `docs/PROTECTION_PATH.md`, "Incident report": this is a mitigation of one reproducible cause, NOT proof; the FLT latency still has to be measured.
* Tests: a new harness group (8 variac snaps: no reversal at a large correction, output follows input, recovers to 230 V; fails 8 of 8 with the guard off); 61 groups pass.

## v0.9.10 — load-current calibration
* With a 150 A / 30 mA CT the load showed 3 A for a real 6.6 A: the scale and the fixed 3 A offset were the original firmware's. Now `current = gain x (count - zero) x 0.213 dA`, calibrated in two steps and stored in flash: ZERO from Set-Mode "Calibration" = Yes (no load),
  SPAN from the new Load Calibration page (MENU x3 in Run-Mode: type the clamp-meter reading, LOG saves). Refused below 2.0 A of load, outside 2.0 to 180.0 A, or for an impossible gain. The defaults reproduce the old behaviour. See `docs/MASTER_UI.md`.
* The glue now hands the application the raw CT count (average of 200 ADC readings x 16). The calibration is a separate flash record; Factory Reset keeps it.
* Tests: UI 254 checks, app 177 (including your 3.0 A to 6.6 A case, persistence, the three refusals and a failed save); 7 breakages caught.

## v0.9.9 — the contactor closes when the output reaches the set voltage
* The contactor closes once the output is within 2 V of the set voltage for 1 s (before: the slave's IN_BAND plus the 5 s Restart Time, and it needed `$E`). "At the set voltage" = the slave's IN_BAND OR the master's own reading while the slave runs, so it works without `$E`.
  The status row shows `To 240 V (231)` while rising, then `At Setpoint...`. Opening rules unchanged. See `docs/MASTER_UI.md`.
* Tests: the whole master against the real slave now checks the contactor closes with the output at 230 V; app suite 135 checks; 5 breakages of the new rule caught. A display bug found by the new test (the "To 240 V (231)" text overwrote its own bracket) is fixed.

## v0.9.8 — test switches: no voltage cuts; whole volts and a steady display
* TEST ONLY: `SLAVE_TEST_NO_VOLTAGE_CUTS` (slave) removes its input-range trip, the input start window and the output-high trip, and announces a wide window in `$H`; ready-made file `slave_config_STAGE3_cap2752_full_range_MASTER_TEST_NOCUTS.h`.
  `MASTER_TEST_NO_VOLTAGE_CUTS` (master, on by default in this release) removes I/P LV, I/P HV, O/P LV, O/P HV and the input check before a start. Hardware over-current, overload, heat, inrush and the other rules stay. A standard config must not carry the switch (runner check).
* Every voltage on the master is whole volts; the voltages and the load are filtered (about 0.6 s, immediate on a step of 5 V or more) with a hysteresis on the shown value, so the last digit no longer flickers. Faults use the raw readings.
* Tests: the slave starts at 130 V and 305 V and regulates to 295 V with the switch (the standard slave refuses both and trips at 295 V); the app suite 130 checks including 60 s of a reading alternating across a rounding boundary (at most one change on screen); 5 breakages caught.

## v0.9.7 — the screen says why it is not starting
* "Autostart and START do nothing" could have several causes (the slave not sending `$E`, input outside the limits, the master TX wire, the slave config) and the screen did not say which. Now: the Run screen's status row shows what the unit waits for
  (`Start in 04 s`, `Press START`, `Starting...`, `In Band: 04 s`, `No $E From Slave`, `Input LV` ...), a third MENU page shows the control diagnostics (frames heard, `$M` sent, unit state, run level, clear counter, the input against its limits, the slave's state and fault),
  and `$M` is also sent after 3 s of `$S` frames when no `$E` has been understood. See `docs/MASTER_UI.md`, "It does not start".
* Tests: app suite 114 checks, UI 240; the whole master against the real slave still 240.

## v0.9.6 — the build no longer overflows the flash
* The IDE build failed with "region FLASH overflowed by 1884 bytes": the original 5,900-line application was still linked although the monitor/app never returns to it. In monitor/app mode the old start-up and main loop in `main()` and the old
  1 ms timer callback are now excluded (`#if (MASTER_MONITOR_MODE == 0)`), and `MasterApp_Run` / `MasterMonitor_Run` are `noreturn`. With `MASTER_MONITOR_MODE 0` the original firmware builds as before. A screen warning ("Flash Full") appears if the
  image still reaches the settings pages. NOT measured here (no ARM toolchain): `docs/MASTER_UI.md` "Flash size" lists what to do if it is still too big (-Os, float printf).

## v0.9.5 — the master runs the whole single-phase unit
* The fault engine is now wired into the application: it decides run, the clear counter, the contactor (`CONT_1`, only after IN_BAND has held), the buzzer (PB14), the screen's fault text and the fault log. New: the master CT is read (ADC channel 0, same scaling as the
  original firmware, 3 A offset), `$E` and `$H` frames are parsed (IN_BAND, the slave's input window and output guard feed Set-Mode's checks), the clear counter is synced to the slave's, and `$M` is sent only after the slave has been heard.
  `MASTER_DRIVE_CONTACTORS 0` keeps the contactors open for a first test. See `docs/MASTER_UI.md`, "Changes in v0.9.5".
* Tests: the harness now runs the WHOLE real master application against the REAL slave controller over actual frame bytes (43 checks: Auto start, the contactor only after IN_BAND, Set-Mode by raw keys, a new Set Voltage regulated to, STOP/START, an inrush with
  its 30 s retries and the latch after the fourth, link loss, an input below the limit with hysteresis, Manual); the application suite grew to 97. 12 deliberate breakages of the wiring caught; the tests found four gaps of their own.
* NOT yet: the Y and B slaves (Industrial and Domestic cannot start), phase-to-phase voltage and phase sequence, calibration, the overheat input, the CT peak, a running-hours counter, the slave's autotune and flip check.

## v0.9.4 — the master now tells the slave; Set-Mode layout changes
* Request by the user: (1) the Set-Mode cursor is a blinking `_` instead of `^`; (3) the master's settings are transmitted to the slave; (4) after one round of the items, MENU leaves Set-Mode (saving) instead of starting again;
  (5) Set-Mode screens centred, no item number; (6) opening Set-Mode stops the slave's PWM and trips its output. (2) Max Correction is explained in `docs/MASTER_UI.md`: it is not a tolerance, the default stays "Not Set".
* New pure module `m_ctl` and the `$M` transmit in `MasterApp_Run`: run level, Set Voltage, Max Correction, clear counter. Needs the slave built with `SLAVE_MASTER_CONTROL 1`.
* Tests: `test_master_ctl.c` 44 checks; the UI suite 239; the application suite 86; the slave harness gained "the real master logic against the real slave" (27 checks: Auto start, Set-Mode stops the PWM in the same tick, new settings
  regulated to, a trip cleared by Auto or waiting for START in Manual, STOP). 15 deliberate breakages caught. The fuzz found a real flaw in my first control logic: right after a restart the slave still reports its OLD fault, which could cancel START in Manual
  mode or bump the counter twice; a 1.5 s settle after each counter change fixes it.

## v0.9.3 — build fix for the STM32G070 HAL
* `m_flash_g0.c` used `FLASH_FLAG_ALL_ERRORS`, which the G070 HAL does not define (the G0B1 headers used for the PC-side compile check do): the IDE build failed with 4 errors. The error mask is now built from the individual
  flags, each only if the HAL has it. The runner now rejects that name in the sources. Nothing else changed; the other warnings in the build log are in the existing `main.c` and `button.c`.

## v0.9.2 — master screens, keys and Set-Mode
* Roadmap stage 7b. The master is now a flashable APPLICATION (`MASTER_APP_MODE 1`, inside the monitor build): Run-Mode screens, the fault-log viewer, and Set-Mode with all 22 items, validation ("Invalid Value"), password,
  30 s idle timeout, save by MENU+LOG, settings kept in flash. It DISPLAYS and CONFIGURES only: the contactors stay OFF and no slave is told to run (stage 7c). See `docs/MASTER_UI.md`.
* New pure modules `m_keys` (debounce, short press on release, the 2 s MENU+LOG combination), `m_ui` (every screen and the editing logic), `m_app` (settings + log + UI + keys); hardware glue `m_flash_g0` (refuses to touch anything
  but its own pages or any page below the end of the program image), `m_board.h` (key pins), and `MasterApp_Run()` in `master_monitor.c`. Single-phase MENU toggles to the old monitor diagnostics.
* Tests: `test_master_ui.c` 185 checks, `test_master_app.c` 67 checks; 35 deliberate breakages caught. They found real bugs in my first version (overlapping text on the single-phase status row, a lone dash for a missing reading, and a design flaw
  that could trap the user on item 1 when the slave's guard made a default value invalid: an edit is now refused only if the edit itself causes the violation).
* `MF_SlaveFaultType` is shared by the engine and the screens. `SLAVE_FW_VERSION` 902 (slave unchanged).

## v0.9.1 — master core: storage, settings, fault log, fault engine
* Roadmap step 7a. Four new pure-C modules in `master/Core` (`m_store`, `m_settings`, `m_log`, `m_faults`), NOT yet called by the master's `main.c`: nothing changes on the bench. See `docs/MASTER_CORE.md`, which also lists the eight choices to confirm.
* Tests: `test_master_store.c` (83 checks: a power cut at every point of a save and of a log append incl. the page erase, 448 single-bit corruptions, failed writes without a restart) and `test_master_faults.c` (111 checks: scenarios against a slave model, the exact display priority, 1.2 million random ticks with four invariants). Twenty-two deliberate breakages caught; the tests found three gaps of their own (a failed write without a restart, the full priority order, and wrong expectations on clear-pulse counts and retry timing).
* `SLAVE_FW_VERSION` is 901 to match VERSION (the slave's behaviour is unchanged).

## v0.9.0 — master control in the slave
* Roadmap step 3 (decisions D1, D5, D9, D12). New `slave_link2.c/h` (pure logic: `$M` dispatch, `$K` with retry de-duplication, `$H` x3 at start and on HELLO, `$E` every 100 ms, `$R` first) and thin glue in `slave_link.c`: a receive interrupt (USART1, new lowest level `IRQ_PRIO_LINK`) into a ring, the USART FIFO, reset-cause capture.
* `SLAVE_MASTER_CONTROL`: no autorun, run is a level, EVERY fault latches until the clear counter changes, 1 s link timeout, set voltage adopted while not energised, `$E` flags RUN_ACK / IN_BAND / AT_CAP / LINK_LOST / FAULT_LATCHED. Both switches are OFF by default: every older config behaves exactly as before (the 48 older groups pass unchanged).
* New `slave_features.h` holds the defaults and guards of these switches. The ready-made configs replace `slave_config.h` as a whole, so defaults placed there would have vanished and broken every older config. All 20 older configs plus the new `slave_config_STAGE3_cap2752_full_range_MASTER.h` compile strictly against the real sources.
* The controller's compile-time `CTRL_TARGET_V` became a runtime value (a constant in builds without master control).
* Tests: `s3_master_control` (170 checks incl. an end-to-end group through frame bytes), `test_slave_link2.c` (50 checks). Fifteen deliberate breakages caught; three gaps found by that and closed (IN_BAND never checked clear, the first-counter rule, a wrap test that only hit one alignment). `tools/check_version.py` keeps `SLAVE_FW_VERSION` equal to VERSION; `check_irq.py` knows the new level.
* Not yet: `$K` commands other than HELLO (UNKNOWN_CMD), Max Correction use, reach, heat/flip/not-tuned flags (steps 4 to 6). The master firmware does not use any of this yet (step 7).
* See `docs/MASTER_CONTROL.md`.

## v0.8.5 — the shared link library, version 2
* Decisions D1, D2, D12 (approved), roadmap step 2. New SHARED source `link_v2.h` / `link_v2.c` (identical in the slave and master projects, checked by the runner): typed builders for `$E`, `$H`, `$R`, `$T`, `$M`, `$K`, one parser for every frame type (`$S` included, delegated to version 1), and a stream receiver. NO change in what the slave sends or does: nothing uses it yet (step 3).
* Version 1 stays frozen: the `$S` frame is unchanged. One change to `link_proto.*`: the v1 receiver now counts a well-formed frame of ANOTHER type in a new `frames_other` instead of `frames_bad`, so a v1 master next to a v2 slave does not raise its error counter.
* The documented range of every field is one table per frame type, used by the builders and the parser, so they cannot disagree; the checksum is only 8 bits, so the ranges are the second line of defence for `$M`.
* `tests/test_link_v2.c`: 304 checks. Every example of the approved workbook parses and rebuilds byte for byte (the examples are generated from the workbook by `tools/gen_link_examples.py`); every field's limits; malformed input; every `$K` command; the stream receiver; 4 million random bytes; 600,000 random frames in and out of range; valid-checksum frames with random edits; ALL 63,240 single-byte substitutions of the examples (none accepted with different content); lengths and field counts; version-1 compatibility. Nine deliberate breakages of the library are caught.
* Found by the tests: a type letter alone was not classed as an unknown type (minimum length off by one); returned frames carried uninitialised padding bytes (now zeroed); my first fuzz never reached the accept path (random bytes almost never form a frame), so the structure-aware property and edit tests were added; the `$K` unknown-command frames are the one frame the parser accepts and the builder refuses, by design.
* See `docs/LINK_V2.md`.

## v0.8.4 — the FLT shutdown comes first
* Decision D25 (approved). The slave CT's over-current comparators reach the CPU on the FLT pin (PD2, EXTI line 2), a SOFTWARE input, not the timer's hardware break; the CPU has no analog primary-current signal. So the delay from a trip to the gate drivers going low is the interrupt latency plus whatever the handler does first, and before this version that was the HAL handler, three counters and `CaptureTrip()`, with FLT sharing priority 0 with four other interrupts.
* `EXTI2_3_IRQHandler`: `SlavePower_AllOffNow()` is the first statement. `SlaveCtrl_OnFltIrq()` also starts with it (the PC tests call it directly). The counters, the trip record and the fault flag follow unchanged (`CaptureTrip` reads only a timer count and two controller fields).
* New `slave_irq_prio.h`: FLT level 0 alone; control timer, zero-cross and both break handlers level 1; measurement timer level 2 (it was 1). Every `HAL_NVIC_SetPriority` uses these names; the header has compile-time ordering checks.
* New bench switch `BRINGUP_FLT_TEST` (stage 0 with the debug PWM only, `#error` with a real power stage) and the config `slave_config_SCOPE_flt_latency_NO_MAINS.h`: a FLT edge switches the debug PWM off, so the delay can be measured with NO MAINS. Found while building it: the FLT handler did not exist in a stage-0 build (it was inside the stage-1-and-up block), so arming the interrupt there would have ended in the default handler, an endless loop. `SlavePower_FltInit()` is now its own function and the handler exists whenever FLT is armed.
* Tests: a probe in the harness stub proves the shutdown comes before any counter or the trip record (fails 3 checks with the old order); `tools/check_irq.py` (new, in the runner) checks the priority names, that FLT is alone at the top, and that the shutdown is the first statement of both FLT functions (caught 5 deliberate breakages). Each stage build compiles clean against the real headers, and the preprocessed handler was checked: present with the shutdown first in exactly the builds that arm FLT.
* NOT measurable on a PC: the timing. My estimates (firmware part about 3.5 us before, about 0.6 us after) are in `docs/PROTECTION_PATH.md` with the scope procedure; please measure it and the primary current rise.

## v0.8.3 — the soft-start ramp is configurable
* Bench (v0.8.2, full-range file): switched on at 205 or 210 V, the output reaches 230 V after a few seconds; the correction seems very slow. It is the old firmware's soft start (3 V/s for 2 s, 10 V/s, 5 V/s for the last 3 V): from RUN, 4.1 s from 205 V, 3.6 s from 210 V, 2.5 s from 220 V (real regulator, measured plant); the whole switch-on is about 9 s including 2 s of relays and the 3 s converter-alone delay.
* The ramp's three slopes and the slow time are now per-instance settings of the regulator (`Reg_t.ss_*`, set by `Reg_Init` to the OLD constants) that the controller overrides from `CTRL_SOFTSTART_SLOW_SLOPE_VPS / FAST_ / FINAL_ / SLOW_TIME_MS`. The defaults are the old values: the regulator is still BIT-IDENTICAL to the old controller (equivalence test). A setting that lives in `slave_config.h` is invisible to `slave_reg.c`, which is why it is passed at run time and not as a macro there.
* New config `slave_config_STAGE3_cap2752_full_range_fast_start.h`: 10 V/s for 0.5 s, 30 V/s, 10 V/s: 1.4 s from 205 V, 1.2 s from 210 V, 0.9 s from 220 V; no overshoot and no hunting in the study. The 3 s converter-alone delay is unchanged. The old values were chosen for a loaded unit: no-load only so far.
* Tests: `tests/test_reg_softstart.c` (the ramp against an independent model: old and fast settings, 205 to 260 V, upward and downward, the defaults are the old constants) and a controller-level group in two new runner jobs (default and fast ramp): the time until Vout is within 1.5 V of 230 V and the time of the last 3 V must follow a model built from the same macros, so a setting that never reaches the regulator fails. Six plumbing breaks are caught (each setting not passed on, the whole override, the regulator ignoring its fields). The tolerance (-0.2 / +0.25 s) was chosen from the measured offset of the correct code (+0.03 to +0.06 s) against the mutants (+0.7 to +1.4 s): my first, looser tolerance let two of them through.
* Firmware: `slave_reg.h`, `slave_reg.c`, `slave_ctrl.h`, `slave_ctrl.c`. The master is unchanged.

## v0.8.2 — the measured gain curve: what a cap is really worth
* Bench: stage 3 with the wider file (cap 1600 ticks, v0.8.0) gave about 20 V of boost and buck; the stage-2 5 % test (160 ticks, v0.8.0) read +2.8 V at 200 V and +3.4 V at 250 V. Together with 64 ticks (1.9 V / 2.1 V) this is the real curve: STEEP over the first 64 ticks, then about 0.009 V per tick at 200 V and 0.0135 V per tick at 250 V (proportional to Vin): about 15 to 17 % of Vin per unit duty, and the old firmware's regulator constant 0.9/5.78 = 0.156 matches. At the 90 % duty ceiling the correction is about 27 V at 200 V and 38 V at 250 V (extrapolated): the unit regulates a window of about 202 to 270 V, roughly +13 % / -15 %, not 180 to 280 V.
* CORRECTIONS to what I told you: (1) the 0.03 V per tick of the 2 % test must not be extrapolated to large duty (the earlier "+/-35 to 40 %" capability and "about 47 V at 1600 ticks" were wrong); (2) the first closed-loop report ("8 to 10 V") was the 640-tick cap saturating (7.5 to 9.6 V), not demand inside the range, so my reading of it as plain success was wrong; (3) the first-run config, its comments and the cap comment in `slave_ctrl.h` said "about 19 V" for a 640 cap: it is about 8 to 10 V (comments only, no behaviour change).
* The regulator against the MEASURED curve (new permanent test `tests/study_reg_curve.c`, real regulator, both assumptions for the unmeasured first 64 ticks, inputs 222 to 260 V): all 18 runs settle within 0.1 V with at most 10 ticks of duty wobble: no hunting near 230 V, where the corrections are 1 to 3 V and the curve is steepest.
* New configs: `STAGE3_cap2000` (reaches 230 V from about 208 to 259 V, Vout guard 260 V) and `STAGE3_cap2752_full_range` (202 to 270 V, the duty ceiling, Vout guard 265 V; the 250 V guard of the first two files trips on a sudden input rise during a hard boost). The first-run and cap-1600 files are unchanged in behaviour; their descriptions now give the true correction. The runner guard checks the cap against the file name and that an output guard of 250, 260 or 265 V exists.
* NOT known: the plant with a LOAD (the dead time acts as an error in the direction of the current; everything here is no-load), the shape of the first 64 ticks, and the curve above 250 V. Firmware behaviour is unchanged since v0.8.1 (only a comment in `slave_ctrl.h`).

## v0.8.1 — the regulator's state on the master display
* Bench (v0.8.0, first-run file): boost below 230 V and buck above it, correcting by about 8 to 10 V: the closed loop works on the real hardware. What the display could not show is how much of the cap is used, and therefore the plant gain at larger duty (only 2 % pulses were measured).
* Display only, no power-stage change: at stage 3, in RUN, the slave sends a regulator word in diag2 (bit 12 = 1, bits 15..13 = 0; bit 11 boost, bit 10 AT THE CAP, bits 9..0 = effective ticks / 4) and the master shows it on row 3 every other 2 s as `BOOST 464t` / `BUCK 312t` / `FREEZE`, plus ` CAP` at the cap. The word is 0 outside RUN (after a fault it disappears: no stale duty). It cannot be mistaken for the transformer, mapping or trip words (each has a higher bit set). SLAVE AND MASTER both change.
* Tests: slave (boost / buck / frozen / at the cap match the real duty, absent after a fault) and master (every page, the largest value, TF page not mistaken for it, the fault page wins); three broken master variants are caught.
* Docs: how to read the new row, and the request to measure the plant gain at large duty at 5 or 6 input voltages.

## v0.8.0 — stage 3 (the closed loop) with the legacy relays
* Bench: stage 2 proved the transformer and the legs: `TF OK` +0.7 V at about 110 V and +1.9 to +2.1 V at 200 to 250 V, boost and buck both in the right direction. That gives the plant gain the regulator header asks for: about 0.011 / 0.030 / 0.033 V per effective tick at 110 / 200 / 250 V, buck and boost nearly equal.
* Stage 3 is released for `SLAVE_RELAYS_MODE 2`. The regulator's output is now the EFFECTIVE on-time; the CCR is dead time + that (`CTRL_EFF_TO_CCR`, shared with the stage-2 test); `CTRL_STAGE3_MAX_EFF_TICKS` (default 640 = about 19 V of correction) caps it and clamps the integrator (anti-wind-up). The first-run configs also trip at Vout above 250 V and limit the input window (190 to 260 V; wider file 170 to 290 V with a cap of 1600).
* NEW study, now a permanent test (`tests/study_reg_plant.c`): the real regulator module against the measured plant (gain curve, dead time, 20 ms RMS delay, noise, duty once per half-cycle), at 0.5x / 1x / 2x of the measured gain: constant inputs settle within 0.2 V, 10 V/s variac ramps across 230 V are followed within 1.2 V, instantaneous 30 to 40 V steps recover in 0.1 to 0.2 s. FINDING: the regulator schedules its buck gain 1.54x the boost gain for the OLD transformer; the measured gains are equal, so the buck loop is about 1.7x more aggressive than designed. It keeps about 2x margin and oscillates at 3x (0.1 V per tick). The regulator gains are NOT changed (a test proves the module identical to the old controller).
* SAFETY FIX: the output contactor pins (CONT1 = PC0, active low; CONT2 = PC1) were never configured in this firmware, so the output triac stage was left to the board's resistors. They are now driven like the OLD firmware's start-up: PC0 HIGH (OFF), PC1 LOW. Contactor control and what CONT2 drives are still unknown / not implemented. A source guard in the runner keeps it.
* Tests: a new legacy stage-3 group (regulation to 230 V from 215 V boost and 245 V buck, freeze band, 10 V/s ramps across the crossover, saturation at the cap and recovery without wind-up, hard fault / soft fault ordering / stop / retry) against the measured plant; the whole existing suite also passes in the stage-3 legacy build; seven of eight broken variants are caught (the eighth, removing the explicit clamp on the commanded duty, is equivalent: the integrator clamp already bounds it). The harness now models the dead time by default. TRAP FOUND: the harness skips any group named "closed loop" when `SKIP_SLOW=1` (the runner's quick mode), so my first stage-3 group was silently skipped; renamed.
* Configs: `STAGE3_first_run_cap640` and `STAGE3_wider_cap1600`; the runner guard checks stage, autorun, cap and the 250 V output guard against the file name.
* Not done: contactor control and the load soft start, the load itself, master run / stop commands, calibration, the plant gain at large duty, what happens to a LOADED series winding when the primary opens at a fault.

## v0.7.1 — fix: the stage-2 test duty was below the dead time, so the inverter output NOTHING
* Bench (v0.7.0, "1 %" at 100 to 120 V): `TF ? +0.0V`. Cause (mine): the dead time is 2.0 us = 128 timer ticks (`POWER_DEADTIME_TICKS`, new) and the 1 / 2 / 3 % test duties were 32 / 64 / 96 ticks. A high-side pulse shorter than the dead time is not generated at all, so the whole ladder produced no output. The simulation did not model it.
* `CTRL_STAGE2_DUTY_COUNTS` is now the EFFECTIVE high-side on-time in ticks (default 64 = 1.0 us = 2 %, limit 160 = 5 %); the controller commands CCR = dead time + effective (zero stays zero; the step from 0 is one tick of real output: continuous). The ladder is 2 % / 3 % / 5 %: the narrowest pulse is 1.0 us.
* Tests: the plant models the vanishing pulse (`W.plant_dead`), a new invariant fails if stage 2 ever commands a nonzero CCR at or below the dead time, a new dead-time test, expectations derived from the setting; the exact v0.7.0 bug is caught (19 failing checks) and so is a dead time added only to the hold. The stage-3 (mode 1) tests are unchanged: the regulator will need the same modelling before stage 3 goes live on the legacy relays.
* Configs: `STAGE2_inverter_120V_2pct / 3pct / 5pct` and `normal_2pct` replace the 1 % files (the runner guard checks stage, autorun, duty and window against the file name). Docs: what the bench showed, what to see on the scope (a 1.0 us high-side pulse at 20 kHz in the active leg), and the rule that exactly `0.0` means no measurable effect, not a failed measurement.
* Firmware: `slave_power.h` (the tick macro), `slave_ctrl.h` (the setting's meaning and default), `slave_ctrl.c` (INJ_CCR). The master is unchanged.

## v0.7.0 — the inverter, stage 2, with the legacy relays
* Bench: the converter-only stage works (positive half first, first diagonal at the line peak). Next step: the inverter. It has never run on live IGBTs in this firmware.
* Stage 2 now works with `SLAVE_RELAYS_MODE 2` (the proven sequence): lows on, SH, W on, converter, 3 s later the inverter (already enabled at zero duty by the legacy low sides: not re-initialised), RUN. Stage 3 is still refused for the legacy relays. `SLAVE_LEGACY_LOWSIDES`, `SLAVE_CONV_FIRST_POL` and `SLAVE_CONV_FIRST_AT_PEAK` apply to stages 1 and 2.
* Found by reading the old code: boost leg selection matches (TIM15 positive half, TIM1 negative half), PWM period matches (3200 counts); the dead time did NOT: the old firmware runs 2.0 us (DTG 128) and this firmware 1.0 us. Now `POWER_DEADTIME_DTG 128` and `POWER_BREAK_FILTER 4` (old: 4).
* Two safety fixes the legacy mode needed before an inverter could run: (1) a soft fault or stop with the inverter active now unwinds in the controlled order (duty zero, coils released, inverter disabled only after the W contact has closed; new `SlaveRelay_UnwindDone()`), because in legacy mode "shorted" only meant "the sequence is complete"; (2) the relay hold is no longer called during the unwind (it would have re-armed the SH pulse).
* Stage 2 is the one-shot injection test (`CTRL_INJ_*`): 1.5 s zero-duty baseline, 2 s ramp to `CTRL_STAGE2_DUTY_COUNTS` (default 32 = 1 %, was a fixed 96), 5 s hold, 1 s down, then zero. Verdict = the change of (Vout - Vin) against the baseline in the expected direction (OK / FLIPPED / inconclusive, dead band 0.3 V, abort at 15 V), sent in the existing diag2 word, shown by the existing master code (`TF OK +x.xV`). The master is unchanged.
* New ready-made configs: scope dead-time check (stage 0, 50 % debug PWM, NO MAINS), stage 2 at about 120 V with 1 / 2 / 3 % duty, stage 2 at normal input with 1 %. The runner guard checks stage, autorun, duty, window and PWM mode against each file name.
* Tests: a new focused group for the legacy inverter (start order, zero duty at RUN, inverter enabled once, phases of the test, boost / buck / reversed / tiny / 2 V sensor offset / strong transformer, hard fault, soft fault ordering and timing, stop without relay re-arm, restart); the whole existing suite also passes in the stage-2 legacy build; ten broken variants are caught. The old stage-2 test was updated for the ramp. The mode-1 and manual-relay stage-2 and stage-3 builds are unchanged and pass.
* Not done: stage 3 with the legacy relays, the output contactor and load soft start, master run / stop commands, calibration, what happens to a LOADED series winding when the primary is opened at a fault (the old firmware has the same exposure).

## v0.6.15 — 3 s retry configs and the first-diagonal-at-the-peak experiment
* Bench: toroid reliable up to about 230 V (1 s W settle), 250 V (5 s), 260 V (10 s); a fit gives a ceiling near 265 V, so waiting cannot reach 300 V. The 10 s between retries is the lockout `CTRL_RESTART_STEP_MS`.
* New `SLAVE_CONV_FIRST_AT_PEAK` (default 0, stage 1 only): the converter's FIRST diagonal is turned on at the line peak (a quarter cycle after the zero-cross event, 5000 us measured in simulation) instead of 40 us after it; later diagonals unchanged. Hypothesis (not shown): no step between the DC link and the line at the peak. Firmware change: `slave_ctrl.c` (one arming delay) and `slave_config.h` (the setting).
* Ready-made configs: `slave_config_normal_settle5s_retry3s.h`, `slave_config_PEAK_settle1s_retry3s.h`, `slave_config_PEAK_settle5s_retry3s.h` (autorun 1, retry 3 s, latch after 10 trips). While making them my generator shipped three files with autorun 0 (they would never have started the converter); the effective-value check caught it before delivery. The runner now guards it: every ready-made config must pass the structure checker, `SLAVE_BENCH_AUTORUN` must match the file name (only `*autorun0*` may be 0) and each file must contain the settings its name promises.
* Tests: peak start (first diagonal 5000 us after its event, second 40 us, also after a retry; three broken variants caught), quick retry (nine 3 s lockouts, the 10th trip latches), the whole suite also passes with the peak option on.
* `tools/check_config.py` prints the key experiment settings it finds and a NOTE for configs without `SLAVE_CONV_FIRST_AT_PEAK`.

## v0.6.14 — 10 s settle experiment config
* Bench: with the 5 s W-RLY settle the toroid starts reliably up to about 250 V (standard 1 s: about 230 V); above 250 V the problem continues. The video also showed self-restarts of the board above 245 V (F16 with Vin / Vout about 1 V, then NO LINK, then a boot) and one F01 at the first positive diagonal with FLT / break events counted while idle.
* New `configs/slave_config_normal_autorun1_settle10s.h` (`SLAVE_LEGACY_W_SETTLE_MS 10000U`) to test a falsifiable prediction (limit moves to about 275 V if it is a slow bleed-down; does not move otherwise). The focused PC test passes at 10 s (first diagonal 10006 ms after W on). No firmware change since v0.6.12.

## v0.6.13 — experiment config for flaky starts of the toroid above 230 V
* Bench: EI and toroidal transformers both work from 90 V to 230 V; the toroid above 230 V starts at once / after retries / never, then runs without problems. The transformer-connection test is postponed to stage 2 (it needs the inverter).
* New `configs/slave_config_normal_autorun1_settle5s.h`: W-RLY settle 5 s before the converter starts (`SLAVE_LEGACY_W_SETTLE_MS 5000U`), to test the hypothesis that the DC link voltage at the first turn-on causes the trip. No firmware change.
* New PC test (own runner job, longer settle): the first diagonal comes 5000 ms plus at most one zero-cross wait after W on, SH first then W, low sides on at the first diagonal, no fault for 6 s, W stays on; a variant that ignores the setting is caught.
* Firmware sources unchanged since v0.6.12.

## v0.6.12 — fix: the transformer-connection result is now actually sent
* v0.6.11 never showed the result on the master: the link sent the passive mapping-check word whenever it was non-zero, and those counters are never cleared (they keep their value from the idle period before the start), so the transformer result was shadowed for good. The slave tests read the result directly and the master tests used made-up values: nothing tested what the link sends in between.
* New `SlaveCtrl_Diag2()` holds the policy in one testable place: trip capture in FAULT > transformer result while the converter runs (no stale mapping word then) > passive mapping check. New test follows every phase, with the precondition that the stale counters really are non-zero; against the v0.6.11 logic it fails (stale `0x7D80` sent while `0x281E` was ready).
* `tools/check_config.py` prints a NOTE when a config predates `SLAVE_TF_CHECK` (the check would be silently off).
* SLAVE ONLY: `slave_ctrl.c`, `slave_ctrl.h`, `slave_link.c`. The master and the config are unchanged from v0.6.11.

## v0.6.11 — transformer-connection ("flip") check on every converter start
* New `SLAVE_TF_CHECK` (default on with `SLAVE_RELAYS_MODE 2` at stage 1): after the converter has really been running for 1.5 s the slave averages (Vout - Vin) over 2 s; the master shows `TF OK +3.2V` / `TF FLIP -3.0V` / `TF ? +0.2V` on row 3 every other 2 s. One result per converter start, survives a trip, replaced at the next start, cleared at power-up. Passive: nothing extra is driven.
* CORRECTION to what I told you about the old test: it does NOT inject. It closes W-RLY, runs the converter, and compares Vout with Vin once (`Vout - Vin > 0` = normal), reporting only. The physics of why that depends on the transformer polarity is not understood: the new docs give a two-step validation (as installed, then one winding swapped) and say what to do if the sign does not follow.
* Link: optional `diag2` bit 13 (bits 15 / 14 clear) carries the verdict (bits 12..11) and Vout - Vin in 0.1 V (bits 10..0, two's complement). MASTER AND SLAVE both change.
* Tests: slave (verdicts, dead band, clamping both ways, settle time really waited, cleared at power-up, survives a trip, re-measured at the next start, no verdict if stopped early) and master (every page; mapping, trip and fault pages are not mistaken for it); every broken variant caught except the unreachable minimum-sample guard (the stale-measurement fault trips first).
* Ready-made configs regenerated (an older config does not define `SLAVE_TF_CHECK`, so the check would stay off): `configs/` now has 90 V step 1, 90 V step 2 and a normal-window autorun-1 file.

## v0.6.10 — config checker and ready-made 90 V configs
* Cause of a failed bench build: `slave_config.h:31: error: unterminated #ifndef` = the include guard's final `#endif` (the LAST line of the file) was missing in the copy that was built, i.e. the end of the file was cut while editing it by hand. Reproduced exactly by deleting that line from the shipped file; the shipped file is intact (28 conditionals, 28 `#endif`).
* New `tools/check_config.py`: finds a missing `#endif`, a `/*` left open (a `/*` found inside a comment), a last line that is not the guard's `#endif`, and more than one active `CTRL_VIN_MIN_V`; prints the main settings. No false alarm on any shipped or historical config.
* New `configs/slave_config_90V_step1_autorun0.h` and `..._step2_autorun1.h`: the v0.6.9 config with the 90 V start window enabled (and autorun 0 / 1), both compiled against the real headers with the effective values checked. Copy over `Core/Inc/slave_config.h` instead of editing the file.
* Firmware sources unchanged since v0.6.9.

## v0.6.9 — low input voltage (90 V) in the config, tested
* A documented option in `slave_config.h` (`CTRL_VIN_MIN_V 80.0f`, commented out): the converter starts from 90 V and trips below 80 V. Nothing changes unless you enable it.
* New PC test (own runner job, `-DCTRL_VIN_MIN_V=80.0f`): starts and runs without a fault at 90 V / 70 V / 120 V amplitudes across the zero-cross edge timing spread; the start and trip levels follow the setting. The sign-detection margin versus input voltage is tabulated in docs/POWER_STAGE_BRINGUP.md (3.2x at 90 V).
* NOT verified: the hardware at 90 V (ZCD IC limit about 70 V on the bench, power-board rails, relay coils) and the bench-proven start rule (positive half first, low sides on), found at 197-245 V.

## v0.6.8 — bench-proven defaults for the legacy converter start
* Bench: with the low sides held on (`SLAVE_LOWSIDES_OFF_AFTER 0`) the converter starts 10 times out of 10 with `SLAVE_CONV_FIRST_POL 1` and 0 times out of 10 with `-1`; once running it survives fast variac changes. With the old rule (lows off at the 2nd diagonal, either half) a start took 1 to 5 attempts.
* Defaults now: `SLAVE_LOWSIDES_OFF_AFTER 0` and, with the legacy relays at stage 1, `SLAVE_CONV_FIRST_POL 1`. Everything else unchanged. The old behaviour stays available (`SLAVE_LOWSIDES_OFF_AFTER 2`, `SLAVE_CONV_FIRST_POL 0`).
* A test pins the shipped defaults (built without overrides); the low-side test covers "lows stay on until the stop". Bench results are tabulated at the top of docs/POWER_STAGE_BRINGUP.md, with what is NOT understood (why the negative half fails as the first diagonal) and what is NOT tested (stages 2 / 3).

## v0.6.7 — the old firmware's inverter low-side state at the converter start
* Bench: the old firmware's converter starts cleanly on the same hardware; this firmware tripped at the first diagonal on EITHER polarity (v0.6.6 video with the first half forced negative: `dt=0.06ms D-`).
* Found in the old code: the inverter outputs are enabled at zero duty from boot / re-arm, i.e. the two low-side IGBTs (1L, 2L) are ON and the transformer primary is shorted; the first converter diagonal runs with them on, they are cleared from the second diagonal. This firmware had kept them OFF since v0.2.0 (a mistake of mine).
* New `SLAVE_LEGACY_LOWSIDES` (default 1 with `SLAVE_RELAYS_MODE 2` at stage 1) and `SLAVE_LOWSIDES_OFF_AFTER` (2 = the old firmware, 0 = keep on): inverter outputs enabled at zero duty when the start sequence begins (before the SH pulse and before W opens the nets), off at the 2nd diagonal, off on every stop / fault, enabled again on the retry. High sides never driven.
* Tests: enabled before the relay sequence and before the first diagonal, first diagonal runs with them on, off at the second diagonal (+10 ms), off on a trip and re-enabled on the retry, never any duty; every break is caught (one equivalent variant is explained in the test notes).
* Docs: a section at the top of POWER_STAGE_BRINGUP.md and a second correction in RELAY_LOGIC_REVIEW.md.

## v0.6.6 — forced first polarity (diagnostic)
* Bench video of v0.6.5 (legacy relays, 245 V): the converter DOES start, then trips 24 us after its first POSITIVE diagonal, on every attempt (`dt=0.06ms D+ ON`); the fault releases both relays. The relay timers are exact multiples of the half-cycle, so the start polarity is always the same.
* New `SLAVE_CONV_FIRST_POL` (0 default / 1 / -1): the converter starts on the chosen half-cycle; other half-cycles are skipped (at most 10 ms). To find out whether the NEGATIVE half starts cleanly.
* New commented bench settings `CTRL_RESTART_STEP_MS` / `CTRL_RESTART_MAX_MS` / `CTRL_LATCH_AFTER` for quick retries instead of the 10 / 20 / 30 s lockouts and the latch.
* Tests: the first diagonal is the chosen polarity at 16 different start phases, both signs and with legacy relays; broken variants are caught. Master unchanged (v0.6.4).

## v0.6.5 — legacy relay sequence (SLAVE_RELAYS_MODE 2)
* Found: the old firmware's own comment says SH-RLY pre-charges the DC-link capacitors (R92 = the DC-link bleeder across V+ / PE) and W-RLY is the main relay that stays on. My bypass-relay reading (v0.6.0) conflicts with that and was never verified; the docs now say so.
* The F02 trips happened with `SLAVE_RELAYS_MODE 0` (relay coils off): the converter ran in a relay state the old firmware never used. The A/B test with the old relay state had never been done.
* New `SLAVE_RELAYS_MODE 2`: the old sequence exactly (SH closed aimed at a zero-cross, 1 s, SH off + W on together, W settle 1 s, converter only then; a fault releases both coils at once and the sequence repeats; W released 1 s after a stop). Converter-only (stage 1): stages 2/3 refuse to build in this mode. Starts only when everything else is ready; `WAIT: R` shows meanwhile.
* The shipped `slave_config.h` now has `SLAVE_RELAYS_MODE 2` (autorun still 0) and a documented optional `CTRL_VIN_MIN_V 180.0f` line to match the old firmware's 180 V range.
* Tests: sequence timing, no converter before W settles, aimed at a zero-cross, fault release and repeat, stop release, never both coils on; each break is caught. Master unchanged (v0.6.4).

## v0.6.4 — passive converter-mapping check, safe default
* Why: the FLT trips happen only while the converter runs, on live IGBTs. The old firmware (known to work) selects the converter diagonal from the ZCD EDGE DIRECTION; this firmware selects it from the measured AC_IN SIGN, and nothing ever confirmed that they agree.
* New, PASSIVE (the converter does not run): a counter of how often the two choices agree / disagree at each valid zero-cross event, shown on master row 3 as `M<agree>/<disagree>` (in the optional `diag2` frame field, bit 14; a running-trip word, bit 15, still takes precedence in FAULT).
* New compile switch `SLAVE_CONV_SWAP` (default 0): swaps which diagonal is energised for each half-cycle. Only to be used after the mapping check and a scope check. Test builds for both settings pass.
* **Safe default:** the shipped `slave_config.h` now has `SLAVE_BENCH_AUTORUN 0` (the converter never starts unless you set it to 1).
* Docs: "STOP: the converter on live hardware", the passive check, and a reduced-voltage scope check against the old firmware.
* Both boards need this version (the master shows the check).

## v0.6.3 — trip timing, break-counter fix
* **Fix:** the FLT interrupt (IRQ 6) runs before the break interrupt (IRQ 13 / 20) for the same pulse and clears "running", so v0.6.2 counted the break copy as IDLE (`F0/3` next to `K3/0` on the bench was the same three trips). An event that belongs to a running incident started by the other interrupt now counts as running.
* **New:** the slave records, for the latest FLT / break event that hit while the converter was RUNNING, the time since the last zero-cross event (16 us units) and which diagonal was energised. Sent in a second optional frame field `diag2` (frames without it still parse). The master alternates its fault row every 2 s between the context and `dt=<ms> D<+|-|0> <ON|off>`.
* Both boards need this version. Tests: protocol field, counter fix, capture at 6 offsets x both polarities (FLT and break paths), display alternation; every change is shown to be caught when undone.

## v0.6.2 — FLT / break event counters
* New OPTIONAL last field `diag` in the status frame (a frame without it still parses; both link_proto copies identical): counts of FLT-pin edges and break events, split into "converter NOT running" and "running" (saturating 31 / 7).
* Master rows 1 / 2 show them as `F<idle>/<run>` and `K<idle>/<run>` once something was seen. Events while idle are not faults. Purpose: tell noise / supply disturbance (events while idle) from the converter's own current (events only while running) - see docs/POWER_STAGE_BRINGUP.md.
* Clarified in the docs: F01 and F02 are the same net event; FLT:H does not prove a glitch; the 5-minute figure resets the trip counter and is not a lockout time.
* Slave and master BOTH need this version (the master shows the counters).

## v0.6.1 — start hysteresis, F03 split
* **F09 flaw fixed:** the converter started at Vin >= 140 V and tripped at Vin < 140 V (no gap), so any sag when the converter loaded the supply tripped it immediately. Start window is now [150, 290] V (`CTRL_VIN_START_HYST_V`, default 10 V); trip window unchanged [140, 300] V. Row 3 shows `WAIT: V` until Vin >= 150 V.
* **F03 split by cause:** F03 = event at the wrong TIME (spacing outside 7.5-12.5 ms); new **F15** = event on time but the sign did not flip / edge type did not alternate; new **F16** = last-moment polarity check disagreed before energising (previously also reported as F03). All escalating, as F03 was.
* Tests: spurious edge, wrong sign, last-moment disagreement, hysteresis (145 V waits, sag to 141 V does not trip, 139 V trips, 295 V waits); each fix is shown to be caught when undone.
* Master unchanged since v0.6.0.

## v0.6.0 — relay logic rework
* SH-RLY and W-RLY are **bypass relays across the series transformer's primary** (power-board schematic, read at 600 dpi). Sequence rebuilt from rules R1-R5 (`docs/RELAY_LOGIC_REVIEW.md`):
  start = relays shorted -> converter -> inverter at zero duty -> shunts opened -> duty ramps; stop = duty 0 -> shunts closed -> inverter disabled;
  soft faults unwind the same way, hard faults (break / FLT / relay) switch everything off at once and close the shunts.
* New relay module (no zero-cross aiming, no 1 s holds). Contact type per relay is a MEASURED setting (`SLAVE_RELAY_SH_CONTACT`, `SLAVE_RELAY_W_CONTACT`);
  stages 2/3 do not build until set, nor if neither relay is a shunt, nor with a settle time < 20 ms.
* Controller: new states UNSHUNT and RESHUNT, `duty_ok` interlock in the interrupt, soft / hard fault paths, wait reason `R`.
* Master display: state names CONV / INVS / UNSH / RSHT, wait letter R.
* Tests: physical relay model (coils, 6-12 ms contact travel) checking the rules on every call of every scenario; passes for all 8 meaningful contact-type combinations at stages 2 and 3; deliberate breaks of each rule are caught.
* `slave_config.h` carries the bench settings: stage 1, autorun on, silent report, relays driven by hand.
* NOT verified on hardware: the contact types, the polarity mapping of the converter diagonals, interrupt timing on the real chip, the MCU_FLT net behaviour on the bench. See `docs/POWER_STAGE_BRINGUP.md`.

## v0.5.0 — wait reasons
* Master row 3 shows what the slave waits for while it sits in START: `WAIT: Z V M F B O`.

## v0.4.0 — fault context, guard persistence
* Fault context byte (state, zero-cross mode, live FLT and BKIN levels) in bits 8..15 of the link `flags` field, shown on master row 3. No protocol change.
* The 20 kHz polarity guard (F11) needs two consecutive bad samples.

## v0.3.0 — zero-cross race fix
* Fixed a bug of mine: the main loop compared its clock with a timestamp written by the zero-cross interrupt; an interrupt in between wrapped the unsigned difference (false "zero-cross lost", the first bench F08). Signed ages everywhere.
* Fault 8 split: 8 = watchdog compare, 13 = main-loop backup check, 14 = converter running but never commutated.
* Preserved gap: the first master state / fault display (row 0) was delivered here but that exact package was not kept; the master stays as uploaded until v0.4.0.

## v0.2.0 — inverter pins held low until enabled
* The four inverter pins are plain LOW outputs until `SlavePower_InverterEnable()` and return to that state on every off path (a configured-but-disabled timer output idled HIGH on the low-side pins).

## bench-as-received
* Your two Core.zip uploads, unchanged. Slave at stage 1; master with the old monitor.

## v0.1.0-legacy
* The old patched firmware and the early bring-up skeleton, for reference (`legacy/`).

## Not preserved
* PC test snapshots before v0.6.0 (the tests were rewritten for the relay rework; the earlier versions were not kept).
