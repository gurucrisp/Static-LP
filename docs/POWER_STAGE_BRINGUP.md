# Power stage bring-up — slave (converter + inverter + bypass relays)
Read `RELAY_LOGIC_REVIEW.md` first: it explains what the relays are (bypass relays across the series transformer's primary) and why the sequence is what it is.

## THE INVERTER (v0.7.0): see docs/INVERTER_STAGE2_BENCH.md
Stage 2 with the legacy relay sequence, the 2 us dead time, the ramped one-shot injection test (and its transformer-flip verdict), and the three bench gates (scope dead time with no mains, stage 2 at about 120 V, stage 2 at normal input). Read its safety section first.

## Toroid above 260 V, 3 s retry, and the peak-start experiment (v0.6.15)
**Bench (toroid, W-RLY settle -> reliable up to):** 1 s -> about 230 V, 5 s -> about 250 V, 10 s -> about 260 V. An exponential fit of these three points gives a time constant of about 5 s and a CEILING near 265 V: more waiting cannot reach 300 V and only slows every start (the earlier 275 V prediction from a crude two-point estimate was too optimistic). The EI core is fine in the whole range.
**Retry:** `CTRL_RESTART_STEP_MS` is the lockout after a trip (10 s, then 20 s, 30 s ... capped at 60 s; the slave latches after `CTRL_LATCH_AFTER` = 6 trips). The ready-made `*_retry3s` files set the lockout to 3 s and the latch to 10 trips (power-cycle to clear). A full retry cycle is the lockout + SH 1 s + the W settle (3 + 1 + 5 = about 9 s with a 5 s settle). Every trip is a REAL current spike: do not leave it retrying unattended.
**New experiment: `SLAVE_CONV_FIRST_AT_PEAK 1`** (stage 1, default 0). The converter's first diagonal is normally turned on 40 us after the zero-cross event, when the line is at about 0 V and the DC link sits near the mains peak: nearly the whole peak voltage is put across the series winding's leakage inductance and the current ramps for about 25 us until the comparator trips (`dt=0.06ms` on the master). The step grows with the voltage and a toroid has little leakage: it trips first, and only at the highest voltages. With this option the FIRST diagonal is turned on at the line PEAK (a quarter cycle, about 5 ms, after the event; measured in simulation: 5000 us), when line and DC link are nearly equal; every later diagonal is exactly as before. This is a HYPOTHESIS, not shown on the bench.
**How to test (toroid, series lamp, output load disconnected; 10 starts each, note the highest voltage with 10 immediate starts in a row):**
1. `slave_config_normal_settle5s_retry3s.h`: the baseline (should be reliable up to about 250 V).
2. `slave_config_PEAK_settle1s_retry3s.h`: the peak start alone, with the standard 1 s settle. If this is reliable up to 300 V the long settle is not needed at all.
3. `slave_config_PEAK_settle5s_retry3s.h`: peak start plus 5 s settle.
If nothing improves, the cause is not the first-diagonal step: report the fault code and the `dt=` line (a trip that moves from `dt=0.06ms` to about 5 ms would show the first diagonal now trips at the peak instead). The earlier video also showed the board RESTARTING by itself above 245 V (fault F16 with Vin / Vout reading about 1 V, then NO LINK, then a boot): that is a supply problem, not a start trip, and no timing option addresses it.

## Toroid, above 230 V: starts are flaky (v0.6.13 experiment)
Bench: EI and toroidal series transformers both run the converter from 90 V to 230 V. With the TOROID above 230 V a start sometimes works at once, sometimes after retries, sometimes never; once running there is no problem.
**Hypothesis (NOT shown):** the converter's first diagonal connects the DC link (charged to about the mains peak by the SH pulse) to a line that is near zero. The current spike grows with the voltage and with how little inductance the winding has; a toroid (little leakage) trips first and only at the highest voltages.
**Experiment, no code change:** `configs/slave_config_normal_autorun1_settle5s.h` = the normal config with `SLAVE_LEGACY_W_SETTLE_MS 5000U` (W-RLY on for 5 s, standard 1 s, before the converter may start). With the PRI nets open the DC link bleeds through its bleeder while W is on, so it is lower at the first turn-on. The master shows `WAIT: R` for about 6 s instead of 2 s.
Compare 10 starts above 230 V on the toroid with the standard config and with this one. If the starts become reliable, shorten the settle step by step (3 s, 2 s) to the smallest value that works; if nothing changes, the hypothesis is wrong: report the fault code (F01/F02/F16 ...) and the `dt=` line of the failed starts. A PC test checks that the sequence is unchanged except for the delay (SH, W 1 s later, first diagonal 5 s after W on, low sides still on, runs without a fault).

**Result of the 5 s experiment (toroid):** with `SLAVE_LEGACY_W_SETTLE_MS 5000U` the starts are reliable up to about 250 V (standard 1 s: about 230 V); above 250 V the same problem continues. The video also showed the board restarting by itself above 245 V (fault `F16` with Vin / Vout reading about 1 V, then `NO LINK`, then a fresh boot) and one `F01` at the first positive diagonal (`dt=0.06ms D+ ON`, `F4/0 K3/1`: FLT and break events also counted while the converter was NOT running).
**Next, falsifiable:** if the limit simply follows a slow bleed-down of the DC link while W is on (a crude two-point estimate gives a time constant of the order of 50 s: NOT measured), a 10 s settle should move the limit from about 250 V to about 275 V; if it does not move, the cause is not the settle time. `configs/slave_config_normal_autorun1_settle10s.h`. Also needed: does the OLD firmware start reliably above 250 V on the toroid? (If it does not either, this is a limit of the transformer / supply, not of this firmware.)

## Transformer-connection ("flip") check on every converter start (v0.6.11, bug fixed in v0.6.12)
**BUG in v0.6.11 (fixed in v0.6.12): the result was never sent to the master.** The link's diag2 field carries one thing at a time, and the passive mapping-check counters are never cleared, so they stayed non-zero for good and always won over the transformer result. v0.6.12 sends: the trip capture while in FAULT, the transformer result while the converter runs (nothing stale), otherwise the passive mapping check (`SlaveCtrl_Diag2()`). A test now follows what the link sends through every phase and fails against the v0.6.11 logic.
Still required on the bench: the MASTER must run v0.6.11 or later (an older master simply ignores the result), and the slave config must define `SLAVE_TF_CHECK` (a config from before v0.6.11 turns the check OFF silently: `python tools/check_config.py <slave_config.h>` prints a NOTE for that).
**CORRECTION first.** I had described the old firmware's transformer test as "a small injection, then compare". Reading its code shows otherwise: it is triggered by a flag (`trasnfomersetflag`), closes W-RLY, waits 2 s, lets the CONVERTER commutate
normally (no inverter duty at all, the inverter low sides stay on), waits 1 s more, measures RMS Vin and Vout once and decides `Flip_mode = (Vout - Vin > 0) ? normal : flipped` (threshold `offset` = 0). It only REPORTS (`flipflag`, shown on its LCD); nothing is corrected.
**What v0.6.11 does** (`SLAVE_TF_CHECK`, on by default with the legacy relays at stage 1): once the converter has really been running for 1.5 s it averages (Vout - Vin) over 2 s and the master shows it on row 3, every other 2 s, in the converter-running state:
`TF OK +3.2V` (Vout above Vin by more than 0.5 V) / `TF FLIP -3.0V` (below by more than 0.5 V) / `TF ? +0.2V` (inside the dead band: the two sensors differ by a few tenths of a volt at rest, so the old strict `> 0` would be a coin toss there). One result per converter start; a trip keeps the last one, the next start replaces it; power-up clears it. Nothing extra is driven.
**HONEST LIMIT - validate it once.** I could NOT derive from the old code WHY Vout - Vin should depend on the transformer polarity when only the converter runs. The rule is copied, its physics is not understood. Protocol (series lamp, output load disconnected):
1. Transformer as installed (it works with the old firmware, so expect OK): note the `TF ... +x.xV` value. Do it at two input voltages (about 150 V and 230 V): the size should scale.
2. Power OFF, wait for the DC link to discharge, swap the two leads of ONE winding (not both), power on, run again: the sign MUST flip (`TF FLIP -x.xV`, about the same size).
3. If the sign does not follow the wiring, this check does not discriminate: do NOT trust it, and the injection test of stage 2 (a small known boost, Vout must rise) is the check to build.
Wire format: `diag2` bit 13 set with bits 15 / 14 clear; bits 12..11 verdict (1 OK, 2 flipped, 3 inconclusive); bits 10..0 = Vout - Vin in 0.1 V, 11-bit two's complement. Settings: `CTRL_TF_SETTLE_MS` 1500, `CTRL_TF_WINDOW_MS` 2000, `CTRL_TF_DEADBAND_V` 0.5. The master needs the matching `master_monitor.c`.

## Low input voltage: starting the converter from 90 V (v0.6.9)
Firmware: remove the comment marks on `#define CTRL_VIN_MIN_V 80.0f` in `slave_config.h` (the 180 V line stays commented): the converter then STARTS from 90 V (80 V + the 10 V start hysteresis) and TRIPS below 80 V.
The polarity sign detection (an 8-sample window 0.2-0.6 ms before the zero-cross edge, threshold 20 counts = 5 V) has this margin (counts per volt from the 140 V = 784 counts point):
| input | mean in the sign window | margin over the threshold |
|---|---|---|
| 230 V | 41 V | 8.1x |
| 140 V | 25 V | 4.9x |
| 90 V | 16 V | 3.2x |
| 70 V | 12 V | 2.5x |
| 45 V | 8 V | 1.6x |
PC test (`-DCTRL_VIN_MIN_V=80.0f`): starts and runs without any fault at 90 V, 70 V and 120 V amplitudes with the zero-cross edge from -120 to +150 us off the true zero and +/-40 us jitter; waits at 85 V, starts at 92 V, trips F09 at 78 V.
NOT known (hardware, must be tried in this order, always with the series lamp, output load disconnected):
1. `SLAVE_BENCH_AUTORUN 0` first: at 90 V the master must show Vin / frequency steadily, no `NO LINK`, and `M<agree>/0` ticking (the ZCD IC only worked above about 70 V on the bench; the power board's own 12 V and 3.3 V rails come from the mains).
2. Only then autorun 1: the relay clicks (12 V coils) and the four gate-driver supplies at 90 V mains.
3. The start rule found at 197-245 V (positive half first, low sides kept on) has NOT been verified at 90 V.
Regulating from 90 V (stages 2 / 3, the inverter) is a separate, untested question: the current demand rises as the voltage falls and the transformer ratio limits the correction.

## BENCH RESULTS SO FAR (converter-only, live IGBTs, legacy relay sequence, 197-245 V) and the settings that work (v0.6.8)
| firmware / setting | result |
|---|---|
| relay coils off (mode 0), inverter outputs off | converter mostly starts; intermittent F02 on fast variac changes |
| legacy relays (SH 1 s, W on = PRI nets open), inverter outputs OFF (v0.6.5, v0.6.6) | EVERY start trips 24 us after its first diagonal, on D+ and on D- |
| + low sides on, off at the 2nd diagonal (v0.6.7 default), either half | starts after 1 to 5 attempts (escalating 10/20/30/40/50 s lockouts); sometimes F02 straight away, sometimes F16 |
| + `SLAVE_LOWSIDES_OFF_AFTER 0`, `SLAVE_CONV_FIRST_POL -1` | **0 starts** |
| + `SLAVE_LOWSIDES_OFF_AFTER 0`, `SLAVE_CONV_FIRST_POL 1` | **10 starts, 0 failures**; once running it survives fast variac changes |
So with the legacy relays at stage 1 the shipped defaults are now: relays mode 2, low sides kept on while only the converter runs (`SLAVE_LOWSIDES_OFF_AFTER 0`), first half positive (`SLAVE_CONV_FIRST_POL 1`).
**NOT understood:** why the negative half fails as the FIRST diagonal although it runs fine as every second diagonal; why the old firmware starts on whichever half comes. NOT tested: stages 2 and 3 (the inverter),
the output load, the master commands. Do not extend these results to the inverter.

## v0.6.7: why the converter tripped at its first diagonal - the transformer primary (READ THIS FIRST)
Bench (v0.6.5 / v0.6.6, legacy relays, 197-245 V): every converter start trips about 24 us after its FIRST diagonal, on BOTH polarities (`dt=0.06ms D+` and, with the first half forced negative, `D-`), after cold power-ups too.
The OLD firmware on the same bench keeps its converter LEDs glowing (it starts cleanly). Differences found by reading its code:
1. **Inverter low sides.** The old firmware starts the inverter timer outputs at boot (and after every fault re-arm) with ZERO duty, which turns the two LOW-side IGBTs (1L, 2L) ON: the transformer primary is
   SHORTED by the IGBTs. Its FIRST converter diagonal runs with the lows still on; only from the SECOND diagonal does it clear the complementary outputs (both half-cycle branches: `if (!delay_active && !pwm_allowed) {first} else if (pwm_allowed) {...} else {CCER &= ~CC1NE}`).
   This firmware kept 1L / 2L OFF from v0.2.0 on: an "unexpected" LED state that I wrongly treated as a defect - it is the old firmware's normal state.
2. **Relay state.** The old sequence ends with SH off and W on = the PRI-PHASE / PRI-NTRL nets OPEN (measured: continuity with both coils off or SH on, none with W on and SH off). With the nets open AND the lows off, the primary is open and uncontrolled at the converter's first turn-on.
   (With the relay coils OFF the nets are shorted by W-RLY, which is why the converter mostly started in the earlier bench runs.)
**v0.6.7 reproduces the old behaviour** (`SLAVE_LEGACY_LOWSIDES`, default ON with legacy relays at stage 1): the inverter outputs are enabled at zero duty (1L and 2L ON, high sides never driven) when the start sequence begins, i.e. before the SH pulse and before W-RLY
opens the nets; the first converter diagonal runs with them on; they go OFF at the second diagonal (`SLAVE_LOWSIDES_OFF_AFTER 2`, 0 = keep them on until a stop / fault).
**1L and 2L glowing during the start is NORMAL now.** Expected: 1L + 2L on -> SH click (1 s) -> W click -> 1 s -> converter LEDs start glowing, 1L / 2L go off about 10 ms later, converter LEDs keep glowing, no fault.
If it still trips, try `SLAVE_LOWSIDES_OFF_AFTER 0` and report what the `dt=` page shows. Series lamp in the line, output load disconnected. (A trip releases everything, including the low sides, as before.)

## What the v0.6.5 bench video showed, and the forced-polarity test (v0.6.6)
Legacy relays, 245 V, converter-only: every attempt = `START` + `WAIT: R` (the 2 s relay sequence) -> the converter starts -> FAULT F01 (the same net event as F02) with `F0/1 K0/1`, `@CONV M:B`
and **`dt=0.06ms D+ ON`**: the FLT pulse arrives about 24 us after the FIRST POSITIVE diagonal (3H+4L) switches on (dead time 40 us after the zero-cross event). The fault releases both relays
(that is why W went off after about 1 s). Lockouts 10 s, 20 s as designed. Identical on every attempt.
Why always `D+`: the relay hold (1000 ms) and settle (1000 ms) are exact multiples of the 10 ms half-cycle, so the converter always starts on the same half-cycle. With the relay coils off, the start phase was random:
trips "sometimes" fits a trip that happens only when the first diagonal is the positive one (a hypothesis, not proven).
**Test:** `SLAVE_CONV_FIRST_POL` in `slave_config.h` (0 = either, 1 = positive first, -1 = negative first). Build with legacy relays, autorun 1, series lamp, output load disconnected.
* `-1` starts cleanly (the converter stays on, W stays on, `RUN`-like state with no fault) -> the first POSITIVE diagonal is the problem: the first-pulse behaviour of the leg-3 high side.
* `-1` also trips (with `D-` in the dt page) -> the first diagonal is the problem whichever polarity: the turn-on itself (DC link voltage against the line at the instant of turn-on).
* `+1` is the control: it should reproduce the video.
For quick retries instead of 10 / 20 / 30 s lockouts and the latch after 6 trips, remove the comment marks on the three `CTRL_*` lines in `slave_config.h` (bench only; every trip is a real spike).

## A/B test: the converter with the OLD relay sequence (SLAVE_RELAYS_MODE 2)
The old firmware, proven on this hardware, closes SH-RLY (aimed at a mains zero-cross) for 1 s, then SH off and W on in the same instant, W stays on, and only
1 s later does the converter start. Until v0.6.4 this firmware ran the converter with the relay coils OFF (mode 0): a state the old firmware never used.
Build: `SLAVE_POWER_STAGE 1`, `SLAVE_RELAYS_MODE 2`, `SLAVE_BENCH_AUTORUN 1`, and (to stay in the old firmware's range) remove the comment marks on
`#define CTRL_VIN_MIN_V 180.0f` in `slave_config.h` (starts at 190 V). Series lamp in the line. Expected: row 3 shows `WAIT: R` for about 2.1 s: SH click, then W click
1 s later, then the converter 1 s after that. A fault releases both relays at once and the whole sequence repeats on the retry.
Read the result: trips gone -> the converter needs W-RLY on (the relay state was the cause). Trips remain -> compare with the `F`/`K` counters and the `dt=` page.

## STOP: the converter on live hardware (read this first)
With live gate drivers, real IGBTs and a charged DC link, FLT trips that occur ONLY while the converter runs (bench: `F0/3`) are real until proven otherwise.
The one converter drive that is known to work on this hardware is the OLD firmware. Compared with it, this firmware differs in:
1. **How the diagonal is chosen.** Old: from the ZCD EDGE DIRECTION (falling -> PB3+PB5 = 3H+4L, rising -> PC4+PC5 = 3L+4H), on 40 us after the edge, off at the next edge. New: from the measured AC_IN SIGN (positive next half -> 3H+4L), also on 40 us after the edge, off 0.3 ms before the next. Nothing on this hardware ever confirmed that the two choose the same diagonal.
2. Start voltage (old: LV 180 V + 5 V hysteresis; new: 150 V), the relay / pre-charge sequence before the converter, and the old inrush handling (`Inrush_present`).
A wrong diagonal connects the DC link across the line through forward-conducting IGBTs: a short circuit every half-cycle.

### Step 1 - PASSIVE mapping check (the converter never runs)
Build with `SLAVE_POWER_STAGE 1` and `SLAVE_BENCH_AUTORUN 0` (the shipped default). Apply the mains through the isolation transformer and a series lamp. Read **row 3** of the master: `M<agree>/<disagree>`.
* `M99/0` (all agree): this firmware energises the same diagonal as the old firmware in every half-cycle.
* `M0/99` (all disagree): it would energise the OPPOSITE diagonal. Do NOT run the converter. After a scope check (step 2) set `SLAVE_CONV_SWAP 1`.
* mixed: the ZCD edge direction does not identify the half-cycle (e.g. one pulse per half-cycle): the old rule cannot be read as I did. Send the numbers and a scope capture of PB0 with AC_IN.

### Step 2 - scope check at reduced voltage, current-limited
Variac to about 80-100 V (the ZCD works above about 70 V) with a series lamp (100-200 W) in the line. Isolated scope. Compile with `-DCTRL_VIN_MIN_V=70.0f` so it starts at 80 V.
Capture the line (isolated probe) with PB3, PB5, PC4, PC5 (the four gate commands). Expect exactly one diagonal per half-cycle, never overlapping, ending about 0.3 ms before the zero. Then flash the OLD firmware at the same voltage and capture the same signals: **the diagonal in each half-cycle must be identical**. Only then go up in voltage, and keep the lamp in until the DC link current looks right.

## Sequence (rules R1-R5)
```
START   relays SHORTED (bypass) -> converter commutating -> inverter ENABLED at zero duty (low sides on = primary shorted by the IGBTs)
        -> shunts OPENED (primary now controlled by the inverter only) -> duty ramps
STOP    duty -> 0 -> shunts CLOSED (wait settle) -> inverter disabled -> converter off
FAULT   soft (anything but overcurrent): converter off + duty 0 at once, shunts closed, THEN inverter disabled
        hard (break / FLT / relay): everything off at once, shunts commanded closed immediately
```
Master row 0 shows `START` for SYNC, CONV, INVS, UNSH; `RUN` while regulating; `FAULT Fnn` after a fault. Row 3 in FAULT: `@<state> M:<mode> FLT:<H/L> BK:<H/L>`
with state = SYNC / CONV / INVS / UNSH (shunts opening) / RUN / RSHT (shunts closing). In SYNC row 3 lists what it waits for: `WAIT: Z V M F B O R`
(zero-cross pattern, input START window 150..290 V - it only TRIPS outside 140..300 V, the 10 V gap is a hysteresis, measurement, FLT low, break input low incl. a floating one, output voltage, relays not yet settled SHORTED).

## Decisions
1. **Regulator = integral only** (PID's P and D removed; proven bit-identical to the old code with Kp = Kd = 0 on 2.8 million comparisons; design study in `tests/study_pid.c`).
2. **Relays** (`slave_relay.h`): driven from the rules, NOT from the old firmware: no zero-cross aiming, no 1 s holds. What each contact does is a MEASURED setting:
   `SLAVE_RELAY_SH_CONTACT` / `SLAVE_RELAY_W_CONTACT` = `RELAY_SHORTS_WHEN_ENERGISED` | `RELAY_SHORTS_WHEN_RELEASED` | `RELAY_NOT_A_SHUNT`.
   **Stages 2 and 3 do not build until both are set**, nor if neither is a shunt, nor with a settle time below 20 ms. Stage 1 needs no setting (until set, both coils stay off).
   `SLAVE_RELAYS_MODE 0` = relays driven by hand (stages 2/3 then also need `SLAVE_RELAYS_CONFIRMED 1`).
3. **The inverter pins (PB14, PB13, PA8, PA7) are plain LOW outputs until the inverter is enabled** (a configured-but-disabled timer output idled HIGH on the low-side pins on the bench) and return to that state on every off path.
4. The FLT input (PD2) is the SAME net as the two break inputs (PB12, PA9): `MCU_FLT`, from the power board's comparators via the interface connector.

## Fault codes
1 break (BKIN) | 2 FLT | 3 zero-cross event at the WRONG TIME (spacing outside 7.5-12.5 ms) | 4 mains too small to read polarity | 5 AC_IN/AC_OUT sign mismatch | 6 output too high | 7 inverter would not enable |
**8 zero-cross event overdue (watchdog compare)** | 9 input out of range | 10 measurement stale | 11 line reversed vs the energised diagonal | **12 relay not in the state the controller requires** |
**13 zero-cross timestamp too old (main-loop backup)** | **14 converter state reached, events arrive, nothing ever commutated** | **15 zero-cross event on time but the measured sign did NOT flip (or the edge type did not alternate)** | **16 last-moment check: the newest samples show the OPPOSITE polarity to the one about to be energised**.
Escalating (retry 10/20/30/40/50 s, latch on the 6th): 1-7, 11, 12, 14, 15, 16. Conditions (retry 2 s): 8, 9, 10, 13. Hard (all off at once): 1, 2, 12.
F11 needs two consecutive bad 50 us samples (one noisy sample cannot trip it).

## F03 / F09 / F15 / F16 right after the converter starts at about 140 V
All four are consequences of a supply that does not hold up when the converter starts drawing current, or of a distorted line:
* **F09** was partly my flaw (start and trip at the same 140 V, no gap). v0.6.1: starts at 150 V, trips below 140 V. If it still trips, the input REALLY fell by more than 10 V when the converter started: read Vin on row 1 just before the start and just before the trip.
* **F03** an edge at the wrong time (extra edge, ring, a distorted zero crossing). **F15** an edge on time but the line sign did not flip. **F16** the last-moment polarity check disagreed.
  Row 3 of the master shows the state and zero-cross mode (`M:B` square wave, `M:F` pulse train) with each fault.
* Measure the input current at the converter start and look at AC_IN and PB0 together on the scope at the moment of the start.

## FLT / break event counters - is the F02 noise, or the converter?
Rows 1 and 2 of the master show, once something has been seen: `F<idle>/<running>` on row 1 and `K<idle>/<running>` on row 2 (counts since power-up).
**F** = falling edges on the FLT pin (PD2), **K** = break-input events (PB12 / PA9). **idle** = the converter was NOT running when it happened
(it cannot be the converter's own current: noise or supply disturbance), **running** = it was running. Events while idle are NOT faults. CLIP overwrites the tail if both appear.
Test: build with `SLAVE_BENCH_AUTORUN 0` (the controller stays OFF, the converter never starts), then change the input voltage fast and slow.
* `F` idle count rises with FAST changes -> the net is disturbed by the line / supply movement itself (variac brush noise, SMPS transient): not the converter. A pull-up on MCU_FLT and a stiffer source are the remedy; the firmware keeps escalating real trips.
* `F` idle stays 0, only `F`/running rises (autorun on) -> the converter's own current or its switching is involved.
Note `F01` and `F02` are the SAME net event: PD2, PB12 and PA9 are one net, and whichever interrupt is served first names the fault.
`FLT:H` on row 3 does not prove a glitch: the interrupt switches the converter off at once, which can itself end a real trip.

## Where in the half-cycle did the trip hit? (v0.6.3)
While the slave is in FAULT, row 3 alternates every 2 s between the context (`@CONV M:B FLT:H BK:H`) and the trip timing: `dt=<ms> D<+|-|0> <ON|off>`.
`dt` = time since the last valid zero-cross event when the FLT / break event hit WHILE THE CONVERTER WAS RUNNING; `D+` / `D-` = the positive / negative half diagonal was energised at that moment (`D0 off` = none).
The converter turns on about 0.04 ms after the zero-cross event and off about 0.3 ms before the next one (about 9.7 ms):
* `dt` about 0.04-0.2 ms every time -> the trip happens at TURN-ON (an inrush / shoot-through spike at the moment the diagonal switches on).
* `dt` about 9.5-9.8 ms -> at TURN-OFF.
* always the SAME polarity (only `D+` or only `D-`) -> an asymmetry between the two half-cycles: a diagonal mapping or hardware difference.
* scattered -> noise, or a current that depends on the line.
Note: **FLT events that occur ONLY while the converter runs, with the power board connected, must be treated as real until proven otherwise** (the comparators watch current). Do not keep cycling a live converter: verify the gate signals and the current on a scope, current-limited, first.
The counters `F` / `K` on rows 1 / 2: a pulse seen by both the FLT pin and the break input counts once in each as RUNNING (v0.6.2 wrongly counted the break copy as idle: `K3/0` next to `F0/3` was the same three events).

## Lockouts
Escalating faults restart after 10, 20, 30, 40, 50 s; the 6th trip LATCHES (needs a power cycle). The counter is cleared only after 5 minutes of fault-free running
(that is not a lockout time). For a bench without a power stage you can override with `-DCTRL_RESTART_STEP_MS=2000 -DCTRL_LATCH_AFTER=255`; do NOT do that with a live power stage.

## Measure BEFORE any inverter test (no mains)
The four continuity measurements of SH-RLY and W-RLY in `RELAY_LOGIC_REVIEW.md` section 6. Set the two contact settings from them.

## Switches (slave_config.h, or -D on the compiler command line)
`SLAVE_POWER_STAGE` 0..3 | `SLAVE_BENCH_AUTORUN` 0/1 (needed until the master sends run commands) | `SLAVE_RELAYS_MODE` 1 firmware / 0 by hand | `SLAVE_RELAYS_CONFIRMED` |
`SLAVE_RELAY_SH_CONTACT`, `SLAVE_RELAY_W_CONTACT`, `SLAVE_RELAY_SETTLE_MS` (default 100) | `BRINGUP_REPORT_MODE` 2 (silent) required by stages >= 1 |
optional: `CTRL_VIN_MIN_V` (140 V; about 80 for a reduced-voltage first test), `CTRL_TARGET_V`, `CTRL_DUTY_MAX_PCT` (90).
Stage 0 = measure and report only. 1 = converter only. 2 = converter + inverter at a fixed 96-count duty. 3 = closed loop.

## Procedure — one stage at a time, STOP at the first thing that does not match
**Always**: isolation transformer + variac + series current limit (lamp); DC link current-limited; isolated scope.
**Before stage 1 (logic supply only)**: PD2 (FLT) and PB12 / PA9 (BKIN) read HIGH (they are one net); converter, inverter and relay pins read 0 V.

**Stage 1 - converter only.** Row 0 `START F00`. Relays stay in the SHORTED pattern (nothing is opened). Scope the gates of 3H, 3L, 4H, 4L with the line: 3H+4L only during
the positive half, 3L+4H only during the negative half, never overlapping, each ending about 0.3 ms BEFORE the zero and the next starting about 0.05 ms AFTER it.
DC link follows the rectified line; line current stays small. If it sits on `WAIT:` row 3 tells you why. Wrong half or current-limit trip: STOP.

**Stage 2 - inverter, open loop** (contacts set, `SLAVE_POWER_STAGE=2`). Expect, in this order: converter runs (about 3 s) -> **1L and 2L go ON** (inverter enabled at zero duty)
-> about 0.1 s later the **relays move** (shunts open) -> about 0.1 s later the H sides start chopping. Only ONE leg modulates at a time; for Vin < 230 V the output must go UP.
First run with the relays' contact action verified by the continuity table. Measure g = (change in Vout) / 96 and send it to me.
Stop (`SetRunRequest(0)` / power-off of the request): duty -> 0, relays move, THEN 1L/2L go off.

**Stage 3 - closed loop**: no load first. Vout ramps from Vin to 230 V (3 V/s for 2 s, then 10 V/s, 5 V/s inside 3 V). Abort on any fault code.

## What was verified on the PC (and how)
| Item | Result |
|---|---|
| Sequencer + relay rules, physical relay model (coils, contacts with 6-12 ms travel) checking R1-R4 on every call | full suite passes for ALL 8 meaningful contact-type combinations (SH, W in {NO, NC, not-a-shunt}; neither-is-a-shunt is a build error) at stage 2 and stage 3; stage 1 passes |
| Relay-specific tests | module unit test, start order, normal stop, stop while opening, soft-fault unwinding, hard fault, fault while unwinding, 90-point injection sweep |
| Tests can fail | deliberate breaks of every rule caught (R1, R2, R3 x3, R4, inverted coil polarity, settle shorter than contact travel, module never closing); a stray-duty bug is shown to be BLOCKED by the interrupt gate |
| Regulator equivalence | 2.8 million comparisons, bit-identical |
| Polarity helper + link frame | 6,763 checks, incl. all 256 fault-context bytes in `flags` bits 8..15 |
| Master display text | 43 checks (states, fault context, wait reasons) |
| Register bit-math of the inverter-pin hand-over | 3,800 checks against the real G0B1 register layout |
| Builds | all sources clean against the real G0B1 / G070 HAL headers at stages 0-3 and both relay modes |

## What is NOT verified (needs the hardware)
* **The contact types (above) — and with them the whole bypass premise.** No sensing of the real contact state exists.
* The converter diagonal <-> line polarity mapping (3H+4L positive, 3L+4H negative) and the leg table (from the old firmware).
* My reading of the schematic nets: converter legs 3/4 on PHASE-OUT / NTRL, inverter legs 1/2 on PRI-PHASE / PRI-NTRL.
* Interrupt timing on the real chip (TIM2 compares, 20 kHz guard, EXTI, breaks). PB0 edge -> gate-on should be about 60 us after the zero.
* Your actual ZCD pattern (square wave and pulse-per-half-cycle supported; one pulse per cycle is refused).
* The `MCU_FLT` net behaviour on your bench (floating without the power board; F02 at 140 V when the converter starts).
* Not implemented: the output triac switch (CONT1) / load soft-start, master run/stop and set-point commands, calibration storage, master protection logic.

## History
* First bench F08 was a bug of mine: the main loop compared its clock with a zero-cross timestamp written by the interrupt; an event firing in between wrapped the unsigned difference to ~4e9. Fixed with signed ages (also in the relay hold timer and the guard blanking).

## Tests (tests/ folder)
`gcc -O0 -ffp-contract=off -I.. test_reg_equiv.c ../slave_reg.c -o t -lm`
`gcc -O1 -Istubs -I.. -DSLAVE_POWER_STAGE=N -DBRINGUP_REPORT_MODE=2 -DSLAVE_RELAY_SH_CONTACT=A -DSLAVE_RELAY_W_CONTACT=B test_ctrl.c ../slave_ctrl.c ../slave_reg.c ../slave_relay.c -o t -lm`
(N = 1,2,3; A, B = 1 shorts-when-energised, 2 shorts-when-released, 3 not-a-shunt; `-DSLAVE_RELAYS_MODE=0` for manual relays; `SKIP_SLOW=1` quick run, `ONLY=text` to pick tests)
`gcc -O1 -I.. test_meas_sign_link.c ../link_proto.c -o t -lm` | `gcc -O1 -I.. test_mon_state.c ../link_proto.c -o t` | `gcc -O1 -w -I.. study_pid.c -o t -lm` (the PID study)
