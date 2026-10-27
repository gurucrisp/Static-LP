# READ THIS FIRST: the autotune has damaged a power stage (v0.9.42)
**v0.9.32 to v0.9.41 wrote the profile to flash at the END of the sweep while the converter and inverter were still running at the last step's duty.** A flash page erase holds the CPU for tens of milliseconds; the commutation interrupts that follow the mains polarity cannot run meanwhile, and a diagonal can stay on across a polarity change. On the bench of the unit that lost 1L and 4H, a high current was seen at the end of every sweep ("exiting the retune") and the stage failed a few seconds into the last step, when the profile was written. The PC test shows the write happened with the controller in RUN in every one of those versions (3 flash operations, 3 of them while energised). v0.9.42 stops the stage FIRST and writes only when it has been OFF for 300 ms.
That is the most likely cause found; it is NOT proven to be the only one. Before any sweep on any unit:
1. Use a CURRENT-LIMITED source (a series lamp or a variac with a fast fuse) and a scope with a current probe on the series transformer's primary the first time.
2. The over-current comparators (U10, U14) trip at a FIXED voltage across the burden: 0.98 V is about 108 A peak with the 22.67 ohm burden. The IKW50N120CS7 is rated 150 A pulsed, so that leaves almost no margin and does not protect against 60 to 100 A peaks lasting seconds. Set the trip to 1.5 to 2 times your largest normal peak primary current and not above about 75 A (reference = I_trip x 22.67 ohm / 2500: 0.54 V for 60 A).
3. Check that the gate drivers have desaturation / short-circuit shutdown: a shoot-through inside a leg does not pass through the primary's current sensor at all, and the IGBT withstands a short circuit for only 8 microseconds.
4. Start with `STAGE3_first_run_cap640` or `STAGE3_wider_cap1600`, not `cap2752_full_range`, and keep Max Correction low (30 V) for the first sweep.
5. Never run a sweep with the output loaded.

# The first transformer autotune on the bench

**Not for a first try on a finished unit.** Variac, no load, a scope, a low Max Correction first. The sweep puts the input voltage PLUS the injection on the output terminals, with the output contactor open.

## What it does
Set-Mode, item 19 Re-tune, Yes (behind the password). The master sends `TUNE`. The slave starts itself (about 6 s: sync, converter, inverter, shunts open), measures the baseline for 3 s, then applies 10 steps of open-loop boost duty (16 to 2752 effective ticks), each held 3 s and measured over its last 1.5 s. At the end it stores the profile (cap, regulator gain scale) and the master shows `Tuning Complete`. About 40 s in all.
The sweep injects at most **80 % of the SMALLER of Max Correction and the headroom** (the output over-voltage limit of 280 V, less 10 V, minus the input). Each step is shortened so that its predicted injection stays under that limit.

## Settings
* Slave: `slave_config_..._fast_correction_100_iout_tune.h` (`SLAVE_AUTOTUNE 1`, `SLAVE_TUNE_MODE 1`). Add `slave_tune_link.c`, `slave_tune.c`, `slave_profile.c`, `slave_store.c`, `slave_flash.c` to the project.
* Master: v0.9.33 or later. Set-Mode item 4 **Max Correction** set (not 0). The unit stopped (STOP).
* The input steady within 1.5 V peak to peak during the sweep (a variac, not the mains).

## The input voltage decides how much the sweep can learn
| Input | Headroom (270 - Vin) | Sweep limit (80 % of the smaller of Max Correction and headroom) | What is learnt |
|---|---|---|---|
| 230 V | 40 V | 32 V | the low end only (about 1070 ticks) |
| 200 V | 70 V | 56 V at Max Correction 70 or more | about 1800 ticks |
| 160 V | 110 V | 88 V at Max Correction 110 or more | the whole range to the 2752 cap (86 V) |
Below 20 V of headroom (input above 250 V) a `TUNE` is refused: `Input Too High / Lower The Mains`.

## The first run (recommended)
1. Input 200 V from the variac, no load, output contactor open. Scope on one gate drive, a meter on the output.
2. Max Correction 60 V. Re-tune, Yes.
3. Watch: `Step n of 10` and the injected volts on the master; the output voltage on the meter (it should be the input plus the injection, at most about 250 V); the scope for anything odd; the Converter Current page (LOG, MENU x7) for the magnetizing current, which should rise with each step.
4. Write down the injection the master shows at each step and the meter's reading. They should agree within a volt or two.
5. STOP at any time ends it at once: the duty goes to zero, the shunts close, nothing is stored.
6. Then repeat at 160 V with Max Correction 110 or more for the whole range.

## What the results mean
* `Tuning Complete`: the profile is stored. The unit still runs on its built-in profile (the stored one is not used yet).
* `Reversed Transformer`: the injection went the wrong way after 128 ticks. Check the transformer connections. Nothing is stored.
* `No Response`: nothing appeared on the output at 256 ticks. No transformer, an open winding, or the stage is not switching.
* `Input Not Steady`: no good measuring window in 30 s on one step. Since v0.9.41 the screen says WHY on its third line: `Input moved 12 V` (the input itself swung more than 8 V peak to peak inside a window: the dial was turned, or the source is very soft), `Reading spread 1.3 V` (the difference output minus input was noisy, or had not settled: its spread; more than about 1 V plus 12 % of the injection is refused), or `Input out of window` (the input left the slave's input window). A refused window is measured AGAIN with the duty unchanged. See the variac notes below.
* `Aborted By Slave`: a fault in the unit (the master log shows which), the flash refused the profile, or the controller never reached RUN in 25 s.
* `Slave Not Answering`: no progress frame for 8 s. The link or the slave stopped.
* The controller has its OWN limit: if the output rises more than Max Correction + 8 V above the input, it stops and latches the over-voltage fault (code 6), whatever the sweep thinks. That is a bug or a wrong transformer, not a normal result.

## Safety
* A fault ends tune mode for good: after it is cleared the unit stays OFF until the master starts it.
* If the master stops sending `$M` for 1 s, the slave stops the sweep by itself.
* The sweep never applies more than the cap (2752 ticks) and never ramps faster than 1500 ticks/s.

## Using the stored profile (`SLAVE_TUNE_APPLY 1`, v0.9.35)
Off by default. With it on, the profile loaded at every start (a) scales the regulator's integral gain (always), (b) caps the regulator at the profile's cap ONLY if that cap is a real limit (a measured saturation knee, or a sweep that reached your Max Correction), and (c) checks at every boost start that the injection matches the profile: a different transformer latches fault 19, "Re-tune Required".
A sweep made at a high input is stopped early by the output headroom and does not cap the regulator, but it still sets the gain scale. For a complete profile tune at about 160 V (see the table above). If you change Max Correction afterwards, tune again.
`SLAVE_TUNE_ENFORCE 1` (needs APPLY and TUNE_MODE) makes a start without a profile latch fault 18, "Re-tune Required"; Re-tune works from that state. Leave it OFF until the sweep has been run on a real unit and the check's tolerance (a first value: 30 % or 1 V) has been confirmed.
Config: `..._fast_correction_100_iout_tune_apply.h` (tune mode and APPLY; ENFORCE left off).

## The tuning screen says what the slave is doing (v0.9.37)
While the sweep has not started (the first 6 to 10 s: the slave starts itself and measures the baseline), the third line of the tuning screen shows the slave's real state, so a start-up that does not progress says why:
| Third line | Meaning |
|---|---|
| `Please wait` | the master is waiting for the slave's answer to TUNE |
| `Slave Not Started` | the slave accepted TUNE but is still OFF: its converter and inverter pulses are NOT on. Nothing is starting it: see below |
| `Slave Starting...` | the start sequence is running (converter, inverter, shunts) |
| `Slave Waits: ZV...` | the start is held by: Z the zero-cross pattern is not validated, V the input is outside the start window, M no valid measurement yet, F the FLT line reads low, B a break input reads low (a FLOATING one counts), O the output is above the protection limit, R the relays have not settled |
| `Slave In RUN` | the slave is running and measuring the baseline; the steps follow |
| `Slave Fault F06` | the slave has a fault (the code): read it in the master's fault log |
`Slave Not Started` for more than a few seconds after "Starting..." means the slave never began: check that its build has `SLAVE_TUNE_MODE 1` AND `SLAVE_MASTER_CONTROL 1`, that a normal START still runs the unit, and the slave's own wait letters on the Run screen after the tune fails.

## A variac as the source (what to expect and how to help it)
A variac with the converter running on it is not a steady source: the converter's input current and the injection's magnetizing current make the voltage sag, the brush contact adds small jumps, and the mains wanders by itself. Since v0.9.38 the sweep tolerates 3 V peak to peak in a window and measures a bad window again, so a normal variac works.
To help it:
* Use the LARGEST variac you have (its rating well above the converter's input current), on a mains that does not share heavy loads.
* Set the dial first, then do not touch it during the sweep (a turn of the knob is a jump of several volts and costs a window, or the whole step if it keeps moving).
* Look at the I/P voltage on the Run screen before you start: if it moves by more than a volt or two by itself, wait or use a steadier source.
* If you still get `Input Not Steady`, the input did not stay within 3 V peak to peak for one whole 1.5 s window within 30 s on one step: the source is too soft.

## Reading the first steps: your transformer's gain
The reference transformer gives, per effective tick: 0.0255 V at 180 V in, 0.0297 V at 200 V, 0.0316 V at 230 V. So at 200 V: 16 ticks about 0.5 V, 32 ticks about 1.0 V, 512 ticks about 15 V, 2752 ticks about 82 V. Readings that are consistently 1.5 to 2 times those (for example 0.7 V at 16 ticks and 1.9 V at 32) mean a transformer with about that much more gain: the sweep finds out exactly, shortens the steps so that the injection stays under the limit, and stores a gain scale below 1 (a transformer with twice the gain gets half the regulator's integral gain when `SLAVE_TUNE_APPLY` is on). Single readings below 2 V are noisy (plus or minus 0.3 V): judge by the later steps.

## The Transformer Profile page: what was stored and what it does (v0.9.39)
LOG, then MENU x8 (the last service page; one more MENU returns to the Run screen):
```
Transformer Profile
Cap 2752 ticks            (+ " Knee" if a saturation knee limited it)
Gain B0.523 U0.523
At 160V Max 110V n10
```
| Line | Meaning | What it does when `SLAVE_TUNE_APPLY 1` (with it OFF, NOTHING: the profile is only stored and shown) |
|---|---|---|
| `Cap` | the largest effective duty the sweep verified (2752 is the hardware ceiling). A smaller number means the sweep stopped earlier: a saturation knee (`Knee`), or the 80 % limit of the Max Correction / headroom it was made with | the regulator is never asked for more than this duty, but only if the cap is a REAL limit: a measured knee, or a sweep that went as far as your own Max Correction. A sweep stopped early by the output headroom (made at a high input) is NOT applied |
| `Gain B / U` | the regulator gain scale = the reference transformer's gain / yours, between 0.100 and 1.500. Below 1: your transformer has MORE gain than the reference. B and U are always the SAME number: the sweep can only boost, so the buck scale is copied from it | the regulator's integral gain is multiplied by it: a transformer with twice the gain gets half the integral gain, so the correction is as fast and as stable as designed |
| `At 160V` | the input voltage of the sweep (the baseline) | the injection curve is scaled to the live input with the reference shape |
| `Max 110V` | the Max Correction the sweep was made with (the smaller of your setting and the headroom) | the cap is applied only while your current Max Correction is not larger than this |
| `n10` | measured points (steps) | the same-transformer check compares the live injection with this curve |
**Effect on the limits you set:** none of your Set-Mode limits (I/P LV, I/P HV, O/P LV, O/P HV, delays, overload) is changed by the profile. Max Correction stays YOUR limit; the cap can only make the regulator stop EARLIER than the hardware ceiling, so the lowest input the unit can still bring to the set voltage is the set voltage minus the injection at the cap. A cap that is much lower than the ceiling means that very low inputs will not reach the set voltage: re-tune at a lower input (about 160 V) with Max Correction 110 or more to learn the whole range.
**A stored profile from a sweep at 230 V** has a low cap (about 1000 ticks, 32 V) only because the output headroom stopped the sweep, not because the transformer cannot do more: it is not applied as a cap. Its gain scale is.
**Reading your own page:** `No profile stored (or an older slave)` means the slave sent no profile fields: nothing was ever stored, or the slave has an older firmware or `SLAVE_AUTOTUNE 0`.

## Reading a profile page: what "Knee" means, and a profile like Cap 64 + Knee (v0.9.40)
`Knee` means the sweep saw the slope of the injection fall to less than 40 % of the previous slope, and stopped one step before. A REAL saturation knee is at a large injection (a core saturates near its rated injection, tens of volts). Since v0.9.40 a slope change is only called a knee when the injection at the cap point is at least **8 V**; below that it is noise (plus or minus 0.3 V) or the converter's non-linear low-duty region, and the sweep goes on to its real limit.
**Profiles stored by v0.9.39 and earlier can show a false knee** (for example `Cap 0064 ticks Knee`, `n04`: a "knee" at 64 ticks, about 4 V). With `SLAVE_TUNE_APPLY 1` such a profile held the regulator to about 4 V of correction (a 190 V input gave 193.5 V out). From v0.9.40 the controller no longer treats a knee below 8 V as a limit, so an old profile like that is harmless; it is still a poor profile (it has only 4 points and a gain scale fitted on 2 of them), so **re-tune** to get a good one.
How the gain scale is found now: the least-squares slope through the points from the first one with at least 3 V of injection up to the cap point (it was the slope between two points, the first with 1 V). A sweep that stops early with few points gives a rough scale: look at `n` on the page; 6 or more points is a good sweep.
A good profile at 160 to 200 V in with Max Correction 60 or more shows no `Knee`, a cap in the hundreds or thousands of ticks, and n of 6 or more.

## What the sweep judges in a window (v0.9.41)
The measurement is the DIFFERENCE d = output voltage minus input voltage. A wandering input, or one that the converter's own current distorts, moves input and output together and cancels in d, so the sweep no longer asks the input to be still (v0.9.40 and earlier wanted 3 V peak to peak, v0.9.37 and earlier 1.5 V, and stopped at the bigger steps of a variac: 4 to 9 V of injection). A 1.5 s window is accepted when:
* the input moved no more than 8 V peak to peak in it (a real jump is refused: the dial was turned),
* the spread (standard deviation) of d is at most 1.0 V + 12 % of the injection, and
* the means of the window's two halves differ by at most 0.6 V + 8 % of the injection (d has settled: a slow plant is measured again until it has).
A refused window is measured again with the duty unchanged; the sweep gives up only after 30 s on one step, and the result screen tells you which of the three it was.

## The converter-current guard and the shorter hold (v0.9.43)
The sweep controls VOLTAGE and cannot see a core that starts to saturate; the converter current can. From v0.9.43 the sweep:
* measures the no-load converter current (RMS) during the 3 s baseline;
* from 0.4 s into every step, ABORTS (the duty drops to zero at once) when the RMS stays above max(3 x the baseline, baseline + 2 A), or the peak above 20 A, for 3 readings in a row, or the amplifier clips. The master says `Current Too High / Converter 5.2 A` (`Converter at limit` if it clipped);
* DOES NOT RUN without a converter-current reading (`SLAVE_IOUT 1`): `No Current Reading / Needs SLAVE_IOUT 1`;
* holds a step whose window is refused for 9 s at most (it was 30 s) before giving up: while the duty is on, a source that will not settle is a reason to stop, not to wait.
LIMITS: the current is measured over about 100 ms, so this catches a current that rises, not a saturation inside ONE mains cycle. Only the hardware over-current trip protects against that: set it (see the SAFETY section). The thresholds (3 x, 2 A, 20 A) are first values, to be set from the bench.
## Reading two real profiles (EI core and toroid)
* **EI core, Max Correction 75, 206 V in: `Cap 2752`, `Gain B1.500 U1.500`, `Max 64V`, `n10`.** The sweep ran all ten steps up to the hardware ceiling without a knee. `Max 64V` is the output headroom (280 - 10 - 206), not your 75 V. **1.500 is the upper limit of the scale (0.1 to 1.5), not a measured value**: the true figure would be higher. The injection per tick falls with duty (about 0.025 V/tick at 1024 ticks, less above), so one scale cannot suit both small and large corrections.
* **Toroid, Max Correction 50, 240 V in: `Cap 0886`, `Gain B1.186 U1.186`, `Max 30V`, `n07`.** The headroom at 240 V is only 30 V, so the sweep stopped at 80 % of it (24 V) on a shortened seventh step of 886 ticks: that cap is the headroom's, not the transformer's, and it is NOT applied as a cap (it was made with less than your Max Correction). The gain scale 1.186 (the transformer has about 0.84 of the reference gain) is valid for the first 24 V only.
* **The toroid was run twice**: an earlier run (Max Correction 75) stopped after step 7 (19.2 V at 1024 ticks) with `Input moved 8 V`; the later run above (Max Correction 50, 240 V in) finished. An input that swings 8 V at only 19 V of injection can be a gapless core starting to saturate (sharp, asymmetric magnetizing-current spikes make the source jump), or just a soft source: the converter-current guard of v0.9.43 is there to tell the two apart.
* To learn the whole range of either core, re-tune at 160 to 200 V in; a gapless toroid deserves a scope on the primary current the first time above 20 V.

## How to check that a profile is stored, and whether it is in use (v0.9.46)
* **Stored:** the profile lives in the SLAVE's flash (pages 124 and 125, 0x0803E000 and 0x0803E800), CRC-checked, two copies. It survives a firmware update unless the programmer ERASES THE WHOLE CHIP first ("mass erase", "erase all"): that also wipes the slave's voltage calibration (pages 126 and 127). Use "erase only the pages that are programmed".
* **On the master:** LOG, then MENU x8 = **Transformer Profile**. `Cap ... Gain ... At ... Max ... n` = a valid profile is stored. `No profile stored (or an older slave)` = there is none, OR the slave is not built with `SLAVE_AUTOTUNE 1` (it is that switch that sends the numbers).
* **When the page updates:** the numbers come in the slave's HELLO, which the slave sends only at its power-up (three times) and when asked. Since v0.9.46 the master asks again when a Re-tune finishes ("Tuning Complete"); in v0.9.45 and earlier the page kept the old numbers until BOTH boards were restarted.
* **In use:** a stored profile is USED (the gain scale reaches the regulator, a measured saturation knee caps it, and a different transformer stops the unit with "Re-tune Required") when the slave is built with `SLAVE_TUNE_APPLY 1`, which since v0.9.47 is the DEFAULT of a production build. The master cannot show that switch; check `slave_config.h`. Without it the profile is stored and shown, nothing more.

## What changed in v0.9.47 (the defaults, and Max Correction)
* **Max Correction is never reduced by the tune.** The profile's gain scale is always applied; its cap is applied ONLY when it is a measured saturation knee (the transformer truly cannot deliver more, at 8 V of injection or more). A sweep that stopped at 80 % of Max Correction, or that the output headroom stopped early, only says "verified up to here" and caps nothing.
* **Defaults.** In a production build (power stage 3, master control, link v2) `SLAVE_AUTOTUNE`, `SLAVE_TUNE_MODE`, `SLAVE_TUNE_APPLY`, `SLAVE_TUNE_ENFORCE` and `SLAVE_IOUT` are ON unless the config says otherwise; in any other build they are OFF. They follow one another (a config that sets `SLAVE_AUTOTUNE 0` switches the rest off, `SLAVE_TUNE_APPLY 0` switches ENFORCE off), so the build-time checks cannot fire by accident.
* **What ENFORCE means for a new unit:** a unit with NO valid profile does not run: the master shows "Re-tune Required". Do Set-Mode, item 19 Re-tune (Max Correction set, the unit stopped, an input with at least 20 V of headroom, about 200 V or lower, from a current-limited source the first time: read the SAFETY section at the top of this document). After a successful tune the unit runs.
* **Inrush retries:** the master now defaults to 3 (`MASTER_INRUSH_RETRIES`): an inrush or power-stage trip is retried after the Restart Time (at least 30 s) up to 3 times, then a 5 minute wait and one more attempt, then it latches. Set it to 0 while commissioning or repairing a stage. The overheat input stays off (`MASTER_HEAT_INPUT 0`).

## Three phases: Re-tune per phase (v0.9.54)
In Industrial and Domestic, Set-Mode item 19 Re-tune offers `R Phase`, `Y Phase`, `B Phase` or `All, One By One`. The command goes to that phase's slave over its own serial port; the tuning screen shows the phase letter at the right of its first row; LOG, then MENU x8 shows the three profiles in turn (`Transformer Prof. R` / `Y` / `B`). `All` tunes R, then Y, then B, only ever one stage at a time, and stops at the first phase that does not end with `Tuning Complete`.
**Before tuning ANY phase, do the SAFETY section above on THAT stage** (the over-current trip lowered, the desaturation checked, a current-limited source). Tune one phase first by itself; use `All` only once each stage has been tuned safely on its own.
