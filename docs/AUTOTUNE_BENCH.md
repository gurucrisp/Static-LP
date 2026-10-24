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
* `Input Not Steady`: the input moved more than 1.5 V during a step. Use a variac, wait, retry.
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
