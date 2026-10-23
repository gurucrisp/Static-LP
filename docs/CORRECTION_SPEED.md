# Correction speed (v0.9.24)

## What limits it (measured in the simulation, 2752-tick cap)
| input step | within 5 V at 30 V/s (default) | at 60 V/s | at 100 V/s |
|---|---|---|---|
| 230 -> 220 V (10 V, no guard) | 0.34 s | 0.34 s | 0.34 s |
| 230 -> 200 V | 0.98 s | 0.56 s | 0.40 s |
| 230 -> 180 V | 1.66 s | 0.90 s | 0.62 s |
| 200 -> 250 V | 0.62 s | 0.38 s | 0.28 s |
| 230 -> 260 V | 0.96 s | 0.54 s | 0.38 s |
* A step of 15 V or more makes the injection guard hold the correction at zero, then restart it and ramp it back at `CTRL_GUARD_RESTART_SLOPE_VPS` (30 V/s). That ramp is the recovery time: about step / slope.
* A change under 15 V is not a guard event: it waits for the freeze debounce (`REG_FREEZE_DEBOUNCE_MS`, 300 ms) and then the regulator corrects. That is the 0.34 s.
* The regulator's integral gain (`REG_KI_SCALE`) made no difference in the simulation (0.98 s at x1, 0.94 s at x3): the ramp, not the gain, is the limit. Left at 1.0.

## The profile
`slave_config_STAGE3_cap2752_full_range_MASTER_fast_start_fast_correction.h`: guard restart slope 60 V/s. Everything else as the fast-start master-control config.

## Why not the debounce
At 150 ms the transformer-flip tests failed (5.2 V instead of under 5 V of wrong-way output before the stop, 11 V instead of under 10 V on a buck unit); 200 ms passes narrowly. The protection matters more than 0.1 to 0.2 s on small steps.

## A ladder for the bench (variac, no load first, then a load)
1. Plain fast-start config (30 V/s): note the recovery time and that nothing trips on 20 V and 50 V steps.
2. The fast-correction config (60 V/s): the same steps. Any F02 / F01 that did not happen before: go back.
3. Only if both are quiet: 100 V/s (`#define CTRL_GUARD_RESTART_SLOPE_VPS 100.0f` in the config).
The simulation cannot tell what a real transformer and IGBT stage suffer from a faster ramp: the bench has the last word.

## v0.9.25: the response timer and the second profile
**Measure instead of guessing.** LOG, MENU x2 (the diagnostics page): row 0 shows `S:Y E:Y H:Y 3.4s/41V` after the first recovery: the time from the output being more than 5 V from the set voltage until it is back within 1 V (3.4 s), and the largest error in between (41 V). Do a 20 V and a 40 V variac step up and down and write the four readings down.
Only recoveries while the unit runs with the contactor closed are timed; an excursion longer than 30 s is dropped (it is not a recovery).

**The profiles (all need a slave reflash; the plain configs are unchanged)**
| config | guard slope | final slope | 40 V step to within 0.5 V (simulation) |
|---|---|---|---|
| fast_start (plain) | 30 V/s | 10 V/s | 1.6 s |
| fast_start_fast_correction | 60 V/s | 10 V/s | 1.0 s |
| fast_start_fast_correction_100 | 100 V/s | 30 V/s | 0.6 s |

**What is NOT offered and why**
* A higher regulator gain: no speed gain in the simulation, and the transformer-flip check lets the output move further the wrong way before it stops (buck unit, test limit 3.5 V): x1.0 1.46 V, x1.5 3.38 V, x1.75 4.88 V, x2.0 6.76 V. That is a protection; it is not traded for speed.
* A shorter freeze debounce: at 150 ms the flip tests failed too.

**If the real unit is still 3 to 4 s with the 100 profile**
Then the limit is in the real stage (its response lag, the real injection gain at low input, the duty cap), which the model does not have. Send me the timer readings and which config is flashed; the next step would be a trace of the duty and the voltages during a step.
