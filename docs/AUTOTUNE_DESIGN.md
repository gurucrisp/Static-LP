# Transformer autotune: design and status (v0.9.28)

## What the user decided
* Autotune runs ONCE, automatically at the first start (no profile stored): no load, the output contactor open.
* EVERY start checks that the same transformer is fitted; if not, the unit stays off (latched "Re-tune Required"), it never re-tunes by itself.
* A password-protected Set-Mode **Re-tune** item lets the installer accept a new transformer (a transformer swap does not change the voltage sensors: the voltage calibration stays).
* The profile has its own flash record (not touched by Factory Reset, calibration or a normal firmware update).

## Done: the core (slave_tune.c, pure, 53 PC checks)
| | |
|---|---|
| sweep | baseline 3 s, then 16, 32, 64, 128, 256, 512, 1024, 1600, 2000, 2752 effective ticks, 3 s each (1.5 s settle, 1.5 s averaged); about 33 s |
| per step | predicted injection kept at or below 80 % of Max Correction (the step is shortened); one `$T` frame |
| stops, nothing stored | flipped (wrong-way injection beyond 1 V after 128 ticks), no response (under 0.5 V after 256 ticks), fault / FLT / break, input unsteady (over 1.5 V peak to peak), injection more than 5 V above Max Correction (backstop), master TABORT |
| ends with a profile | end of the list, 80 % of Max Correction reached, or a saturation knee (slope under 40 % of the previous: the cap is the step before it) |
| profile | the curve, the cap, the regulator gain scale (limited to 0.1 .. 1.5), at most 61 bytes |
| model check | measured injection against the prediction for that input voltage: wrong sign = flip, off by more than 30 % (or 1 V) = mismatch, under 3 V predicted = not judged |

## Assumptions that need the bench (each is a number or a model I chose)
1. **How the injection changes with the input voltage** is taken from the shape of the bench curve (the one the host tests use: 0.011 V/tick at 110 V, 0.0297 at 200 V, 0.0328 from 250 V). The sweep is made at one input voltage only. A transformer with a different shape would make the model check less accurate away from the tuning voltage.
2. **The reference gain** for the regulator scale is that same curve: a transformer like the bench one gets a scale of 1.0.
3. **The buck gain** is taken as 2 x the boost gain (the reference design's 1.8 / 0.9 ratio terms) for the reach at the high end only. The sweep measures boost only.
4. **Tolerance 30 % or 1 V** for the model check: a first value. It must be small enough to catch a swapped transformer and large enough never to stop a good one at a customer (measure the real spread over temperature and input voltage).
5. The numbers 80 %, 5 V, 1.5 V, 40 %, 0.5 V, 1.0 V, the 3 s steps.

## Done: the profile storage (v0.9.29, slave_profile.c, 80 PC checks)
Two CRC-protected copies in flash pages 124 and 125, validated on load, a power cut at any point leaves the old or the new profile. `SLAVE_AUTOTUNE 1` (default 0) initialises it at start and makes `$H tuned` follow it.

## Done: the link (v0.9.32, slave_tune_link.c, 30 PC checks)
`$K TUNE` (refusals with reasons: 4 NOT_STOPPED; 7 with 1 no tune mode / 2 no Max Correction / 3 busy / 4 already running), `$K TABORT`, the `$T` frames (the result repeated 6 times), `$E` BUSY / NOT_TUNED, the profile stored before FINISHED is reported.
Tune mode is still refused ("no tune mode"): the controller does not apply the duty yet.

## Done: the master side (v0.9.33, 37 + 12 PC checks)
The Re-tune item (behind the Set-Mode password) sends TUNE / TABORT, follows the `$T` frames and shows the progress and result screens; the unit is held off, only STOP works; every refusal and failure is explained in words. Not yet: the automatic first tune after the Restart Time.

## Not done yet (in this order)
1. **The automatic first tune** after the Restart Time (a small addition on the master).
2. **Controller integration**: a tune mode in the controller (the normal start sequence up to RUN, but the duty is the tune's open-loop boost request and the regulator is off); applying the cap and the gain scale; refusing closed loop until tuned (code 19); the model check during the soft start; the loop-instability guard (code 18).
4. **Bench validation**, on a variac, no load. The F02 / FLT gate is open (a hardware fault, cleared); a scope on a gate drive for the first run.

## Safety stance
The sweep only ever asks for a duty; the controller must still apply every existing protection (FLT, break, the hardware cap, the master's contactor open). The backstop (injection above Max Correction + 5 V) is independent of the prediction. The first hardware run is on a variac, no load, with Max Correction set low.


## Done: the controller's tune mode (v0.9.34, SLAVE_TUNE_MODE, 40 PC checks on the real controller)
`SlaveCtrl_TuneBegin / TuneRequest / TuneEnd / TuneRunning / TuneReady`. TuneBegin needs: the controller OFF, nothing latched, the master there, Max Correction set. The unit then goes through the normal start sequence by itself (the master's run stays 0) and in RUN, instead of the regulator, applies the requested duty OPEN LOOP as a boost: slew-limited (1500 ticks/s up, at once down), capped at the hardware cap, and with the controller's OWN injection limit (Max Correction + 8 V, otherwise fault 6, latched). Every normal protection still runs. ANY fault, the loss of the master (1 s without `$M`) or a reset ends tune mode.
The link adds a preparation phase: the sweep starts only when the controller has been in RUN for 1 s; until then the master hears a heartbeat `$T` (step 0) every 2 s. The sweep's limit is the SMALLER of Max Correction and the headroom under the output over-voltage limit; a TUNE is refused (reason 6) with less than 20 V of headroom.
**Not done:** the profile is not USED yet (the regulator ignores the stored cap and gain scale), the start-up transformer check (a swapped transformer latches a fault) and "refuse closed loop until tuned" are not built, and the automatic first tune after the Restart Time. These come after the first bench runs have shown the sweep behaves on a real unit.
