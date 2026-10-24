# Link library, version 2 (D1, D2, D12)

`slave/Core/Inc/link_v2.h` and `slave/Core/Src/link_v2.c` are SHARED SOURCE: the slave project, the master project and the PC tests compile the same two files (the runner checks
that the copies are byte-identical, together with `link_proto.*`). No HAL, no globals, no dynamic memory, no libc calls.

Version 1 (`link_proto.*`, the `$S` status frame) is frozen. Version 2 adds the frames of the approved workbook and one parser that knows them all:

| frame | direction | builder | what it carries |
|---|---|---|---|
| `$S` | slave -> master | `Link_BuildStatus` (v1) | status every 10 ms |
| `$E` | slave -> master | `LinkV2_BuildE` | seq, set voltage in use, flags, clear counter echo, reach |
| `$H` | slave -> master | `LinkV2_BuildH` | protocol, firmware, input window, reset cause, tuned, output guard |
| `$R` | slave -> master | `LinkV2_BuildR` | response to a `$K`: id, status, value |
| `$T` | slave -> master | `LinkV2_BuildT` | autotune progress |
| `$M` | master -> slave | `LinkV2_BuildM` | run, set voltage, clear counter, Max Correction (a LEVEL, every 100 ms) |
| `$K` | master -> slave | `LinkV2_BuildK` | acknowledged commands (HELLO, CALV, CALO, CALR, TUNE, TABORT, TPARAM, TSAVE) |

Receiving: feed every byte to `LinkV2Rx_Feed`; it returns the frame type when a valid frame completed. `LinkV2_Parse` takes one line.

## Rules the library enforces

* Framing as in version 1: `$<type>,<fields>*<CS><CR><LF>`, XOR checksum, UPPER-CASE hex, 96 bytes at most.
* More fields than defined: accepted, the extra ones ignored. Fewer: rejected. Unknown frame type with a good checksum: counted (`frames_unknown`) and ignored.
* The documented range of every field is one table per frame type, used by the builders AND the parser, so a frame that cannot be built cannot be parsed either. The checksum is only 8 bits wide;
  the ranges are the second line of defence for the frames that command the power stage ($M).
* A `$K` with a command word this version does not know parses as `LINKCMD_UNKNOWN` (so the slave can answer `UNKNOWN_CMD`); the builder cannot build it.
* A version-1 receiver (`LinkRx_Feed`) does not count a v2 frame as BAD: it counts it in `frames_other`.

## What tests/test_link_v2.c proves (the runner runs it)

Every example of the workbook parses and rebuilds byte for byte; every field's minimum, maximum and one-step-outside values; malformed input of every kind; every `$K` command; the stream
receiver through garbage, a lost tail, bad checksums, unknown types and over-long lines; 4 million random bytes; 600,000 random frames in and out of range (the builders and the parser agree);
valid-checksum frames with random edits; every single-byte substitution of every example (63,240: none accepted with different content); lengths; version-1 compatibility.
Nine deliberate breakages of the library are all caught.

The examples in `tests/link_v2_examples.inc` are generated from the workbook: `python3 tools/gen_link_examples.py <workbook> [--check]`.


## `$E` optional fields (v0.9.30)
`$E,seq,setv,flags2,clr,reach_lo,reach_hi[,iout_rms_ca,iout_pk_ca]`: the last two (the converter-side current, hundredths of an ampere, 0..4000) are OPTIONAL. A sender without the channel omits them (the frame is byte for byte the 6-field frame of the first protocol); a receiver that gets only 6 fields reads them as 0; one that gets 7 reads the peak as 0;
a present optional field must be in range or the frame is rejected; extra fields beyond 8 are ignored. Flags: `IOUT_VALID` 0x0200 (the slave measures it), `IOUT_CLIP` 0x0400 (the measurement is at the limit of its range).


## The autotune over the link (v0.9.32)
`$K,id,TUNE`: start a sweep. R status 0 = started. 4 NOT_STOPPED = the unit runs (send run = 0). 7 NOT_ALLOWED, R value = the reason: 1 this build has no tune mode, 2 Max Correction not set, 3 a calibration or flash write is busy, 4 a sweep is already running.
`$K,id,TABORT`: stop at once, always status 0. `TPARAM` / `TSAVE`: UNKNOWN_CMD in this version.
`$T,step,ticks,vin_dv,dv_dv,result`: one frame per step (result 0), then the final frame (result 1 FINISHED, 2 FLIPPED, 3 NO_RESPONSE, 4 ABORTED, 5 UNSTEADY, 6 BY_MASTER). The final frame is sent again 6 times, 0.5 s apart (`$T` is not acknowledged): a master that sees the same final frame more than once ignores the repeats.
FINISHED is sent only after the profile has been stored; if the flash refuses, the result is 4. `$E` flags: BUSY 0x0080 while a sweep runs, NOT_TUNED 0x0100 without a valid profile or without Max Correction.

**Added in v0.9.34:** the `TUNE` refusal reasons now also include 5 (the controller cannot start now: a fault is latched, or it is not OFF) and 6 (the input is too high: less than 20 V of headroom under the output limit). A started TUNE goes through a PREPARATION of about 6 s (the controller starts itself); during it, and while nothing else is sent for 2 s, the slave sends a HEARTBEAT `$T` with step 0, ticks 0, result 0 (so the master does not time out). A master treats step 0 as "preparing / measuring the baseline". Result 4 (ABORTED) also covers a controller fault during the start-up and a controller that does not reach RUN in 25 s.

**Added in v0.9.39:** `$H` has 7 OPTIONAL trailing fields, sent only when the slave has a stored profile (a slave without one sends the 7-field frame of the first protocol): `cap` (0..4095), `kib`, `kiu` (0..65535, thousandths), `vref` (tenths of a volt), `maxc` (tenths of a volt), `n` (0..15) and `knee` (0/1), so `$H,proto,fw,vin_min,vin_max,rst,tuned,vout_max,cap,kib,kiu,vref,maxc,n,knee`. An older receiver ignores the extra fields; a receiver reads missing ones as 0; a present field must be in range. The longest `$H` is 73 characters (the limit is 80).
