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
