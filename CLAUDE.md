# Crisp static stabilizer firmware (master + slave)

Static (IGBT buck/boost + triac, no servo motor) voltage stabilizer, single and three phase. Maintained for Crisp System India Pvt Ltd (Coimbatore). The user is the firmware/hardware engineer who benches every release; they read answers on a phone: **short, direct answers; when they say "just answer", do not write code.**

## Hardware
* **Master** `master/`: board DISP SS3S LCD4 M1 V1.1, STM32G070RBT6 (128 KB flash, 16 MHz HSI), 4x20 LCD, 4 keys, 3 contactor outputs PA10/PA11/PA12 (high = closed), CT on ADC ch0.
* **Slave** `slave/`: CRISP_PIGGYBACK_CIRCUIT, STM32G0B1RET6. Drives the IGBT H-bridge (PWM period 3200 ticks, dead time 128), triacs/relays; ADC 20 kHz scan of AC_IN, AC_OUT, NTC (PA5), I-OUT (converter current, CT2500, burden 68 ohm x3 = 22.67 ohm, clips at +-26.8 A peak). IGBT IKW50N120CS7. Hardware over-current comparators trip at about 108 A peak.
* **Link** UART 500000 baud, text frames `$S $E $H $T $R $M $K` (docs/LINK_V2.md). One link per slave (no addressing): R = USART1 PC4/PC5, Y = USART4 PC10/PC11, B = USART3 PB2/PB0. Each slave board is built with its own `SLAVE_PHASE_ID` ('R' default, 'Y', 'B') in its `slave_config.h`; the master refuses a link with the wrong letter.

## Layout
`slave/Core/{Src,Inc}`, `master/Core/{Src,Inc}` (CubeIDE projects, sources only), `tests/` (host tests, plus `stubs/`, `hal_check/` master HAL stand-in, `hal_check_slave/`), `tools/`, `configs/` (ready-made `slave_config.h` variants), `docs/`, `CHANGELOG.md` (the full history, newest first), `VERSION`, `MANIFEST.sha256`.
Docs to read before touching an area: MASTER_UI.md, MASTER_CORE.md, LINK_V2.md, AUTOTUNE_DESIGN.md + AUTOTUNE_BENCH.md (SAFETY section first), IOUT.md, SLAVE_LOAD_CAL.md, OVERHEAT.md, PHASE_SEQUENCE.md, THREE_PHASE_PLAN.md (decisions + bench steps).

## Commands
* `tools/run_tests.sh quick` (about 5 min, `NPROC=8`): must end `ALL TESTS PASSED`, 0 warnings (89 groups at v0.9.51). `python3 tools/check_version.py`; `python3 tools/which_version.py`.
* One suite by hand, e.g. `gcc -std=c99 -O2 -Wall -Wextra -Wshadow -Imaster/Core/Inc tests/test_m_phase.c master/Core/Src/m_phase.c -lm`; the runner's lines show the exact source lists. Anything that links `m_app.c` also needs `m_ntc.c` and `m_phase.c` (also in the slave harness job lines).
* Docs: after any file change regenerate `MANIFEST.sha256` (`find slave master docs configs tools tests -type f -not -path '*/__pycache__/*' | LC_ALL=C sort | xargs sha256sum`).
* No ARM toolchain here: the HAL glue (`master_monitor.c`, `slave_link.c`, `slave_zc.c`) is only compile-checked against stand-in headers by the runner; its behaviour on the target is untested until the user benches it.

## Invariants (do not break)
* `VERSION` and `SLAVE_FW_VERSION` (slave/Core/Inc/slave_features.h, e.g. 951UL) must match.
* `link_v2.c/.h` are IDENTICAL in `slave/Core` and `master/Core`: edit one, `cp` to the other, `cmp`.
* `master/Core/Src/main.c` has CRLF line endings: edit it preserving them (Python `newline=''`). Everything else is LF.
* Master flash: the image must end below 0x0801C800 (116,736 bytes) or the boot message "Flash Full / Settings Not Saved" appears; build with `-Os`. The master has grown with every release: be sparing, and tell the user to watch for that message.
* New source files must be added to the CubeIDE projects by the user (slave: `slave_phase.c`; master: `m_ntc.c`, `m_phase.c`, `master_links.c`): say so in the release notes.
* Settings record (master): version 2, 48 bytes; bytes 0 to 31 keep their layout, 32 = Overheat Restart, 33 to 47 are FREE (reserved for per-phase load calibrations). `MS_Load` migrates the version 1 / 32 byte record. Never change a layout without a migration and a test of an old record.
* Slave flash pages: profile 124/125, voltage calibration 126/127. Master pages 0x0801C800 to 0x0801F7FF (log, calibration, settings).
* The autotune switches (`SLAVE_AUTOTUNE`, `TUNE_MODE`, `TUNE_APPLY`, `TUNE_ENFORCE`, `IOUT`, `SEQ_SYNC`) default ON in a production build (stage 3 + master control + link v2) and follow one default; `run_tests.sh` `pin_off()` pins them to 0 for harness jobs that do not ask for them.
* Slave `$E` frames are built field by field in `slave_link2.c`: every field of `LinkE_t` must be initialised there (a missed field once leaked stack garbage; a regression test covers it).

## How a change is made here (the user expects this every time)
1. Read the code first; never answer from memory about what the firmware does.
2. Tests first on the PC, including the awkward cases; then **mutation checks**: break the new logic several ways and confirm a test fails for each (a test that passes on broken code is not a test).
3. Full runner, `CHANGELOG.md` entry (what, why, tests, what is NOT proven), docs, bump `VERSION` + `SLAVE_FW_VERSION`, commit, tag `vX.Y.Z`, rebuild the packages. The user sometimes asks to fold a change into the same version: then keep the number, move the tag, and keep the old zip aside.
4. Say plainly what is not proven on hardware. Report real mistakes (bugs found by the tests) in the changelog.

## Decisions made with the user
* Contactors: Domestic PA10/PA11/PA12 = R/Y/B; Industrial = PA10 only. Master pins PA5/PA6/PA7 are the old firmware's phase inputs and are NOT used (PA5 is the R phase input: `MASTER_HEAT_INPUT` stays 0).
* Phase sequence and phase-to-phase voltage are measured by the SLAVES (`$K SEQ` + one SYNC byte 0x16 on all links, each slave reports delay to its next rising zero-cross); PHASE_SEQUENCE.md.
* Max Correction is the installer's and is never reduced by an autotune: the stored cap applies only for a measured saturation knee; the gain scale is always applied (with `SLAVE_TUNE_APPLY`). `SLAVE_TUNE_ENFORCE`: no run without a valid profile.
* Load current: Set-Mode "Load Source" Master CT / Slave; the slave source converts the converter PEAK (the RMS does not follow the load: 0.9 A RMS at 10 A load) with a zero and gain from the "Slave Load Cal" page; falls back to the master CT whenever the slave's reading is not trustworthy.
* Overheat: slave NTC 10k to ground, 1.5k to 3.3 V on PA5; the master converts (table, beta 3950 ASSUMED: ask for the part number), "Overheat Temp" default 75, "Overheat Restart" 35 to Overheat Temp - 5 (default 65), shown on the Run screens, per phase.
* Inrush retries default 3 (`MASTER_INRUSH_RETRIES`), 0 while commissioning. Factory Reset keeps the load calibration.
* Per-phase Re-tune: one phase at a time, never three stages at once.

## State (v0.9.51, committed and tagged)
Nothing from the autotune onwards, the three-phase work, the NTC or the phase sequence has been proven on the bench. A power stage was destroyed during an early autotune sweep: BEFORE any sweep on any stage the user must lower the hardware over-current trip (about 1.5 to 2 x the largest normal peak, not above about 75 A; reference = I_trip x 22.67 ohm / 2500), check the gate-driver desaturation, and use a current-limited source.
Open: NTC part number / beta; three-phase bench bring-up (links alive with stages unpowered first, then Domestic, then Industrial); per-phase Re-tune (decision above) and per-phase load calibration (bytes 33 to 47 of the settings record); Y and B temperatures are protected but not displayed; power factor needs the load current waveform into a slave ADC input (the master CT is a plain average, the slave's I-OUT is the converter current): unanswered whether the CT signal is raw AC; the zero-cross reference must be the rising edge on every board.
