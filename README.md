# Crisp static stabilizer — firmware repository

Slave: `slave/Core` (STM32G0B1RET6, "PB LP V1.0": converter, inverter, bypass relays, measurement, zero-cross).
Master: `master/Core` (STM32G070RBT6, "DISP SS3S LCD4 M1 V1.1": LCD monitor of the slave link).
**Current version: see `VERSION`.** History: `CHANGELOG.md` / `git log`. This repository replaces all the earlier loose zip files.

## Layout
| Folder | Content |
|---|---|
| `slave/Core`, `master/Core` | The CubeIDE project `Core` folders (Src, Inc, Startup). Copy them over your project's `Core`. |
| `docs/` | `RELAY_LOGIC_REVIEW.md` (what the relays are and the rules), `POWER_STAGE_BRINGUP.md` (stages, fault codes, display, procedure), `static_stabilizer_spec.md` |
| `tests/`, `tools/run_tests.sh` | Host tests (gcc only, no hardware) |
| `tools/which_version.py` | Which version is in a folder on your PC |
| `tools/check_config.py` | Checks a `slave_config.h` BEFORE you build it (missing final `#endif`, a comment left open, two `CTRL_VIN_MIN_V` lines) |
| `configs/` | Ready-made `slave_config.h` files (90 V step 1 and step 2): copy over `Core/Inc/slave_config.h` instead of editing by hand |
| `legacy/` | The old firmware, reference only |
| `MANIFEST.sha256` | SHA-256 of every file of this version |

## Which version is in my project?
    python tools/which_version.py slave  <path to your slave project>/Core
    python tools/which_version.py master <path to your master project>/Core
(needs git on the PATH). Without git: `sha256sum -c MANIFEST.sha256` from the repository root, after copying your files into `slave/Core` and `master/Core`.

## Apply to your projects
1. Slave: copy `slave/Core/Src` and `slave/Core/Inc` over your slave project's `Core/Src` and `Core/Inc`. Master: the same with `master/Core`. Rebuild, flash BOTH.
2. `slave/Core/Inc/slave_config.h` holds the settings (stage, autorun, report mode, relay mode, relay contact types). Check it before building.
   **Since v0.9.56 the repository's `slave_config.h` IS the production config** (stage 3, master control, link v2, I-OUT, autotune with the profile applied; the same file as `configs/slave_config_STAGE3_cap2752_full_range_MASTER_fast_start_fast_correction_100_iout_tune_apply.h`). Copying `slave/Core/Inc` gives a working master-controlled slave. The old bring-up default (stage 1, no master control) is `configs/slave_config_BRINGUP_stage1_autorun0.h`. If you keep your own edits (for example a different `CTRL_VIN_MIN_V`), put them back after copying.
   With this config a slave **without a stored transformer profile refuses to run** ("Re-tune Required", `SLAVE_TUNE_ENFORCE`): Re-tune that phase (AUTOTUNE_BENCH.md, SAFETY first), or add `#define SLAVE_TUNE_ENFORCE 0` before the last `#endif` until it is tuned.
3. **Stages 2 and 3 will not build until you have measured the relay contacts** and set `SLAVE_RELAY_SH_CONTACT` / `SLAVE_RELAY_W_CONTACT` (`docs/RELAY_LOGIC_REVIEW.md`, section 6).
4. The two link files `link_proto.c/.h` must be identical in both projects (they are, in every tagged version).

## Run the PC tests
    tools/run_tests.sh quick     # a few minutes
    tools/run_tests.sh full      # adds the slow simulations
(needs gcc and a POSIX shell; on Windows use Git Bash or WSL.)

## Working rules (to avoid the zip-file confusion)
* One change = one commit; one delivered package = one tag.
* Never edit a tagged state; make a new commit and a new tag.
* Your own settings live only in `slave_config.h`.
