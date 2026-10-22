# Master control (v0.9.0, roadmap step 3; decisions D1, D5, D9, D12)

Two switches in the config (defaults are in `slave_features.h`, which no ready-made config replaces, so every older config still builds with both OFF):

| switch | meaning |
|---|---|
| `SLAVE_LINK_V2 1` | the slave also sends `$H`, `$E`, `$R` and reads `$M`, `$K` (see LINK_V2.md). 0 = exactly the v1 behaviour. |
| `SLAVE_MASTER_CONTROL 1` | the MASTER runs the slave. Needs `SLAVE_LINK_V2 1`, a power stage and `SLAVE_BENCH_AUTORUN 0` (a `#error` says so otherwise). |

Ready-made file: `slave_config_STAGE3_cap2752_full_range_MASTER.h`. Without a master it stays OFF and flags LINK_LOST: bench it alone with the non-MASTER file.

## What master control changes

* **No autorun.** `run` in `$M` is a LEVEL, restated every 100 ms.
* **Every fault latches** (`CTRL_LATCHED`) at once, until the clear counter in `$M` CHANGES. No retry timer, no escalation count, conditions (input out of window, lost zero-cross) included.
  The first `$M` after a start only adopts the counter. A clear acts only on a latched fault; the normal start then checks the cause again (FLT or a break input still asserted keeps it in SYNC).
  Clear with run = 1 restarts; clear with run = 0 leaves it OFF.
* **Link loss (D9):** no valid `$M` for 1 s (`CTRL_LINK_TIMEOUT_MS`, counted from the start too): run is dropped, the unit stops normally (not a fault), `LINK_LOST` is flagged. Valid frames with run = 1 start it again.
* **Set voltage:** adopted while the unit is NOT energised; while energised a different value is ignored. The master repeats it every 100 ms, so it is picked up the moment the unit stops. `$E.setv` is the one in use (0 = none accepted yet).
* **`$E.flags2`:** RUN_ACK (run = 1 and link alive), IN_BAND (running, |Vout - set| <= 2.0 V, `CTRL_IN_BAND_V`), AT_CAP (stage 3, at the correction cap), LINK_LOST, FAULT_LATCHED.
* **`$H`:** protocol 2, firmware (`SLAVE_FW_VERSION`, checked against VERSION by the runner), input window, reset cause (0 power-on, 1 watchdog, 2 other), tuned = 1 (built-in profile), output guard.
* **Not in this version:** every `$K` command except HELLO is answered UNKNOWN_CMD (status 6); `$M.maxc` is stored but unused; `$E.reach` is 0; heat and not-tuned flags are 0 (the transformer-flip check exists from v0.9.18: slave fault 17). Steps 4 to 6.

## Hardware side

* `USART1_IRQHandler` only moves received bytes into a 256-byte ring (`SlaveLink_RxIrq`). Priority `IRQ_PRIO_LINK` = 3, the lowest, so it cannot delay the FLT shutdown or the timing; `tools/check_irq.py` enforces that.
* The USART's 8-byte FIFO is switched on (`SLAVE_LINK_RX_FIFO 1`) to give the interrupt about 160 us of slack at 500 kbaud. Counters `g_link.rx_overruns` / `rx_dropped` show any loss. **First bench check: watch `rx_overruns` stay at 0. If the link misbehaves, set `SLAVE_LINK_RX_FIFO 0`.**
* `$M` and `$K` are acted on in the main loop, next to the controller, never in the interrupt.

## Tests (the runner runs them)

`s3_master_control` (170 checks): no autorun, run/stop/restart, in band (and NOT in band 5 V away), a fault latching for a minute, the clear counter in every combination, first-counter adoption, link loss and 0.9 s tolerated, set-voltage rule, a condition fault, and an end-to-end group through real frame bytes and the real link logic. `test_slave_link2.c` (50 checks): hello timing, `$E` cadence and gaps, `$K` retries, queues, 32-bit time wrap in 40 alignments, fuzz. Fifteen deliberate breakages of the two modules are caught.
