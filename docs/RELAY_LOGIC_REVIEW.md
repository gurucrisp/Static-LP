> **v0.9.65.** The FLT (CT comparator) response no longer switches the low sides off: high sides off, low sides on, shunts closed, then the inverter disabled (`SLAVE_FLT_KEEP_LOWS`). The open-primary window after a fault below now applies to the break input (BKIN) only.

> **CORRECTION 2 (v0.6.7).** Rule R3 below (never leave the primary open and uncontrolled) is CONFIRMED by the old firmware, but not the way I implemented it: the old firmware keeps the inverter LOW sides ON
> (zero duty) from boot until the converter's second diagonal, shorting the primary, and it runs its relay sequence (SH pulse, then W on = nets open) underneath that. v0.2.0 removed that state (I treated 1L / 2L
> glowing as a defect) and every converter start with W on then tripped. The measured relay contacts: W-RLY normally closed, SH-RLY normally open (continuity with both off, none with W on / SH off).
> Rules R4 / R5 and the "converter after the shunts close" ordering of mode 1 contradict the old sequence and are retracted.

> **CORRECTION (v0.6.5).** The old firmware's own comment says SH-RLY **pre-charges the DC-link capacitors** ("...no gap for the caps to bleed back
> down through R92 before the main relay closes") and W-RLY is the **main relay** that stays on. R92 is the DC-link bleeder resistor across `V+` and
> `PE` (power schematic). The bypass-relay reading below was derived from the schematic alone and **conflicts** with that description: treat it as an
> UNVERIFIED hypothesis. The old sequence is proven on this hardware, so `SLAVE_RELAYS_MODE 2` reproduces it exactly; `SLAVE_RELAYS_MODE 1` (rules
> R1-R5) stays unverified until the contact measurement in section 6 is done and explains how SH / W really act. Everything about stages 2 and 3
> in this document is subject to that.

# SH-RLY / W-RLY — relay logic review (derived from the hardware, not from the old firmware)

## 1. What the schematic shows (power board, read at 600 dpi)
| Fact | Evidence | Confidence |
|---|---|---|
| **SH-RLY contact and W-RLY contact each connect the nets `PRI-PHASE` and `PRI-NTRL`** | both contacts' two used terminals go to those two nets | high |
| `PRI-PHASE` = inverter leg-1 mid-point -> current-sense toroid `H2` (CT2500) -> fuse `F1` -> blade terminal `PRI1` (transformer primary). `PRI-NTRL` = leg-2 mid-point -> the other primary terminal | the leg / toroid / fuse / blade drawing | high |
| So **both relays are BYPASS (shunt) relays across the series transformer's primary**: a closed contact shorts the primary and the series winding becomes transparent | follows from the two lines above | high |
| The inverter's two LOW sides on (zero duty) short the primary too | leg-1 / leg-2 low IGBTs on the negative rail | high |
| `CONT1` / `CONT2` are NOT relays: `CONT1` is a zero-cross opto-triac (FODM3083) driving a BTA204S-800C triac on `PHASE-OUT` (output switch) | triac stage drawing | high |
| `RL3` is an alarm dry-contact output (connector CN5, two 320 V varistors), unrelated | RL3 drawing | high |
| Coil drive: MOSFET (Q8 for RL1 = SH, Q3 for RL2 = W), flyback diode across the coil, gate from the MCU pin | relay driver drawing | medium-high |
| **Which coil state closes each contact (normally-open / normally-closed): NOT readable** — the contacts are drawn as three-terminal change-over symbols with one terminal unused | symbol | **unknown -> MEASURE** |
| Relay part numbers and contact travel / bounce times | not on the schematic | unknown -> measure / data sheet |

## 2. Rules the logic must obey (and why)
The series transformer's secondary is IN SERIES with the load path. Therefore:
* **R1** No inverter HIGH side may be driven while any shunt contact is closed. A closed contact joins the two leg mid-points; high side on one leg + low side on the other = the DC link across the contact.
* **R2** A relay coil changes only while the inverter duty is zero (no contact breaking or making under drive).
* **R3** The primary is never left open AND uncontrolled. It must be shorted either by a relay contact or by the inverter's low sides. (An open primary with load current in the secondary = an open-circuited series winding = a dangerous voltage.)
* **R4** The shunts are opened only with the inverter already enabled at zero duty (so the primary stays shorted by the IGBTs while the contacts move).
* **R5** Stopping reverses it: duty to zero -> shunts closed -> THEN the inverter is disabled.
Faults: anything that is not an overcurrent follows the same unwinding (converter off, duty zero, shunts closed, then inverter off). Break / FLT / a relay fault switch everything off at once (hardware already has) and close the shunts immediately.

## 3. The old logic against these rules
| Old behaviour | Finding |
|---|---|
| Relays opened FIRST (SH aimed at a zero-cross, hold 1 s, W takes over, settle 1 s), converter next, inverter 3 s after that | The primary sits open and uncontrolled for the whole converter-only phase (> 3 s). Harmless only while no load is connected - and the slave cannot know that. **Violates R3/R4.** |
| SH close aimed at a mains zero-cross (6 ms pull-in ESTIMATE) | No electrical reason on this board: a shunt closing across a primary that nothing drives has no inrush. Unverified inheritance. **Removed.** |
| 1 s hold + 1 s settle | Arbitrary. Replaced by a settle time that must exceed the contact travel (default 100 ms, build refuses < 20 ms). |
| Fault: outputs off, then both coils released | Correct only if releasing a coil CLOSES the shunt. For a normally-open contact, releasing the coil OPENS it. **Replaced by an explicit SHORTED coil pattern per measured contact type.** |
| Normal stop: W opens 1 s after outputs off | Leaves the primary open for 1 s with nothing driving it. **Replaced by R5.** |
| "SH off / W on in operation" | If both contacts are shunts, this implies SH = normally-open and W = normally-closed — an INFERENCE, not a fact. It is the first thing to check. |

## 4. What cannot be verified in firmware
* **There is no sensing of the real contact state.** "Settled" means the settle time has passed. A welded or stuck contact is not detected by the firmware; the protection is the overcurrent break (CT2500 -> comparators -> BKIN).
* After an overcurrent-class fault the primary is open for the contact travel time (modelled as 6-12 ms; measured in the tests: 7.3 ms). The board has a 188 V TVS (SMCJ188A); I have NOT traced whether it clamps the primary.

## 5. Evidence from the host tests (physical relay model: coils, contacts with 6-12 ms travel, checks R1-R4 on every call of every scenario)
* The whole suite passes for **every combination of contact types** at stages 2 and 3 (SH, W in {normally-open, normally-closed, not-a-shunt}; "neither is a shunt" is a build error).
* Dedicated tests: module unit test, start order, normal stop, stop while the shunts are opening, soft fault unwinding, hard fault, fault while unwinding, and a 90-point injection sweep of stop / soft fault / hard fault through the start-up and RUN windows.
* Deliberate breaks of each rule are caught (R1, R2, R3, R4, inverted coil polarity, module never closing, settle shorter than contact travel).

## 6. WHAT YOU MUST DO — measure the contacts (5 minutes, no mains)
Power the power board's +12 V only (nothing else). Meter in continuity / resistance mode between the nets **PRI-PHASE** and **PRI-NTRL** (e.g. the PRI1 blade and the other primary blade, or the contact pads). To energise a coil, briefly connect +3.3 V to that relay's pin on the interface connector (**RL1 = SH-RLY**, **RL2 = W-RLY**); remove it to release.
| coil state | SH-RLY alone | W-RLY alone |
|---|---|---|
| released (both coils off) | | |
| SH energised, W released | | |
| W energised, SH released | | |
| both energised | | |
(write "short" or "open"; measure each relay separately if the two contacts are separately accessible — if only the parallel combination is visible, the four rows still determine both.)
Then in `slave_config.h` set, for each relay: `RELAY_SHORTS_WHEN_ENERGISED` (short only when its coil is on), `RELAY_SHORTS_WHEN_RELEASED` (short when its coil is off), or `RELAY_NOT_A_SHUNT` (never connects the two nets).
**If neither relay ever connects PRI-PHASE and PRI-NTRL, STOP: my reading of the schematic is wrong. Do not run the inverter.**
Optionally measure the contact travel (scope on a continuity source across the contact while switching the coil): set `SLAVE_RELAY_SETTLE_MS` to at least twice the longest time seen.
