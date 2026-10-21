# Static Stabilizer — Master + Slave Firmware Spec (v0.1, for review)

This consolidates everything decided so far. Nothing has been coded yet — review and
correct this first. Signal *names* below are read reliably from the schematics; exact
*pin numbers* for the slave piggyback could not be reliably reconstructed from the PDF
text extraction (the master's pinout table was a clean numbered list; the slave's
schematic connections are graphical and didn't extract cleanly) — please confirm those
against the actual schematic/PCB or your CubeMX project before I lock in peripheral
init code.

---

## 1. System overview

- **Master** (`DISP SS3S LCD4 M1 V1.1`, STM32G070RBT6): owns all protection decisions
  (LV, HV, OL, phase sequence), owns the load contactors/triacs, owns the LCD + keypad
  UI, talks to 1-3 slaves over isolated UART.
- **Slave** (power board + `CRISP_PIGGYBACK_CIRCUIT` piggyback, STM32G0B1RET6): pure
  voltage-regulation engine. Generates converter (buck/boost) + inverter PWM to hold
  output at the master's commanded setpoint, across the transformer's compensation
  range, regardless of load or input sag/rise. Has its own local surge/inrush
  protection (BKIN hardware comparator) independent of the master's protections.
- One slave per phase. Three working modes on the master select how many slaves and
  contactors are in play (see §4).

---

## 2. Slave firmware

### 2.1 Core regulation objective
- Continuously regulate output to the master-commanded setpoint via closed-loop PWM
  control of the converter + inverter stages.
- Must hold regulation across the full transformer compensation envelope. Default
  outer bound (per your instruction): **input 140-300V**, since actual transformer
  range varies by customer/unit — this is the *outer* firmware-supported range; the
  master's LV/HV cutoffs (set per installation) live inside this envelope.
- Same control logic for EI-core and toroidal-core transformers — no core-type
  firmware branch. (Toroidal-specific inrush/mechanical-noise issues, per your report,
  are a hardware-level concern — inrush limiting or zero-cross-synchronized switching
  on the affected board — not something this control loop should try to compensate
  for.)

### 2.2 BKIN surge/inrush handling
- BKIN hardware event → **PWM stops immediately** (hard disable, same mechanism
  regardless of cause).
- **Escalating auto-restart** (confirmed): 1st trip → retry after 10s. If BKIN trips
  again → 20s. Again → 30s. Linear +10s per occurrence, **capped at 60s**, and
  **latching after the 6th occurrence** requiring a manual reset rather than climbing
  forever.
- **Escalation counter reset** (confirmed): resets back to 10s after **5 minutes** of
  sustained fault-free running since the last trip, so a genuinely rare/intermittent
  event doesn't escalate to absurd wait times over a long operating life.
- **Mandatory zero-cross re-sync on every restart** (not just BKIN — *any* PWM
  restart, from any cause): never resume PWM assuming the previous zero-crossing
  reference is still valid. Always re-acquire a fresh zero-crossing and re-synchronize
  before ramping duty back up. This is layer 2 of the phase-change/IGBT-protection
  design (§4.4) and applies universally as good practice regardless of *why* PWM
  stopped.

### 2.3 Slave-side I/O (signal names, from `CRISP_PIGGYBACK_CIRCUIT`)
- `PWM1H/L, PWM2H/L, PWM3H/L, PWM4H/L` — 4 half-bridge PWM pairs to the power stage.
- `RL1, RL2` — relay drives (precharge/main, by convention — confirm roles).
- `CONT_1` — in use; `CONT2` — **unused**, per your note.
- `ZCD` — dedicated zero-cross detect input.
- `FLT` — fault input from the power board (BKIN-related).
- `NTC`, `FAN_PWM`, `BUZZ`. (`ON/OFF` appears in the schematic net list but is **not**
  used as a slave GPIO — confirmed: run/stop is commanded by the master over UART,
  not a local input.)
- `AC_IN`, `AC_OUT`, `I_OUT` — analog (ADC).
- `UART2` (TX/RX/EN) — link to master.
- `LCD_UART1` (TX/RX/EN) — **unused**, no local LCD, per your note. Left unconfigured
  or repurposed later if needed.

## 2.4 Slave pin map (STM32G0B1RET6, "PB LP V1.0")

Confirmed directly by you so far:

| Pin | Signal | Note |
|---|---|---|
| PF2 | NRST | |
| PF0, PF1 | 8MHz crystal | |
| PC14, PC15 | 32kHz crystal | **not used** |
| PC11 | LED | **not used** |
| PC13 | microswitch | **not used** |
| PC0 | Contactor 1 | **not used** |
| PC1 | Contactor 2 | **not used** |
| PA0 | MCU_AC_IN | ADC, input voltage measurement |
| PA1 | MCU_AC_OUT | ADC, output voltage measurement |
| PA2 | current measurement | **not used** |
| PA4 | LCD_UART1_EN | **not used** |
| PA5 | NTC | thermal protection |
| PA7 | PWM2L | |
| PC4 | PWM3L | |
| PC5 | PWM4H | |
| PB0 | ZCD | |
| PB10 | UART2_TX | **not used** |
| PB11 | — | **not used** |
| PB13 | PWM1L | |
| PB14 | PWM1H | |
| PA13, PA14 | SWD (program/debug — you call this "fusing") | fixed by silicon |
| PA15 | — | not used |
| PA8 | PWM2H | |
| PB3 | PWM3H | |
| PB5 | PWM4L | |
| PC8 | relay W (main) | confirmed against old firmware |
| PC9 | relay SH (precharge) | confirmed against old firmware |
| PD4 | buzzer | |
| PB9 | fan | |
| PD0, PD1, PD3 | — | not used. `ON/OFF` is **not** a slave GPIO at all — confirmed: run/stop is commanded by the master (from its own keypad) over the UART link, not a local input pin on the slave. |
| PB12, PA9, PD2 | BKIN break-inputs for TIM1/TIM15 (inrush) | 3 genuinely separate break-input pins — **which one is which** (TIM1_BKIN, TIM1_BKIN2, TIM15_BKIN)? Need this to wire the right alternate function to each in CubeMX. |
| **PB6** | **Master-link UART TX** | Schematic calls this net "LCD_UART1_TX," but per your answer #4 this is actually the physical link to the master, not a local LCD. |
| **PB7** | **Master-link UART RX** | Same — schematic net "LCD_UART1_RX," actually the master link. |
| PB10 | — | not used (schematic's `UART2_TX` — genuinely spare, not the master link) |
| PB11 | — | not used (presumed `UART2_RX`, spare — confirm) |

One thing to flag: earlier you said CONT_1 was in use and only CONT2 was unused — this
message marks *both* PC0 (Contactor 1) and PC1 (Contactor 2) as "not used." Confirm
whether that's an update (neither contactor output is actually used by this slave —
consistent with contactors living on the master, per §1) or just shorthand for "not
wired into firmware yet."

**Resolved**: `RL1`/`RL2` are just the schematic's generic names for the SH/W relays
already mapped above (PC9/PC8) — no separate pins needed. `RLY3` — confirmed not used.
`CONT_2`, `GPIO_1-4`, `UART2_EN`, `MCU_I_OUT` — confirmed reserved for future
expansion, no pin assignment needed for this build. `LCD_UART1_EN` (PA4) — confirmed
not configured; the master-link UART uses simple RX/TX only, no enable/direction pin.
`UART2` — confirmed entirely unused (both PB10 TX and PB11 RX).

**Slave pin map (§2.4) is now complete** for this build's scope.

### 2.5 Slave PWM/timer architecture (verified against STM32G0B1 datasheet + old code)

Confirmed via direct datasheet alternate-function tables and cross-checked against the
team's clarification (inverter has BKIN, converter has no timer/no BKIN) and the old
firmware's actual behavior:

| Pair | Pins | Role | Mechanism |
|---|---|---|---|
| PWM1 H/L | PB14 / PB13 | Inverter | `TIM15_CH1`/`CH1N` — real hardware complementary PWM, with dead-time insertion and `TIM15_BKIN` |
| PWM2 H/L | PA8 / PA7 | Inverter | `TIM1_CH1`/`CH1N` — real hardware complementary PWM, with dead-time insertion and `TIM1_BKIN` |
| PWM3 H/L | PB3 / PC4 | Converter | Plain GPIO push-pull output, no timer alternate function used, no BKIN |
| PWM4 H/L | PC5 / PB5 | Converter | Plain GPIO push-pull output, no timer alternate function used, no BKIN |

The converter pair pins toggle once per AC half-cycle (at the zero-crossing) to select
current path/polarity — not a switching-frequency PWM signal — so no timer AF or
dead-time hardware is needed there, matching the team's description exactly.

The 3 BKIN pins (PB12, PA9, PD2) apply **only to the inverter side** (TIM1_BKIN =
PB12, TIM15_BKIN = PA9, and PD2 = `FLT`, a separate non-silicon-break fault input read
in software) — the converter GPIO pair has no hardware break protection, consistent
with "no timer involved for converter, no break input."


---

## 3. Master firmware — Set Mode / Run Mode

### 3.0 Master pin map (STM32G070RBT6, LQFP64, "DISP SS3S LCD4 M1 V1.1")

Confirmed directly by you:

| Pin | Signal | Note |
|---|---|---|
| PA10 | CONT R | used in both modes (see below) |
| PA11 | CONT Y | **domestic mode only** — unused in industrial mode, see below |
| PA12 | CONT B | **domestic mode only** — unused in industrial mode, see below |
| PD0 | LCD D4 | 4-bit mode |
| PD1 | LCD D5 | 4-bit mode |
| PD2 | LCD D6 | 4-bit mode |
| PD3 | LCD D7 | 4-bit mode |
| PB4 | LCD RS | |
| PB5 | LCD EN | |
| PC10 | TX Y | three-phase comms |
| PC11 | RX Y | three-phase comms |
| PC4 | TX R | corrects earlier wrong guess of PA6 |
| PC5 | RX R | corrects earlier wrong guess of PA7 |
| PA0 | I-R | confirmed |
| PA1 | I-Y | confirmed |
| PA2 | I-B | confirmed |
| PA3 | F-R | confirmed |
| PA4 | F-Y | confirmed |
| PA5 | F-B | confirmed |
| PB10 | K1 (MENU) | confirmed |
| PB11 | K2 (START) | confirmed |
| PB12 | K3 (STOP) | confirmed |
| PB13 | K4 (LOG) | confirmed |
| PB14 | BUZZER | confirmed — corrects the earlier PB11 mix-up |

**Domestic vs. industrial mode — resolved, maps onto §4 exactly:**
- **Industrial mode** = §4.2 (three-phase, single contactor): only `CONT R` (PA10) is
  used, switching all three phases together. On any fault, the single contactor opens
  and **all three slaves** trip and restart together, per the fault's recovery rule.
- **Domestic mode** = §4.3 (three-phase, three contactor): `CONT R`/`CONT Y`/`CONT B`
  (PA10/PA11/PA12) are independent. On a fault, **only the affected phase's**
  contactor and slave go off and restart per the fault's recovery rule — the other two
  phases keep running unaffected.
- §4 will be updated to use "industrial"/"domestic" as the actual names for these two
  modes.

**Resolved**: the keypad is on PB10-PB13 (above), not PB0/PB2 — so PB0 = RX B and
PB2 = TX B (three-phase comms) stand as correctly confirmed; my original guess placing
the keypad there was simply wrong.

**Resolved**: PB11 = K2, BUZZER = PB14. Master pin map is now complete.


### 3.1 Set Mode parameters
- Set voltage
- HV cutoff, LV cutoff
- Overload current
- Voltage calibration (input + output), current calibration
- Working mode selection (§4)
- Auto/Manual restart mode
- Auto-restart time (delay before LV/HV auto-reconnect attempt, once condition clears)
- **Phase-sequence manual-restart (Y/N)** — new setting (see §3.3): when Y, a phase-
  sequence fault always requires manual restart (like OL) regardless of the global
  Auto/Manual setting; when N, it follows the global Auto/Manual setting the same way
  LV/HV do.

### 3.2 Run Mode
- Live display of output V / input V / load current / frequency, scrolling at 5s
  intervals — confirmed, same pattern as the servo-stabilizer project.

### 3.3 Protection trip logic (LV, HV, OL, phase sequence)
- **Confirm window: 2-3 mains cycles** before declaring a trip (per your answer) — not
  a single sample, not the old codebase's multi-second delay.
- Once confirmed: **trip immediately** — contactor opens, slave commanded to stop PWM,
  no further delay.
- **Recovery**:
  - LV/HV: **auto-recover if Auto mode selected**, after the configured auto-restart
    time has elapsed post-recovery.
  - OL: **always manual restart**, regardless of Auto/Manual mode setting.
  - Phase sequence: **manual restart, like OL — but only when the new "Phase-sequence
    manual-restart" setting (§3.1) is Y.** When that setting is N, phase-sequence
    faults instead follow the global Auto/Manual setting the same way LV/HV do (auto-
    recover after the auto-restart time, if Auto mode is selected).

---

## 4. Working modes

### 4.1 Single phase
- One slave, one UART link (`UART1`), one contactor. No phase-sequence check.

### 4.2 Three phase, single contactor ("industrial mode")
- Three slaves (R/Y/B), **only `CONT R` (PA10) is used**, switching all three phases
  together — `CONT Y`/`CONT B` unused in this mode.
- **Phase-sequence check required** — a wrong rotation direction matters here because
  all three phases connect/disconnect as one unit.
- **Fault scope**: on any fault (any phase), the single contactor opens and **all
  three slaves trip and restart together**, per that fault type's recovery rule
  (§3.3) — the whole three-phase system is one unit.

### 4.3 Three phase, three contactor ("domestic mode")
- Three slaves (R/Y/B), **three independent contactors** (`CONT R`/`CONT Y`/`CONT B`
  = PA10/PA11/PA12), one per phase.
- **No phase-sequence check** — each phase is switched independently, so rotation
  order doesn't create the same hazard as §4.2.
- **Fault scope**: on a fault, **only the affected phase's** contactor and slave go
  off and restart per that fault type's recovery rule (§3.3) — the other two phases
  are unaffected and keep running.

### 4.4 Phase-change → IGBT protection (proposal, awaiting confirmation)
Two layers:
1. **Continuous sequence monitoring during Run Mode**, not just at startup — keep
   validating R/Y/B rotation order/spacing live via the phase-sequence comparator
   circuit. If the detected sequence changes mid-run (wires swapped, generator/ATS
   switch-over with different rotation), trip immediately and open the contactor.
2. **Mandatory zero-cross re-sync on every restart** (slave-side, §2.2) — the part that
   actually protects the IGBTs against a stale-sync condition after any phase change,
   regardless of whether the sequence monitor caught it in advance.

---

## 5. Master-slave communication protocol (new design)

- **UART assignment**: Single-phase mode → `UART1` only. Three-phase modes →
  `UART1` = R, `UART2` = Y, `UART3` = B. (This is the **master's** three UART
  peripherals, one per slave. On the **slave** side, each unit only has the one link,
  physically on PB6/PB7 — schematic-labeled "LCD_UART1" but confirmed as the actual
  master link, §2.4 — regardless of which of the master's three UARTs it's wired to.)
- Designed fresh (not reusing the old protocol), optimized for **fast response** per
  your instruction. Proposed shape (to detail once confirmed):
  - Periodic **status frame**, slave → master: output V, input V, load current,
    PWM-running state, local fault flags. Sent at a fast fixed interval.
  - **Command frame**, master → slave: setpoint voltage, run/stop, and an
    **immediate stop command** the master can issue the instant it declares any trip
    (LV/HV/OL/phase-sequence) — this is the fast path that ties the master's
    protection decisions to the slave actually stopping PWM without waiting on the
    slave's own independent checks.
  - Lightweight framing (start byte + fixed fields + checksum), no multi-step
    handshake states like the old codebase's blocking `while(!flag)` waits — those
    caused real hangs before and won't be repeated here.

---

## 6. Calibration

- **Two-point calibration** (confirmed) per quantity (input V, output V, current):
  operator takes a low reference reading and a high reference reading from a
  standard/reference meter, entering each true value via the keypad while the system
  records its own raw reading at that same point. Firmware fits both **gain and
  offset** from the two (raw, true) pairs, rather than assuming zero offset error —
  more accurate across the full range than single-point.
- Flow per quantity: operator sets/reads a low condition → enters true meter value →
  system logs its raw reading → operator sets/reads a high condition → enters true
  meter value → system logs its raw reading → firmware solves the linear fit and
  stores gain+offset to non-volatile memory.

---

## 7. Open items needing your confirmation before I start coding

1. Exact GPIO pin numbers for the slave piggyback (STM32G0B1RET6) — please
   confirm/share a pin table or the CubeMX `.ioc`, since I couldn't reconstruct them
   reliably from the schematic text. G070 and G0B1 share the same LQFP64 pin
   template, but that alone doesn't tell me which net lands on which pin for this
   specific board's routing.

Everything else in this document is confirmed. Once the pin mapping above is settled,
next step is pin maps + peripheral init (skeleton, no control logic yet) for both
boards, for you to confirm before the state machines go in on top.
