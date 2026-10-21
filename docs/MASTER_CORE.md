# Master core (v0.9.1, roadmap step 7a)

Four pure-C modules in `master/Core` (no HAL, no globals but their own state; the flash is reached through `MFlash_t`), tested on the PC. **They are not yet called by `main.c`**: the display, keys, link and
hardware glue are steps 7b to 7d. Nothing in your master's behaviour changes with this release.

| module | what it is |
|---|---|
| `m_store` | two-slot record store, power-fail safe: erase the other slot, program the record, program the COMMIT word last; CRC-16; highest sequence wins |
| `m_settings` | the Set-Mode settings, defaults, every range and cross rule ("Invalid Value"), including the slave's input window and output guard (D7, D26); factory reset = `MS_Defaults` (calibration, log and tuning profile are separate records) |
| `m_log` | the fault log 00..99: 16-byte records, two pages as a circular buffer, power-cut safe, at least 128 entries always remain, entry 00 is the newest |
| `m_faults` | the fault and restart engine: units (Single / Industrial = one unit / Domestic = three), every fault group, restart rules, hysteresis, retries, latches, buzzer, contactors, run level and clear pulses for the slaves, what to log, what to show |

## Rules of the engine (all from the approved spec and decisions)
Voltage faults after the Volt Cut Delay of CONTINUOUS violation, clear 5 V back inside, Auto restart after the Restart Time or Manual START. Overload and Heat: START only (Heat also needs the slave's flag gone).
Inrush (slave codes 1, 2, or the master CT at full scale) and Power stage (7, 12, 14): trip at once, 3 retries 30 s apart, then latched. Mains group (3, 4, 5, 8, 10, 11, 13, 15, 16): trip at once, restart like the voltage faults.
Code 9 shows as I/P LV or HV, code 6 as O/P HV. Flip (17): latched, buzzer in every mode. Re-tune (18, 19): latched until `MF_RetuneDone`. Link lost: trip at once, Auto restart after the Restart Time.
The contactor closes only after IN_BAND has held for the Restart Time (D27). Set-Mode open = output off (D10). Industrial: one unit, one contactor, any phase trips all, phase-sequence check.

## Choices I made where the spec is silent: please confirm or change
1. The inrush / power-stage retry count is forgotten after the unit has run 5 minutes without tripping (`MF_FORGIVE_MS`). Without this, three separate transients in a month would latch the unit.
2. An unknown slave fault code is treated as a Power-stage fault (3 retries, then latch): unknown means serious.
3. Display priority for the new groups: Re-tune just below Power stage; Phase sequence level with Mains; Link lost below Mains and above Output V.
4. STOP while a fault is waiting for an automatic restart cancels the restart and clears the fault display (the unit goes OFF, START runs it). STOP on a LATCHED fault only silences the buzzer.
5. Every start attempt pulses the slave's clear counter (harmless when the slave is not latched). A slave's old fault code is ignored for 1 s after the pulse, so a slow slave does not cause a second trip; a slave that does not clear within 1 s trips again and is not hidden.
6. Output voltage and overload are judged only while the contactor is closed (the output is live); a slave ramping up is never an output fault.
7. After a successful re-tune the unit is OFF and START runs it (even in Auto).
8. Log time is running MINUTES (a 32-bit count), not hours: there is no RTC (D28).

## Tests (the runner runs them)
`test_master_store.c` (83 checks): power cut at EVERY point of a save, of a first save, of a log append (including the page erase), 448 single-bit corruptions, failed writes without a restart, 100,000 settings round trips.
`test_master_faults.c` (111 checks): about 25 scenarios against a slave model that latches faults until the clear counter changes, the exact display priority, and 1.2 million random ticks with four invariants
(a closed contactor always has its slaves running; nothing energised in Set-Mode; a latch never lets go; a fault is logged only when the unit was energised). Twenty-two deliberate breakages are caught.

## Next
7b Set-Mode / Run-Mode screens and key handling (rendered to a 20x4 buffer and tested); 7c the hardware glue (pins, LCD, keys, contactors, buzzer, CT, link to the slaves); 7d integration against a simulated slave.
