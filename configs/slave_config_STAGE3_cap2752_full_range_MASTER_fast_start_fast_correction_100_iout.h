/**
  ******************************************************************************
  * @file    slave_config.h
  * @brief   Build-time switches for the slave firmware. Every value can be
  *          overridden from the compiler command line (-DNAME=value).
  *
  * THE POWER STAGE IS OFF BY DEFAULT.  With the defaults below the firmware
  * behaves exactly like the bring-up build you already flashed: measurement,
  * link to the master, beep report - and NOTHING energises the IGBTs.
  *
  * Bring the power stage up in order, one stage at a time, verifying each one
  * on the bench BEFORE moving to the next (the procedure is in the hand-over
  * notes). Moving to a higher stage is a deliberate edit of this file.
  *
  *   SLAVE_POWER_STAGE
  *     0  OFF. No power-stage code runs; converter and inverter pins untouched.
  *     1  CONVERTER ONLY. Zero-cross synchronised commutation of the converter
  *        diagonals, inverter never enabled. Verifies sync, polarity and dead
  *        time with the DC link current-limited.
  *     2  CONVERTER + INVERTER, OPEN LOOP. Fixed small duty
  *        (CTRL_STAGE2_DUTY_COUNTS), no regulator. Verifies leg selection and
  *        the injection direction.
  *     3  CLOSED LOOP. Regulates the output to CTRL_TARGET_V.
  *
  *   SLAVE_BENCH_AUTORUN
  *     0  The power stage waits for SlaveCtrl_SetRunRequest(1) (the master's
  *        run command - not implemented yet, so it never starts by itself).
  *     1  Bench only: request "run" automatically once conditions are met.
  ******************************************************************************
  */
/* READY-MADE (v0.9.0): STAGE 3 CLOSED LOOP, FULL RANGE, RUN BY THE MASTER, FAST START: cap 2752 ticks (the duty ceiling) = about 27 V at 200 V and 38 V at 250 V; reaches 230 V from about 202 to 270 V (upper figure extrapolated). Input 205-265 V. NO LOAD, series lamp in. Copy over Core/Inc/slave_config.h */
#ifndef SLAVE_CONFIG_H
#define SLAVE_CONFIG_H

#ifndef SLAVE_POWER_STAGE
#define SLAVE_POWER_STAGE      3
#endif

/* ---- MASTER CONTROL (v0.9.0, roadmap step 3) -------------------------------------------------------------------------------------------------------
 * THIS FILE IS FOR USE WITH THE MASTER (a master built from the same release, 500 kbaud link, its USART RX wired to the slave's TX and the reverse).
 *   - The slave does NOT start by itself: the master's $M frames say run, the set voltage and the clear counter, every 100 ms.
 *   - EVERY fault latches until the master changes the clear counter. There are no automatic retries.
 *   - No valid $M for 1 s: the slave stops (and flags LINK_LOST).
 *   - The slave sends $E, $H and $R frames as well as $S. A master older than v0.8.5 counts them as bad frames on its display; nothing else.
 * Without a master it stays OFF with LINK_LOST. To bench it alone, use slave_config_STAGE3_cap2752_full_range.h instead. */
#ifndef SLAVE_LINK_V2
#define SLAVE_LINK_V2          1
#endif
#ifndef SLAVE_MASTER_CONTROL
#define SLAVE_MASTER_CONTROL   1
#endif
/* A rare zero-cross glitch (F16, F04) while RUNNING skips one half-cycle instead of latching the unit (and opening the load contactor): at most 2 in 2 s, the third is the fault as before. See slave_features.h. */
#ifndef SLAVE_GLITCH_SKIP
#define SLAVE_GLITCH_SKIP      1
#endif
/* The TRANSFORMER-CONNECTION (FLIP) check at every start: if the output moves AGAINST the commanded direction the slave stops with fault 17, latched (the master shows Transformer Flip, buzzer in every mode). See slave_features.h. */
#ifndef SLAVE_TF_RUN_CHECK
#define SLAVE_TF_RUN_CHECK     1
#endif
/* Voltage calibration over the link ($K CALV / CALO / CALR from the master's Voltage Calibration pages): only while the unit is stopped, +-10 %, stored in two flash pages (0x0803F000 / 0x0803F800: the program must end below them). */
#ifndef SLAVE_VOLTAGE_CAL
#define SLAVE_VOLTAGE_CAL      1
#endif
/* ---- FAST SOFT START (the values of slave_config_STAGE3_cap2752_full_range_fast_start.h): the output reaches the set voltage about 1.4 s sooner from 205 V, and much sooner from a low input ---- */
/* the soft-start ramp of the setpoint: 10 V/s for 0.5 s, then 30 V/s, then 10 V/s for the last 3 V (the standard values are 3 V/s for 2 s, 10 V/s, 5 V/s). Measured in simulation on the real regulator: no overshoot.
 * The 3 s the converter runs alone before the inverter starts is a separate delay and is not changed. A faster ramp is a faster rise of the load voltage. */
#define CTRL_SOFTSTART_SLOW_SLOPE_VPS   10.0f
#define CTRL_SOFTSTART_FAST_SLOPE_VPS   30.0f
#define CTRL_SOFTSTART_FINAL_SLOPE_VPS  10.0f
#define CTRL_SOFTSTART_SLOW_TIME_MS     500UL

#ifndef SLAVE_BENCH_AUTORUN
#define SLAVE_BENCH_AUTORUN    0
#endif

/* Relays SH-RLY (PC9) and W-RLY (PC8).
 *   SLAVE_RELAYS_MODE 2 : LEGACY. The OLD firmware's relay sequence, which is proven on this hardware: SH-RLY is closed aimed at a
 *                         mains zero-cross and held for 1 s (pre-charge), then SH-RLY off and W-RLY on in the same instant, W-RLY stays
 *                         on, and the converter starts only after W-RLY has been on for 1 s. A fault releases BOTH coils at once.
 *                         Converter-only (stage 1) until it has been verified for the inverter. Needs no contact settings.
 *   SLAVE_RELAYS_MODE 1 : the firmware drives them by the bypass-relay rules R1-R5 (slave_relay.h): needs the contact types MEASURED.
 *   SLAVE_RELAYS_MODE 0 : the firmware NEVER touches them; you set them by hand. The firmware cannot
 *                         check the relay state, so for stages 2 and 3 (inverter) you must confirm
 *                         it yourself with SLAVE_RELAYS_CONFIRMED 1. */
#ifndef SLAVE_RELAYS_MODE
#define SLAVE_RELAYS_MODE      2
#endif

/* ---- this file (v0.8.2): ---- */
#define SLAVE_CONV_FIRST_AT_PEAK 1       /* the converter's first diagonal at the line peak (bench: starts reliably) */
/* The MASTER owns the voltage limits: the slave's own outer envelope is NOT narrowed here. It keeps its built-in values (input trips below 80 V and above 300 V, it STARTS only from 90 V to 290 V;
 * output trips above 280 V), and announces them in its $H frame, so Set-Mode refuses limits outside them. The master's I/P LV, I/P HV, O/P LV and O/P HV settings, with the Volt Cut Delay, decide inside that envelope
 * (and a gross output over-voltage, 15 V above O/P HV, is cut within 0.2 s). The other full_range files narrow the slave to 195 / 275 / 265 V to mimic the old firmware: that is NOT wanted when the master is in control. */
#define CTRL_STAGE3_MAX_EFF_TICKS 2752U   /* the cap: 2752 effective ticks, the duty ceiling: about 27 V at 200 V and 38 V at 250 V of correction; it reaches 230 V from about 202 V to 270 V (upper figure extrapolated) */
/* (not active) the OLD firmware only ran the converter from about 180 V (its LV cut-out, +5 V hysteresis). To compare against it you would replace the 80 V line below by this, and remove the
 * comment marks below: the converter then starts at 190 V and trips below 180 V. (Defaults: trip below 140 V, start from 150 V.) */
/* #define CTRL_VIN_MIN_V 180.0f */

/* LOW INPUT VOLTAGE (the line below is ACTIVE here): the converter starts from 90 V and trips below 80 V. Firmware margins:
 * the polarity sign detection has 3.2x margin at 90 V (1.6x at 45 V); HARDWARE limits are NOT confirmed: the ZCD IC only worked above about 70 V on the bench,
 * and the power board's own 12 V / 3.3 V rails and the relay coils at 90 V mains are untested. The bench-proven start rule (positive half, low sides on)
 * was found at 197-245 V and is NOT verified at 90 V. Converter-only: regulating from 90 V (the inverter) is a separate question. See docs/POWER_STAGE_BRINGUP.md. */
/* ACTIVE in the master-control files since v0.9.17: the unit is specified to work from 90 V, so the slave's input trip is 80 V and it starts from 90 V (+10 V hysteresis). It announces that in $H. */
#define CTRL_VIN_MIN_V 80.0f

#ifndef SLAVE_RELAYS_CONFIRMED
#define SLAVE_RELAYS_CONFIRMED 0
#endif

/* What each relay contact DOES to the transformer primary. SH-RLY and W-RLY both have a contact between the
 * nets PRI-PHASE and PRI-NTRL (the two inverter-leg mid-points = the primary of the series transformer): they are
 * BYPASS relays - a closed contact SHORTS the primary and makes the series winding transparent.
 * Which COIL STATE closes the contact is NOT readable from the schematic: MEASURE IT (continuity between
 * PRI-PHASE and PRI-NTRL, power off, coil off / coil on) and set the two values below.
 *   RELAY_SHORTS_WHEN_ENERGISED : contact closes when the coil is ON   (normally-open  contact)
 *   RELAY_SHORTS_WHEN_RELEASED  : contact is closed when the coil is OFF (normally-closed contact)
 *   RELAY_NOT_A_SHUNT           : the contact does not connect PRI-PHASE and PRI-NTRL (relay ignored)
 * The inverter stages (2, 3) refuse to build until both are set. */
/* SLAVE_CONV_SWAP: swaps WHICH converter diagonal is energised for each mains half-cycle (3H+4L <-> 3L+4H).
 * Leave it 0. Set it to 1 ONLY if the passive mapping check on the master (row 3, "M<agree>/<disagree>") shows that this firmware
 * chooses the OPPOSITE diagonal to the old firmware in every half-cycle (e.g. M0/90) AND a scope check at reduced voltage
 * (current-limited) confirms it. A wrong diagonal puts the DC link across the line through forward-conducting IGBTs. */
#ifndef SLAVE_CONV_SWAP
#define SLAVE_CONV_SWAP              0
#endif

/* SLAVE_LEGACY_LOWSIDES (stage 1): the OLD firmware's state of the inverter's LOW-SIDE IGBTs (1L, 2L) while the converter starts.
 * Found by reading the old code: it starts the inverter outputs at boot (and after every fault re-arm) with ZERO duty, which turns the two
 * low sides ON, i.e. the transformer PRIMARY IS SHORTED BY THE IGBTs. Its FIRST converter diagonal runs with the lows still on; only from the
 * SECOND diagonal on does it disable the complementary outputs. This firmware kept them OFF from v0.2.0 on (an "unexpected" LED state that I
 * wrongly treated as a defect). With the relays in the old operating state (W-RLY on = the PRI nets OPEN, measured), that leaves the primary
 * open and uncontrolled at the converter's first turn-on: every bench start tripped 24 us after the first diagonal.
 *   1 : enable the inverter outputs at zero duty (1L / 2L ON, highs OFF) when the start sequence begins; they go OFF at diagonal number
 *       SLAVE_LOWSIDES_OFF_AFTER (the old firmware: 2). Default with SLAVE_RELAYS_MODE 2 at stage 1.
 *   0 : inverter outputs stay off (v0.2.0 .. v0.6.6).
 * 1L and 2L LEDs ON during the start is therefore NORMAL. Output load disconnected, series lamp in the line. */
#ifndef SLAVE_LEGACY_LOWSIDES
#if (SLAVE_RELAYS_MODE == 2) && (SLAVE_POWER_STAGE >= 1)
#define SLAVE_LEGACY_LOWSIDES        1
#else
#define SLAVE_LEGACY_LOWSIDES        0
#endif
#endif
/* BENCH RESULT (v0.6.7 / v0.6.8, legacy relays, low sides on): with SLAVE_LOWSIDES_OFF_AFTER 0 the converter starts reliably ONLY on the POSITIVE
 * half-cycle: SLAVE_CONV_FIRST_POL 1 = 10 starts, 0 failures; SLAVE_CONV_FIRST_POL -1 = 0 starts (every one tripped). Without it (either half, lows
 * going off at the 2nd diagonal as in the old firmware) a start took 1 to 5 attempts. The defaults below are therefore 0 (lows stay on while only the
 * converter runs) and +1 (start on the positive half). WHY the negative half fails is NOT understood: the old firmware starts on whichever half comes. */
#ifndef SLAVE_LOWSIDES_OFF_AFTER
#define SLAVE_LOWSIDES_OFF_AFTER     0      /* the lows go off when this many converter diagonals have been energised (0 = keep them on until stop / fault; the old firmware: 2) */
#endif
#if (SLAVE_LEGACY_LOWSIDES != 0) && (SLAVE_POWER_STAGE == 0)
#error "SLAVE_LEGACY_LOWSIDES needs the power stage (SLAVE_POWER_STAGE 1 or 2)"
#endif
#if (SLAVE_LEGACY_LOWSIDES != 0) && (SLAVE_POWER_STAGE >= 2) && (SLAVE_LOWSIDES_OFF_AFTER != 0)
#error "With the inverter (stage 2) the low sides must stay on until a stop / fault: SLAVE_LOWSIDES_OFF_AFTER must be 0"
#endif

/* SLAVE_CONV_FIRST_POL (DIAGNOSTIC): which half-cycle the converter STARTS on after every start / restart.
 *    0 : whichever comes next (default)       +1 : the first diagonal energised is the POSITIVE one (3H+4L)      -1 : the NEGATIVE one (3L+4H)
 * The other half-cycles are skipped (at most 10 ms) until the chosen one comes; everything after the first diagonal is unchanged.
 * Bench history (legacy relays): WITHOUT the low sides (v0.6.5 / v0.6.6) every start tripped 24 us after its first diagonal, on either half (D+ and D-);
 * WITH the low sides on and SLAVE_LOWSIDES_OFF_AFTER 0 (v0.6.7): +1 = 10 starts / 0 failures, -1 = 0 starts. The default is therefore +1 for the legacy
 * converter-only start. Use 0 to start on whichever half comes (the old firmware's rule). */
#ifndef SLAVE_CONV_FIRST_POL
#if (SLAVE_RELAYS_MODE == 2) && (SLAVE_POWER_STAGE >= 1)
#define SLAVE_CONV_FIRST_POL         1      /* converter-only with the legacy relays: start on the POSITIVE half (bench: 10/10 starts; negative: 0/10) */
#else
#define SLAVE_CONV_FIRST_POL         0
#endif
#endif
#if (SLAVE_CONV_FIRST_POL != 0) && (SLAVE_CONV_FIRST_POL != 1) && (SLAVE_CONV_FIRST_POL != -1)
#error "SLAVE_CONV_FIRST_POL must be 0, 1 or -1"
#endif

/* SLAVE_CONV_FIRST_AT_PEAK (EXPERIMENT, stage 1): turn the converter's FIRST diagonal on at the line PEAK (a quarter cycle, about 5 ms, after the zero-cross event)
 * instead of 40 us after it. Later diagonals are unchanged. REASONING (a hypothesis, NOT shown): 40 us after the zero crossing the line is at about 0 V while the DC link
 * sits near the mains peak, so the first turn-on puts nearly the whole peak across the series winding's leakage inductance; the current ramps for about 25 us until the
 * comparator trips (dt=0.06ms on the master). The step grows with the voltage and a toroid has little leakage, so it trips first and only at the highest voltages (bench: the
 * toroid trips above about 230 V; a longer W settle moved that to 250 V (5 s) and 260 V (10 s), a fit that saturates near 265 V). At the peak, line and DC link are nearly equal:
 * no step. 0 = off (default), 1 = on. The start sequence, the low sides and the first half-cycle polarity are as before. */
#ifndef SLAVE_CONV_FIRST_AT_PEAK
#define SLAVE_CONV_FIRST_AT_PEAK     0
#endif
#if (SLAVE_CONV_FIRST_AT_PEAK != 0) && (SLAVE_POWER_STAGE == 0)
#error "SLAVE_CONV_FIRST_AT_PEAK needs the power stage (SLAVE_POWER_STAGE 1 or 2)"
#endif

/* SLAVE_TF_CHECK (stage 1, legacy relays): TRANSFORMER-CONNECTION CHECK on every converter start ("in the boot-up").
 * The old firmware's transformer test (found in its code): W-RLY on (the primary nets open), the converter running, NO inverter duty, then ONE comparison:
 *     Volt_Diff = Vout - Vin ;  Flip_mode = (Volt_Diff > 0) ? normal : flipped        (threshold `offset` = 0)
 * This firmware does the same measurement passively: once the converter has really been running for CTRL_TF_SETTLE_MS it averages (Vout - Vin) over
 * CTRL_TF_WINDOW_MS and shows it on the master, row 3, every other 2 s:   TF OK +3.2V  /  TF FLIP -3.0V  /  TF ? +0.2V (inside the +/-CTRL_TF_DEADBAND_V dead band).
 * NOTHING extra is driven: no duty, no relay change. HONEST LIMIT: it is NOT understood from the old code WHY Vout - Vin should depend on the transformer
 * polarity with the converter only, so VALIDATE it once: run with the transformer as installed (it works with the old firmware: expect OK), then power off,
 * swap the two leads of ONE winding, run again: the sign must flip. If it does not, this check does not discriminate and the injection test of stage 2 is needed.
 * 1 = on (default with SLAVE_RELAYS_MODE 2 at stage 1), 0 = off. */
#ifndef SLAVE_TF_CHECK
#if (SLAVE_RELAYS_MODE == 2) && (SLAVE_POWER_STAGE == 1)
#define SLAVE_TF_CHECK               1
#else
#define SLAVE_TF_CHECK               0
#endif
#endif
#if (SLAVE_TF_CHECK != 0) && (SLAVE_POWER_STAGE != 1)
#error "SLAVE_TF_CHECK is for converter-only operation (SLAVE_POWER_STAGE 1): with the inverter running Vout - Vin means something else"
#endif

/* BENCH ONLY (converter-only tests): retry quickly after a trip instead of the 10 / 20 / 30 s lockouts and the latch after 6 trips (the old
 * firmware re-armed after about 3 s). Every trip is a REAL current spike: use the series lamp and stop after a handful. Remove the comment marks: */
/* #define CTRL_RESTART_STEP_MS 3000UL */
/* #define CTRL_RESTART_MAX_MS  3000UL */
/* #define CTRL_LATCH_AFTER     255U   */

#define RELAY_UNMEASURED             0
#define RELAY_SHORTS_WHEN_ENERGISED  1
#define RELAY_SHORTS_WHEN_RELEASED   2
#define RELAY_NOT_A_SHUNT            3
#ifndef SLAVE_RELAY_SH_CONTACT
#define SLAVE_RELAY_SH_CONTACT       RELAY_UNMEASURED
#endif
#ifndef SLAVE_RELAY_W_CONTACT
#define SLAVE_RELAY_W_CONTACT        RELAY_UNMEASURED
#endif
#ifndef SLAVE_RELAY_SETTLE_MS
#define SLAVE_RELAY_SETTLE_MS        100     /* time allowed for a relay to move (measure the real one; 5..20 ms is typical) */
#endif
#if (SLAVE_RELAY_SETTLE_MS < 20)
#error "SLAVE_RELAY_SETTLE_MS below 20 ms is not credible for a power relay (operate + bounce time): the firmware has NO sensing of the real contact state"
#endif

/* Bring-up helpers (previously defined at the top of main.c) */
#ifndef BRINGUP_PWM_MODE
#define BRINGUP_PWM_MODE       0       /* 0 = PWM off, 1 = 50 % debug PWM (gate supply only!) */
#endif

#ifndef BRINGUP_REPORT_MODE
#define BRINGUP_REPORT_MODE    2       /* 0 = ZC beep codes, 1 = voltage/frequency, 2 = silent */
#endif

/* ---- Compile-time safety cross-checks ------------------------------------ */
#if (SLAVE_POWER_STAGE < 0) || (SLAVE_POWER_STAGE > 3)
#error "SLAVE_POWER_STAGE must be 0, 1, 2 or 3"
#endif

#if (SLAVE_POWER_STAGE >= 1) && (BRINGUP_PWM_MODE != 0)
#error "The 50 % debug PWM (BRINGUP_PWM_MODE) must be OFF when the power stage is enabled"
#endif

#if (SLAVE_POWER_STAGE >= 1) && (BRINGUP_REPORT_MODE != 2)
#error "The power stage needs a non-blocking main loop: set BRINGUP_REPORT_MODE to 2 (silent)"
#endif

#if (SLAVE_POWER_STAGE >= 2) && (SLAVE_RELAYS_MODE == 0) && (SLAVE_RELAYS_CONFIRMED == 0)
#error "Stages 2/3 drive the inverter. With SLAVE_RELAYS_MODE 0 the firmware cannot check the relays: set SLAVE_RELAYS_CONFIRMED 1 only after verifying their state by measurement"
#endif

#if (SLAVE_RELAYS_MODE < 0) || (SLAVE_RELAYS_MODE > 2)
#error "SLAVE_RELAYS_MODE must be 0 (by hand), 1 (bypass-relay rules) or 2 (legacy sequence)"
#endif

/* Stage 3 (closed loop) with the legacy relays: released in v0.8.0 after the stage-2 injection test was proven on the bench (direction OK at 2 %, 0.7 V at about
 * 110 V and 1.9 to 2.1 V at 200 to 250 V). Its FIRST run is capped (CTRL_STAGE3_MAX_EFF_TICKS, default 640 = about 20 V of correction): see docs/INVERTER_STAGE2_BENCH.md. */

#if (SLAVE_POWER_STAGE >= 2) && (SLAVE_RELAYS_MODE == 1) && \
    ((SLAVE_RELAY_SH_CONTACT == RELAY_UNMEASURED) || (SLAVE_RELAY_W_CONTACT == RELAY_UNMEASURED))
#error "Stages 2/3 need to know what the relay contacts do: measure SH-RLY and W-RLY (continuity PRI-PHASE <-> PRI-NTRL, coil off / on) and set SLAVE_RELAY_SH_CONTACT and SLAVE_RELAY_W_CONTACT"
#endif

#if (SLAVE_POWER_STAGE >= 2) && (SLAVE_RELAYS_MODE == 1) && \
    (SLAVE_RELAY_SH_CONTACT == RELAY_NOT_A_SHUNT) && (SLAVE_RELAY_W_CONTACT == RELAY_NOT_A_SHUNT)
#error "If NEITHER relay shorts the primary, the reading of the power schematic (bypass relays across PRI-PHASE/PRI-NTRL) is wrong: the primary would be open whenever the inverter is off. Do not run the inverter - re-examine what SH-RLY / W-RLY switch."
#endif

#if (SLAVE_BENCH_AUTORUN != 0) && (SLAVE_POWER_STAGE == 0)
#error "SLAVE_BENCH_AUTORUN has no meaning with SLAVE_POWER_STAGE 0"
#endif


/* ======================= FAST CORRECTION profile, step 2 (v0.9.25) =======================
 * The fast-start master-control config plus the fastest correction that the simulation and the protection tests accept:
 *   CTRL_GUARD_RESTART_SLOPE_VPS 100  (default 30; the 60 profile is the milder step): after an input step of 15 V or more the correction restarts from zero and ramps back at this many V/s.
 *   CTRL_GUARD_RESTART_FINAL_VPS  30  (default 10): the ramp speed in the last 3 V before the set voltage.
 * Simulation, a 40 V input step: within 0.5 V of the set voltage in 0.6 s (1.6 s with the defaults, 1.0 s with the 60 profile).
 * NOT changed, on purpose: the regulator gain (REG_KI_SCALE 1.0: a higher gain gave NO speed in the simulation and makes the transformer-flip check let the output move further the wrong way: 3.4 V of the 3.5 V allowed at x1.5)
 * and the freeze debounce (300 ms: at 150 ms the flip tests failed).
 * NOT tried on hardware. Measure with the response timer (LOG, MENU x2: the first row shows "3.4s/41V" = the last recovery: seconds / largest error) and go back to the 60 profile or the plain fast-start config if the unit trips (F02 / F01) on input steps. */
#define CTRL_GUARD_RESTART_SLOPE_VPS  100.0f
#define CTRL_GUARD_RESTART_FINAL_VPS  30.0f


/* ======================= + CONVERTER-SIDE CURRENT (I-OUT on PA2) =======================
 * This config = the 100 V/s fast-correction profile above + the 4th ADC channel for the CT2500 sense chain (net I-OUT, PA2): its AC RMS and peak go to the master in $E and are shown on the Converter Current page (LOG, MENU x7).
 * Confirmed on the bench (v0.9.31) with a 68 ohm x 3 burden. v0.9.58: the burden is 180 ohm x 3 = 60 ohm (the drawing's value): 0.1632 V per ampere, 4.9 mA per count, the channel CLIPS at +-10.1 A peak
 * (a load of roughly 13 A at the converter peaks measured on the bench: above it the slave's load reading is "clipped" and the Y / B overload trip has no reading), and the hardware
 * short-circuit comparators trip at about +-41 A peak (it was 108 A with 68 ohm). With 68 ohm resistors fitted put 68.0f back here. See docs/IOUT.md. */
#define SLAVE_IOUT 1
#define IOUT_BURDEN_R_EACH_OHM 180.0f

#endif /* SLAVE_CONFIG_H */
