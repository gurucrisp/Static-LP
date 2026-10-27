/**
  ******************************************************************************
  * @file    slave_features.h
  * @brief   Defaults and guards for the features added from v0.9.0 on: link version 2 and master control.
  *
  * WHY A SEPARATE FILE: the ready-made configs (the files in the configs folder) are copied over slave_config.h as a whole. Anything that lives in slave_config.h is therefore
  * missing when an OLDER config is copied over it. The defaults below are applied after whichever slave_config.h is in use, so every older config still
  * builds, with these features OFF (the v1 behaviour, exactly as before).
  ******************************************************************************
  */
#ifndef SLAVE_FEATURES_H
#define SLAVE_FEATURES_H

#include "slave_config.h"

/* ---- Link version 2 and master control (roadmap step 3; decisions D1, D5, D9, D12) ---------------------------------------------------------------------------
 * SLAVE_LINK_V2         1 = the slave sends $E / $H / $R and reads $M / $K (link library version 2). 0 (the default) = exactly the v1 behaviour: only the $S status frame.
 * SLAVE_MASTER_CONTROL  1 = the MASTER runs the slave: no autorun; the run request is the LEVEL in the $M frames; the set voltage comes from $M; EVERY fault latches until the
 *                       master changes the clear counter (no automatic retries); no valid $M for CTRL_LINK_TIMEOUT_MS (1 s) stops the slave. Needs SLAVE_LINK_V2 and a power stage. */
#ifndef SLAVE_LINK_V2
#define SLAVE_LINK_V2          0
#endif
#ifndef SLAVE_MASTER_CONTROL
#define SLAVE_MASTER_CONTROL   0
#endif
/* v0.9.51: the phase sequence measurement (slave_phase.c): the slave time-stamps the master's SYNC byte in its receive interrupt and reports the time to its next rising zero-cross.
 * ON by default wherever a master runs the slave over link v2 (the three-phase unit needs it; a single-phase unit just answers the rounds if the master starts any: it does not in
 * Single mode). 0 = the build has no such measurement and answers $K SEQ with UNKNOWN_CMD. */
#ifndef SLAVE_SEQ_SYNC
#define SLAVE_SEQ_SYNC         (((SLAVE_MASTER_CONTROL != 0) && (SLAVE_LINK_V2 != 0)) ? 1 : 0)
#endif
#if (SLAVE_SEQ_SYNC != 0) && (SLAVE_LINK_V2 == 0)
#error "SLAVE_SEQ_SYNC needs SLAVE_LINK_V2 (the $K SEQ command and the $E result are version 2 frames)"
#endif
/* 1 = switch on the USART's 8-byte receive FIFO (and the transmit FIFO) when SLAVE_LINK_V2 is on: at 500 kbaud a byte arrives every 20 us, the FIFO gives the receive interrupt about 160 us of slack
 * when a higher-priority interrupt runs long. If the link ever misbehaves after this change, set 0 and tell me: the counters in g_link (rx_overruns) show whether bytes were lost. */
#ifndef SLAVE_LINK_RX_FIFO
#define SLAVE_LINK_RX_FIFO     1
#endif
/* INJECTION GUARD (v0.9.11, stages 3 and up). 1 (the default) = the correction is taken to ZERO and held, and the regulator restarts from zero, (a) before the boost/buck DIRECTION is reversed while the correction is
 * larger than CTRL_DIR_FLIP_MAX_TICKS, and (b) when the input voltage steps by CTRL_VIN_STEP_V or more between two RMS results (a fast variac, a mains switch-over), until the input has been steady for
 * CTRL_GUARD_HOLD_MS. Without it the controller reversed the injection at full duty in the same pass as the input crossed the set point. 0 = the old behaviour, for comparison only. */
/* 1 = a RARE zero-cross glitch while RUNNING does not stop the unit. Two of the slave's polarity checks, F16 (the last-moment sign check before a diagonal is energised) and F04 (the sign at an event is too small to read),
 * used to latch the unit at the first occurrence; in master-control mode that opens the load contactor for a noise spike. With 1 the slave instead SKIPS that half-cycle (nothing is energised: the same safe state the check
 * enforced) and counts it; a third in 2 s fails exactly as before, so a real polarity problem (it fails on every half-cycle) still stops the unit within about 30 ms. Only while RUN: during a start the checks stay strict.
 * 0 = every one of them is a fault (the behaviour of every standalone build). The master-control configs set 1. */
/* 1 = the TRANSFORMER-CONNECTION (FLIP) CHECK at every start, in stage 3: while the regulator starts up, the output must move in the direction it is told to. Before any correction the sensor offset (Vout - Vin) is recorded;
 * then the signed change in the commanded direction is watched in 100 ms windows: about +1 V = correct (the check ends), about -1 V in two windows in a row (or -2.5 V in one) = the series transformer is connected BACKWARDS: fault 17, latched until the master clears it
 * (the wrong-way deviation is limited to a few volts). It only judges once the correction is large enough to see; a unit that needs no correction is not tested (and not at risk); it stops watching 60 s after the start.
 * 0 = no such check (every standalone build). The master-control configs set 1. */
/* 1 = the voltage calibration over the link ($K CALV / CALO / CALR): single-point gain from the meter reading the master sends, only while the unit is STOPPED, averaged for 1 s, +-10 % of nominal, 50 V minimum,
 * stored in two CRC-protected copies in the last two flash pages (0x0803F000 and 0x0803F800). 0 = no calibration commands (they are answered UNKNOWN_CMD). The master-control configs set 1. */
/* 1 = the independent watchdog (IWDG) runs: the main loop kicks it once per pass; if the loop stops for SLAVE_WATCHDOG_MS the CPU is reset, the timers and all pins go back to their reset state (pulses off) and the unit
 * waits for the master again. The reason is reported to the master in $H (rst = 1). 0 = no watchdog. */
/* 1 = the transformer AUTOTUNE chain is built in (profile storage in flash pages 124 and 125, and as the later steps are added: the link commands and the controller's tune mode). 0 = nothing of it: the controller runs on its built-in profile
 * as before and $H reports tuned = 1. OFF until the whole chain has been tested on the bench. */
/* 1 = the controller's TUNE MODE (for the transformer autotune): on a TUNE command the unit starts by itself through the normal start sequence (SYNC, RECT, INVERTER, UNSHUNT) and in RUN, instead of the regulator, applies the sweep's
 * requested duty as an OPEN-LOOP BOOST (slew-limited, never above CTRL_STAGE3_MAX_EFF_TICKS, never more than Max Correction + 8 V of injection), with every normal protection still active. The master keeps the output contactor open. Needs SLAVE_AUTOTUNE 1,
 * SLAVE_MASTER_CONTROL 1 and SLAVE_POWER_STAGE >= 3. 0 = a TUNE command is refused ("no tune mode"). OFF until the first bench run has been done. */
/* v0.9.47: THE AUTOTUNE SWITCHES ARE ON BY DEFAULT in the production build (power stage 3 with a master over link v2) and OFF in any other build (a bench build without a master, a smaller power stage). ONE default feeds
 * SLAVE_AUTOTUNE and the others follow it (TUNE_MODE and APPLY follow AUTOTUNE, ENFORCE follows APPLY and TUNE_MODE), so a config that switches AUTOTUNE off switches them all off and none of the checks below can fire by accident.
 * A config can still set any of them. The converter-current channel (SLAVE_IOUT) follows the same default: the sweep needs it (TUNE_REQUIRE_IOUT). With ENFORCE on, a unit WITHOUT a valid profile does not run until it has been tuned (Set-Mode, Re-tune). */
#if (SLAVE_POWER_STAGE >= 3) && (SLAVE_MASTER_CONTROL != 0) && (SLAVE_LINK_V2 != 0)
#define SLAVE_TUNE_DEFAULT     1
#else
#define SLAVE_TUNE_DEFAULT     0
#endif
#ifndef SLAVE_AUTOTUNE
#define SLAVE_AUTOTUNE         SLAVE_TUNE_DEFAULT
#endif
#ifndef SLAVE_TUNE_MODE
#define SLAVE_TUNE_MODE        SLAVE_AUTOTUNE
#endif

/* 1 = USE the stored transformer profile (the autotune's result): the regulator is capped at the profile's cap, its integral gain is scaled by the profile's gain scale (boost and buck), and at every start in BOOST the injection that is really
 * measured is compared with the one the profile predicts (slave_tunechk.h): a different transformer gives fault 19 (latched, the master shows "Re-tune Required"). 0 = the profile is stored but ignored (the regulator is exactly as before).
 * Needs SLAVE_AUTOTUNE 1, SLAVE_MASTER_CONTROL 1, SLAVE_POWER_STAGE >= 3. OFF until the sweep has been run on a real unit and the tolerances have been checked. */
#ifndef SLAVE_TUNE_APPLY
#define SLAVE_TUNE_APPLY       SLAVE_AUTOTUNE
#endif
/* 1 = REFUSE TO RUN without a valid profile: a start raises fault 18 (latched, "Re-tune Required"). The Re-tune item still works from that state. Needs SLAVE_TUNE_APPLY 1. */
#ifndef SLAVE_TUNE_ENFORCE
#define SLAVE_TUNE_ENFORCE     ((SLAVE_TUNE_APPLY != 0) && (SLAVE_TUNE_MODE != 0))
#endif

#if (SLAVE_TUNE_MODE != 0) && (SLAVE_AUTOTUNE == 0)
#error "SLAVE_TUNE_MODE 1 needs SLAVE_AUTOTUNE 1: the link commands that start a sweep are part of the autotune"
#endif
#if (SLAVE_TUNE_APPLY != 0) && ((SLAVE_AUTOTUNE == 0) || (SLAVE_MASTER_CONTROL == 0) || (SLAVE_POWER_STAGE < 3))
#error "SLAVE_TUNE_APPLY 1 needs SLAVE_AUTOTUNE 1, SLAVE_MASTER_CONTROL 1 and SLAVE_POWER_STAGE >= 3"
#endif
#if (SLAVE_TUNE_ENFORCE != 0) && (SLAVE_TUNE_APPLY == 0)
#error "SLAVE_TUNE_ENFORCE 1 needs SLAVE_TUNE_APPLY 1"
#endif
#if (SLAVE_TUNE_ENFORCE != 0) && (SLAVE_TUNE_MODE == 0)
#error "SLAVE_TUNE_ENFORCE 1 needs SLAVE_TUNE_MODE 1: a unit that refuses to run until it is tuned must be able to tune itself"
#endif

/* 1 = the converter-side CURRENT (net I-OUT, connector pin 32, PA2 = ADC1_IN2) is sampled as a 4th ADC channel together with AC_IN, AC_OUT and the NTC, its AC RMS and peak are computed (slave_iout.h) and sent to the master in $E.
 * The channel is read after the NTC, so the other three keep their places. 0 = the ADC has the 3 channels it always had and nothing changes. (If the project is regenerated with CubeMX, set PA2 as ADC1_IN2 there too.) */
#ifndef SLAVE_IOUT
#define SLAVE_IOUT             SLAVE_TUNE_DEFAULT      /* on in the production build: the sweep needs it, and the master's Slave Load Cal page */
#endif

#ifndef SLAVE_WATCHDOG
#define SLAVE_WATCHDOG         1
#endif
#ifndef SLAVE_WATCHDOG_MS
#define SLAVE_WATCHDOG_MS      500U      /* the longest a main-loop pass may take: the flash erase of a calibration save takes tens of ms; the real timeout is 0.75 to 1.3 times this (LSI spread) */
#endif

#ifndef SLAVE_VOLTAGE_CAL
#define SLAVE_VOLTAGE_CAL      0
#endif
#ifndef SLAVE_TF_RUN_CHECK
#define SLAVE_TF_RUN_CHECK     0
#endif
#ifndef SLAVE_GLITCH_SKIP
#define SLAVE_GLITCH_SKIP      0
#endif
#ifndef SLAVE_INJ_GUARD
#define SLAVE_INJ_GUARD        1
#endif

/* TEST ONLY: 1 = the slave applies NO voltage limits of its own. The input-range trip (CF_VIN_RANGE), the narrower input START window, and the output-high trip (CF_VOUT_HIGH, CTRL_VOUT_MAX_V)
 * are all off, and its $H frame announces a wide window (0 to 650 V) so the master's Set-Mode accepts any cut-off. What stays: the hardware over-current protection (FLT / break input), the sensor and
 * commutation checks, the relays and the zero-cross checks. For bench testing the regulation and the master only: NEVER ship with this on. 0 (the default) = the limits as before. */
#ifndef SLAVE_TEST_NO_VOLTAGE_CUTS
#define SLAVE_TEST_NO_VOLTAGE_CUTS   0
#endif
/* major*10000 + minor*100 + patch of the VERSION file; the test runner checks that they agree. Sent in the $H frame. */
#ifndef SLAVE_FW_VERSION
#define SLAVE_FW_VERSION       953UL
#endif

#if (SLAVE_MASTER_CONTROL != 0) && (SLAVE_LINK_V2 == 0)
#error "SLAVE_MASTER_CONTROL needs SLAVE_LINK_V2 1: the master's commands arrive in version-2 frames"
#endif
#if (SLAVE_MASTER_CONTROL != 0) && (SLAVE_BENCH_AUTORUN != 0)
#error "SLAVE_MASTER_CONTROL and SLAVE_BENCH_AUTORUN are exclusive: with a master, the run request is the master's"
#endif
#if ((SLAVE_MASTER_CONTROL != 0) || (SLAVE_LINK_V2 != 0)) && (SLAVE_POWER_STAGE < 1)
#error "SLAVE_LINK_V2 / SLAVE_MASTER_CONTROL need a power stage (SLAVE_POWER_STAGE 1 or more)"
#endif

#endif /* SLAVE_FEATURES_H */
