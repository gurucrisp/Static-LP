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
#define SLAVE_FW_VERSION       916UL
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
