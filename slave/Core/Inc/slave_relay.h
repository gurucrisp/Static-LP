/**
  ******************************************************************************
  * @file    slave_relay.h
  * @brief   SH-RLY (PC9) and W-RLY (PC8): bypass relays across the primary of the
  *          series-injection transformer. Sequencing derived from the HARDWARE
  *          and from rules, not copied from the old firmware.
  *
  * TOPOLOGY (power-board schematic, read at high zoom)
  *   inverter leg 1 mid-point  PRI-PHASE --- CT2500 --- fuse F1 --- blade PRI1 --+
  *                                                                              | transformer
  *   inverter leg 2 mid-point  PRI-NTRL  ------------------------ blade -------+ PRIMARY
  *   SH-RLY contact and W-RLY contact: each between PRI-PHASE and PRI-NTRL.
  *   The secondary is IN SERIES with the load path. A closed relay contact shorts
  *   the primary -> the series winding is transparent (bypass). The inverter's
  *   LOW sides on (zero duty) short the primary too.
  *
  * RULES (every one is checked by the host tests, for every contact type):
  *   R1  No inverter HIGH side is driven while any shunt contact is closed
  *       (a closed contact joins the two leg mid-points: high side + other leg's low
  *       side = the DC link across the contact).
  *   R2  A relay coil changes only while the inverter duty is zero.
  *   R3  The primary is never left OPEN and UNCONTROLLED: it is shorted either by a
  *       relay contact or by the inverter low sides. (An open primary under load =
  *       an open-circuited series winding = a dangerous voltage.)
  *   R4  The shunts are opened only with the inverter already enabled at zero duty.
  *   R5  Stopping reverses it: duty to zero, shunts closed, THEN the inverter is
  *       disabled.
  *   After a fault that is not an overcurrent, the converter is switched off and the
  *   duty set to zero FIRST; the inverter is disabled only after the shunts have
  *   closed. After an overcurrent / break everything goes off at once and the
  *   shunts are closed immediately.
  *
  * WHAT THE FIRMWARE CANNOT KNOW: which coil state closes each contact. Set
  * SLAVE_RELAY_SH_CONTACT / SLAVE_RELAY_W_CONTACT in slave_config.h from a
  * continuity measurement. There is no sensing of the real contact state: "settled"
  * means SLAVE_RELAY_SETTLE_MS have passed after the coil command.
  ******************************************************************************
  */
#ifndef SLAVE_RELAY_H
#define SLAVE_RELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "slave_config.h"

typedef enum
{
    RLY_TO_SHORTED = 0,   /* coils commanded to the SHORTED pattern, waiting for the contacts to settle */
    RLY_SHORTED,          /* primary shorted by the relays, settled                                     */
    RLY_TO_FREE,          /* coils commanded to the FREE pattern, waiting                               */
    RLY_FREE              /* primary free (no shunt closed), settled                                    */
} RelayState_t;

/* LEGACY mode (SLAVE_RELAYS_MODE 2): the old firmware's sequence, all values from the old code. */
#ifndef SLAVE_LEGACY_SH_HOLD_MS
#define SLAVE_LEGACY_SH_HOLD_MS     1000U   /* SH-RLY on (pre-charge). Old: 1000 ms on a fault re-arm, 500 ms on a connect command */
#endif
#ifndef SLAVE_LEGACY_W_SETTLE_MS
#define SLAVE_LEGACY_W_SETTLE_MS    1000U   /* W-RLY on for this long before the converter may start (old: 1000 ms) */
#endif
#ifndef SLAVE_LEGACY_RELEASE_MS
#define SLAVE_LEGACY_RELEASE_MS     1000U   /* nobody asked for the relays for this long: release both (old: W opens 1 s after a stop) */
#endif
#ifndef SLAVE_LEGACY_SH_PULLIN_US
#define SLAVE_LEGACY_SH_PULLIN_US   6000U   /* ESTIMATE from the old firmware: SH-RLY is commanded this long BEFORE the predicted zero-cross */
#endif
#ifndef SLAVE_LEGACY_MIN_LEAD_US
#define SLAVE_LEGACY_MIN_LEAD_US     300U   /* a closer prediction than this is not trusted: take the next zero-cross */
#endif

/* Main loop. Called every pass while the relays are WANTED: in SYNC once everything else is ready (this STARTS the legacy
 * sequence) and in every converter-running state (this KEEPS the relays). No-op in modes 0 and 1. */
void    SlaveRelay_Hold(void);

/* Drives the SHORTED (bypass) coil pattern. Call once at start-up. */
void    SlaveRelay_Init(void);

/* Main loop. 1 = the primary should be FREE (shunts open), 0 = SHORTED. The caller guarantees R2 (duty zero). */
void    SlaveRelay_Request(uint8_t want_free);

/* Main loop, every pass: advances the settle timer. */
void    SlaveRelay_Service(uint32_t now_ms);

uint8_t SlaveRelay_IsShorted(void);   /* shorted AND settled (always 1 when the relays are driven by hand) */
uint8_t SlaveRelay_IsFree(void);      /* free AND settled    (always 1 when the relays are driven by hand) */
uint8_t SlaveRelay_UnwindDone(void);  /* the primary is shorted again and settled: mode 1 = shunts closed and settled; legacy = both coils released for
                                         SLAVE_RELAY_SETTLE_MS (W-RLY is normally closed); by hand = always 1 */
uint8_t SlaveRelay_State(void);

/* Main loop. Coils to the SHORTED pattern NOW, whatever the state. */
void    SlaveRelay_ShortNow(void);

#ifdef __cplusplus
}
#endif

#endif /* SLAVE_RELAY_H */
