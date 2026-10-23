/**
  * @file    slave_tune_link.h
  * @brief   The autotune seen from the master's link: $K TUNE / TABORT, the $T progress frames, the BUSY / NOT_TUNED flags, and the storing of the profile when a sweep finishes. Glue between the link (slave_link2), the autotune
  *          core (slave_tune), the profile store (slave_profile) and the controller, through a few functions that the caller provides, so the PC tests can run the whole chain against a simulated transformer.
  *
  *          THE SEQUENCE: TUNE -> the controller starts by itself in tune mode (SYNC, RECT, INVERTER, UNSHUNT, RUN: about 6 s; the master sees heartbeat $T frames with step 0) -> when it is in RUN and settled the sweep starts (3 s baseline, then the steps)
  *          -> the duty each step asks for goes to the controller (request) -> at the end, or on TABORT, a fault, the loss of the master or a timeout, the controller is told to stop (end). The controller applies the duty itself, with its own limits.
  *
  *          $K TUNE answers (R status, value = the reason when refused): 0 started; 4 NOT_STOPPED (the unit is running, send run = 0 first); 7 NOT_ALLOWED with value 1 = this build has no tune mode, 2 = Max Correction not set,
  *          3 = a calibration or a flash write is in progress, 4 = a tune is already running, 5 = the controller cannot start now (a fault is latched, or it is not OFF),
  *          6 = the input is too high: less than 20 V between the input and the output over-voltage limit. The sweep injects up to 80 % of the SMALLER of Max Correction and that headroom, so a full-range sweep (up to the 2752-tick cap, about 86 V) needs an input of about 160 V. $K TABORT is always answered 0. TPARAM and TSAVE (a hand-set profile for service) are not in this version: UNKNOWN_CMD.
  *          The last $T frame (the result) is sent again 6 times, 0.5 s apart, because $T is not acknowledged: the master can only be slower than that, never miss the result.
  */
#ifndef SLAVE_TUNE_LINK_H
#define SLAVE_TUNE_LINK_H
#include <stdint.h>
#include "link_v2.h"
#include "slave_tune.h"
#ifdef __cplusplus
extern "C" {
#endif

#define STL_FINAL_REPEATS   6U
#define STL_FINAL_GAP_MS    500UL
#define STL_QUEUE           16U
#define STL_PREP_TIMEOUT_MS 25000UL   /* the controller has this long to go through its start sequence and reach RUN */
#define STL_HEARTBEAT_MS    2000UL    /* while nothing else is sent, a $T with step 0 says "still preparing / measuring the baseline" (the master gives up after 8 s of silence) */

typedef struct
{
    uint16_t (*maxc_dv)(void);        /* the Max Correction last sent in $M, tenths of a volt (0 = not set) */
    uint8_t  (*stopped)(void);        /* the controller is OFF: no pulses, shunted, no run request */
    uint8_t  (*can_run)(void);        /* this build has the controller's tune mode: 0 = TUNE is refused ("no tune mode") */
    uint8_t  (*other_busy)(void);     /* a calibration is averaging or a flash write is in progress */
    uint8_t  (*begin)(void);          /* the controller starts its start sequence in tune mode: 1 = accepted */
    void     (*request)(uint16_t ticks);   /* the open-loop duty the sweep asks for now (effective ticks, boost) */
    void     (*end)(void);            /* the controller stops the sweep's run: duty to zero, shunts close */
    uint8_t  (*ready)(void);          /* the controller is in RUN and settled: the sweep may start */
    uint8_t  (*alive)(void);          /* tune mode is still on in the controller (it is not when the master link was lost) */
    uint16_t (*fault)(void);          /* the controller's fault code, 0 = none */
    uint16_t (*headroom_dv)(void);    /* how much the output may rise above the input now before the OUTPUT over-voltage limit (less a margin), tenths of a volt; NULL = no limit. The sweep's limit is the smaller of this and Max Correction. */
} TuneLinkHw_t;
#define STL_MIN_HEADROOM_DV 200U      /* less than 20 V of headroom: the sweep would learn almost nothing, TUNE is refused (reason 6: lower the mains) */

void     SlaveTuneLink_Init(const TuneLinkHw_t *hw);
int      SlaveTuneLink_Command(uint8_t cmd, const char *name, int32_t num, uint8_t *status, int32_t *value, uint32_t now_ms);   /* the link's tune_command hook */
void     SlaveTuneLink_Service(uint32_t now_ms, const TuneIn_t *in);                 /* every measurement update (10 ms) */
int      SlaveTuneLink_NextFrame(LinkT_t *f, uint32_t now_ms);                       /* the link's tune_frame hook */
uint16_t SlaveTuneLink_RequestTicks(void);                                           /* the open-loop duty the sweep asks for now (0 = none) */
uint8_t  SlaveTuneLink_Active(void);                                                 /* a sweep is running: the controller must run its tune mode */
uint16_t SlaveTuneLink_Flags2(void);                                                 /* LINKV2_FLAG_BUSY while a sweep runs, LINKV2_FLAG_NOT_TUNED without a valid profile or Max Correction */

#ifdef __cplusplus
}
#endif
#endif
