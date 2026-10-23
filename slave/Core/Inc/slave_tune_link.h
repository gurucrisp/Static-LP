/**
  * @file    slave_tune_link.h
  * @brief   The autotune seen from the master's link: $K TUNE / TABORT, the $T progress frames, the BUSY / NOT_TUNED flags, and the storing of the profile when a sweep finishes. Glue between the link (slave_link2), the autotune
  *          core (slave_tune), the profile store (slave_profile) and the controller, through a few functions that the caller provides, so the PC tests can run the whole chain against a simulated transformer.
  *
  *          WHAT IT DOES NOT DO: it asks for an open-loop duty (SlaveTuneLink_RequestTicks), it does not apply it: the controller's tune mode does (the next step). Until then `can_run` says 0 and TUNE is refused ("not supported
  *          in this build"), so a unit never starts a sweep that nothing would drive.
  *
  *          $K TUNE answers (R status, value = the reason when refused): 0 started; 4 NOT_STOPPED (the unit is running, send run = 0 first); 7 NOT_ALLOWED with value 1 = this build has no tune mode, 2 = Max Correction not set,
  *          3 = a calibration or a flash write is in progress, 4 = a tune is already running. $K TABORT is always answered 0. TPARAM and TSAVE (a hand-set profile for service) are not in this version: UNKNOWN_CMD.
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

typedef struct
{
    uint16_t (*maxc_dv)(void);        /* the Max Correction last sent in $M, tenths of a volt (0 = not set) */
    uint8_t  (*stopped)(void);        /* the controller is OFF: no pulses, shunted, no run request */
    uint8_t  (*can_run)(void);        /* this build can apply an open-loop duty (the controller's tune mode): 0 until it exists */
    uint8_t  (*other_busy)(void);     /* a calibration is averaging or a flash write is in progress */
} TuneLinkHw_t;

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
