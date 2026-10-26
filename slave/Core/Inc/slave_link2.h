/**
  ******************************************************************************
  * @file    slave_link2.h
  * @brief   The slave's side of link version 2 (decisions D1, D2, D9, D12): what to do with the frames that arrive and when to send the
  *          frames that go out. PURE LOGIC: no HAL, no registers, no globals other than its own state. It reaches the rest of the slave only
  *          through the hooks below, so the PC tests drive it with fake hooks and, in the integration test, with the real controller.
  *
  *   in   $M control   -> on_control()        a LEVEL, repeated every 100 ms by the master
  *        $K command   -> answered by a $R with the same id. A repeated id is answered AGAIN but NOT executed twice (the master retries with the same id).
  *                        Only HELLO is implemented in this firmware version: every other command word is answered UNKNOWN_CMD (status 6),
  *                        so a master that sends them is told plainly that they are not there yet.
  *   out  $H hello     3 times at start, 100 ms apart, and on $K HELLO
  *        $E extended  every SL2_E_PERIOD_MS (100 ms); its seq counts every attempt, even one that found the queue full, so the master sees the gap
  *        $R response  at once (the first thing sent when several are due)
  *
  * The $S status frame (every 10 ms) is NOT here: it stays in slave_link.c exactly as before.
  * Everything runs in the main-loop context; there is no concurrency inside this module.
  ******************************************************************************
  */
#ifndef SLAVE_LINK2_H
#define SLAVE_LINK2_H

#include <stdint.h>
#include "link_v2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SL2_E_PERIOD_MS    100U
#define SL2_H_BOOT_COUNT   3U
#define SL2_H_PERIOD_MS    100U
#define SL2_R_QUEUE        4U

typedef struct
{
    /* a valid $M arrived (now_ms = the time base the controller uses) */
    void    (*on_control)(const LinkM_t *m, uint32_t now_ms);
    /* fill setv_dv, flags2, clr, reach_lo_dv, reach_hi_dv; the module sets seq */
    void    (*fill_e)(LinkE_t *e);
    /* fill everything except proto, which is the module's */
    void    (*fill_h)(LinkH_t *h);
    /* queue one complete frame (with CR LF) for transmission: 1 = queued, 0 = no room */
    uint8_t (*tx)(const char *frame, uint32_t len);
    /* voltage calibration ($K CALV / CALO / CALR): cmd is the LinkCmd_t value. Return 1 = the answer is ready (status, value); 0 = the averaging has started, the answer comes from cal_poll.
     * NULL = this build has no calibration: the command is answered UNKNOWN_CMD. */
    int     (*cal_command)(uint8_t cmd, int32_t num, char letter, uint8_t *status, int32_t *value, uint32_t now_ms);
    int     (*cal_poll)(uint8_t *status, int32_t *value, uint32_t now_ms);                  /* 1 = the pending command is done */
    /* the transformer autotune ($K TUNE / TABORT / TPARAM / TSAVE): answered at once. Return 0 = this build does not know the command (it is answered UNKNOWN_CMD). NULL = no autotune in this build. */
    int     (*tune_command)(uint8_t cmd, const char *name, int32_t num, uint8_t *status, int32_t *value, uint32_t now_ms);
    /* a $T progress frame is ready: 1 and the frame; the link keeps it and offers it to the transmitter until it is accepted */
    int     (*tune_frame)(LinkT_t *f, uint32_t now_ms);
    /* the phase sequence measurement ($K SEQ <round>): the next SYNC byte is real. Answered OK at once. NULL = this build has no such measurement: UNKNOWN_CMD. (v0.9.51) */
    void    (*seq_arm)(uint8_t round, uint32_t now_ms);
} SL2_Hooks_t;

typedef struct
{
    uint32_t m_ok;          /* valid $M frames                                  */
    uint32_t k_ok;          /* valid $K frames (a retry counts again)           */
    uint32_t k_repeat;      /* $K frames whose id was the previous one          */
    uint32_t r_sent, e_sent, h_sent;
    uint32_t tx_dropped;    /* a frame the hook could not queue                 */
    uint32_t r_dropped;     /* a response that found the response queue full    */
    uint32_t rx_bad;        /* complete lines rejected (checksum, format, range) */
    uint32_t rx_unknown;    /* good checksum, a frame type this version does not know */
} SL2_Stats_t;

/* hooks may be NULL only where a test does not need them (a NULL hook is skipped). */
void SL2_Init(const SL2_Hooks_t *hooks, uint32_t now_ms);
void SL2_Rx(uint8_t byte, uint32_t now_ms);          /* feed every received byte */
void SL2_Service(uint32_t now_ms);                   /* call on every main-loop pass */
const SL2_Stats_t *SL2_Stats(void);

#ifdef __cplusplus
}
#endif

#endif /* SLAVE_LINK2_H */
