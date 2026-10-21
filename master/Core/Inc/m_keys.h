/**
  * @file    m_keys.h
  * @brief   Master core, part 5: key handling. PURE LOGIC: raw key levels in, key events out.
  *          Keys: MENU, START, STOP, LOG. START and STOP act on the debounced PRESS. MENU and LOG give a SHORT event on RELEASE, unless the press was part of the MENU+LOG combination.
  *          MENU and LOG held TOGETHER for 2 s give ONE combo event (enter / leave Set-Mode); nothing else fires until BOTH are released again (release-to-rearm), and neither key gives a short press
  *          for that touch. START and STOP are ignored while MENU or LOG is down, so forming the combo can never start or stop the output.
  */
#ifndef M_KEYS_H
#define M_KEYS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MK_DEBOUNCE_MS   30U
#define MK_COMBO_MS      2000U

typedef enum { MK_NONE = 0, MK_MENU_SHORT, MK_START, MK_STOP, MK_LOG_SHORT, MK_COMBO } MKEvent_t;

typedef struct
{
    uint8_t  raw[4], stable[4];      /* 0 MENU, 1 START, 2 STOP, 3 LOG */
    uint32_t t_change[4];
    uint32_t t_both;                 /* when MENU and LOG both became (debounced) down */
    uint8_t  tainted;                /* this touch involved both MENU and LOG: no short events, until all of them are released */
    uint8_t  combo_fired;
    uint8_t  pend;                   /* a second event waiting (a release can produce one event per call) */
    MKEvent_t queued;
} MKeys_t;

void      MK_Init(MKeys_t *k);
/* call every few ms with the raw levels (1 = pressed); returns at most one event per call */
MKEvent_t MK_Update(MKeys_t *k, uint8_t menu, uint8_t start, uint8_t stop, uint8_t log, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
#endif
