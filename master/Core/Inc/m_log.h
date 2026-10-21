/**
  * @file    m_log.h
  * @brief   Master core, part 3: the fault log, 00 (newest) to 99, 16 bytes a record, two flash pages used as a circular buffer. PURE LOGIC over MFlash_t.
  *          Written only when a fault TRIPS the output (every inrush retry too, D27). The phase byte carries the phase in its low 2 bits (0 R, 1 Y, 2 B, 3 all) and the slave's
  *          fault code in the upper 6 bits (D30). The timestamp is RUNNING time in minutes: there is no RTC (D28).
  *
  * A record is 16 bytes: type, phase|slave_code<<2, vin, vout, load (tenths), run minutes (4), seq (2), CRC-16 (2). Each is programmed as two double words with the CRC in the
  * second, so a power cut leaves a slot that is not blank and not valid: it is skipped and never read. Both pages full: the OLDER page is erased, so at least 128 records always remain.
  */
#ifndef M_LOG_H
#define M_LOG_H
#include <stdint.h>
#include "m_store.h"
#ifdef __cplusplus
extern "C" {
#endif

#define MLOG_REC_BYTES   16U
#define MLOG_SLOTS_PAGE  (MFLASH_PAGE / MLOG_REC_BYTES)      /* 128 */
#define MLOG_SLOTS       (2U * MLOG_SLOTS_PAGE)               /* 256 */
#define MLOG_SHOWN       100U                                  /* entries 00..99 */

/* fault-type byte (the log's type codes, workbook NewScreens) */
#define MLOG_T_IP_LV 1U
#define MLOG_T_IP_HV 2U
#define MLOG_T_OP_LV 3U
#define MLOG_T_OP_HV 4U
#define MLOG_T_OVERLOAD 5U
#define MLOG_T_HEAT 6U
#define MLOG_T_FLIP 7U
#define MLOG_T_INRUSH 8U
#define MLOG_T_PHASE_SEQ 9U
#define MLOG_T_MAINS 10U
#define MLOG_T_POWER_STAGE 11U
#define MLOG_T_RETUNE 12U
#define MLOG_T_LINK_LOST 13U

typedef struct
{
    uint8_t  type;          /* MLOG_T_* */
    uint8_t  phase;         /* 0 R, 1 Y, 2 B, 3 all */
    uint8_t  slave_code;    /* the slave's fault code, 0..63 */
    uint16_t vin_dv, vout_dv, load_da;
    uint32_t run_min;
} MLogRec_t;

typedef struct
{
    const MFlash_t *fl;
    uint32_t page[2];
    uint8_t  active;        /* the page being appended to */
    uint16_t next;          /* the next slot to write in the active page (MLOG_SLOTS_PAGE = full) */
    uint16_t seq;           /* sequence of the newest record */
    uint16_t n;             /* records in the index */
    struct { uint16_t seq; uint32_t addr; } idx[MLOG_SLOTS];    /* newest first */
} MLog_t;

void     MLog_Init(MLog_t *l, const MFlash_t *fl, uint32_t page_a, uint32_t page_b);   /* scans both pages */
int      MLog_Append(MLog_t *l, const MLogRec_t *r);                                    /* 0 = ok */
uint16_t MLog_Count(const MLog_t *l);                                                   /* 0..100 */
int      MLog_Get(const MLog_t *l, uint16_t index, MLogRec_t *out);                     /* 0 = newest; 0 = ok, -1 = no such entry or unreadable */
int      MLog_Clear(MLog_t *l);                                                         /* "Clear Fault Log" */

#ifdef __cplusplus
}
#endif
#endif
