/**
  ******************************************************************************
  * @file    link_v2.h
  * @brief   Master <-> slave link, VERSION 2 (decisions D1, D2, D12; the Frames sheet of
  *          slave_link_protocol_v2_draft.xlsx). SHARED SOURCE: this file and link_v2.c are
  *          identical in the slave and the master projects, and are compiled on the PC by
  *          tests/test_link_v2.c. No HAL, no globals, no dynamic memory.
  *
  * Version 1 (link_proto.h) is UNTOUCHED and FROZEN: the $S status frame is built and parsed
  * exactly as before. Version 2 adds NEW frame types and one parser that knows them all:
  *
  *   slave -> master   $S status (v1, every 10 ms)        $E extended status (every 100 ms)
  *                     $H hello (at start, on request)    $R response to a $K command
  *                     $T autotune progress
  *   master -> slave   $M control (every 100 ms and at once on a change)
  *                     $K command (acknowledged by a $R with the same id)
  *
  * Framing is the one of version 1:   $<type>,<field>,...*<CS><CR><LF>   with CS = XOR of the
  * bytes between '$' and '*', two UPPER-CASE hex digits, 96 bytes at most. Numbers are ASCII
  * decimal ('-' for the signed ones). A frame with MORE fields than defined is accepted (the
  * extra ones are ignored), so either end can gain fields later; a frame with FEWER is rejected.
  * A frame type this version does not know, with a good checksum, is counted and ignored.
  *
  * The documented RANGE of every field is enforced by the builders AND the parser from one
  * table, so they cannot disagree: a frame that is out of range is not built, and a received
  * one is rejected with LINK_ERR_RANGE (the checksum is only 8 bits wide: the ranges are the
  * second line of defence for the frames that command the power stage).
  ******************************************************************************
  */
#ifndef LINK_V2_H
#define LINK_V2_H

#include <stdint.h>
#include "link_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LINKV2_PROTO_VERSION  2U
#define LINKV2_MAX_LEN        80U      /* a v2 frame built here never exceeds this, '$' to '\n' */

typedef enum
{
    LINKV2_NONE = 0,
    LINKV2_S,      /* v1 status frame, parsed by Link_ParseStatus */
    LINKV2_E,
    LINKV2_H,
    LINKV2_R,
    LINKV2_T,
    LINKV2_M,
    LINKV2_K
} LinkV2Type_t;

/* ---- $E extended status: seq,setv,flags2,clr,reach_lo,reach_hi[,iout_rms_ca,iout_pk_ca] ---------- the last two are OPTIONAL: an older sender gives 6 fields, an older receiver ignores the extra ones */
#define LINKV2_FLAG_RUN_ACK        0x0001U   /* the slave has accepted run = 1                                   */
#define LINKV2_FLAG_IN_BAND        0x0002U   /* running and |Vout - set voltage| within the band                 */
#define LINKV2_FLAG_AT_CAP         0x0004U   /* the correction sits at its cap                                   */
#define LINKV2_FLAG_FLIP           0x0008U   /* transformer flip (latched)                                       */
#define LINKV2_FLAG_OVERHEAT       0x0010U   /* the heat-sink limit input is asserted (debounced)                */
#define LINKV2_FLAG_LINK_LOST      0x0020U   /* no valid $M for more than the link timeout: the slave stopped    */
#define LINKV2_FLAG_FAULT_LATCHED  0x0040U   /* a fault is held until the clear counter changes                  */
#define LINKV2_FLAG_BUSY           0x0080U   /* calibration, a flash write or an autotune is in progress         */
#define LINKV2_FLAG_NOT_TUNED      0x0100U   /* no valid autotune profile / Max Correction not set               */
#define LINKV2_FLAG_IOUT_VALID     0x0200U   /* the slave measures the converter-side current (I-OUT): iout_rms_ca / iout_pk_ca are real      */
#define LINKV2_FLAG_IOUT_CLIP      0x0400U   /* that measurement is at the limit of its range: the true current is larger                       */

typedef struct
{
    uint8_t  seq;          /* 0..255                                                   */
    uint16_t setv_dv;      /* set voltage in use, tenths of a volt: 0 (none yet) or 1000..3000 */
    uint16_t flags2;       /* LINKV2_FLAG_*                                            */
    uint8_t  clr;          /* last clear counter seen in $M, 0..255                    */
    uint16_t reach_lo_dv;  /* lowest input from which the set voltage is reachable: 0 (unknown) or 500..6553 */
    uint16_t reach_hi_dv;  /* highest input, same range                                */
    uint16_t iout_rms_ca;  /* converter-side current (I-OUT), AC RMS, hundredths of an ampere, 0..4000 (valid only with LINKV2_FLAG_IOUT_VALID) */
    uint16_t iout_pk_ca;   /* its peak, same unit                                      */
} LinkE_t;

/* ---- $H hello: proto,fw,vin_min,vin_max,rst,tuned,vout_max ----------------------------- */
#define LINKV2_RST_POWER_ON  0U
#define LINKV2_RST_WATCHDOG  1U
#define LINKV2_RST_OTHER     2U

typedef struct
{
    uint8_t  proto;        /* protocol version, 2                                      */
    uint32_t fw;           /* major*10000 + minor*100 + patch, 0..9999999 (803 = 0.8.3) */
    uint16_t vin_min_dv;   /* the slave's own input window, tenths of a volt           */
    uint16_t vin_max_dv;
    uint8_t  rst;          /* LINKV2_RST_*                                             */
    uint8_t  tuned;        /* 1 = a valid autotune profile is stored                   */
    uint16_t vout_max_dv;  /* the slave's own output guard, tenths of a volt           */
} LinkH_t;

/* ---- $R response to a $K: id,status,value ---------------------------------------------- */
#define LINKV2_ST_OK            0U
#define LINKV2_ST_BAD_ARG       1U
#define LINKV2_ST_BAD_SIGNAL    2U
#define LINKV2_ST_OUT_OF_RANGE  3U
#define LINKV2_ST_NOT_STOPPED   4U
#define LINKV2_ST_FLASH_FAIL    5U
#define LINKV2_ST_UNKNOWN_CMD   6U
#define LINKV2_ST_NOT_ALLOWED   7U

typedef struct
{
    uint8_t id;            /* 1..255, the id of the $K it answers                       */
    uint8_t status;        /* LINKV2_ST_*                                               */
    int32_t value;         /* depends on the command (calibration: 0.01 % against nominal) */
} LinkR_t;

/* ---- $T autotune progress: step,ticks,vin,dv,result ------------------------------------ */
#define LINKV2_T_STEP        0U    /* a step was measured                                  */
#define LINKV2_T_FINISHED    1U    /* done, profile stored                                 */
#define LINKV2_T_FLIPPED     2U
#define LINKV2_T_NO_RESPONSE 3U
#define LINKV2_T_ABORTED     4U
#define LINKV2_T_UNSTEADY    5U
#define LINKV2_T_BY_MASTER   6U

typedef struct
{
    uint8_t  step;         /* 0..255                                                    */
    uint16_t ticks;        /* effective on-time of this step, 0..4095                   */
    uint16_t vin_dv;       /* input voltage, tenths of a volt                           */
    int16_t  dv;           /* injected change, tenths of a volt, -999..999              */
    uint8_t  result;       /* LINKV2_T_*                                                */
} LinkT_t;

/* ---- $M control: seq,run,setv,clr,maxc ------------------------------------------------- */
typedef struct
{
    uint8_t  seq;          /* 0..255                                                    */
    uint8_t  run;          /* 0 = stay off, 1 = regulate: a LEVEL, repeated every 100 ms */
    uint16_t setv_dv;      /* set voltage, tenths of a volt, 1000..3000                 */
    uint8_t  clr;          /* clear counter: a CHANGE clears a latched fault            */
    uint16_t maxc_dv;      /* Max Correction, tenths of a volt, 0 (not set) .. 1990     */
} LinkM_t;

/* ---- $K command: id,cmd[,arg...] -------------------------------------------------------- */
typedef enum
{
    LINKCMD_NONE = 0,
    LINKCMD_HELLO,
    LINKCMD_CALV,          /* num = the true input voltage, tenths of a volt, 500..4000      */
    LINKCMD_CALO,          /* num = the true output voltage, tenths of a volt, 500..4000     */
    LINKCMD_CALR,          /* letter = 'V' or 'O'                                            */
    LINKCMD_TUNE,
    LINKCMD_TABORT,
    LINKCMD_TPARAM,        /* name = CAP, KIB, KIU, INJ, DBND or ABRT; num = the value       */
    LINKCMD_TSAVE,
    LINKCMD_UNKNOWN        /* a well-formed $K with a command word this version does not know: name holds it, so the receiver can answer UNKNOWN_CMD */
} LinkCmd_t;

typedef struct
{
    uint8_t   id;          /* 1..255                                                    */
    LinkCmd_t cmd;
    int32_t   num;         /* CALV, CALO, TPARAM                                        */
    char      letter;      /* CALR                                                      */
    char      name[8];     /* TPARAM: the parameter name; UNKNOWN: the first 7 characters of the word */
} LinkK_t;

/* ---- one received frame of any type ----------------------------------------------------- */
typedef struct
{
    LinkV2Type_t type;
    union
    {
        LinkStatus_t s;    /* LINKV2_S */
        LinkE_t      e;
        LinkH_t      h;
        LinkR_t      r;
        LinkT_t      t;
        LinkM_t      m;
        LinkK_t      k;
    } u;
} LinkV2Frame_t;

/* ---- builders: write the complete frame including "\r\n"; return its length or a negative LINK_ERR_* code.
 *      cap must be at least LINKV2_MAX_LEN. LINK_ERR_RANGE if any field is outside its documented range. ---- */
int32_t LinkV2_BuildE(char *out, uint32_t cap, const LinkE_t *f);
int32_t LinkV2_BuildH(char *out, uint32_t cap, const LinkH_t *f);
int32_t LinkV2_BuildR(char *out, uint32_t cap, const LinkR_t *f);
int32_t LinkV2_BuildT(char *out, uint32_t cap, const LinkT_t *f);
int32_t LinkV2_BuildM(char *out, uint32_t cap, const LinkM_t *f);
int32_t LinkV2_BuildK(char *out, uint32_t cap, const LinkK_t *f);   /* cmd LINKCMD_UNKNOWN cannot be built */

/* ---- the parser: one received line ('$' ... optional CR LF) of ANY type, $S included.
 *      LINK_OK and *out filled; LINK_ERR_FORMAT / _CHECKSUM / _RANGE; LINK_ERR_TYPE for a well-formed frame of a type this version
 *      does not know (the receiver counts it as 'unknown' and ignores it). ---- */
int32_t LinkV2_Parse(const char *line, uint32_t len, LinkV2Frame_t *out);

const char *LinkV2_CmdWord(LinkCmd_t cmd);      /* "HELLO", "CALV", ... ; "" for NONE and UNKNOWN */

/* ---- byte-stream receiver, the same behaviour as LinkRx_t: a '$' always restarts a frame, garbage between frames is
 *      ignored, an over-long line is dropped; none of them can stall it. ---- */
typedef struct
{
    char     buf[LINK_MAX_LINE];
    uint32_t len;
    uint8_t  active;
    uint32_t frames_ok;
    uint32_t frames_bad;       /* complete line, rejected: checksum, format or range    */
    uint32_t frames_unknown;   /* good checksum, a frame type this version does not know */
    uint32_t overflows;        /* line longer than LINK_MAX_LINE                         */
} LinkV2Rx_t;

void    LinkV2Rx_Init(LinkV2Rx_t *r);

/* Feed every received byte. Returns the LinkV2Type_t (> 0) when this byte completed a VALID frame (copied to *out), else 0. */
int32_t LinkV2Rx_Feed(LinkV2Rx_t *r, uint8_t byte, LinkV2Frame_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LINK_V2_H */
