/**
  ******************************************************************************
  * @file    link_proto.h
  * @brief   Master <-> slave link protocol, version 1 (status frame).
  *          SHARED SOURCE: the slave and the master compile this same file, so
  *          the two ends cannot disagree about the format. No hardware access,
  *          no HAL, no printf: plain C that is also tested on a PC.
  *
  * PHYSICAL LINK:  UART, 500000 baud, 8 data bits, no parity, 1 stop bit,
  *                 plain TX/RX (no direction-enable pin). One slave per link.
  *                 Both ends use TLP2361 optocouplers (specified for 20 Mbaud).
  *
  *   WHY 500000: it divides BOTH 64 MHz (slave crystal+PLL: BRR 128) and
  *   16 MHz (master internal oscillator: BRR 32) exactly, so the baud rate has
  *   zero rounding error whichever clock the master runs on. 921600, the usual
  *   "fast" rate, would be 2.1 % off on a 16 MHz master before any oscillator
  *   drift. A 50-byte status frame takes 1.0 ms on the wire (4.3 ms at 115200).
  *   LINK_BAUDRATE below is the ONE place this is set for both ends.
  *
  * STATUS FRAME (slave -> master), plain ASCII, human-readable on a terminal:
  *
  *     $S,<ph>,<seq>,<vin>,<vout>,<freq>,<state>,<flags>,<fault>,<ntc>[,<diag>[,<diag2>]]*<CS>\r\n
  *
  *   ph     phase this slave regulates: R, Y or B
  *   seq    frame counter 0..255 (wraps) - the master can see lost frames
  *   vin    input  voltage, RMS, tenths of a volt   (2301 = 230.1 V)
  *   vout   output voltage, RMS, tenths of a volt
  *   freq   mains frequency, hundredths of a hertz  (5000 = 50.00 Hz), 0 = unknown
  *   state  slave state: 0 stopped, 1 starting, 2 running, 3 fault (reserved: 0 for now)
  *   flags  bit field, see LINK_FLAG_* below
  *   fault  fault code, 0 = none (reserved: 0 for now)
  *   ntc    raw 12-bit ADC reading of the heat-sink NTC (conversion to degrees
  *          is done by the master, so it can be tuned without touching the slave)
  *   CS     checksum: XOR of every byte between '$' and '*' (exclusive),
  *          written as two upper-case hex digits
  *
  * Example:  $S,R,12,2301,2298,5002,0,19,0,1871*2F\r\n
  *
  * The field layout is FROZEN for version 1: new information is added by new
  * frame types (a different letter after '$'), never by changing this one.
  * Commands master -> slave (run/stop, setpoint, calibration) will use their
  * own frame type and are not part of this step.
  ******************************************************************************
  */

#ifndef LINK_PROTO_H
#define LINK_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define LINK_BAUDRATE        500000UL
#define LINK_MAX_LINE        96U        /* longest frame we accept, '$' to '\n' */
#define LINK_STATUS_MAX_LEN  80U        /* a built status frame never exceeds this */

/* flags field */
#define LINK_FLAG_CALIBRATED  0x0001U   /* BOTH voltage channels calibrated (not nominal) */
#define LINK_FLAG_FREQ_VALID  0x0002U   /* freq comes from a valid zero-cross rate     */
#define LINK_FLAG_CLIP_IN     0x0004U   /* AC_IN  ADC range was hit (signal too large) */
#define LINK_FLAG_CLIP_OUT    0x0008U   /* AC_OUT ADC range was hit                    */
#define LINK_FLAG_MEAS_VALID  0x0010U   /* the voltage values are real measurements    */

typedef struct
{
    char     phase;      /* 'R', 'Y' or 'B'            */
    uint8_t  seq;        /* 0..255, wraps              */
    uint16_t vin_dv;     /* tenths of a volt           */
    uint16_t vout_dv;    /* tenths of a volt           */
    uint16_t freq_chz;   /* hundredths of a hertz      */
    uint8_t  state;      /* 0..9                       */
    uint16_t flags;      /* LINK_FLAG_*                */
    uint16_t fault;      /* fault code                 */
    uint16_t ntc_raw;    /* 0..4095                    */
    uint16_t diag;       /* bench diagnostics, 0..65535 (OPTIONAL last field: a frame without it parses with diag = 0).
                          * bits 0-4  FLT (PD2) falling edges seen while NOT running (saturates at 31)
                          * bits 5-9  FLT (PD2) falling edges seen while running
                          * bits 10-12 break (BKIN) events while NOT running (saturates at 7)
                          * bits 13-15 break (BKIN) events while running                                     */
    uint16_t diag2;      /* bench diagnostics, 0..65535, OPTIONAL last field (absent = 0): where in the half-cycle the last running
                          * FLT / break event hit - see SlaveCtrl_TripCapture() in slave_ctrl.h                              */
} LinkStatus_t;

/* Result codes */
#define LINK_OK             0
#define LINK_ERR_FORMAT    (-1)   /* not a well-formed frame                */
#define LINK_ERR_CHECKSUM  (-2)   /* checksum does not match                */
#define LINK_ERR_RANGE     (-3)   /* a number is out of range               */
#define LINK_ERR_TYPE      (-4)   /* well-formed, but not a status frame    */
#define LINK_ERR_SPACE     (-5)   /* output buffer too small                */

/* XOR of n bytes. */
uint8_t Link_Checksum(const char *p, uint32_t n);

/* Writes the complete frame, including "\r\n", into out (no terminating NUL
 * is needed or written beyond the "\n"). Returns the length, or a negative
 * LINK_ERR_* code. cap must be at least LINK_STATUS_MAX_LEN. */
int32_t Link_BuildStatus(char *out, uint32_t cap, const LinkStatus_t *s);

/* Parses one received line ('$' ... optional '\r', with or without a
 * trailing '\n'). Returns LINK_OK and fills *s, or a negative LINK_ERR_*. */
int32_t Link_ParseStatus(const char *line, uint32_t len, LinkStatus_t *s);

/* ---- Byte-stream receiver ---------------------------------------------------
 * Feed it every received byte. It looks for '$', collects up to '\n', and
 * validates the frame. Garbage between frames, a '$' in the middle of a
 * frame (lost tail), over-long lines and bad checksums are all handled and
 * counted; none of them can stall it.                                        */
typedef struct
{
    char     buf[LINK_MAX_LINE];
    uint32_t len;
    uint8_t  active;
    uint32_t frames_ok;
    uint32_t frames_bad;     /* complete line, but rejected (checksum/format/range) */
    uint32_t overflows;      /* line longer than LINK_MAX_LINE                      */
    uint32_t frames_other;   /* well-formed, good checksum, but a frame type this receiver does not read (a v2 frame): NOT counted as bad */
} LinkRx_t;

void    LinkRx_Init(LinkRx_t *r);

/* Returns 1 when this byte completed a VALID status frame (copied to *out),
 * 0 otherwise. */
int32_t LinkRx_Feed(LinkRx_t *r, uint8_t byte, LinkStatus_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LINK_PROTO_H */
