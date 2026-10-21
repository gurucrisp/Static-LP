/**
  ******************************************************************************
  * @file    master_monitor.h
  * @brief   MASTER BRING-UP MONITOR: receives the new slave status frame
  *          (link_proto.h) on USART1 (the R link, PC4 TX / PC5 RX) and shows
  *          input voltage, output voltage, mains frequency and link health on
  *          the 20x4 LCD.
  *
  * WHY A SEPARATE MODE: the existing master firmware waits at start-up for the
  * OLD slave protocol ("[V]" / "[R]..." frames and the "[P]/[D]/[K]" handshake)
  * and shows "LCD Power Failure / Restart the System" until it arrives. The new
  * slave speaks the new protocol, so the old start-up could never get past that
  * screen. In monitor mode main() hands control to MasterMonitor_Run() right
  * after the LCD is initialised, BEFORE any of the old logic or its timers
  * start, so nothing in the old firmware can interfere. The contactor outputs
  * are driven to their OFF level and never touched.
  *
  * MASTER_MONITOR_MODE
  *   1 = monitor (this file owns USART1 RX and the LCD; baud = LINK_BAUDRATE)
  *   0 = the original master firmware, unchanged (USART1 at 115200 for the old
  *       slave protocol)
  *
  * RECEIVE PATH: USART1 RX -> circular DMA (256 bytes, set up here) -> polled
  * by the main loop -> LinkRx_Feed(). The old idle-line scheme (normal-mode DMA
  * restarted on every idle event) is NOT used: at 500000 baud a byte arrives
  * every 20 us and a restart can lose one.
  *
  * LCD: the old LCD driver waits ~1 ms per enable pulse, so refreshing the
  * screen with it would block the receiver for tens of milliseconds. The
  * monitor writes the LCD with its own fast routine, ONE character per loop
  * pass, and only characters that changed.
  ******************************************************************************
  */

#ifndef MASTER_MONITOR_H
#define MASTER_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"
#include "link_proto.h"

#define MASTER_MONITOR_MODE     1

#define MON_LINK_TIMEOUT_MS     500U      /* no valid frame for this long = NO LINK */
#define MON_RXBUF_SIZE          256U      /* DMA ring, power of two (12.8 ms at 500000 baud) */
#define MON_LCD_COLS            20U
#define MON_LCD_ROWS            4U
#define MON_COMPOSE_PERIOD_MS   100U

typedef struct
{
    LinkStatus_t last;               /* the latest valid status frame                 */
    uint8_t      have_frame;         /* at least one valid frame has arrived          */
    uint8_t      have_seq;
    uint8_t      last_seq;
    uint32_t     t_last_ms;          /* HAL_GetTick() when the last valid frame came  */

    uint32_t     frames_ok;          /* valid frames                                  */
    uint32_t     frames_bad;         /* complete lines rejected (checksum/format)     */
    uint32_t     overflows;          /* over-long lines                               */
    uint32_t     lost;               /* frames missing, from gaps in the seq number   */
    uint32_t     bytes_rx;           /* every byte the DMA delivered                  */
    uint32_t     uart_errors;        /* overrun / framing / noise flags cleared       */
    uint32_t     fps;                /* valid frames in the last whole second         */
    uint32_t     max_poll_gap_us;    /* longest time the loop went without polling    */
} MasterMon_t;

extern MasterMon_t g_mon;

/* Never returns. Call right after LCD_Init() in main(). */
void MasterMonitor_Run(void);

/* ---- Hardware-independent parts (also used by the PC-side tests) ----------- */

void Mon_Reset(void);

/* Feeds the bytes the DMA has written since *rd, up to write index wr, into the
 * frame receiver and updates g_mon. ring_size must be a power of two. */
void Mon_ProcessBytes(const volatile uint8_t *ring, uint32_t ring_size,
                      uint32_t *rd, uint32_t wr, uint32_t now_ms);

/* Rolls the one-second frame counter. Call with the current time often. */
void Mon_Tick(uint32_t now_ms);

/* Builds the four LCD rows (each exactly MON_LCD_COLS characters + NUL). */
void Mon_Compose(const MasterMon_t *m, uint32_t now_ms,
                 char rows[MON_LCD_ROWS][MON_LCD_COLS + 1U]);

/* Changed-characters-only LCD writer. */
typedef void (*MonLcdWriteFn)(uint8_t rs, uint8_t value);   /* rs 0 = command, 1 = data */
typedef struct
{
    char     have[MON_LCD_ROWS][MON_LCD_COLS + 1U];   /* what is on the display        */
    int32_t  cur_row, cur_col;                        /* where the LCD cursor is, -1 = unknown */
    uint32_t cursor;                                  /* next cell to look at          */
} MonLcd_t;

void     MonLcd_Init(MonLcd_t *l);
/* Writes at most one changed character (plus a cursor command if needed).
 * Returns the number of LCD bytes written (0 = display already up to date). */
uint32_t MonLcd_Step(MonLcd_t *l, const char want[MON_LCD_ROWS][MON_LCD_COLS + 1U],
                     MonLcdWriteFn write);

#ifdef __cplusplus
}
#endif

#endif /* MASTER_MONITOR_H */
