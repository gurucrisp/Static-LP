/**
  ******************************************************************************
  * @file    slave_link.h
  * @brief   Slave side of the master link: sends the status frame (see
  *          link_proto.h) every 10 ms over USART1 (PB6 = TX), without
  *          blocking and without interrupts.
  ******************************************************************************
  */

#ifndef SLAVE_LINK_H
#define SLAVE_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "slave_features.h"

/* v0.9.53: a slave has NO phase letter of its own: the master knows the phase from the UART port the slave is on (R = USART1, Y = USART4, B = USART3),
 * so the three boards of a unit run the same firmware. The phase field of the status frame is always 'R' and the master ignores it on the Y and B ports. */
#ifdef SLAVE_PHASE_ID
#error "SLAVE_PHASE_ID is no longer used (v0.9.53): the master identifies the phase by its UART port. Delete the SLAVE_PHASE_ID line from slave_config.h"
#endif
#define SLAVE_STATUS_PHASE      'R'     /* the fixed phase field of the status frame ($S keeps its format) */

/* One status frame every 10 ms = twice per 50 Hz mains cycle, each with a
 * fresh measurement (MEAS_UPDATE_US in slave_meas.h is the same 10 ms). The
 * master confirms LV/HV/overload over 2-3 mains cycles, so this period, not
 * the baud rate, is what sets how quickly it can react. A frame is about
 * 1 ms long at 500000 baud, so the link is busy only ~10 % of the time. */
#define SLAVE_LINK_PERIOD_US    10000UL

typedef struct
{
    volatile uint32_t frames_sent;      /* frames queued for transmission          */
    volatile uint32_t frames_dropped;   /* skipped because the TX queue was full   */
    volatile uint32_t bytes_sent;       /* bytes handed to the UART                */
    volatile uint32_t rx_overruns;      /* v2: the UART lost a byte before the interrupt read it (a frame is then dropped by its checksum) */
    volatile uint32_t rx_dropped;       /* v2: the receive ring was full                                                                    */
} SlaveLinkStats_t;

extern SlaveLinkStats_t g_link;

/* Link version 2 (SLAVE_LINK_V2 1): the receive interrupt (USART1, IRQ_PRIO_LINK) stores bytes in a ring that SlaveLink_Service() drains. Called by USART1_IRQHandler only. */
void SlaveLink_RxIrq(void);

/* Call once, after SlaveMeas_Init() and after USART1 is initialised (LINK_BAUDRATE, 8N1). */
void SlaveLink_Init(void);

/* Call as often as possible from the main loop. It feeds the UART (one byte
 * every 20 us at 500000 baud) and, every 10 ms, builds a new status frame from
 * the latest measurements. If the loop is busy elsewhere for a while the
 * frame is simply stretched - the receiver does not mind gaps inside a
 * frame, and the checksum covers it. */
void SlaveLink_Service(void);

#ifdef __cplusplus
}
#endif

#endif /* SLAVE_LINK_H */
