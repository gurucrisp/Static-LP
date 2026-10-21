/*
 * uart_receiver.h
 *
 *  Created on: Mar 3, 2026
 *      Author: vasanth
 */
#ifndef INC_UART_RECEIVER_H_
#define INC_UART_RECEIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   // or stm32xxxx_hal.h (main.h is safer for Cube)
#include "display.h"
/* Modes */
#define MODE_SINGLE   0
#define MODE_THREE    1

typedef enum { REQ_NONE=0, REQ_PHASE, REQ_SETTINGS, REQ_PULSEON } hs_req_t;
static volatile hs_req_t hs_req[3] = {REQ_NONE,REQ_NONE,REQ_NONE};

static inline uint8_t GetMode(void)
{
    return (  working_settings.pcb_id == 0) ? MODE_SINGLE : MODE_THREE;
}


#ifdef __cplusplus
}
#endif

#endif /* UART_APP_H */
