/**
  ******************************************************************************
  * @file    master_links.h
  * @brief   The Y and B slave links of a three-phase unit (v0.9.49).
  *
  * The R slave's link has always been handled in master_monitor.c and is left exactly as it was. This module serves the other two: link 0 is the Y slave
  * (USART4: PC10 = TX, PC11 = RX), link 1 is the B slave (USART3: PB2 = TX, PB0 = RX). It has no hardware in it (the receive rings, the serial ports and the
  * DMA are in master_monitor.c): it decodes the bytes of each link, turns the state of a link into the application's data (MAppSlave_t), and decides when a
  * $M frame or a hello request is due, so all of it is tested on the PC.
  *
  * A status frame carries the phase letter of the slave that sent it ('R', 'Y' or 'B'; the slave's SLAVE_PHASE_ID). A Y or B link whose letter is not the
  * expected one is treated as if NOTHING were heard: a swapped serial cable can never make a contactor follow the wrong slave.
  ******************************************************************************
  */
#ifndef MASTER_LINKS_H
#define MASTER_LINKS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "master_monitor.h"
#include "m_app.h"

#define MLINKS_N            2U      /* 0 = the Y slave, 1 = the B slave */
#define MLINKS_M_PERIOD_MS  100U    /* a $M frame at least this often (and at once when its content changes) */
#define MLINKS_HELLO_MS     2000U   /* a hello request is repeated this often until the slave answers */

void MLinks_Init(void);
/* bytes ring[*rd .. wr) of link i go to its decoders */
void MLinks_Feed(uint8_t i, const volatile uint8_t *ring, uint32_t ring_size, uint32_t *rd, uint32_t wr, uint32_t now_ms);
/* the application's data for link i: all zero (nothing heard) if the link is silent, stale or carries the wrong phase letter */
void MLinks_Fill(uint8_t i, MAppSlave_t *s, uint32_t now_ms);
/* the $M frame for link i if one is due (its content changed, or MLINKS_M_PERIOD_MS passed): returns its length, 0 = nothing to send. valid = MApp_ControlValidPh() */
int32_t MLinks_BuildM(uint8_t i, const MCtlOut_t *c, uint8_t valid, uint32_t now_ms, char *out, uint32_t cap);
/* 1 = ask link i's slave for its hello now ($E heard, no $H yet, not asked within MLINKS_HELLO_MS) */
uint8_t MLinks_HelloDue(uint8_t i, uint32_t now_ms);
int32_t MLinks_BuildHello(uint8_t i, char *out, uint32_t cap);
const MasterMon_t *MLinks_Mon(uint8_t i);           /* the link's counters and last frames (diagnostics) */
uint8_t MLinks_PhaseBad(uint8_t i);                 /* 1 = frames are arriving, but with the wrong phase letter */

#ifdef __cplusplus
}
#endif
#endif /* MASTER_LINKS_H */
