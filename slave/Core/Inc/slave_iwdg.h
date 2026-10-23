/**
  * @file    slave_iwdg.h
  * @brief   Independent watchdog (slave, STM32G0B1), by direct register access (no HAL IWDG module needed).
  *          The IWDG runs from the internal LSI (nominally 32 kHz, the real value is 25 to 35 kHz in the worst case: the timeout is therefore between about 0.75 and 1.3 times the nominal one) and cannot be stopped once started.
  *          Start it AFTER the slow initialisation, kick it once per pass of the main loop ONLY (never from an interrupt: that would defeat it).
  */
#ifndef SLAVE_IWDG_H
#define SLAVE_IWDG_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define SLAVE_IWDG_LSI_HZ   32000UL
#define SLAVE_IWDG_MAX_RLR  4095UL

/* The prescaler code (0..6: divide by 4 << code) and the reload value for a timeout in milliseconds. Returns 0 when the timeout can be made (1 ms to about 32 s), else 1. */
uint8_t SlaveIwdg_Calc(uint32_t timeout_ms, uint8_t *pr, uint16_t *rlr);
/* Start the watchdog with this timeout. 0 = started, 1 = refused (bad timeout) or the registers did not accept the values. */
uint8_t SlaveIwdg_Start(uint32_t timeout_ms);
void    SlaveIwdg_Kick(void);

#ifdef __cplusplus
}
#endif
#endif
