/* a SOFTWARE MODEL of the registers master_rtc.c touches, so that its bit encoding and its start-up sequence can be tested on a PC. It models what the reference manual says,
 * not the silicon: the real part still has to be proven on the bench. HAL_GetTick() drives the hardware's own behaviour (the crystal starting, INITF following INIT, RSF). */
#ifndef MAIN_H_STUB
#define MAIN_H_STUB
#include <stdint.h>
typedef struct { volatile uint32_t APBENR1, BDCR, CSR; } RCC_TypeDef;
typedef struct { volatile uint32_t CR1; } PWR_TypeDef;
typedef struct { volatile uint32_t TR, DR, SSR, ICSR, PRER, WUTR, CR, RES0, RES1, WPR; } RTC_TypeDef;
extern RCC_TypeDef rcc_m; extern PWR_TypeDef pwr_m; extern RTC_TypeDef rtc_m;
#define RCC (&rcc_m)
#define PWR (&pwr_m)
#define RTC (&rtc_m)
uint32_t HAL_GetTick(void);
typedef struct { volatile uint32_t BKP0R, BKP1R, BKP2R, BKP3R, BKP4R; } TAMP_TypeDef;
extern TAMP_TypeDef tamp_model;
#define TAMP (&tamp_model)
#endif
