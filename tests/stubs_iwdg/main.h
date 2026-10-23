/* A model of the IWDG registers for the PC tests of the watchdog drivers: every write to KR is recorded, SR can be made busy. */
#ifndef MAIN_H_STUB_IWDG
#define MAIN_H_STUB_IWDG
#include <stdint.h>
typedef struct { volatile uint32_t KR, PR, RLR, SR, WINR; } IWDG_TypeDef;
extern IWDG_TypeDef iwdg_model;
#define IWDG (&iwdg_model)
#endif
