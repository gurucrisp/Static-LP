/** @file master_iwdg.c  @brief Independent watchdog by direct register access. See master_iwdg.h. */
#include "main.h"
#include "master_iwdg.h"

uint8_t MasterIwdg_Calc(uint32_t timeout_ms, uint8_t *pr, uint16_t *rlr)
{
    uint8_t code;
    if (timeout_ms == 0UL) return 1U;
    for (code = 0U; code <= 6U; code++)
    {
        uint32_t div = 4UL << code, ticks = (uint32_t)(((uint64_t)MASTER_IWDG_LSI_HZ * timeout_ms) / (1000ULL * div));
        if ((ticks >= 1UL) && (ticks <= MASTER_IWDG_MAX_RLR)) { *pr = code; *rlr = (uint16_t)ticks; return 0U; }
    }
    return 1U;
}

uint8_t MasterIwdg_Start(uint32_t timeout_ms)
{
    uint8_t pr = 0U; uint16_t rlr = 0U; uint32_t guard;
    if (MasterIwdg_Calc(timeout_ms, &pr, &rlr) != 0U) return 1U;
#ifdef DBG_APB_FZ1_DBG_IWDG_STOP
    DBG->APBFZ1 |= DBG_APB_FZ1_DBG_IWDG_STOP;                         /* a debugger halt does not reset the unit */
#endif
    IWDG->KR = 0x0000CCCCUL;                                           /* start (the LSI is switched on by it) */
    IWDG->KR = 0x00005555UL;                                           /* unlock PR and RLR */
    IWDG->PR = pr; IWDG->RLR = rlr;
    for (guard = 0UL; (IWDG->SR != 0UL) && (guard < 2000000UL); guard++) { }      /* wait until the values are in (a few LSI cycles) */
    if (IWDG->SR != 0UL) return 1U;
    IWDG->KR = 0x0000AAAAUL;                                           /* load the counter */
    return 0U;
}

void MasterIwdg_Kick(void) { IWDG->KR = 0x0000AAAAUL; }
