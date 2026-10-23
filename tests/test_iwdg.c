/* Tests for the independent-watchdog drivers (slave and master): the timeout arithmetic and the register sequence, against a model of the IWDG registers. */
#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "slave_iwdg.h"
#include "master_iwdg.h"
static int tot = 0, bad = 0;
#define CHECK(c, ...) do { tot++; if (!(c)) { bad++; printf("      FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
IWDG_TypeDef iwdg_model;
int main(void)
{
    uint8_t pr; uint16_t rlr; uint32_t t;
    /* the arithmetic: LSI 32 kHz, divider 4 << pr, reload up to 4095: the smallest divider that fits */
    CHECK(SlaveIwdg_Calc(250U, &pr, &rlr) == 0U && pr == 0U && rlr == 2000U, "250 ms: divide by 4, reload 2000 (pr %u, rlr %u)", pr, rlr);
    CHECK(SlaveIwdg_Calc(500U, &pr, &rlr) == 0U && pr == 0U && rlr == 4000U, "500 ms: still divide by 4, reload 4000 (pr %u, rlr %u)", pr, rlr);
    CHECK(SlaveIwdg_Calc(512U, &pr, &rlr) == 0U && pr == 1U && rlr == 2048U, "512 ms needs 4096 ticks at divide by 4: it moves to divide by 8, reload 2048 (pr %u, rlr %u)", pr, rlr);
    CHECK(MasterIwdg_Calc(1000U, &pr, &rlr) == 0U && pr == 1U && rlr == 4000U, "1000 ms: divide by 8, reload 4000 (pr %u, rlr %u)", pr, rlr);
    CHECK(MasterIwdg_Calc(2000U, &pr, &rlr) == 0U && pr == 2U && rlr == 4000U, "2000 ms: divide by 16, reload 4000 (pr %u, rlr %u)", pr, rlr);
    CHECK(MasterIwdg_Calc(1U, &pr, &rlr) == 0U && pr == 0U && rlr == 8U, "1 ms: divide by 4, reload 8 (pr %u, rlr %u)", pr, rlr);
    CHECK(SlaveIwdg_Calc(0U, &pr, &rlr) != 0U, "0 ms is refused");
    CHECK(SlaveIwdg_Calc(9000U, &pr, &rlr) == 0U && pr == 5U && rlr == 2250U, "9 s: divide by 128 (divide by 64 would need 4500 ticks), reload 2250 (pr %u, rlr %u)", pr, rlr);
    CHECK(SlaveIwdg_Calc(40000U, &pr, &rlr) != 0U, "40 s is refused (the longest the IWDG can do is about 32.7 s)");
    for (t = 1U; t <= 32000U; t += 37U)
    {   /* every timeout that is accepted is made to within 1 reload tick, never above 4095, and never zero */
        if (MasterIwdg_Calc(t, &pr, &rlr) == 0U)
        {
            double real_ms = (double)rlr * (double)(4U << pr) * 1000.0 / 32000.0;
            CHECK(rlr >= 1U && rlr <= 4095U && real_ms <= (double)t + 0.001 && real_ms >= (double)t - (double)(4U << pr) / 32.0 - 0.001, "%u ms: reload %u at divider %u gives %.3f ms", (unsigned)t, rlr, 4U << pr, real_ms);
        }
        else CHECK(0, "%u ms was refused", (unsigned)t);
    }
    /* the register sequence: start (0xCCCC), unlock (0x5555), PR, RLR, wait for SR = 0, load the counter (0xAAAA) */
    iwdg_model.KR = 0U; iwdg_model.PR = 99U; iwdg_model.RLR = 99U; iwdg_model.SR = 0U;
    CHECK(SlaveIwdg_Start(250U) == 0U && iwdg_model.PR == 0U && iwdg_model.RLR == 2000U && iwdg_model.KR == 0xAAAAU, "the slave starts at 250 ms: PR 0, RLR 2000, the last key written is 0xAAAA (PR %u, RLR %u, KR 0x%X)", (unsigned)iwdg_model.PR, (unsigned)iwdg_model.RLR, (unsigned)iwdg_model.KR);
    iwdg_model.KR = 0U; SlaveIwdg_Kick(); CHECK(iwdg_model.KR == 0xAAAAU, "a kick writes 0xAAAA");
    iwdg_model.KR = 0U; iwdg_model.SR = 0U; CHECK(MasterIwdg_Start(1000U) == 0U && iwdg_model.PR == 1U && iwdg_model.RLR == 4000U, "the master starts at 1000 ms: PR 1, RLR 4000");
    /* the registers never accept the values (SR stays busy): refused, not hung */
    iwdg_model.SR = 1U; iwdg_model.KR = 0U; CHECK(SlaveIwdg_Start(250U) != 0U && iwdg_model.KR != 0xAAAAU, "SR busy for ever: Start gives up and does not load the counter");
    iwdg_model.SR = 0U; CHECK(SlaveIwdg_Start(0U) != 0U && MasterIwdg_Start(40000U) != 0U, "a bad timeout is refused by Start as well");
    printf("  watchdog drivers: the timeout arithmetic for every accepted value, the register sequence, a busy register does not hang the start\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", tot, bad, bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
