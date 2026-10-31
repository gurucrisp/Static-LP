/* v0.9.79: the brown-out reset decision (slave/Core/Inc/slave_bor.h): set or not, what is written (only the BOR fields, the read protection kept), no second
 * attempt after a reload. gcc -std=c99 -Wall -Wextra -Islave/Core/Inc tests/test_slave_bor.c */
#include <stdio.h>
#include <stdint.h>
#include "slave_bor.h"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

int main(void)
{
    const uint32_t factory = 0xFFFFFEAAUL & ~BOR_OPTR_EN;      /* RDP level 0 (0xAA), BOR off, the other user bits set */
    uint32_t w; unsigned lev;
    CHECK(Bor_IsSet(factory) == 0U && Bor_Decide(factory, 0U) == BOR_PROGRAM, "factory option bytes (BOR off): program it");
    w = Bor_Wanted(factory);
    CHECK(Bor_IsSet(w) == 1U && Bor_Decide(w, 0U) == BOR_OK, "after the write BOR is on at level %u: nothing more to do", (unsigned)SLAVE_BOR_LEVEL);
    CHECK((w & 0xFFUL) == 0xAAUL, "the read protection byte is kept (0x%02X)", (unsigned)(w & 0xFFUL));
    CHECK((w & ~BOR_OPTR_MASK) == (factory & ~BOR_OPTR_MASK), "every bit outside the BOR fields is kept (0x%08lX vs 0x%08lX)", (unsigned long)(w & ~BOR_OPTR_MASK), (unsigned long)(factory & ~BOR_OPTR_MASK));
    CHECK(((w >> BOR_OPTR_F_POS) & 3UL) == SLAVE_BOR_LEVEL && ((w >> BOR_OPTR_R_POS) & 3UL) == SLAVE_BOR_LEVEL && (w & BOR_OPTR_EN) != 0UL, "BOR_EN 1, falling and rising level %u", (unsigned)SLAVE_BOR_LEVEL);
    CHECK(Bor_Decide(factory, 1U) == BOR_GAVE_UP, "still off right after a reload: give up, no second attempt (no reset loop)");
    CHECK(Bor_Decide(w, 1U) == BOR_OK, "on after the reload: OK");
    for (lev = 0U; lev < 4U; lev++)
    {   /* BOR on but at a lower level: raised; at the level: left alone */
        uint32_t o = (factory & ~BOR_OPTR_MASK) | BOR_OPTR_EN | (lev << BOR_OPTR_F_POS) | (lev << BOR_OPTR_R_POS);
        CHECK(Bor_Decide(o, 0U) == ((lev >= SLAVE_BOR_LEVEL) ? BOR_OK : BOR_PROGRAM), "BOR on at level %u: %s", lev, (lev >= SLAVE_BOR_LEVEL) ? "left alone" : "raised");
    }
    {   /* a falling level at 3 but the rising one lower: not enough */
        uint32_t o = (factory & ~BOR_OPTR_MASK) | BOR_OPTR_EN | (3UL << BOR_OPTR_F_POS) | (1UL << BOR_OPTR_R_POS);
        CHECK(Bor_Decide(o, 0U) == BOR_PROGRAM, "falling level 3 but rising level 1: programmed");
        o = (factory & ~BOR_OPTR_MASK) | (3UL << BOR_OPTR_F_POS) | (3UL << BOR_OPTR_R_POS);
        CHECK(Bor_Decide(o, 0U) == BOR_PROGRAM, "levels 3 but BOR_EN off: programmed");
    }
    {   /* v0.9.80: the real STM32G0B1 positions (ST's header): rising [10:9], falling [12:11]; a falling level 3 with rising 0 (0x1900) is not enough, and the wanted
         * value sets exactly 0x1F00 on top of the rest */
        CHECK(BOR_OPTR_R_POS == 9U && BOR_OPTR_F_POS == 11U && BOR_OPTR_MASK == 0x1F00UL, "the BOR fields sit where ST's header puts them (mask 0x%04lX)", (unsigned long)BOR_OPTR_MASK);
        CHECK(Bor_IsSet(0x1900UL) == 0U && Bor_IsSet(0x0700UL) == 0U && Bor_IsSet(0x1F00UL) == 1U, "falling-only or rising-only at level 3: not enough; both: set");
        CHECK((Bor_Wanted(0UL) & BOR_OPTR_MASK) == 0x1F00UL, "the wanted value is BOR_EN + both levels 3 (0x%04lX)", (unsigned long)(Bor_Wanted(0UL) & BOR_OPTR_MASK));
    }
    {   /* a read-protected chip (RDP level 1, 0xBB): the byte must survive */
        uint32_t o = (factory & ~0xFFUL) | 0xBBUL;
        CHECK((Bor_Wanted(o) & 0xFFUL) == 0xBBUL, "RDP level 1 is kept as it is (0x%02X)", (unsigned)(Bor_Wanted(o) & 0xFFUL));
    }
    printf("  BOR option bytes: decision, only the BOR fields written, read protection kept, no second attempt after a reload\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
