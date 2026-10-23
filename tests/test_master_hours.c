/* Tests for the running-hours counter: round trips, every register corrupted, and a power cut between ANY two register writes of a save. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "m_hours.h"
static int tot = 0, bad = 0;
#define CHECK(c, ...) do { tot++; if (!(c)) { bad++; printf("      FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
static uint32_t reg[5]; static int writes_allowed = 1000000, writes_done = 0;
static uint32_t rd(uint8_t r) { return reg[r]; }
static void wr(uint8_t r, uint32_t v) { if (writes_done >= writes_allowed) return; reg[r] = v; writes_done++; }       /* after the power cut nothing is written */
static const MHoursIo_t IO = { rd, wr };
static void clear(void) { memset(reg, 0xA5, sizeof reg); writes_allowed = 1000000; writes_done = 0; }
int main(void)
{
    uint32_t k, m; int cut;
    clear(); CHECK(MHours_Load(&IO) == 0U, "garbage registers (no battery, first start): 0");
    memset(reg, 0, sizeof reg); CHECK(MHours_Load(&IO) == 0U, "registers all zero: 0");
    memset(reg, 0xFF, sizeof reg); CHECK(MHours_Load(&IO) == 0U, "registers all ones: 0");
    clear(); MHours_Save(&IO, 12345U); CHECK(MHours_Load(&IO) == 12345U, "a saved value comes back (12345)");
    MHours_Save(&IO, 0U); CHECK(MHours_Load(&IO) == 0U && reg[4] == MHOURS_MAGIC, "0 is a valid value and the magic stays");
    MHours_Save(&IO, 4000000000UL); CHECK(MHours_Load(&IO) == 4000000000UL, "a very large value (4e9 minutes) is stored as it is");
    for (m = 0U; m < 100000U; m += 977U) { MHours_Save(&IO, m); CHECK(MHours_Load(&IO) == m, "round trip %u", (unsigned)m); }
    /* one register damaged: the other copy is used */
    for (k = 0U; k < 4U; k++) { clear(); MHours_Save(&IO, 777U); reg[k] ^= 0x00010000UL; CHECK(MHours_Load(&IO) == 777U, "register %u damaged: the other copy gives 777", (unsigned)k); }
    clear(); MHours_Save(&IO, 777U); reg[4] ^= 1UL; CHECK(MHours_Load(&IO) == 0U, "the magic damaged: nothing is trusted");
    clear(); MHours_Save(&IO, 777U); reg[0] ^= 1UL; reg[2] ^= 1UL; CHECK(MHours_Load(&IO) == 0U, "both copies damaged: 0");
    /* a power cut after ANY number of writes of a save of 5001 over 5000: the result is 5000 or 5001, never anything else and never 0 */
    for (cut = 0; cut <= 5; cut++)
    {
        uint32_t got;
        clear(); MHours_Save(&IO, 5000U); writes_done = 0; writes_allowed = cut; MHours_Save(&IO, 5001U); got = MHours_Load(&IO);
        CHECK(got == 5000U || got == 5001U, "power cut after %d writes of the second save: %u (5000 or 5001)", cut, (unsigned)got);
    }
    /* the very first save, cut at every point: 0 or the value, never garbage */
    for (cut = 0; cut <= 6; cut++)
    {
        uint32_t got;
        clear(); writes_allowed = cut; MHours_Save(&IO, 4242U); got = MHours_Load(&IO);
        CHECK(got == 0U || got == 4242U, "the first save cut after %d writes: %u (0 or 4242)", cut, (unsigned)got);
    }
    /* a long run of saves with random cuts: after EVERY save, the stored value is the new one or exactly what was stored before it (never older, never garbage, never ahead): at most one save is ever lost */
    { uint32_t seed = 99U, v = 1000U; int i, ok = 1, first_bad = -1; clear(); MHours_Save(&IO, v);
      for (i = 0; i < 20000; i++)
      {
          uint32_t before = MHours_Load(&IO), got; seed = seed * 1664525U + 1013904223U; v++; writes_done = 0; writes_allowed = (int)((seed >> 16) % 8U); MHours_Save(&IO, v); writes_allowed = 1000000; got = MHours_Load(&IO);
          if (got != v && got != before) { ok = 0; if (first_bad < 0) first_bad = i; }
      }
      CHECK(ok, "20000 saves, each cut at a random point: after every one the stored value is the new or the previous one (first violation at %d)", first_bad); }
    /* the case that the first version got wrong: the previous save was cut between copy A and copy B, and THIS save is cut in the middle of its first write */
    { int cut2; for (cut2 = 0; cut2 <= 4; cut2++) { uint32_t got; clear(); MHours_Save(&IO, 1044U); writes_done = 0; writes_allowed = 2; MHours_Save(&IO, 1045U); writes_allowed = 1000000; writes_done = 0; writes_allowed = cut2; MHours_Save(&IO, 1046U); writes_allowed = 1000000; got = MHours_Load(&IO);
        CHECK(got == 1045U || got == 1046U, "two cuts in a row (A complete, B stale, then %d writes of the next save): %u, never back to 1044", cut2, (unsigned)got); } }
    printf("  running-hours counter: two redundant copies in backup registers, every register damaged, a power cut after any write, 20000 random cuts\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", tot, bad, bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
