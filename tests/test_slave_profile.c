/* Tests for the profile store: round trips, a power cut at every point of a save, damaged copies, a record that is intact but not a valid profile. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "slave_profile.h"
static int tot = 0, bad = 0;
#define CHECK(c, ...) do { tot++; if (!(c)) { bad++; printf("      FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define PA 0x1000UL
#define PB 0x1800UL
static uint8_t mem[4096]; static long budget = -1;      /* bytes that may still be programmed before the "power cut"; -1 = unlimited */
static int rd(uint32_t a, uint8_t *b, uint32_t n) { memcpy(b, &mem[a - PA], n); return 0; }
static int er(uint32_t a) { if (budget == 0) return -1; memset(&mem[a - PA], 0xFF, 2048U); return 0; }
static int pr(uint32_t a, const uint8_t *b, uint32_t n)
{   /* programs byte by byte until the budget is used up: a cut leaves a half-written record; only bits that are 1 can be cleared, as in the real part */
    uint32_t i; for (i = 0U; i < n; i++) { if (budget == 0) return -1; mem[a - PA + i] &= b[i]; if (budget > 0) budget--; } return 0;
}
static const SFlash_t FL = { rd, pr, er };

static TuneProfile_t make(uint16_t cap, uint16_t kib, uint16_t seed)
{
    TuneProfile_t p; uint8_t i; memset(&p, 0, sizeof p);
    p.version = TUNE_VERSION; p.n = 8U; p.knee = 0U; p.vin_ref_dv = 2301U; p.maxc_dv = 600U; p.cap_ticks = cap; p.kib_m = kib; p.kiu_m = kib;
    for (i = 0U; i < 8U; i++) { p.ticks[i] = (uint16_t)(16U << i); p.dv_dv[i] = (int16_t)(seed + 12U * i * i); }
    return p;
}
static int same(const TuneProfile_t *a, const TuneProfile_t *b) { return a->n == b->n && a->cap_ticks == b->cap_ticks && a->kib_m == b->kib_m && a->kiu_m == b->kiu_m && a->vin_ref_dv == b->vin_ref_dv && a->maxc_dv == b->maxc_dv && memcmp(a->ticks, b->ticks, sizeof a->ticks) == 0 && memcmp(a->dv_dv, b->dv_dv, sizeof a->dv_dv) == 0; }
static void blank(void) { memset(mem, 0xFF, sizeof mem); budget = -1; SlaveProfile_Init(&FL, PA, PB); }

int main(void)
{
    TuneProfile_t a = make(1600U, 1000U, 5U), b = make(1200U, 640U, 9U), g; long cut;
    /* an empty flash: not tuned */
    blank(); CHECK(SlaveProfile_Tuned() == 0U && SlaveProfile_Get(&g) == 0U, "empty flash: not tuned, nothing to get");
    /* a save and a power cycle */
    CHECK(SlaveProfile_Save(&a) == 0 && SlaveProfile_Tuned() == 1U && SlaveProfile_Get(&g) == 1U && same(&g, &a), "a save: tuned at once, the profile comes back identical");
    SlaveProfile_Init(&FL, PA, PB); CHECK(SlaveProfile_Tuned() == 1U && SlaveProfile_Get(&g) == 1U && same(&g, &a), "after a power cycle (a fresh load from the flash): still tuned, identical");
    /* an invalid profile is refused and writes nothing */
    { TuneProfile_t bad_p = a; uint8_t snap[4096]; memcpy(snap, mem, sizeof mem); bad_p.cap_ticks = 0U;
      CHECK(SlaveProfile_Save(&bad_p) == -1 && memcmp(snap, mem, sizeof mem) == 0 && SlaveProfile_Get(&g) == 1U && same(&g, &a), "a profile with cap 0: refused, the flash is untouched, the old profile stays");
      bad_p = a; bad_p.kib_m = 3000U; CHECK(SlaveProfile_Save(&bad_p) == -1, "a gain scale of 3.0: refused"); }
    /* a re-tune replaces it; the second save goes to the other page */
    CHECK(SlaveProfile_Save(&b) == 0 && SlaveProfile_Get(&g) == 1U && same(&g, &b), "a re-tune replaces the profile");
    SlaveProfile_Init(&FL, PA, PB); CHECK(SlaveProfile_Get(&g) == 1U && same(&g, &b), "and after a power cycle the NEW one is loaded");
    /* a damaged newest copy: the older one is used */
    blank(); (void)SlaveProfile_Save(&a); (void)SlaveProfile_Save(&b);
    {   /* the second save went to the other page: damage that page, the first profile must still be there */
        mem[PB - PA + 20] ^= 0xFFU; SlaveProfile_Init(&FL, PA, PB);
        CHECK(SlaveProfile_Tuned() == 1U && SlaveProfile_Get(&g) == 1U && same(&g, &a), "the newest copy damaged: the older copy (the first profile) is used"); }
    blank(); (void)SlaveProfile_Save(&a); { int k; for (k = 0; k < 4096; k++) if (mem[k] != 0xFFU) { mem[k] ^= 0x01U; break; } } SlaveProfile_Init(&FL, PA, PB);
    CHECK(SlaveProfile_Tuned() == 0U, "the only copy damaged: NOT tuned (it tunes again), never a wrong profile");
    /* a power cut at every 8 bytes of the FIRST save: not tuned, or tuned with the exact profile */
    for (cut = 0; cut <= 120; cut += 4)
    {
        blank(); budget = cut; (void)SlaveProfile_Save(&a); budget = -1; SlaveProfile_Init(&FL, PA, PB);
        CHECK(SlaveProfile_Tuned() == 0U || (SlaveProfile_Get(&g) == 1U && same(&g, &a)), "the first save cut after %ld bytes: not tuned or exactly the profile", cut);
    }
    /* a power cut at every 4 bytes of a RE-TUNE: the old profile or the new one, never anything else, never lost */
    for (cut = 0; cut <= 140; cut += 4)
    {
        blank(); (void)SlaveProfile_Save(&a); budget = cut; (void)SlaveProfile_Save(&b); budget = -1; SlaveProfile_Init(&FL, PA, PB);
        CHECK(SlaveProfile_Tuned() == 1U && SlaveProfile_Get(&g) == 1U && (same(&g, &a) || same(&g, &b)), "a re-tune cut after %ld bytes: the old profile or the new one, tuned either way", cut);
    }
    /* a flash that fails: the previous profile stays active in RAM too */
    blank(); (void)SlaveProfile_Save(&a); budget = 0; CHECK(SlaveProfile_Save(&b) > 0 && SlaveProfile_Get(&g) == 1U && same(&g, &a), "the flash refuses a save: an error code, the old profile stays active"); budget = -1;
    /* a record with a good CRC but not a valid profile (a future version, or damage inside the profile's own checks) is not used */
    { SStore_t st; uint8_t buf[SPROFILE_PAYLOAD]; memset(buf, 0, sizeof buf); buf[0] = 9U; buf[1] = 8U; blank(); SStore_Init(&st, &FL, PA, PB, SPROFILE_VERSION, SPROFILE_PAYLOAD); CHECK(SStore_Save(&st, buf) == 0, "a record is stored directly: an unknown profile version");
      SlaveProfile_Init(&FL, PA, PB); CHECK(SlaveProfile_Tuned() == 0U, "an intact record that is not a valid profile: NOT tuned"); }
    CHECK(TUNE_BYTES(TUNE_MAX_POINTS) <= SPROFILE_PAYLOAD, "the largest profile (%u bytes) fits the payload (%u)", (unsigned)TUNE_BYTES(TUNE_MAX_POINTS), (unsigned)SPROFILE_PAYLOAD);
    printf("  profile store: two copies, round trips, invalid profiles refused, damaged copies, a power cut at every point of a save and of a re-tune\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", tot, bad, bad ? "FAILURES" : "ALL PASSED");
    return bad ? 1 : 0;
}
