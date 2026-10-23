/** @file slave_profile.c  @brief The transformer profile in flash. See slave_profile.h. */
#include "slave_profile.h"

static SStore_t      s_store;
static TuneProfile_t s_prof;
static uint8_t       s_tuned;

void SlaveProfile_Init(const SFlash_t *fl, uint32_t page_a, uint32_t page_b)
{
    uint8_t buf[SPROFILE_PAYLOAD];
    s_tuned = 0U;
    SStore_Init(&s_store, fl, page_a, page_b, SPROFILE_VERSION, SPROFILE_PAYLOAD);
    if (SStore_Load(&s_store, buf) == 0U) return;                                   /* nothing stored, or both copies damaged */
    if (Tune_Deserialize(buf, SPROFILE_PAYLOAD, &s_prof) != 0U) s_tuned = 1U;       /* a record with a good CRC but a profile that is not valid is NOT used */
}

uint8_t SlaveProfile_Tuned(void) { return s_tuned; }
uint8_t SlaveProfile_Get(TuneProfile_t *p) { if (s_tuned == 0U) return 0U; *p = s_prof; return 1U; }

int SlaveProfile_Save(const TuneProfile_t *p)
{
    uint8_t buf[SPROFILE_PAYLOAD]; uint32_t i, n; int st;
    if (Tune_ProfileValid(p) == 0U) return -1;
    for (i = 0U; i < SPROFILE_PAYLOAD; i++) buf[i] = 0U;
    n = Tune_Serialize(p, buf, SPROFILE_PAYLOAD);
    if (n == 0U) return -1;
    st = SStore_Save(&s_store, buf);
    if (st != 0) return st;                                                           /* the flash failed: the previous profile is still the active one */
    s_prof = *p; s_tuned = 1U;
    return 0;
}
