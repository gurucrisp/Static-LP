/** @file m_hours.c  @brief The running-hours counter in the battery-backed registers. See m_hours.h. */
#include "m_hours.h"

static uint8_t valid_pair(const MHoursIo_t *io, uint8_t r, uint32_t *v)
{
    uint32_t a = io->rd(r), b = io->rd((uint8_t)(r + 1U));
    if ((a ^ MHOURS_XOR) != b) return 0U;
    *v = a; return 1U;
}

uint32_t MHours_Load(const MHoursIo_t *io)
{
    uint32_t va = 0U, vb = 0U; uint8_t oka, okb;
    if (io->rd(4U) != MHOURS_MAGIC) return 0U;
    oka = valid_pair(io, 0U, &va); okb = valid_pair(io, 2U, &vb);
    if ((oka != 0U) && (okb != 0U)) return (va > vb) ? va : vb;     /* a cut between the two copies: the newer one is the larger */
    if (oka != 0U) return va;
    if (okb != 0U) return vb;
    return 0U;
}

/* Save: the copy that is NOT the newest valid one is rewritten FIRST, so a power cut in the middle of a save always leaves the newest valid value intact (writing A first every time would destroy the only good copy when
 * the previous save had been cut between A and B). */
void MHours_Save(const MHoursIo_t *io, uint32_t minutes)
{
    uint32_t va = 0U, vb = 0U; uint8_t oka, okb, first_b;
    if (io->rd(4U) != MHOURS_MAGIC) io->wr(4U, MHOURS_MAGIC);
    oka = valid_pair(io, 0U, &va); okb = valid_pair(io, 2U, &vb);
    first_b = (uint8_t)((oka != 0U) && ((okb == 0U) || (va > vb)));              /* A is the newest valid copy: B goes first */
    if (first_b != 0U) { io->wr(2U, minutes); io->wr(3U, minutes ^ MHOURS_XOR); io->wr(0U, minutes); io->wr(1U, minutes ^ MHOURS_XOR); }
    else               { io->wr(0U, minutes); io->wr(1U, minutes ^ MHOURS_XOR); io->wr(2U, minutes); io->wr(3U, minutes ^ MHOURS_XOR); }
}
