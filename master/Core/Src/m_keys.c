/** @file m_keys.c  @brief Key handling. See m_keys.h. */
#include "m_keys.h"

void MK_Init(MKeys_t *k)
{
    uint8_t *p = (uint8_t *)k; uint32_t i;
    for (i = 0U; i < (uint32_t)sizeof(*k); i++) p[i] = 0U;
}

MKEvent_t MK_Update(MKeys_t *k, uint8_t menu, uint8_t start, uint8_t stop, uint8_t log, uint32_t now)
{
    const uint8_t in[4] = { (uint8_t)(menu != 0U), (uint8_t)(start != 0U), (uint8_t)(stop != 0U), (uint8_t)(log != 0U) };
    uint8_t i; MKEvent_t ev = MK_NONE;
    uint8_t edge_dn[4] = { 0U, 0U, 0U, 0U }, edge_up[4] = { 0U, 0U, 0U, 0U };

    for (i = 0U; i < 4U; i++)
    {
        if (in[i] != k->raw[i]) { k->raw[i] = in[i]; k->t_change[i] = now; }
        else if ((k->stable[i] != in[i]) && ((uint32_t)(now - k->t_change[i]) >= MK_DEBOUNCE_MS))
        {
            k->stable[i] = in[i];
            if (in[i] != 0U) edge_dn[i] = 1U; else edge_up[i] = 1U;
        }
    }
    if ((k->stable[0] != 0U) && (k->stable[3] != 0U))
    {
        if (k->tainted == 0U) { k->tainted = 1U; k->t_both = now; }
        else if ((k->combo_fired == 0U) && ((uint32_t)(now - k->t_both) >= MK_COMBO_MS)) { k->combo_fired = 1U; return MK_COMBO; }
    }

    if (k->queued != MK_NONE) { ev = k->queued; k->queued = MK_NONE; }

    /* START and STOP: on the press, and never while MENU or LOG is down or a combo touch is still going */
    if ((edge_dn[1] != 0U) && (k->stable[0] == 0U) && (k->stable[3] == 0U) && (k->tainted == 0U)) { if (ev == MK_NONE) ev = MK_START; else k->queued = MK_START; }
    if ((edge_dn[2] != 0U) && (k->stable[0] == 0U) && (k->stable[3] == 0U) && (k->tainted == 0U)) { if (ev == MK_NONE) ev = MK_STOP;  else k->queued = MK_STOP; }

    /* MENU and LOG: short on release, unless a combo touch is involved */
    if ((edge_up[0] != 0U) && (k->tainted == 0U)) { if (ev == MK_NONE) ev = MK_MENU_SHORT; else k->queued = MK_MENU_SHORT; }
    if ((edge_up[3] != 0U) && (k->tainted == 0U)) { if (ev == MK_NONE) ev = MK_LOG_SHORT;  else k->queued = MK_LOG_SHORT; }

    /* re-arm only when everything that formed the combo is up */
    if ((k->tainted != 0U) && (k->stable[0] == 0U) && (k->stable[3] == 0U) && (k->raw[0] == 0U) && (k->raw[3] == 0U)) { k->tainted = 0U; k->combo_fired = 0U; }
    return ev;
}
