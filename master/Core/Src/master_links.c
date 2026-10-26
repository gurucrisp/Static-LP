/**
  ******************************************************************************
  * @file    master_links.c
  * @brief   The Y and B slave links. See master_links.h. No hardware.
  ******************************************************************************
  */
#include <string.h>
#include "master_links.h"

typedef struct
{
    MasterMon_t  mon;                   /* the decoded state of this link (the same structure as the R link's g_mon) */
    LinkRx_t     lrx;                   /* $S receiver   */
    LinkV2Rx_t   l2rx;                  /* $E $H $T $R receiver */
    uint32_t     fps;
    uint32_t     t_tx, t_hello;
    MCtlOut_t    last;                  /* the content of the last $M sent */
    uint8_t      seq_tx, have_last, hello_n, k_id;
} MLink_t;

static MLink_t     s_l[MLINKS_N];
static const char  s_phase[MLINKS_N] = { 'Y', 'B' };

void MLinks_Init(void)
{
    uint8_t i;
    memset(s_l, 0, sizeof s_l);
    for (i = 0U; i < MLINKS_N; i++) { LinkRx_Init(&s_l[i].lrx); LinkV2Rx_Init(&s_l[i].l2rx); }
}

void MLinks_Feed(uint8_t i, const volatile uint8_t *ring, uint32_t ring_size, uint32_t *rd, uint32_t wr, uint32_t now_ms)
{
    if (i >= MLINKS_N) { *rd = wr & (ring_size - 1U); return; }
    Mon_ProcessBytesCtx(&s_l[i].mon, &s_l[i].lrx, &s_l[i].l2rx, &s_l[i].fps, ring, ring_size, rd, wr, now_ms);
}

static uint8_t letter_ok(uint8_t i) { return (uint8_t)((s_l[i].mon.have_frame != 0U) && (s_l[i].mon.last.phase == s_phase[i])); }

uint8_t MLinks_PhaseBad(uint8_t i) { return (uint8_t)((i < MLINKS_N) && (s_l[i].mon.have_frame != 0U) && (s_l[i].mon.last.phase != s_phase[i])); }
const MasterMon_t *MLinks_Mon(uint8_t i) { return &s_l[(i < MLINKS_N) ? i : 0U].mon; }

void MLinks_Fill(uint8_t i, MAppSlave_t *s, uint32_t now_ms)
{
    const MasterMon_t *m; uint8_t fresh, e_fresh;
    memset(s, 0, sizeof *s);
    if (i >= MLINKS_N) return;
    m = &s_l[i].mon;
    if (letter_ok(i) == 0U) return;                                                        /* nothing heard, or the wrong slave on this cable */
    fresh   = (uint8_t)((uint32_t)(now_ms - m->t_last_ms) < MON_LINK_TIMEOUT_MS);
    e_fresh = (uint8_t)((m->e_valid != 0U) && ((uint32_t)(now_ms - m->e_t_ms) < MON_LINK_TIMEOUT_MS));
    s->e_valid = m->e_valid; s->h_valid = m->h_valid; s->clr_echo = m->e_clr;              /* as the R link: what was last heard, to start the clear counter from */
    if (fresh == 0U) return;                                                               /* a silent link: no readings, the engine sees a lost link */
    s->have = 1U; s->link_ok = 1U;
    s->vin_dv = m->last.vin_dv; s->vout_dv = m->last.vout_dv;
    s->state = m->last.state; s->fault = (uint8_t)m->last.fault;
    s->ntc_raw = m->last.ntc_raw; s->ntc_valid = 1U;                                          /* the slave's heat-sink NTC */
    s->in_band = (uint8_t)(e_fresh && ((m->e_flags2 & LINKV2_FLAG_IN_BAND) != 0U));
    s->heat    = (uint8_t)(e_fresh && ((m->e_flags2 & LINKV2_FLAG_OVERHEAT) != 0U));       /* the slave's own overheat decision, when it makes one */
}

int32_t MLinks_BuildM(uint8_t i, const MCtlOut_t *c, uint8_t valid, uint32_t now_ms, char *out, uint32_t cap)
{
    MLink_t *l; LinkM_t m; int32_t n;
    if ((i >= MLINKS_N) || (valid == 0U)) return 0;
    l = &s_l[i];
    if ((l->have_last != 0U) && (c->run == l->last.run) && (c->clr == l->last.clr) && (c->setv_dv == l->last.setv_dv) && (c->maxc_dv == l->last.maxc_dv)
        && ((uint32_t)(now_ms - l->t_tx) < MLINKS_M_PERIOD_MS)) return 0;
    m.seq = l->seq_tx++; m.run = c->run; m.setv_dv = c->setv_dv; m.clr = c->clr; m.maxc_dv = c->maxc_dv;
    n = LinkV2_BuildM(out, cap, &m);
    l->t_tx = now_ms; l->last = *c; l->have_last = 1U;
    return n;
}

uint8_t MLinks_HelloDue(uint8_t i, uint32_t now_ms)
{
    MLink_t *l;
    if (i >= MLINKS_N) return 0U;
    l = &s_l[i];
    if ((l->mon.e_valid == 0U) || (l->mon.h_valid != 0U) || (letter_ok(i) == 0U)) { l->hello_n = 0U; return 0U; }
    if ((l->hello_n != 0U) && ((uint32_t)(now_ms - l->t_hello) < MLINKS_HELLO_MS)) return 0U;
    l->t_hello = now_ms; l->hello_n = 1U; return 1U;
}

int32_t MLinks_BuildHello(uint8_t i, char *out, uint32_t cap)
{
    LinkK_t k;
    if (i >= MLINKS_N) return 0;
    memset(&k, 0, sizeof k);
    k.id = ++s_l[i].k_id; k.cmd = LINKCMD_HELLO; k.num = 0; k.letter = '\0'; k.name[0] = '\0';
    return LinkV2_BuildK(out, cap, &k);
}
