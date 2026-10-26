/* The slave's half of the phase sequence measurement (slave_phase.c): arm, SYNC timestamp, the next rising zero-cross, timeouts, and a randomised check against the plain definition. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "slave_phase.h"
static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
static int n_lock, n_unlock;
static void lk(void) { n_lock++; }
static void ul(void) { n_unlock++; }
static uint16_t id, dl, pd;
static void res(void) { SlavePhase_Result(&id, &dl, &pd); }
static uint32_t rnd32(void) { static uint32_t s = 0x2545F491U; s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }

/* a steady 50 Hz mains: rising edges at base + k x 20000 */
static void edges(uint32_t base, uint32_t from_k, uint32_t to_k, uint32_t period) { uint32_t k; for (k = from_k; k <= to_k; k++) { SlavePhase_OnZc(base + k * period, 1U); SlavePhase_OnZc(base + k * period + period / 2U, 0U); } }

int main(void)
{
    uint32_t base = 1000000U;
    SlavePhase_Init(lk, ul);
    res(); CHECK(id == 0U && dl == 0U && pd == 0U, "nothing is reported before any round");
    /* a normal round: ARM, edges before, SYNC at +5 ms after an edge, the next rising edge 15 ms later */
    edges(base, 0U, 3U, 20000U);                                            /* edges at base .. base + 60000: the period is known */
    SlavePhase_Arm(7U, 100U); CHECK(n_lock == 1 && n_unlock == 1, "ARM takes the lock once");
    SlavePhase_OnSync(base + 60000U + 5000U);                              /* 5 ms after the last edge */
    SlavePhase_OnZc(base + 60000U + 10000U, 0U); res(); CHECK(id == 0U, "a FALLING edge does not end the measurement");
    SlavePhase_OnZc(base + 80000U, 1U); res();
    CHECK(id == 7U && dl == 15000U && pd == 20000U, "round 7: SYNC 5 ms after an edge: the next rising edge is 15000 us later, period 20000 (id %u, delay %u, period %u)", (unsigned)id, (unsigned)dl, (unsigned)pd);
    /* a second SYNC and a third edge change nothing */
    SlavePhase_OnSync(base + 90000U); SlavePhase_OnZc(base + 100000U, 1U); res(); CHECK(id == 7U && dl == 15000U, "a SYNC byte that was not asked for is ignored");
    /* not armed: nothing */
    SlavePhase_Init(lk, ul); edges(base, 0U, 2U, 20000U); SlavePhase_OnSync(base + 41000U); SlavePhase_OnZc(base + 60000U, 1U); res(); CHECK(id == 0U, "SYNC without a $K SEQ is ignored");
    /* armed, but the SYNC comes too late: Service disarms at 100 ms */
    SlavePhase_Init(lk, ul); edges(base, 0U, 2U, 20000U); SlavePhase_Arm(3U, 1000U); SlavePhase_Service(1100U); SlavePhase_OnSync(base + 41000U); SlavePhase_OnZc(base + 60000U, 1U); res(); CHECK(id == 0U, "SYNC after the window has closed (100 ms) is ignored, and no result is made up");
    SlavePhase_Init(lk, ul); edges(base, 0U, 2U, 20000U); SlavePhase_Arm(3U, 1000U); SlavePhase_Service(1099U); SlavePhase_OnSync(base + 41000U); SlavePhase_OnZc(base + 60000U, 1U); res(); CHECK(id == 3U && dl == 19000U, "SYNC at 99 ms is taken (delay %u)", (unsigned)dl);
    /* the interrupts ran in the other order: the rising edge belongs BEFORE the SYNC byte's time stamp */
    SlavePhase_Init(lk, ul); edges(base, 0U, 2U, 20000U); SlavePhase_Arm(4U, 1000U); SlavePhase_OnSync(base + 60000U + 30U); SlavePhase_OnZc(base + 60000U, 1U); res();
    CHECK(id == 0U, "an edge time-stamped before the SYNC byte is skipped (not a delay of 4 billion)");
    SlavePhase_OnZc(base + 80000U, 1U); res(); CHECK(id == 4U && dl == 19970U, "the next edge is used: %u", (unsigned)dl);
    /* a SYNC right after an edge: a delay of almost a whole period */
    SlavePhase_Init(lk, ul); edges(base, 0U, 2U, 20000U); SlavePhase_Arm(5U, 1000U); SlavePhase_OnSync(base + 40000U + 1U); SlavePhase_OnZc(base + 60000U, 1U); res(); CHECK(id == 5U && dl == 19999U, "SYNC 1 us after an edge: 19999 us");
    /* the timer wraps between the SYNC and the edge */
    SlavePhase_Init(lk, ul); edges(0xFFFFB000U, 0U, 2U, 20000U); SlavePhase_Arm(6U, 1000U); SlavePhase_OnSync(0xFFFFB000U + 40000U + 3000U); SlavePhase_OnZc(0xFFFFB000U + 60000U, 1U); res();
    CHECK(id == 6U && dl == 17000U && pd == 20000U, "across the 32 bit timer wrap: delay %u, period %u", (unsigned)dl, (unsigned)pd);
    /* no rising edge at all (a zero-cross circuit that gives falling edges only): the round fails after 300 ms */
    SlavePhase_Init(lk, ul); SlavePhase_Arm(9U, 1000U); SlavePhase_OnSync(500000U); { uint32_t k; for (k = 1U; k <= 12U; k++) SlavePhase_OnZc(500000U + k * 10000U, 0U); }
    SlavePhase_Service(1299U); res(); CHECK(id == 0U, "at 299 ms the round is still open"); SlavePhase_Service(1300U); res(); CHECK(id == 9U && dl == SPH_NONE, "at 300 ms with no rising edge: round 9 reported as failed (delay %u)", (unsigned)dl);
    /* the first edge after power-up: the period is not known yet */
    SlavePhase_Init(lk, ul); SlavePhase_Arm(2U, 1000U); SlavePhase_OnSync(500000U); SlavePhase_OnZc(512000U, 1U); res(); CHECK(id == 2U && dl == 12000U && pd == 0U, "no earlier edge: the delay is there, the period is 0 (%u)", (unsigned)pd);
    /* a missed edge shows as a double period: not trusted */
    SlavePhase_Init(lk, ul); SlavePhase_OnZc(100000U, 1U); SlavePhase_OnZc(140000U, 1U); SlavePhase_Arm(2U, 1000U); SlavePhase_OnSync(150000U); SlavePhase_OnZc(160000U, 1U); res(); CHECK(id == 2U && pd == 20000U, "after a missed edge the next period (20000) is good again (%u)", (unsigned)pd);
    SlavePhase_Init(lk, ul); SlavePhase_OnZc(100000U, 1U); SlavePhase_OnZc(140000U, 1U); SlavePhase_Arm(2U, 1000U); SlavePhase_OnSync(141000U); SlavePhase_OnZc(140001U + 19999U, 1U); res(); CHECK(id == 2U, "(sanity)");
    SlavePhase_Init(lk, ul); SlavePhase_OnZc(100000U, 1U); SlavePhase_Arm(2U, 1000U); SlavePhase_OnSync(110000U); SlavePhase_OnZc(150000U, 1U); res(); CHECK(id == 2U && dl == 40000U && pd == 0U, "an edge 50 ms after the previous one: period not trusted (0), delay 40000 still reported (%u/%u)", (unsigned)dl, (unsigned)pd);
    /* a delay that cannot be real */
    SlavePhase_Init(lk, ul); SlavePhase_Arm(2U, 1000U); SlavePhase_OnSync(100000U); SlavePhase_OnZc(100000U + 46000U, 1U); res(); CHECK(id == 2U && dl == SPH_NONE, "a delay of 46 ms is reported as failed");
    /* the previous result stays until the next one replaces it */
    SlavePhase_Init(lk, ul); edges(base, 0U, 2U, 20000U); SlavePhase_Arm(1U, 1000U); SlavePhase_OnSync(base + 41000U); SlavePhase_OnZc(base + 60000U, 1U); res(); CHECK(id == 1U, "round 1 done");
    SlavePhase_Arm(2U, 2000U); res(); CHECK(id == 1U && dl == 19000U, "arming round 2 does not wipe round 1's result (the master pairs by id)");
    SlavePhase_OnSync(base + 90000U); SlavePhase_OnZc(base + 100000U, 1U); res(); CHECK(id == 2U && dl == 10000U, "round 2 replaces it");
    /* the lock is taken by the two main-loop calls only */
    n_lock = 0; n_unlock = 0; SlavePhase_Service(5000U); SlavePhase_Arm(3U, 5000U); { uint16_t a, b, c; SlavePhase_Result(&a, &b, &c); } SlavePhase_OnSync(1U); SlavePhase_OnZc(2U, 1U);
    CHECK(n_lock == 2 && n_unlock == 2, "the lock is taken by Service and Arm only, not by the interrupt calls or the read (%d/%d)", n_lock, n_unlock);
    /* RANDOM: 20000 rounds against the plain definition: delay = the first rising edge at or after the SYNC, minus the SYNC; period = that edge minus the one before */
    { int bad = 0, round_n, mism_p = 0; static const uint32_t PER[3] = { 16667U, 20000U, 25000U };
      for (round_n = 0; round_n < 20000; round_n++)
      {
          uint32_t per = PER[rnd32() % 3U], t0 = rnd32(), k, i; uint32_t ts_off = rnd32() % (3U * per); int64_t E[12]; uint32_t ts; uint32_t expect, exp_per; int found = 0;
          for (i = 0U; i < 12U; i++) E[i] = (int64_t)i * per + (int64_t)(rnd32() % 101U) - 50;                       /* +-50 us of jitter, in order */
          ts = (uint32_t)(t0 + 4U * per + ts_off);
          /* replay in time order: the edges, the SYNC where it falls between two of them (the first edge at or after it ends the measurement) */
          SlavePhase_Init(0, 0); SlavePhase_Arm((uint8_t)(1U + round_n % 255U), 0U);
          expect = 0U; exp_per = 0U;
          for (i = 0U; i < 12U; i++)
          {
              uint32_t te = (uint32_t)(t0 + (uint32_t)E[i]);
              if ((found == 0) && ((int32_t)(te - ts) >= 0)) { if (i == 0U) { found = -1; break; } SlavePhase_OnSync(ts); expect = te - ts; exp_per = (uint32_t)(E[i] - E[i - 1U]); found = 1; }
              SlavePhase_OnZc(te, 1U); if (found == 1) break;
          }
          if (found == 1) { res(); k = (uint32_t)id; if (!(k == (uint32_t)(1U + round_n % 255U) && dl == expect)) bad++; if (pd != exp_per) mism_p++; }
      }
      CHECK(bad == 0, "20000 random rounds (period 16.7, 20 or 25 ms, jitter, any SYNC time): the delay is exactly the plain definition (%d wrong)", bad);
      CHECK(mism_p == 0, "and so is the period (%d wrong)", mism_p); }
    printf("  slave phase: arm window, SYNC time stamp, next rising edge only, wrap, missing edge, failed round, result kept, lock use, 20000 random rounds\n  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
