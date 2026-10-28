/* v0.9.66: the FLT cycle-by-cycle limit's decisions (slave/Core/Inc/slave_flt_cbc.h): the hold limit, the edge window, chatter, reset. */
#include <stdio.h>
#include <stdint.h>
#define SLAVE_FLT_CBC 1
#include "slave_flt_cbc.h"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

int main(void)
{
    FltCbc_t c = { 0U, 0U, 0U, 0U, 0U }; uint32_t k; uint8_t r = 0U;
    CHECK(FltCbc_OnPeriod(&c, 1U) == FLT_CBC_RESUME, "not active: a period start just resumes");
    CHECK(FltCbc_OnEdge(&c, 100U) == FLT_CBC_LIMIT && c.active == 1U && c.total == 1U, "an edge: limit");
    CHECK(FltCbc_OnPeriod(&c, 0U) == FLT_CBC_RESUME && c.active == 0U, "released at the next period start: resume");
    /* held: FLT_CBC_MAX_HOLD - 1 period starts hold, the next one is a fault */
    (void)FltCbc_OnEdge(&c, 100U);
    for (k = 0U; k < FLT_CBC_MAX_HOLD - 1U; k++) r = FltCbc_OnPeriod(&c, 1U);
    CHECK(r == FLT_CBC_HOLD && c.active == 1U, "%u period starts still asserted: still held", (unsigned)(FLT_CBC_MAX_HOLD - 1U));
    CHECK(FltCbc_OnPeriod(&c, 1U) == FLT_CBC_FAULT && c.active == 0U, "the %uth: a fault", (unsigned)FLT_CBC_MAX_HOLD);
    /* a new edge while held does NOT restart the hold count */
    c.win_n = 0U; (void)FltCbc_OnEdge(&c, 200U);
    for (k = 0U; k < 10U; k++) (void)FltCbc_OnPeriod(&c, 1U);
    (void)FltCbc_OnEdge(&c, 200U);
    for (k = 0U; k < FLT_CBC_MAX_HOLD - 11U; k++) r = FltCbc_OnPeriod(&c, 1U);
    CHECK(r == FLT_CBC_HOLD, "held 19 periods with a second edge in the middle: still held");
    CHECK(FltCbc_OnPeriod(&c, 1U) == FLT_CBC_FAULT, "and the 20th since the FIRST edge is a fault (chatter cannot keep a hold alive)");
    /* the window: FLT_CBC_MAX_EVENTS edges in 10 ms are limited, the next one trips; a new window starts afresh */
    c.active = 0U; c.win_n = 0U; c.win_ms = 0U;
    for (k = 0U; k < FLT_CBC_MAX_EVENTS; k++) { r = FltCbc_OnEdge(&c, 1000U + (k / 8U)); (void)FltCbc_OnPeriod(&c, 0U); }
    CHECK(r == FLT_CBC_LIMIT, "%u edges within the window: all limited", (unsigned)FLT_CBC_MAX_EVENTS);
    CHECK(FltCbc_OnEdge(&c, 1009U) == FLT_CBC_TRIP && c.active == 0U, "the next one in the same 10 ms: trip");
    CHECK(FltCbc_OnEdge(&c, 1010U) == FLT_CBC_LIMIT && c.win_n == 1U, "10 ms after the window began: a new window, limited again");
    /* isolated edges forever: never a trip */
    { int trips = 0; c.active = 0U; for (k = 0U; k < 5000U; k++) { if (FltCbc_OnEdge(&c, 20000U + k * 5U) == FLT_CBC_TRIP) trips++; (void)FltCbc_OnPeriod(&c, 0U); } CHECK(trips == 0, "5000 edges 5 ms apart: no trip (%d)", trips); }
    (void)FltCbc_OnEdge(&c, 90000U); FltCbc_Reset(&c);
    CHECK(c.active == 0U && c.hold == 0U && FltCbc_OnPeriod(&c, 1U) == FLT_CBC_RESUME, "reset: nothing held");
    printf("  FLT cycle-by-cycle limit: hold limit, a second edge does not extend a hold, the 10 ms window, isolated edges, reset\n");
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
