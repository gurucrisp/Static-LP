/* Tests of what the master tells the slave (m_ctl): run level, set voltage, Max Correction, clear counter. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "m_ctl.h"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

typedef struct { MCtl_t c; MSettings_t s; MCtlOut_t o; uint32_t now; uint8_t setmode, link_ok, fault; uint8_t start, stop; } W_t;

static void world(W_t *w, uint8_t restart_auto)
{
    memset(w, 0, sizeof *w); MS_Defaults(&w->s); w->s.restart_auto = restart_auto; w->s.restart_time = 5U; w->now = 100000U; w->link_ok = 1U; MCtl_Init(&w->c);
}
static void step(W_t *w) { MCtl_Step(&w->c, &w->s, w->setmode, w->start, w->stop, w->link_ok, w->fault, w->now, &w->o); w->start = 0U; w->stop = 0U; w->now += 100U; }
static void run_ms(W_t *w, uint32_t ms) { uint32_t k; for (k = 0U; k < ms / 100U; k++) step(w); }
static void press_start(W_t *w) { w->start = 1U; step(w); }
static void press_stop(W_t *w) { w->stop = 1U; step(w); }

static void t_runlevel(void)
{
    W_t w; uint32_t k, t_run = 0U;
    world(&w, 1U);
    for (k = 0U; k < 100U; k++) { step(&w); if (!t_run && w.o.run) t_run = k * 100U; }
    CHECK(t_run >= 4900U && t_run <= 5100U, "Auto: run goes to 1 after the Restart Time (the power-up on-delay): %u ms", (unsigned)t_run);
    world(&w, 0U); run_ms(&w, 60000U);
    CHECK(w.o.run == 0U, "Manual: run stays 0 for a minute");
    press_start(&w); CHECK(w.o.run == 1U, "START sets run = 1 at once");
    press_stop(&w); CHECK(w.o.run == 0U, "STOP sets run = 0 at once");
    run_ms(&w, 60000U); CHECK(w.o.run == 0U, "and it stays 0, Auto or Manual, until START");
    world(&w, 1U); run_ms(&w, 6000U); press_stop(&w); run_ms(&w, 60000U);
    CHECK(w.o.run == 0U, "Auto: after STOP it does not come back by itself");
    press_start(&w); CHECK(w.o.run == 1U, "START brings it back");
    printf("  run level: Auto on-delay, Manual waits for START, STOP stays off until START\n");
}

static void t_setmode(void)
{
    W_t w;
    world(&w, 1U); run_ms(&w, 8000U); CHECK(w.o.run == 1U, "running");
    w.setmode = 1U; step(&w);
    CHECK(w.o.run == 0U, "Set-Mode opened: run = 0 IN THE SAME STEP (the slave stops its PWM and trips the output)");
    run_ms(&w, 120000U); CHECK(w.o.run == 0U, "and it stays 0 for as long as Set-Mode is open");
    press_start(&w); CHECK(w.o.run == 0U, "START inside Set-Mode does not start it");
    w.setmode = 0U; step(&w); CHECK(w.o.run == 0U, "leaving Set-Mode (Auto): not at once");
    run_ms(&w, 4700U); CHECK(w.o.run == 0U, "not before the Restart Time ...");
    run_ms(&w, 500U); CHECK(w.o.run == 1U, "... then run = 1 again");
    world(&w, 0U); press_start(&w); w.setmode = 1U; step(&w); w.setmode = 0U; run_ms(&w, 60000U);
    CHECK(w.o.run == 0U, "Manual: after Set-Mode it stays stopped until START");
    press_start(&w); CHECK(w.o.run == 1U, "START runs it");
    /* a new Restart Time made in Set-Mode is used straight away */
    world(&w, 1U); run_ms(&w, 8000U); w.setmode = 1U; step(&w); w.s.restart_time = 20U; w.setmode = 0U; run_ms(&w, 19000U);
    CHECK(w.o.run == 0U, "a Restart Time of 20 s set in Set-Mode applies: nothing after 19 s"); run_ms(&w, 1500U); CHECK(w.o.run == 1U, "run after 20 s");
    printf("  Set-Mode: run = 0 in the same step, held for as long as it is open, restart by the Restart Mode\n");
}

static void t_settings_to_slave(void)
{
    W_t w;
    world(&w, 1U); step(&w);
    CHECK(w.o.setv_dv == 2300U && w.o.maxc_dv == 0U, "the default Set Voltage 230 V goes out as 2300, Max Correction 'not set' as 0");
    w.s.set_v = 240U; step(&w); CHECK(w.o.setv_dv == 2400U, "a new Set Voltage follows at once: 2400");
    w.s.set_v = 100U; step(&w); CHECK(w.o.setv_dv == 1000U, "the lowest, 100 V, is 1000"); w.s.set_v = 300U; step(&w); CHECK(w.o.setv_dv == 3000U, "the highest, 300 V, is 3000");
    w.s.maxcorr_v = 25U; step(&w); CHECK(w.o.maxc_dv == 250U, "Max Correction 25 V is 250"); w.s.maxcorr_v = 199U; step(&w); CHECK(w.o.maxc_dv == 1990U, "199 V is 1990: the top of the slave's range");
    printf("  settings to the slave: Set Voltage and Max Correction in tenths of a volt\n");
}

static void t_faults(void)
{
    W_t w; uint8_t c0; int k;
    /* Auto: while the slave reports a fault the counter changes every Restart Time */
    world(&w, 1U); run_ms(&w, 8000U); c0 = w.o.clr; w.fault = 1U;
    step(&w); CHECK(w.o.clr == c0 && w.o.run == 1U, "Auto: a fault does not change the counter at once, run stays 1");
    run_ms(&w, 4800U); CHECK(w.o.clr == c0, "not before the Restart Time");
    run_ms(&w, 400U); CHECK(w.o.clr == (uint8_t)(c0 + 1U), "after the Restart Time the counter has changed once");
    run_ms(&w, 5100U); CHECK(w.o.clr == (uint8_t)(c0 + 2U), "and again each Restart Time while the fault persists (%u)", (unsigned)w.o.clr);
    w.fault = 0U; run_ms(&w, 30000U); CHECK(w.o.clr == (uint8_t)(c0 + 2U) && w.o.run == 1U, "when the fault is gone the counter stops changing");
    /* Manual: stop at the fault; START changes the counter */
    world(&w, 0U); press_start(&w); run_ms(&w, 3000U); c0 = w.o.clr; w.fault = 1U; step(&w);
    CHECK(w.o.run == 0U && w.o.clr == c0, "Manual: a fault stops the unit (run = 0), the counter unchanged");
    run_ms(&w, 120000U); CHECK(w.o.run == 0U && w.o.clr == c0, "and it waits for START, however long");
    press_start(&w); CHECK(w.o.run == 1U && w.o.clr == (uint8_t)(c0 + 1U), "START changes the counter ONCE and runs again");
    w.fault = 0U; run_ms(&w, 20000U); CHECK(w.o.clr == (uint8_t)(c0 + 1U), "and nothing more");
    world(&w, 0U); press_start(&w); press_stop(&w); c0 = w.o.clr; press_start(&w); CHECK(w.o.clr == c0, "START with no fault does not touch the counter");
    /* THE stale report: after START the slave still says "fault" until it has answered the new counter. START must not be cancelled by it. */
    world(&w, 0U); press_start(&w); run_ms(&w, 3000U); w.fault = 1U; step(&w); CHECK(w.o.run == 0U, "Manual: stopped at the fault");
    press_start(&w);
    run_ms(&w, 1200U); CHECK(w.o.run == 1U, "START is NOT cancelled by the slave's old fault report arriving for another 1.2 s");
    w.fault = 0U; run_ms(&w, 10000U); CHECK(w.o.run == 1U, "the slave has cleared: it keeps running");
    world(&w, 0U); press_start(&w); run_ms(&w, 3000U); w.fault = 1U; step(&w); press_start(&w); run_ms(&w, 3000U);
    CHECK(w.o.run == 0U, "but a fault that is STILL there after the settle time stops it again (the cause did not go away)");
    world(&w, 1U); run_ms(&w, 8000U); w.fault = 1U; run_ms(&w, 5300U); c0 = w.o.clr; run_ms(&w, 1000U);
    CHECK(w.o.clr == c0, "Auto: right after a counter change the old report does not cause a second change");
    /* no link: the slave's report cannot be trusted, so nothing is done about it */
    world(&w, 1U); run_ms(&w, 8000U); c0 = w.o.clr; w.link_ok = 0U; w.fault = 1U; run_ms(&w, 60000U);
    CHECK(w.o.clr == c0 && w.o.run == 1U, "with no link the counter is not changed and run is kept");
    /* Auto restart after the on-delay with a fault already latched in the slave */
    world(&w, 1U); w.fault = 1U; c0 = w.o.clr; run_ms(&w, 6000U);
    CHECK(w.o.run == 1U && w.o.clr == (uint8_t)(c0 + 1U), "power-up with the slave latched: after the on-delay run = 1 AND the counter changes (%u)", (unsigned)w.o.clr);
    (void)k;
    printf("  faults: Auto changes the counter every Restart Time, Manual stops and waits for START, no link = hands off\n");
}

static uint32_t rs = 8088U;
static uint32_t rnd(void) { rs = (rs * 1664525U) + 1013904223U; return rs >> 8; }

static void t_fuzz(void)
{
    uint32_t round, k, bad_set = 0U, bad_clr = 0U, bad_val = 0U, steps = 0U;
    for (round = 0U; round < 60U; round++)
    {
        W_t w; uint8_t prev = 0U;
        world(&w, (uint8_t)(rnd() & 1U)); w.s.restart_time = (uint16_t)(1U + rnd() % 20U);
        for (k = 0U; k < 30000U; k++)
        {
            uint32_t ev = rnd() % 300U;
            if (ev == 0U) w.setmode ^= 1U;
            if (ev == 1U) w.start = 1U;
            if (ev == 2U) w.stop = 1U;
            if (ev == 3U) w.fault ^= 1U;
            if (ev == 4U) w.link_ok = (uint8_t)((rnd() % 4U) != 0U);
            if (ev == 5U) { w.s.set_v = (uint16_t)(100U + rnd() % 201U); w.s.maxcorr_v = (uint16_t)(rnd() % 200U); w.s.restart_time = (uint16_t)(1U + rnd() % 30U); }
            step(&w); steps++;
            if (w.setmode && w.o.run) bad_set++;                                         /* never run while Set-Mode is open */
            if ((uint8_t)(w.o.clr - prev) > 1U) bad_clr++;                                /* the counter moves by at most one per step */
            prev = w.o.clr;
            if (w.o.setv_dv != (uint16_t)(w.s.set_v * 10U) || w.o.maxc_dv != (uint16_t)(w.s.maxcorr_v * 10U) || w.o.setv_dv < 1000U || w.o.setv_dv > 3000U || w.o.maxc_dv > 1990U) bad_val++;
        }
    }
    CHECK(bad_set == 0U, "%u random steps: run is never 1 while Set-Mode is open (%u)", (unsigned)steps, (unsigned)bad_set);
    CHECK(bad_clr == 0U, "the clear counter moves by at most one per step (%u)", (unsigned)bad_clr);
    CHECK(bad_val == 0U, "the Set Voltage and Max Correction sent are always inside the slave's accepted ranges (%u)", (unsigned)bad_val);
    printf("  fuzz: %u random steps: never run in Set-Mode, counter steps of one, values always in the slave's range\n", (unsigned)steps);
}

int main(void)
{
    t_runlevel();
    t_setmode();
    t_settings_to_slave();
    t_faults();
    t_fuzz();
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
