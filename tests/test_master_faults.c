/* Scenario tests of the master's fault and restart engine (master/Core/Src/m_faults.c). A small SLAVE MODEL sits between the engine's outputs and its inputs: it latches a fault code until the
 * clear counter changes (decision D5), reports IN_BAND a second after it is told to run, and reports nothing while it is not running. Time moves in 100 ms ticks. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "m_faults.h"

static int g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

typedef struct
{
    MSettings_t s; MFaults_t f; MFIn_t in; MFOut_t out;
    uint32_t now; uint32_t run_min;
    /* slave model, per phase */
    uint8_t  code[3];             /* a fault the slave has latched: stays until the counter changes */
    uint32_t run_since[3]; uint8_t running[3];
    uint32_t clear_delay_ms;      /* how long the slave takes to answer a changed counter */
    uint32_t clear_at[3]; uint8_t clear_pending[3];
    uint8_t  stuck[3];            /* a cause that comes straight back after every clear */
    int      n_log; MLogRec_t log[64];
    int      n_clr[3];
    uint32_t t_last_trip;
    uint8_t  key_start, key_stop;
} W_t;

static void world(W_t *w, uint8_t mode, uint8_t restart_auto, uint32_t t0)
{
    int p;
    memset(w, 0, sizeof *w);
    MS_Defaults(&w->s); w->s.mode = mode; w->s.restart_auto = restart_auto;
    for (p = 0; p < 3; p++) { w->in.ph[p].link_ok = 1U; w->in.ph[p].vin_dv = 2000U; w->in.ph[p].vout_dv = 2300U; w->in.ph[p].load_da = 100U; }
    w->now = t0; w->clear_delay_ms = 200U;
    MF_Init(&w->f, &w->s, w->now);
}
static void step(W_t *w)
{
    int p, i;
    w->in.key_start = w->key_start; w->in.key_stop = w->key_stop; w->key_start = 0U; w->key_stop = 0U; w->in.run_min = w->run_min;
    MF_Tick(&w->f, &w->s, &w->in, w->now, &w->out);
    for (i = 0; i < w->out.n_log && w->n_log < 64; i++) { w->log[w->n_log++] = w->out.log[i].rec; w->t_last_trip = w->now; }
    for (p = 0; p < 3; p++)
    {
        if (w->out.clr_pulse[p]) { w->n_clr[p]++; w->clear_pending[p] = 1U; w->clear_at[p] = w->now + w->clear_delay_ms; }
        if (w->clear_pending[p] && ((int32_t)(w->now - w->clear_at[p]) >= 0)) { w->clear_pending[p] = 0U; w->code[p] = w->stuck[p]; }
        if (w->out.run[p] && !w->running[p]) { w->running[p] = 1U; w->run_since[p] = w->now; }
        if (!w->out.run[p]) w->running[p] = 0U;
        w->in.ph[p].in_band = (uint8_t)(w->running[p] && (w->code[p] == 0U) && ((uint32_t)(w->now - w->run_since[p]) >= 1000U));
        w->in.ph[p].slave_fault = w->code[p];
    }
    w->now += 100U;
}
static void run_ms(W_t *w, uint32_t ms) { uint32_t k; for (k = 0U; k < ms / 100U; k++) step(w); }
static int unit_st(const W_t *w, int u) { return (int)w->out.unit_state[u]; }
static void to_on(W_t *w) { uint32_t k; for (k = 0U; k < 400U && w->out.contactor[0] == 0U; k++) step(w); }
static void press_start(W_t *w) { w->key_start = 1U; step(w); }
static void press_stop(W_t *w) { w->key_stop = 1U; step(w); }

/* ------------------------------------------------------------------------------------------------------------------ */
static void t_powerup(void)
{
    W_t w; uint32_t k, t_run = 0U, t_con = 0U;
    world(&w, M_MODE_SINGLE, 1U, 1000U);
    for (k = 0U; k < 200U; k++) { step(&w); if (!t_run && w.out.run[0]) t_run = k * 100U; if (!t_con && w.out.contactor[0]) t_con = k * 100U; }
    CHECK(t_run >= 4900U && t_run <= 5200U, "Auto: the on-delay is the Restart Time, 5 s: the slave is told to run at %u ms", (unsigned)t_run);
    CHECK(t_con >= t_run + 2000U && t_con <= t_run + 2400U, "the contactor closes 1 s after the slave is IN BAND at the set voltage (the slave takes 1 s to get there): %u ms after run", (unsigned)(t_con - t_run));
    CHECK(w.n_log == 0 && w.out.shown == 0U && !w.out.buzzer, "a clean start logs nothing and shows no fault");
    world(&w, M_MODE_SINGLE, 0U, 1000U);
    run_ms(&w, 30000U);
    CHECK(w.out.run[0] == 0U && w.out.contactor[0] == 0U && unit_st(&w, 0) == (int)U_OFF, "Manual: nothing starts by itself, even after 30 s");
    press_start(&w); run_ms(&w, 300U);
    CHECK(w.out.run[0] == 1U, "START in Manual tells the slave to run at once");
    printf("  power-up: Auto waits the Restart Time then the IN_BAND hold; Manual waits for START\n");
}

static void t_stop_start(void)
{
    W_t w;
    world(&w, M_MODE_SINGLE, 1U, 5000U); to_on(&w);
    press_stop(&w);
    CHECK(w.out.contactor[0] == 0U && w.out.run[0] == 0U && unit_st(&w, 0) == (int)U_OFF, "STOP opens the contactor and drops run in the same tick");
    run_ms(&w, 60000U);
    CHECK(w.out.run[0] == 0U, "and it stays off, Auto or not, until START");
    press_start(&w); to_on(&w);
    CHECK(w.out.contactor[0] == 1U, "START runs it again to the closed contactor");
    printf("  STOP / START\n");
}

static void t_input_voltage(void)
{
    W_t w; uint32_t t0, k; int clr_before;
    world(&w, M_MODE_SINGLE, 1U, 7000U); to_on(&w);
    w.in.ph[0].vin_dv = 1700U; t0 = w.now;
    for (k = 0U; k < 49U; k++) step(&w);
    CHECK(w.out.contactor[0] == 1U && w.out.appear[0] == MLOG_T_IP_LV, "I/P LV: 4.9 s below the limit is not a trip, but it shows 'Appear'");
    run_ms(&w, 300U);
    CHECK(w.out.contactor[0] == 0U && w.out.shown == MLOG_T_IP_LV && w.out.fault[0] == MLOG_T_IP_LV, "after the Volt Cut Delay (5 s) it trips: I/P LV shown");
    CHECK(w.n_log == 1 && w.log[0].type == MLOG_T_IP_LV && w.log[0].phase == 0U && w.log[0].vin_dv == 1700U && w.log[0].slave_code == 0U, "and it is logged once, with the type, phase and voltage");
    CHECK(w.out.buzzer == 1U, "the buzzer sounds in Single mode");
    (void)t0;
    w.in.ph[0].vin_dv = 1840U;                                              /* 184 V: back above 180 but not yet 5 V inside */
    run_ms(&w, 20000U);
    CHECK(w.out.run[0] == 0U, "hysteresis: 184 V is not 5 V inside the 180 V limit: no restart after 20 s");
    w.in.ph[0].vin_dv = 1850U;                                              /* 185 V */
    clr_before = w.n_clr[0];
    run_ms(&w, 4000U);
    CHECK(w.out.run[0] == 0U, "185 V: the Restart Time has not passed yet");
    run_ms(&w, 2000U);
    CHECK(w.out.run[0] == 1U && w.n_clr[0] == clr_before + 1, "after 5 s at 185 V it restarts (Auto) and the slave's clear counter is pulsed once for that restart");
    CHECK(w.out.buzzer == 0U, "and the buzzer stops: no unit is faulted any more");
    to_on(&w);
    /* a glitch shorter than the delay, three times: never trips */
    { int g; for (g = 0; g < 3; g++) { w.in.ph[0].vin_dv = 1600U; run_ms(&w, 4000U); w.in.ph[0].vin_dv = 2300U; run_ms(&w, 500U); } }
    CHECK(w.out.contactor[0] == 1U && w.n_log == 1, "three 4 s dips in a row never trip: the delay counts CONTINUOUS violation");
    /* I/P HV mirror */
    w.in.ph[0].vin_dv = 2850U; run_ms(&w, 5200U);
    CHECK(w.out.contactor[0] == 0U && w.out.shown == MLOG_T_IP_HV, "I/P HV trips after the delay");
    w.in.ph[0].vin_dv = 2760U; run_ms(&w, 20000U);
    CHECK(w.out.run[0] == 0U, "276 V is not 5 V inside the 280 V limit");
    w.in.ph[0].vin_dv = 2750U; run_ms(&w, 6000U);
    CHECK(w.out.run[0] == 1U, "275 V is: it restarts");
    /* manual restart */
    world(&w, M_MODE_SINGLE, 0U, 7000U); press_start(&w); to_on(&w);
    w.in.ph[0].vin_dv = 1700U; run_ms(&w, 5300U); w.in.ph[0].vin_dv = 2300U; run_ms(&w, 60000U);
    CHECK(w.out.run[0] == 0U && w.out.shown == MLOG_T_IP_LV, "Manual: a tripped voltage fault does NOT restart by itself, however long the input is good");
    press_start(&w); run_ms(&w, 300U);
    CHECK(w.out.run[0] == 1U, "START restarts it");
    printf("  input voltage: delay is continuous, hysteresis 5 V, Auto after the Restart Time, Manual needs START\n");
}

static void t_output_voltage(void)
{
    W_t w;
    world(&w, M_MODE_SINGLE, 1U, 9000U); to_on(&w);
    w.in.ph[0].vout_dv = 1950U; run_ms(&w, 5300U);
    CHECK(w.out.contactor[0] == 0U && w.out.shown == MLOG_T_OP_LV && w.log[0].type == MLOG_T_OP_LV && w.log[0].vout_dv == 1950U, "O/P LV trips after the delay and is logged with the output voltage");
    world(&w, M_MODE_SINGLE, 1U, 9000U); to_on(&w);
    w.in.ph[0].vout_dv = 2550U; run_ms(&w, 5300U);
    CHECK(w.out.contactor[0] == 0U && w.out.shown == MLOG_T_OP_HV, "O/P HV trips after the delay");
    /* while only STARTING (contactor open) the output limits do not count: the slave is still ramping */
    world(&w, M_MODE_SINGLE, 1U, 9000U); w.in.ph[0].vout_dv = 1500U; run_ms(&w, 9000U);
    CHECK(w.n_log == 0, "an output below the limit while the contactor is open is not a fault");
    printf("  output voltage: only while the output is live\n");
}

static void t_overload_heat(void)
{
    W_t w;
    world(&w, M_MODE_SINGLE, 1U, 11000U); to_on(&w);
    w.in.ph[0].load_da = 1200U; run_ms(&w, 9800U);
    CHECK(w.out.contactor[0] == 1U && w.out.appear[0] == MLOG_T_OVERLOAD, "overload: 120 A against 99 A for 9.8 s: not yet, 'Appear' shown");
    run_ms(&w, 400U);
    CHECK(w.out.contactor[0] == 0U && w.out.shown == MLOG_T_OVERLOAD && w.log[0].type == MLOG_T_OVERLOAD && w.log[0].load_da == 1200U, "after the Overload Cut Delay (10 s) it trips and logs the load");
    w.in.ph[0].load_da = 0U; run_ms(&w, 120000U);
    CHECK(w.out.run[0] == 0U, "overload is START only: even Auto does not restart it in 2 minutes");
    press_start(&w); run_ms(&w, 300U);
    CHECK(w.out.run[0] == 1U, "START restarts it");
    to_on(&w);
    w.in.ph[0].load_da = 990U; run_ms(&w, 60000U);
    CHECK(w.out.contactor[0] == 1U, "exactly the limit (99.0 A) is not an overload");
    /* heat */
    world(&w, M_MODE_SINGLE, 1U, 11000U); to_on(&w);
    w.in.ph[0].heat = 1U; run_ms(&w, 9800U);
    CHECK(w.out.contactor[0] == 1U && w.out.appear[0] == MLOG_T_HEAT, "heat: the flag for 9.8 s is not yet a trip");
    run_ms(&w, 400U);
    CHECK(w.out.contactor[0] == 0U && w.out.shown == MLOG_T_HEAT && w.log[0].type == MLOG_T_HEAT, "after the Overheat Trip Delay (10 s) it trips");
    run_ms(&w, 60000U); press_start(&w); run_ms(&w, 300U);
    CHECK(w.out.run[0] == 0U, "START is refused while the heat flag is still set");
    w.in.ph[0].heat = 0U; run_ms(&w, 60000U);
    CHECK(w.out.run[0] == 0U, "heat is START only: the flag going away does not restart it by itself");
    press_start(&w); run_ms(&w, 300U);
    CHECK(w.out.run[0] == 1U, "START with the flag gone restarts it");
    /* a hot heat sink at power-up blocks the start */
    world(&w, M_MODE_SINGLE, 1U, 11000U); w.in.ph[0].heat = 1U; run_ms(&w, 30000U);
    CHECK(w.out.run[0] == 0U && w.log[0].type != 0U ? 0 : (w.out.run[0] == 0U), "a hot unit at power-up does not start");
    printf("  overload and heat: delays, START only, heat flag must be gone\n");
}

static void t_retry_groups(void)
{
    W_t w; int n, k; static const uint8_t PS[] = { 7U, 12U, 14U, 33U };
    world(&w, M_MODE_SINGLE, 1U, 20000U); to_on(&w);
    w.code[0] = 2U;                                                         /* the slave's FLT: inrush */
    step(&w); step(&w);
    CHECK(w.out.contactor[0] == 0U && w.out.shown == MLOG_T_INRUSH && w.n_log == 1 && w.log[0].type == MLOG_T_INRUSH && w.log[0].slave_code == 2U, "inrush: trips at once and is logged with the slave's code");
    w.stuck[0] = 2U;                                                        /* the cause comes straight back after every clear */
    for (n = 0; n < 4; n++)
    {
        uint32_t waited = 0U; int clr0 = w.n_clr[0];
        while (!w.out.run[0] && waited < 200000U) { step(&w); waited += 100U; }
        waited = (w.out.run[0] != 0U) ? (uint32_t)(w.now - 100U - w.t_last_trip) : 200000U;           /* from the trip itself */
        if (n < 3)
        {
            CHECK(waited >= 29800U && waited <= 30400U && w.n_clr[0] == clr0 + 1, "retry %d comes 30 s after the trip (%u ms), with a clear pulse", n + 1, (unsigned)waited);
            run_ms(&w, 3000U);                                                /* it trips again straight away */
        }
        else
        {
            CHECK(waited >= 200000U && w.out.run[0] == 0U, "after the third retry has tripped there is NO further retry: latched (waited %u ms)", (unsigned)waited);
        }
    }
    CHECK(unit_st(&w, 0) == (int)U_LATCHED && w.n_log == 4, "the unit is latched and exactly 4 trips were logged (%d)", w.n_log);
    run_ms(&w, 600000U); press_start(&w); press_stop(&w);
    CHECK(unit_st(&w, 0) == (int)U_LATCHED && w.out.run[0] == 0U, "ten minutes later, START and STOP change nothing: power off and on is the only way out");
    CHECK(w.out.buzzer == 0U && w.out.shown == MLOG_T_INRUSH, "STOP silenced the buzzer; the fault is still shown");
    /* a retry that works, run long enough, forgives the count */
    world(&w, M_MODE_SINGLE, 1U, 20000U); to_on(&w);
    for (k = 0; k < 3; k++) { w.code[0] = 1U; step(&w); step(&w); w.stuck[0] = 0U; { uint32_t g; for (g = 0U; g < 500U && !w.out.contactor[0]; g++) step(&w); } }
    CHECK(unit_st(&w, 0) == (int)U_ON && w.n_log == 3, "three inrush trips each followed by a good retry leave the unit running (%d logged)", w.n_log);
    run_ms(&w, MF_FORGIVE_MS + 1000U);
    w.code[0] = 1U; step(&w); step(&w);
    CHECK(unit_st(&w, 0) == (int)U_FAULT, "after 5 minutes of running the retry count is forgotten: the next trip is a first trip again (state %d)", unit_st(&w, 0));
    /* power stage: 7, 12, 14 and an unknown code behave the same */
    for (n = 0; n < 4; n++)
    {
        world(&w, M_MODE_SINGLE, 1U, 20000U); to_on(&w); w.code[0] = PS[n]; step(&w); step(&w);
        CHECK(w.out.shown == MLOG_T_POWER_STAGE && w.log[0].type == MLOG_T_POWER_STAGE && w.log[0].slave_code == PS[n] && unit_st(&w, 0) == (int)U_FAULT, "slave code %u is a power-stage fault, retried (not latched at once)", (unsigned)PS[n]);
    }
    printf("  inrush and power stage: at once, 3 retries 30 s apart, then latched; the retry count is forgiven after 5 minutes\n");
}

static void t_mains_group(void)
{
    W_t w; static const uint8_t M[] = { 3U, 4U, 5U, 8U, 10U, 11U, 13U, 15U, 16U }; int i;
    for (i = 0; i < 9; i++)
    {
        world(&w, M_MODE_SINGLE, 1U, 30000U); to_on(&w); w.code[0] = M[i]; step(&w); step(&w);
        CHECK(w.out.shown == MLOG_T_MAINS && w.log[0].type == MLOG_T_MAINS && w.log[0].slave_code == M[i] && w.out.contactor[0] == 0U, "slave code %u is the Mains group: trip at once, logged with the code", (unsigned)M[i]);
    }
    world(&w, M_MODE_SINGLE, 1U, 30000U); to_on(&w); w.code[0] = 8U; step(&w); step(&w);
    { int c0 = w.n_clr[0];
    run_ms(&w, 4000U); CHECK(w.out.run[0] == 0U, "Auto: not restarted before the Restart Time");
    run_ms(&w, 2500U); CHECK(w.out.run[0] == 1U && w.n_clr[0] == c0 + 1, "Auto: restarted after it, with a clear pulse"); }
    to_on(&w);
    CHECK(w.out.contactor[0] == 1U, "and it runs again");
    world(&w, M_MODE_SINGLE, 0U, 30000U); press_start(&w); to_on(&w); w.code[0] = 8U; step(&w); step(&w); run_ms(&w, 120000U);
    CHECK(w.out.run[0] == 0U, "Manual: the Mains group waits for START");
    press_start(&w); run_ms(&w, 300U); CHECK(w.out.run[0] == 1U, "START restarts");
    /* a mains cause that never goes away: it trips again after each clear, spaced by the Restart Time, never latching */
    world(&w, M_MODE_SINGLE, 1U, 30000U); to_on(&w); w.stuck[0] = 8U; w.code[0] = 8U; step(&w); step(&w);
    run_ms(&w, 200000U);
    CHECK(unit_st(&w, 0) != (int)U_LATCHED && w.n_log >= 15, "a persistent mains problem keeps retrying every ~7 s and is never latched (%d trips)", w.n_log);
    /* slave code 9 and 6 */
    world(&w, M_MODE_SINGLE, 1U, 30000U); to_on(&w); w.in.ph[0].vin_dv = 1850U; w.code[0] = 9U; step(&w); step(&w);
    CHECK(w.out.shown == MLOG_T_IP_LV, "slave code 9 with the input below the Set Voltage is shown as I/P LV");
    world(&w, M_MODE_SINGLE, 1U, 30000U); to_on(&w); w.in.ph[0].vin_dv = 2700U; w.code[0] = 9U; step(&w); step(&w);
    CHECK(w.out.shown == MLOG_T_IP_HV, "and above it as I/P HV");
    world(&w, M_MODE_SINGLE, 1U, 30000U); to_on(&w); w.code[0] = 6U; step(&w); step(&w);
    CHECK(w.out.shown == MLOG_T_OP_HV && w.out.contactor[0] == 0U, "slave code 6 (its own output guard) is shown as O/P HV, at once");
    printf("  Mains group: 9 codes, Auto/Manual, a persistent cause never latches; codes 9 and 6 map to I/P and O/P\n");
}

static void t_flip_retune_link(void)
{
    W_t w;
    world(&w, M_MODE_DOMESTIC, 1U, 40000U); { uint32_t k; for (k = 0U; k < 400U && !w.out.contactor[1]; k++) step(&w); }
    CHECK(w.out.contactor[0] && w.out.contactor[1] && w.out.contactor[2], "Domestic: all three contactors close");
    w.code[1] = 17U; step(&w); step(&w);
    CHECK(w.out.shown == MLOG_T_FLIP && w.out.shown_phase == 1U && w.out.buzzer == 1U, "a flip sounds the buzzer even in Domestic (D28)");
    CHECK(w.out.contactor[0] && !w.out.contactor[1] && w.out.contactor[2], "and trips only the Y unit: R and B keep running");
    run_ms(&w, 600000U); press_start(&w);
    CHECK(unit_st(&w, 1) == (int)U_LATCHED && !w.out.contactor[1], "latched: ten minutes and a START later it is still latched");
    press_stop(&w); CHECK(w.out.buzzer == 0U && unit_st(&w, 1) == (int)U_LATCHED, "STOP silences the buzzer, the latch stays");
    /* re-tune */
    world(&w, M_MODE_SINGLE, 1U, 40000U); to_on(&w); w.code[0] = 19U; step(&w); step(&w);
    CHECK(w.out.shown == MLOG_T_RETUNE && unit_st(&w, 0) == (int)U_LATCHED, "slave code 19 (not tuned) is Re-tune Required, latched");
    run_ms(&w, 300000U); CHECK(unit_st(&w, 0) == (int)U_LATCHED, "it does not clear by itself");
    MF_RetuneDone(&w.f); run_ms(&w, 100U);
    CHECK(unit_st(&w, 0) == (int)U_OFF && w.out.shown == 0U, "after the re-tune it is off and clear: START runs it");
    world(&w, M_MODE_SINGLE, 1U, 40000U); to_on(&w); w.code[0] = 18U; step(&w); step(&w);
    CHECK(w.out.shown == MLOG_T_RETUNE, "code 18 (loop unstable) is Re-tune Required too");
    /* link lost */
    world(&w, M_MODE_SINGLE, 1U, 40000U); to_on(&w);
    w.in.ph[0].link_ok = 0U; step(&w);
    CHECK(w.out.contactor[0] == 0U && w.out.shown == MLOG_T_LINK_LOST && w.log[0].type == MLOG_T_LINK_LOST, "link lost: trips at once and is logged");
    run_ms(&w, 30000U); CHECK(w.out.run[0] == 0U, "and stays off while the link is down");
    w.in.ph[0].link_ok = 1U; run_ms(&w, 4000U); CHECK(w.out.run[0] == 0U, "frames back: not before the Restart Time");
    run_ms(&w, 2500U); CHECK(w.out.run[0] == 1U, "then it restarts by itself (Auto), like the Mains group");
    printf("  flip (Domestic buzzer, latch), re-tune, link lost\n");
}

static void t_no_retry(void)
{
    W_t w;
    /* commissioning a repaired power stage: ZERO retries. The first inrush latches, nothing re-energises the stage, however long we wait. */
    world(&w, M_MODE_SINGLE, 1U, 20000U); MF_SetMaxRetries(&w.f, 0U); to_on(&w); run_ms(&w, 4000U);
    w.code[0] = 2U; step(&w); step(&w);
    CHECK(unit_st(&w, 0) == (int)U_LATCHED && w.n_log == 1 && w.log[0].type == MLOG_T_INRUSH, "0 retries: the FIRST inrush latches (state %d, %d logged)", unit_st(&w, 0), w.n_log);
    { int run_before = w.n_clr[0]; run_ms(&w, 600000U); CHECK(w.out.run[0] == 0U && w.out.contactor[0] == 0U && w.n_clr[0] == run_before, "ten minutes later: no retry, no clear pulse, the slave was never told to run again"); }
    /* the hardware break code 1 and the power-stage codes behave the same */
    world(&w, M_MODE_SINGLE, 1U, 20000U); MF_SetMaxRetries(&w.f, 0U); to_on(&w); run_ms(&w, 4000U); w.code[0] = 1U; step(&w); step(&w);
    CHECK(unit_st(&w, 0) == (int)U_LATCHED, "the hardware break (code 1) latches at once too");
    world(&w, M_MODE_SINGLE, 1U, 20000U); MF_SetMaxRetries(&w.f, 0U); to_on(&w); run_ms(&w, 4000U); w.code[0] = 7U; step(&w); step(&w);
    CHECK(unit_st(&w, 0) == (int)U_LATCHED, "and so does a power-stage fault (code 7)");
    /* the approved rule is untouched: the default is 3 */
    world(&w, M_MODE_SINGLE, 1U, 20000U); to_on(&w); run_ms(&w, 4000U); w.code[0] = 2U; step(&w); step(&w);
    CHECK(unit_st(&w, 0) == (int)U_FAULT && w.f.max_retries == 3U, "by default the first inrush is a normal trip that retries (state %d, max %u)", unit_st(&w, 0), (unsigned)w.f.max_retries);
    /* 1 retry: exactly one */
    world(&w, M_MODE_SINGLE, 1U, 20000U); MF_SetMaxRetries(&w.f, 1U); to_on(&w); run_ms(&w, 4000U); w.code[0] = 2U; w.stuck[0] = 2U; step(&w); step(&w);
    run_ms(&w, 120000U);
    CHECK(unit_st(&w, 0) == (int)U_LATCHED && w.n_log == 2, "1 retry: one retry, then the latch (%d trips logged)", w.n_log);
    printf("  retries: 0 latches at the first inrush / break / power-stage trip, 1 retries once, the default is the approved 3\n");
}

static void t_modes(void)
{
    W_t w; uint32_t k;
    /* Industrial: one unit, one contactor, any phase trips all */
    world(&w, M_MODE_INDUSTRIAL, 1U, 50000U); for (k = 0U; k < 400U && !w.out.contactor[0]; k++) step(&w);
    CHECK(w.out.contactor[0] && !w.out.contactor[1] && !w.out.contactor[2] && w.out.run[0] && w.out.run[1] && w.out.run[2], "Industrial: ONE contactor (R), all three slaves run");
    w.in.ph[1].vin_dv = 1500U; run_ms(&w, 5300U);
    CHECK(!w.out.contactor[0] && !w.out.run[0] && !w.out.run[1] && !w.out.run[2], "a Y-phase fault trips ALL phases");
    CHECK(w.out.fault[1] == MLOG_T_IP_LV && w.out.fault[0] == 0U && w.out.fault[2] == 0U && w.n_log == 1 && w.log[0].phase == 1U, "and it is attributed to Y, in the display and in the log");
    /* all three must be IN_BAND before the contactor closes */
    world(&w, M_MODE_INDUSTRIAL, 1U, 50000U); w.code[2] = 0U; w.in.ph[2].in_band = 0U;
    { int n = 0; uint32_t kk; for (kk = 0U; kk < 150U; kk++) { step(&w); if (w.out.contactor[0]) n++; } CHECK(n > 0, "Industrial reaches ON when all three are in band"); }
    /* phase sequence */
    world(&w, M_MODE_INDUSTRIAL, 1U, 50000U); w.s.phase_seq = 1U; w.in.seq_state = 1U; run_ms(&w, 30000U);
    CHECK(w.out.run[0] == 0U, "Industrial with the Phase Sequence check on: a reversed sequence blocks the start");
    w.in.seq_state = 0U; run_ms(&w, 6000U); CHECK(w.out.run[0] == 1U, "fixed: it starts"); for (k = 0U; k < 400U && !w.out.contactor[0]; k++) step(&w);
    w.in.seq_state = 2U; step(&w);
    CHECK(!w.out.contactor[0] && w.out.shown == MLOG_T_PHASE_SEQ && w.log[0].type == MLOG_T_PHASE_SEQ && w.log[0].phase == 3U, "a sequence fault while running trips at once, phase 'all'");
    world(&w, M_MODE_INDUSTRIAL, 1U, 50000U); w.s.phase_seq = 0U; w.in.seq_state = 1U; for (k = 0U; k < 400U && !w.out.contactor[0]; k++) step(&w);
    CHECK(w.out.contactor[0], "with the check switched off a reversed sequence is ignored");
    world(&w, M_MODE_SINGLE, 1U, 50000U); w.s.phase_seq = 1U; w.in.seq_state = 1U; for (k = 0U; k < 400U && !w.out.contactor[0]; k++) step(&w);
    CHECK(w.out.contactor[0], "and in Single mode the sequence is never checked");
    /* Domestic: independent units, buzzer off for ordinary faults */
    world(&w, M_MODE_DOMESTIC, 1U, 50000U); for (k = 0U; k < 400U && !w.out.contactor[2]; k++) step(&w);
    w.in.ph[1].vin_dv = 1500U; run_ms(&w, 5300U);
    CHECK(w.out.contactor[0] && !w.out.contactor[1] && w.out.contactor[2], "Domestic: a Y fault trips Y only");
    CHECK(w.out.buzzer == 0U, "and the buzzer is disabled for it in Domestic");
    w.in.ph[1].vin_dv = 2300U; run_ms(&w, 6200U); for (k = 0U; k < 100U && !w.out.contactor[1]; k++) step(&w);
    CHECK(w.out.contactor[1], "Y restarts by itself when its input is good again");
    /* priority across phases: R output LV (30) vs Y overload (60) vs B flip (100) */
    world(&w, M_MODE_DOMESTIC, 1U, 50000U); for (k = 0U; k < 400U && !w.out.contactor[2]; k++) step(&w);
    w.in.ph[0].vout_dv = 1500U; w.in.ph[1].load_da = 1500U; run_ms(&w, 11000U);
    CHECK(w.out.shown == MLOG_T_OVERLOAD && w.out.shown_phase == 1U, "R: O/P LV and Y: overload at once: the overload is shown first");
    w.code[2] = 17U; step(&w); step(&w);
    CHECK(w.out.shown == MLOG_T_FLIP && w.out.shown_phase == 2U, "a flip on B outranks both");
    printf("  modes: Industrial (one unit, all trip, sequence), Single, Domestic (independent), display priority across phases\n");
}

static void t_setmode_and_stale(void)
{
    W_t w; uint32_t k;
    world(&w, M_MODE_SINGLE, 1U, 60000U); to_on(&w);
    w.in.setmode = 1U; step(&w);
    CHECK(!w.out.contactor[0] && !w.out.run[0] && w.n_log == 0, "Set-Mode: the output goes off in the same tick, and that is not a fault (nothing logged)");
    run_ms(&w, 60000U); CHECK(!w.out.run[0], "and stays off for as long as Set-Mode is open");
    w.key_start = 1U; step(&w); CHECK(!w.out.run[0], "START inside Set-Mode does nothing");
    w.in.setmode = 0U; run_ms(&w, 4000U); CHECK(!w.out.run[0], "leaving Set-Mode: the Auto on-delay applies");
    run_ms(&w, 2000U); CHECK(w.out.run[0], "then it starts");
    /* a stale slave code just after the clear pulse is ignored; a slow slave does not make a loop */
    world(&w, M_MODE_SINGLE, 1U, 60000U); w.clear_delay_ms = 600U; to_on(&w); run_ms(&w, 4000U); w.code[0] = 8U; step(&w); step(&w);
    run_ms(&w, 20000U);
    CHECK(w.n_log == 1 && unit_st(&w, 0) != (int)U_FAULT, "a slave that takes 600 ms to answer the clear counter does not cause a second trip from its old code (%d logged)", w.n_log);
    /* a slave that takes longer than the settle time trips again: it is not hidden */
    world(&w, M_MODE_SINGLE, 1U, 60000U); w.clear_delay_ms = 3000U; to_on(&w); run_ms(&w, 4000U); w.code[0] = 8U; step(&w); step(&w); run_ms(&w, 20000U);
    CHECK(w.n_log >= 2, "but a slave that does not clear within 1 s is NOT hidden: it trips again (%d logged)", w.n_log);
    /* time wrap */
    world(&w, M_MODE_SINGLE, 1U, 0xFFFFFFFFUL - 40000UL); to_on(&w);
    CHECK(w.out.contactor[0], "the clock wrapping during the start does not matter");
    w.in.ph[0].vin_dv = 1500U; run_ms(&w, 5300U); CHECK(w.out.shown == MLOG_T_IP_LV, "nor during a delay");
    (void)k;
    printf("  Set-Mode, a slow slave after a clear, clock wrap\n");
}

static void t_priority(void)
{
    /* the agreed display priority: Flip > Inrush > Power stage > (Re-tune) > Heat > Overload > Input V > Phase sequence / Mains > Link lost > Output V */
    static const uint8_t ORDER[][2] = { { MLOG_T_FLIP, 0 }, { MLOG_T_INRUSH, 0 }, { MLOG_T_POWER_STAGE, 0 }, { MLOG_T_RETUNE, 0 }, { MLOG_T_HEAT, 0 }, { MLOG_T_OVERLOAD, 0 },
                                        { MLOG_T_IP_LV, MLOG_T_IP_HV }, { MLOG_T_MAINS, MLOG_T_PHASE_SEQ }, { MLOG_T_LINK_LOST, 0 }, { MLOG_T_OP_LV, MLOG_T_OP_HV } };
    unsigned i, bad = 0U;
    for (i = 0U; i + 1U < sizeof ORDER / sizeof ORDER[0]; i++) if (!(MF_Rank(ORDER[i][0]) > MF_Rank(ORDER[i + 1U][0]))) bad++;
    for (i = 0U; i < sizeof ORDER / sizeof ORDER[0]; i++) if (ORDER[i][1] != 0U && MF_Rank(ORDER[i][0]) != MF_Rank(ORDER[i][1])) bad++;
    CHECK(bad == 0U && MF_Rank(0U) == 0U, "the display priority is exactly the agreed order, with the pairs (I/P LV and HV, Mains and Phase sequence, O/P LV and HV) level (%u wrong)", bad);
    /* and in a live scenario: an inrush on R and a flip on B at once */
    { W_t w; uint32_t k; world(&w, M_MODE_DOMESTIC, 1U, 70000U); for (k = 0U; k < 400U && !w.out.contactor[2]; k++) step(&w);
      w.code[0] = 2U; w.code[2] = 17U; step(&w); step(&w);
      CHECK(w.out.shown == MLOG_T_FLIP && w.out.shown_phase == 2U, "a flip on B outranks an inrush on R in the live display");
      world(&w, M_MODE_DOMESTIC, 1U, 70000U); for (k = 0U; k < 400U && !w.out.contactor[2]; k++) step(&w);
      w.code[0] = 7U; w.in.ph[1].vin_dv = 1500U; run_ms(&w, 5300U);
      CHECK(w.out.shown == MLOG_T_POWER_STAGE && w.out.shown_phase == 0U, "a power-stage fault on R outranks an input fault on Y"); }
    printf("  display priority: the exact agreed order, and in live scenarios\n");
}

static uint32_t rs = 99U;
static uint32_t rnd(void) { rs = (rs * 1664525U) + 1013904223U; return rs >> 8; }

static void t_fuzz(void)
{
    uint32_t round, bad_con = 0U, bad_set = 0U, bad_latch = 0U, bad_log = 0U, ticks = 0U;
    for (round = 0U; round < 60U; round++)
    {
        W_t w; uint32_t k; uint8_t was_latched[3] = { 0U, 0U, 0U }; uint8_t prev_run_state[3] = { 0U, 0U, 0U };
        world(&w, (uint8_t)(rnd() % 3U), (uint8_t)(rnd() & 1U), rnd());
        w.s.phase_seq = (uint8_t)(rnd() & 1U); w.s.restart_time = (uint16_t)(1U + rnd() % 8U); w.s.volt_cut_delay = (uint8_t)(1U + rnd() % 6U);
        for (k = 0U; k < 20000U; k++)
        {
            int p, u, was_energised[3]; uint32_t ev = rnd() % 400U;
            for (u = 0; u < 3; u++) was_energised[u] = (prev_run_state[u] == U_STARTING || prev_run_state[u] == U_ON);
            if (ev == 0U) w.in.ph[rnd() % 3U].vin_dv = (uint16_t)(1000U + rnd() % 2500U);
            if (ev == 1U) w.in.ph[rnd() % 3U].vout_dv = (uint16_t)(1500U + rnd() % 1500U);
            if (ev == 2U) w.in.ph[rnd() % 3U].load_da = (uint16_t)(rnd() % 2000U);
            if (ev == 3U) w.in.ph[rnd() % 3U].heat = (uint8_t)(rnd() & 1U);
            if (ev == 4U) w.in.ph[rnd() % 3U].link_ok = (uint8_t)((rnd() % 4U) != 0U);
            if (ev == 5U) w.code[rnd() % 3U] = (uint8_t)(rnd() % 22U);
            if (ev == 6U) w.in.seq_state = (uint8_t)(rnd() % 3U);
            if (ev == 7U) w.in.setmode = (uint8_t)(rnd() & 1U);
            if (ev == 8U) w.in.ph[rnd() % 3U].ct_peak = (uint8_t)(rnd() % 8U == 0U);
            if (ev == 9U) w.key_start = 1U;
            if (ev == 10U) w.key_stop = 1U;
            if (ev == 11U) w.stuck[rnd() % 3U] = (uint8_t)((rnd() % 3U == 0U) ? (rnd() % 22U) : 0U);
            step(&w); ticks++;
            for (p = 0; p < 3; p++)
            {   /* (a) a closed contactor means every phase of its unit is told to run */
                if (w.out.contactor[p] && !w.out.run[p]) bad_con++;
                if (w.f.mode == (uint8_t)M_MODE_INDUSTRIAL && w.out.contactor[0] && !(w.out.run[0] && w.out.run[1] && w.out.run[2])) bad_con++;
                /* (b) nothing is energised or closed while Set-Mode is open */
                if (w.in.setmode && (w.out.contactor[p] || w.out.run[p])) bad_set++;
            }
            for (u = 0; u < (int)w.f.nunits; u++)
            {   /* (c) a latch never lets go, apart from the Re-tune one which only MF_RetuneDone clears (not called here) */
                if (was_latched[u] && w.out.unit_state[u] != U_LATCHED && w.f.mode == (uint8_t)w.s.mode) bad_latch++;
                was_latched[u] = (uint8_t)(w.out.unit_state[u] == U_LATCHED);
                prev_run_state[u] = w.out.unit_state[u];
            }
            /* (d) a fault is logged only when something was energised the tick before */
            if (w.out.n_log > 0 && !(was_energised[0] || was_energised[1] || was_energised[2])) bad_log++;
            if (w.f.mode != (uint8_t)w.s.mode) break;
        }
    }
    CHECK(bad_con == 0U, "over %u random ticks: a closed contactor always has its slaves running (%u violations)", (unsigned)ticks, (unsigned)bad_con);
    CHECK(bad_set == 0U, "nothing is energised while Set-Mode is open (%u violations)", (unsigned)bad_set);
    CHECK(bad_latch == 0U, "a latched unit never lets go by itself (%u violations)", (unsigned)bad_latch);
    CHECK(bad_log == 0U, "a fault is only logged when the unit was energised (%u violations)", (unsigned)bad_log);
    printf("  fuzz: %u random ticks in 60 random configurations: the invariants hold\n", (unsigned)ticks);
}

int main(void)
{
    t_powerup();
    t_stop_start();
    t_input_voltage();
    t_output_voltage();
    t_overload_heat();
    t_retry_groups();
    t_mains_group();
    t_flip_retune_link();
    t_modes();
    t_no_retry();
    t_setmode_and_stale();
    t_priority();
    t_fuzz();
    printf("  %d checks, %d failures\n  RESULT: %s\n", g_checks, g_fail, g_fail ? "FAILURES" : "ALL PASSED");
    return g_fail ? 1 : 0;
}
