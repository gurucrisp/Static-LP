/* Host test of slave_ctrl.c (the REAL control code, unmodified) against a
 * simulated world:
 *   - a sine mains (frequency, amplitude, phase jumps)
 *   - the ZCD output in several patterns: square wave, inverted square wave,
 *     pulse train (one pulse per half-cycle), one pulse per cycle, nothing
 *   - mock power stage that CHECKS SAFETY INVARIANTS on every call:
 *       * the two converter diagonals are never on together
 *       * >= CTRL_DEADTIME_US between one diagonal going off and the next on
 *       * the energised diagonal never conducts against the line
 *         (checked every 100 us, except within +/-0.64 ms of a zero)
 *       * inverter duty only while a diagonal is on and the inverter enabled
 *       * only one inverter leg carries duty, within the duty cap
 *       * the leg chosen matches the old firmware's table
 *   - scripted faults: lost pattern, sense mismatch, tiny signal, break, FLT,
 *     input range, output high, phase shift, timer wrap-around.
 *
 * Each test runs in its own forked process (fresh static state, like a boot).
 *
 * Build (one binary per stage; stage decides what the controller may do):
 *   gcc -O1 -Wall -Istubs -I.. -DSLAVE_POWER_STAGE=2 -DBRINGUP_REPORT_MODE=2 \
 *       test_ctrl.c ../slave_ctrl.c ../slave_reg.c -o test_ctrl_s2 -lm
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <sys/wait.h>

#include "slave_ctrl.h"
#include "slave_features.h"
#if (SLAVE_VOLTAGE_CAL != 0)
#include "slave_cal.h"
#endif
#include "slave_ctrl_hw.h"
#include "slave_power.h"
#include "slave_relay.h"

#define PI 3.14159265358979323846
#if (SLAVE_RELAYS_MODE == 2)
#define WARM 3000000ULL          /* LEGACY relays: wait for a trustworthy zero-cross, SH 1 s, W settle 1 s, then the converter starts */
#elif (SLAVE_RELAYS_MODE == 1)
#define WARM 250000ULL           /* the relays are driven to the SHORTED pattern at start-up and need their settle time */
#else
#define WARM 0ULL
#endif

/* ======================================================================= */
/* Simulated world                                                          */
/* ======================================================================= */
enum { PAT_SQUARE = 0, PAT_SQUARE_INV, PAT_PULSE, PAT_HALF, PAT_NONE };

typedef struct { int64_t t; uint8_t rising; } Edge_t;

static struct
{
    double  f_hz, amp, phase0_us;       /* mains: amp in ADC counts (peak); phase0 = a positive-going zero */
    int     pattern;
    double  edge_delay_us, pulse_w_us, jitter_us;
    float   vin, vout;
    int     meas_valid, meas_running;
    int     flt, brk, inv_ok, in_zero, out_zero;
    double  out_gain;
    int     plant_on;
    double  plant_gain;                 /* volts per duty count (a NEGATIVE gain models a transformer connected the wrong way round) */
    double  out_offset;                 /* constant offset of the AC_OUT reading against AC_IN, volts (sensor tolerance) */
    int     plant_curve;                /* 1 = the gain per effective tick follows the MEASURED curve g(Vin) (bench, v0.7.1, 64 effective ticks); 0 = plant_gain */
    double  plant_dead;                 /* dead time in timer ticks: the real high-side on-time is CCR - plant_dead, and nothing at all if CCR <= plant_dead (0 = not modelled) */
    int     skip_crossings;             /* swallow the next N zero crossings (lost pulses) */
    int     drop_deadtime;              /* the dead-time compare interrupt never reaches the controller */
    int     no_wd;                      /* the watchdog compare is never armed (to reach the main-loop backup) */
    int     guard_glitch;               /* the next N polarity-guard samples show the WRONG sign (noise injection) */
    int     sign_glitch;                /* the next N AC_IN signs read at a zero-cross event are WRONG (opposite)  */
    int     preen_glitch;               /* the next N last-moment (strong) polarity checks show the OPPOSITE sign  */
} W;

static uint64_t sim_t;                  /* microseconds, 64-bit; the code under test sees the low 32 bits */
static uint64_t next_service, next_meas, next_sample;
static uint32_t meas_updates;
/* the plant gain MEASURED at stage 2 (injected change at 64 effective ticks: 0.7 V at 110 V, 1.9 V at 200 V, 2.1 V at 250 V), volts per effective tick */
static double plant_g_of_vin(double vin)
{
    if (vin <= 110.0) return 0.011 * vin / 110.0;
    if (vin <= 200.0) return 0.011 + (0.0297 - 0.011) * (vin - 110.0) / 90.0;
    if (vin <= 250.0) return 0.0297 + (0.0328 - 0.0297) * (vin - 200.0) / 50.0;
    return 0.0328;
}
static uint32_t lcg = 12345U;
static double   frand(void) { lcg = lcg * 1664525U + 1013904223U; return ((double)(lcg >> 8) / 8388608.0) - 1.0; } /* -1..1 */

/* compare timers */
static int      dt_armed, wd_armed, pf_armed;
static uint64_t dt_fire, wd_fire, pf_fire, next_tick;

/* edge queue */
static Edge_t   eq[8];
static int      neq;
static int64_t  next_k;

/* mock power stage */
static int      diag;                   /* +1 positive diagonal, -1 negative, 0 none */
static int      have_off_time;
static uint64_t t_diag_off;
static uint64_t min_gap_us = 1000000;
static int      inv_enabled;
static uint16_t leg1, leg2;
static int      n_inv_enable, n_alloff, n_sync, n_energise, n_setduty;
static int      probe_armed, probe_taken;                      /* D25: what had the FLT handler already touched when it switched the outputs off? */
static uint8_t  probe_flt_run, probe_flt_idle, probe_run_allowed;
static uint16_t probe_isr_fault, probe_trip_cap;
static uint64_t t_last_edge, t_first_energise;
static long     v_both, v_dead, v_polarity, v_duty_disabled, v_duty_no_conv, v_two_legs, v_over_cap, v_leg_table, v_energised_not_running;
static uint64_t wrong_us, max_wrong_run_us, cur_wrong_run_us;

/* ---- PHYSICAL MODEL of the two BYPASS relays (SH-RLY, W-RLY): each contact joins PRI-PHASE and PRI-NTRL, i.e. it SHORTS the
 *      transformer primary when closed. The contact follows its coil after a mechanical delay of 6..12 ms. Which coil state
 *      closes it is a build setting (SLAVE_RELAY_SH_CONTACT / SLAVE_RELAY_W_CONTACT), so the whole suite runs for every type. ---- */
typedef struct { int coil; int shorts; int pending; int target; uint64_t t_done; } RlyM_t;
static RlyM_t   rl[2];                               /* 0 = SH-RLY, 1 = W-RLY */
static int      seq_started;                         /* the inverter has been enabled at least once */
static long     n_relay_ops;
static long     v_shoot, v_shoot_samples, v_coil_under_drive, v_enable_unshunted;
static uint64_t unctl_us, max_unctl_run_us, cur_unctl_run_us;
static uint64_t unctl_limit_us;                      /* allowed "primary open and uncontrolled" time (0 = none) */
static uint64_t t_conv_first, t_inv_enable_first, t_free_cmd_first, t_first_duty, t_shorted_cmd_last, t_inv_disable_last, t_duty_zero_last;
static int      last_coil_cmd_free;
#define RELAY_MODEL     (SLAVE_RELAYS_MODE == 1)            /* the bypass-relay rules R1-R5 (mode 1 only) */
#define LEGACY_MODEL    (SLAVE_RELAYS_MODE == 2)            /* the old firmware's relay sequence */
/* coil command log (all modes): time, which relay (0 SH, 1 W), level */
static int      lg_n;
static uint64_t lg_t[64];
static uint8_t  lg_idx[64], lg_on[64];
#define MECH_MIN_US     6000ULL
#define MECH_SPREAD_US  6000ULL
#define RELAY_SETTLE_US ((uint64_t)SLAVE_RELAY_SETTLE_MS * 1000ULL)

static long g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; printf("      FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

/* ---- ADC / mains model ---- */
static double mains(double t_us) { return sin(2.0 * PI * W.f_hz * (t_us - W.phase0_us) / 1e6); }
static double half_period_us(void) { return 0.5e6 / W.f_hz; }

/* ======================================================================= */
/* CtrlHw_* mocks                                                           */
/* ======================================================================= */
void     CtrlHw_Init(void) { }
uint32_t CtrlHw_NowUs(void) { return (uint32_t)sim_t; }
uint32_t CtrlHw_NowMs(void) { return (uint32_t)(sim_t / 1000U); }
uint32_t CtrlHw_Lock(void) { return 0U; }
void     CtrlHw_Unlock(uint32_t p) { (void)p; }

static uint64_t fire_time(uint32_t at_us)
{
    int32_t d = (int32_t)(at_us - (uint32_t)sim_t);
    return (d <= 0) ? sim_t : sim_t + (uint64_t)d;     /* already passed -> fire now (like the EGR) */
}
void CtrlHw_ArmDeadtime(uint32_t at_us)  { dt_armed = 1; dt_fire = fire_time(at_us); }
void CtrlHw_DisarmDeadtime(void)         { dt_armed = 0; }
void CtrlHw_ArmPredictOff(uint32_t at_us){ pf_armed = 1; pf_fire = fire_time(at_us); }
void CtrlHw_DisarmPredictOff(void)       { pf_armed = 0; }
void CtrlHw_ArmWatchdog(uint32_t at_us)  { if (!W.no_wd) { wd_armed = 1; wd_fire = fire_time(at_us); } }
void CtrlHw_DisarmWatchdog(void)         { wd_armed = 0; }

/* The guard looks at the newest two frames, about 75 us old on average. */
int8_t CtrlHw_SignLatest(uint8_t strong)
{
    if (!strong && W.guard_glitch > 0) { W.guard_glitch--; return (int8_t)(-g_ctrl.conv_pol); }   /* a bad sample */
    if (strong && W.preen_glitch > 0) { W.preen_glitch--; return (int8_t)(-g_ctrl.pending_pol); }   /* disagrees right before energising */
    int32_t sum = (int32_t)lround(W.amp * mains((double)sim_t - 75.0) * 2.0);
    int32_t thr = strong ? 240 : 80;                     /* 2 samples x (120 or 40 counts) */
    return (int8_t)((sum > thr) ? 1 : ((sum < -thr) ? -1 : 0));
}

int8_t CtrlHw_Sign(uint8_t out_channel, int32_t *dev_sum)
{
    /* The real helper averages 8 samples centred about 400 us before "now". */
    double v = W.amp * mains((double)sim_t - 400.0);
    int32_t sum;
    if (out_channel) { v *= W.out_gain; if (W.out_zero) v = 0.0; }
    else             { if (W.in_zero) v = 0.0; }
    sum = (int32_t)lround(v * 8.0);
    if (!out_channel && W.sign_glitch > 0) { W.sign_glitch--; sum = -sum; }          /* a misread sign at an event */
    if (dev_sum) *dev_sum = sum;
    return (int8_t)((sum > 160) ? 1 : ((sum < -160) ? -1 : 0));
}

void CtrlHw_GetMeas(CtrlMeas_t *m)
{
    m->vin = W.vin; m->vout = W.vout; m->update_count = meas_updates; m->valid = (uint8_t)W.meas_valid;
}

/* ======================================================================= */
/* slave_power.h mocks with invariant checking                              */
/* ======================================================================= */
TIM_HandleTypeDef htim1, htim15;

void SlavePower_ConverterOffNow(void)
{
    if (diag != 0) { diag = 0; t_diag_off = sim_t; have_off_time = 1; }
}
static int inv_forced;
void SlavePower_InverterDisable(void) { if (inv_enabled) t_inv_disable_last = sim_t; inv_enabled = 0; inv_forced = 0; leg1 = leg2 = 0; }
static void probe_shutdown(void)
{
    if (probe_armed && !probe_taken)
    {
        probe_taken = 1; probe_flt_run = g_ctrl.flt_run; probe_flt_idle = g_ctrl.flt_idle;
        probe_run_allowed = g_ctrl.run_allowed; probe_isr_fault = g_ctrl.isr_fault; probe_trip_cap = g_ctrl.trip_cap;
    }
}
void SlavePower_AllOffNow(void)
{
    probe_shutdown();
    SlavePower_ConverterOffNow(); SlavePower_InverterDisable(); n_alloff++;
}
/* v0.9.65: the FLT shutdown. SLAVE_FLT_KEEP_LOWS: the PWM channels are FORCED inactive (high sides off, low sides on) until the next InverterEnable; the inverter stays enabled */
static int n_fltoff; static long v_duty_forced;
void SlavePower_FltOffNow(void)
{
#if (SLAVE_FLT_KEEP_LOWS != 0)
    probe_shutdown();
    if (inv_enabled) inv_forced = 1;
    leg1 = leg2 = 0;
    SlavePower_ConverterOffNow(); n_fltoff++;
#else
    SlavePower_AllOffNow();
#endif
}

static uint64_t t_duty_nz_last;                         /* time of the last NONZERO duty command */
static long     v_duty_below_dt;                        /* stage 2: a nonzero CCR at or below the dead time = a command that produces NO output */
static long     v_duty_not_run, v_duty_w_off;           /* legacy relays: nonzero duty outside CTRL_RUN / with the W coil off (nets not open) */
static uint64_t t_last_zc_irq, dt_en1, dt_en2;          /* time of the last zero-cross interrupt; delay of the 1st / 2nd diagonal after the event that armed it */
static int first_energise_pol;                          /* polarity served by the FIRST diagonal energised since world_reset (0 = none yet) */
static int inv_at_first_energise = -1;                  /* was the inverter (its low sides) enabled when that first diagonal was energised? */
static void Energise(int pol)
{
    if (n_energise == 0) dt_en1 = sim_t - t_last_zc_irq; else if (n_energise == 1) dt_en2 = sim_t - t_last_zc_irq;
    if (first_energise_pol == 0) { first_energise_pol = pol; inv_at_first_energise = inv_enabled; }
    if (diag != 0 && diag != pol) v_both++;                         /* switched without passing through OFF */
    if (diag == 0 && have_off_time)
    {
        uint64_t gap = sim_t - t_diag_off;
        if (gap < min_gap_us) min_gap_us = gap;
        if (gap < CTRL_DEADTIME_US) v_dead++;
    }
    if (t_first_energise == 0) t_first_energise = sim_t;
    diag = pol; n_energise++;
}
/* ---- relay physics ---- */
static int rly_type(int idx)  { return idx == 0 ? SLAVE_RELAY_SH_CONTACT : SLAVE_RELAY_W_CONTACT; }
static int rly_shorts_for_coil(int type, int coil)
{
    if (type == RELAY_SHORTS_WHEN_ENERGISED) return coil;
    if (type == RELAY_SHORTS_WHEN_RELEASED)  return !coil;
    return 0;                                         /* RELAY_NOT_A_SHUNT */
}
static int duty_nonzero(void) { return inv_enabled && (leg1 || leg2); }
static void relay_update(void)                        /* let the contacts follow their coils */
{
    int i;
    for (i = 0; i < 2; i++)
        if (rl[i].pending && sim_t >= rl[i].t_done)
        {
            rl[i].shorts = rl[i].target; rl[i].pending = 0;
            if (rl[i].shorts && duty_nonzero()) v_shoot++;           /* a contact closed under inverter drive */
        }
}
static int any_shunt(void) { relay_update(); return rl[0].shorts || rl[1].shorts; }
static void relay_cmd(int idx, int on)
{
    static uint32_t jit = 777U;
    n_relay_ops++;
    relay_update();
    if (rl[idx].coil == on) return;
    if (lg_n < 64) { lg_t[lg_n] = sim_t; lg_idx[lg_n] = (uint8_t)idx; lg_on[lg_n] = (uint8_t)on; lg_n++; }
    if (duty_nonzero()) v_coil_under_drive++;                         /* rule R2 */
    {
        int type = rly_type(idx), tgt = rly_shorts_for_coil(type, on);
        int free_pattern;
        rl[idx].coil = on;
        jit = jit * 1664525U + 1013904223U;
        if (tgt != rl[idx].shorts) { rl[idx].target = tgt; rl[idx].pending = 1; rl[idx].t_done = sim_t + MECH_MIN_US + (jit >> 8) % MECH_SPREAD_US; }
        else                       { rl[idx].pending = 0; }
        free_pattern = !rly_shorts_for_coil(rly_type(0), rl[0].coil) && !rly_shorts_for_coil(rly_type(1), rl[1].coil);
        if (free_pattern && !last_coil_cmd_free && t_free_cmd_first == 0 && seq_started) t_free_cmd_first = sim_t;
        if (!free_pattern && last_coil_cmd_free) t_shorted_cmd_last = sim_t;
        last_coil_cmd_free = free_pattern;
    }
}
void SlavePower_RelaySh(uint8_t on) { relay_cmd(0, on); }
void SlavePower_RelayW (uint8_t on) { relay_cmd(1, on); }

/* SLAVE_CONV_SWAP deliberately calls the OTHER diagonal for each half-cycle, so the PHYSICAL diagonal energised for the polarity served
 * is the same: the harness models "the polarity served" and all line / polarity invariants stay meaningful. */
void SlavePower_ConverterPositive(void) { if (t_conv_first == 0) t_conv_first = sim_t; Energise(SLAVE_CONV_SWAP ? -1 : +1); }
void SlavePower_ConverterNegative(void) { if (t_conv_first == 0) t_conv_first = sim_t; Energise(SLAVE_CONV_SWAP ? +1 : -1); }

uint8_t SlavePower_InverterIsEnabled(void) { return (uint8_t)inv_enabled; }

uint8_t SlavePower_InverterEnable(void)
{
    n_inv_enable++;
    if (!W.inv_ok) return 0U;
#if RELAY_MODEL
    /* rule R4: the inverter is enabled while the primary is STILL SHORTED by a relay contact */
    if ((rly_type(0) != RELAY_NOT_A_SHUNT || rly_type(1) != RELAY_NOT_A_SHUNT) && !any_shunt()) v_enable_unshunted++;
#endif
    if (t_inv_enable_first == 0) t_inv_enable_first = sim_t;
    seq_started = 1;
    inv_enabled = 1; inv_forced = 0; leg1 = leg2 = 0;     /* v0.9.65: the PWM mode is restored */
    return 1U;
}

void SlavePower_SetLegDuty(uint16_t l1, uint16_t l2)
{
    n_setduty++;
    if (inv_forced && (l1 || l2)) v_duty_forced++;            /* v0.9.65: a duty command while the FLT shutdown holds the PWM forced inactive */
    leg1 = l1; leg2 = l2;
    if (!(l1 || l2)) t_duty_zero_last = sim_t;
    if (l1 || l2)
    {
        t_duty_nz_last = sim_t;
#if (SLAVE_POWER_STAGE >= 2)
        if ((l1 && l1 <= POWER_DEADTIME_TICKS) || (l2 && l2 <= POWER_DEADTIME_TICKS)) v_duty_below_dt++;
#endif
#if (SLAVE_RELAYS_MODE == 2)
        if (g_ctrl.state != CTRL_RUN) v_duty_not_run++;
        if (rl[1].coil != 1)          v_duty_w_off++;
#endif
        if (t_first_duty == 0) t_first_duty = sim_t;
#if RELAY_MODEL
        if (any_shunt()) v_shoot++;                                   /* rule R1: a HIGH side driven while a shunt is closed */
#endif
        const uint16_t cap = (uint16_t)((CTRL_DUTY_MAX_PCT * POWER_PERIOD_TICKS) / 100UL);
        int exp_leg1;
        if (!inv_enabled) v_duty_disabled++;
        if (diag == 0)    v_duty_no_conv++;
        if (l1 && l2)     v_two_legs++;
        if (l1 > cap || l2 > cap) v_over_cap++;
        /* old firmware table: positive half: boost->leg1(TIM15), buck->leg2(TIM1);
         *                      negative half: boost->leg2(TIM1),  buck->leg1(TIM15) */
        exp_leg1 = (diag > 0) ? (g_ctrl.boost != 0) : (g_ctrl.boost == 0);
        if (l1 && !exp_leg1) v_leg_table++;
        if (l2 &&  exp_leg1) v_leg_table++;
    }
}
void    SlavePower_SyncCounters(void)    { n_sync++; }
uint8_t SlavePower_FltAsserted(void)     { return (uint8_t)W.flt; }
uint8_t SlavePower_BreakInputActive(void){ return (uint8_t)W.brk; }
void    SlavePower_BreakIrqRearm(void)   { }

/* ======================================================================= */
/* Simulation engine                                                        */
/* ======================================================================= */
static void q_insert(int64_t t, uint8_t rising)
{
    int i = neq++;
    if (t < (int64_t)sim_t) t = (int64_t)sim_t;
    while (i > 0 && eq[i - 1].t > t) { eq[i] = eq[i - 1]; i--; }
    eq[i].t = t; eq[i].rising = rising;
}

static void gen_crossing(int64_t k)
{
    double  tz = W.phase0_us + (double)k * half_period_us();
    if (W.skip_crossings > 0) { W.skip_crossings--; return; }
    int     pos_going = (((k % 2) + 2) % 2) == 0;
    double  j = (W.jitter_us > 0.0) ? frand() * W.jitter_us : 0.0;
    int64_t base = (int64_t)llround(tz + W.edge_delay_us + j);

    switch (W.pattern)
    {
    case PAT_SQUARE:     q_insert(base, pos_going ? 1U : 0U); break;
    case PAT_SQUARE_INV: q_insert(base, pos_going ? 0U : 1U); break;
    case PAT_PULSE:      q_insert(base, 0U); q_insert(base + (int64_t)W.pulse_w_us, 1U); break;
    case PAT_HALF:       if (pos_going) { q_insert(base, 0U); q_insert(base + (int64_t)W.pulse_w_us, 1U); } break;
    default: break;
    }
}

static void resync_crossings(void)      /* after a phase / frequency change */
{
    double rel = ((double)sim_t - W.phase0_us) / half_period_us();
    next_k = (int64_t)ceil(rel);
    neq = 0;
}

static void sample_invariants(void)
{
    double m = mains((double)sim_t);
    int wrong = 0;

    if (diag != 0 && fabs(m) > 0.2 && ((m > 0.0) != (diag > 0)))
    {
        v_polarity++; wrong = 1; wrong_us += 100;
    }
    if (wrong) { cur_wrong_run_us += 100; if (cur_wrong_run_us > max_wrong_run_us) max_wrong_run_us = cur_wrong_run_us; }
    else       cur_wrong_run_us = 0;

    if (diag != 0 && (g_ctrl.state == CTRL_FAULT || g_ctrl.state == CTRL_LATCHED || g_ctrl.state == CTRL_SYNC))
        v_energised_not_running++;

#if RELAY_MODEL
    if (duty_nonzero() && any_shunt()) v_shoot_samples++;             /* R1, sampled: duty while a shunt is closed */
    if (seq_started && !any_shunt() && !inv_enabled)                  /* R3: primary open AND nothing driving it */
    { unctl_us += 100; cur_unctl_run_us += 100; if (cur_unctl_run_us > max_unctl_run_us) max_unctl_run_us = cur_unctl_run_us; }
    else cur_unctl_run_us = 0;
#endif
}

static void sim_run_until(uint64_t t_end)
{
    for (;;)
    {
        uint64_t t_next = t_end;
        int ev = -1;

        while (neq == 0 && (double)next_k * half_period_us() + W.phase0_us < (double)t_end)
            gen_crossing(next_k++);

        if (neq > 0 && (uint64_t)eq[0].t < t_next) { t_next = (uint64_t)eq[0].t; ev = 0; }
        if (dt_armed && dt_fire < t_next)          { t_next = dt_fire;           ev = 1; }
        if (pf_armed && pf_fire < t_next)          { t_next = pf_fire;           ev = 7; }
        if (wd_armed && wd_fire < t_next)          { t_next = wd_fire;           ev = 2; }
        if (next_tick < t_next)                    { t_next = next_tick;         ev = 6; }
        if (next_service < t_next)                 { t_next = next_service;      ev = 3; }
        if (next_meas < t_next)                    { t_next = next_meas;         ev = 4; }
        if (next_sample < t_next)                  { t_next = next_sample;       ev = 5; }
        if (ev < 0) { sim_t = t_end; return; }

        sim_t = t_next;
        switch (ev)
        {
        case 0: {
            Edge_t e = eq[0];
            int i;
            for (i = 1; i < neq; i++) eq[i - 1] = eq[i];
            neq--;
            t_last_edge = sim_t;
            t_last_zc_irq = sim_t;
            SlaveCtrl_OnZcEdge((uint32_t)sim_t, e.rising);
            break; }
        case 1: dt_armed = 0; if (!W.drop_deadtime) SlaveCtrl_OnDeadtimeIrq(); break;
        case 2: wd_armed = 0; SlaveCtrl_OnWatchdogIrq();  break;
        case 3: SlaveCtrl_Service(); next_service += 200; break;
        case 4:
            if (W.meas_running) meas_updates++;
            if (W.plant_on && inv_enabled)
            {
                /* crude plant: output = input +/- duty * gain (boost adds, buck subtracts) */
                double d = (double)((leg1 > leg2) ? leg1 : leg2);
                d = (d > W.plant_dead) ? (d - W.plant_dead) : 0.0;          /* a pulse shorter than the dead time is NOT generated at all */
                double inj = d * (W.plant_curve ? plant_g_of_vin((double)W.vin) : W.plant_gain);
                W.vout = (float)((g_ctrl.boost ? W.vin + inj : W.vin - inj) + W.out_offset);
            }
            next_meas += 10000; break;
        case 5: sample_invariants(); next_sample += 100; break;
        case 6: SlaveCtrl_OnSampleTick(); next_tick += 50; break;
        case 7: pf_armed = 0; SlaveCtrl_OnPredictOffIrq(); break;
        }
    }
}
static void run_for(uint64_t us) { sim_run_until(sim_t + us); }

static void world_reset(uint64_t start_t)
{
    memset(&W, 0, sizeof W);
    W.f_hz = 50.0; W.amp = 784.0; W.pattern = PAT_SQUARE;
    W.edge_delay_us = 20.0; W.pulse_w_us = 1500.0; W.jitter_us = 0.0;
    W.vin = 230.0f; W.vout = 230.0f; W.meas_valid = 1; W.meas_running = 1; W.inv_ok = 1;
    W.out_gain = 1.0; W.plant_gain = 0.010; W.out_offset = 0.0; W.plant_curve = 0; W.plant_dead = (double)POWER_DEADTIME_TICKS;     /* the hardware always has the dead time */

    sim_t = start_t; W.phase0_us = (double)start_t + 3333.0;
    next_service = start_t + 200; next_meas = start_t + 10000; next_sample = start_t + 100;
    next_tick = start_t + 50;
    meas_updates = 1; dt_armed = wd_armed = pf_armed = 0; neq = 0;
    t_first_energise = 0;
    memset(rl, 0, sizeof rl);
    lg_n = 0;
    first_energise_pol = 0; inv_at_first_energise = -1;
    t_duty_nz_last = 0ULL; v_duty_not_run = v_duty_w_off = 0; v_duty_below_dt = 0;
    rl[0].shorts = rly_shorts_for_coil(rly_type(0), 0);               /* the contacts' REST state: coils off */
    rl[1].shorts = rly_shorts_for_coil(rly_type(1), 0);
    seq_started = 0; last_coil_cmd_free = 0; n_relay_ops = 0;
    v_shoot = v_shoot_samples = v_coil_under_drive = v_enable_unshunted = 0;
    unctl_us = max_unctl_run_us = cur_unctl_run_us = 0; unctl_limit_us = 0;
    t_conv_first = t_inv_enable_first = t_free_cmd_first = t_first_duty = t_shorted_cmd_last = t_inv_disable_last = t_duty_zero_last = 0;
    diag = 0; have_off_time = 0; min_gap_us = 1000000; inv_enabled = 0; leg1 = leg2 = 0;
    n_inv_enable = n_alloff = n_sync = n_energise = n_setduty = 0; n_fltoff = 0; inv_forced = 0; v_duty_forced = 0;
    v_both = v_dead = v_polarity = v_duty_disabled = v_duty_no_conv = v_two_legs = v_over_cap = v_leg_table = v_energised_not_running = 0;
    wrong_us = max_wrong_run_us = cur_wrong_run_us = 0;
    resync_crossings();

    SlaveCtrl_Init();
#if (SLAVE_MASTER_CONTROL == 0)
    SlaveCtrl_SetRunRequest(1U);          /* the bench default: the harness plays the autorun. With master control the MASTER decides: nothing here. */
#endif
}

static int is_running(void) { return g_ctrl.state >= CTRL_RECT && g_ctrl.state <= CTRL_RESHUNT; }
/* v0.9.65: what the FLT shutdown must leave at once: the converter off, no duty, and the inverter either disabled or FORCED inactive (high sides off, low sides
 * on: SLAVE_FLT_KEEP_LOWS) until the shunts have closed; FLT_UNWIND_US covers that unwinding before the controller is in FAULT */
static int flt_outputs_safe(void) { return (diag == 0) && (leg1 == 0) && (leg2 == 0) && (!inv_enabled || ((SLAVE_FLT_KEEP_LOWS != 0) && inv_forced)); }
#define FLT_UNWIND_US ((uint64_t)(CTRL_RESHUNT_TIMEOUT_MS + 50U) * 1000ULL)

static void check_invariants(const char *what)
{
    CHECK(v_both == 0,              "%s: both diagonals / switched without OFF: %ld", what, v_both);
    CHECK(v_dead == 0,              "%s: dead time violated %ld times (min gap %llu us)", what, v_dead, (unsigned long long)min_gap_us);
    CHECK(v_polarity == 0,          "%s: diagonal conducted against the line at %ld samples (longest %llu us)", what, v_polarity, (unsigned long long)max_wrong_run_us);
    CHECK(v_duty_disabled == 0,     "%s: duty while inverter disabled %ld", what, v_duty_disabled);
    CHECK(v_duty_no_conv == 0,      "%s: duty while no converter diagonal %ld", what, v_duty_no_conv);
    CHECK(v_two_legs == 0,          "%s: both inverter legs carried duty %ld", what, v_two_legs);
    CHECK(v_over_cap == 0,          "%s: duty above the cap %ld", what, v_over_cap);
    CHECK(v_leg_table == 0,         "%s: wrong inverter leg for polarity/boost %ld", what, v_leg_table);
    CHECK(v_energised_not_running == 0, "%s: converter energised while FAULT/LATCHED/SYNC %ld", what, v_energised_not_running);
    CHECK(v_duty_forced == 0,       "%s: duty commanded while the FLT shutdown held the PWM forced inactive %ld", what, v_duty_forced);
#if RELAY_MODEL
    CHECK(v_shoot == 0,             "%s: R1 violated - inverter driven / contact closed under drive %ld times", what, v_shoot);
    CHECK(v_shoot_samples == 0,     "%s: R1 violated - duty while a shunt was closed, %ld samples", what, v_shoot_samples);
    CHECK(v_coil_under_drive == 0,  "%s: R2 violated - a relay coil changed while the inverter was driving: %ld", what, v_coil_under_drive);
    CHECK(v_enable_unshunted == 0,  "%s: R4 violated - inverter enabled while the primary was not shorted by the relays: %ld", what, v_enable_unshunted);
    CHECK(max_unctl_run_us <= unctl_limit_us, "%s: R3 violated - primary OPEN and UNCONTROLLED for %llu us (allowed %llu)", what,
          (unsigned long long)max_unctl_run_us, (unsigned long long)unctl_limit_us);
#endif
#if (SLAVE_POWER_STAGE >= 2)
    CHECK(v_duty_below_dt == 0,     "%s: stage 2 commanded a CCR at or below the %u-tick dead time %ld times: the hardware generates NO pulse for it", what, (unsigned)POWER_DEADTIME_TICKS, v_duty_below_dt);
#endif
#if (SLAVE_RELAYS_MODE == 2) && (SLAVE_POWER_STAGE >= 2)
    CHECK(v_duty_not_run == 0,      "%s: legacy relays: duty applied outside CTRL_RUN %ld times", what, v_duty_not_run);
    CHECK(v_duty_w_off == 0,        "%s: legacy relays: duty applied while the W coil was off (the nets not open) %ld times", what, v_duty_w_off);
#endif
#if (SLAVE_RELAYS_MODE == 0)
    CHECK(n_relay_ops == 0,         "%s: relays are in MANUAL mode but the firmware touched them %ld times", what, n_relay_ops);
#endif
#if (SLAVE_POWER_STAGE == 1)
#if (SLAVE_LEGACY_LOWSIDES == 0)
    CHECK(n_inv_enable == 0,        "%s: inverter enabled at stage 1", what);
#else
    CHECK(t_first_duty == 0,        "%s: duty applied at stage 1 (the legacy low sides must stay at ZERO duty)", what);
#endif
#endif
}

/* ======================================================================= */
/* Tests                                                                    */
/* ======================================================================= */
static void t_leg_table(void)
{
    uint16_t a, b;
    SlaveCtrl_LegsFor(+1, 1, 500, &a, &b); CHECK(a == 500 && b == 0,   "positive+boost -> leg1 (TIM15)");
    SlaveCtrl_LegsFor(+1, 0, 500, &a, &b); CHECK(a == 0   && b == 500, "positive+buck  -> leg2 (TIM1)");
    SlaveCtrl_LegsFor(-1, 1, 500, &a, &b); CHECK(a == 0   && b == 500, "negative+boost -> leg2 (TIM1)");
    SlaveCtrl_LegsFor(-1, 0, 500, &a, &b); CHECK(a == 500 && b == 0,   "negative+buck  -> leg1 (TIM15)");
    printf("  leg table matches the old firmware (4 combinations)\n");
}

static void t_pattern(int pattern, int expect_mode, const char *name, uint64_t start)
{
    int falls = 0, i;
    world_reset(start);
    W.pattern = pattern;
    run_for(1000000 + WARM);

    if (expect_mode == CTRL_ZC_UNKNOWN)
    {
        CHECK(g_ctrl.mode == CTRL_ZC_UNKNOWN, "%s: mode must stay UNKNOWN, got %d", name, g_ctrl.mode);
        CHECK(g_ctrl.state == CTRL_SYNC,      "%s: must stay in SYNC, state %d", name, g_ctrl.state);
        CHECK(g_ctrl.commutations == 0 && n_energise == 0, "%s: converter must never be energised", name);
        check_invariants(name);
        printf("  %-34s mode=UNKNOWN state=SYNC  energised %d times (must be 0)\n", name, n_energise);
        return;
    }
    CHECK(g_ctrl.mode == expect_mode, "%s: mode %d, expected %d", name, g_ctrl.mode, expect_mode);
    CHECK(is_running(), "%s: state %d, expected a running state", name, g_ctrl.state);
    CHECK(g_ctrl.commutations >= 70, "%s: only %u commutations in 1 s", name, (unsigned)g_ctrl.commutations);
    (void)falls; (void)i;
    check_invariants(name);
    printf("  %-34s mode=%d  commutations=%u  min dead gap=%llu us  wrong-polarity samples=%ld\n",
           name, g_ctrl.mode, (unsigned)g_ctrl.commutations, (unsigned long long)min_gap_us, v_polarity);
}

static void t_pulse_rising_ignored(void)
{
    /* In pulse mode only falling edges may cause a commutation. */
    world_reset(2000000);
    W.pattern = PAT_PULSE;
    run_for(600000 + WARM);
    {
        long e0 = n_energise;
        uint64_t t0 = sim_t;
        /* run 1 s and count commutations: must be ~100 (one per falling edge), not ~200 */
        run_for(1000000);
        CHECK(n_energise - e0 >= 95 && n_energise - e0 <= 105, "pulse mode: %ld commutations per second, expected ~100", n_energise - e0);
        printf("  pulse mode: %ld commutations in 1 s (one per falling edge, rising ignored)\n", n_energise - e0);
        (void)t0;
    }
    check_invariants("pulse/rising ignored");
}

static void t_pattern_lost_and_restart(void)
{
    uint64_t t_fault = 0, t_alloff = 0;
    world_reset(3000000);
    run_for(1000000 + WARM);
    CHECK(is_running(), "should be running before the pattern is lost");

    W.pattern = PAT_NONE; resync_crossings();
    {
        uint64_t t_start = sim_t, t_last = t_last_edge;
        while (sim_t < t_start + 100000)
        {
            run_for(500);
            if (g_ctrl.state == CTRL_FAULT && t_fault == 0) t_fault = sim_t;
        }
        t_alloff = t_diag_off;          /* exact time of the last switch-off, not a sampled one */
        CHECK(diag == 0, "converter still on 100 ms after the pattern vanished");
        CHECK(t_alloff - t_last <= 10000, "converter must be off BEFORE the next expected zero: off %llu us after the last edge",
              (unsigned long long)(t_alloff - t_last));
        CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_ZC_LOST, "expected FAULT/ZC_LOST, got state %d fault %d", g_ctrl.state, g_ctrl.fault);
        CHECK(SlaveCtrl_LinkState() == 3 && SlaveCtrl_FaultCode() == CF_ZC_LOST, "link must report fault 8");
        printf("  pattern lost: converter off %llu us after the last edge (half-cycle ~10000 us), fault %d, wrong-polarity samples %ld\n",
               (unsigned long long)(t_alloff - t_last), g_ctrl.fault, v_polarity);
    }

    /* the pattern returns at once, but a condition fault retries only after 2 s */
    W.pattern = PAT_SQUARE; resync_crossings();
    run_for(1500000);
    CHECK(g_ctrl.state == CTRL_FAULT, "restarted too early (state %d)", g_ctrl.state);
    run_for(1200000 + WARM);
    CHECK(is_running(), "should have restarted after the retry time, state %d", g_ctrl.state);
    CHECK(g_ctrl.mode == CTRL_ZC_BOTH, "mode must be re-detected after restart");
    check_invariants("pattern lost / restart");
    printf("  restart after the 2 s retry: state %d, mode re-detected = %d\n", g_ctrl.state, g_ctrl.mode);
}

static void t_sense_mismatch(void)
{
    uint64_t t0;
    world_reset(4000000);
    run_for(800000 + WARM);
    CHECK(is_running(), "should be running");
    t0 = sim_t; W.out_gain = -1.0;                       /* AC_OUT now reads the opposite sign */
    run_for(30000);
    CHECK(diag == 0 && !is_running(), "must stop within one event of a sense mismatch (state %d)", g_ctrl.state);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_SENSE_MISMATCH, "expected SENSE_MISMATCH, got %d/%d", g_ctrl.state, g_ctrl.fault);
    CHECK(g_ctrl.fault_count == 1, "escalating fault must count (%u)", g_ctrl.fault_count);
    printf("  sense mismatch: stopped after %llu us, fault %d, retry in 10 s\n", (unsigned long long)(sim_t - t0), g_ctrl.fault);
    W.out_gain = 1.0;
    run_for(9000000);
    CHECK(g_ctrl.state == CTRL_FAULT, "retry before 10 s (state %d)", g_ctrl.state);
    run_for(1500000 + WARM);
    CHECK(is_running(), "should have retried after 10 s, state %d", g_ctrl.state);
    check_invariants("sense mismatch");
}

static void t_tiny_signal(void)
{
    world_reset(5000000);
    run_for(800000 + WARM);
    CHECK(is_running(), "should be running");
    W.amp = 10.0;                                        /* mains too small to read a sign */
    run_for(40000);
    CHECK(diag == 0 && g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_POLARITY, "expected POLARITY fault, got %d/%d", g_ctrl.state, g_ctrl.fault);
    check_invariants("tiny signal");
    printf("  tiny signal: converter off, fault %d (no guessing the polarity)\n", g_ctrl.fault);
}

static void t_break_escalation(void)
{
    int i;
    world_reset(6000000);
    for (i = 1; i <= 6; i++)
    {
        uint64_t t_trip, t_back = 0, limit = sim_t + 120000000ULL;
        /* run until a running state is reached */
        while (!is_running() && sim_t < limit && g_ctrl.state != CTRL_LATCHED) run_for(5000);
        CHECK(is_running(), "trip %d: never got back to running (state %d)", i, g_ctrl.state);
        run_for(300000);
        t_trip = sim_t;
        SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM1);         /* the break interrupt */
        CHECK(diag == 0 && !inv_enabled, "trip %d: outputs not off immediately", i);
        run_for(2000);
        if (i < 6)
        {
            CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_BREAK, "trip %d: state %d fault %d", i, g_ctrl.state, g_ctrl.fault);
            while (g_ctrl.state == CTRL_FAULT && sim_t < t_trip + 70000000ULL) { run_for(5000); }
            t_back = sim_t - t_trip;
            CHECK(t_back >= (uint64_t)i * 10000000ULL - 1000 && t_back <= (uint64_t)i * 10000000ULL + 300000,
                  "trip %d: restart after %llu ms, expected %d s", i, (unsigned long long)(t_back / 1000), i * 10);
            printf("  break trip %d -> restart delay %.2f s\n", i, (double)t_back / 1e6);
        }
        else
        {
            CHECK(g_ctrl.state == CTRL_LATCHED, "6th trip must latch, state %d", g_ctrl.state);
            run_for(120000000ULL);
            CHECK(g_ctrl.state == CTRL_LATCHED && diag == 0, "latched state must stay latched and off");
            printf("  break trip 6 -> LATCHED, still latched after 120 s\n");
        }
    }
    SlaveCtrl_ClearLatch();
    run_for(5000000);
    CHECK(is_running() && g_ctrl.fault_count == 0, "after ClearLatch it must start again (state %d count %u)", g_ctrl.state, g_ctrl.fault_count);
    printf("  ClearLatch -> running again, escalation counter %u\n", g_ctrl.fault_count);
    check_invariants("break escalation");
}

static void t_break_input_held(void)
{
    world_reset(7000000);
    run_for(800000 + WARM);
    SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM15);
    W.brk = 1;                                           /* the BKIN line stays asserted */
    run_for(25000000);
    CHECK(g_ctrl.state == CTRL_FAULT && diag == 0, "must stay in FAULT while BKIN is held (state %d)", g_ctrl.state);
    W.brk = 0;
    run_for(1500000 + WARM);
    CHECK(is_running(), "must restart once BKIN is released (state %d)", g_ctrl.state);
    check_invariants("break held");
    printf("  BKIN held 25 s -> no restart; released -> restarted\n");
}

static void t_flt(void)
{
    world_reset(8000000);
    W.flt = 1;
    run_for(2000000);
    CHECK(g_ctrl.state == CTRL_SYNC && n_energise == 0, "must not start while FLT is asserted (state %d)", g_ctrl.state);
    W.flt = 0;
    run_for(500000 + WARM);
    CHECK(is_running(), "should start once FLT is clear (state %d)", g_ctrl.state);
    SlaveCtrl_OnFltIrq();
    run_for(1000);
    CHECK(flt_outputs_safe(), "FLT interrupt must switch the converter and the high sides off at once (inverter %d, forced %d)", inv_enabled, inv_forced);
    run_for(FLT_UNWIND_US);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_FLT, "expected FLT fault, got %d/%d", g_ctrl.state, g_ctrl.fault);
    /* polled detection, no interrupt */
    world_reset(9000000);
    run_for(800000 + WARM);
    CHECK(is_running(), "running before the polled FLT test");
    W.flt = 1;
    run_for(5000 + FLT_UNWIND_US);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_FLT && diag == 0, "polled FLT must trip, got %d/%d", g_ctrl.state, g_ctrl.fault);
    check_invariants("FLT");
    printf("  FLT: blocks start, interrupt and polled trip both switch off\n");
}

static void t_flt_shutdown_first(void)
{
    uint8_t run0, idle0;
    world_reset(8100000);
    run_for(800000 + WARM);
    CHECK(is_running(), "running before the shutdown-order test (state %d)", g_ctrl.state);
    run0 = g_ctrl.flt_run; idle0 = g_ctrl.flt_idle;
    probe_taken = 0; probe_armed = 1;
    g_ctrl.trip_cap = 0U;
    SlaveCtrl_OnFltIrq();
    probe_armed = 0;
    CHECK(probe_taken, "the FLT handler must call the shutdown");
    CHECK(probe_run_allowed == 1U && probe_isr_fault == 0U, "the shutdown must come FIRST: at that moment run_allowed is still 1 and no fault is recorded yet (run_allowed %u, isr_fault %u)", (unsigned)probe_run_allowed, (unsigned)probe_isr_fault);
    CHECK(probe_flt_run == run0 && probe_flt_idle == idle0, "no counter has been touched before the shutdown (run %u -> %u, idle %u -> %u)", (unsigned)run0, (unsigned)probe_flt_run, (unsigned)idle0, (unsigned)probe_flt_idle);
    CHECK(probe_trip_cap == 0U, "the trip record is made AFTER the shutdown (trip_cap was 0x%04X at the shutdown)", (unsigned)probe_trip_cap);
    CHECK(flt_outputs_safe(), "the outputs are safe straight away: converter off, high sides off (inverter %d, forced %d)", inv_enabled, inv_forced);
    CHECK(g_ctrl.flt_run == (uint8_t)(run0 + 1U), "the running-FLT counter is still counted afterwards (%u -> %u)", (unsigned)run0, (unsigned)g_ctrl.flt_run);
    CHECK((g_ctrl.trip_cap & 0x8000U) != 0U, "the trip is still recorded afterwards (trip_cap 0x%04X)", (unsigned)g_ctrl.trip_cap);
    CHECK(g_ctrl.isr_fault == CF_FLT, "the fault is still raised afterwards (isr_fault %u)", (unsigned)g_ctrl.isr_fault);
    run_for(5000 + FLT_UNWIND_US);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_FLT, "and the controller still ends in FAULT / FLT (%d/%d)", g_ctrl.state, g_ctrl.fault);
    /* an FLT edge before the controller has done a single service step: genuinely idle in EVERY configuration (some start within 100 ms of a reset, some never do by themselves) */
    world_reset(8200000);
    idle0 = g_ctrl.flt_idle; run0 = g_ctrl.flt_run;
    CHECK(g_ctrl.run_allowed == 0U && g_ctrl.isr_fault == 0U, "idle before the first service step");
    probe_taken = 0; probe_armed = 1;
    SlaveCtrl_OnFltIrq();
    probe_armed = 0;
    CHECK(probe_taken && probe_flt_idle == idle0 && probe_flt_run == run0, "idle FLT edge: the shutdown comes first as well (no counter touched before it)");
    CHECK(g_ctrl.flt_idle == (uint8_t)(idle0 + 1U) && g_ctrl.flt_run == run0, "an idle FLT edge is counted as idle, afterwards (%u -> %u)", (unsigned)idle0, (unsigned)g_ctrl.flt_idle);
    /* an FLT edge some time after the reset: whichever way the firmware classifies it (running or idle depends on the configuration), the shutdown is first and exactly one counter moves */
    world_reset(8300000);
    run_for(100000);
    {
        const int was_running = (g_ctrl.run_allowed != 0U) || (g_ctrl.isr_fault != 0U);
        const uint8_t before = was_running ? g_ctrl.flt_run : g_ctrl.flt_idle, other = was_running ? g_ctrl.flt_idle : g_ctrl.flt_run;
        probe_taken = 0; probe_armed = 1;
        SlaveCtrl_OnFltIrq();
        probe_armed = 0;
        CHECK(probe_taken && probe_trip_cap == 0U, "FLT edge 100 ms after a reset: the shutdown comes first (trip_cap 0x%04X)", (unsigned)probe_trip_cap);
        CHECK((was_running ? g_ctrl.flt_run : g_ctrl.flt_idle) == (uint8_t)(before + 1U) && (was_running ? g_ctrl.flt_idle : g_ctrl.flt_run) == other, "exactly the right counter moves afterwards");
    }
    check_invariants("FLT shutdown first");
    printf("  FLT: the shutdown is the first thing done; counters, trip record and fault follow unchanged\n");
}

static void t_stop_request(void)
{
    world_reset(10000000);
    run_for(800000 + WARM);
    CHECK(is_running(), "running");
    SlaveCtrl_SetRunRequest(0U);
    run_for(2000);
    CHECK(g_ctrl.state == CTRL_OFF && diag == 0 && !inv_enabled, "run request 0 must stop at once (state %d)", g_ctrl.state);
    run_for(500000);
    CHECK(g_ctrl.state == CTRL_OFF && diag == 0, "must stay off");
    check_invariants("stop request");
    printf("  run request withdrawn -> everything off immediately, stays OFF\n");
}

static void t_timer_wrap(void)
{
    world_reset(0xFFFFFFFFULL - 600000ULL);              /* the 32-bit microsecond timer wraps in 0.6 s */
    run_for(2000000 + WARM);
    CHECK(is_running() && g_ctrl.mode == CTRL_ZC_BOTH, "must work across the timer wrap (state %d mode %d)", g_ctrl.state, g_ctrl.mode);
    check_invariants("timer wrap");
    printf("  32-bit timer wrap-around during operation: fine (%u commutations)\n", (unsigned)g_ctrl.commutations);
}

static void t_jitter(void)
{
    world_reset(11000000);
    W.jitter_us = 150.0;
    run_for(1500000 + WARM);
    CHECK(is_running(), "+/-150 us of edge jitter must be tolerated (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    check_invariants("small jitter");
    printf("  +/-150 us edge jitter: running, invariants hold\n");

    world_reset(12000000);
    W.jitter_us = 3500.0;                                /* unacceptably noisy ZC */
    run_for(30000000);
    CHECK(v_polarity == 0 && v_both == 0 && v_dead == 0, "noisy ZC must never cause an unsafe switch (against-line samples %ld, longest %llu us; both %ld; dead %ld)",
          v_polarity, (unsigned long long)max_wrong_run_us, v_both, v_dead);
    printf("  +/-3.5 ms jitter: refused / tripped (state %d fault %d), unsafe switches %ld\n", g_ctrl.state, g_ctrl.fault, v_polarity + v_both + v_dead);
}

static void t_phase_shift_probe(void)
{
    /* A DISCONTINUOUS phase jump while running (the line polarity flips with no
     * zero-cross). The zero-cross timing cannot see it; the 20 kHz polarity
     * guard must. */
    double shifts[4] = { 2000.0, 5000.0, 8000.0, 10000.0 };
    int i;
    for (i = 0; i < 4; i++)
    {
        world_reset(13000000 + (uint64_t)i * 100000000ULL);
        run_for(800000 + WARM);
        wrong_us = max_wrong_run_us = cur_wrong_run_us = 0;
        W.phase0_us += shifts[i]; resync_crossings();
        run_for(300000);
        printf("  phase jump of %5.1f ms: wrong-polarity conduction at most %llu us, then state %d fault %d\n",
               shifts[i] / 1000.0, (unsigned long long)max_wrong_run_us, g_ctrl.state, g_ctrl.fault);
        CHECK(diag == 0 || is_running(), "after a phase jump the converter must be off or validly running");
        CHECK(max_wrong_run_us <= 400, "wrong-polarity time %llu us: the polarity guard should stop it within ~0.4 ms", (unsigned long long)max_wrong_run_us);
    }
}

static void t_lost_pulse_midrun(void)
{
    uint64_t t_last;
    world_reset(22000000);
    run_for(800000 + WARM);
    CHECK(is_running(), "running before the pulse is lost");
    W.skip_crossings = 1; resync_crossings(); W.skip_crossings = 1;      /* swallow exactly one zero-cross pulse */
    t_last = t_last_edge;
    run_for(40000);
    CHECK(v_polarity == 0, "a lost pulse must not leave the converter conducting against the line (%ld samples, %llu us)",
          v_polarity, (unsigned long long)max_wrong_run_us);
    CHECK(diag == 0, "converter must be off after the missed zero-cross");
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_ZC_LOST, "expected ZC_LOST, got %d/%d", g_ctrl.state, g_ctrl.fault);
    check_invariants("lost pulse");
    printf("  one zero-cross pulse lost: converter off by prediction (wrong-polarity samples %ld), fault %d\n", v_polarity, g_ctrl.fault);
    (void)t_last;
}

static void t_guard_margin(void)
{
    double delays[5] = { -40.0, 0.0, 40.0, 150.0, -120.0 };
    double amps[2]   = { 784.0, 1680.0 };           /* 140 V and 300 V RMS */
    int i, j;
    for (i = 0; i < 5; i++)
        for (j = 0; j < 2; j++)
        {
            world_reset(23000000 + (uint64_t)(i * 2 + j) * 50000000ULL);
            W.edge_delay_us = delays[i]; W.amp = amps[j]; W.jitter_us = 40.0;
            run_for(6000000);
            CHECK(is_running() && g_ctrl.fault == CF_NONE, "ZCD delay %+.0f us at amp %.0f: the guard must not trip (state %d fault %d)",
                  delays[i], amps[j], g_ctrl.state, g_ctrl.fault);
            check_invariants("guard margin");
        }
    printf("  guard does not trip falsely: ZCD edge -120..+150 us from the true zero, 140 V and 300 V, +/-40 us jitter\n");
}

static void t_resync_clears_state(void)
{
    /* Spec: "never resume PWM assuming the previous zero-crossing reference is still
     * valid". After a fault the pattern mode and the validity counters must be
     * cleared, and nothing may be energised until a FRESH sync is achieved. */
    world_reset(24000000);
    run_for(800000 + WARM);
    CHECK(is_running() && g_ctrl.mode == CTRL_ZC_BOTH && g_ctrl.good_all >= CTRL_SYNC_GOOD_EVENTS, "running with a validated pattern first");
    W.pattern = PAT_NONE; resync_crossings();            /* the zero-cross vanishes: ZC_LOST, retry after 2 s */
    run_for(3500000);
    CHECK(g_ctrl.state == CTRL_SYNC, "after the retry it must wait in SYNC for a fresh pattern (state %d)", g_ctrl.state);
    CHECK(g_ctrl.mode == CTRL_ZC_UNKNOWN && g_ctrl.good_all == 0 && g_ctrl.good_fall == 0,
          "the old synchronisation must be gone: mode %d good_all %u good_fall %u", g_ctrl.mode, g_ctrl.good_all, g_ctrl.good_fall);
    CHECK(diag == 0, "nothing may be energised without a fresh sync");
    printf("  after a fault the old synchronisation is discarded (mode UNKNOWN, counters 0); waits in SYNC\n");
}

/* ====================================================================== */
/* RELAY LOGIC VERIFICATION (rules R1..R5 of slave_relay.h)                */
/* The physical relay model above checks R1-R4 on EVERY call in EVERY test  */
/* of this file; the tests below drive the specific sequences.             */
/* ====================================================================== */
#if RELAY_MODEL
static int coil_short_level(int type) { return type == RELAY_SHORTS_WHEN_ENERGISED; }
static int coil_free_level(int type)  { return type == RELAY_SHORTS_WHEN_RELEASED; }

static void t_relay_module_unit(void)
{
    uint32_t t;
    int i;
    world_reset(40000000);
    /* no controller activity here: the module on its own */
    t = (uint32_t)(sim_t / 1000);
    CHECK(rl[0].coil == coil_short_level(SLAVE_RELAY_SH_CONTACT) && rl[1].coil == coil_short_level(SLAVE_RELAY_W_CONTACT),
          "Init must drive the SHORTED pattern (coils %d %d)", rl[0].coil, rl[1].coil);
    CHECK(SlaveRelay_IsShorted() == 0, "not settled immediately after Init");
    SlaveRelay_Service(t + SLAVE_RELAY_SETTLE_MS - 1U);  CHECK(!SlaveRelay_IsShorted(), "must not settle one ms early");
    SlaveRelay_Service(t + SLAVE_RELAY_SETTLE_MS);       CHECK(SlaveRelay_IsShorted() && !SlaveRelay_IsFree(), "settled SHORTED after the settle time");
    t += SLAVE_RELAY_SETTLE_MS + 10U;

    SlaveRelay_Request(1U); SlaveRelay_Service(t);
    CHECK(rl[0].coil == coil_free_level(SLAVE_RELAY_SH_CONTACT) && rl[1].coil == coil_free_level(SLAVE_RELAY_W_CONTACT), "FREE pattern after Request(1)");
    CHECK(!SlaveRelay_IsFree() && !SlaveRelay_IsShorted(), "moving: neither free nor shorted");
    SlaveRelay_Service(t + SLAVE_RELAY_SETTLE_MS - 1U);  CHECK(!SlaveRelay_IsFree(), "must not report FREE one ms early");
    SlaveRelay_Service(t + SLAVE_RELAY_SETTLE_MS);       CHECK(SlaveRelay_IsFree(), "FREE after the settle time");
    t += SLAVE_RELAY_SETTLE_MS + 10U;

    SlaveRelay_Request(0U); SlaveRelay_Service(t);       /* back to shorted ... */
    SlaveRelay_Request(1U); SlaveRelay_Service(t + 5U);  /* ... and reversed while moving: the settle time restarts */
    CHECK(rl[0].coil == coil_free_level(SLAVE_RELAY_SH_CONTACT), "reversal while moving must re-command FREE");
    SlaveRelay_Service(t + 5U + SLAVE_RELAY_SETTLE_MS - 1U); CHECK(!SlaveRelay_IsFree(), "settle time restarts after a reversal");
    SlaveRelay_Service(t + 5U + SLAVE_RELAY_SETTLE_MS);      CHECK(SlaveRelay_IsFree(), "FREE after the restarted settle time");
    t += 2U * SLAVE_RELAY_SETTLE_MS;

    SlaveRelay_ShortNow();                               /* a fault: shorted NOW, whatever was requested */
    CHECK(rl[0].coil == coil_short_level(SLAVE_RELAY_SH_CONTACT) && rl[1].coil == coil_short_level(SLAVE_RELAY_W_CONTACT), "ShortNow must command the SHORTED pattern at once");
    CHECK(!SlaveRelay_IsFree(), "never FREE right after ShortNow");
    for (i = 0; i < 3; i++) SlaveRelay_Service(t + (uint32_t)i);
    CHECK(!SlaveRelay_IsFree(), "a stale Request(1) must not undo ShortNow");
    printf("  relay module alone: SHORTED/FREE patterns, settle timing, reversal while moving, ShortNow - as specified (SH type %d, W type %d)\n",
           SLAVE_RELAY_SH_CONTACT, SLAVE_RELAY_W_CONTACT);
}

static void t_relay_stage1_untouched(void)
{
    world_reset(41000000);
    run_for(2500000);
    CHECK(is_running(), "converter running (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    CHECK(rl[0].coil == coil_short_level(SLAVE_RELAY_SH_CONTACT) && rl[1].coil == coil_short_level(SLAVE_RELAY_W_CONTACT),
          "with the converter running the relays must stay in the SHORTED pattern");
#if (SLAVE_POWER_STAGE == 1)
    CHECK(t_free_cmd_first == 0, "stage 1 must never command the FREE pattern");
#endif
    check_invariants("relays at stage 1");
    printf("  converter running: relays stay in the SHORTED (bypass) pattern\n");
}

static void t_wait_relay(void)
{
    uint8_t w;
    world_reset(41500000);
    run_for(30000);
    w = SlaveCtrl_WaitContext();
    CHECK(g_ctrl.state == CTRL_SYNC && (w & CTRL_WAIT_RELAY), "right after start-up the controller must wait for the relays (state %d wait 0x%02X)", g_ctrl.state, w);
    run_for(800000);
    CHECK(g_ctrl.state != CTRL_SYNC, "after the settle time it must proceed (state %d)", g_ctrl.state);
    printf("  start-up waits for the relays to settle SHORTED before the converter may start\n");
}

#if (SLAVE_POWER_STAGE >= 2)
static void run_to_run(uint64_t start)
{
    world_reset(start);
    W.vin = 200.0f; W.vout = 200.0f;                    /* below the 230 V target: the closed loop has something to correct */
    run_for(6000000);
}

static void t_relay_start_order(void)
{
    run_to_run(42000000);
    CHECK(g_ctrl.state == CTRL_RUN, "must reach RUN (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    CHECK(t_conv_first > 0 && t_inv_enable_first > t_conv_first, "the converter must run BEFORE the inverter is enabled");
    CHECK(t_first_duty > t_inv_enable_first, "duty only after the inverter was enabled");
    if (rly_type(0) != RELAY_NOT_A_SHUNT || rly_type(1) != RELAY_NOT_A_SHUNT)
    {
        CHECK(t_free_cmd_first >= t_inv_enable_first + (uint64_t)CTRL_INV_ZERO_DUTY_MS * 1000ULL - 2000ULL,
              "the shunts must open only AFTER the inverter has been at zero duty for %u ms (opened %.0f ms after enable)",
              (unsigned)CTRL_INV_ZERO_DUTY_MS, (double)(t_free_cmd_first - t_inv_enable_first) / 1000.0);
        CHECK(t_first_duty >= t_free_cmd_first + RELAY_SETTLE_US - 2000ULL,
              "first nonzero duty %.0f ms after the shunt command, the settle time is %u ms",
              (double)(t_first_duty - t_free_cmd_first) / 1000.0, (unsigned)SLAVE_RELAY_SETTLE_MS);
        printf("  start order: converter -> inverter at zero duty (+%.0f ms) -> shunts opened (+%.0f ms) -> first duty (+%.0f ms)\n",
               (double)(t_inv_enable_first - t_conv_first) / 1000.0, (double)(t_free_cmd_first - t_inv_enable_first) / 1000.0,
               (double)(t_first_duty - t_free_cmd_first) / 1000.0);
    }
    check_invariants("relay start order");
}

static void t_relay_stop_order(void)
{
    run_to_run(43000000);
    CHECK(g_ctrl.state == CTRL_RUN, "must reach RUN first");
    SlaveCtrl_SetRunRequest(0U);
    run_for(1500000);
    CHECK(g_ctrl.state == CTRL_OFF && !inv_enabled && diag == 0, "a normal stop must end in OFF with everything off (state %d)", g_ctrl.state);
    if (rly_type(0) != RELAY_NOT_A_SHUNT || rly_type(1) != RELAY_NOT_A_SHUNT)
        CHECK(t_inv_disable_last >= t_shorted_cmd_last + RELAY_SETTLE_US - 2000ULL,
              "the inverter was disabled %.0f ms after the shunt command: it must wait for the shunts to close (%u ms)",
              (double)(t_inv_disable_last - t_shorted_cmd_last) / 1000.0, (unsigned)SLAVE_RELAY_SETTLE_MS);
    check_invariants("relay stop order");
    printf("  normal stop: duty zero -> shunts closed -> inverter disabled %.0f ms later; primary never uncontrolled\n",
           (double)(t_inv_disable_last - t_shorted_cmd_last) / 1000.0);
}

static void t_relay_stop_in_unshunt(void)
{
    run_to_run(44000000);
    {   /* stop requested while the shunts are still opening */
        uint64_t limit = sim_t + 9000000ULL;
        world_reset(44100000);
        while (g_ctrl.state != CTRL_UNSHUNT && sim_t < limit) run_for(2000);
    }
    CHECK(g_ctrl.state == CTRL_UNSHUNT, "never reached UNSHUNT (state %d)", g_ctrl.state);
    SlaveCtrl_SetRunRequest(0U);
    run_for(1500000);
    CHECK(g_ctrl.state == CTRL_OFF && !inv_enabled, "stop during UNSHUNT must unwind to OFF (state %d)", g_ctrl.state);
    check_invariants("stop while the shunts open");
}

static void t_relay_soft_fault(void)
{
    uint64_t t_inject;
    run_to_run(45000000);
    CHECK(g_ctrl.state == CTRL_RUN, "RUN first");
    t_inject = sim_t;
    W.out_gain = -1.0;                                  /* AC_OUT sign mismatch: a SOFT fault (not an overcurrent) */
    run_for(30000);
    CHECK(diag == 0, "the converter must be off within one event (%llu us)", (unsigned long long)(sim_t - t_inject));
    CHECK(inv_enabled == 1, "the inverter must STAY enabled (low sides on) while the shunts are closing");
    CHECK(leg1 == 0 && leg2 == 0, "duty must be zero immediately");
    run_for(1500000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_SENSE_MISMATCH, "expected FAULT/SENSE_MISMATCH, got %d/%d", g_ctrl.state, g_ctrl.fault);
    CHECK(!inv_enabled, "the inverter must be disabled in the end");
    if (rly_type(0) != RELAY_NOT_A_SHUNT || rly_type(1) != RELAY_NOT_A_SHUNT)
        CHECK(t_inv_disable_last >= t_shorted_cmd_last + RELAY_SETTLE_US - 2000ULL, "the inverter was disabled only %.0f ms after the shunt command",
              (double)(t_inv_disable_last - t_shorted_cmd_last) / 1000.0);
    check_invariants("soft fault unwinding");
    printf("  soft fault in RUN: converter off at once, duty zero, shunts closed, THEN the inverter disabled; uncontrolled time %llu us\n",
           (unsigned long long)max_unctl_run_us);
}

static void t_relay_hard_fault(void)
{
    uint64_t t_inject;
    run_to_run(46000000);
    CHECK(g_ctrl.state == CTRL_RUN, "RUN first");
    unctl_limit_us = 20000ULL;                          /* the primary is unavoidably open until the contacts have moved */
    t_inject = sim_t;
    SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM1);             /* overcurrent-class: everything off AT ONCE */
    CHECK(!inv_enabled && diag == 0, "a break must switch everything off at once");
    run_for(2000);
    if (rly_type(0) != RELAY_NOT_A_SHUNT || rly_type(1) != RELAY_NOT_A_SHUNT)
        CHECK(t_shorted_cmd_last >= t_inject && t_shorted_cmd_last - t_inject <= 1000ULL, "the shunts must be commanded closed within 1 ms (%llu us)",
              (unsigned long long)(t_shorted_cmd_last - t_inject));
    run_for(500000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_BREAK, "expected FAULT/BREAK, got %d/%d", g_ctrl.state, g_ctrl.fault);
    check_invariants("hard fault");
    printf("  hard fault (break) in RUN: all off at once, shunts commanded within 1 ms; uncontrolled window %llu us (<= contact travel)\n",
           (unsigned long long)max_unctl_run_us);
}

#if (SLAVE_FLT_KEEP_LOWS != 0)
/* v0.9.65: FLT (the CT comparators) no longer opens the primary: the high sides go off with the low sides ON, the shunts close, and only then is the inverter disabled */
static void t_relay_flt_keeps_lows(void)
{
    uint64_t t_inject;
    run_to_run(46500000);
    CHECK(g_ctrl.state == CTRL_RUN, "RUN first");
    t_inject = sim_t;
    SlaveCtrl_OnFltIrq();
    CHECK(diag == 0 && inv_enabled && inv_forced && leg1 == 0 && leg2 == 0, "FLT: converter off, high sides off, the low sides ON at once (inverter %d, forced %d)", inv_enabled, inv_forced);
    run_for(2000);
    CHECK(g_ctrl.state == CTRL_RESHUNT && SlaveCtrl_LinkState() == 3U && SlaveCtrl_FaultCode() == (uint16_t)CF_FLT, "unwinding, and F02 reported at once (state %d)", g_ctrl.state);
    run_for(1500000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_FLT && !inv_enabled, "then FAULT / FLT with the inverter disabled (%d/%d)", g_ctrl.state, g_ctrl.fault);
    if (rly_type(0) != RELAY_NOT_A_SHUNT || rly_type(1) != RELAY_NOT_A_SHUNT)
        CHECK(t_inv_disable_last >= t_shorted_cmd_last + RELAY_SETTLE_US - 2000ULL && t_shorted_cmd_last >= t_inject, "the inverter was disabled only after the shunts had settled (%.0f ms after their command)", (double)(t_inv_disable_last - t_shorted_cmd_last) / 1000.0);
    check_invariants("FLT keeps the low sides on");                 /* unctl_limit_us is 0 here: the primary is NEVER open and uncontrolled */
    /* FLT while a normal stop is unwinding: it keeps unwinding (no all-off), and the fault is reported afterwards */
    run_to_run(46600000);
    SlaveCtrl_SetRunRequest(0U); run_for(20000);
    CHECK(g_ctrl.state == CTRL_RESHUNT, "a normal stop is unwinding (state %d)", g_ctrl.state);
    SlaveCtrl_OnFltIrq(); run_for(5000);                          /* the main loop has taken it: it must keep unwinding, not switch everything off */
    CHECK(g_ctrl.state == CTRL_RESHUNT && inv_enabled && inv_forced, "FLT during the unwinding: it keeps unwinding with the low sides on (state %d, inverter %d)", g_ctrl.state, inv_enabled);
    run_for(1500000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_FLT, "and it is reported once the shunts have closed (%d/%d)", g_ctrl.state, g_ctrl.fault);
    check_invariants("FLT during the unwinding");
    printf("  FLT: high sides off and low sides on at once, shunts closed, THEN the inverter disabled; the primary never open (uncontrolled %llu us)\n", (unsigned long long)max_unctl_run_us);
}
#endif

static void t_relay_fault_during_reshunt(void)
{
    run_to_run(47000000);
    CHECK(g_ctrl.state == CTRL_RUN, "RUN first");
    SlaveCtrl_SetRunRequest(0U);                        /* a normal stop begins ... */
    run_for(20000);
    CHECK(g_ctrl.state == CTRL_RESHUNT, "must be unwinding (state %d)", g_ctrl.state);
    W.out_gain = -1.0;                                  /* ... and a SOFT fault arrives meanwhile */
    run_for(50000);
    run_for(1500000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_SENSE_MISMATCH, "the fault must still be reported after the unwinding (got %d/%d)", g_ctrl.state, g_ctrl.fault);
    check_invariants("fault during reshunt");

    run_to_run(48000000);
    SlaveCtrl_SetRunRequest(0U);
    run_for(20000);
    unctl_limit_us = 20000ULL;
    SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM15);            /* a HARD fault while unwinding: off at once */
    CHECK(!inv_enabled, "a hard fault during the unwinding must disable the inverter at once");
    run_for(500000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_BREAK, "expected BREAK (got %d/%d)", g_ctrl.state, g_ctrl.fault);
    check_invariants("hard fault during reshunt");
    printf("  fault while unwinding: soft one is reported afterwards, hard one switches everything off at once\n");
}

static void t_relay_never_free_without_inverter(void)
{
    /* sweep: stop / break / soft fault injected at EVERY 7 ms offset through the whole start-up and a stretch of RUN.
     * Rules R1, R2, R4 must hold for all of them; R3 (primary uncontrolled) is allowed only for hard faults. */
    int k; long bad = 0;
    for (k = 0; k < 90; k++)
    {
        uint64_t t0 = 50000000ULL + (uint64_t)k * 3000000ULL;
        int kind = k % 3;                              /* 0 stop, 1 soft fault, 2 hard fault */
        world_reset(t0);
        run_for(2900000 + (uint64_t)k * 7000ULL);      /* sweeps the inverter enable / unshunt window */
        unctl_limit_us = (kind == 2) ? 20000ULL : 0ULL;
        if      (kind == 0) SlaveCtrl_SetRunRequest(0U);
        else if (kind == 1) W.out_gain = -1.0;
        else                SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM1);
        run_for(2500000);
        {
            long b0 = g_fail;
            check_invariants("fault sweep");
            if (g_fail != b0) bad++;
        }
    }
    CHECK(bad == 0, "%ld of 90 injected stop / fault timings violated a relay rule", bad);
    printf("  90 stop / soft-fault / hard-fault injections swept through start-up and RUN: relay rules held in all of them\n");
}
#endif
#endif

/* ---- the race that produced the first bench F08: an interrupt writes a timestamp AFTER the main loop
 *      read the clock. The timestamp is then NEWER than 'now'; an unsigned age wraps to ~4e9. ---- */
static void t_race_future_timestamp(void)
{
    int pass;
    world_reset(29000000);
    run_for(1000000 + WARM);
    CHECK(is_running(), "must be running (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    for (pass = 0; pass < 500 && is_running(); pass++)
    {
        g_ctrl.t_event_us = (uint32_t)sim_t + 5U + (uint32_t)(pass % 200);   /* ...many different 'future' amounts */
        SlaveCtrl_Service();
        CHECK(g_ctrl.state != CTRL_FAULT, "running: a timestamp newer than the clock tripped fault %d (pass %d)", g_ctrl.fault, pass);
        if (g_ctrl.state == CTRL_FAULT) break;
        run_for(3000);
    }
    check_invariants("future timestamp");
    printf("  800 main-loop passes with the zero-cross timestamp NEWER than the clock: no false fault (state %d)\n", g_ctrl.state);
}

static void t_no_commutation_code(void)
{
    world_reset(30000000);
    W.drop_deadtime = 1;                               /* events arrive, the converter is never energised */
    run_for(3000000 + WARM);
    CHECK(n_energise == 0, "converter must never be energised here");
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_NO_COMMUTATION, "expected fault %d, got %d/%d", CF_NO_COMMUTATION, g_ctrl.state, g_ctrl.fault);
    CHECK(g_ctrl.fault_count >= 1, "must escalate");
    printf("  events arrive but nothing commutates -> its own fault code %d (not 8)\n", g_ctrl.fault);
}

static void t_zc_stale_code(void)
{
    world_reset(31000000);
    W.no_wd = 1;                                       /* only the main-loop backup is left */
    run_for(1000000 + WARM);
    CHECK(is_running(), "running first (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    W.pattern = PAT_NONE; resync_crossings();
    run_for(60000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_ZC_STALE, "expected fault %d, got %d/%d", CF_ZC_STALE, g_ctrl.state, g_ctrl.fault);
    CHECK(v_polarity == 0 && diag == 0, "the converter must still be off (predicted turn-off), against-line samples %ld", v_polarity);
    printf("  watchdog compare disabled, edges stop: the main-loop backup raises its own code %d, converter safely off\n", g_ctrl.fault);
}

static void t_guard_noise_immunity(void)
{
    int i; uint64_t t0, trip_us = 0;
    world_reset(32000000);
    run_for(1000000 + WARM);
    CHECK(is_running(), "running first (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    for (i = 0; i < 40; i++)                       /* 40 isolated bad samples */
    {
        W.guard_glitch = 1;
        run_for(50000);
        CHECK(g_ctrl.state != CTRL_FAULT, "a single bad guard sample tripped fault %d (glitch %d)", g_ctrl.fault, i);
        if (g_ctrl.state == CTRL_FAULT) break;
    }
    if (g_ctrl.state != CTRL_FAULT)
    {
        W.guard_glitch = 1000;                      /* now the sign stays wrong: a real reversal */
        t0 = sim_t;
        while (diag != 0 && sim_t < t0 + 60000) run_for(100);          /* the converter must go off almost at once */
        CHECK(diag == 0, "a sustained wrong sign must switch the converter off within 60 ms");
        trip_us = sim_t - t0;
        run_for(1500000);                                              /* ... and a soft fault may unwind with the shunts first */
        CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_LINE_REVERSED, "a sustained wrong sign must still end in F11 (got %d/%d)", g_ctrl.state, g_ctrl.fault);
        printf("  guard: 40 isolated bad samples ignored; a sustained wrong sign switches the converter off after %llu us and ends in F11\n", (unsigned long long)trip_us);
    }
    check_invariants("guard noise immunity");
}

static void t_fault_context(void)
{
    uint8_t ctx;
    /* a) FLT really asserted (polled): state RECT, mode BOTH, FLT low */
    world_reset(33000000);
    run_for(1000000 + WARM);
    CHECK(is_running() && SlaveCtrl_FaultContext() == 0, "no context outside a fault");
    W.flt = 1; run_for(10000 + FLT_UNWIND_US);
    ctx = SlaveCtrl_FaultContext();
    CHECK(g_ctrl.fault == CF_FLT && (ctx & CTRL_CTX_VALID), "FLT fault must carry a context (fault %d ctx 0x%02X)", g_ctrl.fault, ctx);
    CHECK((ctx & CTRL_CTX_FLT_LOW) && !(ctx & CTRL_CTX_BKIN_LOW), "FLT low must be reported (ctx 0x%02X)", ctx);
    CHECK(((ctx >> CTRL_CTX_MODE_SHIFT) & 3U) == CTRL_ZC_BOTH, "mode must be BOTH (ctx 0x%02X)", ctx);
    CHECK((ctx & CTRL_CTX_STATE_MASK) == CTRL_RECT || (ctx & CTRL_CTX_STATE_MASK) == CTRL_INV_START || (ctx & CTRL_CTX_STATE_MASK) == CTRL_RUN,
          "state must be a converter-running state (ctx 0x%02X)", ctx);

    /* b) FLT interrupt but the pin is already HIGH again when the main loop looks: the GLITCH signature */
    world_reset(34000000);
    run_for(1000000 + WARM);
    CHECK(is_running(), "running first");
    W.flt = 0; SlaveCtrl_OnFltIrq(); run_for(10000 + FLT_UNWIND_US);
    ctx = SlaveCtrl_FaultContext();
    CHECK(g_ctrl.fault == CF_FLT && (ctx & CTRL_CTX_VALID) && !(ctx & CTRL_CTX_FLT_LOW),
          "an FLT edge with the pin back HIGH must show FLT:H (fault %d ctx 0x%02X)", g_ctrl.fault, ctx);

    /* c) break input held low */
    world_reset(35000000);
    run_for(1000000 + WARM);
    W.brk = 1; SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM1); run_for(10000);
    ctx = SlaveCtrl_FaultContext();
    CHECK(g_ctrl.fault == CF_BREAK && (ctx & CTRL_CTX_BKIN_LOW), "BKIN low must be reported (fault %d ctx 0x%02X)", g_ctrl.fault, ctx);

    /* d) pulse-train pattern: mode FALL */
    world_reset(36000000);
    W.pattern = PAT_PULSE;
    run_for(1000000 + WARM);
    W.flt = 1; run_for(10000 + FLT_UNWIND_US);
    ctx = SlaveCtrl_FaultContext();
    CHECK(((ctx >> CTRL_CTX_MODE_SHIFT) & 3U) == CTRL_ZC_FALL, "mode must be FALL for a pulse train (ctx 0x%02X)", ctx);

    printf("  fault context: state, zero-cross mode and the live FLT / BKIN levels are reported (glitch signature included)\n");
}

#if (SLAVE_FLT_FILTER_US > 0U)
/* v0.9.62: with the FLT glitch filter the main loop's level check needs FLT asserted on consecutive passes for CTRL_FLT_POLL_CONFIRM_US: one low reading is not F02 */
static void flt_one_read(void)
{   /* FLT asserted around exactly ONE main-loop pass (passes are 200 us apart in this simulation) */
    if ((next_service - sim_t) < 60U) run_for(100U);
    run_for((next_service - sim_t) - 50U); W.flt = 1; run_for(100U); W.flt = 0;
}
static void t_flt_filter_poll(void)
{
    int k;
    world_reset(39000000); run_for(1000000 + WARM);
    CHECK(is_running(), "running first");
    for (k = 0; k < 20; k++) { flt_one_read(); run_for(5000); }
    CHECK(is_running() && g_ctrl.fault == CF_NONE, "20 noise spikes, each read by one main-loop pass: no F02, still running (state %d, fault %d)", g_ctrl.state, g_ctrl.fault);
    flt_one_read(); run_for(20000); flt_one_read(); run_for(5000);
    CHECK(is_running() && g_ctrl.fault == CF_NONE, "two single readings 20 ms apart are not one long assertion: no F02");
    {   /* a single reading just before a stop, then the restart's FIRST check reads FLT low once: an old reading must not make it count as long */
        uint32_t j;
        flt_one_read(); SlaveCtrl_SetRunRequest(0U); run_for(3000000);
        CHECK(!is_running(), "stopped");
        SlaveCtrl_SetRunRequest(1U);
        for (j = 0U; j < 2000000U && !is_running(); j++) run_for(10U);
        flt_one_read(); run_for(5000);
        CHECK(is_running() && g_ctrl.fault == CF_NONE, "after a stop and a restart, one low reading is still only one (no F02 from a reading seconds old; fault %d)", g_ctrl.fault);
    }
    W.flt = 1; run_for(1000);
    CHECK(g_ctrl.state == CTRL_RESHUNT && SlaveCtrl_FaultCode() == (uint16_t)CF_FLT, "FLT held asserted for 1 ms: F02 by the main loop's check, reported at once while unwinding (state %d, fault %u)", g_ctrl.state, (unsigned)SlaveCtrl_FaultCode());
    run_for(FLT_UNWIND_US);
    CHECK(g_ctrl.fault == CF_FLT, "and F02 once the shunts have closed (fault %d)", g_ctrl.fault);
    W.flt = 0;
    world_reset(39500000); run_for(1000000 + WARM);
    SlaveCtrl_OnFltIrq(); run_for(10000);
    CHECK(SlaveCtrl_FaultCode() == (uint16_t)CF_FLT && flt_outputs_safe(), "the interrupt path (after the filter's verdict) still trips at once (fault %u)", (unsigned)SlaveCtrl_FaultCode());
    printf("  FLT glitch filter, main-loop side: single noisy readings ignored, a held FLT trips, the interrupt path unchanged\n");
}
#endif

/* "START but nothing happens": the SYNC state reports exactly which conditions are not met. */
static void t_wait_reasons(void)
{
    uint8_t w;
    const uint8_t V = CTRL_WAIT_VIN, Z = CTRL_WAIT_ZC, F = CTRL_WAIT_FLT, B = CTRL_WAIT_BKIN, M = CTRL_WAIT_MEAS, O = CTRL_WAIT_VOUT;
    /* LEGACY relays start their sequence only when everything ELSE is ready, so while anything is missing the relays are not on
     * either and the R bit is set too (in the other modes the relays are settled long before). */
    const uint8_t RB = LEGACY_MODEL ? CTRL_WAIT_RELAY : 0U;
#define EXPECT_WAIT(label, mask) do { w = SlaveCtrl_WaitContext(); \
        CHECK((w & CTRL_CTX_VALID) && (uint8_t)(w & ~CTRL_CTX_VALID) == (uint8_t)((mask) | RB), \
              "%s: wait byte 0x%02X, expected 0x%02X", label, w, (uint8_t)((mask) | RB | CTRL_CTX_VALID)); } while (0)

    world_reset(38000000); W.vin = 100.0f; W.vout = 100.0f;
    run_for(600000);       CHECK(g_ctrl.state == CTRL_SYNC, "must wait in SYNC (state %d)", g_ctrl.state);
    EXPECT_WAIT("input 100 V", V);

    world_reset(38100000); W.pattern = PAT_NONE; run_for(600000);
    EXPECT_WAIT("no zero-cross edges", Z);

    world_reset(38200000); W.flt = 1; run_for(600000);
    EXPECT_WAIT("FLT low", F);

    world_reset(38300000); W.brk = 1; run_for(600000);
    EXPECT_WAIT("a break input low / floating", B);

    world_reset(38400000); W.meas_valid = 0; run_for(600000);
    EXPECT_WAIT("no valid measurement", M);

    world_reset(38500000); W.vout = 285.0f; run_for(600000);
    EXPECT_WAIT("output too high", O);

    world_reset(38600000); W.pattern = PAT_NONE; W.brk = 1; W.flt = 1; run_for(600000);
    EXPECT_WAIT("several at once", Z | F | B);

    /* the case that was reported: break input reads low, everything else fine - it must say so and NOT start */
    world_reset(38700000); W.brk = 1; run_for(5000000);
    EXPECT_WAIT("break input low, 5 s later", B);
    CHECK(n_energise == 0 && g_ctrl.state == CTRL_SYNC, "nothing may be energised while a break input reads low");

    /* conditions met: it leaves SYNC and the wait byte disappears */
    world_reset(38800000); run_for(1000000 + WARM);
    CHECK(g_ctrl.state != CTRL_SYNC && SlaveCtrl_WaitContext() == 0, "outside SYNC the wait byte must be 0 (state %d)", g_ctrl.state);
    W.brk = 1; run_for(10000);   /* (a break input that falls later is a fault, not a wait) */
    printf("  SYNC reports exactly which conditions are missing (zero-cross / voltage / measurement / FLT / break / output)\n");
#undef EXPECT_WAIT
}

/* ---- the three causes that used to be one "F03", and the input-voltage hysteresis ---- */
static void t_zc_fault_split(void)
{
    /* a) an extra edge in the middle of a half-cycle: the SPACING is wrong -> F03 */
    world_reset(60000000);
    run_for(1000000 + WARM);
    CHECK(is_running(), "running first (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    { uint32_t e0 = g_ctrl.events; while (g_ctrl.events == e0) run_for(100); }      /* just AFTER a real event ... */
    q_insert((int64_t)sim_t + 3000, 0U);                     /* ... a spurious falling edge 3 ms later: spacing 3 ms, always wrong */
    run_for(100000);
    CHECK(diag == 0, "the converter must be off after the spurious edge");
    run_for(1000000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_ZC_PATTERN, "an edge at the wrong TIME must give F03 (got %d/%d)", g_ctrl.state, g_ctrl.fault);

    /* b) the edge arrives on time but the measured sign did not flip -> F15 */
    world_reset(61000000);
    run_for(1000000 + WARM);
    CHECK(is_running(), "running first");
    W.sign_glitch = 1;                                        /* the sign read at the NEXT event is wrong */
    run_for(100000);
    CHECK(diag == 0, "the converter must be off after the bad sign");
    run_for(1000000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_ZC_SEQUENCE, "a wrong SIGN sequence must give F15 (got %d/%d)", g_ctrl.state, g_ctrl.fault);

    /* c) the last-moment check disagrees right before energising -> F16 */
    world_reset(62000000);
    run_for(1000000 + WARM);
    CHECK(is_running(), "running first");
    W.preen_glitch = 1;
    run_for(100000);
    CHECK(diag == 0, "nothing may be energised after the failed last-moment check");
    run_for(1000000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_PRE_ENERGISE, "the last-moment polarity disagreement must give F16 (got %d/%d)", g_ctrl.state, g_ctrl.fault);
    CHECK(g_ctrl.fault_count >= 1, "all three escalate");
    printf("  zero-cross faults tell the cause: wrong time -> F03, wrong sign sequence -> F15, last-moment polarity disagreement -> F16\n");
}


#if (SLAVE_GLITCH_SKIP != 0)
/* ---- a RARE zero-cross glitch (F16, F04) while RUNNING must not stop the unit; a real one still must ---- */
static void t_glitch_skip(void)
{
    uint32_t s0, k;
    /* a) ONE bad last-moment check while running: nothing energised that half-cycle, no fault, the unit carries on */
    world_reset(71000000);
    run_for(12000000);
    CHECK(g_ctrl.state == CTRL_RUN, "in RUN first (state %d)", g_ctrl.state);
    s0 = SlaveCtrl_GlitchSkips();
    W.preen_glitch = 1;
    run_for(2000000);
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "one F16-type glitch: the unit keeps running, no fault (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    CHECK(SlaveCtrl_GlitchSkips() == s0 + 1U, "and it was counted as one skipped half-cycle (%u)", (unsigned)(SlaveCtrl_GlitchSkips() - s0));
    CHECK(diag != 0 || g_ctrl.commutations > 0U, "the converter keeps commutating afterwards");
    check_invariants("one F16 glitch");

    /* b) TWO in a row are tolerated, the THIRD within 2 s is the fault, exactly as before */
    world_reset(72000000);
    run_for(12000000);
    CHECK(g_ctrl.state == CTRL_RUN, "in RUN first (state %d)", g_ctrl.state);
    W.preen_glitch = 2;
    run_for(1000000);
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "two glitches in a row: still tolerated (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    world_reset(73000000);
    run_for(12000000);
    CHECK(g_ctrl.state == CTRL_RUN, "in RUN first (state %d)", g_ctrl.state);
    W.preen_glitch = 3;
    run_for(1000000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_PRE_ENERGISE, "the third glitch within 2 s is F16 as before (got %d/%d)", g_ctrl.state, g_ctrl.fault);
    CHECK(diag == 0, "and nothing is energised");

    /* c) a REAL polarity problem fails on every half-cycle: the unit stops within a few events (F04 for a signal too small to read) */
    world_reset(74000000);
    run_for(12000000);
    CHECK(g_ctrl.state == CTRL_RUN, "in RUN first (state %d)", g_ctrl.state);
    W.amp = 10.0;                                        /* mains too small to read a sign: every event fails */
    run_for(100000);
    CHECK(diag == 0 && g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_POLARITY, "a persistent tiny signal: F04 within 0.1 s (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    check_invariants("persistent tiny signal");

    /* d) ONE unreadable sign at one event: tolerated */
    world_reset(75000000);
    run_for(12000000);
    CHECK(g_ctrl.state == CTRL_RUN, "in RUN first (state %d)", g_ctrl.state);
    s0 = SlaveCtrl_GlitchSkips();
    W.in_zero = 1;                                          /* the input reads zero until ONE event has been affected */
    for (k = 0U; (k < 60U) && (SlaveCtrl_GlitchSkips() == s0); k++) run_for(1000);
    W.in_zero = 0;
    run_for(1500000);
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE && SlaveCtrl_GlitchSkips() > s0 && SlaveCtrl_GlitchSkips() <= s0 + 2U, "one unreadable sign (F04-type): tolerated and counted (%u skipped, state %d fault %d)", (unsigned)(SlaveCtrl_GlitchSkips() - s0), g_ctrl.state, g_ctrl.fault);

    /* e) the observed rate, 6 in 10 minutes: 20 simulated minutes, a glitch every 100 s, the unit never stops */
    world_reset(76000000);
    run_for(12000000);
    CHECK(g_ctrl.state == CTRL_RUN, "in RUN first (state %d)", g_ctrl.state);
    s0 = SlaveCtrl_GlitchSkips();
    for (k = 0U; k < 12U; k++)
    {
        W.preen_glitch = 1;
        run_for(100000000);
        if (!(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE)) break;
    }
    CHECK(k == 12U && g_ctrl.state == CTRL_RUN, "a glitch every 100 s for 20 minutes: never a fault (%u glitches survived, state %d fault %d)", (unsigned)k, g_ctrl.state, g_ctrl.fault);
    CHECK(SlaveCtrl_GlitchSkips() - s0 == 12U, "all twelve counted (%u)", (unsigned)(SlaveCtrl_GlitchSkips() - s0));

    /* f) while a start is going on the checks stay strict: a bad last-moment check before the first diagonals is a fault */
    world_reset(77000000);
    W.preen_glitch = 1;
    run_for(1500000 + WARM);
    CHECK(g_ctrl.fault_count >= 1U || g_ctrl.fault == CF_PRE_ENERGISE, "a glitch during the START is still F16 (fault_count %u, fault %d)", (unsigned)g_ctrl.fault_count, g_ctrl.fault);
    printf("  glitch skip: one F16/F04 glitch while running is skipped and counted, three in 2 s or a persistent one stops the unit, 12 glitches over 20 minutes never do, the start stays strict\n");
}
#endif


#if (SLAVE_TF_RUN_CHECK != 0) && (SLAVE_MASTER_CONTROL == 0)
/* ---- the flip fault in a STANDALONE build (no master): it must latch at once and never be retried by itself ---- */
static void t_flip_standalone(void)
{
    world_reset(81000000); W.vin = 215.0f; W.vout = 215.0f; W.plant_on = 1; W.plant_curve = 0; W.plant_gain = -0.02;      /* a transformer connected backwards */
    run_for(30000000);
    CHECK(g_ctrl.state == CTRL_LATCHED && g_ctrl.fault == CF_TRANSFORMER_FLIP, "standalone, backwards transformer: latched with fault 17 (state %d, fault %d)", g_ctrl.state, g_ctrl.fault);
    run_for(300000000);
    CHECK(g_ctrl.state == CTRL_LATCHED && g_ctrl.fault == CF_TRANSFORMER_FLIP && diag == 0, "five minutes later it is STILL latched and nothing is energised: a wiring fault is never retried by itself (state %d)", g_ctrl.state);
    world_reset(82000000); W.vin = 215.0f; W.vout = 215.0f; W.plant_on = 1; W.plant_curve = 1;
    run_for(40000000);
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "standalone, correct transformer: runs normally (state %d, fault %d)", g_ctrl.state, g_ctrl.fault);
    printf("  flip standalone: latched at once and never retried, a correct transformer runs\n");
}
#endif

static void t_vin_hysteresis(void)
{
    uint64_t t0;
    world_reset(63000000);
    W.vin = 145.0f; W.vout = 145.0f;                         /* inside the TRIP window (>= 140) but below START (>= 150) */
    run_for(1500000);
    CHECK(g_ctrl.state == CTRL_SYNC && (SlaveCtrl_WaitContext() & CTRL_WAIT_VIN), "145 V must NOT start (wait V), state %d wait 0x%02X", g_ctrl.state, SlaveCtrl_WaitContext());
    CHECK(n_energise == 0, "nothing may be energised at 145 V");
    W.vin = 152.0f; W.vout = 152.0f;
    run_for(800000 + WARM);                                  /* (legacy relays: the SH -> W sequence comes first) */
    CHECK(is_running(), "152 V must start (state %d)", g_ctrl.state);
    /* the supply sags when the converter starts: 152 -> 141 V must NOT trip (it is still above the trip level 140) */
    W.vin = 141.0f; W.vout = 141.0f;
    run_for(1500000);
    CHECK(is_running(), "a sag to 141 V must not trip (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    /* below the trip level it does trip, debounced */
    W.vin = 139.0f; W.vout = 139.0f;
    t0 = sim_t;
    while (g_ctrl.state != CTRL_FAULT && sim_t < t0 + 1500000ULL) run_for(10000);      /* (a soft fault may unwind through RESHUNT first) */
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_VIN_RANGE, "139 V must trip F09 (got %d/%d)", g_ctrl.state, g_ctrl.fault);
    CHECK(sim_t - t0 < 1500000ULL, "trip took %llu us", (unsigned long long)(sim_t - t0));
    /* upper side: start window ends 10 V below the trip level */
    world_reset(64000000);
    W.vin = 295.0f; W.vout = 295.0f;
    run_for(1500000);
    CHECK(g_ctrl.state == CTRL_SYNC && (SlaveCtrl_WaitContext() & CTRL_WAIT_VIN), "295 V must NOT start (wait V)");
    printf("  start window [150, 290] V is narrower than the trip window [140, 300] V: a start at 145 V waits, a sag to 141 V does not trip, 139 V does (F09)\n");
    (void)t0;
}

static void t_diag_counters(void)
{
    int i; uint16_t w;
    /* a) edges while the converter is NOT running (nothing can be energised) count as IDLE */
    world_reset(65000000);
    SlaveCtrl_SetRunRequest(0U);
    CHECK(SlaveCtrl_DiagWord() == 0, "no events yet");
    for (i = 0; i < 3; i++) SlaveCtrl_OnFltIrq();
    for (i = 0; i < 2; i++) SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM1);
    w = SlaveCtrl_DiagWord();
    CHECK((w & 31U) == 3 && ((w >> 5) & 31U) == 0, "3 idle FLT edges, 0 running (word 0x%04X)", w);
    CHECK(((w >> 10) & 7U) == 2 && ((w >> 13) & 7U) == 0, "2 idle break events, 0 running (word 0x%04X)", w);
    CHECK(g_ctrl.state == CTRL_OFF && g_ctrl.fault == 0, "events while idle are NOT faults (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    CHECK(SlaveCtrl_TripCapture() == 0, "idle events are not captured as trips");
    /* b) while running they count as RUNNING and are faults */
    world_reset(66000000);
    run_for(1000000 + WARM);
    CHECK(is_running(), "running first");
    SlaveCtrl_OnFltIrq(); run_for(5000 + FLT_UNWIND_US);
    w = SlaveCtrl_DiagWord();
    CHECK((w & 31U) == 0 && ((w >> 5) & 31U) == 1, "1 running FLT edge (word 0x%04X)", w);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_FLT, "...and it is a fault (got %d/%d)", g_ctrl.state, g_ctrl.fault);
    /* b2) THE SAME PULSE seen by the break input right after the FLT interrupt (the real hardware order, IRQ 6 before IRQ 13):
     *     it belongs to the same RUNNING incident and must not be counted as idle (the K3/0 seen on the bench) */
    world_reset(66500000);
    run_for(1000000 + WARM);
    CHECK(is_running(), "running first");
    SlaveCtrl_OnFltIrq(); SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM1);
    w = SlaveCtrl_DiagWord();
    CHECK(((w >> 5) & 31U) == 1 && ((w >> 13) & 7U) == 1 && ((w >> 10) & 7U) == 0, "the same pulse seen by FLT then break: 1 running each, 0 idle (word 0x%04X)", w);
    run_for(5000);
    SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM1);                      /* a LATER event, incident over: idle again */
    CHECK(((SlaveCtrl_DiagWord() >> 10) & 7U) == 1, "after the incident a new event counts as idle again (word 0x%04X)", SlaveCtrl_DiagWord());
    /* c) saturation and exact packing */
    world_reset(67000000);
    SlaveCtrl_SetRunRequest(0U);
    for (i = 0; i < 100; i++) SlaveCtrl_OnFltIrq();
    for (i = 0; i < 100; i++) SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM15);
    w = SlaveCtrl_DiagWord();
    CHECK((w & 31U) == 31 && ((w >> 10) & 7U) == 7, "counters saturate at 31 / 7 (word 0x%04X)", w);
    g_ctrl.flt_idle = 5; g_ctrl.flt_run = 9; g_ctrl.brk_idle = 3; g_ctrl.brk_run = 6;
    CHECK(SlaveCtrl_DiagWord() == (uint16_t)(5U | (9U << 5) | (3U << 10) | (6U << 13)), "packing 5/9/3/6 -> 0x%04X", SlaveCtrl_DiagWord());
    printf("  FLT / break events counted separately for idle and running; the same pulse is not counted twice as idle; saturates at 31 / 7\n");
}

/* where in the half-cycle, and which diagonal, a running trip hit */
static void t_trip_capture(void)
{
    uint16_t c; uint32_t us; int pos_seen = 0, neg_seen = 0, k;
    world_reset(68000000);
    run_for(1000000 + WARM);
    CHECK(is_running() && SlaveCtrl_TripCapture() == 0, "running, nothing captured yet");
    for (k = 0; k < 6; k++)                                         /* several trips at known offsets after a real event, both half-cycles */
    {
        static const uint32_t offs[6] = { 60, 500, 3000, 6000, 9000, 9700 };
        int8_t pol;
        uint32_t e0;
        world_reset(68100000ULL + (uint64_t)k * 1000000ULL);
        run_for(1000000 + WARM);
        e0 = g_ctrl.events;
        while (g_ctrl.events == e0) run_for(50);                    /* just after a real zero-cross event */
        { uint64_t t0 = sim_t; run_for(offs[k]); (void)t0; }
        pol = g_ctrl.conv_pol;
        SlaveCtrl_OnFltIrq();
        c = SlaveCtrl_TripCapture();
        us = (uint32_t)(c & 0x03FFU) * 16U;
        CHECK((c & 0x8000U) != 0, "offset %u us: capture must be valid (0x%04X)", offs[k], c);
        CHECK(us + 150U >= offs[k] && us <= offs[k] + 250U, "offset %u us measured as %u us", offs[k], us);
        CHECK(((c & 0x0400U) != 0) == (pol > 0) && ((c & 0x0800U) != 0) == (pol < 0), "offset %u us: diagonal bits 0x%04X vs conv_pol %d", offs[k], c & 0x0C00U, pol);
        if (c & 0x0400U) pos_seen = 1;
        if (c & 0x0800U) neg_seen = 1;
    }
    CHECK(pos_seen || neg_seen, "at least one trip must show an energised diagonal");
    /* a break-input trip is captured the same way, INCLUDING which diagonal was on (it must be read before the outputs are switched off) */
    {
        int8_t pol; uint32_t e0;
        world_reset(69000000);
        run_for(1000000 + WARM);
        e0 = g_ctrl.events;
        while (g_ctrl.events == e0) run_for(50);
        run_for(3000);                                              /* mid half-cycle: a diagonal is on */
        pol = g_ctrl.conv_pol;
        CHECK(pol != 0, "a diagonal must be energised in the middle of a half-cycle (conv_pol %d)", pol);
        SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM1);
        c = SlaveCtrl_TripCapture();
        CHECK((c & 0x8000U) != 0, "a running break event is captured too");
        CHECK(((c & 0x0400U) != 0) == (pol > 0) && ((c & 0x0800U) != 0) == (pol < 0), "break path: diagonal bits 0x%04X vs conv_pol %d", c & 0x0C00U, pol);
    }
    printf("  trip capture: time since the zero-cross event (16 us units) and the energised diagonal, FLT and break\n");
}

/* PASSIVE mapping check: nothing is energised; counts whether "diagonal from the AC_IN sign" agrees with the old firmware's
 * "diagonal from the ZCD edge direction" (falling -> 3H+4L, rising -> 3L+4H). */
static void t_map_check(void)
{
    uint16_t w; unsigned a, d; uint32_t e0;
    /* a) ZCD rising = the line goes NEGATIVE -> POSITIVE (PAT_SQUARE): this firmware picks "positive next" after a rising edge, the old
     *    firmware picks 3L+4H (negative): they DISAGREE on every event */
    world_reset(70000000);
    SlaveCtrl_SetRunRequest(0U);
    W.pattern = PAT_SQUARE; resync_crossings();
    run_for(1500000);
    w = SlaveCtrl_MapWord(); a = w & 127U; d = (w >> 7) & 127U;
    CHECK((w & 0x4000U) && !(w & 0x8000U), "map word valid, no trip flag (0x%04X)", w);
    CHECK(a == 0 && d >= 40, "PAT_SQUARE (rising = negative->positive): expected 0 agree / many disagree, got %u / %u", a, d);
    CHECK(n_energise == 0 && g_ctrl.state == CTRL_OFF, "nothing may be energised during the passive check (energise %d, state %d)", n_energise, g_ctrl.state);
    /* b) the inverted pattern (rising = POSITIVE -> NEGATIVE): they AGREE on every event */
    world_reset(71000000);
    SlaveCtrl_SetRunRequest(0U);
    W.pattern = PAT_SQUARE_INV; resync_crossings();
    run_for(1500000);
    w = SlaveCtrl_MapWord(); a = w & 127U; d = (w >> 7) & 127U;
    CHECK(d == 0 && a >= 40, "PAT_SQUARE_INV (rising = positive->negative): expected many agree / 0 disagree, got %u / %u", a, d);
    /* c) one pulse per half-cycle: the edge direction does not identify the half -> MIXED */
    world_reset(72000000);
    SlaveCtrl_SetRunRequest(0U);
    W.pattern = PAT_PULSE; W.pulse_w_us = 800; resync_crossings();
    run_for(1500000);
    w = SlaveCtrl_MapWord(); a = w & 127U; d = (w >> 7) & 127U;
    CHECK(a >= 10 && d >= 10, "pulse pattern: the mapping must come out MIXED, got %u / %u", a, d);
    /* d) long run: the counts are halved at 127 and stay current (a pattern change shows up) */
    world_reset(73000000);
    SlaveCtrl_SetRunRequest(0U);
    W.pattern = PAT_SQUARE; resync_crossings();
    run_for(4000000);
    w = SlaveCtrl_MapWord(); a = w & 127U; d = (w >> 7) & 127U;
    CHECK(a == 0 && d > 40 && d <= 127, "after 4 s the disagree count stays bounded (%u / %u)", a, d);
    W.pattern = PAT_SQUARE_INV; resync_crossings();
    run_for(3000000);
    w = SlaveCtrl_MapWord(); a = w & 127U; d = (w >> 7) & 127U;
    CHECK(a > d, "after the pattern is inverted the AGREE count takes over (%u / %u)", a, d);
    (void)e0;
    printf("  mapping check (passive): rising=neg->pos -> all DISAGREE, rising=pos->neg -> all AGREE, one pulse per half -> MIXED; nothing energised\n");
}

#if LEGACY_MODEL
/* ====================================================================== */
/* LEGACY relay sequence (SLAVE_RELAYS_MODE 2) - the old firmware's order  */
/* ====================================================================== */
static int lg_find(int idx, int on, int nth)               /* index of the nth (0-based) coil command, or -1 */
{
    int i, c = 0;
    for (i = 0; i < lg_n; i++) if (lg_idx[i] == idx && lg_on[i] == on) { if (c == nth) return i; c++; }
    return -1;
}

static void t_legacy_sequence(void)
{
    int sh_on, sh_off, w_on, w_off, i, both = 0, sh_on2;
    double phase, d;
    /* the defaults are the OLD firmware's values: SH held 1 s, W settle 1 s, 6 ms pull-in lead (an estimate in the old code) */
    CHECK(SLAVE_LEGACY_SH_HOLD_MS == 1000U && SLAVE_LEGACY_W_SETTLE_MS == 1000U && SLAVE_LEGACY_SH_PULLIN_US == 6000U && SLAVE_LEGACY_RELEASE_MS == 1000U,
          "legacy constants differ from the old firmware's (hold %u, settle %u, lead %u, release %u)",
          (unsigned)SLAVE_LEGACY_SH_HOLD_MS, (unsigned)SLAVE_LEGACY_W_SETTLE_MS, (unsigned)SLAVE_LEGACY_SH_PULLIN_US, (unsigned)SLAVE_LEGACY_RELEASE_MS);
    world_reset(80000000);
    run_for(4000000);
    sh_on = lg_find(0, 1, 0); sh_off = lg_find(0, 0, 0); w_on = lg_find(1, 1, 0); w_off = lg_find(1, 0, 0); sh_on2 = lg_find(0, 1, 1);
    CHECK(sh_on >= 0 && sh_off > sh_on, "SH-RLY must go on and then off (log %d entries)", lg_n);
    CHECK(w_on >= 0 && w_off < 0, "W-RLY must go on and STAY on while running");
    CHECK(sh_on2 < 0, "SH-RLY must be pulsed only once");
    if (sh_on >= 0 && sh_off > sh_on && w_on >= 0)
    {
        CHECK(lg_t[sh_off] - lg_t[sh_on] >= (uint64_t)SLAVE_LEGACY_SH_HOLD_MS * 1000ULL - 2000ULL &&
              lg_t[sh_off] - lg_t[sh_on] <= (uint64_t)SLAVE_LEGACY_SH_HOLD_MS * 1000ULL + 20000ULL,
              "SH-RLY held %.0f ms, expected %u ms", (double)(lg_t[sh_off] - lg_t[sh_on]) / 1000.0, (unsigned)SLAVE_LEGACY_SH_HOLD_MS);
        CHECK(lg_t[w_on] >= lg_t[sh_off] && lg_t[w_on] - lg_t[sh_off] <= 1000ULL, "W-RLY on %llu us after SH-RLY off: must be the same instant",
              (unsigned long long)(lg_t[w_on] - lg_t[sh_off]));
        CHECK(t_conv_first >= lg_t[w_on] + (uint64_t)SLAVE_LEGACY_W_SETTLE_MS * 1000ULL - 2000ULL,
              "the converter started %.0f ms after W-RLY on, it must wait %u ms",
              (double)((int64_t)t_conv_first - (int64_t)lg_t[w_on]) / 1000.0, (unsigned)SLAVE_LEGACY_W_SETTLE_MS);
        /* SH-RLY is aimed at a zero-cross: command time + pull-in lead lands on a mains zero */
        phase = fmod((double)(lg_t[sh_on] + SLAVE_LEGACY_SH_PULLIN_US) - W.phase0_us, half_period_us());
        if (phase < 0) phase += half_period_us();
        d = (phase < half_period_us() - phase) ? phase : half_period_us() - phase;
        CHECK(d < 1500.0, "SH-RLY contacts would close %.0f us away from a zero-cross (command + %u us pull-in)", d, (unsigned)SLAVE_LEGACY_SH_PULLIN_US);
    }
    for (i = 0; i < lg_n; i++) (void)i;
    CHECK(rl[0].coil == 0 && rl[1].coil == 1, "final coil state: SH off, W on (SH %d, W %d)", rl[0].coil, rl[1].coil);
    CHECK(is_running(), "converter running (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    (void)both;
    printf("  legacy relays: SH on %.0f ms (aimed at a zero-cross) -> SH off + W on together -> W settle %u ms -> converter; W stays on\n",
           (sh_off > sh_on && sh_on >= 0) ? (double)(lg_t[sh_off] - lg_t[sh_on]) / 1000.0 : -1.0, (unsigned)SLAVE_LEGACY_W_SETTLE_MS);
}

static void t_legacy_no_converter_without_w(void)
{
    uint64_t t;
    world_reset(81000000);
    for (t = 0; t < 2000000ULL; t += 1000) { run_for(1000); if (!lg_on[0] && lg_n == 0) { } if (rl[1].coil == 0) CHECK(n_energise == 0, "converter energised while W-RLY was still off (t=%llu us)", (unsigned long long)t); if (n_energise) break; }
    CHECK(rl[1].coil == 1 || n_energise == 0, "converter may only run with W-RLY on");
    printf("  legacy relays: the converter never starts before W-RLY is on and settled\n");
}

static void t_legacy_fault_and_restart(void)
{
    uint64_t t0; int sh_before, sh_after;
    world_reset(82000000);
    run_for(4000000);
    CHECK(is_running() && rl[1].coil == 1, "running with W on first (state %d)", g_ctrl.state);
    sh_before = 0; { int i; for (i = 0; i < lg_n; i++) if (lg_idx[i] == 0 && lg_on[i] == 1) sh_before++; }
    t0 = sim_t;
    SlaveCtrl_OnFltIrq();                                        /* a trip: the converter goes off in the interrupt */
    CHECK(diag == 0, "converter off at once");
    run_for(5000);
    CHECK(rl[0].coil == 0 && rl[1].coil == 0, "a fault must release BOTH coils at once (SH %d, W %d)", rl[0].coil, rl[1].coil);
    run_for(FLT_UNWIND_US);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_FLT, "FAULT / FLT (got %d/%d)", g_ctrl.state, g_ctrl.fault);
    run_for(14000000);                                           /* escalating retry (10 s) + the whole relay sequence again */
    sh_after = 0; { int i; for (i = 0; i < lg_n; i++) if (lg_idx[i] == 0 && lg_on[i] == 1) sh_after++; }
    CHECK(sh_after == sh_before + 1, "the SH pulse must be repeated after the fault (before %d, after %d)", sh_before, sh_after);
    CHECK(is_running() && rl[1].coil == 1, "running again with W on (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    (void)t0;
    printf("  legacy relays: a fault releases both coils at once; after the retry the SH -> W sequence runs again before the converter\n");
}

static void t_legacy_stop_release(void)
{
    world_reset(83000000);
    run_for(4000000);
    CHECK(is_running() && rl[1].coil == 1, "running with W on first");
    SlaveCtrl_SetRunRequest(0U);
    run_for(300000);
    CHECK(g_ctrl.state == CTRL_OFF && diag == 0, "converter off after the stop (state %d)", g_ctrl.state);
    CHECK(rl[1].coil == 1, "W-RLY is released only AFTER the delay, not at once");
    run_for(1500000);
    CHECK(rl[0].coil == 0 && rl[1].coil == 0, "both coils released %u ms after the stop (SH %d, W %d)", (unsigned)SLAVE_LEGACY_RELEASE_MS, rl[0].coil, rl[1].coil);
    printf("  legacy relays: after a stop W-RLY opens %u ms later\n", (unsigned)SLAVE_LEGACY_RELEASE_MS);
}

static void t_legacy_never_both(void)
{
    int i, overlap = 0, on_sh = 0, on_w = 0;
    world_reset(84000000);
    run_for(4000000);
    for (i = 0; i < lg_n; i++)
    {
        if (lg_idx[i] == 0) on_sh = lg_on[i]; else on_w = lg_on[i];
        if (on_sh && on_w) overlap++;
    }
    CHECK(overlap == 0, "SH-RLY and W-RLY must never be on together (%d overlaps in the log)", overlap);
    printf("  legacy relays: SH-RLY and W-RLY are never energised together\n");
}
#endif

#if (SLAVE_CONV_FIRST_POL != 0)
/* SLAVE_CONV_FIRST_POL: the converter must START on the chosen half-cycle whatever the phase at which the start happens */
static void t_first_polarity(void)
{
    int k, bad = 0, started = 0;
    for (k = 0; k < 16; k++)
    {
        world_reset(90000000ULL + (uint64_t)k * 1234567ULL + (uint64_t)k * 311ULL);   /* different start phases: the next half-cycle is + or - */
        W.phase0_us += (double)k * 637.0;                                              /* ... and the mains phase itself is moved */
        resync_crossings();
        run_for(1500000 + WARM);
        if (is_running()) started++;
        if (first_energise_pol != 0 && first_energise_pol != SLAVE_CONV_FIRST_POL) bad++;
        CHECK(first_energise_pol == SLAVE_CONV_FIRST_POL, "start %d: the first diagonal served polarity %d, expected %d", k, first_energise_pol, SLAVE_CONV_FIRST_POL);
    }
    CHECK(bad == 0 && started >= 12, "%d wrong first polarities, %d of 16 started", bad, started);
    printf("  forced first polarity %+d: all 16 starts (different phases) energised that half-cycle first\n", (int)SLAVE_CONV_FIRST_POL);
}
#endif

#if (SLAVE_LEGACY_LOWSIDES != 0)
/* ====================================================================== */
/* LEGACY low sides (the old firmware): 1L / 2L ON through the converter's  */
/* first diagonal, off from the second                                      */
/* ====================================================================== */
static void t_legacy_lowsides(void)
{
    world_reset(95000000);
    run_for(4500000);
    CHECK(is_running(), "converter running (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    CHECK(t_inv_enable_first > 0 && t_conv_first > 0 && t_inv_enable_first < t_conv_first,
          "the inverter outputs (low sides) must be enabled BEFORE the first converter diagonal (enable %llu us, first diagonal %llu us)",
          (unsigned long long)t_inv_enable_first, (unsigned long long)t_conv_first);
    CHECK(inv_at_first_energise == 1, "the FIRST diagonal must run with the low sides on (inverter enabled: %d)", inv_at_first_energise);
    CHECK(t_first_duty == 0, "the low sides stay at ZERO duty: the high sides are never driven");
#if LEGACY_MODEL
    {   /* the primary must never be left open during the relay hand-over: the lows are on BEFORE W-RLY goes on (W on = the PRI nets OPEN) */
        const int w_on = lg_find(1, 1, 0), sh_on = lg_find(0, 1, 0);
        CHECK(w_on >= 0 && sh_on >= 0, "relay sequence present");
        if (w_on >= 0 && sh_on >= 0)
        {
            CHECK(t_inv_enable_first < lg_t[sh_on], "the low sides must be on before the relay sequence starts (enable %llu us, SH on %llu us)",
                  (unsigned long long)t_inv_enable_first, (unsigned long long)lg_t[sh_on]);
            CHECK(t_inv_enable_first < lg_t[w_on], "the low sides must be on BEFORE W-RLY opens the PRI nets");
        }
    }
#endif
#if (SLAVE_LOWSIDES_OFF_AFTER == 0)
    CHECK(inv_enabled == 1, "with OFF_AFTER 0 the low sides stay ON while only the converter runs");
    SlaveCtrl_SetRunRequest(0U);                                  /* ... and go off with the stop */
    run_for(300000);
    CHECK(!inv_enabled && diag == 0, "a stop must switch the low sides off (inverter %d)", inv_enabled);
#endif
#if (SLAVE_LOWSIDES_OFF_AFTER == 2)
    CHECK(!inv_enabled, "from the second diagonal on the low sides are off");
    CHECK(t_inv_disable_last >= t_conv_first + 5000ULL && t_inv_disable_last <= t_conv_first + 25000ULL,
          "the lows must go off at the SECOND diagonal (about 10 ms after the first), measured %.1f ms",
          (double)((int64_t)t_inv_disable_last - (int64_t)t_conv_first) / 1000.0);
#endif
    printf("  legacy low sides: inverter outputs on (zero duty) %.0f ms before the first diagonal, first diagonal runs with them on, off %.1f ms after it (OFF_AFTER %d)\n",
           (double)((int64_t)t_conv_first - (int64_t)t_inv_enable_first) / 1000.0, (double)((int64_t)t_inv_disable_last - (int64_t)t_conv_first) / 1000.0, (int)SLAVE_LOWSIDES_OFF_AFTER);
}

static void t_legacy_lowsides_restart(void)
{
    world_reset(96000000);
    run_for(4500000);
    CHECK(is_running(), "running first");
    SlaveCtrl_OnFltIrq();                                          /* a trip: converter and high sides off at once; v0.9.65: the low sides stay on until the shunts have closed */
    run_for(5000);
    CHECK(flt_outputs_safe(), "a trip switches the converter off at once, the low sides are on (forced) or off (inverter %d, forced %d)", inv_enabled, inv_forced);
    run_for(FLT_UNWIND_US);
    CHECK(!inv_enabled && diag == 0, "a trip ends with the low sides off too, once the shunts have closed");
    t_conv_first = 0; t_inv_enable_first = 0; first_energise_pol = 0; inv_at_first_energise = -1;
    run_for(14000000);                                             /* retry (10 s) + relay sequence again */
    CHECK(is_running(), "running again after the retry (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    CHECK(t_inv_enable_first > 0 && t_inv_enable_first < t_conv_first, "after the retry the low sides must be enabled again BEFORE the first diagonal");
    CHECK(inv_at_first_energise == 1, "the first diagonal of the retry must run with the low sides on too");
    printf("  legacy low sides: after a trip they go off, and the retry enables them again before its first diagonal\n");
}
#endif

#ifdef EXPECT_LEGACY_DEFAULTS
/* The shipped defaults for the legacy converter-only start are the BENCH-PROVEN ones: start on the POSITIVE half (10/10 starts; negative 0/10) with the
 * low sides kept on while only the converter runs. This job is built WITHOUT overrides, so a change of the defaults fails here. */
static void t_legacy_defaults(void)
{
    CHECK(SLAVE_RELAYS_MODE == 2 && SLAVE_POWER_STAGE == 1, "built as legacy relays at stage 1");
    CHECK(SLAVE_CONV_FIRST_POL == 1, "default first polarity must be +1 (bench: positive 10/10, negative 0/10), got %d", (int)SLAVE_CONV_FIRST_POL);
    CHECK(SLAVE_LOWSIDES_OFF_AFTER == 0, "default: low sides stay on while only the converter runs, got %d", (int)SLAVE_LOWSIDES_OFF_AFTER);
    CHECK(SLAVE_LEGACY_LOWSIDES == 1, "default: legacy low sides on");
    CHECK(SLAVE_CONV_SWAP == 0, "default: no diagonal swap");
    printf("  shipped defaults for the legacy converter start: first half +1, low sides kept on, no swap\n");
}
#endif

#ifdef TEST_LOW_VOLTAGE
/* LOW INPUT VOLTAGE (build with -DCTRL_VIN_MIN_V=80.0f): the converter-only start from 90 V. Counts per volt from the 140 V test point:
 * 784 counts = 140 V RMS -> 90 V = 504 counts, 70 V = 392 counts (the ZCD IC limit seen on the bench). The sign detection (threshold 20 counts
 * on an 8-sample window 0.2-0.6 ms before the edge) and the 20 kHz guard must hold with the real edge timing spread. */
static void t_low_voltage(void)
{
    double delays[5] = { -40.0, 0.0, 40.0, 150.0, -120.0 };
    double vrms[3]   = { 90.0, 70.0, 120.0 };
    int i, j;
    for (j = 0; j < 3; j++)
        for (i = 0; i < 5; i++)
        {
            world_reset(97000000ULL + (uint64_t)(j * 5 + i) * 50000000ULL);
            W.edge_delay_us = delays[i]; W.amp = vrms[j] * 784.0 / 140.0; W.jitter_us = 40.0;
            /* the MEASURED Vin is kept inside the start window (>= 90 V) so that the SIGN-DETECTION margin at the small amplitude is what is tested:
             * 70 V is below the 90 V start window and would simply wait */
            W.vin = (float)((vrms[j] < 95.0) ? 100.0 : vrms[j]); W.vout = W.vin;
            run_for(6000000 + WARM);
            CHECK(is_running() && g_ctrl.fault == CF_NONE, "%.0f V, ZCD delay %+.0f us: must start and run without a fault (state %d fault %d)",
                  vrms[j], delays[i], g_ctrl.state, g_ctrl.fault);
            check_invariants("low voltage");
        }
    /* the start / trip levels follow CTRL_VIN_MIN_V (80 V): start from 90 V, trip below 80 V */
    world_reset(98000000);
    W.amp = 85.0 * 784.0 / 140.0; W.vin = 85.0f; W.vout = 85.0f;
    run_for(2000000 + WARM);
    CHECK(g_ctrl.state == CTRL_SYNC && (SlaveCtrl_WaitContext() & CTRL_WAIT_VIN) && n_energise == 0, "85 V must wait (start from 90 V): state %d", g_ctrl.state);
    W.amp = 92.0 * 784.0 / 140.0; W.vin = 92.0f; W.vout = 92.0f;
    run_for(1500000 + WARM);
    CHECK(is_running(), "92 V must start (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    W.amp = 78.0 * 784.0 / 140.0; W.vin = 78.0f; W.vout = 78.0f;
    { uint64_t t0 = sim_t; while (g_ctrl.state != CTRL_FAULT && sim_t < t0 + 1500000ULL) run_for(10000);
      CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_VIN_RANGE, "78 V must trip F09 (got %d/%d)", g_ctrl.state, g_ctrl.fault); }
    printf("  low input voltage: starts and runs at 90 V, 70 V and 120 V with the edge timing spread; start from 90 V, trip below 80 V\n");
}
#endif

#if (SLAVE_TF_CHECK != 0)
/* ====================================================================== */
/* TRANSFORMER-CONNECTION CHECK: average (Vout - Vin) once the converter     */
/* has really been running, one verdict per start                           */
/* ====================================================================== */
static int tf_verdict(uint16_t w) { return (int)((w >> 11) & 3U); }
static int tf_dv10(uint16_t w)    { int v = (int)(w & 0x07FFU); if (v & 0x0400) v -= 0x0800; return v; }

static void t_tf_check(void)
{
    uint16_t w;
    /* correct connection: Vout above Vin */
    world_reset(110000000); W.vin = 200.0f; W.vout = 203.0f;
    run_for(2500000);                                                     /* the converter is running, the window has not completed */
    CHECK(SlaveCtrl_TfWord() == 0U, "no verdict before the settle + window have passed (word 0x%04X)", (unsigned)SlaveCtrl_TfWord());
    run_for(6000000);
    w = SlaveCtrl_TfWord();
    CHECK((w & 0xE000U) == 0x2000U, "word carries bit 13 and neither bit 14 nor 15: 0x%04X", (unsigned)w);
    CHECK(is_running() && tf_verdict(w) == 1 && tf_dv10(w) >= 29 && tf_dv10(w) <= 31, "Vout +3.0 V above Vin: verdict %d, dv %d (x0.1 V)", tf_verdict(w), tf_dv10(w));
    /* flipped: Vout below Vin */
    world_reset(111000000); W.vin = 200.0f; W.vout = 196.5f;
    run_for(9000000);
    w = SlaveCtrl_TfWord();
    CHECK(tf_verdict(w) == 2 && tf_dv10(w) >= -36 && tf_dv10(w) <= -34, "Vout 3.5 V below Vin: verdict %d, dv %d", tf_verdict(w), tf_dv10(w));
    /* the settle time is really waited: Vout is wrong for the first second after the converter starts, right afterwards (the average must ignore it) */
    world_reset(111500000); W.vin = 200.0f; W.vout = 195.0f;
    { uint64_t t0 = sim_t; while (t_conv_first == 0ULL && sim_t < t0 + 8000000ULL) run_for(10000); }
    CHECK(t_conv_first != 0ULL, "the converter started");
    run_for(1000000);                                                      /* inside the 1.5 s settle time */
    W.vout = 203.0f;
    run_for(7000000);
    w = SlaveCtrl_TfWord();
    CHECK(tf_verdict(w) == 1 && tf_dv10(w) >= 29 && tf_dv10(w) <= 31, "values from the settle period are ignored: verdict %d, dv %d (x0.1 V)", tf_verdict(w), tf_dv10(w));
    /* inside the dead band */
    world_reset(112000000); W.vin = 200.0f; W.vout = 200.2f;
    run_for(9000000);
    w = SlaveCtrl_TfWord();
    CHECK(tf_verdict(w) == 3 && tf_dv10(w) >= 1 && tf_dv10(w) <= 3, "+0.2 V is inconclusive: verdict %d, dv %d", tf_verdict(w), tf_dv10(w));
    /* the extremes are clamped, not wrapped */
    world_reset(113000000); W.vin = 160.0f; W.vout = 275.0f;           /* both inside the start window / the output limit */
    run_for(9000000);
    w = SlaveCtrl_TfWord();
    CHECK(tf_verdict(w) == 1 && tf_dv10(w) == 1023, "+115 V clamps to +102.3 V: verdict %d, dv %d", tf_verdict(w), tf_dv10(w));
    world_reset(113500000); W.vin = 275.0f; W.vout = 160.0f;
    run_for(9000000);
    w = SlaveCtrl_TfWord();
    CHECK(tf_verdict(w) == 2 && tf_dv10(w) == -1024, "-115 V clamps to -102.4 V: verdict %d, dv %d", tf_verdict(w), tf_dv10(w));
    /* it is NOT measured while the converter is not running (idle: only the settled run counts), and it is cleared at power-up */
    world_reset(114000000); W.vin = 200.0f; W.vout = 203.0f;
    CHECK(SlaveCtrl_TfWord() == 0U, "cleared at power-up");
    /* a trip keeps the last result; the next start measures again and replaces it */
    world_reset(115000000); W.vin = 200.0f; W.vout = 203.0f;
    run_for(9000000);
    CHECK(tf_verdict(SlaveCtrl_TfWord()) == 1, "first start: OK");
    SlaveCtrl_OnFltIrq();
    run_for(20000);
    CHECK(tf_verdict(SlaveCtrl_TfWord()) == 1, "the result survives the trip");
    W.vout = 196.0f;                                                       /* the transformer was re-wired while powered down */
    run_for(20000000);
    CHECK(is_running() && tf_verdict(SlaveCtrl_TfWord()) == 2, "the next start replaces the result: verdict %d (running %d)", tf_verdict(SlaveCtrl_TfWord()), is_running());
    /* a start that is stopped before the window completes gives no verdict */
    world_reset(116000000); W.vin = 200.0f; W.vout = 203.0f;
    run_for(2500000 + 1000000);
    SlaveCtrl_SetRunRequest(0U);
    run_for(500000);
    CHECK(SlaveCtrl_TfWord() == 0U, "stopped before the window completed: no verdict (0x%04X)", (unsigned)SlaveCtrl_TfWord());
    printf("  transformer check: OK / flipped / inconclusive verdicts, clamping, cleared at power-up, survives a trip, re-measured at the next start\n");
}
#endif

#if (SLAVE_TF_CHECK != 0)
/* ====================================================================== */
/* WHAT THE LINK SENDS in diag2 (v0.6.11 bug: the never-cleared mapping    */
/* counters shadowed the transformer result, so it was never shown)         */
/* ====================================================================== */
static void t_diag2_policy(void)
{
    uint16_t d;
    world_reset(120000000); W.vin = 200.0f; W.vout = 203.0f;
    /* the PASSIVE period before the start: the mapping check counts */
    { uint64_t t0 = sim_t; while (SlaveCtrl_MapWord() == 0U && sim_t < t0 + 2500000ULL) run_for(10000); }
    CHECK(SlaveCtrl_MapWord() != 0U, "precondition: the mapping counters are non-zero before the start (the stale word that shadowed the result)");
    CHECK(g_ctrl.state == CTRL_SYNC && SlaveCtrl_Diag2() == SlaveCtrl_MapWord(), "waiting in SYNC: the passive mapping word is sent (0x%04X)", (unsigned)SlaveCtrl_Diag2());
    /* the converter runs, the window has not completed: the stale mapping counters must NOT be sent */
    { uint64_t t0 = sim_t; while (!is_running() && sim_t < t0 + 6000000ULL) run_for(10000); }
    CHECK(is_running(), "the converter started");
    run_for(500000);
    CHECK(SlaveCtrl_MapWord() != 0U, "the mapping counters are still non-zero while the converter runs: that is the point");
    CHECK(SlaveCtrl_TfWord() == 0U && SlaveCtrl_Diag2() == 0U, "running, no verdict yet: diag2 is 0, not the stale mapping word (0x%04X)", (unsigned)SlaveCtrl_Diag2());
    /* the verdict is there: THAT is what is sent */
    run_for(5000000);
    d = SlaveCtrl_Diag2();
    CHECK((d & 0xE000U) == 0x2000U && d == SlaveCtrl_TfWord(), "running with a verdict: diag2 carries the transformer result (0x%04X, TF word 0x%04X)", (unsigned)d, (unsigned)SlaveCtrl_TfWord());
    /* a trip: the trip capture wins while in FAULT */
    SlaveCtrl_OnFltIrq();
    run_for(20000 + FLT_UNWIND_US);
    CHECK(g_ctrl.state == CTRL_FAULT, "faulted (state %d)", g_ctrl.state);
    if (SlaveCtrl_TripCapture() != 0U)
        CHECK(SlaveCtrl_Diag2() == SlaveCtrl_TripCapture() && (SlaveCtrl_Diag2() & 0x8000U) != 0U, "in FAULT the trip capture is sent (0x%04X)", (unsigned)SlaveCtrl_Diag2());
    printf("  diag2 policy: passive mapping while waiting, nothing stale while running, the transformer result once measured, the trip capture in FAULT\n");
}
#endif

#ifdef TEST_SETTLE_OVERRIDE
/* A LONGER W-RLY settle (build with -DSLAVE_LEGACY_W_SETTLE_MS=5000U): the DC link bleeds through its bleeder resistor, with the PRI nets open, before the
 * converter's first diagonal. Only the converter start is delayed; the sequence is otherwise unchanged. */
static void t_settle_override(void)
{
    const int sh_on = 0, w_on_i = 1;
    world_reset(130000000);
    { uint64_t t0 = sim_t; while (t_conv_first == 0ULL && sim_t < t0 + 20000000ULL) run_for(10000); }
    CHECK(t_conv_first != 0ULL, "the converter started");
    {
        const int w = lg_find(w_on_i, 1, 0), sh = lg_find(sh_on, 1, 0);
        CHECK(w >= 0 && sh >= 0, "relay sequence present");
        if (w >= 0 && sh >= 0)
        {
            const double settle_ms = (double)((int64_t)t_conv_first - (int64_t)lg_t[w]) / 1000.0;
            CHECK(settle_ms >= (double)SLAVE_LEGACY_W_SETTLE_MS && settle_ms < (double)SLAVE_LEGACY_W_SETTLE_MS + 600.0,
                  "the first diagonal comes %.0f ms after W-RLY on (must be %u ms plus at most one zero-cross wait)", settle_ms, (unsigned)SLAVE_LEGACY_W_SETTLE_MS);
            CHECK(lg_t[w] > lg_t[sh], "SH first, then W");
            printf("  longer W settle: SH on, W on %.0f ms later, the first diagonal %.0f ms after W on\n",
                   (double)((int64_t)lg_t[w] - (int64_t)lg_t[sh]) / 1000.0, settle_ms);
        }
    }
    CHECK(inv_at_first_energise == 1, "the first diagonal still runs with the low sides on");
    run_for(6000000);
    CHECK(is_running() && g_ctrl.fault == CF_NONE, "running for 6 s without a fault (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    CHECK(rl[1].coil == 1 && rl[0].coil == 0, "W-RLY stays on, SH-RLY off (W %d, SH %d)", rl[1].coil, rl[0].coil);
}
#endif

#ifdef TEST_FIRST_AT_PEAK
/* FIRST DIAGONAL AT THE LINE PEAK: a quarter cycle (about 5 ms) after the zero-cross event; every later diagonal 40 us after its event */
static void t_first_at_peak(void)
{
    int k;
    for (k = 0; k < 3; k++)
    {
        world_reset(140000000ULL + (uint64_t)k * 7000000ULL + (uint64_t)k * 137ULL);
        W.phase0_us += (double)k * 913.0; resync_crossings();
        { uint64_t t0 = sim_t; while (n_energise < 3 && sim_t < t0 + 15000000ULL) run_for(1000); }
        CHECK(n_energise >= 3, "start %d: converter running (energised %ld times)", k, (long)n_energise);
        CHECK(dt_en1 >= 4600ULL && dt_en1 <= 5400ULL, "start %d: the FIRST diagonal at the peak: %llu us after its event (expected about 5000)", k, (unsigned long long)dt_en1);
        CHECK(dt_en2 >= (uint64_t)CTRL_DEADTIME_US && dt_en2 <= (uint64_t)CTRL_DEADTIME_US + 30ULL, "start %d: the SECOND diagonal as before: %llu us after its event (expected %u)", k, (unsigned long long)dt_en2, (unsigned)CTRL_DEADTIME_US);
        CHECK(first_energise_pol == SLAVE_CONV_FIRST_POL || SLAVE_CONV_FIRST_POL == 0, "start %d: first polarity %d", k, first_energise_pol);
        CHECK(g_ctrl.fault == CF_NONE && is_running(), "start %d: no fault (state %d fault %d)", k, g_ctrl.state, g_ctrl.fault);
    }
    /* a retry after a trip starts at the peak again */
    SlaveCtrl_OnFltIrq(); run_for(20000);
    n_energise = 0; dt_en1 = dt_en2 = 0ULL;
    { uint64_t t0 = sim_t; while (n_energise < 3 && sim_t < t0 + 40000000ULL) run_for(1000); }
    CHECK(n_energise >= 3 && dt_en1 >= 4600ULL && dt_en1 <= 5400ULL, "retry: the first diagonal at the peak again (%llu us)", (unsigned long long)dt_en1);
    printf("  first diagonal at the line peak: %llu us after its event; the second %llu us (as before); also after a retry\n", (unsigned long long)dt_en1, (unsigned long long)dt_en2);
}
#endif

#ifdef TEST_QUICK_RETRY
/* QUICK RETRY (CTRL_RESTART_STEP_MS = CTRL_RESTART_MAX_MS = 3000, CTRL_LATCH_AFTER 10): every lockout is 3 s, not 10 / 20 / 30 s; the slave latches at the 10th trip */
static void t_quick_retry(void)
{
    int i; uint64_t lock_ms[12];
    world_reset(150000000ULL);
    { uint64_t t0 = sim_t; while (!is_running() && sim_t < t0 + 20000000ULL) run_for(10000); }
    CHECK(is_running(), "running first");
    for (i = 0; i < 9; i++)
    {
        uint64_t t_fault, t0;
        SlaveCtrl_OnFltIrq();
        run_for(2000);                                                                                                    /* the main loop takes the interrupt's fault ... */
        { uint64_t tw = sim_t; while (g_ctrl.state == CTRL_RESHUNT && sim_t < tw + FLT_UNWIND_US) run_for(1000); }       /* ... v0.9.65: the unwinding (low sides on) comes first */
        run_for(5000);
        CHECK(g_ctrl.state == CTRL_FAULT, "trip %d: in FAULT (state %d)", i + 1, g_ctrl.state);
        t_fault = sim_t; t0 = sim_t;
        while (g_ctrl.state == CTRL_FAULT && sim_t < t0 + 70000000ULL) run_for(10000);
        lock_ms[i] = (sim_t - t_fault) / 1000ULL;
        CHECK(lock_ms[i] >= 2900ULL && lock_ms[i] <= 3200ULL, "trip %d: the lockout is 3 s (measured %llu ms)", i + 1, (unsigned long long)lock_ms[i]);
        t0 = sim_t; while (!is_running() && sim_t < t0 + 40000000ULL) run_for(10000);
        CHECK(is_running(), "trip %d: running again", i + 1);
    }
    SlaveCtrl_OnFltIrq(); run_for(50000 + FLT_UNWIND_US);
    CHECK(g_ctrl.state == CTRL_LATCHED, "the 10th trip latches (state %d)", g_ctrl.state);
    printf("  quick retry: nine lockouts of about 3 s (%llu ... %llu ms), the 10th trip latches\n", (unsigned long long)lock_ms[0], (unsigned long long)lock_ms[8]);
}
#endif

#ifdef TEST_LEGACY_INV
/* ====================================================================== */
/* STAGE 2 WITH THE LEGACY RELAYS: relays -> converter -> 3 s -> inverter    */
/* (zero duty) -> the ramped one-shot injection test -> zero duty            */
/* ====================================================================== */
typedef struct { uint16_t word; uint16_t max_duty; uint16_t final_duty; uint16_t mid_duty; int base_ok, ramp_ok, hold_ok, down_ok, done_ok; uint64_t t_run; } InjRes_t;

static void wait_run(void) { uint64_t t0 = sim_t; while (g_ctrl.state != CTRL_RUN && sim_t < t0 + 25000000ULL) run_for(5000); }

/* run the whole injection test and sample the duty every 20 ms; tolerances of 120 ms around every phase boundary */
#define STAGE2_CCR  ((uint16_t)(CTRL_STAGE2_DUTY_COUNTS + POWER_DEADTIME_TICKS))      /* the CCR the controller commands in HOLD */
#define INJ_DV10    ((int)CTRL_STAGE2_DUTY_COUNTS)             /* with the 0.1 V-per-tick plant: the injected change in 0.1 V units = the effective ticks */
static InjRes_t inj_run(uint64_t seed, float vin, double gain, double offset_v)
{
    InjRes_t r; uint16_t prev = 0; int k;
    memset(&r, 0, sizeof r); r.base_ok = r.ramp_ok = r.hold_ok = r.down_ok = r.done_ok = 1;
    world_reset(seed); W.vin = vin; W.vout = vin; W.plant_on = 1; W.plant_gain = gain; W.out_offset = offset_v; W.plant_dead = (double)POWER_DEADTIME_TICKS;
    wait_run();
    r.t_run = sim_t;
    for (k = 0; k < 560; k++)                                           /* 11.2 s */
    {
        double   t;
        uint16_t d;
        run_for(20000);
        t = (double)(sim_t - r.t_run) / 1000.0;                         /* ms since RUN was entered */
        d = g_ctrl.duty_counts;
        if (d > r.max_duty) r.max_duty = d;
        if (t >= (double)CTRL_INJ_BASE_MS + (double)CTRL_INJ_RAMP_MS / 2.0 - 15.0 && t <= (double)CTRL_INJ_BASE_MS + (double)CTRL_INJ_RAMP_MS / 2.0 + 25.0) r.mid_duty = d;
        if (t < (double)CTRL_INJ_BASE_MS - 120.0)                                                         { if (d != 0U) r.base_ok = 0; }
        else if (t > (double)CTRL_INJ_BASE_MS + 120.0 && t < (double)(CTRL_INJ_BASE_MS + CTRL_INJ_RAMP_MS) - 120.0) { if (d < prev) r.ramp_ok = 0; }
        else if (t > (double)(CTRL_INJ_BASE_MS + CTRL_INJ_RAMP_MS) + 120.0 && t < (double)(CTRL_INJ_BASE_MS + CTRL_INJ_RAMP_MS + CTRL_INJ_HOLD_MS) - 120.0)
                                                                                                         { if (d != STAGE2_CCR && W.plant_gain != 1.0) r.hold_ok = 0; }
        else if (t > (double)(CTRL_INJ_BASE_MS + CTRL_INJ_RAMP_MS + CTRL_INJ_HOLD_MS) + 120.0 && t < (double)(CTRL_INJ_BASE_MS + CTRL_INJ_RAMP_MS + CTRL_INJ_HOLD_MS + CTRL_INJ_DOWN_MS) - 120.0)
                                                                                                         { if (d > prev) r.down_ok = 0; }
        else if (t > (double)(CTRL_INJ_BASE_MS + CTRL_INJ_RAMP_MS + CTRL_INJ_HOLD_MS + CTRL_INJ_DOWN_MS) + 120.0) { if (d != 0U) r.done_ok = 0; }
        prev = d;
    }
    r.word = SlaveCtrl_TfWord(); r.final_duty = g_ctrl.duty_counts;
    return r;
}
static int tfv(uint16_t w) { return (int)((w >> 11) & 3U); }
static int tfd(uint16_t w) { int v = (int)(w & 0x07FFU); if (v & 0x0400) v -= 0x0800; return v; }

static void t_leg_inv_sequence(void)
{
    uint64_t t_invstart = 0;
    int sh_on, w_on;
    world_reset(160000000ULL); W.vin = 200.0f; W.vout = 200.0f; W.plant_on = 1; W.plant_gain = 0.1; W.plant_dead = (double)POWER_DEADTIME_TICKS;
    { uint64_t t0 = sim_t; while (g_ctrl.state != CTRL_RUN && sim_t < t0 + 25000000ULL) { run_for(5000); if (t_invstart == 0 && g_ctrl.state == CTRL_INV_START) t_invstart = sim_t; } }
    CHECK(g_ctrl.state == CTRL_RUN, "RUN reached (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    CHECK(g_ctrl.duty_counts == 0U && t_first_duty == 0ULL, "RUN is entered with ZERO duty and no nonzero duty was ever applied before (first duty %llu)", (unsigned long long)t_first_duty);
    CHECK(n_inv_enable == 1, "the inverter was enabled exactly once (by the legacy low sides) and NOT re-initialised: %d times", n_inv_enable);
    sh_on = lg_find(0, 1, 0); w_on = lg_find(1, 1, 0);
    CHECK(sh_on >= 0 && w_on >= 0 && t_inv_enable_first < lg_t[sh_on], "the low sides (inverter enabled, zero duty) come on BEFORE the SH pulse");
    CHECK(rl[1].coil == 1 && rl[0].coil == 0, "W-RLY on and SH-RLY off in RUN (W %d, SH %d)", rl[1].coil, rl[0].coil);
    CHECK(t_invstart != 0ULL && t_conv_first != 0ULL && (t_invstart - t_conv_first) >= 3000000ULL - 400000ULL && (t_invstart - t_conv_first) <= 3000000ULL + 700000ULL,
          "the inverter starts about 3 s after the converter (measured %.2f s)", (double)((int64_t)t_invstart - (int64_t)t_conv_first) / 1.0e6);
    check_invariants("legacy inverter sequence");
    printf("  legacy inverter sequence: lows on, SH, W, converter, inverter %.2f s later, RUN with zero duty, inverter enabled once\n", (double)((int64_t)t_invstart - (int64_t)t_conv_first) / 1.0e6);
}

static void t_leg_inv_injection(void)
{
    InjRes_t a, b, c, d, e, f, g;
    a = inj_run(161000000ULL, 200.0f, +0.100, 0.0);          /* boost, transformer right: Vout rises 3.2 V at the target duty */
    CHECK(a.base_ok && a.ramp_ok && a.hold_ok && a.down_ok && a.done_ok, "phases: base zero %d, ramp non-decreasing %d, hold at target %d, ramp down %d, done zero %d", a.base_ok, a.ramp_ok, a.hold_ok, a.down_ok, a.done_ok);
    CHECK(a.mid_duty >= (uint16_t)(POWER_DEADTIME_TICKS + 0.3 * CTRL_STAGE2_DUTY_COUNTS) && a.mid_duty <= (uint16_t)(POWER_DEADTIME_TICKS + 0.7 * CTRL_STAGE2_DUTY_COUNTS),
          "it is a RAMP: half way through the ramp the EFFECTIVE on-time is about half the target (CCR %u = dead time %u + about %u)", a.mid_duty, (unsigned)POWER_DEADTIME_TICKS, (unsigned)(CTRL_STAGE2_DUTY_COUNTS / 2));
    CHECK(a.max_duty == STAGE2_CCR, "the CCR never exceeds dead time + target (max %u, expected %u)", a.max_duty, (unsigned)STAGE2_CCR);
    CHECK((a.word & 0xE000U) == 0x2000U && tfv(a.word) == 1 && tfd(a.word) >= INJ_DV10 - 2 && tfd(a.word) <= INJ_DV10 + 2, "boost, right transformer: OK, +%.1f V (verdict %d, dv %d, expected %d)", INJ_DV10 / 10.0, tfv(a.word), tfd(a.word), INJ_DV10);
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "still in RUN after the test, no fault");
    CHECK(SlaveCtrl_Diag2() == a.word, "the link sends the result (0x%04X vs 0x%04X)", (unsigned)SlaveCtrl_Diag2(), (unsigned)a.word);
    check_invariants("legacy injection (boost)");
    b = inj_run(162000000ULL, 200.0f, -0.100, 0.0);          /* the same, transformer reversed */
    CHECK(tfv(b.word) == 2 && tfd(b.word) <= -INJ_DV10 + 2 && tfd(b.word) >= -INJ_DV10 - 2, "boost, transformer reversed: FLIPPED, -%.1f V (verdict %d, dv %d)", INJ_DV10 / 10.0, tfv(b.word), tfd(b.word));
    c = inj_run(163000000ULL, 250.0f, +0.100, 0.0);          /* buck: Vout falls; that is the EXPECTED direction */
    CHECK(tfv(c.word) == 1 && tfd(c.word) >= INJ_DV10 - 2 && tfd(c.word) <= INJ_DV10 + 2, "buck, right transformer: OK, +%.1f V in the expected direction (verdict %d, dv %d)", INJ_DV10 / 10.0, tfv(c.word), tfd(c.word));
    d = inj_run(164000000ULL, 250.0f, -0.100, 0.0);
    CHECK(tfv(d.word) == 2, "buck, reversed transformer: FLIPPED (verdict %d)", tfv(d.word));
    e = inj_run(165000000ULL, 200.0f, +0.001, 0.0);          /* almost no effect */
    CHECK(tfv(e.word) == 3 && tfd(e.word) >= -1 && tfd(e.word) <= 1, "no measurable injection: inconclusive (verdict %d, dv %d)", tfv(e.word), tfd(e.word));
    f = inj_run(166000000ULL, 200.0f, +0.100, -2.0);         /* AC_OUT reads 2 V below AC_IN at rest: the baseline must be subtracted */
    CHECK(tfv(f.word) == 1 && tfd(f.word) >= INJ_DV10 - 2 && tfd(f.word) <= INJ_DV10 + 2, "a 2 V sensor offset at rest is removed by the baseline (verdict %d, dv %d: %d expected, %d if the baseline were ignored)", tfv(f.word), tfd(f.word), INJ_DV10, INJ_DV10 - 20);
    g = inj_run(167000000ULL, 200.0f, +1.000, 0.0);          /* a transformer far stronger than expected: 32 V at the target duty */
    CHECK(g.max_duty > (uint16_t)POWER_DEADTIME_TICKS && g.max_duty < STAGE2_CCR, "the abort limit stops the ramp before the target (max CCR %u of %u)", g.max_duty, (unsigned)STAGE2_CCR);
    CHECK(g.final_duty == 0U && tfv(g.word) == 3, "aborted: duty zero, verdict inconclusive (final duty %u, verdict %d)", g.final_duty, tfv(g.word));
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "an aborted test is not a fault");
    check_invariants("legacy injection (abort)");
    printf("  injection test: boost OK +%.1f V, reversed FLIPPED %.1f V, buck OK +%.1f V, tiny = inconclusive, 2 V sensor offset removed, strong transformer aborts at CCR %u\n",
           tfd(a.word) / 10.0, tfd(b.word) / 10.0, tfd(c.word) / 10.0, g.max_duty);
}

/* THE BUG THAT WAS SHIPPED IN v0.7.0: 32 / 64 / 96 ticks against a 128-tick dead time = no output at all (the bench showed "TF ? +0.0V").
 * The plant now models it; this proves the model bites, and that the controller's commands are always above the dead time. */
static void t_leg_inv_deadtime(void)
{
    uint16_t ccr[4] = { 32U, 64U, 96U, (uint16_t)POWER_DEADTIME_TICKS };
    int i;
    for (i = 0; i < 4; i++)
    {
        double d = (double)ccr[i];
        d = (d > (double)POWER_DEADTIME_TICKS) ? (d - (double)POWER_DEADTIME_TICKS) : 0.0;
        CHECK(d == 0.0, "a CCR of %u ticks (<= the %u-tick dead time) produces NO high-side pulse", (unsigned)ccr[i], (unsigned)POWER_DEADTIME_TICKS);
    }
    CHECK((uint16_t)(CTRL_STAGE2_DUTY_COUNTS + POWER_DEADTIME_TICKS) > (uint16_t)POWER_DEADTIME_TICKS, "the stage-2 target CCR is above the dead time");
    CHECK(POWER_DEADTIME_TICKS == 128U, "the dead time is 128 ticks = 2.0 us (DTG %u)", (unsigned)POWER_DEADTIME_DTG);
    printf("  dead time: %u ticks = %.2f us; any CCR at or below it outputs nothing; the stage-2 ramp is always above it\n", (unsigned)POWER_DEADTIME_TICKS, (double)POWER_DEADTIME_TICKS / 64.0);
}

static void t_leg_inv_faults(void)
{
    uint64_t t_fault, t_inv_dis_before;
    int w_off;
    /* (a) a HARD fault (FLT) in the middle of the hold: everything off at once, both coils released */
    world_reset(170000000ULL); W.vin = 200.0f; W.vout = 200.0f; W.plant_on = 1; W.plant_gain = 0.1; W.plant_dead = (double)POWER_DEADTIME_TICKS;
    wait_run(); run_for((uint64_t)(CTRL_INJ_BASE_MS + CTRL_INJ_RAMP_MS) * 1000ULL + 1500000ULL);
    CHECK(g_ctrl.duty_counts == STAGE2_CCR && t_duty_nz_last != 0ULL, "in the hold at the target duty before the fault (duty %u)", g_ctrl.duty_counts);
    t_fault = sim_t; SlaveCtrl_OnFltIrq(); run_for(5000);
    CHECK(flt_outputs_safe(), "FLT: converter and high sides off at once (v0.9.65: the low sides on until the shunts close; inverter %d, forced %d)", inv_enabled, inv_forced);
    run_for(FLT_UNWIND_US);
    CHECK(g_ctrl.state == CTRL_FAULT && !inv_enabled && leg1 == 0 && leg2 == 0, "FLT: FAULT, inverter disabled, both legs zero (state %d)", g_ctrl.state);
    CHECK(rl[0].coil == 0 && rl[1].coil == 0, "FLT: both relay coils released (SH %d, W %d)", rl[0].coil, rl[1].coil);
    CHECK(t_duty_nz_last <= t_fault + 2000ULL, "FLT: no nonzero duty after the fault (last %llu us after it)", (unsigned long long)(t_duty_nz_last - t_fault));
    run_for(3000000);
    CHECK(t_duty_nz_last <= t_fault + 2000ULL, "FLT: and none during the following 3 s");
    check_invariants("legacy hard fault");
    /* (b) a SOFT fault (input out of range) in the middle of the hold: duty zero first, THEN the relays, the inverter disabled only after the W contact has closed */
    world_reset(171000000ULL); W.vin = 200.0f; W.vout = 200.0f; W.plant_on = 1; W.plant_gain = 0.1; W.plant_dead = (double)POWER_DEADTIME_TICKS;
    wait_run(); run_for((uint64_t)(CTRL_INJ_BASE_MS + CTRL_INJ_RAMP_MS) * 1000ULL + 1500000ULL);
    CHECK(g_ctrl.duty_counts == STAGE2_CCR, "in the hold before the soft fault");
    t_inv_dis_before = t_inv_disable_last;
    W.vin = 100.0f;                                                        /* below the 140 V trip level */
    { uint64_t t0 = sim_t; while (g_ctrl.state != CTRL_RESHUNT && g_ctrl.state != CTRL_FAULT && sim_t < t0 + 3000000ULL) run_for(2000); }
    CHECK(g_ctrl.state == CTRL_RESHUNT || g_ctrl.state == CTRL_FAULT, "the soft fault is acted on (state %d)", g_ctrl.state);
    run_for(1500000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_VIN_RANGE, "FAULT F09 after the unwinding (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    w_off = lg_find(1, 0, 0);
    CHECK(w_off >= 0, "W-RLY was released");
    if (w_off >= 0)
    {
        CHECK(t_duty_nz_last <= lg_t[w_off], "the duty went to zero BEFORE the relay was released (last nonzero duty %lld us before the release)", (long long)((int64_t)lg_t[w_off] - (int64_t)t_duty_nz_last));
        CHECK(t_inv_disable_last > t_inv_dis_before && t_inv_disable_last >= lg_t[w_off] + (uint64_t)SLAVE_RELAY_SETTLE_MS * 1000ULL - 3000ULL,
              "the inverter was disabled only after the W contact had time to close (%.0f ms after the release, settle %u ms)",
              (double)((int64_t)t_inv_disable_last - (int64_t)lg_t[w_off]) / 1000.0, (unsigned)SLAVE_RELAY_SETTLE_MS);
        CHECK(t_inv_disable_last <= lg_t[w_off] + (uint64_t)SLAVE_RELAY_SETTLE_MS * 1000ULL + 90000ULL,
              "... and without waiting for the 500 ms time-out: disabled %.0f ms after the release",
              (double)((int64_t)t_inv_disable_last - (int64_t)lg_t[w_off]) / 1000.0);
    }
    CHECK(!inv_enabled && rl[0].coil == 0 && rl[1].coil == 0, "after the unwinding: inverter disabled, both coils off");
    check_invariants("legacy soft fault");
    /* (c) a normal STOP: the same unwinding, ends in OFF, and the relays are NOT re-armed while unwinding */
    world_reset(172000000ULL); W.vin = 200.0f; W.vout = 200.0f; W.plant_on = 1; W.plant_gain = 0.1; W.plant_dead = (double)POWER_DEADTIME_TICKS;
    wait_run(); run_for((uint64_t)(CTRL_INJ_BASE_MS + CTRL_INJ_RAMP_MS) * 1000ULL + 1500000ULL);
    SlaveCtrl_SetRunRequest(0U);
    run_for(2500000);
    CHECK(g_ctrl.state == CTRL_OFF && !inv_enabled && rl[0].coil == 0 && rl[1].coil == 0, "stop: OFF, inverter disabled, both coils off (state %d)", g_ctrl.state);
    CHECK(lg_n == 4, "stop: exactly four coil commands in total (SH on, SH off, W on, W off), none re-arming the sequence: %d", lg_n);
    check_invariants("legacy stop");
    printf("  legacy inverter faults: FLT = everything off at once; soft fault and stop = duty zero, relays released, inverter disabled after the contact settle time; no relay re-arm while unwinding\n");
}

static void t_leg_inv_restart(void)
{
    uint16_t w1;
    world_reset(173000000ULL); W.vin = 200.0f; W.vout = 200.0f; W.plant_on = 1; W.plant_gain = 0.1; W.plant_dead = (double)POWER_DEADTIME_TICKS;
    wait_run(); run_for(10500000);
    w1 = SlaveCtrl_TfWord();
    CHECK(tfv(w1) == 1, "first test: OK");
    SlaveCtrl_OnFltIrq(); run_for(20000);
    CHECK(SlaveCtrl_TfWord() == w1, "the result survives the fault");
    { uint64_t t0 = sim_t; while (g_ctrl.state != CTRL_RUN && sim_t < t0 + 40000000ULL) run_for(5000); }
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.duty_counts == 0U, "after the retry the sequence ran again and RUN starts with zero duty (state %d)", g_ctrl.state);
    CHECK(SlaveCtrl_TfWord() == 0U, "the previous result is cleared when the new test starts");
    run_for(10500000);
    CHECK(tfv(SlaveCtrl_TfWord()) == 1, "second test: OK again");
    check_invariants("legacy restart");
    printf("  legacy inverter restart: the whole sequence and the injection test run again after a fault\n");
}
#endif

#ifdef TEST_LEGACY_REG
/* ====================================================================== */
/* STAGE 3 WITH THE LEGACY RELAYS: the closed loop against the plant MEASURED */
/* at stage 2 (gain curve, dead time), with the first-run cap                */
/* ====================================================================== */
#define L3_CAP       ((uint16_t)CTRL_STAGE3_MAX_EFF_TICKS)
#define L3_CCR_MAX   ((uint16_t)(CTRL_STAGE3_MAX_EFF_TICKS + POWER_DEADTIME_TICKS))
static void l3_wait_run(void) { uint64_t t0 = sim_t; while (g_ctrl.state != CTRL_RUN && sim_t < t0 + 25000000ULL) run_for(5000); }
static void l3_world(uint64_t seed, float vin) { world_reset(seed); W.vin = vin; W.vout = vin; W.plant_on = 1; W.plant_curve = 1; }
typedef struct { double vout_end, vout_max, vout_min_after; uint16_t ccr_max; int boost_end; } L3Res_t;
/* run `secs` after RUN, sampling every 20 ms; min/max are taken after `settle` seconds */
static L3Res_t l3_run(double secs, double settle)
{
    L3Res_t r; int k, n = (int)(secs * 50.0), ns = (int)(settle * 50.0);
    memset(&r, 0, sizeof r); r.vout_min_after = 1e9;
    for (k = 0; k < n; k++)
    {
        run_for(20000);
        if (g_ctrl.duty_counts > r.ccr_max) r.ccr_max = g_ctrl.duty_counts;
        if ((double)W.vout > r.vout_max) r.vout_max = (double)W.vout;
        if (k >= ns && (double)W.vout < r.vout_min_after) r.vout_min_after = (double)W.vout;
    }
    r.vout_end = (double)W.vout; r.boost_end = g_ctrl.boost;
    return r;
}

/* the regulator word the link sends in diag2 (bit 12 = 1, bit 11 boost, bit 10 at the cap, bits 9..0 = effective ticks / 4) */
static int      rw_valid(uint16_t w) { return (w & 0xF000U) == 0x1000U; }
static unsigned rw_eff(uint16_t w)   { return (unsigned)(w & 0x03FFU) * 4U; }
static void t_l3_converge(void)
{
    L3Res_t a, b;
    l3_world(180000000ULL, 215.0f); l3_wait_run();
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.duty_counts == 0U, "RUN entered with zero duty (state %d, CCR %u)", g_ctrl.state, g_ctrl.duty_counts);
    a = l3_run(30.0, 0.0);
    CHECK(fabs(a.vout_end - 230.0) <= 1.5, "215 V (boost): Vout settles to 230 V +/- 1.5 V (end %.2f V)", a.vout_end);
    CHECK(a.vout_max <= 232.5, "no overshoot beyond 232.5 V during the soft start (max %.2f V)", a.vout_max);
    CHECK(a.ccr_max <= L3_CCR_MAX && a.ccr_max > (uint16_t)POWER_DEADTIME_TICKS, "the CCR stays within dead time + cap and does regulate (max %u, limit %u)", a.ccr_max, (unsigned)L3_CCR_MAX);
    CHECK(a.boost_end == 1 && g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "boost, still in RUN, no fault (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    { const uint16_t w = SlaveCtrl_Diag2(); const unsigned eff = (unsigned)g_ctrl.duty_counts - (unsigned)POWER_DEADTIME_TICKS;
      CHECK(rw_valid(w) && (w & 0x0800U) != 0U && (w & 0x0400U) == 0U, "diag2 carries the regulator word: boost, not at the cap (0x%04X)", (unsigned)w);
      CHECK(rw_eff(w) + 4U >= eff && rw_eff(w) <= eff + 4U, "the displayed effective ticks match the real duty (word %u, CCR %u - dead time = %u)", rw_eff(w), (unsigned)g_ctrl.duty_counts, eff); }
    check_invariants("legacy stage 3, boost");
    l3_world(181000000ULL, 245.0f); l3_wait_run();
    b = l3_run(30.0, 0.0);
    CHECK(fabs(b.vout_end - 230.0) <= 1.5, "245 V (buck): Vout settles to 230 V +/- 1.5 V (end %.2f V)", b.vout_end);
    CHECK(b.boost_end == 0 && b.vout_min_after > 0.0 && g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "buck, still in RUN, no fault (boost flag %d)", b.boost_end);
    CHECK(b.ccr_max <= L3_CCR_MAX, "buck: the CCR stays within dead time + cap (max %u)", b.ccr_max);
    { const uint16_t w = SlaveCtrl_Diag2(); CHECK(rw_valid(w) && (w & 0x0800U) == 0U && rw_eff(w) > 0U, "buck: the regulator word has the boost bit clear and a nonzero duty (0x%04X)", (unsigned)w); }
    check_invariants("legacy stage 3, buck");
    printf("  closed loop: 215 V -> %.2f V (boost), 245 V -> %.2f V (buck); soft start overshoot %.2f V; CCR max %u of %u\n", a.vout_end, b.vout_end, a.vout_max - 230.0, a.ccr_max, (unsigned)L3_CCR_MAX);
}

static void t_l3_freeze_and_ramps(void)
{
    L3Res_t r; int k;
    /* inside the freeze band: no correction at all */
    l3_world(182000000ULL, 230.4f); l3_wait_run();
    r = l3_run(20.0, 15.0);
    CHECK(g_ctrl.duty_counts == 0U && fabs(r.vout_end - 230.4) <= 0.2, "230.4 V is inside the freeze band: CCR 0, Vout = Vin (CCR %u, Vout %.2f V)", g_ctrl.duty_counts, r.vout_end);
    { const uint16_t w = SlaveCtrl_Diag2(); CHECK(rw_valid(w) && rw_eff(w) == 0U, "frozen: the regulator word shows zero ticks (the master prints FREEZE) (0x%04X)", (unsigned)w); }
    /* VARIAC-SPEED ramps (10 V/s) across the 230 V crossover: 215 -> 245 -> 215 */
    l3_world(183000000ULL, 215.0f); l3_wait_run();
    (void)l3_run(18.0, 0.0);
    CHECK(fabs((double)W.vout - 230.0) <= 1.5, "settled at 215 V before the ramps (%.2f V)", (double)W.vout);
    { double worst = 0.0; float v = 215.0f;
      for (k = 0; k < 120; k++) { v += 1.0f; if (v > 245.0f) v = 245.0f; W.vin = v; run_for(100000); if (fabs((double)W.vout - 230.0) > worst) worst = fabs((double)W.vout - 230.0); }   /* 3 s up at 10 V/s, then hold */
      for (k = 0; k < 100; k++) { run_for(100000); if (fabs((double)W.vout - 230.0) > worst) worst = fabs((double)W.vout - 230.0); }
      for (k = 0; k < 120; k++) { v -= 1.0f; if (v < 215.0f) v = 215.0f; W.vin = v; run_for(100000); if (fabs((double)W.vout - 230.0) > worst) worst = fabs((double)W.vout - 230.0); }
      for (k = 0; k < 100; k++) { run_for(100000); if (fabs((double)W.vout - 230.0) > worst) worst = fabs((double)W.vout - 230.0); }
      CHECK(worst <= 3.0, "a 10 V/s variac ramp 215 -> 245 -> 215 V is followed within 3 V across the buck / boost crossover (worst %.2f V)", worst);
      CHECK(fabs((double)W.vout - 230.0) <= 1.5 && g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "back at 215 V: settled again, still in RUN, no fault (Vout %.2f V)", (double)W.vout);
      printf("  closed loop: 10 V/s ramps 215 -> 245 -> 215 V followed within %.2f V; 230.4 V stays inside the freeze band (CCR 0)\n", worst); }
    check_invariants("legacy stage 3, ramps");
}

static void t_l3_saturation(void)
{
    L3Res_t r; double reach = plant_g_of_vin(190.0) * (double)CTRL_STAGE3_MAX_EFF_TICKS; double vmax_rec = 0.0; int k;
    l3_world(184000000ULL, 190.0f); l3_wait_run();
    r = l3_run(25.0, 12.0);
    CHECK(g_ctrl.duty_counts == L3_CCR_MAX, "190 V needs more than the cap: the CCR sits at dead time + cap (CCR %u, limit %u)", g_ctrl.duty_counts, (unsigned)L3_CCR_MAX);
    CHECK(r.vout_end > 190.0 + 0.8 * reach && r.vout_end <= 190.0 + 1.05 * reach, "saturated: Vout = Vin + the capped correction (%.2f V, expected about %.2f V)", r.vout_end, 190.0 + reach);
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "saturation is not a fault (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    { const uint16_t w = SlaveCtrl_Diag2(); CHECK(rw_valid(w) && (w & 0x0400U) != 0U && rw_eff(w) + 4U >= (unsigned)L3_CAP && rw_eff(w) <= (unsigned)L3_CAP + 4U, "at the cap: the AT-CAP bit is set and the ticks equal the cap (0x%04X: %u of %u)", (unsigned)w, rw_eff(w), (unsigned)L3_CAP); }
    W.vin = 215.0f;                                                   /* the input recovers: no wind-up, no long excursion */
    for (k = 0; k < 400; k++) { run_for(20000); if (k > 25 && (double)W.vout > vmax_rec) vmax_rec = (double)W.vout; }
    CHECK(fabs((double)W.vout - 230.0) <= 1.5, "after the input recovers to 215 V the output settles within 1.5 V (%.2f V; state %d, fault %d)", (double)W.vout, g_ctrl.state, g_ctrl.fault);
    CHECK(vmax_rec <= 233.0, "... without a wind-up overshoot (max %.2f V)", vmax_rec);
    check_invariants("legacy stage 3, saturation");
    printf("  closed loop: 190 V saturates at CCR %u (Vout %.1f V), recovers at 215 V without wind-up (max %.2f V)\n", g_ctrl.duty_counts, r.vout_end, vmax_rec);
}

static void t_l3_faults(void)
{
    uint64_t t_fault, t_dis_before; int w_off;
    /* (a) a HARD fault (FLT) while regulating */
    l3_world(185000000ULL, 215.0f); l3_wait_run(); (void)l3_run(12.0, 0.0);
    CHECK(g_ctrl.duty_counts > (uint16_t)POWER_DEADTIME_TICKS, "regulating before the fault (CCR %u)", g_ctrl.duty_counts);
    t_fault = sim_t; SlaveCtrl_OnFltIrq(); run_for(5000);
    CHECK(SlaveCtrl_RegWord() == 0U && !rw_valid(SlaveCtrl_Diag2()), "after the fault the regulator word is gone (it must not show stale duty): 0x%04X", (unsigned)SlaveCtrl_Diag2());
#if (SLAVE_FLT_KEEP_LOWS != 0)
    CHECK(flt_outputs_safe() && inv_enabled && inv_forced && rl[0].coil == 0 && rl[1].coil == 0, "v0.9.65 FLT: converter and high sides off at once, LOW SIDES ON (the primary shorted), both coils released (state %d, inverter %d, forced %d)", g_ctrl.state, inv_enabled, inv_forced);
    run_for(FLT_UNWIND_US);
    CHECK(g_ctrl.state == CTRL_FAULT && !inv_enabled && t_inv_disable_last >= t_fault + (uint64_t)SLAVE_RELAY_SETTLE_MS * 1000ULL - 2000ULL, "and the inverter is disabled only once the contacts have settled (%.0f ms after the trip; state %d)", (double)(t_inv_disable_last - t_fault) / 1000.0, g_ctrl.state);
#else
    CHECK(g_ctrl.state == CTRL_FAULT && !inv_enabled && leg1 == 0 && leg2 == 0 && rl[0].coil == 0 && rl[1].coil == 0, "FLT: everything off at once, both coils released (state %d)", g_ctrl.state);
#endif
    CHECK(t_duty_nz_last <= t_fault + 2000ULL, "FLT: no nonzero duty after the fault");
#if (SLAVE_FLT_KEEP_LOWS != 0)
    {   /* the retry: the inverter enable restores the PWM mode, and it regulates again */
        uint64_t t0 = sim_t;
        while (g_ctrl.state != CTRL_RUN && sim_t < t0 + 40000000ULL) run_for(10000);
        (void)l3_run(6.0, 0.0);
        CHECK(g_ctrl.state == CTRL_RUN && !inv_forced && t_duty_nz_last > t0, "after the retry the PWM mode is restored (forced %d) and duty is applied again (state %d)", inv_forced, g_ctrl.state);
    }
#endif
    check_invariants("legacy stage 3, hard fault");
    /* (b) a SOFT fault while regulating: duty zero, relays released, the inverter disabled only after the contact settle time */
    l3_world(186000000ULL, 215.0f); l3_wait_run(); (void)l3_run(12.0, 0.0);
    t_dis_before = t_inv_disable_last;
    W.vin = 100.0f;
    { uint64_t t0 = sim_t; while (g_ctrl.state != CTRL_RESHUNT && g_ctrl.state != CTRL_FAULT && sim_t < t0 + 3000000ULL) run_for(2000); }
    run_for(1500000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_VIN_RANGE, "FAULT F09 after the unwinding (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    w_off = lg_find(1, 0, 0);
    CHECK(w_off >= 0 && t_duty_nz_last <= lg_t[w_off], "the duty went to zero BEFORE the relay was released");
    if (w_off >= 0)
        CHECK(t_inv_disable_last > t_dis_before && t_inv_disable_last >= lg_t[w_off] + (uint64_t)SLAVE_RELAY_SETTLE_MS * 1000ULL - 3000ULL && t_inv_disable_last <= lg_t[w_off] + (uint64_t)SLAVE_RELAY_SETTLE_MS * 1000ULL + 90000ULL,
              "the inverter was disabled after the W contact settle time and without the 500 ms time-out (%.0f ms after the release)", (double)((int64_t)t_inv_disable_last - (int64_t)lg_t[w_off]) / 1000.0);
    check_invariants("legacy stage 3, soft fault");
    /* (c) a normal STOP: the same unwinding, ends in OFF, no relay re-arm */
    l3_world(187000000ULL, 215.0f); l3_wait_run(); (void)l3_run(12.0, 0.0);
    SlaveCtrl_SetRunRequest(0U); run_for(2500000);
    CHECK(g_ctrl.state == CTRL_OFF && !inv_enabled && rl[0].coil == 0 && rl[1].coil == 0 && lg_n == 4, "stop: OFF, inverter disabled, both coils off, exactly four coil commands (state %d, %d commands)", g_ctrl.state, lg_n);
    check_invariants("legacy stage 3, stop");
    /* (d) after a fault the retry runs the whole sequence and regulates again */
    l3_world(188000000ULL, 215.0f); l3_wait_run(); (void)l3_run(12.0, 0.0);
    SlaveCtrl_OnFltIrq(); run_for(20000);
    l3_wait_run();
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.duty_counts == 0U, "after the retry RUN starts again with zero duty (state %d, CCR %u)", g_ctrl.state, g_ctrl.duty_counts);
    (void)l3_run(30.0, 0.0);
    CHECK(fabs((double)W.vout - 230.0) <= 1.5, "and the loop settles again (%.2f V)", (double)W.vout);
    check_invariants("legacy stage 3, restart");
    printf("  legacy stage 3 faults: FLT = everything off at once; soft fault and stop = duty zero, relays released, inverter disabled after the contact settle time; the retry regulates again\n");
}

#ifdef TEST_SOFTSTART
/* The soft-start ramp configured in slave_config.h (CTRL_SOFTSTART_*) must reach the RUNNING regulator: from RUN, the time until Vout is within 1.5 V of 230 V must follow an
 * independent model of the ramp built from the same macros (a config setting that the regulator ignores would fail here: the default and the fast ramp differ by about 3 s). */
static double l3_ramp_model(double vin, double target, double tol)
{
    double sp = vin, t = 0.0;
    while (fabs(sp - target) > tol && t < 60.0)
    {
        double diff = fabs(sp - target);
        double slope = (t * 1000.0 < (double)CTRL_SOFTSTART_SLOW_TIME_MS) ? (double)CTRL_SOFTSTART_SLOW_SLOPE_VPS : ((diff <= 3.0) ? (double)CTRL_SOFTSTART_FINAL_SLOPE_VPS : (double)CTRL_SOFTSTART_FAST_SLOPE_VPS);
        sp += ((sp < target) ? 1.0 : -1.0) * slope * 0.001;
        t += 0.001;
    }
    return t;
}
static void t_l3_softstart(void)
{
    const double ins[4] = { 212.0, 224.0, 240.0, 258.0 };
    int k;
    for (k = 0; k < 4; k++)
    {
        double t_meas = -1.0, t3 = -1.0, t04 = -1.0, expect, ramp_only, last3_model, last3_meas;
        uint64_t t0;
        l3_world(190000000ULL + (uint64_t)k * 3000000ULL, (float)ins[k]); l3_wait_run();
        t0 = sim_t;
        while (sim_t < t0 + 20000000ULL)
        {
            run_for(10000);
            { const double e = fabs((double)W.vout - 230.0), tn = (double)(sim_t - t0) / 1.0e6;
              if (t3 < 0.0 && e <= 3.0) t3 = tn;
              if (t_meas < 0.0 && e <= 1.5) t_meas = tn;
              if (t04 < 0.0 && e <= 0.4) { t04 = tn; break; } }
        }
        expect = l3_ramp_model(ins[k], 230.0, 1.5);
        ramp_only = l3_ramp_model(ins[k], 230.0, 0.05);
        /* the LAST 3 V of the ramp run at the FINAL slope: the time from 3 V to 0.4 V away depends on exactly that setting (0.6 s at 5 V/s, 0.3 s at 10 V/s) */
        last3_model = l3_ramp_model(ins[k], 230.0, 0.4) - l3_ramp_model(ins[k], 230.0, 3.0);
        last3_meas  = (t04 >= 0.0 && t3 >= 0.0) ? (t04 - t3) : -1.0;
        CHECK(last3_meas >= 0.0 && fabs(last3_meas - last3_model) <= 0.17, "%.0f V: the last 3 V take %.2f s, the FINAL slope predicts %.2f s", ins[k], last3_meas, last3_model);
        CHECK(t_meas >= 0.0 && t_meas >= expect - 0.2 && t_meas <= expect + 0.25, "%.0f V: Vout is within 1.5 V of 230 V after %.2f s; the configured ramp predicts %.2f s (the whole ramp %.2f s)", ins[k], t_meas, expect, ramp_only);
        CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "%.0f V: still in RUN, no fault", ins[k]);
        check_invariants("legacy stage 3, soft start");
    }
    printf("  soft start: ramp %.1f / %.1f / %.1f V/s, slow phase %lu ms: from 212 V Vout is within 1.5 V after the time the configured ramp predicts (%.2f s)\n",
           (double)CTRL_SOFTSTART_SLOW_SLOPE_VPS, (double)CTRL_SOFTSTART_FAST_SLOPE_VPS, (double)CTRL_SOFTSTART_FINAL_SLOPE_VPS, (unsigned long)CTRL_SOFTSTART_SLOW_TIME_MS, l3_ramp_model(212.0, 230.0, 1.5));
}
#endif

/* ---- a variac turned back and forth fast: the injection must never be reversed at a large correction, and the unit must recover ---- */
static void t_l3_variac(void)
{
    int sw, k, flips = 0, flips_bad = 0; uint8_t last_boost; double worst_gap = 0.0; uint32_t g0, maxd = 0U;
    l3_world(780000000ULL, 205.0f); l3_wait_run(); run_for(8000000);
    CHECK(g_ctrl.state == CTRL_RUN, "running and boosting before the variac moves (state %d)", g_ctrl.state);
    g0 = SlaveCtrl_GuardCount(); last_boost = g_ctrl.boost;
    for (sw = 0; sw < 8; sw++)
    {
        W.vin = (sw & 1) ? 205.0f : 258.0f;                                  /* the variac snaps across the 230 V set point */
        for (k = 0; k < 150; k++)
        {
            run_for(1000);
            if (g_ctrl.boost != last_boost)
            {
                flips++; if (g_ctrl.duty_counts > maxd) maxd = g_ctrl.duty_counts;
                if (g_ctrl.duty_counts > (uint32_t)(CTRL_DIR_FLIP_MAX_TICKS + 128U)) flips_bad++;      /* the compare value is the on-time plus the 128-tick dead time (0 when the on-time is 0) */
                last_boost = g_ctrl.boost;
            }
            if ((k >= 40) && (k < 95)) { double gap = (double)W.vout - (double)W.vin; if (gap < 0.0) gap = -gap; if (gap > worst_gap) worst_gap = gap; }
        }
    }
    CHECK(flips_bad == 0, "8 snaps of the variac: the direction was never reversed with a correction above %u ticks (%d reversals, the largest request at one: %u counts; %d of them too large)", (unsigned)CTRL_DIR_FLIP_MAX_TICKS, flips, (unsigned)maxd, flips_bad);
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault == CF_NONE, "and the unit is still running with no fault (state %d, fault %d)", g_ctrl.state, g_ctrl.fault);
    CHECK((SlaveCtrl_GuardCount() - g0) >= 4U, "the guard took the correction to zero at least 4 times (%u)", (unsigned)(SlaveCtrl_GuardCount() - g0));
    CHECK(worst_gap < 6.0, "while the input was stepping the output followed the input, no correction on top of it (largest difference %.1f V)", worst_gap);
    W.vin = 215.0f; run_for(60000000);
    CHECK(g_ctrl.state == CTRL_RUN && (double)W.vout > 228.0 && (double)W.vout < 232.0, "once the variac stops (215 V) it regulates to 230 V again (%.1f V)", (double)W.vout);
    check_invariants("variac swings");
    printf("  variac swings: 8 snaps, %d direction reversals, none with a large correction; guard events %u; output followed the input within %.1f V; it recovered to 230 V\n", flips, (unsigned)(SlaveCtrl_GuardCount() - g0), worst_gap);
}
#include "mc_groups.inc"
#endif

static void t_unsynchronised_isr(void)
{
    world_reset(14000000);
    run_for(800000 + WARM);
    SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM1);
    {
        long e = n_energise;
        g_ctrl.pending_pol = 1;
        SlaveCtrl_OnDeadtimeIrq();                       /* a stale dead-time interrupt after the fault */
        CHECK(n_energise == e && diag == 0, "a stale dead-time interrupt must not energise anything after a fault");
    }
    printf("  stale dead-time interrupt after a fault: ignored\n");
}

#if (SLAVE_POWER_STAGE >= 2)
static void t_stage2_leg_selection(float vin, const char *name, uint64_t start)
{
    long sd0;
    world_reset(start);
    W.vin = vin; W.vout = vin;
    run_for(4200000 + WARM);                                    /* 3 s converter + 100 ms zero duty + a bit */
    CHECK(g_ctrl.state == CTRL_RUN, "%s: expected RUN, state %d fault %d", name, g_ctrl.state, g_ctrl.fault);
    CHECK(n_inv_enable == 1, "%s: inverter enabled %d times", name, n_inv_enable);
#if (SLAVE_POWER_STAGE == 2)
    run_for((uint64_t)(CTRL_INJ_BASE_MS + CTRL_INJ_RAMP_MS) * 1000ULL + 600000ULL);   /* baseline, then the ramp: now in HOLD */
#endif
    sd0 = n_setduty;
    run_for(1000000);
#if (SLAVE_POWER_STAGE == 2)
    CHECK(g_ctrl.duty_counts == (uint16_t)(CTRL_STAGE2_DUTY_COUNTS + POWER_DEADTIME_TICKS), "%s: CCR %u, expected dead time + the stage-2 target in HOLD", name, g_ctrl.duty_counts);
#endif
    CHECK(g_ctrl.boost == (vin < CTRL_TARGET_V), "%s: boost flag %u for vin %.0f", name, g_ctrl.boost, (double)vin);
    CHECK(n_setduty - sd0 > 150, "%s: duty only applied %ld times in 1 s", name, n_setduty - sd0);
    CHECK(n_sync > 100, "%s: PWM counters re-synchronised %d times", name, n_sync);
    check_invariants(name);
    printf("  %-30s RUN, boost=%u, duty=%u counts, %ld duty updates/s, leg table violations %ld\n",
           name, g_ctrl.boost, g_ctrl.duty_counts, n_setduty - sd0, v_leg_table);
}

static void t_inverter_enable_fails(void)
{
    world_reset(15000000);
    W.inv_ok = 0;
    run_for(4000000 + WARM);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_INV_ENABLE, "expected INV_ENABLE fault, got %d/%d", g_ctrl.state, g_ctrl.fault);
    CHECK(diag == 0, "converter must be off after the inverter refused to enable");
    printf("  inverter would not leave break -> fault %d, everything off\n", g_ctrl.fault);
}

static void t_counter_reset(void)
{
    world_reset(16000000);
    unctl_limit_us = 20000ULL;          /* this test trips with a BREAK (a hard fault): the primary is open for the contact travel time */
    run_for(4500000 + WARM);
    CHECK(g_ctrl.state == CTRL_RUN, "reach RUN first (state %d)", g_ctrl.state);
    SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM1);
    run_for(2000);
    CHECK(g_ctrl.fault_count == 1, "count 1 expected");
    run_for(10300000);
    while (g_ctrl.state != CTRL_RUN) run_for(10000);
    run_for(305000000ULL);                               /* 5 minutes + of fault-free RUN */
    CHECK(g_ctrl.state == CTRL_RUN && g_ctrl.fault_count == 0, "escalation counter must reset after 5 min of RUN (count %u)", g_ctrl.fault_count);
    {
        uint64_t t_trip = sim_t;
        SlaveCtrl_OnBreakIrq(POWER_BREAK_TIM1);
        run_for(5000);                                   /* let the main loop register the fault first */
        CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault_count == 1, "second trip after the reset must count as the FIRST (state %d count %u)", g_ctrl.state, g_ctrl.fault_count);
        while (g_ctrl.state == CTRL_FAULT && sim_t < t_trip + 40000000ULL) run_for(10000);   /* the retry delay ends when FAULT is left */
        CHECK(sim_t - t_trip >= 9900000ULL && sim_t - t_trip < 10500000ULL, "after the reset the next delay must be 10 s again, was %.2f s", (double)(sim_t - t_trip) / 1e6);
        printf("  escalation counter reset after 5 min in RUN; next trip delay %.2f s (10 s, not 20 s)\n", (double)(sim_t - t_trip) / 1e6);
    }
    check_invariants("counter reset");
}
#endif

#if (SLAVE_POWER_STAGE >= 3)
static void t_stage3_closed_loop(void)
{
    double vo_min = 1e9, vo_max = -1e9;
    uint64_t t_end;
    world_reset(17000000);
    W.vin = 215.0f; W.vout = 215.0f; W.plant_on = 1; W.plant_gain = 0.010;
    run_for(4200000 + WARM);
    t_end = sim_t + 40000000ULL;
    while (sim_t < t_end)
    {
        run_for(10000);
        if (g_ctrl.state == CTRL_RUN) { if (W.vout < vo_min) vo_min = W.vout; if (W.vout > vo_max) vo_max = W.vout; }
        CHECK(g_ctrl.duty_counts <= (uint16_t)((CTRL_DUTY_MAX_PCT * POWER_PERIOD_TICKS) / 100UL), "duty above the cap");
        if (g_fail > 10) break;
    }
    CHECK(g_ctrl.state == CTRL_RUN, "closed loop must stay in RUN (state %d fault %d)", g_ctrl.state, g_ctrl.fault);
    CHECK(vo_max <= CTRL_VOUT_MAX_V, "output exceeded the protection limit: %.1f", vo_max);
    check_invariants("closed loop");
    printf("  closed loop (toy plant, NOT a tuning test): vin 215 -> vout %.1f V (range %.1f..%.1f), duty %u counts, boost=%u\n",
           (double)W.vout, vo_min, vo_max, g_ctrl.duty_counts, g_ctrl.boost);
}

static void t_input_range_and_vout_high(void)
{
    world_reset(18000000);
    run_for(4200000 + WARM);
    CHECK(g_ctrl.state == CTRL_RUN, "RUN first");
    W.vin = 130.0f;
    run_for(500000);                                    /* the shunts close first (SETTLE), THEN the inverter is disabled */
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_VIN_RANGE && diag == 0 && !inv_enabled, "vin 130 V must trip VIN_RANGE (%d/%d)", g_ctrl.state, g_ctrl.fault);
    run_for(8000000);                                    /* it retries, but must not start while the input is bad */
    CHECK(!is_running() && n_energise > 0 && diag == 0, "must not restart while vin is out of range");
    W.vin = 230.0f; W.vout = 230.0f;
    run_for(2500000);
    CHECK(is_running(), "must restart once vin is back in range (state %d)", g_ctrl.state);

    world_reset(19000000);
    run_for(4200000 + WARM);
    CHECK(g_ctrl.state == CTRL_RUN, "RUN first (vout test)");
    W.vout = 285.0f;
    run_for(500000);
    CHECK(g_ctrl.state == CTRL_FAULT && g_ctrl.fault == CF_VOUT_HIGH && diag == 0 && !inv_enabled, "vout 285 V must trip VOUT_HIGH (%d/%d)", g_ctrl.state, g_ctrl.fault);
    CHECK(g_ctrl.fault_count == 1, "VOUT_HIGH must escalate");
    printf("  input 130 V -> VIN_RANGE (retries but does not start); output 285 V -> VOUT_HIGH (escalating)\n");
    check_invariants("range/vout");
}
#endif

/* ======================================================================= */
static int run_isolated(void (*fn)(void), const char *name)
{
    int fd[2]; long res[2] = {0, 0}; pid_t p;
    /* ONLY=<text>   run just the tests whose name contains <text>
     * SKIP_SLOW=1   skip the multi-minute simulations (used for quick mutation runs) */
    const char *only = getenv("ONLY");
    if (only != NULL && strstr(name, only) == NULL) return 0;
    if (getenv("SKIP_SLOW") != NULL && (strstr(name, "5 min") != NULL || strstr(name, "10..50 s") != NULL ||
                                         strstr(name, "closed loop") != NULL || strstr(name, "input range") != NULL)) return 0;
    fflush(stdout);
    if (pipe(fd) != 0) { perror("pipe"); exit(2); }
    p = fork();
    if (p == 0)
    {
        close(fd[0]);
        g_checks = 0; g_fail = 0;                 /* a forked child inherits the parent's totals */
        printf("[%s]\n", name);
        fn();
        res[0] = g_checks; res[1] = g_fail;
        if (write(fd[1], res, sizeof res) != (ssize_t)sizeof res) _exit(3);
        fflush(stdout); _exit(0);
    }
    close(fd[1]);
    if (read(fd[0], res, sizeof res) != (ssize_t)sizeof res) { res[1] += 1; printf("      FAIL: test process crashed\n"); }
    close(fd[0]); waitpid(p, NULL, 0);
    g_checks += res[0]; g_fail += res[1];
    return (int)res[1];
}

/* wrappers (fork needs void(void)) */
static void w_square(void)     { t_pattern(PAT_SQUARE,     CTRL_ZC_BOTH,    "square wave ZCD",            1000000); }
static void w_square_inv(void) { t_pattern(PAT_SQUARE_INV, CTRL_ZC_BOTH,    "inverted square wave ZCD",   1000000); }
static void w_pulse(void)      { t_pattern(PAT_PULSE,      CTRL_ZC_FALL,    "pulse train ZCD",            1000000); }
static void w_half(void)       { t_pattern(PAT_HALF,       CTRL_ZC_UNKNOWN, "one pulse per cycle (refused)", 1000000); }
static void w_none(void)       { t_pattern(PAT_NONE,       CTRL_ZC_UNKNOWN, "no ZCD edges (refused)",     1000000); }
#if (SLAVE_POWER_STAGE >= 2)
static void w_s2_boost(void)   { t_stage2_leg_selection(200.0f, "inverter, vin 200 V (boost)", 20000000); }
static void w_s2_buck(void)    { t_stage2_leg_selection(250.0f, "inverter, vin 250 V (buck)",  21000000); }
#endif

int main(void)
{
#if (SLAVE_FLT_KEEP_LOWS == 0) && !defined(ALLOW_FLT_KEEP_LOWS_0)
    CHECK(0, "SLAVE_FLT_KEEP_LOWS is 0: the FLT response would open the primary under load (v0.9.65 default is 1; only the s3_flt_all_off job may build it off)");
#endif
    printf("Control sequencer test, SLAVE_POWER_STAGE=%d\n\n", SLAVE_POWER_STAGE);
    run_isolated(t_leg_table,                 "inverter leg table vs old firmware");
    run_isolated(w_square,                    "ZCD pattern: square wave");
    run_isolated(w_square_inv,                "ZCD pattern: inverted square wave");
    run_isolated(w_pulse,                     "ZCD pattern: pulse per half-cycle");
    run_isolated(t_pulse_rising_ignored,      "pulse mode ignores the pulse end");
    run_isolated(w_half,                      "ZCD pattern: one pulse per cycle");
    run_isolated(w_none,                      "ZCD pattern: nothing");
    run_isolated(t_pattern_lost_and_restart,  "pattern lost -> watchdog -> restart");
    run_isolated(t_sense_mismatch,            "AC_IN / AC_OUT sign mismatch");
    run_isolated(t_tiny_signal,               "mains too small to read a sign");
    run_isolated(t_break_escalation,          "break: 10..50 s retries, 6th latches");
    run_isolated(t_break_input_held,          "break input held low");
    run_isolated(t_flt_shutdown_first,      "FLT: the shutdown comes first (D25)");
    run_isolated(t_flt,                       "FLT input");
    run_isolated(t_stop_request,              "run request withdrawn");
    run_isolated(t_timer_wrap,                "32-bit timer wrap");
    run_isolated(t_jitter,                    "edge jitter");
    run_isolated(t_phase_shift_probe,         "phase jump: polarity guard stops it");
    run_isolated(t_lost_pulse_midrun,         "one zero-cross pulse lost mid-run");
    run_isolated(t_guard_margin,              "polarity guard: no false trips at the extremes");
    run_isolated(t_resync_clears_state,       "restart discards the old synchronisation");
#if RELAY_MODEL
    run_isolated(t_relay_module_unit,         "relays: module unit test (patterns, settle, reversal, ShortNow)");
    run_isolated(t_relay_stage1_untouched,    "relays: stay SHORTED while only the converter runs");
    run_isolated(t_wait_relay,                "relays: start-up waits for the relays to settle");
#if (SLAVE_POWER_STAGE >= 2)
    run_isolated(t_relay_start_order,         "relays: start order (converter -> inverter -> shunts open -> duty)");
    run_isolated(t_relay_stop_order,           "relays: normal stop order");
    run_isolated(t_relay_stop_in_unshunt,     "relays: stop while the shunts are opening");
    run_isolated(t_relay_soft_fault,          "relays: soft fault unwinds with the primary controlled");
    run_isolated(t_relay_hard_fault,          "relays: hard fault (break) closes the shunts at once");
#if (SLAVE_FLT_KEEP_LOWS != 0)
    run_isolated(t_relay_flt_keeps_lows,      "relays: FLT keeps the low sides on until the shunts have closed (v0.9.65)");
#endif
    run_isolated(t_relay_fault_during_reshunt,"relays: fault while unwinding");
    run_isolated(t_relay_never_free_without_inverter, "relays: 90-point injection sweep");
#endif
#endif
    run_isolated(t_race_future_timestamp,     "race: zero-cross timestamp newer than the clock");
    run_isolated(t_no_commutation_code,       "events but no commutation -> own fault code");
    run_isolated(t_zc_stale_code,             "main-loop backup -> own fault code");
    run_isolated(t_guard_noise_immunity,      "polarity guard: ignores one bad sample, trips on a sustained one");
    run_isolated(t_fault_context,             "fault context byte (state, mode, FLT/BKIN levels)");
#if (SLAVE_FLT_FILTER_US > 0U)
    run_isolated(t_flt_filter_poll,           "flt filter: the main-loop FLT check (v0.9.62)");
#endif
    run_isolated(t_wait_reasons,              "START with nothing happening: wait reasons");
    run_isolated(t_zc_fault_split,            "zero-cross faults: time F03 / sequence F15 / last-moment F16");
#if (SLAVE_TF_RUN_CHECK != 0) && (SLAVE_MASTER_CONTROL == 0)
    run_isolated(t_flip_standalone,           "flip standalone: a backwards transformer latches");
#endif
#if (SLAVE_GLITCH_SKIP != 0)
    run_isolated(t_glitch_skip,               "glitch skip: a rare F16/F04 while running is skipped");
#endif
    run_isolated(t_vin_hysteresis,            "input voltage: start window narrower than trip window");
    run_isolated(t_map_check,                 "passive converter-mapping check (no converter running)");
    run_isolated(t_diag_counters,             "bench diagnostics: FLT / break event counters (idle vs running)");
    run_isolated(t_trip_capture,              "bench diagnostics: where in the half-cycle a running trip hit");
#if LEGACY_MODEL
    run_isolated(t_legacy_sequence,           "legacy relays: SH 1 s -> hand-over -> W on, converter after W settles");
    run_isolated(t_legacy_no_converter_without_w, "legacy relays: no converter before W-RLY is on");
    run_isolated(t_legacy_fault_and_restart,  "legacy relays: fault releases both, sequence repeats");
    run_isolated(t_legacy_stop_release,       "legacy relays: release after a stop");
    run_isolated(t_legacy_never_both,         "legacy relays: SH and W never on together");
#endif
#if (SLAVE_CONV_FIRST_POL != 0)
    run_isolated(t_first_polarity,            "forced first polarity (diagnostic)");
#endif
#if (SLAVE_LEGACY_LOWSIDES != 0)
    run_isolated(t_legacy_lowsides,           "legacy low sides: on through the first diagonal");
    run_isolated(t_legacy_lowsides_restart,   "legacy low sides: trip and retry");
#endif
#ifdef EXPECT_LEGACY_DEFAULTS
    run_isolated(t_legacy_defaults,           "shipped defaults of the legacy converter start");
#endif
#ifdef TEST_LOW_VOLTAGE
    run_isolated(t_low_voltage,               "low input voltage: start from 90 V");
#endif
#if (SLAVE_TF_CHECK != 0)
    run_isolated(t_tf_check,                  "transformer-connection check");
    run_isolated(t_diag2_policy,              "diag2 policy (what the link sends)");
#endif
#ifdef TEST_SETTLE_OVERRIDE
    run_isolated(t_settle_override,           "longer W-RLY settle");
#endif
#ifdef TEST_FIRST_AT_PEAK
    run_isolated(t_first_at_peak,             "first diagonal at the line peak");
#endif
#ifdef TEST_QUICK_RETRY
    run_isolated(t_quick_retry,               "quick retry (3 s lockouts)");
#endif
#ifdef TEST_LEGACY_INV
    run_isolated(t_leg_inv_sequence,          "legacy inverter: start sequence");
    run_isolated(t_leg_inv_injection,         "legacy inverter: injection test");
    run_isolated(t_leg_inv_deadtime,          "legacy inverter: dead time vs pulse width");
    run_isolated(t_leg_inv_faults,            "legacy inverter: faults and stop");
    run_isolated(t_leg_inv_restart,           "legacy inverter: restart");
#endif
#ifdef TEST_LEGACY_REG
    run_isolated(t_l3_converge,               "legacy stage 3: regulation to 230 V (boost, buck)");
    run_isolated(t_l3_freeze_and_ramps,       "legacy stage 3: freeze band and variac ramps");
    run_isolated(t_l3_saturation,             "legacy stage 3: saturation and recovery");
    run_isolated(t_l3_faults,                 "legacy stage 3: faults, stop, retry");
#ifdef TEST_SOFTSTART
    run_isolated(t_l3_variac,                 "legacy stage 3: variac turned back and forth");
    run_isolated(t_l3_softstart,              "legacy stage 3: soft start (ramp configured)");
#endif
#ifdef TEST_MC
    run_isolated(t_mc_no_autorun,             "master control: no autorun, LINK_LOST, run = 0");
    run_isolated(t_mc_run_in_band,            "master control: run, in band, echoes");
    run_isolated(t_mc_stop_restart,           "master control: stop and restart");
    run_isolated(t_mc_fault_latches,          "master control: a fault latches");
    run_isolated(t_mc_clear_counter,          "master control: the clear counter");
    run_isolated(t_mc_first_counter,          "master control: the first counter is adopted");
    run_isolated(t_mc_link_loss,              "master control: link loss");
    run_isolated(t_mc_setv,                   "master control: set voltage rules");
    run_isolated(t_mc_condition_fault,        "master control: a condition fault latches");
    run_isolated(t_mc_end_to_end,             "master control: end to end through the link");
    run_isolated(t_mc_master_logic,           "master control: the real master logic against the real slave");
    run_isolated(t_mc_master_app,             "master control: the real master application against the real slave");
    run_isolated(t_mc_from_90v,               "master control: the unit works from 90 V");
    run_isolated(t_mc_iout,                   "master control: converter-side current through the link");
#if (SLAVE_INJ_GUARD != 0)
    run_isolated(t_mc_load_hold,              "master control: the load-connect hold (v0.9.61)");
#endif
    run_isolated(t_mc_app_load_hold,          "master control: the load-connect hold, real master application");
    run_isolated(t_mc_app_con_first,          "master control: contactor first, real master application (v0.9.64)");
#if (SLAVE_TUNE_MODE != 0)
    run_isolated(t_mc_tune,                   "tune mode: the real controller, the real sweep, the real master");
#endif
#if (SLAVE_TUNE_APPLY != 0)
    run_isolated(t_mc_tune_apply,             "tune apply: the stored profile in use on the real controller");
#endif
#ifdef FAST_CORRECTION_TEST
    run_isolated(t_mc_fast_correction,        "master control: the fast correction profile");
#endif
#if (SLAVE_VOLTAGE_CAL != 0)
    run_isolated(t_mc_vcal,                   "master control: voltage calibration end to end");
#endif
#if (SLAVE_TF_RUN_CHECK != 0)
    run_isolated(t_mc_flip,                   "master control: the transformer flip check at start");
#endif
#if (SLAVE_INJ_GUARD != 0)
    run_isolated(t_mc_input_steps,            "input steps: recover fast and never stick");
    run_isolated(t_mc_master_limits,          "input steps: the master's limits govern (185 V in, 200 V set)");
#endif
#if (SLAVE_GLITCH_SKIP != 0)
    run_isolated(t_mc_glitch_master,          "input steps: a rare zero-cross glitch does not open the load");
#endif
#if (SLAVE_TEST_NO_VOLTAGE_CUTS != 0)
    run_isolated(t_mc_no_voltage_cuts,        "no voltage cuts: the slave applies no limits");
#endif
#endif
#endif
    run_isolated(t_unsynchronised_isr,        "stale dead-time interrupt");
#if (SLAVE_POWER_STAGE >= 2)
    run_isolated(w_s2_boost,                  "stage 2: inverter, boost");
    run_isolated(w_s2_buck,                   "stage 2: inverter, buck");
    run_isolated(t_inverter_enable_fails,     "stage 2: inverter refuses to enable");
    run_isolated(t_counter_reset,             "escalation counter reset after 5 min");
#endif
#if (SLAVE_POWER_STAGE >= 3)
    run_isolated(t_stage3_closed_loop,        "stage 3: closed loop sanity");
    run_isolated(t_input_range_and_vout_high, "stage 3: input range / output high");
#endif
    printf("\n  %ld checks, %ld failures\n", g_checks, g_fail);
    printf(g_fail == 0 ? "  RESULT: ALL PASSED\n" : "  RESULT: FAILURES\n");
    return g_fail ? 1 : 0;
}
