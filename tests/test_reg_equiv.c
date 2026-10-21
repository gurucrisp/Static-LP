/* Equivalence test: ORIGINAL old-firmware regulator, with its proportional and derivative
 * gains forced to ZERO, vs the new integral-only slave_reg.c.
 *
 * The files in old_snippets/ are cut verbatim out of the old main.c (see
 * make_snippets.sh). They are compiled here with minimal stubs, then driven
 * with exactly the same inputs as Reg_Service(); every step the outputs must
 * be IDENTICAL (same float bits), not merely close.
 *
 * Build: gcc -O0 -ffp-contract=off -Wall -I.. test_reg_equiv.c ../slave_reg.c -o test_reg_equiv -lm
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <unistd.h>
#include <sys/wait.h>
#include "slave_reg.h"

/* ------------------------------------------------------------------ stubs */
static uint32_t g_tick;
static uint32_t HAL_GetTick(void) { return g_tick; }
#define __disable_irq() ((void)0)
#define __enable_irq()  ((void)0)

#define PWM_PERIOD                   3199
#define FREEZE_DEBOUNCE_MS           300U
#define SOFTSTART_SLOW_TIME_MS       2000U
#define SOFTSTART_SLOW_SLOPE_VPS     3.0f
#define SOFTSTART_FAST_SLOPE_VPS     10.0f
#define SOFTSTART_FINAL_SLOPE_VPS    5.0f
#define SOFTSTART_FINAL_BAND_V       3.0f
#define SOFTSTART_MAX_DT_S           0.050f

static volatile float   vrms_in, vrms_out;
static volatile float   voltage_setpoint     = 230.0f;
static volatile float   voltage_setpoint_pid = 0.0f;
static volatile float   pwm_output           = 0.0f;
static volatile uint8_t pid_freeze_active    = 0U;
/* test-only: scale factors the (otherwise verbatim) old function multiplies Kp and Kd by.
 * Zero = the old controller with P and D removed, which is exactly what slave_reg.c implements. */
static double g_kp_scale = 0.0, g_kd_scale = 0.0;
static float ratio_buck  = (1.8f) / 5.78f;
static float ratio_boost = (0.9f) / 5.78f;
static uint8_t gbIsSystemNormalState, Mreceivedflag_pulse_on, pwm_allowed;

/* ------------------------------------------------- the ORIGINAL code, verbatim */
#include "old_snippets/autopid_struct.inc"
static AutoPID pid;
#include "old_snippets/autopid_init.inc"
#include "old_snippets/autopid_compute_scaled.inc"
#include "old_snippets/ramp.inc"

static void old_main_loop_block(void)
{
#include "old_snippets/mainloop_block.inc"
}

/* ------------------------------------------------------------------ harness */
static uint32_t rng = 1U;
static uint32_t rnd(void) { rng = rng * 1664525U + 1013904223U; return rng >> 8; }
static float    frnd(float lo, float hi) { return lo + (hi - lo) * ((float)(rnd() & 0xFFFFU) / 65535.0f); }

static long g_checks, g_fail;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fail++; if (g_fail < 12) { printf("  FAIL: " __VA_ARGS__); printf("\n"); } } } while (0)

static int same_bits(float a, float b) { return memcmp(&a, &b, sizeof a) == 0; }

/* One scenario. mode selects how vin moves / how arming toggles. */
static void scenario(unsigned seed, uint32_t start_tick, int steps, int mode)
{
    Reg_t r;
    int   i, armed = 0, arm_hold = 0;
    float vin, vout, plant_gain;
    int   steps_boost = 0, steps_buck = 0, steps_freeze = 0, arm_edges = 0;

    rng = seed; g_tick = start_tick;
    vin  = frnd(150.0f, 290.0f);
    vout = vin;

    /* ---- old init (as in the old main) ---- */
    vrms_in = vin; vrms_out = vout;
    voltage_setpoint     = 230.0f;
    voltage_setpoint_pid = vrms_in;
    AutoPID_Init(&pid, &vrms_out, &voltage_setpoint_pid, &pwm_output,
                 0.18f, 4.0f, 0.00f, 0.0f, (float)PWM_PERIOD, 5);
    pwm_output = 0.0f;
    gbIsSystemNormalState = Mreceivedflag_pulse_on = pwm_allowed = 0U;

    /* ---- new init ---- */
    Reg_Init(&r, 230.0f, vin, vout, g_tick);

    plant_gain = frnd(0.002f, 0.006f);          /* volts of correction per duty count */

    for (i = 0; i < steps; i++)
    {
        uint32_t dt_ms;
        uint32_t k = rnd() % 100U;
        float    corr;

        /* time: mostly 1..20 ms, sometimes a stall (tests the 50 ms dt clamp) */
        if      (k < 90U) dt_ms = 1U + (rnd() % 20U);
        else if (k < 98U) dt_ms = 20U + (rnd() % 80U);
        else              dt_ms = 200U + (rnd() % 400U);
        g_tick += dt_ms;

        /* input voltage movement */
        if      (mode == 0) vin += frnd(-0.6f, 0.6f);                       /* slow drift      */
        else if (mode == 1) { if ((rnd() % 400U) == 0U) vin = frnd(140.0f, 300.0f); else vin += frnd(-0.2f, 0.2f); }
        else if (mode == 2) vin = 230.0f + frnd(-1.8f, 1.8f);               /* inside / on the freeze band */
        else                vin = 230.0f + frnd(-30.0f, 30.0f);             /* buck <-> boost crossings    */
        if (vin < 100.0f) vin = 100.0f;
        if (vin > 320.0f) vin = 320.0f;

        /* arming: random holds, several arm/disarm edges */
        if (arm_hold <= 0)
        {
            int na = ((rnd() % 5U) != 0U);
            if (na != armed) arm_edges++;
            armed = na;
            arm_hold = 20 + (int)(rnd() % 400U);
        }
        arm_hold--;
        gbIsSystemNormalState = Mreceivedflag_pulse_on = pwm_allowed = (uint8_t)armed;

        /* crude plant so the loop actually moves: output = input +/- duty * gain */
        corr = (float)pwm_output * plant_gain;
        vout = (vin < voltage_setpoint_pid) ? (vin + corr) : (vin - corr);
        vout += frnd(-0.15f, 0.15f);
        if (!armed) vout = vin;

        vrms_in = vin; vrms_out = vout;

        /* OLD: soft-start block, then the PID (in the order the new service uses) */
        old_main_loop_block();
        AutoPID_Compute(&pid);

        /* NEW */
        Reg_Service(&r, g_tick, vin, vout, (uint8_t)armed);

        CHECK(same_bits((float)pwm_output, r.output),
              "seed %u mode %d step %d: duty old=%.9g new=%.9g", seed, mode, i, (double)pwm_output, (double)r.output);
        CHECK(same_bits((float)voltage_setpoint_pid, r.sp_v),
              "seed %u mode %d step %d: setpoint old=%.9g new=%.9g", seed, mode, i, (double)voltage_setpoint_pid, (double)r.sp_v);
        CHECK(same_bits(pid.integral, r.integral),
              "seed %u mode %d step %d: integral old=%.9g new=%.9g", seed, mode, i, (double)pid.integral, (double)r.integral);
        CHECK(pid_freeze_active == r.freeze_active,
              "seed %u mode %d step %d: freeze flag old=%d new=%d", seed, mode, i, pid_freeze_active, r.freeze_active);
        CHECK(Reg_IsBoost(&r, vin) == (uint8_t)(vin < voltage_setpoint_pid),
              "seed %u mode %d step %d: boost/buck decision", seed, mode, i);

        if (armed) { if (vin < r.sp_v) steps_boost++; else steps_buck++; if (r.freeze_active) steps_freeze++; }
    }
    printf("  seed %-4u mode %d start tick 0x%08X: %d steps, armed boost %d / buck %d, frozen %d, arm edges %d\n",
           seed, mode, (unsigned)start_tick, steps, steps_boost, steps_buck, steps_freeze, arm_edges);
}

/* The old functions keep static local variables (was_armed, was_in_freeze, the
 * debounce state ...). To start every scenario from the pristine state of a
 * real boot, each scenario runs in its own forked child; the parent never
 * executes the old code, so every child inherits untouched statics. */
static void run_isolated(unsigned seed, uint32_t start_tick, int steps, int mode, long *checks, long *fails)
{
    int   fd[2];
    long  res[2] = {0, 0};
    pid_t p;

    fflush(stdout);
    if (pipe(fd) != 0) { perror("pipe"); exit(2); }
    p = fork();
    if (p == 0)
    {
        close(fd[0]);
        scenario(seed, start_tick, steps, mode);
        res[0] = g_checks; res[1] = g_fail;
        if (write(fd[1], res, sizeof res) != (ssize_t)sizeof res) _exit(3);
        fflush(stdout);
        _exit(0);
    }
    close(fd[1]);
    if (read(fd[0], res, sizeof res) != (ssize_t)sizeof res) { res[1] += 1; }
    close(fd[0]);
    waitpid(p, NULL, 0);
    *checks += res[0];
    *fails  += res[1];
}

int main(void)
{
    unsigned s;
    int      m;
    long     checks = 0, fails = 0;

    printf("Regulator equivalence: ORIGINAL old code (P and D gains = 0) vs new slave_reg.c\n");
    printf("(each scenario runs in a fresh process = pristine static state, like a boot)\n\n");

    for (m = 0; m < 4; m++)
        for (s = 1; s <= 6; s++)
            run_isolated(s * 7919U + (unsigned)m, 1000U, 20000, m, &checks, &fails);

    printf("\n  with the millisecond tick starting just before its 32-bit wrap:\n");
    for (m = 0; m < 4; m++)
        run_isolated(4242U + (unsigned)m, 0xFFFFF000U, 20000, m, &checks, &fails);

    printf("\n  %ld comparisons, %ld mismatches\n", checks, fails);
    printf(fails == 0 ? "  RESULT: BIT-IDENTICAL to the original\n" : "  RESULT: DIFFERENCES FOUND\n");
    return fails ? 1 : 0;
}
