/* Design study: does the controller need its P and D terms?
 * The ORIGINAL old compute function (verbatim, plus two test-only scale factors on Kp and Kd) is run
 * against a plant with the same STRUCTURE as the real one (static gain x duty, duty updated once per
 * half-cycle, RMS measured over 20 ms and refreshed every 10 ms, measurement noise), for a range of
 * plant gains, because the real gain is not known.
 * Variants: A = old PID | B = PI (D removed) | C = I only (P and D removed). */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <unistd.h>
#include <sys/wait.h>

static uint32_t g_tick;  static uint32_t HAL_GetTick(void) { return g_tick; }
#define __disable_irq() ((void)0)
#define __enable_irq()  ((void)0)
#define PWM_PERIOD 3199
#define FREEZE_DEBOUNCE_MS 300U
#define SOFTSTART_SLOW_TIME_MS 2000U
#define SOFTSTART_SLOW_SLOPE_VPS 3.0f
#define SOFTSTART_FAST_SLOPE_VPS 10.0f
#define SOFTSTART_FINAL_SLOPE_VPS 5.0f
#define SOFTSTART_FINAL_BAND_V 3.0f
#define SOFTSTART_MAX_DT_S 0.050f
static volatile float vrms_in, vrms_out, voltage_setpoint = 230.0f, voltage_setpoint_pid, pwm_output;
static volatile uint8_t pid_freeze_active;
static float ratio_buck = (1.8f)/5.78f, ratio_boost = (0.9f)/5.78f;
static uint8_t gbIsSystemNormalState, Mreceivedflag_pulse_on, pwm_allowed;
static double g_kp_scale = 1.0, g_kd_scale = 1.0;
#include "old_snippets/autopid_struct.inc"
static AutoPID pid;
#include "old_snippets/autopid_init.inc"
#include "old_snippets/autopid_compute_scaled.inc"
#include "old_snippets/ramp.inc"
static void old_block(void) {
#include "old_snippets/mainloop_block.inc"
}

static uint32_t rng = 99; static double rn(double a) { rng = rng*1664525u+1013904223u; return a*(((double)(rng>>8)/8388608.0)-1.0); }

#define N 24000
static double verr[N], vduty[N];

static void run(double kp, double kd, double g, int variant_noise)
{
    int i; double vin = 215.0; double win[20]; int wi = 0; double meas_out = 215.0, meas_in = 215.0;
    double duty_applied = 0.0; double sag = 0.0;
    memset(win, 0, sizeof win); for (i = 0; i < 20; i++) win[i] = 215.0;
    g_kp_scale = kp; g_kd_scale = kd; g_tick = 5000; rng = 99u + variant_noise;
    vrms_in = 215.0f; vrms_out = 215.0f; voltage_setpoint = 230.0f; voltage_setpoint_pid = vrms_in;
    AutoPID_Init(&pid, &vrms_out, &voltage_setpoint_pid, &pwm_output, 0.18f, 4.0f, 0.00f, 0.0f, (float)PWM_PERIOD, 5);
    pwm_output = 0.0f; gbIsSystemNormalState = Mreceivedflag_pulse_on = pwm_allowed = 1;

    for (i = 0; i < N; i++)                        /* 1 ms steps, 24 s */
    {
        double t = i * 0.001, vout_true;
        g_tick++;
        sag = (t >= 9.0 && t < 15.0) ? -6.0 : ((t >= 15.0) ? 0.0 : 0.0);   /* mains sag of 6 V from 9 s to 15 s */
        if (t >= 18.0) sag = +6.0;                                          /* and a 6 V rise from 18 s */
        if ((i % 10) == 0) duty_applied = pwm_output;                       /* duty is updated once per half-cycle */
        vout_true = (vin + sag) + g * duty_applied;                         /* boost: injection adds */
        win[wi] = vout_true; wi = (wi + 1) % 20;
        if ((i % 10) == 0)                                                  /* RMS refreshed every 10 ms over 20 ms */
        {
            double a = 0; int k; for (k = 0; k < 20; k++) a += win[k];
            meas_out = a / 20.0 + rn(0.15); meas_in = (vin + sag) + rn(0.15);
        }
        vrms_in = (float)meas_in; vrms_out = (float)meas_out;
        old_block(); AutoPID_Compute(&pid);
        verr[i] = vout_true - 230.0; vduty[i] = pwm_output;
    }
}

static void report(const char *name, double g)
{
    int i; double rms_ss = 0, mean_d = 0, sd = 0, peak_sag = 0, rec_sag = -1, peak_rise = 0, maxabs_end = 0; int n = 0;
    for (i = 6000; i < 8000; i++) { rms_ss += verr[i]*verr[i]; mean_d += vduty[i]; n++; }
    rms_ss = sqrt(rms_ss / n); mean_d /= n;
    for (i = 6000; i < 8000; i++) sd += (vduty[i]-mean_d)*(vduty[i]-mean_d);
    sd = sqrt(sd / n);
    for (i = 9000; i < 14000; i++) { if (fabs(verr[i]) > peak_sag) peak_sag = fabs(verr[i]); }
    for (i = 9000; i < 14000; i++) if (fabs(verr[i]) < 1.0) { int j, ok = 1; for (j = i; j < i + 300 && j < N; j++) if (fabs(verr[j]) >= 1.0) { ok = 0; break; } if (ok) { rec_sag = (i - 9000) * 0.001; break; } }
    for (i = 18000; i < 23000; i++) { if (fabs(verr[i]) > peak_rise) peak_rise = fabs(verr[i]); }
    for (i = 22000; i < N; i++) if (fabs(verr[i]) > maxabs_end) maxabs_end = fabs(verr[i]);
    printf("  %-22s g=%5.3f V/count | steady err %.2f V rms, duty jitter %5.1f counts | sag: peak %4.1f V, back within 1 V in %s | rise: peak %4.1f V%s\n",
           name, g, rms_ss, sd, peak_sag, rec_sag < 0 ? "  never" : (char[16]){0}, peak_rise, maxabs_end > 15 ? "  UNSTABLE" : "");
    if (rec_sag >= 0) { /* reprint with the number (compound literal trick above cannot format) */ }
}
int main(void)
{
    double gs[6] = { 0.004, 0.008, 0.015, 0.03, 0.06, 0.12 };
    struct { const char *n; double kp, kd; } v[3] = { { "A old PID", 1.0, 1.0 }, { "B PI (no D)", 1.0, 0.0 }, { "C I only", 0.0, 0.0 } };
    int a, b;
    printf("Plant: injection = g x duty, one update per half-cycle; RMS over 20 ms refreshed every 10 ms, +/-0.15 V noise.\n");
    printf("Start: Vin 215 V, target 230 V (soft-start ramp). Mains sag -6 V at 9 s .. 15 s, +6 V from 18 s.\n\n");
    for (b = 0; b < 6; b++)
    {
        for (a = 0; a < 3; a++)
        {
            int fd[2]; pid_t p; double out[8];
            if (pipe(fd)) return 2;
            fflush(stdout);
            p = fork();
            if (p == 0)
            {
                int i; double rms_ss = 0, mean_d = 0, sd = 0, pk_sag = 0, rec = -1, pk_rise = 0, mx_end = 0; int n = 0;
                run(v[a].kp, v[a].kd, gs[b], a);
                for (i = 6000; i < 8000; i++) { rms_ss += verr[i]*verr[i]; mean_d += vduty[i]; n++; }
                rms_ss = sqrt(rms_ss/n); mean_d /= n;
                for (i = 6000; i < 8000; i++) sd += (vduty[i]-mean_d)*(vduty[i]-mean_d);
                sd = sqrt(sd/n);
                for (i = 9000; i < 14000; i++) if (fabs(verr[i]) > pk_sag) pk_sag = fabs(verr[i]);
                for (i = 9000; i < 14000; i++) if (fabs(verr[i]) < 1.0) { int j, ok = 1; for (j = i; j < i+300 && j < N; j++) if (fabs(verr[j]) >= 1.0) { ok = 0; break; } if (ok) { rec = (i-9000)*0.001; break; } }
                for (i = 18000; i < 23000; i++) if (fabs(verr[i]) > pk_rise) pk_rise = fabs(verr[i]);
                for (i = 22000; i < N; i++) if (fabs(verr[i]) > mx_end) mx_end = fabs(verr[i]);
                out[0]=rms_ss; out[1]=sd; out[2]=pk_sag; out[3]=rec; out[4]=pk_rise; out[5]=mx_end;
                if (write(fd[1], out, sizeof out) < 0) _exit(3); _exit(0);
            }
            close(fd[1]); if (read(fd[0], out, sizeof out) != (ssize_t)sizeof out) memset(out, 0, sizeof out); close(fd[0]); waitpid(p, NULL, 0);
            printf("  g=%5.3f  %-12s | steady error %.2f V rms | duty jitter %5.1f counts | -6 V sag: peak %4.1f V, recovered ", gs[b], v[a].n, out[0], out[1], out[2]);
            if (out[3] < 0) printf("never"); else printf("%.2f s", out[3]);
            printf(" | +6 V rise: peak %4.1f V%s\n", out[4], out[5] > 15.0 ? "  ** UNSTABLE **" : "");
        }
        printf("\n");
    }
    (void)report; return 0;
}
