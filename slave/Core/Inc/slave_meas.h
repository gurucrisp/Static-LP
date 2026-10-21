/**
  ******************************************************************************
  * @file    slave_meas.h
  * @brief   AC voltage measurement for the slave: timer-triggered ADC +
  *          circular DMA, RMS over exactly one mains period.
  *
  * HOW IT WORKS
  *   - TIM3 triggers an ADC1 scan every 50 us (20 kHz) over three channels:
  *     AC_IN (PA0), AC_OUT (PA1), NTC (PA5). DMA writes the frames into a
  *     circular buffer; no interrupts are used.
  *   - Every 10 ms SlaveMeas_Service() takes the most recent N frames,
  *     N = one mains period (from the zero-cross rate measured by
  *     slave_zc.c; 50 Hz is assumed until a plausible rate is seen), removes
  *     the DC bias and computes the RMS. The RMS over exactly one period of a
  *     periodic signal is exact whatever the phase, so the window does not
  *     need to start at a zero-crossing.
  *   - ADC counts -> volts uses g_meas_cal[], ONE ENTRY PER CHANNEL (AC_IN and
  *     AC_OUT go through separate op-amps with separate resistors, so each
  *     needs its own calibration). They start at the NOMINAL values worked out
  *     from the power-board schematic, corrected by the field-calibration
  *     constants MEAS_CAL_* below. The two-point calibration done from the
  *     master keypad (stored in the slave's own flash) replaces these later.
  *
  * SENSING CIRCUIT (from the power-board schematic)
  *   Each of AC_IN / AC_OUT is a differential op-amp stage (MCP6004): phase and
  *   neutral each through 470 kohm, feedback 3k || 3k = 1.5 kohm, output biased
  *   to 3.3 V/2 = 1.65 V by a 3k/3k divider:
  *        Vpin = 1.65 V + 0.003191 * (Vphase - Vneutral)
  *   300 V RMS (424 V peak) gives 1.65 V +/- 1.35 V, i.e. about 0.3 V to
  *   3.0 V: it fits the 0-3.3 V ADC range, and the signal would clip near
  *   354 V RMS.
  ******************************************************************************
  */

#ifndef SLAVE_MEAS_H
#define SLAVE_MEAS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32g0xx_hal.h"

/* ---- Sampling layout ---------------------------------------------------- */
#define MEAS_CHANNELS          3U        /* AC_IN, AC_OUT, NTC (rank order)     */
#define MEAS_CH_IN             0U
#define MEAS_CH_OUT            1U
#define MEAS_CH_NTC            2U
#define MEAS_BUF_FRAMES        1024U     /* frames kept (51 ms at 20 kHz)       */
#define MEAS_SAMPLE_RATE_HZ    20000UL
#define MEAS_SAMPLE_PERIOD_US  50U       /* 1e6 / 20 kHz                        */
#define MEAS_WINDOW_MAX        800U      /* longest window allowed (frames)     */
#define MEAS_UPDATE_US         10000UL   /* recompute every 10 ms (= link period) */
#define MEAS_WARMUP_US         100000UL  /* ignore the first 100 ms after start */

/* ---- Nominal scaling from the schematic ----------------------------------
 * volts (mains-equivalent) per ADC count of AC-RMS:
 *      (Vref / 4095) / (R_fb / R_in)                                         */
#define MEAS_VREF_V            3.3f
#define MEAS_R_IN_OHM          470000.0f
#define MEAS_R_FB_OHM          1500.0f   /* 3k || 3k                           */
#define MEAS_NOMINAL_V_PER_COUNT \
        ((MEAS_VREF_V / 4095.0f) / (MEAS_R_FB_OHM / MEAS_R_IN_OHM))

/* ---- Field calibration (interim, compile-time) ---------------------------
 * Applied on top of the nominal scaling:   volts = A * nominal_volts + B
 * where nominal_volts is what the board shows now. Defaults A = 1, B = 0 mean
 * "not calibrated". To calibrate a channel, read the displayed value D at two
 * different voltages against a true-RMS meter reading T:
 *        A = (T2 - T1) / (D2 - D1)         B = T1 - A * D1
 * and put A and B here (separately for AC_IN and AC_OUT), then rebuild.      */
#define MEAS_CAL_IN_A    1.0f
#define MEAS_CAL_IN_B    0.0f
#define MEAS_CAL_OUT_A   1.0f
#define MEAS_CAL_OUT_B   0.0f

typedef struct
{
    float   gain_v_per_count;   /* volts per ADC count (AC-RMS)                 */
    float   offset_v;           /* volts added after the gain                   */
    uint8_t calibrated;         /* 0 = nominal values (A = 1, B = 0)            */
} MeasCal_t;

typedef struct
{
    /* Calibrated results (volts RMS, mains-equivalent) */
    volatile float    vrms_in;
    volatile float    vrms_out;

    /* Raw results, for calibration and diagnostics */
    volatile float    rms_in_counts;      /* AC-RMS in ADC counts               */
    volatile float    rms_out_counts;
    volatile float    mean_in_counts;     /* DC bias in counts (about 2048)     */
    volatile float    mean_out_counts;
    volatile uint16_t min_in, max_in;     /* extremes in the last window        */
    volatile uint16_t min_out, max_out;
    volatile uint8_t  clipped_in;         /* 1 if the ADC range was hit         */
    volatile uint8_t  clipped_out;

    /* Timing */
    volatile float    freq_hz;            /* mains frequency from ZC rate, 0 if unknown */
    volatile uint8_t  freq_valid;         /* 1 once a plausible ZC period was seen      */
    volatile uint32_t window_frames;      /* N used for the last result         */

    volatile uint32_t ntc_counts;         /* latest NTC reading                 */
    volatile uint32_t update_count;       /* results computed since start       */
    volatile uint8_t  valid;              /* 1 after the first full result      */
} SlaveMeas_t;

extern SlaveMeas_t g_meas;
extern MeasCal_t   g_meas_cal[2];       /* [MEAS_CH_IN], [MEAS_CH_OUT] */
extern ADC_HandleTypeDef hadc1;

/* Starts TIM3 (20 kHz trigger), calibrates the ADC and starts the circular
 * DMA. Call after SlaveZC_Init() (it uses the TIM2 microsecond timebase). */
void SlaveMeas_Init(void);

/* Call often from the main loop; it recomputes every 10 ms. */
void SlaveMeas_Service(void);

/* ---- Hardware-independent helpers (also used by the host-side tests) ---- */

/* Mean-removed RMS, mean, min and max of one channel over n frames of an
 * interleaved buffer, starting at frame index 'start' and wrapping at
 * 'frames'. */
void SlaveMeas_ComputeStats(const volatile uint16_t *buf, uint32_t frames, uint32_t start,
                            uint32_t n, uint32_t channels, uint32_t ch,
                            float *rms, float *mean, uint16_t *vmin, uint16_t *vmax);

/* Counts (AC-RMS) -> volts for one channel's calibration; never negative. */
float    SlaveMeas_ApplyCal(const MeasCal_t *cal, float rms_counts);

/* Window length in frames for a given mains period in microseconds. */
uint32_t SlaveMeas_WindowFrames(float period_us);

/* Mains period from the rising-edge period of the ZC signal, assuming either
 * one rising edge per mains cycle or one per half-cycle. Returns 0 if the
 * period is not plausible for 40-65 Hz mains. */
float    SlaveMeas_MainsPeriodFromZc(uint32_t zc_rise_period_us);


/* ---- Instantaneous polarity of the mains (used by the converter sequencer) ---
 * The sign of the mains just BEFORE a zero-cross tells which half-cycle is
 * ending, i.e. which converter diagonal is correct for the NEXT one. It is
 * taken from the ADC ring: the SUM of MEAS_SIGN_SPAN_FRAMES samples that ended
 * MEAS_SIGN_BACK_FRAMES frames before the newest complete frame (about
 * 0.2 ... 0.6 ms before the edge, where a 50/60 Hz sine is already well away
 * from zero), compared with the channel's DC bias from the last RMS window.
 * Positive = (phase - neutral) > 0 in the differential sense stage.
 *
 * INTEGER ARITHMETIC ONLY: it runs inside the zero-cross interrupt on a
 * Cortex-M0+ (no FPU), where every float operation costs 50-100 cycles. The
 * bias is converted to an integer sum ONCE per RMS window in the main loop. */
#define MEAS_SIGN_BACK_FRAMES   4U
#define MEAS_SIGN_SPAN_FRAMES   8U
#define MEAS_SIGN_MIN_COUNTS    20                                   /* ~5 V of mains */
#define MEAS_SIGN_MIN_SUM       (MEAS_SIGN_MIN_COUNTS * (int32_t)MEAS_SIGN_SPAN_FRAMES)

/* Pure helper (host-testable). mean_sum = round(bias_counts * span).
 * Returns +1 / -1, or 0 if |sum - mean_sum| <= min_abs_sum. dev_sum_out
 * (optional) receives the signed (sum - mean_sum); divide by span for counts. */
int8_t SlaveMeas_SignFromRing(const volatile uint16_t *buf, uint32_t frames, uint32_t frames_done,
                              uint32_t channels, uint32_t ch, int32_t mean_sum,
                              uint32_t back, uint32_t span, int32_t min_abs_sum,
                              int32_t *dev_sum_out);

/* ISR-SAFE wrapper on the live DMA ring and the bias of the last RMS window.
 * Returns 0 until the first RMS window exists. */
int8_t SlaveMeas_SignBeforeNow(uint32_t ch, int32_t *dev_sum_out);

/* Polarity GUARD: sign of the NEWEST two frames of a channel with a wider dead
 * band (MEAS_GUARD_MIN_COUNTS), run from the 20 kHz tick to catch a line that
 * reverses while a converter diagonal is on. ISR-SAFE, integer only. */
#define MEAS_GUARD_SPAN_FRAMES  2U
#define MEAS_GUARD_MIN_COUNTS   40            /* ~10 V of mains: the 20 kHz guard                   */
#define MEAS_GUARD_STRONG_COUNTS 120          /* ~30 V: "is this zero-cross event really early?"    */
int8_t SlaveMeas_SignLatest(uint32_t ch, int32_t min_counts);

/* Enables the TIM3 update interrupt (the 20 kHz ADC trigger tick) at priority 1
 * and routes it to SlaveCtrl_OnSampleTick() (see stm32g0xx_it.c). */
void   SlaveMeas_TickIrqEnable(void);

#ifdef __cplusplus
}
#endif

#endif /* SLAVE_MEAS_H */
