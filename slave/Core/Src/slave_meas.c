/**
  ******************************************************************************
  * @file    slave_meas.c
  * @brief   AC voltage measurement for the slave. See slave_meas.h.
  ******************************************************************************
  */

#include <math.h>
#include "main.h"
#include "slave_meas.h"
#include "slave_meas_due.h"
#if (SLAVE_IOUT != 0)
#include "slave_iout.h"
#endif
#include "slave_irq_prio.h"
#include "slave_zc.h"

ADC_HandleTypeDef hadc1;                 /* AC_IN, AC_OUT, NTC */
static DMA_HandleTypeDef hdma_adc1;
static TIM_HandleTypeDef s_htim3;        /* 20 kHz ADC trigger */

SlaveMeas_t g_meas;
MeasCal_t   g_meas_cal[2] =
{
    /* [MEAS_CH_IN]  */ { MEAS_NOMINAL_V_PER_COUNT * MEAS_CAL_IN_A,  MEAS_CAL_IN_B,
                          (uint8_t)(((MEAS_CAL_IN_A  != 1.0f) || (MEAS_CAL_IN_B  != 0.0f)) ? 1U : 0U) },
    /* [MEAS_CH_OUT] */ { MEAS_NOMINAL_V_PER_COUNT * MEAS_CAL_OUT_A, MEAS_CAL_OUT_B,
                          (uint8_t)(((MEAS_CAL_OUT_A != 1.0f) || (MEAS_CAL_OUT_B != 0.0f)) ? 1U : 0U) },
};

/* Written by DMA, read by the CPU: volatile. */
static volatile uint16_t s_buf[MEAS_BUF_FRAMES * MEAS_CHANNELS];

/* DC bias of AC_IN / AC_OUT as an integer sum over MEAS_SIGN_SPAN_FRAMES samples,
 * refreshed each RMS window; read by the zero-cross interrupt (integer math only). */
static volatile int32_t s_sign_mean_sum[2];

static MeasClock_t s_clk;          /* v0.9.71: the update period and the warm-up, wrap-safe (slave_meas_due.h) */
static float    s_period_filt_us;        /* low-pass filtered mains period */

/* ------------------------------------------------------------------------- */
/* Hardware-independent helpers                                              */
/* ------------------------------------------------------------------------- */

void SlaveMeas_ComputeStats(const volatile uint16_t *buf, uint32_t frames, uint32_t start,
                            uint32_t n, uint32_t channels, uint32_t ch,
                            float *rms, float *mean, uint16_t *vmin, uint16_t *vmax)
{
    uint64_t sum   = 0U;
    uint64_t sumsq = 0U;
    uint32_t lo    = 0xFFFFU;
    uint32_t hi    = 0U;
    uint32_t idx   = start;
    uint32_t k;

    if (n == 0U)
    {
        *rms = 0.0f; *mean = 0.0f; *vmin = 0U; *vmax = 0U;
        return;
    }

    for (k = 0U; k < n; k++)
    {
        uint32_t x = buf[(idx * channels) + ch];
        sum   += x;
        sumsq += (uint32_t)(x * x);          /* x <= 4095, so x*x fits in 32 bits */
        if (x < lo) lo = x;
        if (x > hi) hi = x;
        if (++idx >= frames) idx = 0U;
    }

    /* n^2 * variance = n * sum(x^2) - (sum x)^2, in exact integer arithmetic.
     * It cannot be negative (Cauchy-Schwarz). The DC bias cancels out. */
    {
        uint64_t num = ((uint64_t)n * sumsq) - (sum * sum);
        float var    = (float)num / ((float)n * (float)n);
        *rms  = sqrtf(var);
        *mean = (float)sum / (float)n;
    }
    *vmin = (uint16_t)lo;
    *vmax = (uint16_t)hi;
}

float SlaveMeas_ApplyCal(const MeasCal_t *cal, float rms_counts)
{
    float v = (cal->gain_v_per_count * rms_counts) + cal->offset_v;
    return (v > 0.0f) ? v : 0.0f;
}

uint32_t SlaveMeas_WindowFrames(float period_us)
{
    float    f = (period_us / (float)MEAS_SAMPLE_PERIOD_US) + 0.5f;
    uint32_t n = (f < 1.0f) ? 1U : (uint32_t)f;
    if (n > MEAS_WINDOW_MAX) n = MEAS_WINDOW_MAX;
    return n;
}

float SlaveMeas_MainsPeriodFromZc(uint32_t p)
{
    if ((p >= 15000U) && (p <= 27000U))   /* one rising edge per mains cycle (37-66 Hz; v0.9.76: was 25000 = exactly 40 Hz) */
    {
        return (float)p;
    }
    if ((p >= 7500U) && (p <= 13500U))    /* one rising edge per half-cycle (74-133 Hz)    */
    {
        return 2.0f * (float)p;
    }
    return 0.0f;
}

/* Index of the first frame of a window of n frames that ends just before
 * 'frames_done' (the number of complete frames written so far, 0..frames-1). */
static uint32_t WindowStart(uint32_t frames_done, uint32_t n)
{
    return (frames_done >= n) ? (frames_done - n)
                              : (frames_done + MEAS_BUF_FRAMES - n);
}

/* ------------------------------------------------------------------------- */
/* Hardware                                                                  */
/* ------------------------------------------------------------------------- */

/* HAL calls this from HAL_ADC_Init (state RESET): clocks + DMA for ADC1. */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        __HAL_RCC_DMA1_CLK_ENABLE();

        hdma_adc1.Instance                 = DMA1_Channel1;
        hdma_adc1.Init.Request             = DMA_REQUEST_ADC1;
        hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode                = DMA_CIRCULAR;
        hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();

        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);
    }
}

void SlaveMeas_Init(void)
{
    ADC_ChannelConfTypeDef  sConfig = {0};
    TIM_MasterConfigTypeDef sMaster = {0};
    uint32_t i;

    /* Known mid-scale contents until the first real samples arrive. */
    for (i = 0U; i < (MEAS_BUF_FRAMES * MEAS_CHANNELS); i++) s_buf[i] = 2048U;

    /* TIM3: update event every 50 us (20 kHz) -> TRGO -> ADC trigger. */
    __HAL_RCC_TIM3_CLK_ENABLE();
    s_htim3.Instance               = TIM3;
    s_htim3.Init.Prescaler         = 0U;
    s_htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim3.Init.Period            = (uint32_t)(64000000UL / MEAS_SAMPLE_RATE_HZ) - 1U;   /* 3199 */
    s_htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&s_htim3) != HAL_OK) Error_Handler();
    sMaster.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMaster.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&s_htim3, &sMaster) != HAL_OK) Error_Handler();

#if (SLAVE_IOUT != 0)
    {   /* PA2 as an analogue input (a CubeMX regeneration with PA2 = ADC1_IN2 does the same) */
        GPIO_InitTypeDef gi = {0}; __HAL_RCC_GPIOA_CLK_ENABLE();
        gi.Pin = GPIO_PIN_2; gi.Mode = GPIO_MODE_ANALOG; gi.Pull = GPIO_NOPULL; HAL_GPIO_Init(GPIOA, &gi);
    }
#endif
    /* ADC1: 3- (4- with SLAVE_IOUT) channel scan, one scan per TIM3 trigger, circular DMA. */
    __HAL_RCC_ADC_CLK_ENABLE();
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;       /* 16 MHz */
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = MEAS_CHANNELS;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T3_TRGO;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.SamplingTimeCommon1   = ADC_SAMPLETIME_39CYCLES_5;
    hadc1.Init.SamplingTimeCommon2   = ADC_SAMPLETIME_39CYCLES_5;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    /* Rank order = order of the values inside each DMA frame. */
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    sConfig.Channel = ADC_CHANNEL_0;  sConfig.Rank = ADC_REGULAR_RANK_1;     /* PA0 AC_IN  */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
    sConfig.Channel = ADC_CHANNEL_1;  sConfig.Rank = ADC_REGULAR_RANK_2;     /* PA1 AC_OUT */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
    sConfig.Channel = ADC_CHANNEL_5;  sConfig.Rank = ADC_REGULAR_RANK_3;     /* PA5 NTC    */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
#if (SLAVE_IOUT != 0)
    sConfig.Channel = ADC_CHANNEL_2;  sConfig.Rank = ADC_REGULAR_RANK_4;     /* PA2 I-OUT: the converter-side current (slave_iout.h) */
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
#endif

    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) Error_Handler();

    MeasDue_Init(&s_clk, SlaveZC_NowUs());

    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_buf, MEAS_BUF_FRAMES * MEAS_CHANNELS) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_Base_Start(&s_htim3) != HAL_OK) Error_Handler();
}

void SlaveMeas_Service(void)
{
    uint32_t now = SlaveZC_NowUs();
    uint32_t total, remaining, frames_done, n, start;
    float    period_us, t;
    float    rms_i, mean_i, rms_o, mean_o;
    uint16_t min_i, max_i, min_o, max_o;

    if (MeasDue(&s_clk, now, MEAS_UPDATE_US, MEAS_WARMUP_US) == 0U) return;     /* v0.9.71: the warm-up is no longer re-checked at every TIM2 wrap (F10 every 71.6 min) */

    /* Mains period from the zero-cross rate - only if the ZC edges are fresh
     * (seen within the last 100 ms) and the period is plausible. Otherwise
     * assume 50 Hz and say so (freq_valid = 0). */
    t = SlaveMeas_MainsPeriodFromZc(g_zc.period_rise_us);
    if ((t > 0.0f) && (SlaveZC_AgeUs() < 100000UL))
    {
        if (s_period_filt_us <= 0.0f) s_period_filt_us = t;
        else                          s_period_filt_us += (t - s_period_filt_us) * 0.125f;
        g_meas.freq_valid = 1U;
        g_meas.freq_hz    = 1000000.0f / s_period_filt_us;
        period_us         = s_period_filt_us;
    }
    else
    {
        s_period_filt_us  = 0.0f;
        g_meas.freq_valid = 0U;
        g_meas.freq_hz    = 0.0f;
        period_us         = 20000.0f;
    }
    n = SlaveMeas_WindowFrames(period_us);

    /* Where is DMA now? CNDTR counts the transfers still to do (circular). */
    total     = MEAS_BUF_FRAMES * MEAS_CHANNELS;
    remaining = hdma_adc1.Instance->CNDTR;
    if (remaining > total) remaining = total;
    frames_done = ((total - remaining) / MEAS_CHANNELS) % MEAS_BUF_FRAMES;   /* complete frames */
    start = WindowStart(frames_done, n);

    SlaveMeas_ComputeStats(s_buf, MEAS_BUF_FRAMES, start, n, MEAS_CHANNELS, MEAS_CH_IN,
                           &rms_i, &mean_i, &min_i, &max_i);
    SlaveMeas_ComputeStats(s_buf, MEAS_BUF_FRAMES, start, n, MEAS_CHANNELS, MEAS_CH_OUT,
                           &rms_o, &mean_o, &min_o, &max_o);

    g_meas.rms_in_counts   = rms_i;   g_meas.mean_in_counts  = mean_i;
    g_meas.rms_out_counts  = rms_o;   g_meas.mean_out_counts = mean_o;
    s_sign_mean_sum[MEAS_CH_IN]  = (int32_t)((mean_i * (float)MEAS_SIGN_SPAN_FRAMES) + 0.5f);
    s_sign_mean_sum[MEAS_CH_OUT] = (int32_t)((mean_o * (float)MEAS_SIGN_SPAN_FRAMES) + 0.5f);
    g_meas.min_in = min_i;  g_meas.max_in = max_i;
    g_meas.min_out = min_o; g_meas.max_out = max_o;
    g_meas.clipped_in  = ((max_i >= 4090U) || (min_i <= 5U)) ? 1U : 0U;
    g_meas.clipped_out = ((max_o >= 4090U) || (min_o <= 5U)) ? 1U : 0U;

    g_meas.vrms_in  = SlaveMeas_ApplyCal(&g_meas_cal[MEAS_CH_IN],  rms_i);
    g_meas.vrms_out = SlaveMeas_ApplyCal(&g_meas_cal[MEAS_CH_OUT], rms_o);

#if (SLAVE_IOUT != 0)
    {   /* the converter-side current: the same window as the voltages */
        float rms_c, mean_c; uint16_t min_c, max_c; IoutResult_t ir;
        SlaveMeas_ComputeStats(s_buf, MEAS_BUF_FRAMES, start, n, MEAS_CHANNELS, MEAS_CH_IOUT, &rms_c, &mean_c, &min_c, &max_c);
        Iout_Compute(rms_c, mean_c, min_c, max_c, &ir);
        g_meas.irms_a = ir.irms_a; g_meas.ipk_a = ir.ipk_a; g_meas.clipped_iout = ir.clipped;
    }
#endif
    g_meas.window_frames = n;
    g_meas.ntc_counts    = s_buf[((((frames_done + MEAS_BUF_FRAMES) - 1U) % MEAS_BUF_FRAMES) * MEAS_CHANNELS)
                                 + MEAS_CH_NTC];
    g_meas.update_count++;
    g_meas.valid = 1U;
}

/* ------------------------------------------------------------------------- */
/* Instantaneous polarity (see slave_meas.h) - integer only, ISR-safe         */
/* ------------------------------------------------------------------------- */
int8_t SlaveMeas_SignFromRing(const volatile uint16_t *buf, uint32_t frames, uint32_t frames_done,
                              uint32_t channels, uint32_t ch, int32_t mean_sum,
                              uint32_t back, uint32_t span, int32_t min_abs_sum,
                              int32_t *dev_sum_out)
{
    uint32_t k, idx;
    int32_t  sum = 0, dev;

    if ((frames == 0U) || (span == 0U))
    {
        if (dev_sum_out != 0) *dev_sum_out = 0;
        return 0;
    }

    for (k = 0U; k < span; k++)
    {
        /* newest complete frame is frames_done - 1; step back 'back' + k frames */
        idx  = (frames_done + frames - 1U - back - k) % frames;
        sum += (int32_t)buf[(idx * channels) + ch];
    }
    dev = sum - mean_sum;

    if (dev_sum_out != 0) *dev_sum_out = dev;
    if (dev >  min_abs_sum) return  1;
    if (dev < -min_abs_sum) return -1;
    return 0;
}

int8_t SlaveMeas_SignLatest(uint32_t ch, int32_t min_counts)
{
    uint32_t total, remaining, frames_done;

    if (!g_meas.valid) return 0;

    total       = MEAS_BUF_FRAMES * MEAS_CHANNELS;
    remaining   = hdma_adc1.Instance->CNDTR;
    frames_done = ((total - remaining) / MEAS_CHANNELS) % MEAS_BUF_FRAMES;

    /* bias of MEAS_GUARD_SPAN_FRAMES samples = bias_sum(8 samples) * 2 / 8 */
    return SlaveMeas_SignFromRing(s_buf, MEAS_BUF_FRAMES, frames_done, MEAS_CHANNELS, ch,
                                  (s_sign_mean_sum[ch] * (int32_t)MEAS_GUARD_SPAN_FRAMES) / (int32_t)MEAS_SIGN_SPAN_FRAMES,
                                  0U, MEAS_GUARD_SPAN_FRAMES,
                                  (int32_t)(min_counts * (int32_t)MEAS_GUARD_SPAN_FRAMES), 0);
}

void SlaveMeas_TickIrqEnable(void)
{
    TIM3->SR    = ~TIM_SR_UIF;
    TIM3->DIER |= TIM_DIER_UIE;
    HAL_NVIC_SetPriority(TIM3_TIM4_IRQn, IRQ_PRIO_MEAS, 0);
    HAL_NVIC_EnableIRQ(TIM3_TIM4_IRQn);
}

int8_t SlaveMeas_SignBeforeNow(uint32_t ch, int32_t *dev_sum_out)
{
    uint32_t total, remaining, frames_done;

    if (!g_meas.valid)
    {
        if (dev_sum_out != 0) *dev_sum_out = 0;
        return 0;
    }

    total       = MEAS_BUF_FRAMES * MEAS_CHANNELS;
    remaining   = hdma_adc1.Instance->CNDTR;
    frames_done = ((total - remaining) / MEAS_CHANNELS) % MEAS_BUF_FRAMES;

    return SlaveMeas_SignFromRing(s_buf, MEAS_BUF_FRAMES, frames_done, MEAS_CHANNELS, ch,
                                  s_sign_mean_sum[ch],
                                  MEAS_SIGN_BACK_FRAMES, MEAS_SIGN_SPAN_FRAMES,
                                  MEAS_SIGN_MIN_SUM, dev_sum_out);
}
