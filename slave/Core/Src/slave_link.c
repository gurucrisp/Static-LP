/**
  ******************************************************************************
  * @file    slave_link.c
  * @brief   Slave side of the master link. See slave_link.h and link_proto.h.
  *
  * Transmission is polled, not interrupt-driven: SlaveLink_Service() puts the
  * next byte into the UART data register whenever the register is empty. That
  * keeps the module free of IRQ handlers and of any blocking wait; the price
  * is that the main loop must call it often (see slave_link.h).
  ******************************************************************************
  */

#include "main.h"
#include "link_proto.h"
#include "slave_link.h"
#include "slave_meas.h"
#include "slave_zc.h"
#include "slave_config.h"
#include "slave_features.h"
#if (SLAVE_IOUT != 0)
#include "slave_iout.h"
#include "slave_meas.h"
#endif
#if (SLAVE_AUTOTUNE != 0)
#include "slave_profile.h"
#endif
#if (SLAVE_POWER_STAGE >= 1)
#include "slave_ctrl.h"
#endif
#if (SLAVE_LINK_V2 != 0)
#include "slave_ctrl_hw.h"        /* CtrlHw_NowMs(): the time base the controller uses */
#include "slave_irq_prio.h"
#include "link_v2.h"
#include "slave_link2.h"
#if (SLAVE_SEQ_SYNC != 0)
#include "slave_phase.h"           /* the phase sequence measurement (v0.9.51) */
#endif
#endif

extern UART_HandleTypeDef huart1;          /* USART1: PB6 = TX, PB7 = RX (main.c) */

#define TXBUF_SIZE   256U                  /* power of two */
#define TXBUF_MASK   (TXBUF_SIZE - 1U)

static uint8_t           s_tx[TXBUF_SIZE];
static uint32_t          s_head;           /* next byte to be written into the queue */
static uint32_t          s_tail;           /* next byte to be sent                   */
static uint32_t          s_t_last_us;
static uint8_t           s_seq;

SlaveLinkStats_t g_link;

static uint32_t tx_free(void)
{
    return (TXBUF_SIZE - 1U) - ((s_head - s_tail) & TXBUF_MASK);
}

#if (SLAVE_LINK_V2 != 0)
/* ---- Link version 2: the receive side, and the hooks of slave_link2.c --------------------------------------------------------------------------------------------- */
#define RXBUF_SIZE   256U                  /* power of two */
#define RXBUF_MASK   (RXBUF_SIZE - 1U)
static volatile uint8_t  s_rx[RXBUF_SIZE];
static volatile uint32_t s_rx_head;        /* written by the receive interrupt only */
static volatile uint32_t s_rx_tail;        /* written by the main loop only          */
static uint8_t           s_reset_cause = 2U;   /* 0 power-on, 1 watchdog, 2 other (pin, software, low power) */

void SlaveLink_RxIrq(void)
{
    USART_TypeDef *u = huart1.Instance;
    uint32_t isr = u->ISR;
    if ((isr & USART_ISR_ORE) != 0U)
    {
        u->ICR = USART_ICR_ORECF;                                  /* a byte was lost before we read it: the frame it belonged to is dropped by its checksum */
        g_link.rx_overruns++;
    }
    if ((isr & (USART_ISR_FE | USART_ISR_NE)) != 0U)
    {
        u->ICR = USART_ICR_FECF | USART_ICR_NECF;
    }
    while ((u->ISR & USART_ISR_RXNE_RXFNE) != 0U)
    {
        uint8_t  b  = (uint8_t)u->RDR;
        uint32_t nh;
#if (SLAVE_SEQ_SYNC != 0)
        if (b == (uint8_t)LINKV2_SYNC_BYTE) { SlavePhase_OnSync(SlaveZC_NowUs()); continue; }       /* the SYNC byte of the phase sequence measurement: time-stamped HERE with the zero-cross clock; it is no part of any frame */
#endif
        nh = (s_rx_head + 1U) & RXBUF_MASK;
        if (nh != s_rx_tail) { s_rx[s_rx_head] = b; s_rx_head = nh; }
        else                 { g_link.rx_dropped++; }
    }
}

static uint16_t to_dv(float v);

static void hk_on_control(const LinkM_t *m, uint32_t now_ms)
{
    SlaveCtrl_OnControl(m->run, m->setv_dv, m->clr, m->maxc_dv, now_ms);
    SlaveCtrl_OnLoadHold(m->hold, now_ms);                    /* v0.9.61 */
}
#if (SLAVE_AUTOTUNE != 0)
#include "slave_tune_link.h"
#include "slave_profile.h"
/* the autotune over the link: $K TUNE / TABORT and the $T progress frames (slave_tune_link.h) */
static int hk_tune_cmd(uint8_t cmd, const char *name, int32_t num, uint8_t *status, int32_t *value, uint32_t now_ms) { return SlaveTuneLink_Command(cmd, name, num, status, value, now_ms); }
static int hk_tune_frame(LinkT_t *f, uint32_t now_ms) { return SlaveTuneLink_NextFrame(f, now_ms); }
#define SLAVE_TUNE_HOOKS hk_tune_cmd, hk_tune_frame
#else
#define SLAVE_TUNE_HOOKS 0, 0
#endif

#if (SLAVE_SEQ_SYNC != 0)
static uint32_t s_ph_pm;                                                                     /* the interrupt mask saved by the lock */
static void ph_lock(void)   { s_ph_pm = __get_PRIMASK(); __disable_irq(); }
static void ph_unlock(void) { __set_PRIMASK(s_ph_pm); }
static void hk_seq_arm(uint8_t round, uint32_t now_ms) { SlavePhase_Arm(round, now_ms); }
#define SLAVE_SEQ_HOOK hk_seq_arm
#else
#define SLAVE_SEQ_HOOK 0
#endif

static void hk_fill_e(LinkE_t *e)
{
    e->setv_dv      = SlaveCtrl_SetvDv();
    e->flags2       = SlaveCtrl_Flags2();
    e->clr          = SlaveCtrl_ClrEcho();
    e->reach_lo_dv  = 0U;                  /* the input range the unit can regulate: measured by the autotune (step 5) */
    e->reach_hi_dv  = 0U;
#if (SLAVE_AUTOTUNE != 0)
    e->flags2 = (uint16_t)(e->flags2 | SlaveTuneLink_Flags2());       /* BUSY while a sweep runs, NOT_TUNED without a valid profile or Max Correction */
#endif
#if (SLAVE_SEQ_SYNC != 0)
    SlavePhase_Result(&e->sq_id, &e->sq_delay_us, &e->sq_period_us);                 /* the last phase sequence round: id, time from the SYNC byte to this slave's rising zero-cross, its mains period */
#endif
#if (SLAVE_IOUT != 0)
    e->iout_rms_ca  = Iout_ToCentiAmps(g_meas.irms_a);          /* the converter-side current, hundredths of an ampere (slave_iout.h) */
    e->iout_pk_ca   = Iout_ToCentiAmps(g_meas.ipk_a);
    if (g_meas.valid != 0U) e->flags2 = (uint16_t)(e->flags2 | LINKV2_FLAG_IOUT_VALID | ((g_meas.clipped_iout != 0U) ? LINKV2_FLAG_IOUT_CLIP : 0U));
#endif
}
static void hk_fill_h(LinkH_t *h)
{
    h->fw          = SLAVE_FW_VERSION;
#if (SLAVE_TEST_NO_VOLTAGE_CUTS == 0)
    h->vin_min_dv  = to_dv(CTRL_VIN_MIN_V);
    h->vin_max_dv  = to_dv(CTRL_VIN_MAX_V);
#else
    h->vin_min_dv  = 0U;                    /* TEST: the slave announces no limits of its own */
    h->vin_max_dv  = 6500U;
#endif
    h->rst         = s_reset_cause;
#if (SLAVE_AUTOTUNE != 0)
    h->tuned       = SlaveProfile_Tuned();   /* a valid autotune profile is stored in flash */
#if (SLAVE_AUTOTUNE != 0)
    {   /* the profile's numbers, for the master's "Transformer Profile" page (optional trailing fields of $H) */
        TuneProfile_t p;
        if (SlaveProfile_Get(&p) == 1U) { h->prof_cap = p.cap_ticks; h->prof_kib = p.kib_m; h->prof_kiu = p.kiu_m; h->prof_vref_dv = p.vin_ref_dv; h->prof_maxc_dv = p.maxc_dv; h->prof_n = p.n; h->prof_knee = p.knee; }
    }
#endif
#else
    h->tuned       = 1U;                   /* the built-in profile is in force (SLAVE_AUTOTUNE 0) */
#endif
#if (SLAVE_TEST_NO_VOLTAGE_CUTS == 0)
    h->vout_max_dv = to_dv(CTRL_VOUT_MAX_V);
#else
    h->vout_max_dv = 6500U;
#endif
}
static uint8_t hk_tx(const char *frame, uint32_t n)
{
    uint32_t i;
    if (tx_free() < n) return 0U;
    for (i = 0U; i < n; i++) { s_tx[s_head] = (uint8_t)frame[i]; s_head = (s_head + 1U) & TXBUF_MASK; }
    g_link.frames_sent++;
    return 1U;
}
#if (SLAVE_VOLTAGE_CAL != 0)
#include "slave_cal.h"
/* voltage calibration over $K (SLAVE_VOLTAGE_CAL): the link's command code is translated to the calibration module's */
static int hk_cal_command(uint8_t cmd, int32_t num, char letter, uint8_t *status, int32_t *value, uint32_t now_ms)
{
    uint8_t c = (cmd == (uint8_t)LINKCMD_CALV) ? SCAL_CMD_CALV : ((cmd == (uint8_t)LINKCMD_CALO) ? SCAL_CMD_CALO : SCAL_CMD_CALR);
    return SlaveCal_Command(c, num, letter, status, value, now_ms);
}
static int hk_cal_poll(uint8_t *status, int32_t *value, uint32_t now_ms) { return SlaveCal_Poll(status, value, now_ms); }
static const SL2_Hooks_t s_hooks = { hk_on_control, hk_fill_e, hk_fill_h, hk_tx, hk_cal_command, hk_cal_poll, SLAVE_TUNE_HOOKS, SLAVE_SEQ_HOOK };
#else
static const SL2_Hooks_t s_hooks = { hk_on_control, hk_fill_e, hk_fill_h, hk_tx, 0, 0, SLAVE_TUNE_HOOKS, SLAVE_SEQ_HOOK };
#endif

static void capture_reset_cause(void)
{
    uint32_t csr = RCC->CSR;
    if      ((csr & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) != 0U) s_reset_cause = 1U;
    else if ((csr & RCC_CSR_PWRRSTF) != 0U)                       s_reset_cause = 0U;
    else                                                          s_reset_cause = 2U;
    RCC->CSR |= RCC_CSR_RMVF;              /* clear the flags so the next reset is read on its own */
}
#endif /* SLAVE_LINK_V2 */

/* volts -> tenths of a volt, rounded and limited to the 16-bit field */
static uint16_t to_dv(float v)
{
    float x = (v * 10.0f) + 0.5f;
    if (x < 0.0f)       x = 0.0f;
    if (x > 65535.0f)   x = 65535.0f;
    return (uint16_t)x;
}

static void queue_status_frame(void)
{
    LinkStatus_t st;
    char         f[LINK_STATUS_MAX_LEN];
    int32_t      n;
    uint32_t     i;

    st.phase = SLAVE_STATUS_PHASE;
    st.seq   = s_seq++;                    /* counts even when a frame is dropped,
                                              so the master can see the gap        */
    st.flags = 0U;

    if (g_meas.valid)
    {
        st.vin_dv  = to_dv(g_meas.vrms_in);
        st.vout_dv = to_dv(g_meas.vrms_out);
        st.flags  |= LINK_FLAG_MEAS_VALID;
    }
    else
    {
        st.vin_dv  = 0U;
        st.vout_dv = 0U;
    }

    if (g_meas.freq_valid)
    {
        float hz100 = (g_meas.freq_hz * 100.0f) + 0.5f;
        st.freq_chz = (uint16_t)((hz100 > 65535.0f) ? 65535.0f : hz100);
        st.flags   |= LINK_FLAG_FREQ_VALID;
    }
    else
    {
        st.freq_chz = 0U;
    }

    if (g_meas_cal[MEAS_CH_IN].calibrated && g_meas_cal[MEAS_CH_OUT].calibrated)
    {
        st.flags |= LINK_FLAG_CALIBRATED;      /* both channels calibrated */
    }
    if (g_meas.clipped_in)     st.flags |= LINK_FLAG_CLIP_IN;
    if (g_meas.clipped_out)    st.flags |= LINK_FLAG_CLIP_OUT;

#if (SLAVE_POWER_STAGE >= 1)
    st.state   = SlaveCtrl_LinkState();    /* 0 stopped, 1 starting, 2 running, 3 fault */
    st.fault   = SlaveCtrl_FaultCode();    /* CF_* code while in fault, else 0          */
    {
        st.diag  = SlaveCtrl_DiagWord();
        st.diag2 = SlaveCtrl_Diag2();     /* trip capture in FAULT, the transformer result while the converter runs, else the passive mapping check */
        uint8_t ctx = SlaveCtrl_FaultContext();                  /* fault context (state, mode, FLT/BKIN levels) ...   */
        if (ctx == 0U) ctx = SlaveCtrl_WaitContext();            /* ... or, while waiting in SYNC, what it waits for  */
        st.flags |= (uint16_t)((uint16_t)ctx << 8);              /* bits 8..15 of flags (see slave_ctrl.h)            */
    }
#else
    st.state   = 0U;                       /* power stage off in this build             */
    st.fault   = 0U;
#endif
    st.ntc_raw = (uint16_t)((g_meas.ntc_counts > 4095U) ? 4095U : g_meas.ntc_counts);

    n = Link_BuildStatus(f, sizeof f, &st);
    if ((n > 0) && (tx_free() >= (uint32_t)n))
    {
        for (i = 0U; i < (uint32_t)n; i++)
        {
            s_tx[s_head] = (uint8_t)f[i];
            s_head = (s_head + 1U) & TXBUF_MASK;
        }
        g_link.frames_sent++;
    }
    else
    {
        g_link.frames_dropped++;
    }
}

void SlaveLink_Init(void)
{
    s_head = 0U;
    s_tail = 0U;
    s_seq  = 0U;
    g_link.frames_sent    = 0U;
    g_link.frames_dropped = 0U;
    g_link.bytes_sent     = 0U;
    g_link.rx_overruns    = 0U;
    g_link.rx_dropped     = 0U;
    s_t_last_us = SlaveZC_NowUs();
#if (SLAVE_LINK_V2 != 0)
    capture_reset_cause();
    s_rx_head = 0U; s_rx_tail = 0U;
#if (SLAVE_LINK_RX_FIFO != 0)
    (void)HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8);     /* the interrupt fires per byte; the FIFO only gives it slack */
    (void)HAL_UARTEx_EnableFifoMode(&huart1);
#endif
    huart1.Instance->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
#if (SLAVE_SEQ_SYNC != 0)
    SlavePhase_Init(ph_lock, ph_unlock);
#endif
    SL2_Init(&s_hooks, CtrlHw_NowMs());
    huart1.Instance->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
    HAL_NVIC_SetPriority(USART1_IRQn, IRQ_PRIO_LINK, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
#endif
}

void SlaveLink_Service(void)
{
    USART_TypeDef *u = huart1.Instance;
    uint32_t       now;

    /* Feed the UART: one byte whenever its data register is empty. */
    while ((s_tail != s_head) && ((u->ISR & USART_ISR_TXE_TXFNF) != 0U))
    {
        u->TDR = s_tx[s_tail];
        s_tail = (s_tail + 1U) & TXBUF_MASK;
        g_link.bytes_sent++;
    }

#if (SLAVE_LINK_V2 != 0)
    {   /* version 2: take what the receive interrupt stored, then let the link logic send its frames. $M and $K are acted on HERE, in the main loop, next to the controller. */
        uint32_t now_ms = CtrlHw_NowMs();
        while (s_rx_tail != s_rx_head)
        {
            SL2_Rx(s_rx[s_rx_tail], now_ms);
            s_rx_tail = (s_rx_tail + 1U) & RXBUF_MASK;
        }
        SL2_Service(now_ms);
#if (SLAVE_SEQ_SYNC != 0)
        SlavePhase_Service(now_ms);
#endif
    }
#endif

    /* Every SLAVE_LINK_PERIOD_US (10 ms), queue a new status frame. */
    now = SlaveZC_NowUs();
    if ((uint32_t)(now - s_t_last_us) >= SLAVE_LINK_PERIOD_US)
    {
        s_t_last_us = now;
        queue_status_frame();
    }
}
