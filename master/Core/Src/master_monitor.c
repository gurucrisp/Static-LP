/**
  ******************************************************************************
  * @file    master_monitor.c
  * @brief   Master bring-up monitor. See master_monitor.h.
  ******************************************************************************
  */

#include <stdio.h>
#include <string.h>
#include "master_monitor.h"
#if (MASTER_RTC_ENABLED == 1)
#include "master_rtc.h"
#include "master_iwdg.h"
#include "m_hours.h"
#endif
#include "lcd.h"
#include "link_v2.h"
#include "master_links.h"

MasterMon_t g_mon;
static LinkV2Rx_t s_l2rx;       /* a second receiver for the version 2 frames ($E, $H); $S still goes through the version 1 one */

static LinkRx_t  s_lrx;
static uint32_t  s_fps_count;
static uint32_t  s_fps_window_start_ms;

/* ========================================================================= */
/* Frame bookkeeping (no hardware)                                           */
/* ========================================================================= */

void Mon_Reset(void)
{
    memset(&g_mon, 0, sizeof g_mon);
    LinkRx_Init(&s_lrx);
    LinkV2Rx_Init(&s_l2rx);
    s_fps_count = 0U;
    s_fps_window_start_ms = 0U;
}

static void on_frame(MasterMon_t *m, uint32_t *fps, const LinkStatus_t *st, uint32_t now_ms)
{
    /* Count missing frames from gaps in the sequence number - but only while
     * the link was alive. After a silent spell (longer than the link timeout)
     * the slave has probably restarted and its counter began again at 0, so
     * just re-synchronise. A jump of more than 100 is treated the same way. */
    if (m->have_seq &&
        ((uint32_t)(now_ms - m->t_last_ms) < MON_LINK_TIMEOUT_MS))
    {
        uint8_t gap = (uint8_t)(st->seq - m->last_seq - 1U);
        if (gap <= 100U)
        {
            m->lost += gap;
        }
    }
    m->last_seq   = st->seq;
    m->have_seq   = 1U;
    m->last       = *st;
    m->have_frame = 1U;
    m->t_last_ms  = now_ms;
    (*fps)++;
}

/* The decoder of ONE slave link: its own monitor state, version 1 receiver ($S), version 2 receiver ($E $H $T $R) and frame counter. The R link uses the globals (Mon_ProcessBytes), the Y and B links their own (master_links.c). */
void Mon_ProcessBytesCtx(MasterMon_t *m, LinkRx_t *lrx, LinkV2Rx_t *l2rx, uint32_t *fps,
                         const volatile uint8_t *ring, uint32_t ring_size,
                         uint32_t *rd, uint32_t wr, uint32_t now_ms)
{
    LinkStatus_t st;
    uint32_t     r    = *rd;
    uint32_t     mask = ring_size - 1U;

    while (r != wr)
    {
        uint8_t b = ring[r];
        r = (r + 1U) & mask;
        m->bytes_rx++;
        if (LinkRx_Feed(lrx, b, &st))
        {
            on_frame(m, fps, &st, now_ms);
        }
        {   LinkV2Frame_t f2; int32_t t2 = LinkV2Rx_Feed(l2rx, b, &f2);
            if (t2 == (int32_t)LINKV2_E) { m->e_valid = 1U; m->e_flags2 = f2.u.e.flags2; m->e_clr = f2.u.e.clr; m->e_setv_dv = f2.u.e.setv_dv; m->e_iout_rms_ca = f2.u.e.iout_rms_ca; m->e_iout_pk_ca = f2.u.e.iout_pk_ca; m->e_sq_id = f2.u.e.sq_id; m->e_sq_delay_us = f2.u.e.sq_delay_us; m->e_sq_period_us = f2.u.e.sq_period_us; m->e_t_ms = now_ms; }
            else if (t2 == (int32_t)LINKV2_T) { m->t_step = f2.u.t.step; m->t_ticks = f2.u.t.ticks; m->t_vin_dv = f2.u.t.vin_dv; m->t_dv_dv = f2.u.t.dv; m->t_result = f2.u.t.result; m->t_n++; }
            else if (t2 == (int32_t)LINKV2_R) { m->r_id = f2.u.r.id; m->r_status = f2.u.r.status; m->r_value = f2.u.r.value; m->r_n++; }
            else if (t2 == (int32_t)LINKV2_H) { m->h_valid = 1U; m->h_win_lo_dv = f2.u.h.vin_min_dv; m->h_win_hi_dv = f2.u.h.vin_max_dv; m->h_vout_max_dv = f2.u.h.vout_max_dv; m->h_rst = f2.u.h.rst; m->h_tuned = f2.u.h.tuned; m->h_prof_cap = f2.u.h.prof_cap; m->h_prof_kib = f2.u.h.prof_kib; m->h_prof_kiu = f2.u.h.prof_kiu; m->h_prof_vref_dv = f2.u.h.prof_vref_dv; m->h_prof_maxc_dv = f2.u.h.prof_maxc_dv; m->h_prof_n = f2.u.h.prof_n; m->h_prof_knee = f2.u.h.prof_knee; }
        }
    }
    *rd = r;
    m->frames_ok  = lrx->frames_ok;
    m->frames_bad = lrx->frames_bad;
    m->overflows  = lrx->overflows;
}

void Mon_ProcessBytes(const volatile uint8_t *ring, uint32_t ring_size,
                      uint32_t *rd, uint32_t wr, uint32_t now_ms)
{
    Mon_ProcessBytesCtx(&g_mon, &s_lrx, &s_l2rx, &s_fps_count, ring, ring_size, rd, wr, now_ms);        /* the R link */
}

void Mon_Tick(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - s_fps_window_start_ms) >= 1000U)
    {
        g_mon.fps = s_fps_count;
        s_fps_count = 0U;
        s_fps_window_start_ms = now_ms;
    }
}

/* ========================================================================= */
/* Screen composition (no hardware)                                          */
/* ========================================================================= */

static void put_text(char *row, uint32_t col, const char *s)
{
    while ((*s != '\0') && (col < MON_LCD_COLS))
    {
        row[col++] = *s++;
    }
}

static uint32_t cap99(uint32_t v)
{
    return (v > 99U) ? 99U : v;
}

void Mon_Compose(const MasterMon_t *m, uint32_t now_ms,
                 char rows[MON_LCD_ROWS][MON_LCD_COLS + 1U])
{
    char     tmp[40];
    uint32_t r;
    int      linked = (m->have_frame != 0U) &&
                      ((uint32_t)(now_ms - m->t_last_ms) < MON_LINK_TIMEOUT_MS);

    for (r = 0U; r < MON_LCD_ROWS; r++)
    {
        memset(rows[r], ' ', MON_LCD_COLS);
        rows[r][MON_LCD_COLS] = '\0';
    }

    if (linked)
    {
        const LinkStatus_t *s = &m->last;

        if ((s->state != 0U) || (s->fault != 0U))
        {
            /* The slave's power stage reports something: show it instead of the
             * calibration flag. state: 1 starting, 2 running, 3 fault;
             * fault: slave_ctrl.h CF_* code (e.g. 1 break, 2 FLT, 8 ZC lost). */
            const char *sn = (s->state == 1U) ? "START" : ((s->state == 2U) ? "RUN" :
                             ((s->state == 3U) ? "FAULT" : "?"));
            (void)snprintf(tmp, sizeof tmp, "SLAVE %c %-5s F%02u", s->phase, sn,
                           (unsigned)((s->fault > 99U) ? 99U : s->fault));
        }
        else
        {
            (void)snprintf(tmp, sizeof tmp, "SLAVE %c LINK OK %s", s->phase,
                           ((s->flags & LINK_FLAG_CALIBRATED) != 0U) ? "CAL" : "RAW");
        }
        put_text(rows[0], 0U, tmp);

        if ((s->flags & LINK_FLAG_MEAS_VALID) != 0U)
        {
            (void)snprintf(tmp, sizeof tmp, "Vin  %u.%u V",
                           (unsigned)(s->vin_dv / 10U), (unsigned)(s->vin_dv % 10U));
            put_text(rows[1], 0U, tmp);
            (void)snprintf(tmp, sizeof tmp, "Vout %u.%u V",
                           (unsigned)(s->vout_dv / 10U), (unsigned)(s->vout_dv % 10U));
            put_text(rows[2], 0U, tmp);
        }
        else
        {
            put_text(rows[1], 0U, "Vin  --- V");
            put_text(rows[2], 0U, "Vout --- V");
        }
        /* bench diagnostics: F<idle>/<running> = FLT pin falling edges while the converter was NOT running / running;
         * K<idle>/<running> = break-input events. Shown only once something has been seen. */
        if ((s->diag & 0x03FFU) != 0U)
        {
            (void)snprintf(tmp, sizeof tmp, "F%u/%u", (unsigned)(s->diag & 31U), (unsigned)((s->diag >> 5) & 31U));
            put_text(rows[1], 13U, tmp);
        }
        if ((s->diag & 0xFC00U) != 0U)
        {
            (void)snprintf(tmp, sizeof tmp, "K%u/%u", (unsigned)((s->diag >> 10) & 7U), (unsigned)((s->diag >> 13) & 7U));
            put_text(rows[2], 13U, tmp);
        }
        if ((s->flags & LINK_FLAG_CLIP_IN) != 0U)  put_text(rows[1], 15U, "CLIP");
        if ((s->flags & LINK_FLAG_CLIP_OUT) != 0U) put_text(rows[2], 15U, "CLIP");

        if ((s->flags & LINK_FLAG_FREQ_VALID) != 0U)
        {
            (void)snprintf(tmp, sizeof tmp, "%u.%02uHz",
                           (unsigned)(s->freq_chz / 100U), (unsigned)(s->freq_chz % 100U));
        }
        else
        {
            (void)snprintf(tmp, sizeof tmp, "--.--Hz");
        }
        put_text(rows[3], 0U, tmp);
        if (((s->diag2 & 0x4000U) != 0U) && ((s->diag2 & 0x8000U) == 0U))
        {
            /* passive converter-mapping check: M<agree>/<disagree> (see slave_ctrl.h; replaces the B / f statistics) */
            (void)snprintf(tmp, sizeof tmp, " M%u/%u L%u", (unsigned)(s->diag2 & 127U), (unsigned)((s->diag2 >> 7) & 127U),
                           (unsigned)cap99(m->lost));
        }
        else
        {
            (void)snprintf(tmp, sizeof tmp, " L%u B%u f%u",
                           (unsigned)cap99(m->lost), (unsigned)cap99(m->frames_bad),
                           (unsigned)((m->fps > 999U) ? 999U : m->fps));
        }
        put_text(rows[3], 7U, tmp);

        if (((s->diag2 & 0xF000U) == 0x1000U) && (((now_ms / 2000U) & 1U) != 0U))
        {
            /* stage 3 regulator state (slave SlaveCtrl_RegWord): every other 2 s row 3 shows   BOOST 462t / BUCK 310t / FREEZE, plus " CAP" when the correction sits at the cap.
             * diag2 bit 11 = boost, bit 10 = at the cap, bits 9..0 = the effective on-time in ticks / 4. The fault and wait pages below replace this row again. */
            const uint32_t eff = (uint32_t)(s->diag2 & 0x03FFU) * 4U;
            uint32_t       k;
            if (eff == 0U) (void)snprintf(tmp, sizeof tmp, "FREEZE");
            else           (void)snprintf(tmp, sizeof tmp, "%s %ut%s", ((s->diag2 & 0x0800U) != 0U) ? "BOOST" : "BUCK", (unsigned)eff, ((s->diag2 & 0x0400U) != 0U) ? " CAP" : "");
            for (k = 0U; k < MON_LCD_COLS; k++) rows[3][k] = ' ';
            put_text(rows[3], 0U, tmp);
        }
        else if (((s->diag2 & 0xE000U) == 0x2000U) && (((now_ms / 2000U) & 1U) != 0U))
        {
            /* transformer-connection check (slave SLAVE_TF_CHECK): every other 2 s row 3 shows   TF OK +3.2V / TF FLIP -3.0V / TF ? +0.2V
             * diag2 bits 12..11 = verdict (1 OK, 2 flipped, 3 inconclusive), bits 10..0 = (Vout - Vin) in 0.1 V, two's complement. The fault and
             * wait pages below replace this row again. */
            int32_t        dv  = (int32_t)(s->diag2 & 0x07FFU);
            const uint32_t vd  = (uint32_t)((s->diag2 >> 11) & 3U);
            uint32_t       k, a;
            if ((dv & 0x0400) != 0) dv -= 0x0800;
            a = (uint32_t)((dv < 0) ? -dv : dv);
            (void)snprintf(tmp, sizeof tmp, "TF %s %c%u.%uV", (vd == 1U) ? "OK" : ((vd == 2U) ? "FLIP" : "?"),
                           (dv < 0) ? '-' : '+', (unsigned)(a / 10U), (unsigned)(a % 10U));
            for (k = 0U; k < MON_LCD_COLS; k++) rows[3][k] = ' ';
            put_text(rows[3], 0U, tmp);
        }

        if ((s->state == 3U) && ((s->flags & 0x8000U) != 0U))
        {
            /* Slave in FAULT: row 3 shows the fault CONTEXT instead of the link counters:
             *   @<state> M:<zero-cross mode> FLT:<level> BK:<level>      (H = high/idle, L = low/asserted)
             * state: SYNC / CONV (converter) / INVS (inverter at zero duty) / UNSH (shunts opening) / RUN / RSHT (shunts closing),
             * mode: B both edges, F falling edges, U unknown. */
            const uint8_t ctx = (uint8_t)(s->flags >> 8);
            const uint8_t st  = (uint8_t)(ctx & 0x07U);
            const char *sn = (st == 1U) ? "SYNC" : ((st == 2U) ? "CONV" : ((st == 3U) ? "INVS" :
                             ((st == 4U) ? "UNSH" : ((st == 5U) ? "RUN" : ((st == 6U) ? "RSHT" : "?")))));
            const uint8_t md = (uint8_t)((ctx >> 3) & 0x03U);
            (void)snprintf(tmp, sizeof tmp, "@%-4s M:%c FLT:%c BK:%c", sn,
                           (md == 1U) ? 'B' : ((md == 2U) ? 'F' : 'U'),
                           ((ctx & 0x20U) != 0U) ? 'L' : 'H',
                           ((ctx & 0x40U) != 0U) ? 'L' : 'H');
            put_text(rows[3], 0U, tmp);

            /* Every other 2 s the same row shows WHERE in the mains half-cycle the last running FLT / break event hit:
             *   dt=<ms since the last zero-cross event> D<+|-|0> <ON|off>
             * D+ / D- = the positive / negative half diagonal was energised at that moment, D0 = none. dt about 0.05 ms = at turn-on,
             * about 9.7 ms = at turn-off, anything else = in the middle of the half-cycle (or noise). */
            if (((s->diag2 & 0x8000U) != 0U) && (((now_ms / 2000U) & 1U) != 0U))
            {
                const uint32_t us  = (uint32_t)(s->diag2 & 0x03FFU) * 16U;
                const char     pol = ((s->diag2 & 0x0400U) != 0U) ? '+' : (((s->diag2 & 0x0800U) != 0U) ? '-' : '0');
                uint32_t       k;
                (void)snprintf(tmp, sizeof tmp, "dt=%u.%02ums D%c %s", (unsigned)(us / 1000U), (unsigned)((us % 1000U) / 10U),
                               pol, (pol != '0') ? "ON" : "off");
                for (k = 0U; k < MON_LCD_COLS; k++) rows[3][k] = ' ';
                put_text(rows[3], 0U, tmp);
            }
        }
        else if ((s->state == 1U) && ((s->flags & 0x8000U) != 0U))
        {
            /* Slave waiting in SYNC: row 3 lists what it is waiting for (letters shown = NOT met yet):
             *   Z zero-cross pattern   V input voltage window   M measurement   F FLT low
             *   B break input low (also a FLOATING one)         O output voltage too high
             *   R relays not yet settled in the SHORTED (bypass) state
             * "WAIT: -" = nothing is missing, it is about to start. */
            const uint8_t wc = (uint8_t)(s->flags >> 8);
            static const char letters[7] = { 'Z', 'V', 'M', 'F', 'B', 'O', 'R' };
            uint32_t k, n;
            (void)snprintf(tmp, sizeof tmp, "WAIT:");
            n = 5U;
            for (k = 0U; k < 7U; k++)
            {
                if (((wc >> k) & 1U) != 0U) { tmp[n++] = ' '; tmp[n++] = letters[k]; }
            }
            if (n == 5U) { tmp[n++] = ' '; tmp[n++] = '-'; }
            tmp[n] = '\0';
            for (k = 0U; k < MON_LCD_COLS; k++) rows[3][k] = ' ';     /* the wait text is shorter than the normal row */
            put_text(rows[3], 0U, tmp);
        }
    }
    else
    {
        put_text(rows[0], 0U, "SLAVE R NO LINK");
        put_text(rows[1], 0U, "Vin  --- V");
        put_text(rows[2], 0U, "Vout --- V");
        /* Bytes arriving but no valid frame = baud / format / noise problem;
         * no bytes at all = wiring or the optocouplers. */
        (void)snprintf(tmp, sizeof tmp, "rx %lu B%u E%u",
                       (unsigned long)(m->bytes_rx % 1000000UL),
                       (unsigned)cap99(m->frames_bad), (unsigned)cap99(m->uart_errors));
        put_text(rows[3], 0U, tmp);
    }
}

/* ========================================================================= */
/* Changed-characters-only LCD writer (no hardware: the write is a callback) */
/* ========================================================================= */

static const uint8_t k_row_addr[MON_LCD_ROWS] = { 0x80U, 0xC0U, 0x94U, 0xD4U };   /* same as lcd.c */

void MonLcd_Init(MonLcd_t *l)
{
    uint32_t r;
    for (r = 0U; r < MON_LCD_ROWS; r++)
    {
        memset(l->have[r], ' ', MON_LCD_COLS);     /* LCD_Init() clears the display */
        l->have[r][MON_LCD_COLS] = '\0';
    }
    l->cur_row = -1;
    l->cur_col = -1;
    l->cursor  = 0U;
}

uint32_t MonLcd_Step(MonLcd_t *l, const char want[MON_LCD_ROWS][MON_LCD_COLS + 1U],
                     MonLcdWriteFn write)
{
    uint32_t cells = MON_LCD_ROWS * MON_LCD_COLS;
    uint32_t n;

    for (n = 0U; n < cells; n++)
    {
        uint32_t idx = (l->cursor + n) % cells;
        uint32_t row = idx / MON_LCD_COLS;
        uint32_t col = idx % MON_LCD_COLS;

        if (want[row][col] != l->have[row][col])
        {
            uint32_t written = 0U;

            if (((int32_t)row != l->cur_row) || ((int32_t)col != l->cur_col))
            {
                write(0U, (uint8_t)(k_row_addr[row] + col));      /* set DDRAM address */
                written++;
            }
            write(1U, (uint8_t)want[row][col]);
            written++;

            l->have[row][col] = want[row][col];
            l->cursor         = (idx + 1U) % cells;
            l->cur_row        = (int32_t)row;
            l->cur_col        = (int32_t)col + 1;
            if (l->cur_col >= (int32_t)MON_LCD_COLS)
            {
                l->cur_row = -1;                  /* wrapped past the row end: position unknown */
                l->cur_col = -1;
            }
            return written;
        }
    }
    return 0U;
}

/* ========================================================================= */
/* Hardware part (not built for the PC-side tests)                           */
/* ========================================================================= */
#ifndef MON_HOST_TEST

extern UART_HandleTypeDef huart1;
#if (MASTER_YB_LINKS == 1)
extern UART_HandleTypeDef huart3, huart4;                  /* B: USART3 (PB2 TX, PB0 RX), Y: USART4 (PC10 TX, PC11 RX) */
extern DMA_HandleTypeDef  hdma_usart3_rx, hdma_usart4_rx;
static volatile uint8_t s_rx_yb[MLINKS_N][MON_RXBUF_SIZE];                                       /* the DMA rings of the Y (0) and B (1) links */
static uint32_t s_rd_yb[MLINKS_N];
static UART_HandleTypeDef *const s_uart_yb[MLINKS_N] = { &huart4, &huart3 };
static DMA_HandleTypeDef  *const s_dma_yb[MLINKS_N]  = { &hdma_usart4_rx, &hdma_usart3_rx };
#endif
extern DMA_HandleTypeDef  hdma_usart1_rx;

static volatile uint8_t s_rx[MON_RXBUF_SIZE];

/* Free-running CPU-clock counter built from SysTick: 62.5 ns resolution at
 * 16 MHz, independent of every timer the old firmware uses. Differences are
 * exact modulo 2^32. */
static uint32_t cyc_now(void)
{
    uint32_t ms, v;
    uint32_t per_ms = SystemCoreClock / 1000U;
    do
    {
        ms = HAL_GetTick();
        v  = SysTick->VAL;
    } while (ms != HAL_GetTick());
    return (ms * per_ms) + ((per_ms - 1U) - v);
}

static void delay_us(uint32_t us)
{
    uint32_t t0 = cyc_now();
    uint32_t n  = us * (SystemCoreClock / 1000000U);
    while ((uint32_t)(cyc_now() - t0) < n) { }
}

/* ---- fast LCD byte write: pins as in lcd.h (RS PB4, EN PB5, D4..D7 PD0..PD3) ---- */
static void lcd_nibble(uint8_t n)
{
    uint32_t set = 0U, rst = 0U;
    if ((n & 1U) != 0U) set |= LCD_D4_PIN; else rst |= LCD_D4_PIN;
    if ((n & 2U) != 0U) set |= LCD_D5_PIN; else rst |= LCD_D5_PIN;
    if ((n & 4U) != 0U) set |= LCD_D6_PIN; else rst |= LCD_D6_PIN;
    if ((n & 8U) != 0U) set |= LCD_D7_PIN; else rst |= LCD_D7_PIN;
    GPIOD->BSRR = set | (rst << 16);
}

static void lcd_pulse(void)
{
    GPIOB->BSRR = (uint32_t)LCD_EN_PIN;
    delay_us(2U);
    GPIOB->BSRR = ((uint32_t)LCD_EN_PIN) << 16;
    delay_us(2U);
}

static void lcd_byte(uint8_t rs, uint8_t v)
{
    if (rs != 0U) GPIOB->BSRR = (uint32_t)LCD_RS_PIN;
    else          GPIOB->BSRR = ((uint32_t)LCD_RS_PIN) << 16;
    lcd_nibble((uint8_t)(v >> 4));
    lcd_pulse();
    lcd_nibble((uint8_t)(v & 0x0FU));
    lcd_pulse();
    delay_us(45U);                         /* HD44780 needs ~37 us per command/data byte */
}

/* ---- receive ---- */
static uint32_t s_rd;

static void rx_start(void)
{
    /* The original code leaves this DMA channel in NORMAL mode. Switch it to
     * CIRCULAR and start it by hand: no interrupts, no HAL receive state. */
    hdma_usart1_rx.Init.Mode = DMA_CIRCULAR;
    if (HAL_DMA_DeInit(&hdma_usart1_rx) != HAL_OK) Error_Handler();
    if (HAL_DMA_Init(&hdma_usart1_rx)   != HAL_OK) Error_Handler();
    if (HAL_DMA_Start(&hdma_usart1_rx, (uint32_t)&huart1.Instance->RDR,
                      (uint32_t)s_rx, MON_RXBUF_SIZE) != HAL_OK)
    {
        Error_Handler();
    }
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);      /* USART RX -> DMA requests */
    s_rd = 0U;
#if (MASTER_YB_LINKS == 1)
    {   /* the Y and B links, the same way: circular DMA into a ring, no interrupts, no HAL receive state */
        uint8_t i;
        MLinks_Init();
        for (i = 0U; i < MLINKS_N; i++)
        {
            s_dma_yb[i]->Init.Mode = DMA_CIRCULAR;
            if (HAL_DMA_DeInit(s_dma_yb[i]) != HAL_OK) Error_Handler();
            if (HAL_DMA_Init(s_dma_yb[i])   != HAL_OK) Error_Handler();
            if (HAL_DMA_Start(s_dma_yb[i], (uint32_t)&s_uart_yb[i]->Instance->RDR, (uint32_t)s_rx_yb[i], MON_RXBUF_SIZE) != HAL_OK) Error_Handler();
            SET_BIT(s_uart_yb[i]->Instance->CR3, USART_CR3_DMAR);
            s_rd_yb[i] = 0U;
        }
    }
#endif
}

static void rx_service(void)
{
    static uint32_t last_us_cyc;
    static uint8_t  have_last;
    uint32_t now_cyc = cyc_now();
    uint32_t wr, isr;

    /* How long did the loop go without polling? (must stay far below 12.8 ms) */
    if (have_last)
    {
        uint32_t gap_us = (uint32_t)(now_cyc - last_us_cyc) / (SystemCoreClock / 1000000U);
        if (gap_us > g_mon.max_poll_gap_us) g_mon.max_poll_gap_us = gap_us;
    }
    last_us_cyc = now_cyc;
    have_last   = 1U;

    /* Overrun / framing / noise: clear the flags so reception never stalls. */
    isr = huart1.Instance->ISR;
    if ((isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) != 0U)
    {
        huart1.Instance->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
        g_mon.uart_errors++;
    }

    /* Bytes the DMA has written so far: ring size minus the transfers left. */
    wr = (MON_RXBUF_SIZE - hdma_usart1_rx.Instance->CNDTR) & (MON_RXBUF_SIZE - 1U);
    Mon_ProcessBytes(s_rx, MON_RXBUF_SIZE, &s_rd, wr, HAL_GetTick());
    Mon_Tick(HAL_GetTick());
#if (MASTER_YB_LINKS == 1)
    {   uint8_t i;
        for (i = 0U; i < MLINKS_N; i++)
        {
            uint32_t yisr = s_uart_yb[i]->Instance->ISR, ywr;
            if ((yisr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) != 0U) s_uart_yb[i]->Instance->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;      /* never stall */
            ywr = (MON_RXBUF_SIZE - s_dma_yb[i]->Instance->CNDTR) & (MON_RXBUF_SIZE - 1U);
            MLinks_Feed(i, s_rx_yb[i], MON_RXBUF_SIZE, &s_rd_yb[i], ywr, HAL_GetTick());
        }
    }
#endif
}

void MasterMonitor_Run(void)
{
    static char want[MON_LCD_ROWS][MON_LCD_COLS + 1U];
    MonLcd_t    lcd;
    uint32_t    t_compose = 0U;

    /* The monitor never switches a contactor: force them to the OFF level
     * (RESET opens them in the original firmware) and leave them there. */
    HAL_GPIO_WritePin(GPIOA, CONT_1_Pin | CONT_2_Pin | CONT_3_Pin, GPIO_PIN_RESET);

    Mon_Reset();
    MonLcd_Init(&lcd);
    Mon_Compose(&g_mon, HAL_GetTick(), want);

    rx_start();

    for (;;)
    {
        rx_service();

        if ((uint32_t)(HAL_GetTick() - t_compose) >= MON_COMPOSE_PERIOD_MS)
        {
            t_compose = HAL_GetTick();
            Mon_Compose(&g_mon, t_compose, want);
        }

        (void)MonLcd_Step(&lcd, want, lcd_byte);     /* at most ~60-120 us */
    }
}


#if (MASTER_APP_MODE == 1)
#include "m_app.h"
#include "m_board.h"
#include "m_flash_g0.h"
#include "link_v2.h"

extern uint32_t ADC_Read(uint32_t channel);       /* the original firmware's single-sample ADC read (main.c) */

/* The CT, R phase: the average of MASTER_CT_SAMPLES readings (12 bit), times 16, for the application's calibration (zero and gain, in flash). */
#if (MASTER_RTC_ENABLED == 1)
static const MHoursIo_t s_hours_io = { MRtc_BkpRead, MRtc_BkpWrite };
#endif

static uint16_t ct_read_x16(void)
{
    uint32_t sum = 0U, i;
    for (i = 0U; i < MASTER_CT_SAMPLES; i++) sum += ADC_Read(ADC_CHANNEL_0);
    return (uint16_t)((sum * 16U) / MASTER_CT_SAMPLES);
}

static uint8_t key_down(GPIO_TypeDef *port, uint16_t pin) { return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1U : 0U; }     /* active low */

/* The application: Run-Mode screens, Set-Mode, settings in flash, the fault log. It shares everything of the monitor (the receive path, the LCD writer) and, like it, NEVER drives a contactor. */
void MasterApp_Run(void)
{
    static MApp_t app;
    static char   want[MON_LCD_ROWS][MON_LCD_COLS + 1U];
    MonLcd_t      lcd;
    uint32_t      t_step = 0U, t_tx = 0U, t_ct = 0U;
    uint16_t      ct_x16 = 0U, ct_fast_max = 0U;
    MDateTime_t   clk = { 0U, 0U, 0U, 0U, 0U }; uint8_t clk_ok = 0U, rtc_src = 0U; uint16_t rtc_bd = 0U, rtc_ic = 0U; uint32_t t_clk = 0U;      /* the real-time clock as last read, and when */
    uint8_t       seq_tx = 0U, have_last = 0U; uint32_t r_seen = 0U;
    MCtlOut_t     last;

    HAL_GPIO_WritePin(GPIOA, CONT_1_Pin | CONT_2_Pin | CONT_3_Pin, GPIO_PIN_RESET);
    Mon_Reset();
    g_mon.rst_wdg = (uint8_t)(((RCC->CSR & RCC_CSR_IWDGRSTF) != 0U) ? 1U : 0U); RCC->CSR |= RCC_CSR_RMVF;        /* why this start: read once, then clear the flags */
    MonLcd_Init(&lcd);
    last.run = 0U; last.clr = 0U; last.setv_dv = 0U; last.maxc_dv = 0U;
    MApp_Init(&app, MFlashG0_Iface(), MAPP_PAGE_SETTINGS_A, MAPP_PAGE_SETTINGS_B, MAPP_PAGE_LOG_A, MAPP_PAGE_LOG_B, HAL_GetTick());
#if (MASTER_HEAT_INPUT == 1)
    {   GPIO_InitTypeDef hg = {0}; __HAL_RCC_GPIOA_CLK_ENABLE();                      /* PA5: a plain input; the board has the pull-up */
        hg.Pin = MHEAT_PIN; hg.Mode = GPIO_MODE_INPUT; hg.Pull = GPIO_NOPULL; HAL_GPIO_Init(MHEAT_PORT, &hg); }
#endif
#if (MASTER_RTC_ENABLED == 1)
    MRtc_Init();                                                            /* the real-time clock: crystal + battery; a running clock is left alone */
#endif
    MApp_InitCal(&app, MAPP_PAGE_CALIB_A, MAPP_PAGE_CALIB_B);
#if (MASTER_RTC_ENABLED == 1)
    MApp_SetRunMinutes(&app, MHours_Load(&s_hours_io));                          /* the running hours from the battery-backed registers (0 without a battery) */
#endif
    MApp_BootReset(&app, g_mon.rst_wdg);                                         /* a restart by the watchdog is logged */
    MApp_SetTestNoCuts(&app, MASTER_TEST_NO_VOLTAGE_CUTS);
    MApp_SetInrushRetries(&app, MASTER_INRUSH_RETRIES);
    MApp_SetNtcHeat(&app, (uint8_t)MASTER_NTC_HEAT);                              /* the heat-sink temperature trips the unit (the Overheat Temp in Set-Mode) */
    rx_start();
#if (MASTER_TEST_NO_VOLTAGE_CUTS != 0)
    MUI_Notify(&app.ui, "TEST: NO V CUTS", "Not For Field Use", HAL_GetTick());                  /* every start of a build with the voltage cuts removed says so */
#endif
    if (MFlashG0_ImageEnd() > MFLASHG0_LOW) MUI_Notify(&app.ui, "Flash Full", "Settings Not Saved", HAL_GetTick());     /* the program reaches the storage pages: saving is refused, see MASTER_UI.md */

#if (MASTER_WATCHDOG != 0)
    (void)MasterIwdg_Start(MASTER_WATCHDOG_MS);        /* after ALL the slow initialisation; it cannot be stopped again */
#endif
    for (;;)
    {
        uint32_t now;
#if (MASTER_WATCHDOG != 0)
        MasterIwdg_Kick();                             /* once per pass of the main loop, never from an interrupt */
#endif
        rx_service();
#if (MASTER_CT_PEAK == 1)
        {   uint16_t cs = (uint16_t)(ADC_Read(ADC_CHANNEL_0) * 16U); if (cs > ct_fast_max) ct_fast_max = cs; }      /* every pass: the largest reading of the 10 ms step */
#endif
        now = HAL_GetTick();
        if ((uint32_t)(now - t_step) >= 10U)
        {
            MAppData_t d;
            t_step = now;
            Mon_Tick(now);
            d.have     = (uint8_t)((g_mon.have_frame != 0U) && ((uint32_t)(now - g_mon.t_last_ms) < MON_LINK_TIMEOUT_MS));
            d.vin_dv   = g_mon.last.vin_dv;
            d.vout_dv  = g_mon.last.vout_dv;
            d.freq_chz = g_mon.last.freq_chz;
            d.state    = g_mon.last.state;
            d.fault    = (uint8_t)g_mon.last.fault;
            d.link_ok  = d.have;
            {   /* the slave's version 2 data: $E must be fresh (the slave sends one every 100 ms) */
                uint8_t e_fresh = (uint8_t)((g_mon.e_valid != 0U) && ((uint32_t)(now - g_mon.e_t_ms) < MON_LINK_TIMEOUT_MS));
                d.e_valid = g_mon.e_valid; d.clr_echo = g_mon.e_clr; d.e_setv_dv = g_mon.e_setv_dv;
                d.iout_rms_ca = g_mon.e_iout_rms_ca; d.iout_pk_ca = g_mon.e_iout_pk_ca;
                d.prof_tuned = g_mon.h_tuned; d.prof_cap = g_mon.h_prof_cap; d.prof_kib = g_mon.h_prof_kib; d.prof_kiu = g_mon.h_prof_kiu; d.prof_vref_dv = g_mon.h_prof_vref_dv; d.prof_maxc_dv = g_mon.h_prof_maxc_dv; d.prof_n = g_mon.h_prof_n; d.prof_knee = g_mon.h_prof_knee;
                d.tune_n = g_mon.t_n; d.tune_step = g_mon.t_step; d.tune_result = g_mon.t_result; d.tune_ticks = g_mon.t_ticks; d.tune_vin_dv = g_mon.t_vin_dv; d.tune_dv_dv = g_mon.t_dv_dv;
                d.iout_valid = (uint8_t)(e_fresh && ((g_mon.e_flags2 & LINKV2_FLAG_IOUT_VALID) != 0U)); d.iout_clip = (uint8_t)(e_fresh && ((g_mon.e_flags2 & LINKV2_FLAG_IOUT_CLIP) != 0U));
                d.in_band = (uint8_t)(e_fresh && ((g_mon.e_flags2 & LINKV2_FLAG_IN_BAND) != 0U));
                d.heat = 0U; d.ct_peak = 0U;
                d.ntc_raw = g_mon.last.ntc_raw; d.ntc_valid = d.have;
                if (g_mon.e_valid != 0U) { d.sq_id = g_mon.e_sq_id; d.sq_delay_us = g_mon.e_sq_delay_us; d.sq_period_us = g_mon.e_sq_period_us; } else { d.sq_id = 0U; d.sq_delay_us = 0U; d.sq_period_us = 0U; }      /* the R slave's last phase sequence round */                       /* the R slave's heat-sink NTC, in every $S frame */
                d.wait_valid = (uint8_t)((g_mon.last.state == 1U) && ((g_mon.last.flags & 0x8000U) != 0U));       /* the slave waits in SYNC and says what for */
                d.wait_ctx   = (uint8_t)((g_mon.last.flags >> 8) & 0x7FU);
                d.h_valid = g_mon.h_valid; d.win_lo_dv = g_mon.h_win_lo_dv; d.win_hi_dv = g_mon.h_win_hi_dv; d.vout_max_dv = g_mon.h_vout_max_dv;
            }
            if ((uint32_t)(now - t_ct) >= 200U)
            {   /* the load current, every 200 ms (the averaging takes a few milliseconds: the receive DMA ring has 12.8 ms and the slave's frames come every 10 ms, so nothing is lost) */
                t_ct = now; ct_x16 = ct_read_x16();
            }
#if (MASTER_CT_PEAK == 1)
            d.ct_max_x16 = ct_fast_max; ct_fast_max = 0U;
#else
            d.ct_max_x16 = 0U;
#endif
            d.ct_x16 = ct_x16; d.ct_raw = (uint8_t)(t_ct != 0U); d.load_da = 0U; d.load_ok = 0U;
#if (MASTER_RTC_ENABLED == 1)
            if (((uint32_t)(now - t_clk) >= 1000U) || (t_clk == 0U)) { t_clk = now; clk_ok = MRtc_Get(&clk); MRtc_Status(&rtc_src, &rtc_bd, &rtc_ic); }        /* about once a second */
#endif
#if (MASTER_HEAT_INPUT == 1)
            d.heat = key_down(MHEAT_PORT, MHEAT_PIN);                                  /* PA5 low = overheat */
#endif
#if (MASTER_RTC_ENABLED == 1)
            if (MApp_TakeRunDirty(&app) != 0U) MHours_Save(&s_hours_io, MApp_RunMinutes(&app));      /* once a minute while the output is on */
#endif
            d.slave_wdg = (uint8_t)((g_mon.h_valid != 0U) && (g_mon.h_rst == 1U));       /* the slave's last reset was its watchdog */
#if (MASTER_YB_LINKS == 1)
            { uint8_t yb; for (yb = 0U; yb < MLINKS_N; yb++) MLinks_Fill(yb, &d.yb[yb], now); }          /* the Y and B slaves (all zero while nothing is heard on their cables) */
#else
            memset(&d.yb, 0, sizeof d.yb);
#endif
            d.clock = clk; d.clock_ok = clk_ok; d.rtc_src = rtc_src; d.rtc_bdcr = rtc_bd; d.rtc_icsr = rtc_ic;
            MApp_Step(&app, key_down(MKEY_MENU_PORT, MKEY_MENU_PIN), key_down(MKEY_START_PORT, MKEY_START_PIN), key_down(MKEY_STOP_PORT, MKEY_STOP_PIN),
                      key_down(MKEY_LOG_PORT, MKEY_LOG_PIN), &d, now, want);
#if (MASTER_RTC_ENABLED == 1)
            {   MDateTime_t nt; if (MApp_TakeClockSet(&app, &nt) != 0U) { MRtc_Set(&nt); clk = nt; clk_ok = 1U; t_clk = now; } }      /* a Date or Time saved in Set-Mode */
#endif
#if (MASTER_YB_LINKS == 1)
            {   /* the phase sequence measurement (v0.9.51): $K SEQ <round> on the three links, then ONE SYNC byte on the three links at the same moment: each slave times it against its own rising zero-cross */
                uint8_t sq_round, sq_act = MApp_TakeSeqAction(&app, &sq_round);
                if (sq_act == 1U)
                {
                    LinkK_t k; char sb[LINKV2_MAX_LEN + 8U]; int32_t sn; uint8_t yb3;
                    memset(&k, 0, sizeof k); k.id = MApp_NextKId(&app); k.cmd = LINKCMD_SEQ; k.num = (int32_t)sq_round;
                    sn = LinkV2_BuildK(sb, sizeof sb, &k); if (sn > 0) (void)HAL_UART_Transmit(&huart1, (uint8_t *)sb, (uint16_t)sn, 5U);
                    for (yb3 = 0U; yb3 < MLINKS_N; yb3++) { sn = MLinks_BuildSeq(yb3, sq_round, sb, sizeof sb); if (sn > 0) (void)HAL_UART_Transmit(s_uart_yb[yb3], (uint8_t *)sb, (uint16_t)sn, 5U); }
                }
                else if (sq_act == 2U)
                {   /* wait until the three transmitters are free, then write the three data registers back to back: the bytes start within about a microsecond of each other */
                    uint8_t sy = (uint8_t)LINKV2_SYNC_BYTE; USART_TypeDef *u0 = huart1.Instance, *u1 = s_uart_yb[0]->Instance, *u2 = s_uart_yb[1]->Instance; uint32_t guard = 20000U;
                    while ((guard > 0U) && ((u0->ISR & u1->ISR & u2->ISR & USART_ISR_TXE_TXFNF) == 0U)) guard--;
                    u0->TDR = sy; u1->TDR = sy; u2->TDR = sy;
                }
            }
#else
            {   uint8_t sq_round; (void)MApp_TakeSeqAction(&app, &sq_round); }                          /* no Y and B links: no measurement */
#endif
            if (MApp_ShowsMonitor(&app) != 0U)
            {   /* the monitor diagnostics, one MENU press away. In Industrial and Domestic mode it shows the R, Y and B links in turn, 3 s each: row 0 names the phase letter the slave on that cable reports (a Y board on the B cable says "SLAVE Y" in the B turn). */
                const MasterMon_t *mm = &g_mon;
#if (MASTER_YB_LINKS == 1)
                if (app.settings.mode != (uint8_t)M_MODE_SINGLE) { uint32_t sel = (now / 3000U) % 3U; if (sel == 1U) mm = MLinks_Mon(0U); else if (sel == 2U) mm = MLinks_Mon(1U); }
#endif
                Mon_Compose(mm, now, want);
            }
            if (MApp_ShowsDiag(&app) != 0U) MApp_Diag(&app, now, want);
            if (MApp_ShowsTune(&app) != 0U) MApp_TuneScreen(&app, now, want);          /* the transformer tuning screen over everything */
            if (MApp_ShowsIout(&app) != 0U) MApp_IoutPage(&app, now, want);
            if (MApp_ShowsProfile(&app) != 0U) MApp_ProfilePage(&app, now, want);            /* the converter-side current, LOG then MENU x7 */
            if (MApp_ShowsClock(&app) != 0U) MApp_ClockPage(&app, now, want);          /* the Clock page, LOG then MENU x4 */               /* the control diagnostics, two MENU presses away */
            {   /* tell the slave what to do: a $M frame every 100 ms, and AT ONCE when run, the set voltage, the Max Correction or the clear counter changes (Set-Mode = run 0 = the slave stops its PWM) */
                const MCtlOut_t *c = MApp_Control(&app);
                if ((MApp_ControlValid(&app) != 0U) && ((have_last == 0U) || (c->run != last.run) || (c->clr != last.clr) || (c->setv_dv != last.setv_dv) || (c->maxc_dv != last.maxc_dv) || ((uint32_t)(now - t_tx) >= 100U)))
                {
                    LinkM_t m; char b[LINKV2_MAX_LEN + 8U]; int32_t n;
                    m.seq = seq_tx++; m.run = c->run; m.setv_dv = c->setv_dv; m.clr = c->clr; m.maxc_dv = c->maxc_dv;
                    n = LinkV2_BuildM(b, sizeof b, &m);
                    if (n > 0) (void)HAL_UART_Transmit(&huart1, (uint8_t *)b, (uint16_t)n, 5U);      /* about 0.6 ms at 500 kbaud; the receive DMA keeps running */
                    MApp_NoteFrameSent(&app); t_tx = now; last = *c; have_last = 1U;
                }
            }
#if (MASTER_YB_LINKS == 1)
            {   /* the Y and B slaves: their own $M (run level, clear counter), only once heard, and the hello request. About 0.6 ms per frame. */
                uint8_t yb;
                for (yb = 0U; yb < MLINKS_N; yb++)
                {
                    char bb[LINKV2_MAX_LEN + 8U]; int32_t nn;
                    nn = MLinks_BuildM(yb, MApp_ControlPh(&app, (uint8_t)(1U + yb)), MApp_ControlValidPh(&app, (uint8_t)(1U + yb)), now, bb, sizeof bb);
                    if (nn > 0) (void)HAL_UART_Transmit(s_uart_yb[yb], (uint8_t *)bb, (uint16_t)nn, 5U);
                    if (MLinks_HelloDue(yb, now) != 0U)
                    {
                        nn = MLinks_BuildHello(yb, bb, sizeof bb);
                        if (nn > 0) (void)HAL_UART_Transmit(s_uart_yb[yb], (uint8_t *)bb, (uint16_t)nn, 5U);
                    }
                }
            }
#endif
            if (MApp_HelloDue(&app, now) != 0U)
            {   /* the slave says hello only at ITS power-up: if this master started later, ask for the slave's limits ($K HELLO) */
                LinkK_t k; char b[LINKV2_MAX_LEN + 8U]; int32_t n;
                k.id = MApp_NextKId(&app); k.cmd = LINKCMD_HELLO; k.num = 0; k.letter = '\0'; k.name[0] = '\0';
                n = LinkV2_BuildK(b, sizeof b, &k);
                if (n > 0) (void)HAL_UART_Transmit(&huart1, (uint8_t *)b, (uint16_t)n, 5U);
            }
            {   /* a voltage calibration command (and its retries, same id) */
                MCalK_t ck;
                if (MApp_TakeKCommand(&app, &ck, now) != 0U)
                {
                    LinkK_t k; char b[LINKV2_MAX_LEN + 8U]; int32_t n;
                    k.id = ck.id; k.cmd = (ck.cmd == MAPP_KCMD_CALV) ? LINKCMD_CALV : ((ck.cmd == MAPP_KCMD_CALO) ? LINKCMD_CALO : ((ck.cmd == MAPP_KCMD_TUNE) ? LINKCMD_TUNE : ((ck.cmd == MAPP_KCMD_TABORT) ? LINKCMD_TABORT : LINKCMD_CALR))); k.num = ck.num; k.letter = ck.letter; k.name[0] = '\0';
                    n = LinkV2_BuildK(b, sizeof b, &k);
                    if (n > 0) (void)HAL_UART_Transmit(&huart1, (uint8_t *)b, (uint16_t)n, 5U);
                }
                if (g_mon.r_n != r_seen) { r_seen = g_mon.r_n; MApp_OnKResult(&app, g_mon.r_id, g_mon.r_status, g_mon.r_value); }
            }
            {   const MFOut_t *eo = MApp_Engine(&app);
#if (MASTER_DRIVE_CONTACTORS == 1)
                /* closed only while the engine says ON; opened by every trip, Set-Mode, STOP and a lost link. PA10..PA12 high = closed (as in the original firmware). */
                HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, (eo->contactor[0] != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, CONT_2_Pin, (eo->contactor[1] != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOA, CONT_3_Pin, (eo->contactor[2] != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (eo->buzzer != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);          /* the buzzer: PB14 high = on */
#else
                HAL_GPIO_WritePin(GPIOA, CONT_1_Pin | CONT_2_Pin | CONT_3_Pin, GPIO_PIN_RESET);                   /* test mode: never closed */
                (void)eo;
#endif
            }
        }
        (void)MonLcd_Step(&lcd, want, lcd_byte);        /* at most ~60-120 us */
    }
}
#endif /* MASTER_APP_MODE */

#endif /* MON_HOST_TEST */
