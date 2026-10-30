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
        if ((m->e_valid != 0U) && ((m->e_lp_ms != 0U) || (m->e_flt_raw != 0U) || (m->e_zc_noise != 0U)))
        {   /* v0.9.70 diagnostics, the whole row:  P<longest loop pass ms>/<FLT irqs in it>/<zero-cross irqs in it> F<all FLT irqs> Z<zero-cross noise edges> C<limited pulses> */
            uint32_t k;
            (void)snprintf(tmp, sizeof tmp, "P%u/%u/%u F%u Z%u C%u", (unsigned)m->e_lp_ms, (unsigned)m->e_lp_flt, (unsigned)m->e_lp_zc,
                           (unsigned)m->e_flt_raw, (unsigned)m->e_zc_noise, (unsigned)m->e_cbc);
            for (k = 0U; k < MON_LCD_COLS; k++) rows[3][k] = ' ';
            put_text(rows[3], 0U, tmp);
            tmp[0] = '\0';
        }
        else if (((s->diag2 & 0x4000U) != 0U) && ((s->diag2 & 0x8000U) == 0U))
        {
            /* passive converter-mapping check (v0.9.70: after the diagnostics): M<agree>/<disagree> (see slave_ctrl.h; replaces the B / f statistics) */
            (void)snprintf(tmp, sizeof tmp, " M%u/%u L%u", (unsigned)(s->diag2 & 127U), (unsigned)((s->diag2 >> 7) & 127U),
                           (unsigned)cap99(m->lost));
        }
        else if ((m->e_valid != 0U) && (m->e_cbc != 0U))
        {   /* v0.9.68: the cycle-by-cycle limit has ridden through C pulses since the slave's power-up (the bad-frame and frame-rate counters make room) */
            (void)snprintf(tmp, sizeof tmp, " C%u L%u", (unsigned)m->e_cbc, (unsigned)cap99(m->lost));
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
