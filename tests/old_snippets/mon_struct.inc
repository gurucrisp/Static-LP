typedef struct
{
    LinkStatus_t last;               /* the latest valid status frame                 */
    uint8_t      have_frame;         /* at least one valid frame has arrived          */
    uint8_t      have_seq;
    uint8_t      last_seq;
    uint32_t     t_last_ms;          /* HAL_GetTick() when the last valid frame came  */

    uint32_t     frames_ok;          /* valid frames                                  */
    uint32_t     frames_bad;         /* complete lines rejected (checksum/format)     */
    uint32_t     overflows;          /* over-long lines                               */
    uint32_t     lost;               /* frames missing, from gaps in the seq number   */
    uint32_t     bytes_rx;           /* every byte the DMA delivered                  */
    uint32_t     uart_errors;        /* overrun / framing / noise flags cleared       */
    uint32_t     fps;                /* valid frames in the last whole second         */
    uint32_t     max_poll_gap_us;    /* longest time the loop went without polling    */
    uint8_t      e_valid;
    uint16_t     e_flags2;   /* v0.9.79: NOBOR */            /* v0.9.68: the fields the live Mon_Compose reads for the cycle-by-cycle count */
    uint16_t     e_cbc;
    uint16_t     e_lp_ms, e_lp_flt, e_lp_zc, e_flt_raw, e_zc_noise;   /* v0.9.70 */
} MasterMon_t;
