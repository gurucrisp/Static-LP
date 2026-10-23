/**
  * @file    slave_iout.h
  * @brief   The CONVERTER-SIDE CURRENT (net I-OUT, connector pin 32, MCU pin PA2 = ADC1_IN2) from the power board's CT2500 sense chain. PURE LOGIC: counts in, amperes out.
  *
  *          THE CIRCUIT (schematic 22122016-EB, read from the drawing; confirm on the board): the CT2500 secondary (2500 turns) works into three 180 ohm resistors in parallel (R42, R43, R44 = 60 ohm). A differential stage (U8C, MCP6004)
  *          with R46 = 1k, R47 = 6k8 (gain 6.8) amplifies the burden voltage around a 1.65 V midpoint (VCC/2, from R37 .. R40). So   V(I-OUT) = 1.65 V +- 6.8 x 60 ohm x I_secondary,   I_secondary = I_primary / 2500,
  *          i.e. 0.1632 V per ampere of CT primary current. The output cannot go beyond the rails: it clips at about +-10.1 A peak (7.1 A RMS). Full load of the original board is 5.89 A RMS (8.33 A peak, 1.36 V of swing).
  *          The hardware short-circuit comparators (U10, U14) trip at about +-0.98 V across the burden, about +-41 A peak: far above what this channel can show.
  *
  *          WHAT IT MEASURES: the current in the conductor that passes through the CT. The drawing does not show which one. From the net names (PRI-PHASE, PRI-NTRL) and the full-load figure it looks like the converter side of the
  *          series transformer: the load current is then this current times the transformer's turns ratio (unknown here: it differs per transformer). Until that is known the value is the PRIMARY current.
  */
#ifndef SLAVE_IOUT_H
#define SLAVE_IOUT_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define IOUT_VREF_V          3.3f
#define IOUT_ADC_FULL        4095.0f
#define IOUT_CT_TURNS        2500.0f           /* CT2500 */
#define IOUT_BURDEN_OHM      60.0f             /* 180 ohm / 3 */
#define IOUT_GAIN            6.8f              /* R47 / R46 */
#define IOUT_V_PER_A         ((IOUT_GAIN * IOUT_BURDEN_OHM) / IOUT_CT_TURNS)                 /* 0.1632 V per ampere of primary current */
#define IOUT_A_PER_COUNT     ((IOUT_VREF_V / IOUT_ADC_FULL) / IOUT_V_PER_A)                  /* about 4.94 mA per ADC count */
#ifndef IOUT_CAL
#define IOUT_CAL             1.0f              /* field calibration factor (1.0 = nominal): the measured current is multiplied by it */
#endif
#define IOUT_CLIP_HI         4090U             /* the amplifier output is at a rail */
#define IOUT_CLIP_LO         5U

typedef struct
{
    float   irms_a;       /* AC RMS of the current, amperes (the DC offset is removed) */
    float   ipk_a;        /* the largest excursion from the mean in the window, amperes */
    uint8_t clipped;      /* 1 = the output touched a rail: the true current is larger than shown */
} IoutResult_t;

/* From the window statistics of the ADC channel (slave_meas's ComputeStats): the AC-RMS in counts, the mean, the extremes. */
void     Iout_Compute(float rms_counts, float mean_counts, uint16_t min_counts, uint16_t max_counts, IoutResult_t *r);
uint16_t Iout_ToCentiAmps(float amps);       /* hundredths of an ampere, 0..4000 (the $E field range) */

#ifdef __cplusplus
}
#endif
#endif
