/**
  * @file    slave_iout.h
  * @brief   The CONVERTER-SIDE CURRENT (net I-OUT, connector pin 32, MCU pin PA2 = ADC1_IN2) from the power board's CT2500 sense chain. PURE LOGIC: counts in, amperes out.
  *
  *          THE CIRCUIT (schematic 22122016-EB, read from the drawing, confirmed by the builder): the CT2500 secondary (2500 turns) works into THREE EQUAL RESISTORS IN PARALLEL (R42, R43, R44) that form the burden. The drawing has 180 ohm each (60 ohm
  *          in all); THIS BOARD HAS 68 ohm each (22.67 ohm in all): IOUT_BURDEN_R_EACH_OHM below, change it in slave_config.h if the resistors change. A differential stage (U8C, MCP6004) with R46 = 1k, R47 = 6k8 (gain 6.8) amplifies the burden
  *          voltage around a 1.65 V midpoint (VCC/2, from R37 .. R40). So   V(I-OUT) = 1.65 V +- 6.8 x R_burden x I_secondary,   I_secondary = I_primary / 2500:
  *                                  burden 22.67 ohm (68 x 3, this board):  0.0617 V per ampere, clips at +-26.8 A peak (18.9 A RMS), 13.1 mA per ADC count
  *                                  burden 60 ohm (180 x 3, the drawing):  0.1632 V per ampere, clips at +-10.1 A peak (7.1 A RMS),  4.9 mA per ADC count
  *          The hardware short-circuit comparators (U10, U14) have FIXED thresholds of about +-0.98 V across the burden, so THEIR trip current moves with the burden: about +-41 A peak with 60 ohm, about +-108 A peak with 22.67 ohm.
  *
  *          WHAT IT MEASURES: the current in the conductor that passes through the CT. The drawing does not show which one. From the net names (PRI-PHASE, PRI-NTRL) and the full-load figure it looks like the converter side of the
  *          series transformer: the load current is then this current times the transformer's turns ratio (unknown here: it differs per transformer). Until that is known the value is the PRIMARY current.
  */
#ifndef SLAVE_IOUT_H
#define SLAVE_IOUT_H
#include <stdint.h>
#include "slave_features.h"          /* so that IOUT_BURDEN_R_EACH_OHM can be set in slave_config.h */
#ifdef __cplusplus
extern "C" {
#endif

#define IOUT_VREF_V          3.3f
#define IOUT_ADC_FULL        4095.0f
#define IOUT_CT_TURNS        2500.0f           /* CT2500 */
#ifndef IOUT_BURDEN_R_EACH_OHM
#define IOUT_BURDEN_R_EACH_OHM  68.0f          /* R42, R43, R44, each (180 on the drawing) */
#endif
#define IOUT_BURDEN_PARALLEL 3.0f
#define IOUT_BURDEN_OHM      (IOUT_BURDEN_R_EACH_OHM / IOUT_BURDEN_PARALLEL)
#define IOUT_GAIN            6.8f              /* R47 / R46 */
#define IOUT_V_PER_A         ((IOUT_GAIN * IOUT_BURDEN_OHM) / IOUT_CT_TURNS)                 /* 0.0617 V per ampere of primary current with 68 ohm x 3 (0.1632 with 180 ohm x 3) */
#define IOUT_A_PER_COUNT     ((IOUT_VREF_V / IOUT_ADC_FULL) / IOUT_V_PER_A)                  /* about 13.1 mA per ADC count with 68 ohm x 3 (4.94 mA with 180 ohm x 3) */
#ifndef IOUT_CAL
#define IOUT_RAIL_V          1.65f             /* the amplifier output swings this far each way around the midpoint before it clips */
#define IOUT_PK_LIMIT_A      (IOUT_RAIL_V / IOUT_V_PER_A)                                    /* the largest PEAK current the channel can show: 26.8 A (10.1 A with 180 ohm x 3) */
#define IOUT_TRIP_V          0.982f            /* the short-circuit comparators' threshold across the burden (R37 .. R40 divider) */
#define IOUT_TRIP_A          ((IOUT_TRIP_V / IOUT_BURDEN_OHM) * IOUT_CT_TURNS)               /* the hardware trip: about 108 A peak (41 A with 180 ohm x 3) */
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
