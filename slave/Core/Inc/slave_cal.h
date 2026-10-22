/**
  * @file    slave_cal.h
  * @brief   Voltage calibration of the slave (input and output channel): single-point gain, from the meter reading the master sends with $K CALV / CALO.
  *          PURE LOGIC: the measurement, the controller state and the flash are reached through SlaveCalHw_t, so the PC tests can run it against a simulated measurement and a flash that loses power.
  *
  *          Only while the unit is STOPPED (no pulses, shunted): a gain change while regulating would make the output jump by the same percentage. The slave averages the raw reading for 1 s, computes
  *          total correction = meter / (raw reading x nominal gain), accepts it only if the reading is at least 50 V, steady, and the correction is within +-10 % of nominal, applies it and stores it
  *          (two CRC-protected copies; a failed store puts the old value back). The correction is kept as an integer ratio in 0.01 % against the NOMINAL gain (10000 = nominal).
  */
#ifndef SLAVE_CAL_H
#define SLAVE_CAL_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define SCAL_MIN_READING_V    50.0f        /* the reading must be at least this (the master also refuses below it)                              */
#define SCAL_RATIO_MIN        9000U        /* total correction accepted: 0.90 ... 1.10 of nominal, in 0.01 %                                    */
#define SCAL_RATIO_MAX        11000U
#define SCAL_AVG_MS           1000UL       /* the raw reading is averaged this long                                                              */
#define SCAL_MIN_SAMPLES      20U          /* fewer new results than this in the window: no usable signal                                        */
#define SCAL_SPREAD_PCT       2.0f         /* max - min over the window, as % of the average: more means the input is not steady                */

typedef struct
{
    float    (*raw_counts)(uint8_t ch, uint8_t *valid);   /* the latest raw AC-RMS result of channel 0 (input) / 1 (output), and whether it is valid      */
    uint32_t (*update_count)(void);                       /* increments with every new RMS result (every 10 ms)                                            */
    uint8_t  (*is_stopped)(void);                         /* 1 = the controller is OFF: no pulses, shunted                                                 */
    float    (*nominal_gain)(uint8_t ch);                 /* volts per count of the NOMINAL (uncalibrated) chain                                            */
    void     (*set_ratio)(uint8_t ch, uint16_t ratio);    /* apply: gain = nominal x ratio / 10000                                                         */
    int      (*store_load)(uint16_t ratio[2]);            /* 0 = a valid stored record was read into ratio[0..1]                                           */
    int      (*store_save)(const uint16_t ratio[2]);      /* 0 = stored; else a POSITIVE diagnostic code that is passed to the master in the answer's value (FLASH_FAIL)              */
} SlaveCalHw_t;

void     SlaveCal_Init(const SlaveCalHw_t *hw);            /* load the stored ratios (if any) and apply them */
/* cmd: 1 = CALV, 2 = CALO, 3 = CALR (letter 'V' or 'O'); num_dv = the meter reading in tenths of a volt (CALV / CALO).
 * Returns 1 when the answer is ready now (status / value), 0 when the averaging has started: call SlaveCal_Poll until it returns 1. */
int      SlaveCal_Command(uint8_t cmd, int32_t num_dv, char letter, uint8_t *status, int32_t *value, uint32_t now_ms);
int      SlaveCal_Poll(uint8_t *status, int32_t *value, uint32_t now_ms);
uint8_t  SlaveCal_Busy(void);
uint16_t SlaveCal_Ratio(uint8_t ch);                       /* the ratio in force, 10000 = nominal */

#define SCAL_CMD_CALV 1U
#define SCAL_CMD_CALO 2U
#define SCAL_CMD_CALR 3U

#ifdef __cplusplus
}
#endif
#endif
