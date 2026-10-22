/**
  * @file    slave_cal_hw.h
  * @brief   Binds the calibration logic (slave_cal.h) to the real measurement chain (g_meas, g_meas_cal), the controller state and the flash store. Only with SLAVE_VOLTAGE_CAL.
  */
#ifndef SLAVE_CAL_HW_H
#define SLAVE_CAL_HW_H
#ifdef __cplusplus
extern "C" {
#endif
void SlaveCalHw_Init(void);       /* once, after SlaveMeas_Init(): loads the stored calibration and applies it to the measurement */
#ifdef __cplusplus
}
#endif
#endif
