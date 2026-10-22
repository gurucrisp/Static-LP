/** @file slave_cal_hw.c  @brief Binding of the calibration logic to the measurement, the controller and the flash. See slave_cal_hw.h. */
#include "slave_features.h"
#if (SLAVE_VOLTAGE_CAL != 0)
#include "slave_cal.h"
#include "slave_cal_hw.h"
#include "slave_flash.h"
#include "slave_meas.h"
#include "slave_ctrl.h"

static SStore_t s_store;

static float hw_raw(uint8_t ch, uint8_t *valid) { *valid = (uint8_t)(g_meas.valid != 0U); return (ch == 0U) ? g_meas.rms_in_counts : g_meas.rms_out_counts; }
static uint32_t hw_count(void) { return g_meas.update_count; }
static uint8_t hw_stopped(void) { return (uint8_t)(g_ctrl.state == CTRL_OFF); }
static float hw_nominal(uint8_t ch) { return MEAS_NOMINAL_V_PER_COUNT * ((ch == 0U) ? MEAS_CAL_IN_A : MEAS_CAL_OUT_A); }
static void hw_set(uint8_t ch, uint16_t ratio)
{
    g_meas_cal[ch].gain_v_per_count = hw_nominal(ch) * ((float)ratio / 10000.0f);          /* the offset stays what the build says */
    g_meas_cal[ch].calibrated = (uint8_t)((ratio != 10000U) ? 1U : g_meas_cal[ch].calibrated);
}
static int hw_load(uint16_t r[2]) { uint8_t b[4]; if (SStore_Load(&s_store, b) == 0U) return -1; r[0] = (uint16_t)(b[0] | (b[1] << 8)); r[1] = (uint16_t)(b[2] | (b[3] << 8)); return 0; }
static int hw_save(const uint16_t r[2])
{
    uint8_t b[4]; int st;
    b[0] = (uint8_t)r[0]; b[1] = (uint8_t)(r[0] >> 8); b[2] = (uint8_t)r[1]; b[3] = (uint8_t)(r[1] >> 8);
    st = SStore_Save(&s_store, b);                                            /* 0 = ok, 1 = the erase failed, 2 = the record, 3 = the commit word */
    if (st == 0) return 0;
    return (int)(((uint32_t)st * 10000000UL) + SlaveFlash_Diag());            /* stage x 10,000,000 + the driver's reason / flash mode / flags (see slave_flash.h) */
}

static const SlaveCalHw_t HW = { hw_raw, hw_count, hw_stopped, hw_nominal, hw_set, hw_load, hw_save };

void SlaveCalHw_Init(void)
{
    SStore_Init(&s_store, &g_slave_flash, SFLASH_CAL_PAGE_A, SFLASH_CAL_PAGE_B, 1U, 4U);      /* version 1, a 4 byte payload: the two ratios */
    SlaveCal_Init(&HW);
}
#endif
