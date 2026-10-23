/**
  * @file    m_hours.h
  * @brief   The running-hours counter (minutes of output ON) kept in the battery-backed registers of the RTC (TAMP backup registers 0..4: the master has a battery on VBAT). PURE LOGIC: the registers are reached through
  *          MHoursIo_t, so the PC tests can lose power between any two writes.
  *
  *          Layout: reg 4 = a magic number; regs 0,1 = copy A (the value, the value XOR a constant); regs 2,3 = copy B (the same). A save rewrites the copy that is NOT the newest valid one first, then the other, so a power cut at any point leaves the newest valid value intact:
  *          a load takes the larger of the valid copies. Nothing valid (first start, no battery): 0. The counter is saved once a minute while the output is on: at most about a minute is lost at a power cut.
  */
#ifndef M_HOURS_H
#define M_HOURS_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define MHOURS_MAGIC  0x52554E31UL      /* "RUN1" */
#define MHOURS_XOR    0xA5C3D2E1UL
#define MHOURS_REGS   5U

typedef struct
{
    uint32_t (*rd)(uint8_t reg);
    void     (*wr)(uint8_t reg, uint32_t value);
} MHoursIo_t;

uint32_t MHours_Load(const MHoursIo_t *io);                      /* minutes; 0 when nothing valid is stored */
void     MHours_Save(const MHoursIo_t *io, uint32_t minutes);

#ifdef __cplusplus
}
#endif
#endif
