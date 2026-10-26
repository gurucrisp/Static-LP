/**
  ******************************************************************************
  * @file    slave_phase.c
  * @brief   The slave's half of the phase sequence measurement. See slave_phase.h. No hardware.
  ******************************************************************************
  */
#include "slave_phase.h"

enum { ST_IDLE = 0, ST_ARMED = 1, ST_WAITING = 2 };

static void (*s_lock)(void);
static void (*s_unlock)(void);
static volatile uint8_t  s_state;
static volatile uint8_t  s_round;
static volatile uint32_t s_arm_ms;
static volatile uint32_t s_sync_us;
static volatile uint32_t s_last_rise_us;
static volatile uint8_t  s_have_rise;
static volatile uint16_t s_period_us;
static volatile uint8_t  s_ver;                                 /* odd while the result is being written */
static volatile uint16_t s_res_id, s_res_delay, s_res_period;

void SlavePhase_Init(void (*lock)(void), void (*unlock)(void))
{
    s_lock = lock; s_unlock = unlock;
    s_state = ST_IDLE; s_round = 0U; s_arm_ms = 0U; s_sync_us = 0U; s_last_rise_us = 0U; s_have_rise = 0U; s_period_us = 0U;
    s_ver = 0U; s_res_id = 0U; s_res_delay = 0U; s_res_period = 0U;
}

static void lock(void)   { if (s_lock != 0) s_lock(); }
static void unlock(void) { if (s_unlock != 0) s_unlock(); }

static void publish(uint16_t id, uint16_t delay, uint16_t period)
{
    s_ver++; s_res_id = id; s_res_delay = delay; s_res_period = period; s_ver++;
}

void SlavePhase_Arm(uint8_t round, uint32_t now_ms)
{
    lock();
    s_round = round; s_arm_ms = now_ms; s_state = ST_ARMED;
    unlock();
}

void SlavePhase_OnSync(uint32_t now_us)
{
    if (s_state == ST_ARMED) { s_sync_us = now_us; s_state = ST_WAITING; }      /* a SYNC byte that was not asked for, or a second one, is ignored */
}

void SlavePhase_OnZc(uint32_t now_us, uint8_t rising)
{
    uint32_t d;
    if (rising == 0U) return;                                                     /* ONE reference edge per cycle: the rising one */
    if (s_have_rise != 0U)
    {
        d = (uint32_t)(now_us - s_last_rise_us);
        s_period_us = ((d >= SPH_PERIOD_MIN_US) && (d <= SPH_PERIOD_MAX_US)) ? (uint16_t)d : 0U;      /* an edge that was missed shows as two periods: not trusted */
    }
    s_last_rise_us = now_us; s_have_rise = 1U;
    if (s_state != ST_WAITING) return;
    if ((int32_t)(now_us - s_sync_us) < 0) return;                                /* this edge was BEFORE the SYNC byte (the interrupts ran in the other order): the next one counts */
    d = (uint32_t)(now_us - s_sync_us);
    if (d > SPH_MAX_DELAY_US) publish(s_round, (uint16_t)SPH_NONE, s_period_us);
    else                      publish(s_round, (uint16_t)d, s_period_us);
    s_state = ST_IDLE;
}

void SlavePhase_Service(uint32_t now_ms)
{
    lock();
    if ((s_state == ST_ARMED) && ((uint32_t)(now_ms - s_arm_ms) >= SPH_ARM_WINDOW_MS)) s_state = ST_IDLE;               /* no SYNC came: nothing to report, the master gives up on its own */
    else if ((s_state == ST_WAITING) && ((uint32_t)(now_ms - s_arm_ms) >= SPH_FAIL_AFTER_MS)) { publish(s_round, (uint16_t)SPH_NONE, s_period_us); s_state = ST_IDLE; }
    unlock();
}

void SlavePhase_Result(uint16_t *id, uint16_t *delay_us, uint16_t *period_us)
{
    uint32_t tries;
    for (tries = 0U; tries < 4U; tries++)
    {
        uint8_t v1 = s_ver, v2;
        *id = s_res_id; *delay_us = s_res_delay; *period_us = s_res_period;
        v2 = s_ver;
        if (((v1 & 1U) == 0U) && (v1 == v2)) return;                              /* a consistent read: not torn by the interrupt writing it */
    }
}
