/**
  ******************************************************************************
  * @file    slave_relay.c
  * @brief   SH-RLY / W-RLY bypass-relay driver (see slave_relay.h for the rules).
  ******************************************************************************
  */
#include "slave_relay.h"
#include "slave_ctrl_hw.h"
#include "slave_power.h"

#if (SLAVE_RELAYS_MODE == 1)

/* Coil level that makes a relay's contact SHORT / NOT short the primary. A relay that is not a shunt keeps its coil off. */
#define COIL_SHORT(type)  (((type) == RELAY_SHORTS_WHEN_ENERGISED) ? 1U : 0U)
#define COIL_FREE(type)   (((type) == RELAY_SHORTS_WHEN_RELEASED)  ? 1U : 0U)

static volatile uint8_t s_state;
static uint8_t          s_want_free;
static uint32_t         s_t_cmd_ms;

static void DriveShorted(void)
{
    SlavePower_RelaySh(COIL_SHORT(SLAVE_RELAY_SH_CONTACT));
    SlavePower_RelayW (COIL_SHORT(SLAVE_RELAY_W_CONTACT));
}

static void DriveFree(void)
{
    SlavePower_RelaySh(COIL_FREE(SLAVE_RELAY_SH_CONTACT));
    SlavePower_RelayW (COIL_FREE(SLAVE_RELAY_W_CONTACT));
}

void SlaveRelay_Init(void)
{
    DriveShorted();
    s_want_free = 0U;
    s_state     = RLY_TO_SHORTED;
    s_t_cmd_ms  = CtrlHw_NowMs();
}

void SlaveRelay_ShortNow(void)
{
    DriveShorted();
    s_want_free = 0U;
    s_state     = RLY_TO_SHORTED;
    s_t_cmd_ms  = CtrlHw_NowMs();
}

void SlaveRelay_Request(uint8_t want_free)
{
    s_want_free = (uint8_t)(want_free != 0U);
}

uint8_t SlaveRelay_IsShorted(void) { return (uint8_t)(s_state == RLY_SHORTED); }
uint8_t SlaveRelay_UnwindDone(void){ return (uint8_t)(s_state == RLY_SHORTED); }
uint8_t SlaveRelay_IsFree(void)    { return (uint8_t)(s_state == RLY_FREE);    }
uint8_t SlaveRelay_State(void)     { return s_state; }
void    SlaveRelay_Hold(void)      { }

void SlaveRelay_Service(uint32_t now_ms)
{
    switch (s_state)
    {
    case RLY_SHORTED:
        if (s_want_free != 0U) { DriveFree(); s_state = RLY_TO_FREE; s_t_cmd_ms = now_ms; }
        break;

    case RLY_TO_SHORTED:
        if (s_want_free != 0U)                                   /* changed our mind while moving */
        {
            DriveFree(); s_state = RLY_TO_FREE; s_t_cmd_ms = now_ms;
        }
        else if ((int32_t)(now_ms - s_t_cmd_ms) >= (int32_t)SLAVE_RELAY_SETTLE_MS)
        {
            s_state = RLY_SHORTED;
        }
        break;

    case RLY_FREE:
        if (s_want_free == 0U) { DriveShorted(); s_state = RLY_TO_SHORTED; s_t_cmd_ms = now_ms; }
        break;

    case RLY_TO_FREE:
        if (s_want_free == 0U)
        {
            DriveShorted(); s_state = RLY_TO_SHORTED; s_t_cmd_ms = now_ms;
        }
        else if ((int32_t)(now_ms - s_t_cmd_ms) >= (int32_t)SLAVE_RELAY_SETTLE_MS)
        {
            s_state = RLY_FREE;
        }
        break;

    default:
        DriveShorted(); s_state = RLY_TO_SHORTED; s_t_cmd_ms = now_ms;
        break;
    }
}

#elif (SLAVE_RELAYS_MODE == 2)

/* ------------------------------------------------------------------------------------------------------------------
 * LEGACY: the old firmware's sequence. In this mode "shorted" and "free" both simply mean "the sequence is complete
 * and W-RLY is on", the state the converter must run in (the old firmware never ran it in any other state).
 *   Hold() -> wait for a trustworthy zero-cross time -> SH-RLY on, aimed (6 ms pull-in lead) at a mains zero-cross ->
 *   held 1 s -> SH off + W on in the same instant -> settle 1 s -> ON (converter may start). Fault: both coils off at once.
 * ------------------------------------------------------------------------------------------------------------------ */
#include "slave_ctrl.h"

typedef enum { LEG_OFF = 0, LEG_ARM, LEG_WAIT_CLOSE, LEG_SH_HOLD, LEG_SETTLE, LEG_ON } LegState_t;

static volatile uint8_t s_leg;
static uint32_t         s_t_close_us;
static uint32_t         s_t_ms;
static uint32_t         s_keep_ms;
static uint32_t         s_t_off_ms;           /* when both coils were last released */

static void LegOff(void)
{
    SlavePower_RelaySh(0U);
    SlavePower_RelayW(0U);
    s_leg = LEG_OFF;
    s_t_off_ms = CtrlHw_NowMs();
}

void SlaveRelay_Init(void)                  { LegOff(); s_keep_ms = CtrlHw_NowMs(); }
void SlaveRelay_Request(uint8_t want_free)  { (void)want_free; }
void SlaveRelay_ShortNow(void)              { LegOff(); }                       /* a fault: BOTH coils released at once */
uint8_t SlaveRelay_IsShorted(void)          { return (uint8_t)(s_leg == LEG_ON); }
uint8_t SlaveRelay_IsFree(void)             { return (uint8_t)(s_leg == LEG_ON); }
uint8_t SlaveRelay_State(void)              { return (uint8_t)((s_leg == LEG_ON) ? RLY_SHORTED : RLY_TO_SHORTED); }
/* both coils released AND long enough ago for the normally-closed W-RLY contact to have closed the primary again */
uint8_t SlaveRelay_UnwindDone(void)         { return (uint8_t)((s_leg == LEG_OFF) && ((int32_t)(CtrlHw_NowMs() - s_t_off_ms) >= (int32_t)SLAVE_RELAY_SETTLE_MS)); }

void SlaveRelay_Hold(void)
{
    s_keep_ms = CtrlHw_NowMs();
    if (s_leg == LEG_OFF) s_leg = LEG_ARM;
}

void SlaveRelay_Service(uint32_t now_ms)
{
    const uint32_t now_us = CtrlHw_NowUs();

    if (s_leg == LEG_OFF) return;

    /* nobody has wanted the relays for the release time (stopped, or the start conditions were lost): release both */
    if ((int32_t)(now_ms - s_keep_ms) >= (int32_t)SLAVE_LEGACY_RELEASE_MS) { LegOff(); return; }

    switch (s_leg)
    {
    case LEG_ARM:
    {
        const uint32_t iv  = g_ctrl.interval_us;                       /* half-cycle length, kept current by the zero-cross interrupt */
        const int32_t  age = (int32_t)(now_us - g_ctrl.t_event_us);    /* signed: the interrupt may have fired after the clock read   */
        uint32_t       tz;
        uint8_t        guard = 0U;
        if ((iv < 7000U) || (iv > 13000U) || (age < 0) || (age > (int32_t)(3U * iv))) break;   /* no trustworthy timing yet: wait */
        tz = (uint32_t)(g_ctrl.t_event_us + iv);                       /* the next predicted zero-cross */
        while (((int32_t)(tz - SLAVE_LEGACY_SH_PULLIN_US - SLAVE_LEGACY_MIN_LEAD_US - now_us) < 0) && (guard++ < 16U)) tz += iv;
        s_t_close_us = (uint32_t)(tz - SLAVE_LEGACY_SH_PULLIN_US);
        s_leg = LEG_WAIT_CLOSE;
        break;
    }
    case LEG_WAIT_CLOSE:
        if ((int32_t)(now_us - s_t_close_us) >= 0)
        {
            SlavePower_RelaySh(1U);                                    /* aimed so that the CONTACTS close at the zero-cross */
            s_leg = LEG_SH_HOLD;
            s_t_ms = now_ms;
        }
        else if ((int32_t)(s_t_close_us - now_us) > 40000)
        {
            s_leg = LEG_ARM;                                           /* the arithmetic is off: start over */
        }
        break;
    case LEG_SH_HOLD:
        if ((int32_t)(now_ms - s_t_ms) >= (int32_t)SLAVE_LEGACY_SH_HOLD_MS)
        {
            SlavePower_RelaySh(0U);                                    /* hand-over in the same instant: no gap for the DC link to bleed down */
            SlavePower_RelayW(1U);
            s_leg = LEG_SETTLE;
            s_t_ms = now_ms;
        }
        break;
    case LEG_SETTLE:
        if ((int32_t)(now_ms - s_t_ms) >= (int32_t)SLAVE_LEGACY_W_SETTLE_MS) s_leg = LEG_ON;
        break;
    default:
        break;
    }
}

#else   /* SLAVE_RELAYS_MODE == 0: relays are driven by hand, the firmware never touches PC8 / PC9 */

void    SlaveRelay_Init(void)                { }
void    SlaveRelay_Request(uint8_t want_free){ (void)want_free; }
void    SlaveRelay_Service(uint32_t now_ms)  { (void)now_ms; }
uint8_t SlaveRelay_IsShorted(void)           { return 1U; }
uint8_t SlaveRelay_UnwindDone(void)          { return 1U; }
uint8_t SlaveRelay_IsFree(void)              { return 1U; }
uint8_t SlaveRelay_State(void)               { return (uint8_t)RLY_FREE; }
void    SlaveRelay_ShortNow(void)            { }
void    SlaveRelay_Hold(void)                { }

#endif
