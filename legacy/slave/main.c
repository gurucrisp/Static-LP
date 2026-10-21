// 2nd Sept 2026 Soft start with different slope checked for Toroidal and EI core_working fine
// AUG29 11.48 Incrush time relay added and rectifer inverter delay added and relay macros added
// AUG28 9.30PM Relay Delay Solved for Start stop and setting menu


/* 1st_Version_For_Controller_No_LCD_CODE*/

/* ADC Readings, PID Regulator, Hardware Tested, 5th Feb*/
/** SLAVE **/

// Runtime Menu setting done slave
// Demo Bending
// Smooth PWM ON without BKIN
// BKIN Working and verified in Actual system but affescting regulatoin
// Breakinput communication to Master

// DEMO // /// ///// PID UPDATED 41 51 70 100
// Modified Inrush Current logic 11/05/2026

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>    // <- add this line
#include "main.h"

#define PWM_PERIOD 3199  // Define once globally
#define RELAY_PORT GPIOC

#define TRANSFORMER_ELAPSE_MAX 3000
#define TRANSFORMER_ELAPSE_MIN 50

#define MAX_ELAPSE_TIME_EMERGENCY_REBOOT  100

// --- Pin numbers (not HAL masks, just bit positions) ---
#define RELAY1_PIN 9    // PC9 → SH-RLY (pre-charge relay)
#define RELAY2_PIN 8    // PC8 → W-RLY (main working relay)
#define CONT1_PIN  0    // PC0 → CONT1 (contactor control)

// --- Helper macros using BSRR (atomic set/reset, 1 cycle write) ---
#define RELAY1_SET()    (GPIOC->BSRR = (1U << RELAY1_PIN))         // SH-RLY ON
#define RELAY1_RESET()  (GPIOC->BSRR = (1U << (RELAY1_PIN+16)))    // SH-RLY OFF

#define RELAY2_SET()    (GPIOC->BSRR = (1U << RELAY2_PIN))         // W-RLY ON
#define RELAY2_RESET()  (GPIOC->BSRR = (1U << (RELAY2_PIN+16)))    // W-RLY OFF

#define CONT1_SET()     (GPIOC->BSRR = (1U << CONT1_PIN))          // CONT1 OFF
#define CONT1_RESET()   (GPIOC->BSRR = (1U << (CONT1_PIN+16)))     // CONT1 ON

/* Output contactor closes only after regulated output is stable. */
#define CONTACTOR_VOLTAGE_TOLERANCE_V   2.0f
#define CONTACTOR_STABLE_TIME_MS        1000U
/* "PID implemented" gate: the soft-start ramp must have actually reached
 * the real setpoint (voltage_setpoint_pid == voltage_setpoint) before the
 * contactor is allowed to close - i.e. the loop must be running on the
 * true target, not still ramping toward it. */
#define CONTACTOR_PID_REACHED_TOL_V     0.5f

#define TIM1_TIM15_FIXED_ARR 3199

__IO uint8_t queueLock = 0;

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
//TIM_HandleTypeDef htim6;
//TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim14;
//TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

typedef struct {

    volatile float *input;
    volatile float *setpoint;
    volatile float *output;
    float Kp;
    float Ki; // units: output/(volt * second)
    float Kd;
    float outputMin;
    float outputMax;
    float integral;
    float integralMin;
    float integralMax;
    float derivative;
    float lastError;
    float derivFilterAlpha;
    float measFiltered;
    float rmsFilterAlpha;
    uint32_t lastTime;
    uint32_t timeStepMs;
    uint8_t enabled;
    float lastOutput;  // <-- Added to hold last PID output for freeze

} AutoPID;

AutoPID pid;
/* TEMPORARY bench-diagnostic: set by AutoPID_Compute() so other code (the
 * post-contactor trace below) can see whether the PID is currently frozen
 * (vrms_in within the freeze band of setpoint -> pwm_output forced to 0). */
volatile uint8_t pid_freeze_active = 0U;
/* TEMPORARY bench-diagnostic: set (non-blocking) by update_rectifier_allowed()
 * in the zero-cross ISR, EDGE-TRIGGERED - only on the actual 1->0 transition
 * of rectifier_allowed, never while it's already sitting off (that was the
 * bug in the previous version: it kept re-firing on every ISR pass during
 * the normal pre-connection wait, not just on a real drop). Identifies
 * WHICH of the four reasons caused this specific drop:
 *   1 = v_trip           (fast voltage-boundary check)
 *   2 = Mreceivedflag_pulse_on == 0  (master command lost)
 *   3 = Inrush_present == 1
 *   4 = gbIsSystemNormalState == 0   (already told to disconnect)
 * 0 = nothing pending. Consumed by Rectifier_Drop_Diag_Service() (main
 * loop), which does the actual (blocking) short chirp(s) - deliberately
 * shorter than the real fault beeps so they can't be confused with the
 * FAULT_SOURCE_* codes. */
volatile uint8_t rectifier_drop_diag_code = 0U;
/* Minimum time the raw freeze/unfreeze condition must hold steady before
 * AutoPID_Compute() actually changes state - prevents mains noise/ripple
 * near the freeze boundary from repeatedly toggling pwm_output between 0
 * and active regulation. */
#define FREEZE_DEBOUNCE_MS   300U


/////////////////////////////////////////////////////////////////////


#define SETTINGS_MAGIC     0xA5A5A5A5
#define FLASH_SETTINGS_ADDR 0x0807F800



/* ============================================================================
 * LOCAL PROTECTION / INPUT VALIDATION HELPERS
 *
 * Each protection can be switched off with its macro if it gets in the way of
 * bench work. Limits in the "frame limits" group are ASSUMED product ratings -
 * adjust them to the real hardware before release.
 * ========================================================================== */

/* Set to 1 once a valid [S] frame (with LV/HV/setpoint) has been received
 * from the master. Used to gate whether Effective_LV()/Effective_HV() trust
 * the master's StartRelay_Volt/HV_CUTOFF or fall back to the safety values. */
volatile uint8_t sFrameReceived = 0;


/* Input-voltage window enforced locally.
 *   Operating limits = the LV / HV the master sends in the [S] frame
 *                      (StartRelay_Volt / HV_CUTOFF), used exactly as given,
 *                      with NO clamping against the safety values below.
 *   Safety fallback  = INPUT_LV_SAFETY_V / INPUT_HV_SAFETY_V: used ONLY
 *                      before the first valid [S] frame has been received
 *                      from the master (sFrameReceived == 0). Once a
 *                      frame has been received, the master's LV/HV are
 *                      used as-is, however wide or narrow they are.
 *   input outside the effective window -> converter stops at once (zero-cross ISR) and,
 *   if it persists, the system is disconnected exactly like the master's [F] command
 *   (state 0, W-RLY opens after 1 s, contactor opens).
 *   Reconnect (same steps as the master's [C] command) only after the input has
 *   stayed inside [LV + hyst, HV - hyst] for the hold time.
 * A master [F]/[C]/[A] command cancels a pending local reconnect. */
#define LOCAL_VOLTAGE_LIMITS_ENABLED   1
#define INPUT_LV_SAFETY_V              140.0f /* fallback floor,   used only until the first [S] frame arrives */
#define INPUT_HV_SAFETY_V              300.0f /* fallback ceiling, used only until the first [S] frame arrives */
#define INPUT_LV_HYST_V                5.0f   /* restart margin above effective LV */
#define INPUT_HV_HYST_V                5.0f   /* restart margin below effective HV */
#define INPUT_TRIP_SAMPLES             3U     /* consecutive bad mains cycles (~60 ms) before disconnect */
/* Separate, FASTER debounce for update_rectifier_allowed() (runs every
 * zero-cross, not just once per Protection_Service loop pass) - stops a
 * single borderline/noisy sample right at the LV/HV boundary from
 * instantly cascading into rectifier_allowed -> pwm_allowed -> CONT1
 * opening and causing rapid contactor clicking during e.g. a slow variac
 * sweep through the threshold. */
#define RECTIFIER_TRIP_SAMPLES         3U
#define LOCAL_INPUT_AUTO_RECONNECT     1
#define INPUT_RECONNECT_HOLD_MS        5000U

/* Gross output over-voltage (any time the inverter runs, incl. start-up ramp,
 * when the output legitimately follows the input): trip when
 * Vout > INPUT_HV_SAFETY_V + margin for N consecutive fresh measurements
 * (one measurement per mains cycle).
 * NOTE: kept independent of the master's HV_CUTOFF on purpose - this is a
 * fixed hardware overvoltage backstop for the output stage, not the input
 * operating window, so it is unaffected by the LV/HV-from-master change below. */
#define LOCAL_OUTPUT_OV_ENABLED        1
#define OUTPUT_OV_MARGIN_V             10.0f
#define OUTPUT_OV_LIMIT_V              (INPUT_HV_SAFETY_V + OUTPUT_OV_MARGIN_V)
#define OUTPUT_OV_TRIP_SAMPLES         3U

/* Regulated-output window (nominal +/- X %): REMOVED per request - disabled
 * below by setting LOCAL_OUTPUT_WINDOW_ENABLED to 0. Left in place (not
 * deleted) so it can be switched back on with a single macro if needed;
 * with it at 0 none of this code compiles in or runs. */
#define LOCAL_OUTPUT_WINDOW_ENABLED    0
#define OUTPUT_NOMINAL_V               230.0f
#define OUTPUT_WINDOW_PCT              0.15f
#define OUTPUT_WIN_HI_V                (OUTPUT_NOMINAL_V * (1.0f + OUTPUT_WINDOW_PCT))
#define OUTPUT_WIN_LO_V                (OUTPUT_NOMINAL_V * (1.0f - OUTPUT_WINDOW_PCT))
#define OUTPUT_WIN_ARM_MS              2000U
#define OUTPUT_WIN_OV_SAMPLES          3U     /* ~60 ms  */
#define OUTPUT_WIN_UV_SAMPLES          25U    /* ~500 ms */

/* Over-temperature: NTC line active this long -> latched fault, and rearm is
 * held off while it stays active. (The original relay_handler had this check
 * commented out - set to 0 if the NTC input gives false trips on your boards.) */
#define LOCAL_OVERTEMP_TRIP_ENABLED    1
#define OVERTEMP_TRIP_MS               3000U

/* A measurement older than this is treated as missing. The RMS routine runs in
 * the main loop, so keep this well above one mains cycle (20 ms @ 50 Hz). */
#define MEAS_STALE_MS                  250U

/* Boot: give up waiting for the first zero-cross cycle after this long. */
#define BOOT_ZC_WAIT_MS                3000U

/* [S] frame: only the regulated-output setpoint is range-checked. HV/LV are
 * accepted as before (they no longer drive protection), so a frame the master
 * already sends is never rejected - a rejected frame would stall the handshake. */
#define SET_OUT_MIN_V                  100
#define SET_OUT_MAX_V                  300

/* [I]/[O] calibration limits. */
#define CAL_TARGET_MIN_V               20.0f
#define CAL_TARGET_MAX_V               350.0f
#define CAL_TRIM_MAX_V                 50.0f

volatile uint8_t  meas_valid   = 0U;
volatile uint32_t meas_last_ms = 0U;
volatile uint32_t meas_seq     = 0U;

static inline void Measurement_Mark(void)
{
    meas_last_ms = HAL_GetTick();
    meas_seq++;
    meas_valid = 1U;
}

static inline uint8_t Measurement_IsFresh(void)
{
    return (meas_valid && ((uint32_t)(HAL_GetTick() - meas_last_ms) <= MEAS_STALE_MS)) ? 1U : 0U;
}

/* Input-voltage permit. While running the window is exactly [lv, hv] (trip at
 * the limit); to (re)start the input must be back inside by the hysteresis
 * margin. NaN fails safe (returns 0). */
static inline uint8_t Input_Voltage_Permit(uint8_t currently_allowed, float v, float lv, float hv)
{
    if (currently_allowed)
        return (v >= lv && v <= hv) ? 1U : 0U;
    return (v >= (lv + INPUT_LV_HYST_V) && v <= (hv - INPUT_HV_HYST_V)) ? 1U : 0U;
}

/* Master's LV/HV used exactly as sent - no clamping to the safety values.
 * The safety values are used only as a fallback before the master has ever
 * sent a valid [S] frame (sFrameReceived == 0), e.g. right after power-up. */
static inline float Effective_LV(float master_lv)
{
    return sFrameReceived ? master_lv : INPUT_LV_SAFETY_V;
}

static inline float Effective_HV(float master_hv)
{
    return sFrameReceived ? master_hv : INPUT_HV_SAFETY_V;
}

static inline bool Settings_Setpoint_Valid(int setv)
{
    return (setv >= SET_OUT_MIN_V && setv <= SET_OUT_MAX_V);
}

/* trim = round(target - RAW reading). Must be given the UNTRIMMED measurement,
 * otherwise a second calibration would store only the residual. */
static inline bool Cal_ComputeTrim(float target, float raw, int16_t *trim_out)
{
    float t;

    if (!(target >= CAL_TARGET_MIN_V && target <= CAL_TARGET_MAX_V)) return false;
    if (!(raw > 0.0f && raw < 1000.0f))                               return false;

    t = target - raw;
    t += (t >= 0.0f) ? 0.5f : -0.5f;
    if (!(t >= -CAL_TRIM_MAX_V && t <= CAL_TRIM_MAX_V))               return false;

    *trim_out = (int16_t)t;
    return true;
}

void Load_Settings_From_Flash(void);
typedef struct
{
    // Calibrate settings
    int16_t input_v_r;
    int16_t output_v_r;
    uint32_t valid_flag;

    float Nbb_tf;
} Settings_t;

Settings_t saved_settings;
Settings_t working_settings;

/*
 * Default settings:
 * Restart_Mode:
 * 'Y' = Auto (Default)
 * 'N' = Manual
 */

const Settings_t Default_Settings =
{
    .input_v_r = 0,
    .output_v_r = 0,
    .valid_flag = 0xA5A5A5A5,
    .Nbb_tf = 2.3f
};

/* ---------------- Flash ---------------- */

void Flash_Erase_Page(uint32_t page_address)
{
    (void)page_address;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.Page = (FLASH_SETTINGS_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;
    eraseInitStruct.NbPages = 1;
    (void)HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);

    HAL_FLASH_Lock();
}

void Load_Settings_From_Flash(void)
{
    const Settings_t* ptr = (const Settings_t*) FLASH_SETTINGS_ADDR;

    if (ptr->valid_flag == SETTINGS_MAGIC) {
        saved_settings = *ptr;
    } else {
        saved_settings = Default_Settings;
        saved_settings.valid_flag = SETTINGS_MAGIC;
    }

    /* A record can carry the right magic yet hold garbage; reject wild trims. */
    if (saved_settings.input_v_r  >  (int16_t)CAL_TRIM_MAX_V || saved_settings.input_v_r  < -(int16_t)CAL_TRIM_MAX_V ||
        saved_settings.output_v_r >  (int16_t)CAL_TRIM_MAX_V || saved_settings.output_v_r < -(int16_t)CAL_TRIM_MAX_V)
    {
        saved_settings = Default_Settings;
        saved_settings.valid_flag = SETTINGS_MAGIC;
    }

    working_settings = saved_settings;
}

static bool Save_Settings_To_Flash(void)
{
    uint64_t buf[(sizeof(Settings_t) + 7U) / 8U];
    uint32_t i;
    bool ok = true;

    working_settings.valid_flag = SETTINGS_MAGIC;

    /* Copy into a doubleword-sized buffer so the last program cycle never reads
       past the end of Settings_t (it is 12 bytes, not a multiple of 8). */
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, &working_settings, sizeof(Settings_t));

    Flash_Erase_Page(FLASH_SETTINGS_ADDR);

    HAL_FLASH_Unlock();
    for (i = 0U; i < (sizeof(buf) / sizeof(buf[0])); i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              FLASH_SETTINGS_ADDR + (8U * i), buf[i]) != HAL_OK)
        {
            ok = false;
            break;
        }
    }
    HAL_FLASH_Lock();

    /* Read back: catches a failed erase/program instead of trusting it. */
    if (ok && (memcmp((const void *)FLASH_SETTINGS_ADDR, &working_settings, sizeof(Settings_t)) != 0))
        ok = false;

    if (ok)
        saved_settings = working_settings;   /* RAM mirror only on success */

    return ok;
}
////////////////////////////////////////////////////////////////////

//uint16_t adc_dma_buffer[NUM_SAMPLES * NUM_CHANNELS];
//volatile uint8_t adc_done ;   // ready initially
float t;
char uart_buf[150];
uint8_t counter=0;
uint8_t counter2=0;

float irms;
//float vrms_in = 1;
//float vrms_out = 1;
/*****************************************************/

uint8_t StartRelay_Volt = 180;  // LV Cut OFF voltage
float HV_CUTOFF = 280.0f;
float adcoffset = 1.678f;
float input_v_offset = 0;
float output_v_offset = 0;


/******************************************************/
float error;
float F_MAINS ;
volatile int half_cycle = 1;
float Duty_PWM;
float ac_in_voltage;
float ac_out_voltage;
float vin;
float vout;
float ac_crt;

volatile float vrms_in;
volatile float vrms_out;
volatile static uint8_t pwm_allowed = 0;
volatile static uint8_t rectifier_allowed = 0;  // NEW FLAG for rectifier hysteresis
volatile float voltage_setpoint = 230.0f;
volatile float voltage_setpoint_pid = 0.0f;
volatile float pwm_output = 0.0f;
volatile uint8_t gbIsSystemNormalState = 1;
volatile uint8_t Inrush_present = 0;

/* Raw (untrimmed) measurements, needed so calibration is correct whether the
 * command arrives in the settings menu (raw shown) or while running (trimmed). */
volatile float vrms_in_raw  = 0.0f;
volatile float vrms_out_raw = 0.0f;

/* Set when THIS firmware disconnected the system because of the input window;
 * cleared on local reconnect or by any master [F]/[C]/[A] command. */
volatile uint8_t input_tripped = 0U;

/* Effective input limits: what the master sent, clamped to the 140/300 backstop. */
static inline float Input_LV_Limit(void) { return Effective_LV((float)StartRelay_Volt); }
static inline float Input_HV_Limit(void) { return Effective_HV(HV_CUTOFF); }

void Stop_All_PWM(void);

/* ---- Independent watchdog (register level, no HAL IWDG module needed) ------
 * Started only after the boot handshake. Kicked from the main loop and from the
 * handshake wait loops. Timeout must exceed the longest blocking sequence
 * between kicks (transformer test ~3 s). A reset is safe: all outputs default
 * off and the normal startup sequence runs again. Set USE_IWDG to 0 to disable. */
#define USE_IWDG          1
#define IWDG_TIMEOUT_MS   8000U

static inline void Watchdog_Kick(void)
{
#if USE_IWDG
    IWDG->KR = 0x0000AAAAU;
#endif
}

static void Watchdog_Init(void)
{
#if USE_IWDG
    uint32_t reload = (IWDG_TIMEOUT_MS * 125U) / 1000U;   /* LSI ~32 kHz / 256 = 125 Hz */
    uint32_t guard  = 1000000U;

#ifdef __HAL_DBGMCU_FREEZE_IWDG
#ifdef __HAL_RCC_DBGMCU_CLK_ENABLE
    __HAL_RCC_DBGMCU_CLK_ENABLE();
#endif
    __HAL_DBGMCU_FREEZE_IWDG();          /* don't reset while halted at a breakpoint */
#endif
    IWDG->KR  = 0x0000CCCCU;             /* start                       */
    IWDG->KR  = 0x00005555U;             /* unlock PR / RLR             */
    IWDG->PR  = 6U;                      /* prescaler /256              */
    IWDG->RLR = reload;
    while ((IWDG->SR != 0U) && (guard-- != 0U)) { }
    IWDG->KR  = 0x0000AAAAU;             /* reload + lock               */
#endif
}

/* ---- Deferred settings save -------------------------------------------------
 * A flash page erase stalls instruction fetch (~20 ms). Doing that from the UART
 * callback while the bridge is switching would delay the zero-cross/dead-time
 * ISRs, so [I]/[O] only queue a save; it runs from the main loop when both the
 * rectifier bridge and the inverter are idle. */
volatile uint8_t settings_save_pending = 0U;
volatile uint8_t settings_save_failed  = 0U;

static void Settings_Service(void)
{
    if (!settings_save_pending)
        return;
    if (pwm_allowed || rectifier_allowed)
        return;

    settings_save_pending = 0U;
    if (!Save_Settings_To_Flash())
        settings_save_failed = 1U;
}

char msg[100];
//int AVR;
int Tim4Prd = 49;   // 5ms defaul

//*****************************************************
uint8_t relay2_on = 0;
uint32_t LOW_Voltflag = 0;
static uint32_t lowVoltStart = 0;      // Timestamp when low voltage first appeared
static uint8_t lowVoltFlag = 0;        // Flag to indicate low voltage timing is running
//*****************************************************


uint8_t next_rectifier_state = 0;  // 1 = positive half, 0 = negative half
uint8_t counter_gar=0;

uint8_t wait_done = 0;  // 0 = still showing Wait screen, 1 = done, move to main
uint32_t wait_start_time = 0;
extern uint32_t startup_time;
int LOAD;
float KVA;
float rated_load;
volatile uint8_t buzzer_state = 0;
int Flip_mode = 1;
float Volt_Diff;
int offset =0;

int HV_trip = 0;
int HV_active = 0;
int HV_recover = 0;
int CONT1_ON = 0; // Cont is off by default

volatile uint8_t transformer_test_active = 0;
volatile uint32_t transformer_test_timer = 0;
volatile uint8_t flipflag;
volatile uint8_t  onetimeflag = 0;


int flt=1;
int flt_load=1;
int flt_CN7=1;
int ntc=1;
int on_off=0;
//int after_relay_rs485_flag = 0;

unsigned char gbIsEmergencyShutdown  = 0;
unsigned char gbIsSendEmergencyShutdown  = 0;

//unsigned int gui8SysHealthStatus  = SYS_HEALTH_NORMAL;
unsigned int gui8SysFaultType;// = SYS_FAULT_TYPE_HEAT;
char gchSysFaultPhase = '0';

uint8_t XLoadstatusFlag = 0;
uint8_t YLoadstatusFlag = 0;
uint8_t ZLoadstatusFlag = 0;

uint8_t   gui8VendorIndx = 0;

uint8_t flt_in_other =1;


#define SOFTSTART_SLOW_TIME_MS       2000U
#define SOFTSTART_SLOW_SLOPE_VPS     3.0f
#define SOFTSTART_FAST_SLOPE_VPS     10.0f
#define SOFTSTART_FINAL_SLOPE_VPS    5.0f
#define SOFTSTART_FINAL_BAND_V       3.0f
#define SOFTSTART_MAX_DT_S           0.050f

/* Startup glitch guard: hard zero duty for this long right after PWM
 * enable. The stepped-percentage ramp that used to follow this has been
 * removed - the voltage_setpoint_pid slope ramp (ramp_setpoint_slope_ts)
 * is now the sole authority over how fast duty/current rises, since it's
 * a real closed-loop ramp rather than open-loop fixed steps that were
 * clipping the PID's own smooth output. */
#define INVERTER_STARTUP_DUTY_0_MS   100U

/* ============================================================================
 * LOAD / OUTPUT CONTACTOR SOFT-START
 * ============================================================================
 * Software-only PWM limiter after CONT1 closes. No ADC/DMA/timer/ISR setup is
 * changed and no fixed kVA/current trip is used on the slave.
 *
 * Shrunk to a 300 ms / 85%-start token ramp on request, so the output settles
 * back to setpoint almost immediately after CONT1 closes. This is the ONLY
 * inrush limiting on the load path (no current sense/hardware trip backs it
 * up) - it now gives only a thin margin of protection. If nuisance
 * trips/stress show up on the load side after this change, widen the window
 * (larger LOAD_SOFTSTART_TIME_MS) or lower the starting floor
 * (smaller LOAD_SOFTSTART_DUTY_0_PCT) back up.
 */
#define LOAD_SOFTSTART_TIME_MS       300U
#define LOAD_SOFTSTART_DUTY_0_PCT    0.85f
#define LOAD_SOFTSTART_DUTY_1_PCT    0.65f
#define LOAD_SOFTSTART_DUTY_2_PCT    0.80f
#define LOAD_SOFTSTART_DUTY_3_PCT    0.90f

volatile uint8_t load_softstart_active = 0U;
volatile uint32_t load_softstart_begin_ms = 0U;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
//static void MX_TIM6_Init(void);
//static void MX_TIM7_Init(void);
//static void MX_TIM16_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM17_Init(void);
void System_op();
void relay_handler(float vrms_in);
static void Output_Contactor_Service(void);

float Read_AC_Voltage_RMS(uint32_t channel);
float Read_Current_RMS(void);

int mode = 0; // default 0 single phase, 1 three phase

/*============================ Function Definitions ============================*/

//float Nbb_tf = 5.78f; //Default Value 237/41 ratio
float Nbb=5.78f;

float ratio_buck = (1.8f)/5.78f; // 237/41 Buck Boost ratio
float ratio_boost = (0.9f)/5.78f; // 237/41 Buck Boost ratio // 1.2

////////////////////////////////////////////////////////////////////////////////////////
#define RX_BUF_SIZE     256
#define LINE_BUF_SIZE  128

uint8_t uart1_rx_dma[RX_BUF_SIZE];
char lineBuf[LINE_BUF_SIZE];

volatile uint16_t lineIndex = 0;
volatile uint8_t phasecame = 0;
char sFrameData[LINE_BUF_SIZE];


/* Current phase */
char currentPhase = 'R';
static char txBuf[64];
char uart2_tx_buf[64];

volatile uint8_t settingmenu_completed_flag = 0 ;
volatile uint8_t Mreceivedflag_pulse_on = 0;
volatile uint8_t  runtime_setting_menu = 0;
volatile uint8_t trasnfomersetflag = 0;

float step = 1.0f;
int dir_in  = 1;   // +1 = up, -1 = down
int dir_out = 1;

volatile uint8_t excessheatframe = 0;

volatile uint8_t gFaultLatched = 0;
volatile uint8_t gFaultdisplay = 0;
volatile uint8_t gFaultResetRequest = 0;
volatile uint8_t gFaultSlowCleanupRequest = 0;   // optional (explained later)
//int con_state=1;



volatile uint8_t  rectifier_to_inverter_delay_active = 0;
volatile uint32_t rectifier_on_time = 0;
#define RECTIFIER_TO_INVERTER_DELAY_MS    3000U

volatile uint8_t inverter_startup_active = 0U;
volatile uint32_t inverter_startup_begin_ms = 0U;

volatile uint8_t relay_pending = 0;
volatile uint8_t relay_pending_state = 0;   // 0 = OFF, 1 = ON
volatile uint32_t relay_delay_start = 0;
volatile uint8_t c_sequence_active = 0;
volatile uint8_t c_sequence_step = 0;
volatile uint32_t c_sequence_time = 0;

volatile uint8_t c_relay_close_pending = 0;
volatile uint32_t c_relay_close_time = 0;

volatile uint8_t a_relay_close_pending = 0;
volatile uint32_t a_relay_close_time = 0;

volatile uint8_t  Relay_operation_starting_time = 0;

/* Relay-1 (SH-RLY) close scheduling state - declared here (ahead of
   Fault_Shutdown_Latch_Fast) so a fault can cancel a pending scheduled
   closure before it fires. Definitions/logic for arming these live
   further down, next to ZeroCross_CheckAndUpdate(). */
volatile uint8_t relay1_close_armed = 0;
volatile uint8_t relay1_predictive_pending = 0;

static inline void Relay1_CancelPendingClose(void)
{
    relay1_close_armed = 0;
    relay1_predictive_pending = 0;
    __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);
}

static inline void Rectifier_AllOff_Immediate(void)
{
    // Your rectifier drive pins in code:
    GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;  // PC4 PC5 OFF
    GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;  // PB3 PB5 OFF
}

#if 0
static inline void Fault_Shutdown_Latch_Fast(void)
{
    if (gFaultLatched)
        return;

    gFaultLatched = 1;

    // --- 1) Rectifier OFF immediately (GPIO BSRR resets) ---
    GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;
    GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;

    // --- 2) Stop TIM14 update interrupt (rectifier scheduling) ---
    TIM14->DIER &= ~TIM_DIER_UIE;  // disable update IRQ
    TIM14->CR1  &= ~TIM_CR1_CEN;   // stop counter
    TIM14->SR   &= ~TIM_SR_UIF;    // clear pending update flag

    // --- 3) Force PWM outputs off (TIM1 + TIM15) ---
    // Set duty to 0
    TIM1->CCR1  = 0;
    TIM15->CCR1 = 0;

    // Disable channel outputs (CH1 and CH1N)
    TIM1->CCER  &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    TIM15->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);

    // Disable main outputs (MOE) - strongest "stop driving"
    TIM1->BDTR  &= ~TIM_BDTR_MOE;
    TIM15->BDTR &= ~TIM_BDTR_MOE;

    // Optionally stop counters too (not required, but reduces internal switching)
    TIM1->CR1   &= ~TIM_CR1_CEN;
    TIM15->CR1  &= ~TIM_CR1_CEN;

    gbIsSystemNormalState = 0;

    con_state = 0;

    // Ensure writes complete before leaving ISR
    __DMB();
}
#endif

////// Added Code for Modified Inrush Logic ///////
///////////////////////////////////////////////////
volatile uint8_t gFaultLatched_dummy = 0;
#define FAULT_REARM_DELAY_MS   10000U

static inline uint8_t BKIN_FaultActive(void);
static void Fault_ReArm_Service(void);


volatile static uint8_t pwm_allowed_Cont = 0;
volatile uint8_t gPwmStatusHold = 0;

#define PWM_STATUS_RELEASE_DELAY_MS   300U
static void PwmStatusHold_Service(void);

/* Diagnostic only: records which trigger last called
 * Fault_Shutdown_Latch_Fast(), so you can tell a genuine hardware
 * BKIN/overcurrent trip apart from our own zero-cross monitor tripping
 * (phase_seq_fault) - both currently report through the same
 * Inrush_present flag downstream, so this is the only way to
 * distinguish them without a scope. Check via breakpoint/watch. */
#define FAULT_SOURCE_NONE      0U
#define FAULT_SOURCE_BKIN_HW   1U   // real hardware overcurrent comparator trip
#define FAULT_SOURCE_PHASE_ZC  2U   // our own zero-cross anomaly monitor
#define FAULT_SOURCE_SW_PROTECT 3U  // local software protection (output OV / over-temp)
volatile uint8_t gLastFaultSource = FAULT_SOURCE_NONE;

/* ---- Diagnostic buzzer: distinct beep pattern per fault source ----
 * 1 beep = genuine hardware BKIN/overcurrent trip
 * 2 beeps = our own zero-cross monitor (phase_seq_fault)
 * Only runs from the main loop (never from an ISR). Uses a brief
 * software-toggled tone burst on the ON portion of each beep so it's
 * audible on a passive buzzer too, not just an active one. This briefly
 * blocks the main loop, which is acceptable here because it only runs
 * while the system is already in a faulted/shutdown state (PWM already
 * disabled, relay already forced open) - remove once no longer needed
 * for diagnosis. */
#define BUZZ_TONE_PIN         GPIO_PIN_6
#define BUZZ_TONE_PORT        GPIOA
#define BUZZ_BEEP_MS          150U
#define BUZZ_GAP_MS           150U
#define BUZZ_TONE_HALFPERIOD_US 250U   // ~2kHz tone

volatile uint8_t gBuzzPatternArmedFor = FAULT_SOURCE_NONE;

static inline void Diagnostic_Buzzer_OneBeep(void)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < BUZZ_BEEP_MS)
    {
        HAL_GPIO_TogglePin(BUZZ_TONE_PORT, BUZZ_TONE_PIN);
        for (volatile uint32_t i = 0; i < (BUZZ_TONE_HALFPERIOD_US * 8U); i++) { }
    }
    HAL_GPIO_WritePin(BUZZ_TONE_PORT, BUZZ_TONE_PIN, GPIO_PIN_RESET);
    HAL_Delay(BUZZ_GAP_MS);
}

static inline void Diagnostic_Buzzer_Service(void)
{
    if (gLastFaultSource == FAULT_SOURCE_NONE)
    {
        gBuzzPatternArmedFor = FAULT_SOURCE_NONE;   // ready for the next fault
        return;
    }

    if (gLastFaultSource == gBuzzPatternArmedFor)
        return;   // already beeped for this fault instance

    gBuzzPatternArmedFor = gLastFaultSource;

    uint8_t beeps = (gLastFaultSource == FAULT_SOURCE_BKIN_HW) ? 1U :
                     (gLastFaultSource == FAULT_SOURCE_PHASE_ZC) ? 2U : 3U;
    for (uint8_t i = 0; i < beeps; i++)
    {
        Diagnostic_Buzzer_OneBeep();
    }
}

/* TEMPORARY bench-diagnostic: consumes rectifier_drop_diag_code (set in the
 * zero-cross ISR by update_rectifier_allowed(), see its comment). Fires
 * that many SHORT chirps - deliberately shorter than a real fault beep, and
 * never mixed into the same call, so they can't be confused with the
 * fault-source codes above. Call from the main loop only (this blocks
 * briefly, proportional to the code). */
static inline void Rectifier_Drop_Diag_Service(void)
{
    if (rectifier_drop_diag_code == 0U)
        return;

    uint8_t chirps = rectifier_drop_diag_code;
    rectifier_drop_diag_code = 0U;

    for (uint8_t i = 0; i < chirps; i++)
    {
        uint32_t start = HAL_GetTick();
        while ((HAL_GetTick() - start) < (BUZZ_BEEP_MS / 3U))
        {
            HAL_GPIO_TogglePin(BUZZ_TONE_PORT, BUZZ_TONE_PIN);
            for (volatile uint32_t j = 0; j < (BUZZ_TONE_HALFPERIOD_US * 8U); j++) { }
        }
        HAL_GPIO_WritePin(BUZZ_TONE_PORT, BUZZ_TONE_PIN, GPIO_PIN_RESET);
        HAL_Delay(BUZZ_GAP_MS / 2U);
    }
}

/* ==================== BLACK BOX (diagnostics) ====================
 * Works without a buzzer or any extra hardware:
 *   - Debugger: add  g_diag  to Live Expressions (STM32CubeIDE) and look at it after a shutdown.
 *   - Optional text log: set DIAG_UART_ENABLED to 1 to print lines on USART3 (115200 8N1).
 *     That is NOT the master link, but only enable it if USART3's pins are free.
 *
 * reset_cause (why the controller last restarted):
 *   0 unknown | 1 NRST pin | 2 power-on/brown-out | 3 software | 4 IWDG watchdog | 5 WWDG | 6 low-power
 *   (an external reset or supply dip shows 1 or 2; the watchdog shows 4)
 * last_reason (why the last latched fault happened):
 *   1 hardware overcurrent (BKIN)   2 output > 310 V   3 output > 253 V (regulated)
 *   4 output < 207 V (regulated)    5 over-temperature 6 other
 * ================================================================= */
#define DIAG_UART_ENABLED   0

#define DIAG_R_NONE          0U
#define DIAG_R_BKIN_HW       1U
#define DIAG_R_OUT_GROSS_OV  2U
#define DIAG_R_OUT_WIN_OV    3U
#define DIAG_R_OUT_WIN_UV    4U
#define DIAG_R_OVERTEMP      5U
#define DIAG_R_OTHER         6U

#define DIAG_RST_UNKNOWN     0U
#define DIAG_RST_PIN         1U
#define DIAG_RST_POR_BOR     2U
#define DIAG_RST_SOFTWARE    3U
#define DIAG_RST_IWDG        4U
#define DIAG_RST_WWDG        5U
#define DIAG_RST_LPWR        6U

/* RCC->CSR reset flags (STM32G0) */
#define DIAG_CSR_RMVF        (1UL << 23)
#define DIAG_CSR_PINRSTF     (1UL << 26)
#define DIAG_CSR_PWRRSTF     (1UL << 27)
#define DIAG_CSR_SFTRSTF     (1UL << 28)
#define DIAG_CSR_IWDGRSTF    (1UL << 29)
#define DIAG_CSR_WWDGRSTF    (1UL << 30)
#define DIAG_CSR_LPWRRSTF    (1UL << 31)

typedef struct
{
    uint32_t reset_csr;              /* raw RCC->CSR captured at boot                   */
    uint8_t  reset_cause;            /* DIAG_RST_*                                       */
    uint8_t  last_reason;            /* DIAG_R_* of the most recent latched fault        */
    uint16_t fault_count[7];         /* per DIAG_R_*, since this boot                    */
    uint32_t last_fault_ms;          /* HAL tick when it happened                        */
    int16_t  last_vin;               /* volts, at the fault                              */
    int16_t  last_vout;
    int16_t  last_irms;
    uint16_t input_disconnects;      /* local input-window disconnects since boot        */
    int16_t  last_disconnect_vin;
    uint16_t stale_lockouts;         /* measurement-stale lockouts since boot            */
} Diag_t;

volatile Diag_t g_diag;
static volatile uint8_t diag_next_reason = DIAG_R_OTHER;

static inline uint8_t Diag_DecodeReset(uint32_t csr)
{
    /* Order matters: watchdog / software / power resets ALSO set the NRST-pin flag. */
    if (csr & DIAG_CSR_IWDGRSTF) return DIAG_RST_IWDG;
    if (csr & DIAG_CSR_WWDGRSTF) return DIAG_RST_WWDG;
    if (csr & DIAG_CSR_LPWRRSTF) return DIAG_RST_LPWR;
    if (csr & DIAG_CSR_SFTRSTF)  return DIAG_RST_SOFTWARE;
    if (csr & DIAG_CSR_PWRRSTF)  return DIAG_RST_POR_BOR;
    if (csr & DIAG_CSR_PINRSTF)  return DIAG_RST_PIN;
    return DIAG_RST_UNKNOWN;
}

static void Diag_CaptureResetCause(void)
{
    uint32_t csr = RCC->CSR;
    g_diag.reset_csr   = csr;
    g_diag.reset_cause = Diag_DecodeReset(csr);
    RCC->CSR |= DIAG_CSR_RMVF;                 /* clear the flags for the next boot */
}

static inline int16_t Diag_Round16(float v)
{
    if (!(v > -32000.0f)) return -32000;       /* also catches NaN */
    if (v > 32000.0f)     return 32000;
    return (int16_t)(v + ((v >= 0.0f) ? 0.5f : -0.5f));
}

/* Called from Fault_Shutdown_Latch_Fast() (may run in an ISR: no blocking, no printing). */
static inline void Diag_RecordFault(void)
{
    uint8_t r = diag_next_reason;
    diag_next_reason = DIAG_R_OTHER;
    if (r > DIAG_R_OTHER) r = DIAG_R_OTHER;

    g_diag.last_reason = r;
    if (g_diag.fault_count[r] < 65535U) g_diag.fault_count[r]++;
    g_diag.last_fault_ms = HAL_GetTick();
    g_diag.last_vin  = Diag_Round16(vrms_in);
    g_diag.last_vout = Diag_Round16(vrms_out);
    g_diag.last_irms = Diag_Round16(irms);
}

static void Diag_BootReport(void)
{
#if DIAG_UART_ENABLED
    char b[64];
    int n = snprintf(b, sizeof b, "\r\n[BOOT] reset=%u csr=%08lX\r\n",
                     (unsigned)g_diag.reset_cause, (unsigned long)g_diag.reset_csr);
    if (n > 0) HAL_UART_Transmit(&huart3, (uint8_t *)b, (uint16_t)n, 30U);
#endif
}

/* Main-loop only. Prints one line per new event (fault / disconnect / stale lockout). */
static void Diag_Service(void)
{
#if DIAG_UART_ENABLED
    static uint16_t p_faults = 0U, p_disc = 0U, p_stale = 0U;
    char b[96];
    int n;
    uint16_t faults = 0U;
    uint8_t i;

    for (i = 0U; i <= DIAG_R_OTHER; i++) faults = (uint16_t)(faults + g_diag.fault_count[i]);

    if (faults != p_faults)
    {
        p_faults = faults;
        n = snprintf(b, sizeof b, "[FAULT] t=%lu reason=%u vin=%d vout=%d irms=%d\r\n",
                     (unsigned long)g_diag.last_fault_ms, (unsigned)g_diag.last_reason,
                     (int)g_diag.last_vin, (int)g_diag.last_vout, (int)g_diag.last_irms);
        if (n > 0) HAL_UART_Transmit(&huart3, (uint8_t *)b, (uint16_t)n, 30U);
    }
    if (g_diag.input_disconnects != p_disc)
    {
        p_disc = g_diag.input_disconnects;
        n = snprintf(b, sizeof b, "[INPUT-DISCONNECT] vin=%d count=%u\r\n",
                     (int)g_diag.last_disconnect_vin, (unsigned)p_disc);
        if (n > 0) HAL_UART_Transmit(&huart3, (uint8_t *)b, (uint16_t)n, 30U);
    }
    if (g_diag.stale_lockouts != p_stale)
    {
        p_stale = g_diag.stale_lockouts;
        n = snprintf(b, sizeof b, "[STALE-DATA] count=%u\r\n", (unsigned)p_stale);
        if (n > 0) HAL_UART_Transmit(&huart3, (uint8_t *)b, (uint16_t)n, 30U);
    }
#endif
}
/* ==================== end BLACK BOX ==================== */

static inline void Fault_Shutdown_Latch_Fast(void)
{
    if (gFaultLatched)
        return;

    Diag_RecordFault();

    /* Cancel any pending/scheduled RELAY1 (SH-RLY) close first - otherwise
       a closure armed just before this fault could still fire a few ms
       later and re-close the relay right after we've just forced it open. */
    Relay1_CancelPendingClose();
    RELAY1_RESET();

    gFaultLatched = 1;
    gFaultdisplay =  1;

    /* Hold PWM status reported to master during fault + rearm window */
    gPwmStatusHold = 1;

    /* Open output/load contactor immediately on any inverter fault. */
    CONT1_ON = 0;
    CONT1_SET();

    /* Clear permission flags immediately */
    pwm_allowed = 0;
    rectifier_allowed = 0;

    /* --- 1) Rectifier OFF immediately --- */
    GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;
    GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;

    /* --- 2) Stop TIM14 update interrupt (rectifier scheduling) --- */
    TIM14->DIER &= ~TIM_DIER_UIE;
    TIM14->CR1  &= ~TIM_CR1_CEN;
    TIM14->SR   &= ~TIM_SR_UIF;

    /* --- 3) Force PWM outputs off (TIM1 + TIM15) --- */
    TIM1->CCR1 = 0;
    TIM15->CCR1 = 0;

    /* Disable channel outputs */
    TIM1->CCER  &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    TIM15->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);

    /* Disable main outputs */
    TIM1->BDTR  &= ~TIM_BDTR_MOE;
    TIM15->BDTR &= ~TIM_BDTR_MOE;

    /* Keep counters stopped in fault-latched state */
    TIM1->CR1  &= ~TIM_CR1_CEN;
    TIM15->CR1 &= ~TIM_CR1_CEN;

    Inrush_present = 1;

    /* Fault disconnects the load, so cancel its soft-start state. */
    load_softstart_active = 0U;
    load_softstart_begin_ms = 0U;
 //   gbIsSystemNormalState = 0;
  //  con_state = 0;

    __DMB();
}

/* ---- Mains phase-change / phase-loss detection (zero-crossing based) ---- */
#define ZC_HALFCYCLE_MIN_US   8000U   // below this (e.g. ~6.x ms) = abnormal for 50Hz
#define ZC_HALFCYCLE_MAX_US  12000U   // above this (e.g. ~12.x ms) = abnormal for 50Hz
#define ZC_SUDDEN_DELTA_US    3000U   // sudden jump vs previous half-cycle
#define ZC_HEALTHY_STREAK       20U   // consecutive good edges needed before allowing rearm
#define ZC_STARTUP_GRACE_EDGES   8U   // edges to ignore right after power-on/rearm (relay
                                       // bounce / inrush noise settling, ~4 line cycles)

static volatile uint16_t zc_last_edge_time_us    = 0;
static volatile uint16_t zc_last_halfcycle_us    = 0;
static volatile uint8_t  zc_last_edge_was_rising = 0;
static volatile uint8_t  zc_edge_index           = 0;
static volatile uint8_t  zc_bad_edge_count       = 0;
static volatile uint8_t  zc_good_streak          = 0;
static volatile uint8_t  zc_startup_counter      = 0;
volatile uint8_t phase_seq_fault = 0;

/* Software (zero-cross) fault trip: disabled again. Detection and
 * timing bookkeeping below still run as normal (Relay1_ArmClose's
 * predictive scheduling depends on it) - only the actual
 * Fault_Shutdown_Latch_Fast() trip on a detected anomaly is suppressed. */
#define ZC_FAULT_ACTION_ENABLED   0

/* TEMPORARY: proves in the BUILD LOG what this actually resolves to for
 * this specific build/toolchain/project settings - in case a Makefile,
 * IDE project setting, or main.h is silently overriding it with a -D flag
 * or a conflicting #define elsewhere that isn't visible in this file.
 * Look for this line in your build output. Remove once confirmed. */
#if ZC_FAULT_ACTION_ENABLED
#warning "BUILD CHECK: ZC_FAULT_ACTION_ENABLED resolves to 1 (or nonzero) for THIS build - phase/zero-cross fault CAN trip and beep 2x."
#else
#warning "BUILD CHECK: ZC_FAULT_ACTION_ENABLED resolves to 0 for THIS build - phase/zero-cross fault action is compiled OUT, cannot beep 2x from that path."
#endif

/* relay1_close_armed is declared earlier (ahead of Fault_Shutdown_Latch_Fast)
   so a fault can cancel a pending scheduled closure. Arm it here to close
   RELAY1 (SH-RLY) on the very next zero-crossing edge, instead of at an
   arbitrary point in the AC cycle. Cleared automatically once the edge
   fires and the relay is commanded closed. */

static inline void ZeroCross_CheckAndUpdate(uint8_t is_rising)
{
    uint16_t now     = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
    uint16_t half_us = (uint16_t)(now - zc_last_edge_time_us);  // uint16 wrap-safe
    uint8_t  bad     = 0;

    /* Startup grace period: just record history, don't judge it yet.
       Power-on/rearm relay bounce and inrush noise are expected here and
       are not a real phase fault. */
    if (zc_startup_counter < ZC_STARTUP_GRACE_EDGES)
    {
        zc_startup_counter++;
        zc_last_halfcycle_us    = half_us;
        zc_last_edge_time_us    = now;
        zc_last_edge_was_rising = is_rising;
        if (zc_edge_index < 3) zc_edge_index++;
        return;
    }

    if (zc_edge_index >= 1)
    {
        /* Same edge polarity twice in a row (a missed edge) or an extra
           unexpected edge appearing => a rising or falling edge was
           skipped/duplicated - a hard, unambiguous phase-change/loss
           signature. Trip immediately, no debounce. */
        if (zc_last_edge_was_rising == is_rising)
        {
            bad = 1;
        }
        else if (zc_edge_index >= 2)
        {
            /* Half-cycle time outside the 50Hz window (nominal ~10ms).
               Trip immediately, no debounce. */
            if (half_us < ZC_HALFCYCLE_MIN_US || half_us > ZC_HALFCYCLE_MAX_US)
            {
                bad = 1;
            }
            else if (zc_edge_index >= 3)
            {
                /* Sudden jump vs the previous half-cycle. Trip immediately too. */
                uint16_t diff = (half_us > zc_last_halfcycle_us)
                                  ? (uint16_t)(half_us - zc_last_halfcycle_us)
                                  : (uint16_t)(zc_last_halfcycle_us - half_us);
                if (diff > ZC_SUDDEN_DELTA_US) bad = 1;
            }
        }
    }

    if (bad)
    {
#if ZC_FAULT_ACTION_ENABLED
        phase_seq_fault = 1;
        zc_bad_edge_count = 0;
        zc_good_streak = 0;
        gLastFaultSource = FAULT_SOURCE_PHASE_ZC;
        Fault_Shutdown_Latch_Fast();     // reuse existing safe shutdown path
#else
        /* Software trip temporarily disabled - detection still runs
           (visible here if you add a debug counter/breakpoint) but does
           not shut anything down. Treat this edge as "bad" for the
           streak bookkeeping only. */
        zc_bad_edge_count = 0;
        zc_good_streak = 0;
#endif
    }
    else
    {
        zc_bad_edge_count = 0;
        if (zc_good_streak < 0xFF) zc_good_streak++;
    }

    zc_last_halfcycle_us    = half_us;
    zc_last_edge_time_us    = now;
    zc_last_edge_was_rising = is_rising;
    if (zc_edge_index < 3) zc_edge_index++;
}

/* ---- Predictive (lead-compensated) closure of RELAY1 (SH-RLY) ----
 *
 * Reactive closing (relay1_close_armed, fires on whatever edge comes
 * next) always lands the actual contact closure AFTER the true zero
 * crossing, by however long the relay's coil-energize-to-contact-close
 * (pull-in) delay is - typically several ms, which is a large fraction
 * of a 10ms half-cycle.
 *
 * This instead predicts when the NEXT zero-crossing will occur (from
 * the running half-cycle measurement already tracked above), and
 * schedules a TIM2 output-compare interrupt to fire the close command
 * that many microseconds EARLY, so the contacts are aimed to land ON
 * the zero crossing instead of trailing behind it.
 *
 * RELAY1_PULLIN_DELAY_US is an ESTIMATE, not a measured value - verify
 * it on a scope (time from the close command to actual current start)
 * and correct it for your specific relay. Getting this wrong doesn't
 * cause a hazard, it just under- or over-compensates the lead time.
 */
#define RELAY1_PULLIN_DELAY_US      6000U   // ESTIMATE ONLY - verify on a scope and correct
#define RELAY1_PREDICT_MIN_LEAD_US   300U   // if predicted lead is closer than this (or already
                                             // passed), don't trust it - fall back to reactive
#define RELAY1_PREDICT_MAX_LEAD_US 20000U   // sanity ceiling - a bigger "lead" than this means
                                             // the math likely wrapped around; fall back instead

/* relay1_predictive_pending is declared earlier (ahead of Fault_Shutdown_Latch_Fast). */

static inline void Relay1_ArmClose(void)
{
    /* Need at least 2 real edges of history to have a valid half-cycle
       measurement to predict from. */
    if (zc_edge_index >= 2)
    {
        uint16_t now           = (uint16_t)__HAL_TIM_GET_COUNTER(&htim2);
        uint16_t predicted_edge = (uint16_t)(zc_last_edge_time_us + zc_last_halfcycle_us);
        uint16_t target         = (uint16_t)(predicted_edge - RELAY1_PULLIN_DELAY_US);
        uint16_t lead           = (uint16_t)(target - now);   // wrap-safe time-until-target

        if (lead >= RELAY1_PREDICT_MIN_LEAD_US && lead <= RELAY1_PREDICT_MAX_LEAD_US)
        {
            __disable_irq();
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, target);
            relay1_predictive_pending = 1;
            __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC2);
            __enable_irq();
            return;
        }
    }

    /* Not enough history yet, or this cycle's prediction isn't
       trustworthy - fall back to the simple reactive close-on-next-edge. */
    relay1_close_armed = 1;
}

/* Called from TIM2_IRQHandler in stm32g0xx_it.c */
void Relay1_TIM2_CC_IRQHandler(void)
{
    if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_CC2) != RESET &&
        __HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC2) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);
        __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);
        relay1_predictive_pending = 0;
        RELAY1_SET();
    }
}

static inline uint8_t BKIN_FaultActive(void)
{
    /* Comparator output is ACTIVE LOW:
       LOW  = fault present
       HIGH = fault cleared
    */

    uint8_t tim1_fault  = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) ? 1U : 0U;
    uint8_t tim15_fault = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)  == GPIO_PIN_RESET) ? 1U : 0U;

    return (uint8_t)(tim1_fault || tim15_fault);
}


/* Rearm is held off while temperature is still excessive or the input is above
 * the HV trip limit. (Deliberately NOT gated on low/stale input: ACin is measured
 * with the relays open, but gating rearm on it could deadlock the recovery.) */
static inline uint8_t Rearm_Blocked_By_Protection(void)
{
#if LOCAL_OVERTEMP_TRIP_ENABLED
    if (excessheatframe)
        return 1U;
#endif
#if LOCAL_VOLTAGE_LIMITS_ENABLED
    if (vrms_in > Input_HV_Limit())
        return 1U;
#endif
    return 0U;
}

/* Main-loop protection: stale-measurement lockout, output over-voltage,
 * regulated-output window (230 V +/- 10 %) and over-temperature. Input over/under-voltage is enforced in
 * update_rectifier_allowed() (runs from the zero-cross ISR, so it is fast). */
static void Protection_Service(void)
{
    static uint32_t last_seq     = 0U;
    static uint8_t  ov_count     = 0U;
    static uint8_t  ot_timing    = 0U;
    static uint32_t ot_start_ms  = 0U;
    static uint8_t  win_armed    = 0U;
    static uint32_t win_since_ms = 0U;
    static uint8_t  win_ov       = 0U;
    static uint8_t  win_uv       = 0U;

    uint32_t seq        = meas_seq;
    uint8_t  new_sample = (seq != last_seq) ? 1U : 0U;
    uint32_t now        = HAL_GetTick();

    last_seq = seq;
    (void)new_sample;
    (void)now;
    (void)win_armed; (void)win_since_ms; (void)win_ov; (void)win_uv;

    if (gFaultLatched)
    {
        ov_count  = 0U;
        ot_timing = 0U;
        win_armed = 0U;
        win_ov    = 0U;
        win_uv    = 0U;
        return;
    }

    /* 1) Measurements stopped arriving: never keep running on old data. */
    if ((pwm_allowed || rectifier_allowed) && !Measurement_IsFresh())
    {
        __disable_irq();
        pwm_allowed       = 0U;
        rectifier_allowed = 0U;
        Stop_All_PWM();
        __enable_irq();
        if (g_diag.stale_lockouts < 65535U) g_diag.stale_lockouts++;
        return;
    }

#if LOCAL_OUTPUT_OV_ENABLED
    /* 2) Gross output over-voltage, evaluated once per fresh measurement. */
    if (pwm_allowed)
    {
        if (new_sample)
        {
            if (vrms_out > OUTPUT_OV_LIMIT_V)
            {
                if (ov_count < 255U) ov_count++;
            }
            else
            {
                ov_count = 0U;
            }
        }

        if (ov_count >= OUTPUT_OV_TRIP_SAMPLES)
        {
            ov_count = 0U;
            diag_next_reason = DIAG_R_OUT_GROSS_OV;
            gLastFaultSource = FAULT_SOURCE_SW_PROTECT;
            Fault_Shutdown_Latch_Fast();
            return;
        }
    }
    else
    {
        ov_count = 0U;
    }
#endif

#if LOCAL_OUTPUT_WINDOW_ENABLED
    /* 2b) Regulated-output window, nominal +/- 10 %. Armed only while the output
       is really regulated; any interruption (contactor open, load soft-start
       restarting, PWM off) disarms and restarts the settling time. */
    if (pwm_allowed && CONT1_ON && !load_softstart_active)
    {
        if (!win_armed)
        {
            win_armed    = 1U;
            win_since_ms = now;
            win_ov       = 0U;
            win_uv       = 0U;
        }
        else if ((uint32_t)(now - win_since_ms) >= OUTPUT_WIN_ARM_MS)
        {
            if (new_sample)
            {
                if (vrms_out > OUTPUT_WIN_HI_V)
                {
                    if (win_ov < 255U) win_ov++;
                }
                else
                {
                    win_ov = 0U;
                }

                /* written as !(>=) so a NaN reading counts as undervoltage */
                if (!(vrms_out >= OUTPUT_WIN_LO_V))
                {
                    if (win_uv < 255U) win_uv++;
                }
                else
                {
                    win_uv = 0U;
                }
            }

            if ((win_ov >= OUTPUT_WIN_OV_SAMPLES) || (win_uv >= OUTPUT_WIN_UV_SAMPLES))
            {
                diag_next_reason = (win_ov >= OUTPUT_WIN_OV_SAMPLES) ? DIAG_R_OUT_WIN_OV : DIAG_R_OUT_WIN_UV;
                win_ov = 0U;
                win_uv = 0U;
                win_armed = 0U;
                gLastFaultSource = FAULT_SOURCE_SW_PROTECT;
                Fault_Shutdown_Latch_Fast();
                return;
            }
        }
    }
    else
    {
        win_armed = 0U;
        win_ov    = 0U;
        win_uv    = 0U;
    }
#endif

#if LOCAL_OVERTEMP_TRIP_ENABLED
    /* 3) Sustained over-temperature. */
    if (excessheatframe && (pwm_allowed || rectifier_allowed))
    {
        if (!ot_timing)
        {
            ot_timing   = 1U;
            ot_start_ms = now;
        }
        else if ((uint32_t)(now - ot_start_ms) >= OVERTEMP_TRIP_MS)
        {
            ot_timing = 0U;
            diag_next_reason = DIAG_R_OVERTEMP;
            gLastFaultSource = FAULT_SOURCE_SW_PROTECT;
            Fault_Shutdown_Latch_Fast();
            return;
        }
    }
    else
    {
        ot_timing = 0U;
    }
#else
    (void)ot_timing;
    (void)ot_start_ms;
#endif
}

/* Input-window disconnect / reconnect. Mirrors what the master already does:
 *   disconnect = the [F] command  (state 0, relay2_on 0, W-RLY opens after 1 s;
 *                                  the main loop already handles that timer)
 *   reconnect  = the [C] command  (predictive SH-RLY close + c_sequence)
 * Acts only on fresh data. The converter itself is stopped much faster by
 * update_rectifier_allowed() in the zero-cross ISR. */
static void Input_Window_Service(void)
{
#if LOCAL_VOLTAGE_LIMITS_ENABLED
    static uint32_t last_seq       = 0U;
    static uint8_t  bad_count      = 0U;
    static uint8_t  good_timing    = 0U;
    static uint32_t good_since_ms  = 0U;

    uint32_t seq        = meas_seq;
    uint8_t  new_sample = (seq != last_seq) ? 1U : 0U;
    uint32_t now        = HAL_GetTick();
    float    lv         = Input_LV_Limit();
    float    hv         = Input_HV_Limit();

    last_seq = seq;

    if (!Measurement_IsFresh())
    {
        bad_count   = 0U;
        good_timing = 0U;
        return;               /* stale data is handled by Protection_Service() */
    }

    if (!input_tripped)
    {
        good_timing = 0U;

        /* Only while connected and not in the middle of a connect sequence. */
        if (gbIsSystemNormalState == 1U && !c_sequence_active)
        {
            if (new_sample)
            {
                if (vrms_in >= lv && vrms_in <= hv)
                    bad_count = 0U;
                else if (bad_count < 255U)
                    bad_count++;
            }

            if (bad_count >= INPUT_TRIP_SAMPLES)
            {
                bad_count = 0U;

                /* Same actions as the master's [F] command. */
                gbIsSystemNormalState = 0U;
                relay2_on             = 0U;
                c_relay_close_time    = now;
                c_relay_close_pending = 1U;

                input_tripped = 1U;

                if (g_diag.input_disconnects < 65535U) g_diag.input_disconnects++;
                g_diag.last_disconnect_vin = Diag_Round16(vrms_in);
            }
        }
        else
        {
            bad_count = 0U;
        }
        return;
    }

    /* We disconnected: wait for a healthy input, then reconnect like [C]. */
    bad_count = 0U;
#if LOCAL_INPUT_AUTO_RECONNECT
    if (Input_Voltage_Permit(0U, vrms_in, lv, hv))
    {
        if (!good_timing)
        {
            good_timing   = 1U;
            good_since_ms = now;
        }
        else if (((uint32_t)(now - good_since_ms) >= INPUT_RECONNECT_HOLD_MS) &&
                 !c_relay_close_pending && !c_sequence_active && !gFaultLatched &&
                 gbIsSystemNormalState == 0U)
        {
            good_timing = 0U;
            input_tripped = 0U;

            /* Same actions as the master's [C] command. */
            Relay1_ArmClose();
            c_sequence_time   = now;
            c_sequence_step   = 1U;
            c_sequence_active = 1U;
        }
    }
    else
    {
        good_timing = 0U;
    }
#endif
#endif
}
#if 0
static void Fault_ReArm_Service(void)
{
    static uint32_t MOE_enabl_ctr = 0U;

    if (!gFaultLatched)
    {
        MOE_enabl_ctr = 0U;
        return;
    }

    /* Fault still active -> keep waiting */
    if (BKIN_FaultActive())
    {
        MOE_enabl_ctr = 0U;
        return;
    }

    /* Fault became inactive -> start stability timer */
    if (MOE_enabl_ctr == 0U)
    {
        MOE_enabl_ctr = HAL_GetTick();
        return;
    }

    /* Need continuous fault-free window */
    if ((HAL_GetTick() - MOE_enabl_ctr) < FAULT_REARM_DELAY_MS)
    {
        return;
    }

    /* --- Safe software re-arm sequence --- */
    __disable_irq();

    /* Keep duty at zero during re-arm */
    TIM1->CCR1  = 0;
    TIM15->CCR1 = 0;

    /* Keep permissions reset initially */
    pwm_allowed       = 0;
    rectifier_allowed = 0;

    /* Restore state flags */
    con_state = 1;
    gbIsSystemNormalState = 1;

#if 0
    /* Reset PID state for clean resume */
    voltage_setpoint_pid = vrms_in;
    pid.integral   = 0.0f;
    pid.derivative = 0.0f;
    pid.lastError  = voltage_setpoint_pid - vrms_out;
    pid.lastOutput = 0.0f;
    pid.lastTime   = HAL_GetTick();
#endif

    /* Clear any stale update flag on TIM14 */
    TIM14->SR &= ~TIM_SR_UIF;

    /* Re-enable timer counters first */
    TIM1->CR1  |= TIM_CR1_CEN;
    TIM15->CR1 |= TIM_CR1_CEN;

    /* Re-enable channel outputs */
    TIM1->CCER  |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    TIM15->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);

    /* Re-enable MOE last */
    TIM1->BDTR  |= TIM_BDTR_MOE;
    TIM15->BDTR |= TIM_BDTR_MOE;

    /* Clear software fault latch only after the hardware path is restored */
    gFaultLatched = 0;

    __DMB();
    __enable_irq();

    MOE_enabl_ctr = 0U;
}
#endif
static void Fault_ReArm_Service(void)
{
    static uint32_t rearm_start_time = 0U;
    static uint32_t relay1_step_time = 0U;
    static uint8_t relay1_rearm_step = 0U;
    uint32_t now;
    /*
     * No software-latched fault:
     * Reset all local rearm states.
     */
    if (!gFaultLatched)
    {
        rearm_start_time = 0U;
        relay1_step_time = 0U;
        relay1_rearm_step = 0U;
        return;
    }
    /*
     * Physical BKIN fault is still active.
     *
     * Do not start the rearm timer or Relay 1 sequence.
     * If BKIN reappears during recovery, restart everything.
     */
    if (BKIN_FaultActive() || Rearm_Blocked_By_Protection() ||
        (phase_seq_fault && zc_good_streak < ZC_HEALTHY_STREAK))
    {
        rearm_start_time = 0U;
        relay1_step_time = 0U;
        relay1_rearm_step = 0U;
        /* Keep Relay 1 reset while the fault is active, and cancel any
           scheduled close so it can't fire while we're forcing it open. */
        Relay1_CancelPendingClose();
        RELAY1_RESET();
        RELAY2_RESET();
        return;
    }
    now = HAL_GetTick();
    /*
     * BKIN has become inactive.
     *
     * Start the 2-second continuous fault-free timer and
     * the initial 100 ms waiting period simultaneously.
     */
    if (rearm_start_time == 0U)
    {
        rearm_start_time = now;
        relay1_step_time = now;
        relay1_rearm_step = 1U;
        /* Relay 1 stays reset during the initial 100 ms */
        RELAY1_RESET();
        RELAY2_RESET();
        return;
    }
    /*
     * Relay 1 rearm sequence:
     *
     * Step 1: Wait 100 ms after BKIN fault clearance.
     * Step 2: Set Relay 1 and hold it set for 500 ms.
     * Step 3: Reset Relay 1 and wait another 100 ms.
     * Step 4: Relay sequence completed.
     */
    if (relay1_rearm_step == 1U)
    {
        /* Initial 100 ms wait after BKIN becomes inactive */
        if ((uint32_t)(now - relay1_step_time) >= 1000U)
        {
            Relay1_ArmClose();    // predictive close, lead-compensated for pull-in delay
            relay1_step_time = now;
            relay1_rearm_step = 2U;
        }
    }
    else if (relay1_rearm_step == 2U)
    {
        /* Hold Relay 1 (SH-RLY) set for ~1s to precharge, then hand off to
           Relay 2 (W-RLY) in the same instant - no gap for the caps to
           bleed back down through R92 before the main relay closes. */
        if ((uint32_t)(now - relay1_step_time) >= 1000U)
        {
            RELAY1_RESET();
            RELAY2_SET();
            relay1_step_time = 0U;
            relay1_rearm_step = 4U;
        }
    }
    /*
     * Continue waiting until the complete 2-second continuous
     * fault-free period has elapsed.
     */
    if ((uint32_t)(now - rearm_start_time) < FAULT_REARM_DELAY_MS)
    {
        return;
    }
    /*
     * Additional interlock:
     * Do not rearm unless the complete Relay 1 sequence
     * has finished.
     */
    if (relay1_rearm_step != 4U)
    {
        return;
    }
    /* ---------------------------------------------------------
     * SAFE SOFTWARE REARM AFTER 2-SECOND FAULT-FREE WINDOW
     * --------------------------------------------------------- */
    __disable_irq();
    /* Ensure Relay 1 remains reset before restarting */
    RELAY1_RESET();
    /* Keep inverter duty at zero during timer rearm */
    TIM1->CCR1 = 0U;
    TIM15->CCR1 = 0U;
    inverter_startup_active = 0U;
    inverter_startup_begin_ms = 0U;
    pwm_output = 0.0f;
    pid.integral = 0.0f;
    pid.derivative = 0.0f;
    pid.lastOutput = 0.0f;
    /* Keep inverter and rectifier permissions cleared initially */
    pwm_allowed = 0U;
    rectifier_allowed = 0U;
    /*
     * Force a fresh rectifier-to-inverter startup sequence.
     * The 3-second delay starts only after the rectifier
     * physically starts at a valid zero crossing.
     */
    rectifier_to_inverter_delay_active = 0U;
    rectifier_on_time = 0U;
    /* Restore system state so the rectifier can restart.
     * NOTE: previously commented out - this left gbIsSystemNormalState
     * stuck at 0 after a BKIN/inrush fault, which blocks rectifier_allowed
     * and pwm_allowed forever (both require gbIsSystemNormalState == 1),
     * so the unit would clear the fault latch but never actually restart
     * unless the master separately sent a [C] command.
     * (con_state is not used here: its declaration is commented out
     * elsewhere in this file, so it isn't a live variable any more.) */
    gbIsSystemNormalState = 1U;
    Inrush_present = 0;
    /* Clear stale TIM14 update event */
    TIM14->SR &= ~TIM_SR_UIF;
    /* Restart TIM1 and TIM15 counters */
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM15->CR1 |= TIM_CR1_CEN;
    /*
     * Keep main and complementary inverter channels disabled.
     * They will be enabled after the existing 3-second delay.
     */
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    TIM15->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE);
    /*
     * Restore Main Output Enable.
     * PWM pins remain inactive because channel-enable bits
     * CC1E and CC1NE are still cleared.
     */
    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM15->BDTR |= TIM_BDTR_MOE;

    /* Clear the software fault latch last */
    phase_seq_fault    = 0;
    zc_startup_counter = 0;   // fresh grace period for the next restart
    zc_edge_index      = 0;
    zc_bad_edge_count  = 0;
    zc_good_streak     = 0;
    gLastFaultSource   = FAULT_SOURCE_NONE;
    gFaultLatched = 0U;
    __DMB();
    __enable_irq();
    /* Reset local variables for the next fault event */
    rearm_start_time = 0U;
    relay1_step_time = 0U;
    relay1_rearm_step = 0U;
}

static void PwmStatusHold_Service(void)
{
    static uint32_t pwm_status_release_tmr = 0U;

    if (!gPwmStatusHold)
    {
        pwm_status_release_tmr = 0U;
        return;
    }

    /* Still faulted or not yet re-armed -> keep reporting forced 1 */
//    if (gFaultLatched || (con_state == 0) || (gbIsSystemNormalState == 0))
    if (gFaultLatched || gbIsSystemNormalState == 0)
    {
        pwm_status_release_tmr = 0U;
        return;
    }

    /* After recovery, wait until real pwm_allowed becomes 1 */
    if (pwm_allowed == 1U)
    {
        if (pwm_status_release_tmr == 0U)
        {
            pwm_status_release_tmr = HAL_GetTick();
            return;
        }

        if ((HAL_GetTick() - pwm_status_release_tmr) >= PWM_STATUS_RELEASE_DELAY_MS)
        {
            gPwmStatusHold = 0U;
            pwm_status_release_tmr = 0U;
        }
    }
    else
    {
        /* Still not fully settled */
        pwm_status_release_tmr = 0U;
    }
}

#define TX_BUF_SIZE 128

uint8_t uart1_tx_dma[TX_BUF_SIZE];
volatile uint8_t uart1_tx_busy = 0;

void SendPhaseStatus(char phase)
{
    if (uart1_tx_busy)
        return;

    if (gPwmStatusHold)
    	pwm_allowed_Cont = 1;
    else
    	pwm_allowed_Cont = pwm_allowed;

//    int len = snprintf((char*)uart1_tx_dma,
//                       TX_BUF_SIZE,
//                       "[%c]%d,%d,%d,%d,%d\r\n",
//                       phase,
//                       (int)vrms_in,
//                       (int)vrms_out,
//					   pwm_allowed,
//                       excessheatframe,
//                       gFaultLatched_dummy);   // <-- NEW: last field

        int len = snprintf((char*)uart1_tx_dma,
                           TX_BUF_SIZE,
                           "[%c]%d,%d,%d,%d,%d\r\n",
                           phase,
                           (int)vrms_in,
                           (int)vrms_out,
    					   pwm_allowed,
                           excessheatframe,
						   gFaultdisplay);   // <-- NEW: last field

    if (len <= 0) return;

    uart1_tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart1, uart1_tx_dma, len);
}

///////////////////////////////////////////////////////

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1 || htim->Instance == TIM15)
    {

		if(Mreceivedflag_pulse_on == 1)
		{
			diag_next_reason = DIAG_R_BKIN_HW;
			gLastFaultSource = FAULT_SOURCE_BKIN_HW;
			Fault_Shutdown_Latch_Fast();
		}

    }
}

void AutoPID_Init(AutoPID *pid, volatile float *input, volatile float *setpoint, volatile float *output,
                  float Kp, float Ki, float Kd, float outputMin, float outputMax,
                  uint32_t timeStepMs)
{
    pid->input = input;
    pid->setpoint = setpoint;
    pid->output = output;

    // Set gains (Ki here is in units of output-per-volt-per-second)
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    // actuator limits
    pid->outputMin = outputMin;
    pid->outputMax = outputMax;

    // integrator limits and state
    pid->integral = 0.0f;
    pid->integralMin = outputMin; // prevent windup (tune if needed)
    pid->integralMax = outputMax;

    // derivative & filtering state
    pid->derivative = 0.0f;
    pid->lastError = (*(setpoint) - *(input));
    pid->derivFilterAlpha = 0.90f; // 0.85-0.95 recommended

    // RMS (measurement) smoothing state
    pid->measFiltered = *input;
    pid->rmsFilterAlpha = 0.95f; // 0.9-0.98 recommended

    // timing & enabling
    pid->lastTime = HAL_GetTick();
    pid->timeStepMs = timeStepMs;
    pid->enabled = 1;

    // initialize lastOutput to current output value
    pid->lastOutput = *output;
}

void AutoPID_Compute(AutoPID *pid)
{
    if (!pid->enabled) return;

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - pid->lastTime;
    if (elapsed < pid->timeStepMs) return;
    float dt = elapsed / 1000.0f;

    // const float freeze_min = *(pid->setpoint) - 0.5f;
     const float freeze_min = *(pid->setpoint) - 0.75f;
     const float freeze_max = *(pid->setpoint) + 0.75f;

    static bool was_in_freeze = false;
    static uint32_t freeze_pending_since_ms = 0U;
    static bool freeze_pending_state = false;

    bool raw_in_band = (vrms_in >= freeze_min && vrms_in <= freeze_max);
    if (was_in_freeze)
    {
		raw_in_band = (vrms_in >= (freeze_min - 0.75f) && vrms_in <= (freeze_max + 0.75f));
	}

    /* Debounce the freeze/unfreeze transition. Without this, vrms_in
     * sitting close to the boundary (real mains noise/ripple, ADC/RMS
     * measurement jitter) can cross it every few samples, repeatedly
     * snapping pwm_output between 0 and active regulation - seen as
     * continuous "regulating -> pulses off -> regulating" cycling. The
     * raw condition must hold steady for FREEZE_DEBOUNCE_MS before the
     * actual freeze state is allowed to change; until then, keep
     * whatever state was already in effect. */
    bool in_freeze_band = was_in_freeze;
    if (raw_in_band != was_in_freeze)
    {
        if ((freeze_pending_since_ms == 0U) || (freeze_pending_state != raw_in_band))
        {
            freeze_pending_since_ms = now;
            freeze_pending_state    = raw_in_band;
        }
        else if ((now - freeze_pending_since_ms) >= FREEZE_DEBOUNCE_MS)
        {
            in_freeze_band = raw_in_band;   /* confirmed - take effect now */
        }
    }
    else
    {
        freeze_pending_since_ms = 0U;   /* raw condition matches current state already */
    }

    if (in_freeze_band)
    {
    	*(pid->output) = 0.0f;
        pid->integral = 0.0f;
         pid->derivative = 0.0f;
        pid->lastTime = now;
        was_in_freeze = true;
        pid_freeze_active = 1U;   /* TEMPORARY bench-diagnostic observability */
        // Leave pid->integral, pid->derivative and pid->lastError untouched for smoother resume.
        return;
    }

    was_in_freeze = false;
    pid_freeze_active = 0U;   /* TEMPORARY bench-diagnostic observability */
    // --- normal PID processing (not frozen)

    // compute soft mode blending 't' around setpoint
    float band = 0.2f;
    float t = 0.5f * (1.0f + (vrms_in - *(pid->setpoint)) / band);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    // gains interpolation
//    float Kp_boost = ratio_bb * 2.0f, Ki_boost = ratio_bb * (1.5f * 2600.0f), Kd_boost = 1.0f;
//    float Kp_buck  = ratio_bb * 1.0f, Ki_buck  = ratio_bb * (1.5f * 2000.0f) , Kd_buck  = 0.6f;

//    float Kp_boost = ratio_boost * 2.0f, Ki_boost = ratio_boost * (1.5f * 2600.0f), Kd_boost = 1.0f;  1.8
//    float Kp_buck  =  ratio_buck * 1.0f, Ki_buck  = ratio_buck * (1.5f * 2000.0f) , Kd_buck  = 0.6f;  2.3

    float Kp_boost = ratio_boost * 2.0f, Ki_boost = ratio_boost * (1.5f * 2600.0f), Kd_boost = 1.0f;  // 1.2
    float Kp_buck  =  ratio_buck * 1.0f, Ki_buck  = ratio_buck * (1.5f * 2000.0f) , Kd_buck  = 0.6f;  // 2.3

    float d_alpha = 0.9f;

    pid->Kp = (1.0f - t) * Kp_boost + t * Kp_buck;
    pid->Ki = (1.0f - t) * Ki_boost + t * Ki_buck;
    pid->Kd = (1.0f - t) * Kd_boost + t * Kd_buck;

    // error calculation (do NOT force input to setpoint)
    float boostErr = *(pid->setpoint) - *(pid->input);
    float buckErr  = *(pid->input)   - *(pid->setpoint);
    float error_raw = (1.0f - t) * boostErr + t * buckErr;
    float error = error_raw;

    // derivative (filtered)
    float deriv_unf = (error - pid->lastError) / dt;
    pid->derivative = d_alpha * pid->derivative + (1.0f - d_alpha) * deriv_unf;
    // compute unclamped output for anti-windup check
    float out_unclamped = pid->Kp * error + pid->integral + pid->Kd * pid->derivative;

    // conditional integration (anti-windup)
    if (!((out_unclamped > pid->outputMax && error > 0.0f) ||
          (out_unclamped < pid->outputMin && error < 0.0f)))
    {
        pid->integral += pid->Ki * error * dt;
    }
    else
    {
        // small leak to avoid permanent wind-up
        pid->integral *= 0.999f;
    }

    // clamp integral
    if (pid->integral > pid->integralMax) pid->integral = pid->integralMax;
    if (pid->integral < pid->integralMin) pid->integral = pid->integralMin;
    // final output (signed)
    float out = pid->Kp * error + pid->integral + pid->Kd * pid->derivative;
    // clamp final output to allowed range (preserve sign)
    if (out > pid->outputMax) out = pid->outputMax;
    if (out < pid->outputMin) out = pid->outputMin;

    // Store output and remember it for freeze-holding
    *(pid->output) = out;

    pid->lastOutput = out;    // <-- IMPORTANT: persist the exact last value (signed)
    pid->lastError = error;
    pid->lastTime = now;

}

////////////////ADC changes for 45Hz to 65Hz Range/////////////////
///////////////////////////////////////////////////////////////////
#define NUM_CHANNELS    3
#define NUM_SAMPLES     1024   // was 266; 512 triplets safely covers 45–65 Hz at ~13.33 kHz
#define TOTAL_WORDS    (NUM_SAMPLES * NUM_CHANNELS)

uint16_t adc_dma_buffer[TOTAL_WORDS];


static volatile uint16_t zc_prev_ndtr = 0, zc_curr_ndtr = 0;
static volatile uint8_t  cycle_ready  = 0;
static volatile uint8_t zc_seen_once = 0;


// Convert NDTR to triplet index (no modulo/div in ISR)
static inline uint32_t ndtr_to_triplet_idx(uint16_t ndtr_words)
{
    // TOTAL_WORDS = NUM_SAMPLES * NUM_CHANNELS
    uint32_t wr_words = (TOTAL_WORDS - ndtr_words);
    // If NUM_SAMPLES is power-of-two (e.g., 512), you can mask triplet index:
    // return (wr_words / NUM_CHANNELS) & (NUM_SAMPLES - 1);

    // Otherwise, plain divide is fine here (not in ISR):
    return (wr_words / NUM_CHANNELS) % NUM_SAMPLES;
}

static float Read_AC_Voltage_RMS_DMA(uint32_t start_triplet, uint32_t end_triplet, uint8_t channel_index)
{
    uint32_t count = (end_triplet >= start_triplet)
                   ? (end_triplet - start_triplet)
                   : (NUM_SAMPLES - start_triplet + end_triplet);

    if (count < 8) return 0.0f;  // guard

    double sum_sq = 0.0;
    const double k = 3.3 / 4095.0;

    for (uint32_t i = 0; i < count; ++i)
    {
        uint32_t trip = (start_triplet + i) & (NUM_SAMPLES - 1); // NUM_SAMPLES is power-of-2
        uint16_t raw = adc_dma_buffer[trip * NUM_CHANNELS + channel_index];

        double v = raw * k;
        v -= (double)adcoffset;  // keep your offset method
        sum_sq += v * v;
    }

    double mean_sq = sum_sq / (double)count;
    //return (float)sqrt(mean_sq);

	double vrms = sqrtf(mean_sq);

    if (channel_index == 0) // ADC_CHANNEL_0 = ACin
    {
    	vrms =(421.61*powf(vrms,4) - 851.4*powf(vrms,3) + 597.09*powf(vrms,2) + 154.9*vrms + 14.7); // New board B2-1
    	ac_in_voltage = vrms ;
    }
    else if (channel_index == 1) // ADC_CHANNEL_1 = ACout
    {
   	    vrms = 116.488577*powf(vrms,2) + 151.158079*vrms + 63.0; // New board B2-1 and B2 - 2
//   	    vrms = 116.488577*powf(vrms,2) + 151.158079*vrms + 61.5; // New board B2-3
        ac_out_voltage = vrms;
    }
    else if (channel_index == 2) // ADC_CHANNEL_2 = Current
    {
     	vrms =(-16488.265*powf(vrms,4) + 9275.894 *powf(vrms,3) - 1925.228*powf(vrms,2) + 209.500*vrms - 5.7775);
    	ac_crt = vrms;
    }
    return (float)vrms;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart1_tx_busy = 0;
    }
}

#if 0
void SendPhaseStatus(char phase)
{
    if (uart1_tx_busy)
        return;

    int len = snprintf((char*)uart1_tx_dma,
                       TX_BUF_SIZE,
                       "[%c]%d,%d,%d,%d,%d\r\n",
                       phase,
                       (int)vrms_in,
                       (int)vrms_out,
                       pwm_allowed,
                       excessheatframe,
                       gFaultLatched);   // <-- NEW: last field

    if (len <= 0) return;

    uart1_tx_busy = 1;
    HAL_UART_Transmit_DMA(&huart1, uart1_tx_dma, len);
}
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM17)
	{
//		if (phasecame && Mreceivedflag_pulse_on) // only send if a phase was received
		if(Mreceivedflag_pulse_on)
		{
		      SendPhaseStatus(currentPhase);
		 }
	}
    if (htim->Instance == TIM4)
    {
        if (gFaultLatched)
        {
            Rectifier_AllOff_Immediate();
            return;
        }

    	AutoPID_Compute(&pid);
    }

    if (htim->Instance == TIM7)
	{
	  buzzer_state ^= 1;
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4,buzzer_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}

}

int temp = 1;


void TIM1_TIM15_UpdateParameters(uint16_t prescaler, uint8_t deadtime)
{
    // Disable timer and outputs before modifying
    __HAL_TIM_DISABLE(&htim1);
    __HAL_TIM_MOE_DISABLE(&htim1);  // Disable main output enable (MOE)
    __HAL_TIM_DISABLE(&htim15);
    __HAL_TIM_MOE_DISABLE(&htim15);  // Disable main output enable (MOE)

    // --- Update Prescaler and fixed ARR ---
    TIM1->PSC = prescaler;
    TIM1->ARR = TIM1_TIM15_FIXED_ARR;
    TIM1->EGR = TIM_EGR_UG;  // Apply new PSC and ARR immediately

    TIM15->PSC = prescaler;
     TIM15->ARR = TIM1_TIM15_FIXED_ARR;
     TIM15->EGR = TIM_EGR_UG;  // Apply new PSC and ARR immediately

    // --- Update Dead Time only (DTG bits [7:0]) ---
    TIM1->BDTR = (TIM1->BDTR & ~TIM_BDTR_DTG_Msk) | (deadtime & TIM_BDTR_DTG_Msk);
    TIM15->BDTR = (TIM15->BDTR & ~TIM_BDTR_DTG_Msk) | (deadtime & TIM_BDTR_DTG_Msk);


    // Re-enable outputs and timer
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_ENABLE(&htim1);
    __HAL_TIM_MOE_ENABLE(&htim15);
    __HAL_TIM_ENABLE(&htim15);
}
//void Start_PWM_Buzzer(void)
//{
//    buzzer_state = 0;
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
//    HAL_TIM_Base_Start_IT(&htim7); // Start 125 µs toggling (4 kHz, 50% duty)
//}
//
//void Stop_PWM_Buzzer(void)
//{
//    HAL_TIM_Base_Stop_IT(&htim7);
//    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
//    buzzer_state = 0;
//}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_FLAG(huart,
                              UART_CLEAR_NEF |
                              UART_CLEAR_FEF |
                              UART_CLEAR_OREF);

        HAL_UART_AbortReceive(huart);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}

/* UART RX Idle DMA Callback ------------------------------------------------*/
/* UART RX Idle DMA Callback ------------------------------------------------*/
/* UART RX Idle DMA Callback ------------------------------------------------*/
void Transformer_Test_Start(void);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != USART1)
        return;

    for (uint16_t i = 0; i < Size; i++)
    {
        char c = uart1_rx_dma[i];
        if (c == '\r') continue;

        if (c == '\n')
        {
            lineBuf[lineIndex] = '\0';

            if (lineBuf[0] == '[' && lineBuf[2] == ']')
            {
                char phase = lineBuf[1];

                /* A master [F]/[C]/[A] takes over: drop any pending local reconnect. */
                if (phase == 'F' || phase == 'C' || phase == 'A') input_tripped = 0U;

                if (phase == 'R' || phase == 'Y' || phase == 'B')
                {
                    currentPhase = phase;
                    phasecame = 1;
                }
                else if(phase == 'T')
                {
                     // TRASNDFORMER FUCNTION
                	trasnfomersetflag = 1;


                }
                else if (phase == 'N')
                {
                    settingmenu_completed_flag = 1;
                }
                else if (phase == 'M')
				{
                	Mreceivedflag_pulse_on = 1;
				}
                else if (phase == 'A'&& !runtime_setting_menu)
				{
					gbIsSystemNormalState = 0;
					relay2_on = 0;
                    a_relay_close_time = HAL_GetTick();
                    a_relay_close_pending = 1;
                    runtime_setting_menu = 1;

				}

                else if (phase == 'S')
                {
                    strncpy(sFrameData, &lineBuf[3], LINE_BUF_SIZE - 1);
                    sFrameData[LINE_BUF_SIZE - 1] = '\0';

                    int hv_tmp, lv_tmp, setv_tmp;
                    /* Only the regulated-output setpoint is range-checked (a garbled frame must
                       not be able to command an arbitrary output voltage). HV/LV are accepted as
                       before so the master's handshake can never stall on them. */
                    if (sscanf(sFrameData, "%d,%d,%d", &hv_tmp, &lv_tmp, &setv_tmp) == 3 &&
                        Settings_Setpoint_Valid(setv_tmp))
                    {
                        HV_CUTOFF        = (float)hv_tmp;
                        StartRelay_Volt  = (uint8_t)lv_tmp;
                        voltage_setpoint = (float)setv_tmp;
                        sFrameReceived   = 1;
                    }
                }
                else if (phase == 'I')
                {
                    int vin_tmp;
                    int16_t trim;
                    if (sscanf(&lineBuf[3], "%d", &vin_tmp) == 1 &&
                        Cal_ComputeTrim((float)vin_tmp, vrms_in_raw, &trim))
                    {
                        working_settings.input_v_r = trim;
                        settings_save_pending = 1U;   /* flash is written from the main loop */
                    }
                }
                else if (phase == 'O')
                {
                    int vout_tmp;
                    int16_t trim;
                    if (sscanf(&lineBuf[3], "%d", &vout_tmp) == 1 &&
                        Cal_ComputeTrim((float)vout_tmp, vrms_out_raw, &trim))
                    {
                        working_settings.output_v_r = trim;
                        settings_save_pending = 1U;
                    }
                }
                else if (phase == 'F' && gbIsSystemNormalState == 1)
                {
                    gbIsSystemNormalState = 0;
                    relay2_on = 0;
                    c_relay_close_time = HAL_GetTick();
                    c_relay_close_pending = 1;

                }

                else if (phase == 'C' && gbIsSystemNormalState == 0)
                {
                    // Step 1: PIN 9 ON close relay 1
                 //   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
                	Relay1_ArmClose();    // predictive close, lead-compensated for pull-in delay
                    c_sequence_time = HAL_GetTick();
                    c_sequence_step = 1;
                    c_sequence_active = 1;
                }
            }

            lineIndex = 0;
        }
        else if (lineIndex < LINE_BUF_SIZE - 1)
        {
            lineBuf[lineIndex++] = c;
        }
        else
        {
            lineIndex = 0; // overflow protection
        }
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma, RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

/* Send String DMA ----------------------------------------------------------*/

void UART_SendString_DMA(UART_HandleTypeDef *huart, const char *str)
{
    HAL_UART_Transmit_DMA(huart, (uint8_t *)str, strlen(str));
}

/* Send Phase + Values ------------------------------------------------------*/


void UART_SendPhaseValues(UART_HandleTypeDef *huart,
                          char phase,
                          int hv,
                          int lv,
                          int setv)
{
    snprintf(txBuf, sizeof(txBuf),
             "[%c]%d,%d,%d\r\n",
             phase, hv, lv, setv);

    HAL_UART_Transmit_DMA(huart, (uint8_t*)txBuf, strlen(txBuf));
}

void UART_SendVoltages(UART_HandleTypeDef *huart, float vin, float vout)
{
    // Format the float values into txBuf
    snprintf(txBuf, sizeof(txBuf),
             "[V]%.0f,%.0f\r\n",  // 2 decimal places
             vin, vout);

    HAL_UART_Transmit_DMA(huart, (uint8_t*)txBuf, strlen(txBuf));
}
void SYS_CheckForFaults(void)
{

    ntc     = HAL_GPIO_ReadPin(GPIOA, NTC_Pin);
    if(ntc == 0)
    {
    	  excessheatframe = 1;
    }
    else
    {
    	  excessheatframe = 0;
    }
}
//void SYS_CheckForFaults(void)
//{
//
//    flt_CN7 = HAL_GPIO_ReadPin(GPIOD, FLT_Pin);
//    ntc     = HAL_GPIO_ReadPin(GPIOA, NTC_Pin);
//
//    System_op();
//
//}
//void System_op(void)
//{
//    static uint32_t ntcFaultStartTick = 0;
//    static uint8_t  ntcFaultActive    = 0;
//
//    ntc = HAL_GPIO_ReadPin(GPIOA, NTC_Pin);
//
//    if (ntc == 0)
//    {
//        /* FAN ON immediately */
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//
//        /* 🔴 SET FRAME FLAG = 1 */
//        excessheatframe = 1;
//
//        if (!ntcFaultActive)
//        {
//            ntcFaultStartTick = HAL_GetTick();
//            ntcFaultActive = 1;
//        }
//
//        if ((uint32_t)(HAL_GetTick() - ntcFaultStartTick) >= 10000U)
//        {
//            ntcFaultActive = 0;
//            ntcFaultStartTick = 0;
//        }
//    }
//    else
//    {
//        /* FAN OFF */
//        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//
//        /* 🟢 CLEAR FRAME FLAG */
//        excessheatframe = 0;
//
//        ntcFaultActive = 0;
//        ntcFaultStartTick = 0;
//    }
//}


void setting_menu_handshake()
{
	while(!settingmenu_completed_flag)
	        {
	            Watchdog_Kick();
	            Settings_Service();

	            uint8_t ready;
	        	uint16_t prev_ndtr, curr_ndtr;

	        	__disable_irq();
	        	ready = cycle_ready;
	        	if (ready)
	        	{
	        		cycle_ready = 0;
	        		prev_ndtr = zc_prev_ndtr;
	        		curr_ndtr = zc_curr_ndtr;
	        	}
	        	__enable_irq();

	        	if (ready)
	        	{
	        	   uint32_t prev_trip = ndtr_to_triplet_idx(prev_ndtr);
	        	   uint32_t curr_trip = ndtr_to_triplet_idx(curr_ndtr);


	        	   //   Check plausible full-cycle length (do bounds here, not in ISR)
	               uint32_t count = (curr_trip >= prev_trip)? (curr_trip - prev_trip) : (NUM_SAMPLES - prev_trip + curr_trip);

	               //    if (count >= 190 && count <= 320)
	        	   if (count >= 290 && count <= 480)

	        		{
	        			 vrms_in_raw = Read_AC_Voltage_RMS_DMA(prev_trip, curr_trip, 0);
	        			 vrms_in = vrms_in_raw;
	        			 vrms_out_raw = Read_AC_Voltage_RMS_DMA(prev_trip, curr_trip, 1);
	        			 vrms_out = vrms_out_raw;
	        			 irms      = Read_AC_Voltage_RMS_DMA(prev_trip, curr_trip, 2);

	        		}
	           	 }

	        	if(!trasnfomersetflag)
	        	{
					UART_SendVoltages(&huart1, vrms_in, vrms_out);
					HAL_Delay(100); // adjust refresh rate as needed
	        	}

	        	else if(trasnfomersetflag)
	        	{

	              if(!onetimeflag)
	              {
	            	  TIM1_TIM15_UpdateParameters(0, 255); // 143 is 2.5us // 128 is 2us // 64 is 1us
	            	  Transformer_Test_Start();
	              }

	              if(flipflag == 2)
	              {
	            	  UART_SendString_DMA(&huart1, "[K]\r\n"); /// ASKING PHASE
	            	  HAL_Delay(100);
	              }

	              else if(flipflag == 1)
	              {
	            	  UART_SendString_DMA(&huart1, "[W]\r\n"); /// ASKING PHASE
	            	  HAL_Delay(100);
	              }

	              onetimeflag = 1;
	        	}

	        	TIM1_TIM15_UpdateParameters(0, 128); // 143 is 2.5us // 128 is 2us // 64 is 1us

	        }

		    while (!phasecame)
		    {
		        Watchdog_Kick();
		        UART_SendString_DMA(&huart1, "[P]\r\n"); /// ASKING PHASE
		        HAL_Delay(100);
		    }
		    /* Wait for S Frame */

		    while (!sFrameReceived)
		    {
		        Watchdog_Kick();
		        UART_SendString_DMA(&huart1, "[D]\r\n"); // ASKING DATA HV,LV,OUT
		        HAL_Delay(100);
		    }

	        while (sFrameReceived && !Mreceivedflag_pulse_on)
	        {
		        Watchdog_Kick();
	            UART_SendString_DMA(&huart1, "[A]\r\n"); // ALL OK and M need to receive
	            HAL_Delay(100);
	        }

	        // Reseting the Handshake Flags


}


//static inline void ramp_setpoint_slope_ts(volatile float *sp,
//                                         float target,
//                                         float slope_V_per_s,
//                                         uint32_t *last_ms)
//{
//    uint32_t now = HAL_GetTick();
//
//    if (*last_ms == 0U) {
//        *last_ms = now;
//        return;
//    }
//
//    float dt = (now - *last_ms) * 0.001f;
//    *last_ms = now;
//
//    if (dt <= 0.0f) return;
//
//    float step = slope_V_per_s * dt;
//
//    float s = *sp;
//
//    if (target > s) {
//        s += step;
//        if (s > target) s = target;
//    } else {
//        s -= step;
//        if (s < target) s = target;
//    }
//
//    __disable_irq();
//    *sp = s;
//    __enable_irq();
//}

static inline void ramp_setpoint_slope_ts(volatile float *sp,
                                          float target,
                                          float slope_V_per_s,
                                          uint32_t *last_ms)
{
    uint32_t now;
    uint32_t elapsed_ms;
    float dt;
    float step;
    float new_sp;

    /*
     * Basic input protection.
     * slope must always be positive.
     */
    if ((sp == NULL) || (last_ms == NULL) || (slope_V_per_s <= 0.0f))
    {
        return;
    }

    now = HAL_GetTick();

    /*
     * First execution after ramp initialization.
     * Capture the current time without changing the setpoint.
     */
    if (*last_ms == 0U)
    {
        *last_ms = now;
        return;
    }

    /*
     * Unsigned subtraction safely handles HAL_GetTick() rollover.
     */
    elapsed_ms = (uint32_t)(now - *last_ms);

    if (elapsed_ms == 0U)
    {
        return;
    }

    /*
     * Save the timestamp for the next ramp calculation.
     */
    *last_ms = now;

    /*
     * Convert elapsed time from milliseconds to seconds.
     */
    dt = (float)elapsed_ms * 0.001f;

    /*
     * Prevent a large setpoint jump if the main loop was delayed
     * by blocking code, UART activity, relay handling, or any
     * other lengthy operation.
     */
    if (dt > SOFTSTART_MAX_DT_S)
    {
        dt = SOFTSTART_MAX_DT_S;
    }

    /*
     * Calculate the allowed voltage change during this execution.
     */
    step = slope_V_per_s * dt;

    /*
     * Take a local copy of the current PID setpoint.
     */
    new_sp = *sp;

    /*
     * Ramp upward for boost operation.
     */
    if (target > new_sp)
    {
        new_sp += step;

        if (new_sp > target)
        {
            new_sp = target;
        }
    }
    /*
     * Ramp downward for buck operation.
     */
    else if (target < new_sp)
    {
        new_sp -= step;

        if (new_sp < target)
        {
            new_sp = target;
        }
    }
    /*
     * Setpoint has already reached the target.
     */
    else
    {
        return;
    }

    /*
     * Protect the shared floating-point variable because it is
     * also accessed from the TIM4 PID interrupt.
     */
    __disable_irq();
    *sp = new_sp;
    __enable_irq();
}

char uart_tx_buffer[64];
int main(void)
{
    HAL_Init();
    Diag_CaptureResetCause();   /* why did we (re)start? see g_diag */
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART3_UART_Init();
    MX_ADC1_Init();
    MX_DMA_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
  //  MX_TIM7_Init();
    MX_TIM17_Init();
    MX_TIM14_Init();
    MX_TIM15_Init();
    MX_USART1_UART_Init();
    Diag_BootReport();

    Load_Settings_From_Flash();

    /* ---- Initial PID init (ONLY ONCE here) ----
       outputMax = 0.9 * PWM_PERIOD
    */
#if 0
    AutoPID_Init(&pid,
                 &vrms_out,
                 &voltage_setpoint,
                 &pwm_output,
                 0.18f, 4.0f, 0.00f,
                 0.0f,
                 0.9f * (float)PWM_PERIOD,
                 5);
#endif

//    HAL_TIM_Base_Start_IT(&htim4);
//    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start(&htim3);      // TIM3 drives ADC
    HAL_TIM_Base_Start(&htim2);      // free-running 1us tick for zero-cross timing
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);   // used for the relay predictive-close compare match (CC2)
    HAL_TIM_Base_Start_IT(&htim17);

    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, TOTAL_WORDS);

    /* Optional initial cycle compute (your existing code) */
    {
        uint8_t ready;
        uint16_t prev_ndtr, curr_ndtr;

        uint32_t boot_wait_start = HAL_GetTick();
        /* Bounded: with no mains the old loop hung here forever. With no reading,
           vrms_in stays low and the rectifier is never allowed. */
        while (!cycle_ready &&
               (uint32_t)(HAL_GetTick() - boot_wait_start) < BOOT_ZC_WAIT_MS) { }

        __disable_irq();
        ready = cycle_ready;
        if (ready)
        {
            cycle_ready = 0;
            prev_ndtr = zc_prev_ndtr;
            curr_ndtr = zc_curr_ndtr;
        }
        __enable_irq();

        if (ready)
        {
            uint32_t prev_trip = ndtr_to_triplet_idx(prev_ndtr);
            uint32_t curr_trip = ndtr_to_triplet_idx(curr_ndtr);

            uint32_t count = (curr_trip >= prev_trip)
                           ? (curr_trip - prev_trip)
                           : (NUM_SAMPLES - prev_trip + curr_trip);

            if (count >= 290 && count <= 480)
            {
                vrms_in_raw = Read_AC_Voltage_RMS_DMA(prev_trip, curr_trip, 0);
                vrms_in = vrms_in_raw;
                vrms_out_raw = Read_AC_Voltage_RMS_DMA(prev_trip, curr_trip, 1);
                vrms_out = vrms_out_raw;
                irms     = Read_AC_Voltage_RMS_DMA(prev_trip, curr_trip, 2);
            }
        }
    }
    // Initialize and start PID after you measure the voltage first time
    // After you compute an initial vrms_in/vrms_out (ready==1 path)
    voltage_setpoint_pid = vrms_in;   // start with error ≈ 0, freeze holds output at 0
    AutoPID_Init(&pid,
                 &vrms_out,
                 &voltage_setpoint_pid,   // <— point PID to the ramped setpoint
                 &pwm_output,
                 0.18f, 4.0f, 0.00f,
                 0.0f,
                 (float)PWM_PERIOD,
                 5);

    HAL_TIM_Base_Start_IT(&htim4);

    TIM1_TIM15_UpdateParameters(0, 128);

    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
    __HAL_TIM_ENABLE_IT(&htim15, TIM_IT_BREAK);

    /* Start PWM outputs */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);

    /* Start UART DMA RX */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma, RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

//    while(!Relay_operation_starting_time)
//    {
//        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // relay 1 close
//        HAL_Delay(500);
//        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); // relay 1 open
//        HAL_Delay(100);
//        Relay_operation_starting_time = 1;
//    }
    while(!Relay_operation_starting_time)
    {
        /* Close SH-RLY predictively (lead-compensated) if we already have
           zero-cross history, otherwise fall back reactively. Either way,
           timeout after 50ms in case no AC/edges show up (e.g. bench
           testing with no mains connected), so this can't hang forever. */
        Relay1_ArmClose();
        {
            uint32_t arm_start = HAL_GetTick();
            while ((relay1_close_armed || relay1_predictive_pending) &&
                   (HAL_GetTick() - arm_start) < 50U) { }
            if (relay1_close_armed || relay1_predictive_pending)
            {
                relay1_close_armed = 0;
                relay1_predictive_pending = 0;
                __HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC2);
                RELAY1_SET();
            }
        }
    	HAL_Delay(500);
        RELAY1_RESET();
        HAL_Delay(100);
        Relay_operation_starting_time = 1;
    }

    /* Your handshake */
    setting_menu_handshake();

    Watchdog_Init();   /* after the (unbounded) master handshake */

    while (1)
    {
        Watchdog_Kick();
        Diagnostic_Buzzer_Service();   // TEMP: audible fault-source diagnostic, remove after root-cause found
        Rectifier_Drop_Diag_Service();   // TEMP: 4-way edge-triggered chirp, remove after root-cause found


        while (runtime_setting_menu)
        {
            Watchdog_Kick();
            while (a_relay_close_pending)
              {
                  Watchdog_Kick();
                  if ((HAL_GetTick() - a_relay_close_time) >= 1000U)
                  {
//                      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // relay 2 close
                	  RELAY2_RESET();
                      a_relay_close_pending = 0;
                      a_relay_close_time = 0;
                      runtime_setting_menu = 0;
                  }
              }

            settingmenu_completed_flag = 0;
            trasnfomersetflag = 0;
            onetimeflag = 0;
            phasecame = 0;
            sFrameReceived = 0;
            Mreceivedflag_pulse_on = 0;
            setting_menu_handshake();
        }
        while(!runtime_setting_menu)
        {
            Watchdog_Kick();   /* the running loop never leaves this while(): kick HERE */
    		Fault_ReArm_Service();
    	    PwmStatusHold_Service();
            Protection_Service();
            Input_Window_Service();
            Settings_Service();
            Diag_Service();
            Mreceivedflag_pulse_on = 1;

            uint8_t ready;
            uint16_t prev_ndtr, curr_ndtr;

            __disable_irq();
            ready = cycle_ready;

            if (ready)
            {
                cycle_ready = 0;
                prev_ndtr = zc_prev_ndtr;
                curr_ndtr = zc_curr_ndtr;
            }
            __enable_irq();

            if (ready)
            {
                uint32_t prev_trip = ndtr_to_triplet_idx(prev_ndtr);
                uint32_t curr_trip = ndtr_to_triplet_idx(curr_ndtr);

                uint32_t count = (curr_trip >= prev_trip)
                               ? (curr_trip - prev_trip)
                               : (NUM_SAMPLES - prev_trip + curr_trip);

                if (count >= 290 && count <= 480)
                {
                    vrms_in_raw = Read_AC_Voltage_RMS_DMA(prev_trip, curr_trip, 0);
                    vrms_in = vrms_in_raw + (float)working_settings.input_v_r;
                    vrms_out_raw = Read_AC_Voltage_RMS_DMA(prev_trip, curr_trip, 1);
                    vrms_out = vrms_out_raw + (float)working_settings.output_v_r;
                    irms     = Read_AC_Voltage_RMS_DMA(prev_trip, curr_trip, 2);
                    Measurement_Mark();
                }
            }

//            static uint32_t ramp_last_ms = 0;
//            static uint8_t was_armed = 0;
//
//            uint8_t armed = (gbIsSystemNormalState == 1 &&
//                             Mreceivedflag_pulse_on == 1 &&
//                             pwm_allowed == 1);
//
//            if (armed && !was_armed) {
//                __disable_irq();
//                voltage_setpoint_pid = vrms_in;
//                __enable_irq();
//
//                pid.integral = 0.0f;
//                pid.derivative = 0.0f;
//                pid.lastError  = voltage_setpoint_pid - vrms_out;
//
//                ramp_last_ms = HAL_GetTick();   // <-- IMPORTANT reset timing here
//                was_armed = 1;
//            }
//
//            if (armed) {
//                ramp_setpoint_slope_ts(&voltage_setpoint_pid, voltage_setpoint,10.0f, &ramp_last_ms);
//            } else {
//                was_armed = 0;
//                ramp_last_ms = 0;              // <-- IMPORTANT: avoid dt explosion when re-armed
//                __disable_irq();
//                voltage_setpoint_pid = vrms_in;
//                __enable_irq();
//            }

            static uint32_t ramp_last_ms = 0U;
            static uint32_t softstart_begin_ms = 0U;
            static uint8_t was_armed = 0U;

            uint8_t armed = (gbIsSystemNormalState == 1U &&
                             Mreceivedflag_pulse_on == 1U &&
                             pwm_allowed == 1U);

            if (armed && !was_armed)
            {
                uint32_t now = HAL_GetTick();
                /*
                 * Initialize the ramp*d PID setpoint to the present inpu*
                 * voltage so the initial cor*ection demand is close to zero.
              *  */
                __disable_irq();
                voltage_setpoint_pid = vrms_in;
                __enable_irq();

                /*
                 * Reset P*D states for bumpless startup.
               * */
                pid.integral = 0.0f;
                pid.derivative = 0.0f;
                pid.lastOutput = 0.0f;
                pwm_output = 0.0f;

                /*
                 * Initialize the stored PID error according to the same
                 * buck/boost convention used in AutoPID_Compute().
                 */
                if (vrms_in < voltage_setpoint)
                {
                    pid.lastError = voltage_setpoint_pid - vrms_out;
                }
                else
                {
                    pid.lastError = vrms_out - voltage_setpoint_pid;
                }

                /*
                 * Initialize the ramp timestamps.
                 */
                ramp_last_ms = now;
                softstart_begin_ms = now;

                was_armed = 1U;
            }

            if (armed)
            {
                uint32_t softstart_elapsed_ms;
                float voltage_difference;
                float selected_slope;

                softstart_elapsed_ms =
                    (uint32_t)(HAL_GetTick() - softstart_begin_ms);

                voltage_difference =
                    voltage_setpoint_pid - voltage_setpoint;

                /*
                 * Obtain absolute voltage difference without using fabsf().
                 */
                if (voltage_difference < 0.0f)
                {
                    voltage_difference = -voltage_difference;
                }

                /*
                 * Stage 1:
                 * Use a slow 5 V/s slope for the first one second.
                 */
                if (softstart_elapsed_ms < SOFTSTART_SLOW_TIME_MS)
                {
                    selected_slope = SOFTSTART_SLOW_SLOPE_VPS;
                }
                /*
                 * Final stage:
                 * Slow down when the setpoint is within 3 V of the target.
                 */
                else if (voltage_difference <= SOFTSTART_FINAL_BAND_V)
                {
                    selected_slope = SOFTSTART_FINAL_SLOPE_VPS;
                }
                /*
                 * Stage 2:
                 * Use the faster slope after the first one second.
                 */
                else
                {
                    selected_slope = SOFTSTART_FAST_SLOPE_VPS;
                }

                ramp_setpoint_slope_ts(&voltage_setpoint_pid,
                                       voltage_setpoint,
                                       selected_slope,
                                       &ramp_last_ms);
            }
            else
            {
                /*
                 * Reset the soft-start state whenever PWM is not armed.
                 */
                was_armed = 0U;
                ramp_last_ms = 0U;
                softstart_begin_ms = 0U;

                __disable_irq();
                voltage_setpoint_pid = vrms_in;
                __enable_irq();
            }

            /* Close the output contactor only after Vout is stable. */
            Output_Contactor_Service();

            SYS_CheckForFaults();

            if (c_sequence_active)
            {
            	uint32_t now = HAL_GetTick();
                if (c_sequence_step == 1)
                {
                    // Wait 1 second after PIN 9 SET
                    if ((now - c_sequence_time) >= 500U)
                    {
                        // PIN 9 OFF open relay 1
 //                       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
                        RELAY1_RESET();

                        // Now PIN 8 ON open relay 2
 //                       HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
                        RELAY2_SET();
                        c_sequence_time = now;
                        c_sequence_step = 2;
                    }
                }
                else if (c_sequence_step == 2)
                {
                    if ((now - c_sequence_time) >= 1000U)
                    {
     					Mreceivedflag_pulse_on = 1;
                        gbIsSystemNormalState = 1;
                        relay2_on = 1;

                        // Sequence completed
                        c_sequence_active = 0;
                        c_sequence_step = 0;
                        c_sequence_time =0;

                    }
                }
            }
            if (c_relay_close_pending)
            {
                if ((HAL_GetTick() - c_relay_close_time) >= 1000U)
                {
                   // HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); // relay 2 close
                	RELAY2_RESET();
                    c_relay_close_pending = 0;
                    c_relay_close_time = 0;
                }
            }

        }
    }

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_7CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART3_UART_Init(void)
{
  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CDE END USART3_Init 0 */
  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
	    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
	    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
	    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
	    Error_Handler();
  }

  /* USER CODE BEGIN USART3_Init 2 */



  /* USER CODE END USART3_Init 2 */



}
static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 6399;      // Divides 64 MHz by 1024 -> 62.5 kHz // 1023       // 6399
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = Tim4Prd;          // 1000 counts -> 16 ms  // 999                      // 99
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
//    htim3.Init.Period = 4799;  // → 13.33kHz
    htim3.Init.Period = 3199;  // → 20kHz

    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim3);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

    HAL_TIM_Base_Start(&htim3); // Start TIM3
}

/********************************************************************************/

uint8_t started = 0;             // Tracks if starting sequence completed
int R2=80;



// Threshold settings


/* Dead-time between forcing both bridge diagonals off at the
 * zero-crossing and re-engaging the new one (TIM14 runs at exactly
 * 1MHz, so this value is in microseconds). Was 1us - almost certainly
 * too short for these IGBTs once gate-driver propagation delay and
 * IGBT turn-off "tail current" are accounted for, especially at higher
 * bus voltage where any shoot-through overlap develops current faster.
 * Increased to 5us first - confirmed to move the failure threshold from
 * ~240V to ~250V on the bench, so this lever is verified to work in the
 * right direction. Increased to 10us, then 20us, to cover margin above
 * ~255V. Increased further to 40us here for testing at 290V - NOTE this
 * is beyond the 280V ceiling dsPIC was actually proven at on this same
 * hardware, so a failure at 290V may not be a dead-time question at all.
 * VERIFY ON A SCOPE (actual gate/collector-emitter turn-off timing) when
 * possible - still inferred from pass/fail testing, not directly
 * observed. */
int DT = 40;

/*********************************************************************************/

static void Output_Contactor_Service(void)
{
    static uint32_t stable_start_ms = 0U;
    static uint8_t stable_timer_active = 0U;

    /* Never keep the load connected during fault, inrush or PWM-off state.
     * (Reverted to instant open, no discharge-wait delay - the delay logic
     * that used to be here was added chasing a contactor jerk/chatter
     * issue that turned out to be specific to a toroidal output
     * transformer's inrush/magnetizing-current behavior, not a firmware
     * timing problem - EI transformer confirmed fine without any of it.) */
    if (gFaultLatched || Inrush_present ||
        !gbIsSystemNormalState || !pwm_allowed)
    {
        CONT1_ON = 0;
        CONT1_SET();
        stable_timer_active = 0U;
        stable_start_ms = 0U;
        load_softstart_active = 0U;
        load_softstart_begin_ms = 0U;
        return;
    }

    /* Once CONT1 is ON, leave it ON until the normal shutdown path opens it. */
    if (CONT1_ON)
        return;

    /* Only consider closing once:
     *  (a) the soft-start ramp has actually reached the real setpoint -
     *      i.e. the PID is now running on the true target, not still
     *      ramping toward it, and
     *  (b) Vout has stayed within +/-2 V of the final set voltage
     * for CONTACTOR_STABLE_TIME_MS continuously. */
    {
        uint8_t pid_on_target = (fabsf(voltage_setpoint_pid - voltage_setpoint) <= CONTACTOR_PID_REACHED_TOL_V) ? 1U : 0U;
        uint8_t vout_in_band  = (fabsf(vrms_out - voltage_setpoint) <= CONTACTOR_VOLTAGE_TOLERANCE_V) ? 1U : 0U;

        if (pid_on_target && vout_in_band)
        {
            if (!stable_timer_active)
            {
                stable_timer_active = 1U;
                stable_start_ms = HAL_GetTick();
            }
            else if ((HAL_GetTick() - stable_start_ms) >= CONTACTOR_STABLE_TIME_MS)
            {
                CONT1_ON = 1;
                CONT1_RESET();       /* Contactor ON */

                /* Start non-blocking load soft-start. */
                load_softstart_active = 1U;
                load_softstart_begin_ms = HAL_GetTick();

                stable_timer_active = 0U;
                stable_start_ms = 0U;
            }
        }
        else
        {
            stable_timer_active = 0U;
            stable_start_ms = 0U;
        }
    }
}

/* ============================================================================
 * (retired: the post-contactor "voltage-ramp via buzzer" diagnostic that
 * used to live here has been removed - it shared 1-4 beep codes with the
 * real fault-source buzzer above and was causing ambiguous readings once
 * the buzzer was correctly wired to PA6. Rely on the fault-source buzzer
 * (Diagnostic_Buzzer_Service: 1 beep = BKIN HW trip, 2 = phase-seq/ZC,
 * 3 = other) instead.)
 * ============================================================================ */

void relay_handler(float vrms_in)
{
    // --- One-time flags for controlling pulses ---
    static uint8_t pulse45_done = 0;           // Tracks if 45V SH-RLY pulse was already given
    static uint8_t pulse180_done = 0;          // Tracks if 100V W-RLY pulse was already given
    static uint8_t pulse180_recover_done = 0;  // Tracks if W-RLY recovery pulse was given
    static uint8_t pulse30_done = 0;           // Tracks if safety 30V shutdown pulse was given

    // --- Non-blocking timing variables ---
    static uint32_t delay_start = 0;           // Timestamp for delays
    static uint8_t waiting_for_2s = 0;         // Flag for pending 2s delay
    static uint8_t waiting_for_5s = 0;         // Flag for pending 5s delay

    // --- HV cutoff variables ---
    static uint8_t hv_active = 0;              // HV trip active
    // (if you want a timed recovery add hv_delay_start and hv_recover_wait similarly)

    // --- Fault & protection logic ---
//    if (on_off == 1 || con_state == 0 || flt_in_other == 0) //|| ntc == 0 )
    if (on_off == 1  || flt_in_other == 0) //|| ntc == 0 )
    {
        CONT1_ON = 0;
        CONT1_SET();  // CONT1 OFF in normal case
        return; // Skip rest of logic if fault condition is active
    }

    // ---------- HIGH VOLTAGE CUTOFF HANDLING (global) ----------
    // If HV occurs at any time (starting or running), immediately set contractor and disable relay pulses.

    // --- HV cutoff handling ---
    static uint8_t hv_trip_wait = 0;       // waiting for HV trip delay
    static uint32_t hv_trip_start = 0;

    if (!hv_active && vrms_in >= HV_CUTOFF )
    {
        if (!hv_trip_wait)   // first time crossing HV cutoff
        {
            hv_trip_start = HAL_GetTick();
            hv_trip_wait = 1;
            HV_trip = 1;    // When the flag is 1 lcd.c will print HV appeared

        }

        // After 1s above HV_CUTOFF → trip
        if (hv_trip_wait && (HAL_GetTick() - hv_trip_start >= 5000))
        {
        	CONT1_ON = 0;
            CONT1_SET();      // Contractor OFF
            HAL_Delay(50);    // <-- still blocking, can be converted later

            relay2_on = 0;    // stop W-RLY pulses
            HAL_Delay(R2);
//            RELAY2_RESET(); // W-RLY OFF**************

            hv_active = 1;    // mark HV active
            HV_active = 1;    // When the flag is 1 lcd.c will print HV_CUTOFF
            HV_trip = 0;
            hv_trip_wait = 0; // clear wait flag

            return;
        }
    }
    else
    {
        // If voltage drops below cutoff quickly, cancel pending trip
        hv_trip_wait = 0;
        HV_trip = 0;     // HV_trip and HV_active is used for printing lcd.c
    }


    // If HV was active, wait until voltage falls by hysteresis below HV_CUTOFF before recovery
    if (hv_active)
    {
        static uint8_t hv_recover_wait = 0;
        static uint32_t hv_recover_start = 0;

        // Voltage dropped below cutoff - hysteresis
        if (!hv_recover_wait && vrms_in < (HV_CUTOFF))
        {
            // Start the 5s recovery timer
            hv_recover_start = HAL_GetTick();
            hv_recover_wait = 1;
            HV_recover = 1;
        }

        // If recovery timer is running, check timeout
        if (hv_recover_wait && (HAL_GetTick() - hv_recover_start >= 5000) )
        {
        	HAL_Delay(R2);

            // HV has cleared long enough — perform recovery
            relay2_on = 1;
            hv_active = 0;    // clear HV flag
            HV_active=0;

            hv_recover_wait = 0;
            HV_recover = 0;  // HV_recover is used for printing normal_op in lcd.c
            pulse180_recover_done = 1;
        }

        return; // Exit while in HV handling
    }


    // ---------- STARTING PROCEDURE ----------
    if (!started)
    {
        // Step 1: If >=45V and SH-RLY not yet pulsed → give 50ms SH-RLY pulse
        if (vrms_in >= 45 && relay2_on == 0 && !pulse45_done)
        {
        	CONT1_ON = 0;
            CONT1_SET();  // keep CONT1 OFF
            pulse45_done = 1;
        }

        // Step 2: If >= StartRelay_Volt + HYST → schedule W-RLY ON after 5s
        if (vrms_in >= (StartRelay_Volt) && !pulse180_done && !waiting_for_2s)
        {
            waiting_for_2s = 1;
            delay_start = HAL_GetTick();
        }

        // After 5s → give SH pulse + enable W-RLY + enable CONT1
        if (waiting_for_2s && (HAL_GetTick() - delay_start >= 5000))
        {
            HAL_Delay(R2);
            relay2_on = 1;
            started = 1;           // Mark system as started
            pulse180_done = 1;
            waiting_for_2s = 0;
        }

        return; // exit until start complete
    }

    // ---------- POST-START RUNNING LOGIC ----------
    // Condition: If < StartRelay_Volt - HYST → shutdown W-RLY + CONT1
    if (vrms_in < (StartRelay_Volt) && relay2_on)
    {
        if (!lowVoltFlag)
        {
            // Low voltage detected first time, start timer
            lowVoltStart = HAL_GetTick();
            lowVoltFlag = 1;
            LOW_Voltflag = 1;
        }
        else
        {
            // Check if 5 seconds elapsed
            if ((HAL_GetTick() - lowVoltStart) >= 5000 )
            {
                // Voltage has been low continuously for 5 sec → perform action
            	CONT1_ON = 0;
                CONT1_SET();  // CONT1 OFF
                HAL_Delay(50);
                relay2_on = 0;
                HAL_Delay(R2);

                // Reset flag/timer for next detection
                lowVoltFlag = 0;
                LOW_Voltflag = 0;
                pulse180_recover_done = 0; // allow recovery later
            }
        }
    }
    else
    {
        // Voltage recovered before 5 sec → reset timer and flag
        lowVoltFlag = 0;
        LOW_Voltflag = 0;
    }

    // Condition: If >= StartRelay_Volt + HYST and W-RLY is OFF → recover after 5s
    if (vrms_in >= (StartRelay_Volt) && !relay2_on && !pulse180_recover_done && !waiting_for_5s)
    {
        waiting_for_5s = 1;
        delay_start = HAL_GetTick();
    }

    if (waiting_for_5s && (HAL_GetTick() - delay_start >= 5000))
    {
        HAL_Delay(R2);

        relay2_on = 1;
        pulse180_recover_done = 1;
        waiting_for_5s = 0;
    }

    // Condition: If <30V → safety shutdown pulse
    if (vrms_in < 30 && !pulse30_done)
    {
    	CONT1_ON = 0;
        CONT1_SET();  // CONT1 OFF
        pulse30_done = 1;
    }

    // ---------- RESET ONE-TIME FLAGS (allow future triggers) ----------
    if (vrms_in < 45) pulse45_done = 0;
    if (vrms_in < (StartRelay_Volt)) pulse180_done = 0;
    if (vrms_in >= 30) pulse30_done = 0;
}

void Stop_All_PWM(void)
{
    inverter_startup_active = 0U;
    inverter_startup_begin_ms = 0U;

    TIM1->CCR1  = 0;
    TIM15->CCR1 = 0;

    TIM1->CCER  &= ~TIM_CCER_CC1NE;
    TIM15->CCER &= ~TIM_CCER_CC1NE;

    GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;  // reset
    GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;  // reset
}

void Enable_All_PWM(void)
{
    TIM1->CCER  |= TIM_CCER_CC1NE;
    TIM15->CCER |= TIM_CCER_CC1NE;
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);
    TIM15->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE);

}
//
//static inline void update_rectifier_allowed(float vrms_in)
//{
//	if(con_state == 0)
//		{
//			return;
//		}
//    if ((!rectifier_allowed && vrms_in >= (((float)StartRelay_Volt)) &&  relay2_on == 1)
//    		&&  gbIsSystemNormalState == 1 && vrms_in <= HV_CUTOFF && flt == 1 && Mreceivedflag_pulse_on == 1)
//    {
//        rectifier_allowed = 1;
//    }
//
//    else if ((rectifier_allowed && vrms_in < ((float)StartRelay_Volt) && relay2_on == 0) || gbIsSystemNormalState == 0 || flt == 0 || Mreceivedflag_pulse_on == 0)
//
//    {
//        rectifier_allowed = 0;
//    }
//}
//static inline void update_pwm_allowed(float vrms_in)
//{
//    if (con_state == 0)
//    {
//        pwm_allowed = 0;
//        rectifier_to_inverter_delay_active = 0;
//
//        Stop_All_PWM();
//        return;
//    }
//
//    // ---------------------------------------------------------
//    // RECTIFIER -> INVERTER 2 SECOND DELAY
//    //
//    // rectifier_on_time is started when the rectifier GPIO
//    // is actually enabled inside TIM14_IRQHandler().
//    // ---------------------------------------------------------
//    if ((!pwm_allowed && (vrms_in >= (float)StartRelay_Volt && vrms_in <= HV_CUTOFF) && relay2_on == 1
//    		&& gbIsSystemNormalState == 1 && flt == 1 && Mreceivedflag_pulse_on == 1))
//    {
//		if (rectifier_allowed && rectifier_to_inverter_delay_active)
//		{
//			if ((HAL_GetTick() - rectifier_on_time) >=
//				RECTIFIER_TO_INVERTER_DELAY_MS)
//			{
//				// 2 seconds completed
//				pwm_allowed = 1;
//
//				// Enable inverter PWM outputs
//				Enable_All_PWM();
//			}
//			else
//			{
//				// Still waiting for 2 seconds.
//				// Keep INVERTER OFF.
//				pwm_allowed = 0;
//
//				TIM1->CCR1  = 0;
//				TIM15->CCR1 = 0;
//
//				TIM1->CCER  &= ~TIM_CCER_CC1NE;
//				TIM15->CCER &= ~TIM_CCER_CC1NE;
//			}
//		}
//    }
//
//    // ---------------------------------------------------------
//    // RECTIFIER NOT ALLOWED
//    //
//    // Immediately stop inverter and reset the 2 sec sequence.
//    // ---------------------------------------------------------
//    if (!rectifier_allowed)
//    {
//        rectifier_to_inverter_delay_active = 0;
//        rectifier_on_time = 0;
//
//        pwm_allowed = 0;
//
//        Stop_All_PWM();
//    }
//
//    // ---------------------------------------------------------
//    // SAFETY / FAULT CONDITIONS
//    // ---------------------------------------------------------
//    if ((pwm_allowed && relay2_on == 0) ||
//        gbIsSystemNormalState == 0 ||
//        flt == 0 ||
//        Mreceivedflag_pulse_on == 0)
//    {
//        pwm_allowed = 0;
//
//        rectifier_to_inverter_delay_active = 0;
//        rectifier_on_time = 0;
//
//        Stop_All_PWM();
//    }
//}


/* NOTE: A converter (AC->DC bridge) soft-start via half-cycle skipping
 * was tried here and reverted after it was reported to produce a
 * stuck-diagonal condition (one leg pair latched on, the other off) -
 * a real hardware-safety issue, not just an ineffective fix. Root cause
 * was not confirmed before reverting. Do not re-add a converter soft-
 * start without first understanding why that happened. The bridge is
 * back to its original, unramped engagement, matching the known-working
 * dsPIC behavior on the same hardware. */

/* Settling delay: once SH-RLY/W-RLY have finished and the system is
 * otherwise ready, wait this long before letting the converter bridge
 * engage for the first time, so any relay-closure transient/ringing on
 * the DC bus has time to die down first. This only delays WHEN the
 * existing engagement is allowed to start - it does not touch the
 * bridge's own switching pattern at all.
 *
 * Scaled with measured input voltage: higher line voltage means more
 * stored energy in the bus caps and (plausibly) more relay-closure
 * ringing/overshoot to settle out, so it gets a longer wait. Tune all
 * four values on the bench. */
#define CONVERTER_ON_DELAY_BASE_MS    500U    // delay at/below V_LOW
#define CONVERTER_ON_DELAY_MAX_MS    2000U    // delay at/above V_HIGH
#define CONVERTER_ON_DELAY_V_LOW     220.0f   // volts - below this, use the base delay
#define CONVERTER_ON_DELAY_V_HIGH    260.0f   // volts - at/above this, use the max delay

volatile uint32_t converter_on_delay_start = 0;
volatile uint8_t  converter_on_delay_armed = 0;
volatile uint32_t converter_on_delay_target_ms = CONVERTER_ON_DELAY_BASE_MS;

static inline uint32_t Converter_On_Delay_For_Voltage(float v)
{
    float frac;

    if (v <= CONVERTER_ON_DELAY_V_LOW)  return CONVERTER_ON_DELAY_BASE_MS;
    if (v >= CONVERTER_ON_DELAY_V_HIGH) return CONVERTER_ON_DELAY_MAX_MS;

    frac = (v - CONVERTER_ON_DELAY_V_LOW) / (CONVERTER_ON_DELAY_V_HIGH - CONVERTER_ON_DELAY_V_LOW);
    return (uint32_t)((float)CONVERTER_ON_DELAY_BASE_MS
             + frac * (float)(CONVERTER_ON_DELAY_MAX_MS - CONVERTER_ON_DELAY_BASE_MS));
}

static inline void update_rectifier_allowed(float vrms_in)
{
    static uint8_t v_bad_count = 0U;
    uint8_t v_ok = 1U;
#if LOCAL_VOLTAGE_LIMITS_ENABLED
    /* Input window = master LV/HV clamped to the 140/300 backstop, FRESH data only. */
    v_ok = (Measurement_IsFresh() &&
            Input_Voltage_Permit(rectifier_allowed, vrms_in, Input_LV_Limit(), Input_HV_Limit())) ? 1U : 0U;
#else
    (void)vrms_in;
#endif

    /* Debounce ONLY the voltage-based trip-off: require RECTIFIER_TRIP_SAMPLES
     * consecutive bad zero-cross samples before treating it as a real drop-
     * out. Without this, a single borderline/noisy reading right at the
     * LV/HV boundary - e.g. slowly sweeping a variac through the threshold -
     * instantly collapses rectifier_allowed -> pwm_allowed -> CONT1 opening,
     * causing rapid contactor clicking right at that voltage. Every other
     * reason to turn off (explicit command loss, inrush, or the system
     * already being told to disconnect) still acts immediately below -
     * this only smooths the raw voltage reading. */
    if (!v_ok)
    {
        if (v_bad_count < RECTIFIER_TRIP_SAMPLES) v_bad_count++;
    }
    else
    {
        v_bad_count = 0U;
    }
    uint8_t v_trip = (v_bad_count >= RECTIFIER_TRIP_SAMPLES) ? 1U : 0U;

    uint8_t want_on  = (gbIsSystemNormalState == 1 && Mreceivedflag_pulse_on == 1 && Inrush_present == 0 && v_ok);
    uint8_t want_off = ((rectifier_allowed && gbIsSystemNormalState == 0 ) || Mreceivedflag_pulse_on == 0 || Inrush_present ==  1 || v_trip);

    if (!rectifier_allowed && want_on)
    {
        if (!converter_on_delay_armed)
        {
            converter_on_delay_armed     = 1U;
            converter_on_delay_start     = HAL_GetTick();
            converter_on_delay_target_ms = Converter_On_Delay_For_Voltage(vrms_in);
        }
        else if ((uint32_t)(HAL_GetTick() - converter_on_delay_start) >= converter_on_delay_target_ms)
        {
            rectifier_allowed = 1;
            /* Re-arm the zero-cross monitor's grace period here too - the
               converter's own switch-on causes a brief, normal line
               disturbance that the monitor would otherwise be fully
               strict for, since the power-on grace period has long since
               expired by this point (up to ~2s later with the voltage-
               scaled settling delay). Counts fresh from the very next
               edge, same mechanism as the power-on/rearm grace period. */
            zc_startup_counter = 0U;
        }
    }

    else if (want_off)

    {
        /* TEMPORARY bench-diagnostic: EDGE-triggered - only when
         * rectifier_allowed is actually 1 right now (about to transition to
         * 0), never while it's already sitting off. Identifies which of
         * the four reasons caused THIS drop, checked in priority order so
         * exactly one code applies per event. Must NOT call the (blocking)
         * buzzer directly here - this runs in the zero-cross ISR. A
         * main-loop service consumes the code instead. */
        if (rectifier_allowed && rectifier_drop_diag_code == 0U)
        {
            if (gbIsSystemNormalState == 0)        rectifier_drop_diag_code = 4U;
            else if (Mreceivedflag_pulse_on == 0)  rectifier_drop_diag_code = 2U;
            else if (Inrush_present == 1)          rectifier_drop_diag_code = 3U;
            else if (v_trip)                       rectifier_drop_diag_code = 1U;
        }

        rectifier_allowed = 0;
        converter_on_delay_armed = 0U;
        v_bad_count = 0U;
    }
    else if (!want_on)
    {
        /* Not currently eligible and not asking to turn off either -
           keep the timer cleared so the next eligibility window gets
           its own fresh delay rather than an old timestamp. */
        converter_on_delay_armed = 0U;
    }

}

static inline uint16_t Inverter_Startup_Duty_Limit(void)
{
    uint32_t elapsed;

    if (!inverter_startup_active)
        return (uint16_t)PWM_PERIOD;

    elapsed = (uint32_t)(HAL_GetTick() - inverter_startup_begin_ms);

    /* First 100 ms: hard zero duty. This is kept purely as a startup-glitch
     * guard (a clean, known-zero state right at PWM enable) - not as a
     * ramp stage. Once past it, duty is no longer capped here at all;
     * the voltage_setpoint_pid slope ramp (see the softstart block that
     * calls ramp_setpoint_slope_ts) is now the SOLE authority over how
     * fast duty/current actually rises, since it's a real closed-loop
     * ramp instead of open-loop fixed percentage steps that were
     * clipping the PID's own smooth output and reintroducing step
     * transients at each stage boundary. */
    if (elapsed < INVERTER_STARTUP_DUTY_0_MS)
        return 0U;

    inverter_startup_active = 0U;
    return (uint16_t)PWM_PERIOD;
}

static inline uint16_t Inverter_Get_Limited_Duty(void)
{
    float duty = pwm_output;
    uint16_t limit = Inverter_Startup_Duty_Limit();

    /* Load soft-start limiter: minimum of the existing transformer startup
     * limit and this load-connection limit is always applied.
     * Ceiling ramps CONTINUOUSLY from LOAD_SOFTSTART_DUTY_0_PCT to 100 %
     * over LOAD_SOFTSTART_TIME_MS, instead of jumping in four 500 ms
     * blocks. The old block schedule made the ceiling itself cross above
     * or below whatever the PID was asking for in big jumps, which showed
     * up as a one-time output dip/hike right after CONT1 closed - a
     * smooth ramp removes that step without changing how long inrush is
     * limited for or how conservative the starting point is. */
    if (load_softstart_active)
    {
        uint32_t elapsed = (uint32_t)(HAL_GetTick() - load_softstart_begin_ms);
        float load_limit_pct;
        uint16_t load_limit;

        if (elapsed >= LOAD_SOFTSTART_TIME_MS)
        {
            load_softstart_active   = 0U;
            load_softstart_begin_ms = 0U;
            load_limit_pct = 1.0f;
        }
        else
        {
            float frac = (float)elapsed / (float)LOAD_SOFTSTART_TIME_MS;
            load_limit_pct = LOAD_SOFTSTART_DUTY_0_PCT +
                              frac * (1.0f - LOAD_SOFTSTART_DUTY_0_PCT);
        }

        load_limit = (uint16_t)(load_limit_pct * (float)PWM_PERIOD);
        if (load_limit < limit)
            limit = load_limit;
    }

    if (duty < 0.0f)
        duty = 0.0f;
    if (duty > (float)PWM_PERIOD)
        duty = (float)PWM_PERIOD;
    if (duty > (float)limit)
        duty = (float)limit;

    return (uint16_t)duty;
}

static inline void update_pwm_allowed(float vrms_in)
{
    // ---------------------------------------------------------
    // RECTIFIER -> INVERTER 2 SECOND DELAY
    //
    // rectifier_on_time is started when the rectifier GPIO
    // is actually enabled inside TIM14_IRQHandler().
    // ---------------------------------------------------------
    if (!pwm_allowed && gbIsSystemNormalState == 1  && Mreceivedflag_pulse_on == 1 && Inrush_present == 0)
    {
		if (rectifier_allowed && rectifier_to_inverter_delay_active)
		{
			if ((HAL_GetTick() - rectifier_on_time) >= RECTIFIER_TO_INVERTER_DELAY_MS)
			{
                /* Fresh, deterministic inverter start. */
                pid.integral = 0.0f;
                pid.derivative = 0.0f;
                pid.lastOutput = 0.0f;
                pid.lastError = 0.0f;
                pwm_output = 0.0f;

                __HAL_TIM_SET_COUNTER(&htim1, 0U);
                __HAL_TIM_SET_COUNTER(&htim15, 0U);
                TIM1->CCR1 = 0U;
                TIM15->CCR1 = 0U;

                inverter_startup_begin_ms = HAL_GetTick();
                inverter_startup_active = 1U;
				pwm_allowed = 1;
				gFaultdisplay  = 0;
				Enable_All_PWM();
			}
			else
			{
				// Still waiting for 2 seconds.
				// Keep INVERTER OFF.
				pwm_allowed = 0;

				TIM1->CCR1  = 0;
				TIM15->CCR1 = 0;

				TIM1->CCER  &= ~TIM_CCER_CC1NE;
				TIM15->CCER &= ~TIM_CCER_CC1NE;
			}
		}
    }

    // ---------------------------------------------------------
    // RECTIFIER NOT ALLOWED
    //
    // Immediately stop inverter and reset the 2 sec sequence.
    // ---------------------------------------------------------
    if (!rectifier_allowed)
    {
        rectifier_to_inverter_delay_active = 0;
        rectifier_on_time = 0;

        pwm_allowed = 0;

        Stop_All_PWM();
    }

    // ---------------------------------------------------------
    // SAFETY / FAULT CONDITIONS
    // ---------------------------------------------------------
    if ((pwm_allowed &&  gbIsSystemNormalState == 0) ||  Mreceivedflag_pulse_on == 0 || Inrush_present == 1)
    {
        pwm_allowed = 0;

        rectifier_to_inverter_delay_active = 0;
        rectifier_on_time = 0;

        Stop_All_PWM();
    }
}
void Start_Deadtime(uint16_t delay_us)
{
    __HAL_TIM_SET_AUTORELOAD(&htim14, delay_us);
    __HAL_TIM_SET_COUNTER(&htim14, 0);
    HAL_TIM_Base_Start_IT(&htim14);
}
void TIM14_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim14, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim14, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);
            HAL_TIM_Base_Stop_IT(&htim14);

            /* One identical duty magnitude is used on both polarities. */
            uint16_t startup_limited_duty;

            // Update rectifier hysteresis status
            update_rectifier_allowed(vrms_in);
            update_pwm_allowed(vrms_in);
            startup_limited_duty = Inverter_Get_Limited_Duty();

            if (half_cycle == 1)
            {
                if (transformer_test_active)
                {
                    uint32_t elapsed_tf = HAL_GetTick() - transformer_test_timer;
                    if (elapsed_tf >= TRANSFORMER_ELAPSE_MIN)
                    {
                        // PC4/PC5 HIGH, PB3/PB5 LOW
                        GPIOC->BSRR = GPIO_PIN_4 | GPIO_PIN_5;              // set
                        GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;      // reset
                    }
                    return;
                }
//                if (con_state == 0) return;

                if (rectifier_allowed)
                {
                    GPIOC->BSRR = GPIO_PIN_4 | GPIO_PIN_5;              // set
                    GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;      // reset
					if (!rectifier_to_inverter_delay_active && !pwm_allowed)
					{
						rectifier_on_time = HAL_GetTick();
						rectifier_to_inverter_delay_active = 1;
					}
					else if (pwm_allowed == 1)
				    {
                        __HAL_TIM_SET_COUNTER(&htim1, 0);
                        __HAL_TIM_SET_COUNTER(&htim15, 0);

				    	if (vrms_in < voltage_setpoint_pid)
						{
							TIM1->CCR1  = startup_limited_duty;
							TIM15->CCR1 = 0;
						} else
						{
							TIM1->CCR1  = 0;
							TIM15->CCR1 = startup_limited_duty;
						}
				    }
				    else
				    {
				    	TIM1->CCR1  = 0;
						TIM15->CCR1 = 0;

						TIM1->CCER  &= ~TIM_CCER_CC1NE;
						TIM15->CCER &= ~TIM_CCER_CC1NE;
				    }
                }
                else
                {
                    GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;      // reset
                    GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;      // reset
                rectifier_to_inverter_delay_active = 0;
                rectifier_on_time = 0;
                }
            }
            else
            {
                if (transformer_test_active)
                {
                    uint32_t elapsed_tf = HAL_GetTick() - transformer_test_timer;
                    if (elapsed_tf >= TRANSFORMER_ELAPSE_MIN)
                    {
                        // PC4/PC5 LOW, PB3/PB5 HIGH
                        GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;   // reset
                        GPIOB->BSRR = GPIO_PIN_3 | GPIO_PIN_5;           // set
                    }
                    return;
                }
 //               if (con_state == 0) return;

                if (rectifier_allowed)
                {
                    GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;      // reset
                    GPIOB->BSRR = GPIO_PIN_3 | GPIO_PIN_5;              // set
                    if (!rectifier_to_inverter_delay_active && !pwm_allowed)
                    {
                        rectifier_on_time = HAL_GetTick();
                        rectifier_to_inverter_delay_active = 1;
                    }
                    else if (pwm_allowed == 1)
                    {
                        __HAL_TIM_SET_COUNTER(&htim1, 0);
                        __HAL_TIM_SET_COUNTER(&htim15, 0);

                    	if (vrms_in < voltage_setpoint_pid)
                		{

                			TIM1->CCR1  = 0;
                			TIM15->CCR1 = startup_limited_duty;

                		}
                		else
                		{
                			TIM1->CCR1  = startup_limited_duty;
                			TIM15->CCR1 = 0;
                		}

                    }
                    else
                    {
                    	TIM1->CCR1  = 0;
                		TIM15->CCR1 = 0;

                		TIM1->CCER  &= ~TIM_CCER_CC1NE;
                		TIM15->CCER &= ~TIM_CCER_CC1NE;
                    }
                }
                else
                {
                    GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;      // reset
                    GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;      // reset

                    // Rectifier OFF -> cancel inverter startup delay
                    rectifier_to_inverter_delay_active = 0;
                    rectifier_on_time = 0;
                }
            }
        }
    }
}


void Transformer_Test_Start(void)
{
    if (!transformer_test_active)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); // relay 2 close
        HAL_Delay(2000);
        transformer_test_active = 1;
//        con_state=0;
        transformer_test_timer = HAL_GetTick(); // mark start time
        HAL_Delay(1000);


//		uint8_t ready;
		uint16_t prev_ndtr, curr_ndtr;

		while(!cycle_ready)
		{

		}

//		__disable_irq();
//		ready = cycle_ready;
//		if (ready)
//		{
//			cycle_ready = 0;
//			prev_ndtr = zc_prev_ndtr;
//			curr_ndtr = zc_curr_ndtr;
//		}
//		__enable_irq();

		if(cycle_ready)
		{
						cycle_ready = 0;
						prev_ndtr = zc_prev_ndtr;
						curr_ndtr = zc_curr_ndtr;
			uint32_t prev_trip = ndtr_to_triplet_idx(prev_ndtr);
			uint32_t curr_trip = ndtr_to_triplet_idx(curr_ndtr);


			// Check plausible full-cycle length (do bounds here, not in ISR)
			uint32_t count = (curr_trip >= prev_trip)
							 ? (curr_trip - prev_trip)
							 : (NUM_SAMPLES - prev_trip + curr_trip);

			if (count >= 290 && count <= 480)

//			if (count >= 190 && count <= 320)
			{
			   vin  = Read_AC_Voltage_RMS_DMA(prev_trip, curr_trip, 0);
			   vout = Read_AC_Voltage_RMS_DMA(prev_trip, curr_trip, 1);
			}

		}

		Volt_Diff = vout - vin;

//	    working_settings.Nbb_tf = vin/Volt_Diff;

		Flip_mode = (Volt_Diff > offset) ? 1 : -1;

		HAL_Delay(2000); // Wait till the PWM pulses are off
		//Display the transformer connection status only after the pulses are off

	    if (Flip_mode < 0)
		{
           flipflag = 1;

		}
		else if (Flip_mode > 0)
		{
           flipflag = 2;
		}
    }
}

void Transformer_StopIfRequired(void)
{
    if (transformer_test_active)
    {
       uint32_t elapsed_tf = HAL_GetTick() - transformer_test_timer;

        if (elapsed_tf >= TRANSFORMER_ELAPSE_MAX) // 2s wait + 10s active
        {
            transformer_test_active = 0;

            transformer_test_timer = 0;
   //         con_state=1;
            // ---- Reset everything ----
            Stop_All_PWM();
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_5, GPIO_PIN_RESET);

        }
    }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{

    if (GPIO_Pin != GPIO_PIN_0) return;

    /* Highest priority: force BOTH rectifier diagonals off before any bookkeeping.
       Everything below (relay close, ZC monitor, DMA snapshot, permission updates)
       used to run first and delayed the turn-off after the zero-crossing. */
    if (!transformer_test_active)
    {
        GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;
        GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;
    }


    if (relay1_close_armed)
    {
        relay1_close_armed = 0;
        RELAY1_SET();
    }

    if (!transformer_test_active) ZeroCross_CheckAndUpdate(1);

    // Snapshot DMA remaining words and mark a cycle
	uint16_t ndtr = __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle);
	zc_prev_ndtr  = zc_curr_ndtr;
	zc_curr_ndtr  = ndtr;

	if (zc_seen_once) cycle_ready = 1;
	else zc_seen_once = 1;
	// ---------------------------------------------------------------------------

    if (transformer_test_active)
    {
        uint32_t elapsed_tf = HAL_GetTick() - transformer_test_timer;
        if (elapsed_tf >= TRANSFORMER_ELAPSE_MIN)
        {
            half_cycle = 1;

            // Reset output pins directly (faster than HAL_GPIO_WritePin)
            GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;
            GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;

            Enable_All_PWM();

            // Direct compare update
            TIM1->CCR1  = 2500;
            TIM15->CCR1 = 0;

            Start_Deadtime(DT);
        }
        Transformer_StopIfRequired();
        return;
    }

    update_rectifier_allowed(vrms_in);
    update_pwm_allowed(vrms_in);

    half_cycle = 1;

    GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;
    GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;

    Start_Deadtime(DT);

}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != GPIO_PIN_0) return;

    /* Highest priority: force BOTH rectifier diagonals off before any bookkeeping.
       Everything below (relay close, ZC monitor, DMA snapshot, permission updates)
       used to run first and delayed the turn-off after the zero-crossing. */
    if (!transformer_test_active)
    {
        GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;
        GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;
    }


    if (relay1_close_armed)
    {
        relay1_close_armed = 0;
        RELAY1_SET();
    }

    if (!transformer_test_active) ZeroCross_CheckAndUpdate(0);

    if (transformer_test_active)
    {
        uint32_t elapsed_tf = HAL_GetTick() - transformer_test_timer;
        if (elapsed_tf >= TRANSFORMER_ELAPSE_MIN)
        {
            half_cycle = 0;

            GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;
            GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;

            Enable_All_PWM();

            TIM1->CCR1  = 0;
            TIM15->CCR1 = 2500;


            Start_Deadtime(DT);
        }
        Transformer_StopIfRequired();
        return;
    }

    update_rectifier_allowed(vrms_in);
    update_pwm_allowed(vrms_in);
    half_cycle = 0;

    GPIOC->BSRR = (GPIO_PIN_4 | GPIO_PIN_5) << 16;
    GPIOB->BSRR = (GPIO_PIN_3 | GPIO_PIN_5) << 16;

    Start_Deadtime(DT);
}

static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch1_5_DMAMUX1_OVR_IRQn);

}
void adc_offset()
{
	const uint16_t total_cycles = 500;
    float offset_accum = 0.0f;
  //  float adc_offset_Local;

    for (uint16_t cycle = 0; cycle < total_cycles; cycle++)
    {
        // Wait until DMA transfer of one full cycle completes

    		uint8_t ready;
			uint16_t prev_ndtr, curr_ndtr;

			while (!cycle_ready);

			__disable_irq();
			ready = cycle_ready;
			if (ready)
			{
				cycle_ready = 0;
				prev_ndtr = zc_prev_ndtr;
				curr_ndtr = zc_curr_ndtr;
			}
			__enable_irq();

			uint32_t prev_trip = ndtr_to_triplet_idx(prev_ndtr);
			uint32_t curr_trip = ndtr_to_triplet_idx(curr_ndtr);


			// Check plausible full-cycle length (do bounds here, not in ISR)
			uint32_t count = (curr_trip >= prev_trip)
							 ? (curr_trip - prev_trip)
							 : (NUM_SAMPLES - prev_trip + curr_trip);

			//if (count < 8) return 0.0f;  // guard
			if (count >= 290 && count <= 480)
//			if (count >= 190 && count <= 320)

			{
				// Compute mean ADC value of channel 0 for this cycle
				uint32_t sum = 0;
				for (int i = 0; i < count; i++)
				{
					uint32_t trip = (prev_trip + i) & (NUM_SAMPLES - 1); // NUM_SAMPLES is power-of-2
					uint16_t raw = adc_dma_buffer[trip * NUM_CHANNELS + 0]; // channel 0
					sum += raw;

				}

        float avg_raw = (float)sum / count;
        float avg_voltage = (avg_raw * 3.3f) / 4095.0f;

        offset_accum += avg_voltage;
			}
    }

    adcoffset = offset_accum / total_cycles;
}

//static void MX_TIM7_Init(void)
//{
//
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  htim7.Instance = TIM7;
//  htim7.Init.Prescaler = 63;
//  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim7.Init.Period = 124;
//  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//}

static void MX_TIM14_Init(void)
{
    __HAL_RCC_TIM14_CLK_ENABLE();

    htim14.Instance = TIM14;
//    htim14.Init.Prescaler = 63;
//    htim14.Init.Period    = 200 - 1;
    htim14.Init.Prescaler = (64000000 / 1000000) - 1;  // 1 MHz timer clock → tick = 1 µs
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 100 - 1;                      // 100 µs delay
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM14_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM14_IRQn);
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};


  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.Period = 3199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  /* ------------ MASTER CONFIGURATION ------------ */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;          // Generate TRGO on update event
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;   // Enable master-slave mode
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* ------------------------------------------------ */

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }


  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;   // safer outputs when stopped
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;

  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;          // optional but recommended

  sBreakDeadTimeConfig.DeadTime = 64;                       // keep your deadtime

  sBreakDeadTimeConfig.BreakState    = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;  // if FLT goes LOW on fault
  sBreakDeadTimeConfig.BreakFilter   = 4;                    // small filter (0..15), tune

  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE; // AOE OFF = manual restart only
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
      Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}


/* ================================
 * TIM15 (SLAVE) INITIALIZATION
 * ================================ */
static void MX_TIM15_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};   // For synchronization
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.Period = 3199;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }

  /* ------------ SLAVE CONFIGURATION ------------ */
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;   // Reset counter on trigger
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;        // TIM15 triggered by TIM1 TRGO (ITR0)
  if (HAL_TIM_SlaveConfigSynchro(&htim15, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* --------------------------------------------- */

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  // --- TIM15 Break/Deadtime config (ONLY ONCE) ---
  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;

  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_1;     // optional but recommended
  sBreakDeadTimeConfig.DeadTime  = 64;

  sBreakDeadTimeConfig.BreakState    = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW; // if FLT pulls low on fault
  sBreakDeadTimeConfig.BreakFilter   = 4;               // small filter, tune later

  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE; // AOE OFF (manual restart)

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
      Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim15);
}


static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;  /* free-running 1us tick, used as zero-cross timebase (was 999) */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}
static void MX_TIM17_Init(void)
{
  /* USER CODE BEGIN TIM17_Init 0 */
  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */
  /* USER CODE END TIM17_Init 1 */

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 64000 - 1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 100;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM17_Init 2 */
  /* USER CODE END TIM17_Init 2 */
}


static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}


void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}



static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC11 PC1 PC4 PC5
                           PC8 PC9 */

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8; //RELAY 2
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);


   GPIO_InitStruct.Pin = GPIO_PIN_9; //RELAY 1
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_PULLDOWN;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);


  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);


  /*Configure GPIO pin : NTC_Pin */
  GPIO_InitStruct.Pin = NTC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NTC_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FU1_Pin FLT_Pin ON_OFF_Pin */
  GPIO_InitStruct.Pin = FU1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FLT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);


  GPIO_InitStruct.Pin = ON_OFF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 (buzzer - BUZZ_TONE_PORT/BUZZ_TONE_PIN) */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB3 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


   GPIO_InitStruct.Pin = GPIO_PIN_9;// FAN_Pin
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);


  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}
