/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  *
  * 1_2_3_4_5_6_9_11_12_13_16_17_19_All_point_addressed
  *
  *
  * Change 1 : Current & Button Press Working
  * Change 2 : Removed Done and added Discard Changes
  * Change 3 : Memory updated permanently
  * Change 4 : Fault Detection
  * Change 5 : Runtime menu setting done
  * Change 6 : Demo bending
  * Change 7 : Industrial Mode Done
  * Change 8 : Domestic Mode Done
  * Change 9 : Set Mode
  * Change 10 : Phase OUT OLD code
  * Change 11 : Delay Added for Domestic, ON and OFF mode turn ON
  * Change 12 : Domestic and Industrial Working...new file
  * Change 13 : Domestic individual start
  * Change 14 : Inrush Current display added scrolling
  * Change 15 : Inrush Display code && Median filter
  * Change 16 : Domestic Manual phase out issues solved
  * Change 17 : Domestic pulse on off rectified
  * Change 18 : Demo code ready
  * Change 19 : Domestic debouncing addedededed
  *
  * DEMO CODE  :  Phase out phase sequence need to add
  *
  * Final Demo
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <math.h>
#include "lcd.h"
#include "button.h"
#include "display.h"
#include "string.h"
#include "uart_receiver.h"
#include "master_monitor.h"   /* monitor mode: see master_monitor.h */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart4_rx;
DMA_HandleTypeDef hdma_usart4_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static uint8_t Industrial_RuntimePhaseSeq_CheckAndDisplay(void);
static uint8_t Domestic_HandlePB11_SelectiveRestart_AllFaults(void);
static inline uint8_t domestic_phase_present_now(uint8_t);
static uint8_t Domestic_DisplayOffState_WithRealFaults(void);


/* USER CODE BEGIN PFP */

#define MSG_LEN 6 // "[X]hi\r\n" is 6 characters
uint8_t rx1[MSG_LEN];
uint8_t rx2[MSG_LEN];
uint8_t rx3[MSG_LEN];
uint8_t uart1_ready = 0;
uint8_t uart2_ready = 0;
uint8_t uart3_ready = 0;
uint8_t Recceived =0 ;
uint8_t menureceiveflag = 0;
static char HVLVOUTtxbuf[64];
static inline void PowerButtons_Task_TIM6(void);
volatile uint8_t Fliptestcorrect= 0;

#define LINE_BUF_SIZE   64
#define RX_BUF_SIZE     64

// DMA RX buffers

volatile uint8_t buzzer_muted = 0;


// Parsed results
char currentPhase1 = 0, currentPhase2 = 0, currentPhase3 = 0;
uint8_t phasecame1 = 0, phasecame2 = 0, phasecame3 = 0;
uint8_t sFrameReceived1 = 0, sFrameReceived2 = 0, sFrameReceived3 = 0;

char sFrameData1[LINE_BUF_SIZE];
char sFrameData2[LINE_BUF_SIZE];
char sFrameData3[LINE_BUF_SIZE];

int HV1, LV1, SetVoltage1;
int HV2, LV2, SetVoltage2;
int HV3, LV3, SetVoltage3;
float vrms_in = 0.0f;
float vrms_out = 0.0f;
char phaseChar = 0;
int phaseRinput = 0;
int phaseRoutput = 0;
int phaseRcont = 0;

int phaseRheat = 0;
char currentPhase ;
int phaseInput;
int phaseOutput;
int phaseCont;
int pHeatFlag;
// vasanth #define FAULT_DELAY_SEC  5
/* Reduced from 5 to 2 per request (2-3 cycles instead of the previous
 * 5s) - single-phase-mode equivalent of FAULT_DELAY_LV/HV below. This
 * name suggests each tick here is ~1s (unlike the 3-phase path's 0.5s/
 * tick), so 2 is the direct "2 cycles" reading; nudge to 3 if 2 proves
 * too twitchy on your actual line noise. */
#define FAULT_DELAY_SEC  2

volatile uint8_t hv_count = 0;
volatile uint8_t lv_count = 0;

volatile uint8_t hv_fault = 0;
volatile uint8_t lv_fault = 0;

volatile uint8_t volt_state = 0;
/*
0 = normal
1 = HV counting
2 = LV counting
*/
uint8_t recover=1;

uint32_t contactor_timer = 0;


volatile uint8_t voltage_updated = 0;

// vasanth #define RECOVER_DELAY_SEC  5
#define RECOVER_DELAY_SEC  17

volatile uint8_t hv_recover_count = 0;
volatile uint8_t lv_recover_count = 0;
volatile uint8_t heat_count = 0;

volatile uint8_t recovering_hv = 0;
volatile uint8_t recovering_lv = 0;

volatile uint8_t USER_OFF = 0;


char uart_tx_buf[64];
uint8_t afterstartingmenu=0;
uint16_t curr_R;
uint16_t curr_Y;
uint16_t curr_B;

/////////////////////////////////////////////////////////////

/* Ring buffer for period samples (in microseconds, 16-bit is enough) */
#define FREQ_FBUF_SIZE     8                 // must be power of two: 2,4,8,16...
#define PERIOD_MIN_US      8000U             // ~ >120 Hz guard
#define PERIOD_MAX_US      30000U            // ~ <33 Hz guard
#define EDGE_GUARD_US      3000U             // ignore edges <3ms (chatter/noise)

static volatile uint16_t freq_buf[FREQ_FBUF_SIZE];
static volatile uint8_t  freq_head = 0;
static volatile uint8_t  freq_tail = 0;
static volatile uint16_t last_edge_us16 = 0;

/* Published frequency in centi-Hz (atomic 32-bit) to avoid float sharing */
static volatile uint32_t mains_freq_centiHz = 0;   // e.g., 500000 == 50.000 Hz

/* Optional: expose a cached float if you prefer (read via getter) */
static float mains_freq_hz_cached = 0.0f;

/* Helper macro: 16-bit ring */
#define FBUF_MASK          (FREQ_FBUF_SIZE - 1)
////////////////////////////////////////////////////////////


extern volatile uint8_t g_force_settings_menu;
extern void welcome_message(void);


#define DEBOUNCE_TICKS 25u  // adjust if TIM6 tick is not 1ms

typedef struct {
    uint8_t init_done;
    uint8_t integrator;
    uint8_t stable;     // 0 = released, 1 = pressed
} btn_int_db_t;


static btn_int_db_t db10 = {0,0,0}; // PB10
static btn_int_db_t db11 = {0,0,0}; // PB11
static btn_int_db_t db12 = {0,0,0}; // PB12
static btn_int_db_t db13 = {0,0,0}; // PB13


#define SETTINGS_COMBO_HOLD_MS  1500u
#define OFF_GUARD_MS            500u

static volatile uint8_t enter_settings_req = 0;

static uint8_t  combo_active = 0;
static uint32_t combo_start_tick = 0;

static uint8_t  pb12_pending_off = 0;
static uint32_t pb12_pending_tick = 0;
/* Domestic Manual startup: send C to only-present boards from main loop */
static volatile uint8_t dom_manual_start_req = 0u;

// Prevent power handler interfering while inside menu
typedef enum {
    UI_RUN = 0,
    UI_MENU
} ui_mode_t;

typedef enum
{
    PHCHK_OK = 1u,
    PHCHK_OPEN_SETTINGS = 2u
} phchk_result_t;

static phchk_result_t Phase_Check_Before_Start(void);
static phchk_result_t Phase_Presence_Check_Before_Start(void);

static volatile ui_mode_t ui_mode = UI_RUN;

typedef enum {
    PHASE_R = 0,
    PHASE_Y,
    PHASE_B
} phase_t;

#define IS_FAULT_STATE(s)  ( \
    (s == STATE_OC_FAULT)   || \
    (s == STATE_HV_FAULT)   || \
    (s == STATE_LV_FAULT)   || \
    (s == STATE_HEAT_FAULT) || \
    (s == STATE_HV_RECOVER) || \
    (s == STATE_LV_RECOVER) \
)


#define IS_CLEAR_STATE(s) ( \
    (s == STATE_CLEAR_HV_FAULT)  || \
    (s == STATE_CLEAR_LV_FAULT)  || \
    (s == STATE_CLEAR_OC_FAULT)  || \
    (s == STATE_CLEAR_HEAT_FAULT) \
)


typedef enum
{
    STATE_NORMAL = 0,

    STATE_HV_COUNT,
    STATE_LV_COUNT,
    STATE_OC_COUNT,
    STATE_HEAT_COUNT,       // NEW

    STATE_HV_FAULT,
    STATE_LV_FAULT,
    STATE_OC_FAULT,
    STATE_HEAT_FAULT,       // NEW

    STATE_HV_RECOVER,
    STATE_LV_RECOVER,

    STATE_CLEAR_HV_FAULT,
    STATE_CLEAR_LV_FAULT,
    STATE_CLEAR_OC_FAULT,
    STATE_CLEAR_HEAT_FAULT, // NEW

    STATE_OFF

} volt_state_t;


uint8_t oc_count = 0;          // seconds
// vasanth #define OC_DELAY_SEC 10
#define OC_DELAY_SEC 10
volatile uint8_t oc_recover_count = 0;

/* ================= UART FAULT STATUS ================= */
volatile uint8_t curr_fault_active = 0;
volatile uint8_t prev_fault_active = 0;

volatile uint8_t uart_send_F = 0;
volatile uint8_t uart_send_S = 0;

volatile uint8_t system_fault_active = 0;
volatile uint8_t contactor_enable_request = 0;

volatile uint8_t recovery_just_completed = 0;
//volatile uint8_t  system_is_off = 0;
static volatile uint8_t system_is_off = 0;

/* USER CODE END PV */
//#define HEAT_DELAY_SEC 30
#define HEAT_DELAY_SEC 1
uint32_t V_HYST = 5;
/* ================= INDUSTRIAL RUNTIME PHASE SEQUENCE =================
   Only used in 3P Industrial mode after runtime phase-out and return.
   Does NOT change phase-out logic; only gates restart when enabled. */
static volatile uint8_t ind_rt_phase_seq_required = 0u;   // set once any runtime phase-out occurs
static volatile uint8_t ind_manual_restart_req = 0u;      // PB11 request handled in main loop
static uint32_t ind_rt_phase_seq_last_try_ms = 0u;
#define IND_RT_PHASE_SEQ_RETRY_MS 700u
static volatile uint8_t ind_rt_phase_seq_display_lock = 0u; /* 1 = hold fixed runtime phase-seq fail screen */
static volatile uint8_t ind_rt_phase_seq_fail_result  = 0u; /* 2 = reverse, 3 = same phase */

/* ================= PHASE RETURN STABILITY FILTER ================= */
#define PHASE_MONITOR_PERIOD_MS    50u      // runtime sampling period
#define PHASE_RETURN_STABLE_MS   3000u      // phase must be continuously present for 3s before restart

static volatile uint8_t g_phase_present[3]     = {1,1,1}; // debounced / effective presence
static volatile uint8_t g_phase_raw_present[3] = {1,1,1}; // immediate detection only (optional debug)

static uint32_t g_phase_return_start_ms[3] = {0,0,0};     // recovery hold timer per phase


volatile uint8_t Rinrushjustdisplayflag;
volatile uint8_t Yinrushjustdisplayflag;
volatile uint8_t Binrushjustdisplayflag;
volatile uint8_t Sinrushjustdisplayflag;


void UART_SendStatus(char c)
{
    uint8_t tx[5] = { '[', c, ']', '\r', '\n' };
    HAL_UART_Transmit(&huart1, tx, 5, 100);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void send_based_on_pcb_id(void)
{
        HAL_UART_Transmit_DMA(&huart1,(uint8_t*)"[R]\r\n",5);
}


void send_settings_to_all(void)
{
    sprintf(HVLVOUTtxbuf,
            "[S]%lu,%lu,%lu\r\n",
            working_settings.hv_cut,
            working_settings.lv_cut,
            working_settings.ac_output_voltage);

        HAL_UART_Transmit_DMA(&huart1,(uint8_t*)HVLVOUTtxbuf,strlen(HVLVOUTtxbuf));
}

#define total_samples_adc      400
#define Current_CTR            766 //1914 //957 //1914  // 1246 PIC
/* Was 87 - over-reporting current by ~1.33x (reported: OL set to 8A was
 * tripping at an actual measured 6A). Corrected using that exact ratio:
 * 87 * (6/8) = 65.25. Re-verify against a real ammeter once you can and
 * nudge further if it's still off - this corrects the reported ratio
 * exactly, but wasn't independently re-measured here. */
#define current_multiplier     65.25f

uint32_t ADC_Read(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_39CYCLES_5;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint16_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return val;
}

unsigned int current_read(uint32_t adc_channel, uint8_t channel_num)
{
    uint32_t adc_sum = 0;
    uint16_t adc_avg;
    float current_f;

    // Safety: channel_num must be 0..2
    if (channel_num > 2) channel_num = 0;

    // 1) Average ADC samples
    for (uint16_t i = 0; i < total_samples_adc; i++)
    {
        adc_sum += (uint32_t)ADC_Read(adc_channel);
    }

    adc_avg = (uint16_t)(adc_sum / total_samples_adc);

    // 2) Convert STM32 12-bit ADC -> 10-bit (PIC-style)
    adc_avg >>= 2;

    // 3) Compute float current (no filtering)
    current_f = ((float)adc_avg * (float)current_multiplier) / (float)Current_CTR;             //   1914

    /* Clamp before the unsigned cast: at true zero current, sensor/ADC
     * noise can push current_f slightly negative. Casting a negative
     * float to unsigned int is undefined behavior in C - on this target
     * it was producing a wrapped-around large value that, once narrowed
     * to a smaller display field downstream, showed up as small negative
     * readings (-1, -2) with the contactor open and no real current
     * flowing. Current can't physically be negative here, so 0 is the
     * correct floor regardless. */
    if (current_f < 0.0f) current_f = 0.0f;

    // 4) Round to nearest integer
    return (unsigned int)roundf(current_f);
//    return (unsigned int)(current_f + 0.5f + 0.3f);

}

uint8_t phase_seq_result;


static inline uint8_t read_phase(phase_t ph)
{
    switch (ph)
    {
        case PHASE_R:
            return HAL_GPIO_ReadPin(R_Phase_GPIO_Port, R_Phase_Pin);

        case PHASE_Y:
            return HAL_GPIO_ReadPin(Y_Phase_GPIO_Port, Y_Phase_Pin);

        case PHASE_B:
            return HAL_GPIO_ReadPin(B_Phase_GPIO_Port, B_Phase_Pin);

        default:
            return 0;
    }
}


uint8_t wait_rising_edge(phase_t ph)
{
    uint32_t timeout = 0;

    // wait LOW
    while (read_phase(ph))
    {
        if (++timeout > 65000) return 0;
    }
//    typedef struct {
//        uint8_t init_done;
//        uint8_t integrator;
//        uint8_t stable; // 0 released, 1 pressed
//    } btn_int_db_t;


    timeout = 0;

    // wait HIGH
    while (!read_phase(ph))
    {
        if (++timeout > 65000) return 0;
    }

    return 1;
}
#if 0
uint8_t is_phase_present(phase_t ph)
{
    uint8_t seen_low = 0, seen_high = 0;

    for (uint32_t i = 0; i < 65000; i++)
    {
        uint8_t v = read_phase(ph);
        if (v) seen_high = 1;
        else   seen_low  = 1;

        if (seen_low && seen_high)
            return 1;
    }
    return 0;
}

#endif

uint8_t is_phase_present(phase_t ph)
{
    uint32_t start = HAL_GetTick();
    uint8_t first = read_phase(ph);

    while ((HAL_GetTick() - start) < 25u)   // watch for 25 ms
    {
        if (read_phase(ph) != first)
            return 1u;   // edge seen => phase present
    }

    return 0u;
}
static void PhaseOut_RuntimeTask(void)
{
    static uint32_t last_ms = 0;
    uint32_t now = HAL_GetTick();

    /* Run faster than before so phase loss is detected sooner */
    if ((now - last_ms) < PHASE_MONITOR_PERIOD_MS)
        return;
    last_ms = now;

    uint8_t raw[3];

    /* Raw instantaneous presence check */
    raw[0] = is_phase_present(PHASE_R) ? 1u : 0u;
    raw[1] = is_phase_present(PHASE_Y) ? 1u : 0u;
    raw[2] = is_phase_present(PHASE_B) ? 1u : 0u;

    for (uint8_t i = 0; i < 3; i++)
    {
        g_phase_raw_present[i] = raw[i];

        if (raw[i] == 0u)
        {
            /* Immediate OFF behavior on phase loss */
            g_phase_present[i] = 0u;
            g_phase_return_start_ms[i] = 0u;
        }
        else
        {
            /* Phase is seen present in raw check */

            if (g_phase_present[i] == 0u)
            {
                /* Phase is returning from OUT -> require continuous stable presence */
                if (g_phase_return_start_ms[i] == 0u)
                {
                    g_phase_return_start_ms[i] = now;
                }
                else if ((now - g_phase_return_start_ms[i]) >= PHASE_RETURN_STABLE_MS)
                {
                    /* Phase has been continuously present long enough */
                    g_phase_present[i] = 1u;
                    g_phase_return_start_ms[i] = 0u;
                }
            }
            else
            {
                /* Already healthy */
                g_phase_return_start_ms[i] = 0u;
            }
        }
    }
}



uint8_t detect_phase_out(void)
{

    if (!is_phase_present(PHASE_R)) return 3;
    if (!is_phase_present(PHASE_Y)) return 4;
    if (!is_phase_present(PHASE_B)) return 5;
    return 0;
}
const char* phase_out_to_string(uint8_t po)
{
    switch (po)
    {
        case 0: return "ALL PHASES PRESENT";
        case 3: return "  R PHASE MISSING  ";
        case 4: return "  Y PHASE MISSING  ";
        case 5: return "  B PHASE MISSING  ";
        default: return "UNKNOWN PHASE STATE";
    }
}

#if 0
uint8_t detect_phase_sequence(void)
{

    // Reference: B rising
    if (!wait_rising_edge(PHASE_B))
        return 2;

    uint8_t R_at_B = read_phase(PHASE_R);
    uint8_t Y_at_B = read_phase(PHASE_Y);

    // Reference: R rising
    if (!wait_rising_edge(PHASE_R))
        return 2;

    uint8_t Y_at_R = read_phase(PHASE_Y);
    uint8_t B_at_R = read_phase(PHASE_B);

    // Correct: R → Y → B
    if ((Y_at_B == 1 && R_at_B == 0) &&
        (B_at_R == 1 && Y_at_R == 0))
    {
        return 1;
    }

    return 2; // Reverse
}

#endif

uint8_t detect_phase_sequence(void)
{
    /* ---- 1) Reference: B rising ---- */
    if (!wait_rising_edge(PHASE_B))
        return 2;  // keep your existing timeout path

    uint8_t R_at_B = read_phase(PHASE_R);
    uint8_t Y_at_B = read_phase(PHASE_Y);

    /* In a valid 3φ system, at B↑ the other two must be opposite.
       If they are equal, R and Y are in-phase (duplication). */
    if (R_at_B == Y_at_B)
        return 3;  // SAME-PHASE (e.g., RRY/YYR)

    /* ---- 2) Reference: R rising ---- */
    if (!wait_rising_edge(PHASE_R))
        return 2;

    uint8_t Y_at_R = read_phase(PHASE_Y);
    uint8_t B_at_R = read_phase(PHASE_B);

    /* If equal here, Y and B are in-phase (duplication). */
    if (Y_at_R == B_at_R)
        return 3;  // SAME-PHASE (e.g., YYB/YBB)

    /* ---- 3) Reference: Y rising (added) ---- */
    if (!wait_rising_edge(PHASE_Y))
        return 2;

    uint8_t B_at_Y = read_phase(PHASE_B);
    uint8_t R_at_Y = read_phase(PHASE_R);

    /* If equal here, B and R are in-phase (duplication). */
    if (B_at_Y == R_at_Y)
        return 3;  // SAME-PHASE (e.g., RBB/BRR)

    /* ---- 4) Your existing correctness test ---- */
    /* Correct: R → Y → B
       (same as your current logic; left untouched) */
    if ((Y_at_B == 1 && R_at_B == 0) &&
        (B_at_R == 1 && Y_at_R == 0))
    {
        return 1;   // correct sequence (RYB)
    }

    return 2;       // reverse sequence
}

/* ================= DISPLAY-ONLY FILTERED VOLTAGES ================= */
int phaseRoutput_disp = 0;   // single-phase display only

int Rphaseoutput_disp = 0;   // three-phase display only
int Yphaseoutput_disp = 0;
int Bphaseoutput_disp = 0;
/////////////////////////////////////////////////////////////////////////

#define MED_WIN 5   // 5-sample median window

typedef struct
{
    int buf[MED_WIN];
    uint8_t idx;
    uint8_t count;
} median_filter_t;

static median_filter_t mf_sp_vout = {0};
static median_filter_t mf_r_vout  = {0};
static median_filter_t mf_y_vout  = {0};
static median_filter_t mf_b_vout  = {0};

static int median_filter_push(median_filter_t *f, int sample)
{
    int temp[MED_WIN];
    uint8_t n;
    int key;
    int j;

    /* Insert new sample into circular buffer */
    f->buf[f->idx] = sample;
    f->idx = (f->idx + 1) % MED_WIN;

    if (f->count < MED_WIN)
        f->count++;

    n = f->count;

    /* Copy valid samples */
    for (uint8_t i = 0; i < n; i++)
        temp[i] = f->buf[i];

    /* Insertion sort (fast enough for 5 samples) */
    for (uint8_t i = 1; i < n; i++)
    {
        key = temp[i];
        j = i - 1;

        while ((j >= 0) && (temp[j] > key))
        {
            temp[j + 1] = temp[j];
            j--;
        }
        temp[j + 1] = key;
    }

    /* Return median */
    return temp[n / 2];
}

static int apply_deadband(int displayed, int filtered, int db)
{
    if ((filtered > displayed + db) || (filtered < displayed - db))
        return filtered;
    return displayed;
}

//////////////////////three phase variables ///////////////////////

uint8_t uart1_rx_dma[RX_BUF_SIZE];
uint8_t uart4_rx_dma[RX_BUF_SIZE];
uint8_t uart3_rx_dma[RX_BUF_SIZE];

// Line buffers
char lineBuf1[LINE_BUF_SIZE];
char lineBuf4[LINE_BUF_SIZE];
char lineBuf3[LINE_BUF_SIZE];

volatile uint16_t lineIndex1 = 0;
volatile uint16_t lineIndex4 = 0;
volatile uint16_t lineIndex3 = 0;

volatile uint8_t RRecceived =0 ;
volatile uint8_t YRecceived =0 ;
volatile uint8_t BRecceived =0 ;

volatile uint8_t Ruart1_ready = 0;
volatile uint8_t Yuart2_ready = 0;
volatile uint8_t Buart3_ready = 0;

volatile uint8_t RFliptestcorrect= 0;
volatile uint8_t YFliptestcorrect= 0;
volatile uint8_t BFliptestcorrect= 0;

volatile uint8_t Rmenureceiveflag = 0;
volatile uint8_t Ymenureceiveflag = 0;
volatile uint8_t Bmenureceiveflag = 0;

char RphaseChar = 0;int Rphaseinput = 0;int Rphaseoutput = 0;int Rphasecont = 0;int Rphaseheat = 0;
char YphaseChar = 0;int Yphaseinput = 0;int Yphaseoutput = 0;int Yphasecont = 0;int Yphaseheat = 0;
char BphaseChar = 0;int Bphaseinput = 0;int Bphaseoutput = 0;int Bphasecont = 0;int Bphaseheat = 0;

char RreceiverChar = 0;int Rreceiverinput = 0;int RreceiverOutput = 0;int RreceiverCont = 0;int Rreceiverheat = 0; volatile uint8_t RreceiverInrush = 0;
char YreceiverChar = 0;int Yreceiverinput = 0;int YreceiverOutput = 0;int YreceiverCont = 0;int Yreceiverheat = 0; volatile uint8_t YreceiverInrush = 0;
char BreceiverChar = 0;int Breceiverinput = 0;int BreceiverOutput = 0;int BreceiverCont = 0;int Breceiverheat = 0; volatile uint8_t BreceiverInrush = 0;

/* ================= INRUSH STATUS ================= */

/* 1P */
volatile uint8_t sp_inrush_latched = 0;

/* 3P */
volatile uint8_t tp_inrush_latched[3] = {0,0,0};          // R,Y,B

static inline uint8_t Any_TP_Inrush_Active(void)
{
    return (RreceiverInrush || YreceiverInrush || BreceiverInrush) ? 1u : 0u;
}


uint8_t lcd_screen_index = 0;
uint32_t lcd_last_switch_time = 0;

// Three Phase Cut off Delays
//vasanth #define FAULT_DELAY_LV      10  // 5 sec
//vasanth #define FAULT_DELAY_HV      10
//vasanth #define FAULT_DELAY_OVERLOAD 20 // 10 sec
//vasanth #define FAULT_DELAY_HEAT   75  // 30 sec
/* Reduced from 10/10/20 ticks (5/5/10 sec) to 3 ticks (~1.5 sec) per
 * request: the real disconnect (contactor_enable_request / uart_send_F,
 * elsewhere) already reacts immediately on its own separate check - this
 * delay only gates how long the CONFIRMED fault display (SYS_FAULT_LV /
 * _HV / _OVERLOAD) lags behind that, so it was showing "...APPEARED" for
 * up to 5-10s after the real cutoff had already happened. Tune between 2
 * and 3 (ticks) if 3 is still too slow or too twitchy. */
#define FAULT_DELAY_LV      3
#define FAULT_DELAY_HV      3
#define FAULT_DELAY_OVERLOAD 3
#define FAULT_DELAY_HEAT   75

uint32_t R_lv_timer = 0;
uint32_t R_hv_timer = 0;
uint32_t R_overload_timer = 0;
uint32_t R_heat_timer = 0;

uint32_t Y_lv_timer = 0;
uint32_t Y_hv_timer = 0;
uint32_t Y_overload_timer = 0;
uint32_t Y_heat_timer = 0;

uint32_t B_lv_timer = 0;
uint32_t B_hv_timer = 0;
uint32_t B_overload_timer = 0;
uint32_t B_heat_timer = 0;

float Mainfrequency;



#define IS_PHASE_FAULT(s)   ( \
    (s == SYS_FAULT_LV)        || \
    (s == SYS_FAULT_HV)        || \
    (s == SYS_FAULT_OVERLOAD)  || \
    (s == SYS_FAULT_HEAT)      || \
    (s == SYS_FAULT_LV_ACTIVE) || \
    (s == SYS_FAULT_HV_ACTIVE) || \
    (s == SYS_FAULT_OVERLOAD_ACTIVE) || \
    (s == SYS_FAULT_LV_CLEAR)  || \
    (s == SYS_FAULT_HV_CLEAR)  || \
    (s == SYS_FAULT_OVERLOAD_CLEAR) || \
    (s == SYS_FAULT_HEAT_CLEAR)  || \
    (s == SYS_FAULT_PHASE_OUT)     || \
    (s == SYS_FAULT_PHASE_OUT_CLEAR) )  /* ✅ ADD */

#define IS_CLEAR_FAULT(s)   ( \
    (s == SYS_FAULT_LV_CLEAR) || \
    (s == SYS_FAULT_HV_CLEAR) || \
    (s == SYS_FAULT_OVERLOAD_CLEAR) || \
    (s == SYS_FAULT_HEAT_CLEAR) || \
    (s == SYS_FAULT_PHASE_OUT_CLEAR) )   /* ✅ ADD */

typedef enum
{
    R_PHASE = 0,
    Y_PHASE,
    B_PHASE

}SYS_PHASE_TYPE;

typedef enum
{
    SYS_FAULT_NONE = 0,


    SYS_FAULT_SYSTEM_OFF,     // ✅ NEW: for display when user presses STOP

    SYS_FAULT_LV_APPEAR,
    SYS_FAULT_HV_APPEAR,
    SYS_FAULT_OVERLOAD_APPEAR,
    SYS_FAULT_HEAT_APPEAR,

    SYS_FAULT_HEAT,
    SYS_FAULT_OVERLOAD,
    SYS_FAULT_HV,
    SYS_FAULT_LV,

    SYS_FAULT_LV_CLEAR,
    SYS_FAULT_HV_CLEAR,
    SYS_FAULT_OVERLOAD_CLEAR,
    SYS_FAULT_HEAT_CLEAR,

    SYS_FAULT_PHASE_OUT,

    SYS_FAULT_PHASE_OUT_CLEAR,     // ✅ ADD THIS

	SYS_FAULT_HV_ACTIVE,
	SYS_FAULT_LV_ACTIVE,
	SYS_FAULT_OVERLOAD_ACTIVE,   /* mirrors HV_ACTIVE/LV_ACTIVE, per request */

}SYS_FAULT_TYPE;

typedef struct
{

	SYS_FAULT_TYPE R_fault;
	SYS_FAULT_TYPE Y_fault;
	SYS_FAULT_TYPE B_fault;

}SYS_FAULT_STATUS;

SYS_FAULT_STATUS SysFault;

SYS_FAULT_TYPE R_prev_fault = SYS_FAULT_NONE;
SYS_FAULT_TYPE Y_prev_fault = SYS_FAULT_NONE;
SYS_FAULT_TYPE B_prev_fault = SYS_FAULT_NONE;



const char* GetFaultName(SYS_FAULT_TYPE fault)
{
    switch(fault)
    {
        case SYS_FAULT_NONE:              return "SYSTEM ON ";

        case SYS_FAULT_SYSTEM_OFF:  return "SYSTEM OFF ";

        case SYS_FAULT_LV_APPEAR:        return "LV APPEARED";
        case SYS_FAULT_HV_APPEAR:        return "HV APPEARED";
        case SYS_FAULT_OVERLOAD_APPEAR:  return "OVL APPEARED";
        case SYS_FAULT_HEAT_APPEAR:      return "HEAT APPEAR";

        case SYS_FAULT_HEAT:             return "HEAT      ";
        case SYS_FAULT_OVERLOAD:         return "OVER LOAD ";
        case SYS_FAULT_HV:               return "HIGH VOLT ";
        case SYS_FAULT_LV:               return "LOW VOLT  ";

        case SYS_FAULT_PHASE_OUT:        return "PHASE OUT ";
        case SYS_FAULT_PHASE_OUT_CLEAR:  return "PH OUT CLR";   // ✅ ADD
        case SYS_FAULT_LV_CLEAR:         return "LV CLEARED ";
        case SYS_FAULT_HV_CLEAR:         return "HV CLEARED ";
        case SYS_FAULT_OVERLOAD_CLEAR:   return "OVL CLEARED";
        case SYS_FAULT_HEAT_CLEAR:       return "HEAT CLEAR ";
        case SYS_FAULT_HV_ACTIVE: return "HV ACTIVE ";
        case SYS_FAULT_LV_ACTIVE: return "LV ACTIVE ";
        case SYS_FAULT_OVERLOAD_ACTIVE: return "OVL ACTIVE";

        default: return "UNKNOWN";
    }
}

typedef struct {
    int phaseInput;
    int phaseHeat;
    int currLoad;

    uint8_t phasePresent;     // ✅ ADD THIS

    uint32_t *lv_timer;
    uint32_t *hv_timer;
    uint32_t *overload_timer;
    uint32_t *heat_timer;
    uint32_t *active_timer;
    SYS_FAULT_TYPE *prev_fault;
    SYS_FAULT_TYPE *fault_out;
} PhaseContext;

//vasanth #define FAULT_ACTIVE_DISPLAY_TIME 15
#define FAULT_ACTIVE_DISPLAY_TIME 45
uint32_t R_active_timer=0;
uint32_t Y_active_timer=0;
uint32_t B_active_timer=0;

char lcd_buffer[4][21];
char lcd_old_buffer[4][21];


void LCD_Update(void)
{
    if(memcmp(lcd_old_buffer, lcd_buffer, sizeof(lcd_buffer)) != 0)
    {
        for(int i = 0; i < 4; i++)
        {
            LCD_SetCursor(i, 0);
            LCD_Print(lcd_buffer[i]);
        }

        memcpy(lcd_old_buffer, lcd_buffer, sizeof(lcd_buffer));
    }
}

void UART_SendStatus_All(char c)
{
    uint8_t tx[5] = { '[', c, ']', '\r', '\n' };

    HAL_UART_Transmit(&huart1, tx, 5, 100);

    HAL_UART_Transmit(&huart4, tx, 5, 100);
    HAL_UART_Transmit(&huart3, tx, 5, 100);
}



static uint8_t  dom_sent_F[3]       = {0,0,0};     // send F once per fault-entry

void UART_SendStatus_Domestic(UART_HandleTypeDef *huart, char c)
{
    uint8_t tx[5] = { '[', c, ']', '\r', '\n' };
    HAL_UART_Transmit(huart, tx, 5, 100);
}

/* =========================
   DOMESTIC (3P-D) CONTACTOR FSM
   Prevents flicker and enforces per-board pulse-based ON
   ========================= */
typedef enum
{
    DOM_ST_OFF = 0,        // system off
    DOM_ST_WAIT_LOW,       // wait slave ready=0 (flush stale 1)
    DOM_ST_WAIT_HIGH,      // wait slave ready=1 (after pulse ON)
    DOM_ST_DELAY_ON,       // delay before turning contactor ON
    DOM_ST_ON              // contactor ON
} dom_cont_fsm_t;

static dom_cont_fsm_t dom_fsm[3] = { DOM_ST_OFF, DOM_ST_OFF, DOM_ST_OFF };
static uint32_t       dom_fsm_tmr[3] = {0,0,0};

/* Send one 'C' automatically when a blocked board becomes unblocked (AUTO mode) */
static uint8_t        dom_need_C[3] = {0,0,0};

/* Track global OFF->ON transition for Domestic */
static uint8_t        dom_prev_system_off = 1u;

#define ALL_PHASES_PRESENT() (g_phase_present[0] && g_phase_present[1] && g_phase_present[2])

/* ================= 3P STAGGERED 'C' SENDER ================= */
#define C_STAGGER_GAP_MS   3000u   // 3 seconds between boards

static volatile uint8_t  cseq_active = 0;
static uint8_t           cseq_idx = 0;
static uint32_t          cseq_next_ms = 0;

static void CSeq_Stop_3Phase(void)
{
    cseq_active  = 0;
    cseq_idx     = 0;
    cseq_next_ms = 0;
}

/* Start sending C to 3 boards with delay */
static void CSeq_Start_3Phase(void)
{
    if (cseq_active) return;        // already running
    cseq_active  = 1;
    cseq_idx     = 0;
    cseq_next_ms = HAL_GetTick();   // first send immediately
}

static volatile uint8_t clear_sent = 1;

static void CSeq_Task_3Phase(void)
{
    if (!cseq_active) return;
    if (ui_mode == UI_MENU) return;

    /* Strong protection:
       If system is OFF, or all 3 phases are not stably present, or any fault exists,
       cancel the C sequence immediately. */
    uint8_t any_fault =
        IS_PHASE_FAULT(SysFault.R_fault) ||
        IS_PHASE_FAULT(SysFault.Y_fault) ||
        IS_PHASE_FAULT(SysFault.B_fault);

    if (system_is_off || !ALL_PHASES_PRESENT() || any_fault)
    {
        CSeq_Stop_3Phase();
        clear_sent = 0;   // restart sequence must begin fresh only after stable recovery
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - cseq_next_ms) < 0) return;

    UART_HandleTypeDef *hu[3] = { &huart1, &huart4, &huart3 }; // R,Y,B order

    /* Only block if TX is busy */
    if (hu[cseq_idx]->gState != HAL_UART_STATE_READY)
    {
        cseq_next_ms = now + 20u;
        return;
    }

    UART_SendStatus_Domestic(hu[cseq_idx], 'C');
    UART_SendStatus_Domestic(hu[cseq_idx], 'C');


    cseq_idx++;
    if (cseq_idx >= 3)
    {
        cseq_active = 0;   // sequence complete
    }
    else
    {
        cseq_next_ms = now + C_STAGGER_GAP_MS;
    }
}


/* Only monitors whether a runtime phase-out happened in Industrial + PhaseSeq=Y.
   This does NOT modify phase-out logic. */
static inline void Industrial_RuntimePhaseSeq_Monitor(void)
{
    if ((working_settings.pcb_id == 1) &&
        (working_settings.commericial_set == 'Y') &&
        (working_settings.ph_seq_cut == 'Y'))
    {
        if (!ALL_PHASES_PRESENT())
        {
            ind_rt_phase_seq_required = 1u;

            /* phase lost again -> clear fixed runtime seq fail display */
            ind_rt_phase_seq_display_lock = 0u;
            ind_rt_phase_seq_fail_result  = 0u;
        }
    }
    else
    {
        ind_rt_phase_seq_required = 0u;
        ind_rt_phase_seq_display_lock = 0u;
        ind_rt_phase_seq_fail_result  = 0u;
    }
}

/* AUTO mode helper:
   When all phases returned after runtime phase-out, check sequence before restart.
   Returns 1 only when restart is allowed. */
static uint8_t Industrial_RuntimePhaseSeq_ReadyForAutoRestart(void)
{
    if ((working_settings.pcb_id != 1) ||
        (working_settings.commericial_set != 'Y'))
    {
        return 1u;
    }

    /* If Phase Sequence setting is NO, do not block restart */
    if (working_settings.ph_seq_cut != 'Y')
    {
        ind_rt_phase_seq_required = 0u;
        return 1u;
    }

    /* Wait until all phases are stably present */
    if (!ALL_PHASES_PRESENT())
    {
        return 0u;
    }

    /* No runtime phase-out happened -> nothing to check */
    if (!ind_rt_phase_seq_required)
    {
        return 1u;
    }

    /* Retry with a small gap so it is not called every loop */
    uint32_t now = HAL_GetTick();
    if ((now - ind_rt_phase_seq_last_try_ms) < IND_RT_PHASE_SEQ_RETRY_MS)
    {
        return 0u;
    }
    ind_rt_phase_seq_last_try_ms = now;

    return Industrial_RuntimePhaseSeq_CheckAndDisplay();
}

/* MANUAL industrial restart is requested from PB11 path,
   but actual sequence check must run in main loop. */
static void Industrial_ManualRestart_Task(void)
{
    if (!ind_manual_restart_req)
        return;

    /* Safety: only for 3P Industrial */
    if ((working_settings.pcb_id != 1) ||
        (working_settings.commericial_set != 'Y'))
    {
        ind_manual_restart_req = 0u;
        return;
    }

    /* Existing behavior: wait until all phases are present */
    if (!ALL_PHASES_PRESENT())
        return;

    /* If phase sequence check is enabled and runtime phase-out happened,
       check sequence now before allowing restart */
    if ((working_settings.ph_seq_cut == 'Y') && ind_rt_phase_seq_required)
    {
        if (!Industrial_RuntimePhaseSeq_CheckAndDisplay())
        {
            /* wrong sequence -> stay OFF */
            ind_manual_restart_req = 0u;
            system_is_off = 1u;
            return;
        }
    }

    /* Existing 3P restart sequence */
    SysFault.R_fault = SYS_FAULT_NONE;
    SysFault.Y_fault = SYS_FAULT_NONE;
    SysFault.B_fault = SYS_FAULT_NONE;

    R_prev_fault = SYS_FAULT_NONE;
    Y_prev_fault = SYS_FAULT_NONE;
    B_prev_fault = SYS_FAULT_NONE;

    CSeq_Start_3Phase();
    system_is_off = 0;
    clear_sent = 2;
    contactor_timer = HAL_GetTick();

    ind_manual_restart_req = 0u;
}
/* Helper: true for 3-phase */
static inline uint8_t Is3Phase(void)
{
    return (working_settings.pcb_id == 1);
}
static inline uint8_t CSeq_IsActive(void)
{
    return cseq_active;
}

static inline void Clear_SystemOff_Faults_3Phase(void)
{
    if (SysFault.R_fault == SYS_FAULT_SYSTEM_OFF) SysFault.R_fault = SYS_FAULT_NONE;
    if (SysFault.Y_fault == SYS_FAULT_SYSTEM_OFF) SysFault.Y_fault = SYS_FAULT_NONE;
    if (SysFault.B_fault == SYS_FAULT_SYSTEM_OFF) SysFault.B_fault = SYS_FAULT_NONE;

    if (R_prev_fault == SYS_FAULT_SYSTEM_OFF) R_prev_fault = SYS_FAULT_NONE;
    if (Y_prev_fault == SYS_FAULT_SYSTEM_OFF) Y_prev_fault = SYS_FAULT_NONE;
    if (B_prev_fault == SYS_FAULT_SYSTEM_OFF) B_prev_fault = SYS_FAULT_NONE;
}

//////////////////////three phase variables ///////////////////////
int remain;
char buf[32];

void LCD_Display_Status_singlephase(void)
{
    /* ===== 1st priority: Inrush fault ===== */

	if(Sinrushjustdisplayflag)
	{
        snprintf(lcd_buffer[0],21," Single Phase Mode ");
        snprintf(lcd_buffer[1],21,"Vout:%03d  Vin:%03d",
                phaseRoutput,phaseRinput);
        snprintf(lcd_buffer[2],21,"Load:%02dA  F:%.1fHz",
                curr_R,Mainfrequency);
        snprintf(lcd_buffer[3], 21, "INRUSH CURRENT FLT");
        LCD_Update();
        return;
	}
    if (RreceiverInrush)
    {
        snprintf(lcd_buffer[0],21," Single Phase Mode ");
        snprintf(lcd_buffer[1],21,"Vout:%03d  Vin:%03d",
                phaseRoutput,phaseRinput);
        snprintf(lcd_buffer[2],21,"Load:%02dA  F:%.1fHz",
                curr_R,Mainfrequency);
        snprintf(lcd_buffer[3], 21, "INRUSH CURRENT FLT");
        LCD_Update();
        return;
    }

    snprintf(lcd_buffer[0],21," Single Phase Mode ");
    snprintf(lcd_buffer[1],21,"Vout:%03d  Vin:%03d",
            phaseRoutput,phaseRinput);
    snprintf(lcd_buffer[2],21,"Load:%02dA  F:%.1fHz",
    		 (curr_R > 0 ? curr_R - 3 : 0),Mainfrequency);

    switch (volt_state)
    {
    case STATE_OC_COUNT:
        snprintf(lcd_buffer[3],21,"OVERLOAD APPEARED");
        break;
    case STATE_HV_COUNT:
        snprintf(lcd_buffer[3],21,"HIGH VOLT APPEAR ");
        break;
    case STATE_LV_COUNT:
        snprintf(lcd_buffer[3],21,"LOW VOLT APPEAR  ");
        break;
    case STATE_HEAT_COUNT:
        snprintf(lcd_buffer[3],21,"HEAT APPEARED     ");
        break;
    case STATE_HV_FAULT:
        snprintf(lcd_buffer[3],21,"FAULT: HIGH VOLT ");
        break;
    case STATE_LV_FAULT:
        snprintf(lcd_buffer[3],21,"FAULT: LOW VOLT  ");
        break;
    case STATE_HEAT_FAULT:
        snprintf(lcd_buffer[3],21,"FAULT: HEAT      ");
        break;
    case STATE_OC_FAULT:
        snprintf(lcd_buffer[3],21,"FAULT: OVERLOAD  ");
        break;
    case STATE_HV_RECOVER:
        snprintf(lcd_buffer[3],21,"HV ACTIVE        ");
        break;
    case STATE_LV_RECOVER:
        snprintf(lcd_buffer[3],21,"LV ACTIVE        ");
        break;
    case STATE_CLEAR_HV_FAULT:
        snprintf(lcd_buffer[3],21,"FAULT: HV - CLR  ");
        break;
    case STATE_CLEAR_LV_FAULT:
        snprintf(lcd_buffer[3],21,"FAULT: LV - CLR  ");
        break;
    case STATE_CLEAR_OC_FAULT:
        snprintf(lcd_buffer[3],21,"FAULT: OVERLOAD-CLR");
        break;
    case STATE_CLEAR_HEAT_FAULT:
        snprintf(lcd_buffer[3],21,"FAULT: HEAT-CLR  ");
        break;
    case STATE_OFF:
        snprintf(lcd_buffer[3],21,"SYSTEM OFF        ");
        break;
    default:
        snprintf(lcd_buffer[3],21,"SYSTEM ON          ");
        break;
    }
    LCD_Update();
}


uint8_t clr = 0;

void LCD_Display_Status_threephase(void)
{
	   /* =====================================================
	       1st priority: INRUSH display
	       - Industrial : keep dedicated full-screen display
	       - Domestic   : DO NOT return here, allow normal scrolling
	       ===================================================== */
	    if (Any_TP_Inrush_Active() && working_settings.commericial_set == 'Y')
	    {
	        if (!clr)
	        {
	            LCD_Clear();
	            clr = 1;
	        }

	        snprintf(lcd_buffer[0], 21, " RYB : SYSTEM OFF");
	        uint8_t line = 1;

	        if (RreceiverInrush && line <= 3)
	            snprintf(lcd_buffer[line++], 21, "R : %-15s", "Inrush Current");

	        if (YreceiverInrush && line <= 3)
	            snprintf(lcd_buffer[line++], 21, "Y : %-15s", "Inrush Current");

	        if (BreceiverInrush && line <= 3)
	            snprintf(lcd_buffer[line++], 21, "B : %-15s", "Inrush Current");

	        /* IMPORTANT: clear unused remaining lines */
	        while (line <= 3)
	        {
	            snprintf(lcd_buffer[line], 21, "                    ");
	            line++;
	        }

	        LCD_Update();
	        return;
	    }

//		if((Rinrushjustdisplayflag || Yinrushjustdisplayflag || Binrushjustdisplayflag) && working_settings.commericial_set == 'Y')
//		{
//	        if (!clr)
//	        {
//	            LCD_Clear();
//	            clr = 1;
//	        }
//
//	        snprintf(lcd_buffer[0], 21, " RYB : SYSTEM OFF");
//	        uint8_t line = 1;
//
//	        if (Rinrushjustdisplayflag && line <= 3)
//	            snprintf(lcd_buffer[line++], 21, "R : %-15s", "Inrush Current");
//
//	        if (Yinrushjustdisplayflag && line <= 3)
//	            snprintf(lcd_buffer[line++], 21, "Y : %-15s", "Inrush Current");
//
//	        if (Binrushjustdisplayflag && line <= 3)
//	            snprintf(lcd_buffer[line++], 21, "B : %-15s", "Inrush Current");
//
//	        /* IMPORTANT: clear unused remaining lines */
//	        while (line <= 3)
//	        {
//	            snprintf(lcd_buffer[line], 21, "                    ");
//	            line++;
//	        }
//
//	        LCD_Update();
//	        return;
//		}
	    /* =====================================================
	           Runtime phase-sequence fail display lock
	           Industrial mode only:
	           If reverse phase or same phase happened after runtime phase return,
	           show only that message and do not allow scrolling screens.
	           ===================================================== */
	        if ((working_settings.commericial_set == 'Y') &&
	            ind_rt_phase_seq_display_lock)
	        {
	            LCD_Clear();

	            if (ind_rt_phase_seq_fail_result == 3u)
	            {
	                snprintf(lcd_buffer[0], 21, " ");
	                snprintf(lcd_buffer[1], 21, " PHASE FAULT ");
	                snprintf(lcd_buffer[2], 21, " SAME PHASE ");
	                snprintf(lcd_buffer[3], 21, " ");
	            }
	            else
	            {
	                snprintf(lcd_buffer[0], 21, " ");
	                snprintf(lcd_buffer[1], 21, " REVERSE SEQUENCE ");
	                snprintf(lcd_buffer[2], 21, " ");
	                snprintf(lcd_buffer[3], 21, " ");
	            }

	            LCD_Update();
	            return;
	        }

		if (HAL_GetTick() - lcd_last_switch_time >= 5000)
		{
			lcd_last_switch_time = HAL_GetTick();

			lcd_screen_index++;

			if(lcd_screen_index >= 0)
				lcd_screen_index = 0;
		}

		if(lcd_screen_index == 1)
		{
			if(working_settings.commericial_set == 'Y')
				snprintf(lcd_buffer[0],21,"3P-I  R    Y    B   ");
			else
				snprintf(lcd_buffer[0],21,"3P-D  R    Y    B   ");

			// R Phase
			if (SysFault.R_fault == SYS_FAULT_PHASE_OUT)
			{
			    Rphaseinput = 0;
			    Rphaseoutput = 0;
			}

			// Y Phase
			if (SysFault.Y_fault == SYS_FAULT_PHASE_OUT)
			{
			    Yphaseinput = 0;
			    Yphaseoutput = 0;
			}

			// B Phase
			if (SysFault.B_fault == SYS_FAULT_PHASE_OUT)
			{
			    Bphaseinput = 0;
			    Bphaseoutput = 0;
			}

			snprintf(lcd_buffer[1],21,"Vout %03d  %03d  %03d  ",Rphaseoutput_disp, Yphaseoutput_disp, Bphaseoutput_disp);
			snprintf(lcd_buffer[2],21,"Vin  %03d  %03d  %03d  ",Rphaseinput,Yphaseinput,Bphaseinput);
			//snprintf(lcd_buffer[3],21,"Load %03d  %03d  %03d  ",curr_R,curr_Y,curr_B);
			snprintf(lcd_buffer[3],21,"Load %03d  %03d  %03d  ",(curr_R > 0 ? curr_R - 3 : 0),(curr_Y > 0 ? curr_Y - 3 : 0),(curr_B > 0 ? curr_B - 3 : 0));
		}

		else if(lcd_screen_index == 2)
		{
			int Vin_RY = (Rphaseinput + Yphaseinput) * 0.866;
			int Vin_YB = (Yphaseinput + Bphaseinput) * 0.866;
			int Vin_BR = (Bphaseinput + Rphaseinput) * 0.866;

			int Vout_RY = (Rphaseoutput_disp + Yphaseoutput_disp) * 0.866;
			int Vout_YB = (Yphaseoutput_disp + Bphaseoutput_disp) * 0.866;
			int Vout_BR = (Bphaseoutput_disp + Rphaseoutput_disp) * 0.866;


			if(working_settings.commericial_set == 'Y')
				snprintf(lcd_buffer[0],21,"3P-I R-Y  Y-B  B-R   ");
			else
				snprintf(lcd_buffer[0],21,"3P-D R-Y  Y-B  B-R   ");

			snprintf(lcd_buffer[1],21,"Vout %03d  %03d  %03d",Vout_RY,Vout_YB,Vout_BR);
			snprintf(lcd_buffer[2],21,"Vin  %03d  %03d  %03d",Vin_RY,Vin_YB,Vin_BR);
			snprintf(lcd_buffer[3],21,"Freq %0.1f Hz       ",Mainfrequency);
		}
		else if (lcd_screen_index == 0)
		{
			/* =========================
			   INDUSTRIAL MODE (3P-I)
			   ========================= */
			if (working_settings.commericial_set == 'Y')
			{
				const char *sysTxt = system_is_off ? "SYSTEM OFF" : "SYSTEM ON   ";
				snprintf(lcd_buffer[0], 21, "   RYB : %-10s", sysTxt);

				uint8_t line = 1;

				for (int ph = 0; ph < 3 && line <= 3; ph++)
				{
					SYS_FAULT_TYPE f =
						(ph == 0) ? SysFault.R_fault :
						(ph == 1) ? SysFault.Y_fault :
									SysFault.B_fault;

					char phaseChar = (ph == 0) ? 'R' : (ph == 1) ? 'Y' : 'B'; /* ===================================================== CHECK INRUSH DISPLAY FLAG ===================================================== */
					uint8_t inrushFlag =
							(ph == 0) ? Rinrushjustdisplayflag :
							(ph == 1) ? Yinrushjustdisplayflag :
									Binrushjustdisplayflag; /* * If Inrush flag is received, display Inrush Current */ if (inrushFlag == 1) { snprintf(lcd_buffer[line], 21, "%c - %-16s", phaseChar, "Inrush Current"); line++; continue; }

					if (f == SYS_FAULT_NONE)
						continue;

					const char *msg;

					switch (f)
					{
						case SYS_FAULT_HV_APPEAR:       msg = "HIGH VOLT APP";      break;
						case SYS_FAULT_LV_APPEAR:       msg = "LOW VOLT APP";      break;
						case SYS_FAULT_OVERLOAD_APPEAR: msg = "Overload APP";   break;
						case SYS_FAULT_HEAT_APPEAR:     msg = "Heat APP";       break;

						case SYS_FAULT_HV:              msg = "High Volt";      break;
						case SYS_FAULT_LV:              msg = "Low Volt";       break;
						case SYS_FAULT_OVERLOAD:        msg = "Overload";       break;
						case SYS_FAULT_HEAT:            msg = "Excess Heat";           break;

						case SYS_FAULT_HV_CLEAR:        msg = "High Volt CLR";  break;
						case SYS_FAULT_LV_CLEAR:        msg = "Low Volt CLR";   break;
						case SYS_FAULT_OVERLOAD_CLEAR:  msg = "Overload CLR";   break;
						case SYS_FAULT_HEAT_CLEAR:      msg = "Heat CLR";       break;

						case SYS_FAULT_HV_ACTIVE:       msg = "HV Active";      break;
						case SYS_FAULT_LV_ACTIVE:       msg = "LV Active";      break;
						case SYS_FAULT_OVERLOAD_ACTIVE: msg = "Overload Active"; break;

						case SYS_FAULT_PHASE_OUT:       msg = "Phase Out";      break;
						case SYS_FAULT_PHASE_OUT_CLEAR:  msg = "Phase Out CLR";     break;
						case SYS_FAULT_SYSTEM_OFF:      msg = "SYSTEM OFF";     break;

						default:                        msg = "Unknown";       break;
					}

					//char phaseChar = (ph == 0) ? 'R' : (ph == 1) ? 'Y' : 'B';
					snprintf(lcd_buffer[line], 21, "%c - %-16s", phaseChar, msg);
					line++;
				}

				/* No events -> show No Fault only when OFF */
				if (line == 1)
				{
					if (system_is_off)
					{
						snprintf(lcd_buffer[1], 21, "      No Fault      ");
						snprintf(lcd_buffer[2], 21, "                    ");
						snprintf(lcd_buffer[3], 21, "                    ");
					}
					else
					{
						snprintf(lcd_buffer[1], 21, "                    ");
						snprintf(lcd_buffer[2], 21, "                    ");
						snprintf(lcd_buffer[3], 21, "                    ");
					}
				}
				else
				{
					while (line <= 3)
					{
						snprintf(lcd_buffer[line], 21, "                    ");
						line++;
					}
				}
			}

			/* =========================
			   DOMESTIC MODE (3P-D)
			   ========================= */
			else
			{
				uint8_t auto_mode = (working_settings.Restart_Mode == 'Y');

				SYS_FAULT_TYPE fr = SysFault.R_fault;
				SYS_FAULT_TYPE fy = SysFault.Y_fault;
				SYS_FAULT_TYPE fb = SysFault.B_fault;

				/* Separate helper:
				   In Domestic Auto, while system is OFF, preserve visible fault
				   information like PHASE OUT instead of overwriting all with SYSTEM OFF */
				if (Domestic_DisplayOffState_WithRealFaults())
				{
				    LCD_Update();
				    return;
				}

				/* In AUTO: hide CLR for all faults EXCEPT overload/heat (need PB11 ack) */
				if (auto_mode)
				{
				 if (IS_CLEAR_FAULT(fr) && fr != SYS_FAULT_OVERLOAD_CLEAR && fr != SYS_FAULT_HEAT_CLEAR) fr = SYS_FAULT_NONE;
				 if (IS_CLEAR_FAULT(fy) && fy != SYS_FAULT_OVERLOAD_CLEAR && fy != SYS_FAULT_HEAT_CLEAR) fy = SYS_FAULT_NONE;
				 if (IS_CLEAR_FAULT(fb) && fb != SYS_FAULT_OVERLOAD_CLEAR && fb != SYS_FAULT_HEAT_CLEAR) fb = SYS_FAULT_NONE;
				}

				/* Header:
				   - "Fault Status" if any confirmed fault OR (manual clear visible)
				   - else "System STATUS"
				*/
				uint8_t any_confirm =
					(fr == SYS_FAULT_HV) || (fr == SYS_FAULT_LV) || (fr == SYS_FAULT_OVERLOAD) || (fr == SYS_FAULT_HEAT) ||
					(fy == SYS_FAULT_HV) || (fy == SYS_FAULT_LV) || (fy == SYS_FAULT_OVERLOAD) || (fy == SYS_FAULT_HEAT) ||
					(fb == SYS_FAULT_HV) || (fb == SYS_FAULT_LV) || (fb == SYS_FAULT_OVERLOAD) || (fb == SYS_FAULT_HEAT);



				uint8_t any_clear_visible =
				 ((!auto_mode) &&
				  (IS_CLEAR_FAULT(SysFault.R_fault) ||
				   IS_CLEAR_FAULT(SysFault.Y_fault) ||
				   IS_CLEAR_FAULT(SysFault.B_fault)))
				 ||
				 (auto_mode &&
				  ((SysFault.R_fault == SYS_FAULT_OVERLOAD_CLEAR) || (SysFault.R_fault == SYS_FAULT_HEAT_CLEAR) ||
				   (SysFault.Y_fault == SYS_FAULT_OVERLOAD_CLEAR) || (SysFault.Y_fault == SYS_FAULT_HEAT_CLEAR) ||
				   (SysFault.B_fault == SYS_FAULT_OVERLOAD_CLEAR) || (SysFault.B_fault == SYS_FAULT_HEAT_CLEAR)));


				snprintf(lcd_buffer[0], 21, "%-20s",
						 (any_confirm || any_clear_visible) ? "Fault Status" : "System STATUS");

				/* ---------- Build text for each phase using switch(f) ---------- */
				const char *msgR = "SYSTEM ON";
				const char *msgY = "SYSTEM ON";
				const char *msgB = "SYSTEM ON";


				if (system_is_off)
				{
				    /* Domestic Manual only:
				       show Phase Out / Phase Out CLR even while system is OFF */
				    if (working_settings.Restart_Mode == 'N')
				    {
				        if (SysFault.R_fault == SYS_FAULT_PHASE_OUT)
				            msgR = "Phase Out";
				        else if (SysFault.R_fault == SYS_FAULT_PHASE_OUT_CLEAR)
				            msgR = "Phase Out CLR";
				        else
				            msgR = "SYSTEM OFF";

				        if (SysFault.Y_fault == SYS_FAULT_PHASE_OUT)
				            msgY = "Phase Out";
				        else if (SysFault.Y_fault == SYS_FAULT_PHASE_OUT_CLEAR)
				            msgY = "Phase Out CLR";
				        else
				            msgY = "SYSTEM OFF";

				        if (SysFault.B_fault == SYS_FAULT_PHASE_OUT)
				            msgB = "Phase Out";
				        else if (SysFault.B_fault == SYS_FAULT_PHASE_OUT_CLEAR)
				            msgB = "Phase Out CLR";
				        else
				            msgB = "SYSTEM OFF";
				    }
				    else
				    {
				        msgR = "SYSTEM OFF";
				        msgY = "SYSTEM OFF";
				        msgB = "SYSTEM OFF";
				    }
				}

				else
				{
				   /* ---------- R : INRUSH has highest priority ---------- */
					if(Rinrushjustdisplayflag)
					{
						msgR = "Inrush Current";
					}
					if (RreceiverInrush)
					{
						msgR = "Inrush Current";
					}
					else if (fr != SYS_FAULT_NONE)
					{
						switch (fr)
						{
							/* APPEAR */
							case SYS_FAULT_HV_APPEAR:       msgR = "High Volt APP";  break;
							case SYS_FAULT_LV_APPEAR:       msgR = "Low Volt APP";   break;
							case SYS_FAULT_OVERLOAD_APPEAR: msgR = "Overload APP";   break;
							case SYS_FAULT_HEAT_APPEAR:     msgR = "Heat APP";       break;

							/* CONFIRMED */
							case SYS_FAULT_HV:              msgR = "High Volt";      break;
							case SYS_FAULT_LV:              msgR = "Low Volt";       break;
							case SYS_FAULT_OVERLOAD:        msgR = "Overload";       break;
							case SYS_FAULT_HEAT:            msgR = "Excess Heat";           break;

							/* CLEAR */
							case SYS_FAULT_HV_CLEAR:        msgR = "High Volt CLR";  break;
							case SYS_FAULT_LV_CLEAR:        msgR = "Low Volt CLR";   break;
							case SYS_FAULT_OVERLOAD_CLEAR:  msgR = "Overload CLR  ";   break;
							case SYS_FAULT_HEAT_CLEAR:      msgR = "Heat CLR";       break;

							/* Optional / informational */
							case SYS_FAULT_HV_ACTIVE:       msgR = "HV Active";      break;
							case SYS_FAULT_LV_ACTIVE:       msgR = "LV Active";      break;
							case SYS_FAULT_OVERLOAD_ACTIVE: msgR = "Overload Active"; break;
							case SYS_FAULT_PHASE_OUT:        msgR = "Phase Out";    break;
							case SYS_FAULT_PHASE_OUT_CLEAR:  msgR = "Phase Out CLR";   break;

							default:                        msgR = "SYSTEM ON";      break;
						}
					}

					/* ---------- Y : INRUSH has highest priority ---------- */
					if(Yinrushjustdisplayflag)
					{
						msgY = "Inrush Current";
					}
					if (YreceiverInrush)
					{
						msgY = "Inrush Current";
					}
					else if (fy != SYS_FAULT_NONE)
					{
						switch (fy)
						{
							case SYS_FAULT_HV_APPEAR:       msgY = "High Volt APP";  break;
							case SYS_FAULT_LV_APPEAR:       msgY = "Low Volt APP";   break;
							case SYS_FAULT_OVERLOAD_APPEAR: msgY = "Overload APP";   break;
							case SYS_FAULT_HEAT_APPEAR:     msgY = "Heat APP";       break;

							case SYS_FAULT_HV:              msgY = "High Volt";      break;
							case SYS_FAULT_LV:              msgY = "Low Volt";       break;
							case SYS_FAULT_OVERLOAD:        msgY = "Overload";       break;
							case SYS_FAULT_HEAT:            msgY = "Excess Heat";           break;

							case SYS_FAULT_HV_CLEAR:        msgY = "High Volt CLR";  break;
							case SYS_FAULT_LV_CLEAR:        msgY = "Low Volt CLR";   break;
							case SYS_FAULT_OVERLOAD_CLEAR:  msgY = "Overload CLR  ";   break;
							case SYS_FAULT_HEAT_CLEAR:      msgY = "Heat CLR";       break;

							case SYS_FAULT_HV_ACTIVE:       msgY = "HV Active";      break;
							case SYS_FAULT_LV_ACTIVE:       msgY = "LV Active";      break;
							case SYS_FAULT_OVERLOAD_ACTIVE: msgY = "Overload Active"; break;
							case SYS_FAULT_PHASE_OUT:        msgY = "Phase Out";    break;
							case SYS_FAULT_PHASE_OUT_CLEAR:  msgY = "Phase Out CLR";   break;

							default:                        msgY = "SYSTEM ON";      break;
						}
					}

					/* ---------- B : INRUSH has highest priority ---------- */
					if(Binrushjustdisplayflag)
					{
						msgB = "Inrush Current";
					}
					if (BreceiverInrush)
					{
						msgB = "Inrush Current";
					}
					else if (fb != SYS_FAULT_NONE)
					{
						switch (fb)
						{
							case SYS_FAULT_HV_APPEAR:       msgB = "High Volt APP";  break;
							case SYS_FAULT_LV_APPEAR:       msgB = "Low Volt APP";   break;
							case SYS_FAULT_OVERLOAD_APPEAR: msgB = "Overload APP";   break;
							case SYS_FAULT_HEAT_APPEAR:     msgB = "Heat APP";       break;

							case SYS_FAULT_HV:              msgB = "High Volt";      break;
							case SYS_FAULT_LV:              msgB = "Low Volt";       break;
							case SYS_FAULT_OVERLOAD:        msgB = "Overload";       break;
							case SYS_FAULT_HEAT:            msgB = "Excess Heat";           break;

							case SYS_FAULT_HV_CLEAR:        msgB = "High Volt CLR";  break;
							case SYS_FAULT_LV_CLEAR:        msgB = "Low Volt CLR";   break;
							case SYS_FAULT_OVERLOAD_CLEAR:  msgB = "Overload CLR  ";   break;
							case SYS_FAULT_HEAT_CLEAR:      msgB = "Heat CLR";       break;

							case SYS_FAULT_HV_ACTIVE:       msgB = "HV Active";      break;
							case SYS_FAULT_LV_ACTIVE:       msgB = "LV Active";      break;
							case SYS_FAULT_OVERLOAD_ACTIVE: msgB = "Overload Active"; break;
							case SYS_FAULT_PHASE_OUT:        msgB = "Phase Out";    break;
							case SYS_FAULT_PHASE_OUT_CLEAR:  msgB = "Phase Out CLR";   break;

							default:                        msgB = "SYSTEM ON";      break;
						}
					}
				}

				/* IMPORTANT: Use %-12.12s to prevent overflow on LCD */
				snprintf(lcd_buffer[1], 21, "R : %-17.17s", msgR);
				snprintf(lcd_buffer[2], 21, "Y : %-17.17s", msgY);
				snprintf(lcd_buffer[3], 21, "B : %-17.17s", msgB);
			}
		}

	LCD_Update();
}

uint8_t cont1_delay_started = 0;
uint32_t cont1_delay_start_time = 0;

void Contactor1_Handler(void)
{

    /* ===== INRUSH handling: highest priority ===== */
    if (RreceiverInrush)
    {
        sp_inrush_latched = 1u;

        HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_RESET);
        UART_SendStatus('F');          // keep slave OFF

        system_is_off = 1u;
        USER_OFF = 0u;
        return;
    }

    /* Inrush removed -> clear latch immediately */
    if (sp_inrush_latched && !RreceiverInrush)
    {
        sp_inrush_latched = 0u;
    }


    /* ========== FAULT MESSAGE ========== */
    if (uart_send_F)
    {
        HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_RESET);
        UART_SendStatus('F');

        cont1_delay_started = 0;   // reset delay
        return;                   // stop further execution
    }

    if (system_is_off)
    {
        HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_RESET);
        return;
    }


    /* ======================================================
       AUTO MODE  (Restart_Mode == 'Y')
       ====================================================== */
    if (working_settings.Restart_Mode == 'N')
    {
        if (phaseRcont == 1)
        {
            if (!cont1_delay_started)
            {
                cont1_delay_started = 1;
                cont1_delay_start_time = HAL_GetTick();
            }

            if ((HAL_GetTick() - cont1_delay_start_time) >= 250)
            {
                HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_SET);
            }
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_RESET);
            cont1_delay_started = 0;
        }
    }

    /* ======================================================
       MANUAL MODE  (Restart_Mode == 'N')
       ====================================================== */
    else
    {
        /* Wait for recovery message */
        if (uart_send_S)
        {
            uart_send_F = 0;
            uart_send_S = 0;
            UART_SendStatus('C');
            UART_SendStatus('C');
        }

        if (!uart_send_F && phaseRcont == 1)
        {
            if (!cont1_delay_started)
            {
                cont1_delay_started = 1;
                cont1_delay_start_time = HAL_GetTick();
            }

            if ((HAL_GetTick() - cont1_delay_start_time) >= 250)
            {
                HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_SET);
            }
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_RESET);
            cont1_delay_started = 0;
        }
    }
}

static volatile uint8_t off_screen_req = 0;   // request LCD "SYSTEM OFF" draw (from main)

/* USER CODE BEGIN 0 */


//static btn_int_db_t db11 = {0,0,0};
//static btn_int_db_t db12 = {0,0,0};

static inline uint8_t raw_pressed(GPIO_TypeDef *port, uint16_t pin)
{
    // Your GPIO is configured PULLUP, so pressed = RESET (active-low)
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

static inline uint8_t debounce_press_edge(btn_int_db_t *db, uint8_t raw)
{
    if (!db->init_done) {
        db->init_done = 1;
        db->stable = raw;
        db->integrator = raw ? DEBOUNCE_TICKS : 0u;
        return 0u; // never generate event on init
    }

    if (raw) {
        if (db->integrator < DEBOUNCE_TICKS) db->integrator++;
    } else {
        if (db->integrator > 0u) db->integrator--;
    }

    uint8_t new_stable = (db->integrator >= DEBOUNCE_TICKS) ? 1u : 0u;

    // press edge: stable changes 0 -> 1
    if ((new_stable == 1u) && (db->stable == 0u)) {
        db->stable = 1u;
        return 1u;
    }

    // update stable when fully released
    if ((new_stable == 0u) && (db->integrator == 0u)) {
        db->stable = 0u;
    }

    return 0u;
}
static volatile uint8_t startup_wait_manual = 0;
static volatile uint8_t startup_pb11_event  = 0;
/* USER CODE END PV */

static btn_int_db_t db_start_pb11 = {0,0,0};
static inline void StartupButton_Task_TIM6(void)
{
    // Only active while we are waiting at startup
    if (!startup_wait_manual) return;

    // Detect PB11 press edge using a dedicated debounce state
    uint8_t pb11_press = debounce_press_edge(&db_start_pb11,
                                             raw_pressed(GPIOB, GPIO_PIN_11));

    if (pb11_press)
    {
        startup_pb11_event = 1;   // signal main()
        startup_wait_manual = 0;  // stop waiting
    }
}

static inline void System_EnterOffState_ISRsafe(void)
{
	if(working_settings.pcb_id == 0)
	{
    // OFF outputs
		HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_RESET);
		UART_SendStatus('F');
		system_is_off = 1u;
		USER_OFF = 1;
		off_screen_req = 1u; // ask main loop to draw OFF screen
		volt_state = STATE_OFF;
	}
	else if(working_settings.pcb_id == 1)
	{
	    HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOA, CONT_2_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOA, CONT_3_Pin, GPIO_PIN_RESET);
        UART_SendStatus_All('F');
        UART_SendStatus_All('F');
		system_is_off = 1u;

	}
}

static inline void runtime_setting_menu(void)
{
    // OFF outputs
	if(working_settings.pcb_id == 0)
	{
    HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_RESET);
    UART_SendStatus('A');
    UART_SendStatus('A');
	UART_SendStatus('A'); // When slave receives A pulse should be OFF and recall the Handshake while loop inside the main while loop
    system_is_off = 1u;
    USER_OFF = 1;
    off_screen_req = 1u; // ask main loop to draw OFF screen
    volt_state = STATE_OFF;
	}
	else if(working_settings.pcb_id == 1)
	{
	    HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOA, CONT_2_Pin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOA, CONT_3_Pin, GPIO_PIN_RESET);

	    UART_SendStatus_All('A');
	    UART_SendStatus_All('A');
	    UART_SendStatus_All('A');

	    system_is_off = 1u;
	    /* Set all phases to SYSTEM OFF */
	    SysFault.R_fault = SYS_FAULT_SYSTEM_OFF;
	    SysFault.Y_fault = SYS_FAULT_SYSTEM_OFF;
	    SysFault.B_fault = SYS_FAULT_SYSTEM_OFF;

	    R_prev_fault = SYS_FAULT_SYSTEM_OFF;
	    Y_prev_fault = SYS_FAULT_SYSTEM_OFF;
	    B_prev_fault = SYS_FAULT_SYSTEM_OFF;
	}
}



/* ✅ ADD THIS: remember which fault caused blocking (needed for MANUAL phase-out auto recovery) */
static SYS_FAULT_TYPE dom_last_block_fault[3] =
{
    SYS_FAULT_NONE, SYS_FAULT_NONE, SYS_FAULT_NONE
};

/* =========================================================
   Domestic selective restart request (separate/additive)
   bit0 = R, bit1 = Y, bit2 = B
   ========================================================= */
static volatile uint8_t dom_partial_start_req  = 0u;
static volatile uint8_t dom_partial_start_mask = 0u;

/* =========================================================
   Domestic LCD fault text helper
   Separate function only
   ========================================================= */
static const char* Domestic_FaultMsg_ForLCD(SYS_FAULT_TYPE f)
{
    switch (f)
    {
        case SYS_FAULT_HV_APPEAR:        return "High Volt APP";
        case SYS_FAULT_LV_APPEAR:        return "Low Volt APP";
        case SYS_FAULT_OVERLOAD_APPEAR:  return "Overload APP";
        case SYS_FAULT_HEAT_APPEAR:      return "Heat APP";

        case SYS_FAULT_HV:               return "High Volt";
        case SYS_FAULT_LV:               return "Low Volt";
        case SYS_FAULT_OVERLOAD:         return "Overload";
        case SYS_FAULT_HEAT:             return "Excess Heat";

        case SYS_FAULT_HV_CLEAR:         return "High Volt CLR";
        case SYS_FAULT_LV_CLEAR:         return "Low Volt CLR";
        case SYS_FAULT_OVERLOAD_CLEAR:   return "Overload CLR";
        case SYS_FAULT_HEAT_CLEAR:       return "Heat CLR";

        case SYS_FAULT_HV_ACTIVE:        return "HV Active";
        case SYS_FAULT_LV_ACTIVE:        return "LV Active";
        case SYS_FAULT_OVERLOAD_ACTIVE:  return "Overload Active";

        case SYS_FAULT_PHASE_OUT:        return "Phase Out";
        case SYS_FAULT_PHASE_OUT_CLEAR:  return "Phase Out CLR";

        case SYS_FAULT_SYSTEM_OFF:       return "SYSTEM OFF";
        case SYS_FAULT_NONE:
        default:                         return "SYSTEM ON";
    }
}

/* =========================================================
   Helper: true if this fault should remain visible on LCD
   while system is OFF
   ========================================================= */
static inline uint8_t Domestic_IsVisibleFault(SYS_FAULT_TYPE f)
{
    return (f != SYS_FAULT_NONE && f != SYS_FAULT_SYSTEM_OFF) ? 1u : 0u;
}

/* =========================================================
   Helper: current inrush status by board index
   ========================================================= */
static inline uint8_t Domestic_InrushByIndex(uint8_t idx)
{
    switch (idx)
    {
        case 0: return (uint8_t)RreceiverInrush;
        case 1: return (uint8_t)YreceiverInrush;
        case 2: return (uint8_t)BreceiverInrush;
        default: return 0u;
    }
}

/* =========================================================
   Helper: get fault pointer by board index
   ========================================================= */
static inline SYS_FAULT_TYPE* Domestic_FaultPtr(uint8_t idx)
{
    switch (idx)
    {
        case 0: return &SysFault.R_fault;
        case 1: return &SysFault.Y_fault;
        case 2: return &SysFault.B_fault;
        default: return &SysFault.R_fault;
    }
}

static inline SYS_FAULT_TYPE* Domestic_PrevFaultPtr(uint8_t idx)
{
    switch (idx)
    {
        case 0: return &R_prev_fault;
        case 1: return &Y_prev_fault;
        case 2: return &B_prev_fault;
        default: return &R_prev_fault;
    }
}

/* =========================================================
   Helper: board is blocked for restart if any real fault /
   clear / active / inrush / phase-missing condition exists.
   This is intentionally broader than phase-out only.
   ========================================================= */
static inline uint8_t Domestic_IsBoardBlockedForRestart(uint8_t idx)
{
    SYS_FAULT_TYPE f = *Domestic_FaultPtr(idx);

    /* No stable phase -> do not restart */
    if (!g_phase_present[idx])
        return 1u;

    /* Inrush -> do not restart */
    if (Domestic_InrushByIndex(idx))
        return 1u;

    switch (f)
    {
        /* appearance states */
        case SYS_FAULT_HV_APPEAR:
        case SYS_FAULT_LV_APPEAR:
        case SYS_FAULT_OVERLOAD_APPEAR:
        case SYS_FAULT_HEAT_APPEAR:

        /* confirmed fault states */
        case SYS_FAULT_HV:
        case SYS_FAULT_LV:
        case SYS_FAULT_OVERLOAD:
        case SYS_FAULT_HEAT:

        /* active / hold states */
        case SYS_FAULT_HV_ACTIVE:
        case SYS_FAULT_LV_ACTIVE:
        case SYS_FAULT_OVERLOAD_ACTIVE:

        /* phase states */
        case SYS_FAULT_PHASE_OUT:
        case SYS_FAULT_PHASE_OUT_CLEAR:

        /* clear states must not auto-run here;
           your existing PB11 clear handling remains untouched */
        case SYS_FAULT_HV_CLEAR:
        case SYS_FAULT_LV_CLEAR:
        case SYS_FAULT_OVERLOAD_CLEAR:
        case SYS_FAULT_HEAT_CLEAR:
            return 1u;

        case SYS_FAULT_NONE:
        case SYS_FAULT_SYSTEM_OFF:
        default:
            return 0u;
    }
}

/* =========================================================
   Send C only to selected boards
   bit0 = R, bit1 = Y, bit2 = B
   This does NOT overwrite phase presence arrays.
   ========================================================= */
static void Domestic_SendC_To_SelectedBoards(uint8_t mask)
{
    if ((mask & 0x01u) && domestic_phase_present_now(0))
    {
        while (huart1.gState != HAL_UART_STATE_READY) { HAL_Delay(1); }
        UART_SendStatus_Domestic(&huart1, 'C');
        UART_SendStatus_Domestic(&huart1, 'C');
        HAL_Delay(100);
    }

    if ((mask & 0x02u) && domestic_phase_present_now(1))
    {
        while (huart4.gState != HAL_UART_STATE_READY) { HAL_Delay(1); }
        UART_SendStatus_Domestic(&huart4, 'C');
        UART_SendStatus_Domestic(&huart1, 'C');
        HAL_Delay(100);
    }

    if ((mask & 0x04u) && domestic_phase_present_now(2))
    {
        while (huart3.gState != HAL_UART_STATE_READY) { HAL_Delay(1); }
        UART_SendStatus_Domestic(&huart3, 'C');
        UART_SendStatus_Domestic(&huart1, 'C');
        HAL_Delay(100);
    }
}

/* =========================================================
   Domestic OFF-state LCD override:
   While system is OFF, preserve real fault display for
   faulty boards and SYSTEM OFF only for healthy boards.
   Works for BOTH Auto and Manual.
   Returns 1 if display handled.
   ========================================================= */
static uint8_t Domestic_DisplayOffState_WithRealFaults(void)
{
    if (working_settings.pcb_id != 1) return 0u;
    if (working_settings.commericial_set != 'N') return 0u; /* Domestic only */
    if (!system_is_off) return 0u;

    SYS_FAULT_TYPE fR = SysFault.R_fault;
    SYS_FAULT_TYPE fY = SysFault.Y_fault;
    SYS_FAULT_TYPE fB = SysFault.B_fault;

    uint8_t any_visible_fault =
        Domestic_IsVisibleFault(fR) ||
        Domestic_IsVisibleFault(fY) ||
        Domestic_IsVisibleFault(fB);

    if (!any_visible_fault)
        return 0u; /* keep your existing SYSTEM OFF screen */

    const char *msgR = Domestic_IsVisibleFault(fR) ? Domestic_FaultMsg_ForLCD(fR) : "SYSTEM OFF";
    const char *msgY = Domestic_IsVisibleFault(fY) ? Domestic_FaultMsg_ForLCD(fY) : "SYSTEM OFF";
    const char *msgB = Domestic_IsVisibleFault(fB) ? Domestic_FaultMsg_ForLCD(fB) : "SYSTEM OFF";

    snprintf(lcd_buffer[0], 21, "%-20s", "Fault Status");
    snprintf(lcd_buffer[1], 21, "R : %-17.17s", msgR);
    snprintf(lcd_buffer[2], 21, "Y : %-17.17s", msgY);
    snprintf(lcd_buffer[3], 21, "B : %-17.17s", msgB);

    return 1u;
}

/* =========================================================
   Domestic PB11 selective restart for ALL faults
   - Faulty board stays OFF and keeps its fault text
   - Healthy board(s) restart
   - Works in BOTH Auto and Manual
   - Does not replace existing logic; only intercepts
     partial-restart case
   Returns 1 if handled.
   ========================================================= */
static uint8_t Domestic_HandlePB11_SelectiveRestart_AllFaults(void)
{
    if (working_settings.pcb_id != 1) return 0u;
    if (working_settings.commericial_set != 'N') return 0u; /* Domestic only */
    if (!system_is_off) return 0u;

    uint8_t restart_mask = 0u;
    uint8_t blocked_count = 0u;

    for (uint8_t i = 0; i < 3; i++)
    {
        if (Domestic_IsBoardBlockedForRestart(i))
        {
            blocked_count++;
        }
        else
        {
            restart_mask |= (1u << i);
        }
    }

    /* No board can restart -> keep OFF, keep faults visible */
    if (restart_mask == 0u)
    {
        return 1u;
    }

    /* All 3 boards are healthy -> keep your old full-restart logic */
    if (restart_mask == 0x07u && blocked_count == 0u)
    {
        return 0u;
    }

    /* Partial restart case:
       only healthy boards should become SYSTEM ON */
    for (uint8_t i = 0; i < 3; i++)
    {
        SYS_FAULT_TYPE *pf = Domestic_FaultPtr(i);
        SYS_FAULT_TYPE *pp = Domestic_PrevFaultPtr(i);

        if (restart_mask & (1u << i))
        {
            /* Restartable board should run and show normal status */
            *pf = SYS_FAULT_NONE;
            *pp = SYS_FAULT_NONE;
        }
        else
        {
            /* Keep the actual fault state.
               If phase missing but state was overwritten, restore PHASE OUT. */
            if (!g_phase_present[i] &&
                (*pf == SYS_FAULT_NONE || *pf == SYS_FAULT_SYSTEM_OFF))
            {
                *pf = SYS_FAULT_PHASE_OUT;
                *pp = SYS_FAULT_PHASE_OUT;
            }
        }
    }

    system_is_off = 0u;
    USER_OFF = 0u;
    clear_sent = 1u;
    contactor_timer = HAL_GetTick();

    dom_partial_start_mask = restart_mask;
    dom_partial_start_req  = 1u;

    return 1u;
}



static inline void PowerButtons_Task_TIM6(void)
{
    if (volt_state == STATE_CLEAR_HEAT_FAULT ||
        volt_state == STATE_CLEAR_HV_FAULT   ||
        volt_state == STATE_CLEAR_LV_FAULT   ||
        volt_state == STATE_CLEAR_OC_FAULT)
    {
        system_is_off = 1u;
        USER_OFF = 0;
    }

    /* In 3P-D (Domestic) clear is per-board, so DON'T force global OFF */
    if (working_settings.pcb_id == 1 && working_settings.commericial_set == 'Y')
    {
        if ( IS_CLEAR_FAULT(SysFault.R_fault) ||
             IS_CLEAR_FAULT(SysFault.Y_fault) ||
             IS_CLEAR_FAULT(SysFault.B_fault) )
        {
            system_is_off = 1u;
            USER_OFF = 0;
        }
    }

    if (ui_mode == UI_MENU) return;

    if(working_settings.pcb_id == 0)
    {
		if (volt_state == STATE_NORMAL)
		{
			buzzer_muted = 0;   // ready for next fault
		}
    }
    else
    {
			if (!IS_PHASE_FAULT(SysFault.R_fault) &&
			!IS_PHASE_FAULT(SysFault.Y_fault) &&
			!IS_PHASE_FAULT(SysFault.B_fault))
		{
			buzzer_muted = 0;   // ready for next fault
		}
    }

    uint8_t r10 = raw_pressed(GPIOB, GPIO_PIN_10);
    uint8_t r11 = raw_pressed(GPIOB, GPIO_PIN_11);
    uint8_t r12 = raw_pressed(GPIOB, GPIO_PIN_12);
    uint8_t r13 = raw_pressed(GPIOB, GPIO_PIN_13);

    (void)debounce_press_edge(&db10, r10);
    (void)debounce_press_edge(&db13, r13);

    uint8_t pb11_press = debounce_press_edge(&db11, r11);
    uint8_t pb12_press = debounce_press_edge(&db12, r12);

    uint8_t pb10_stable = db10.stable;
    uint8_t pb13_stable = db13.stable;

    /* =========================================================
       3P DOMESTIC PB11 HANDLING
       ---------------------------------------------------------
       1) First acknowledge INRUSH CLR (per-board only)
       2) Then acknowledge normal CLR faults (existing behavior)
       ========================================================= */
    if (working_settings.pcb_id == 1 && working_settings.commericial_set == 'N')
    {
        if (pb11_press)
        {
            UART_HandleTypeDef *hu[3] = { &huart1, &huart4, &huart3 };

            SYS_FAULT_TYPE *pf[3] = { &SysFault.R_fault, &SysFault.Y_fault, &SysFault.B_fault };
            SYS_FAULT_TYPE *pp[3] = { &R_prev_fault,     &Y_prev_fault,     &B_prev_fault     };

            /* ---------- 2) EXISTING NORMAL CLR ACK ---------- */
            for (int i = 0; i < 3; i++)
            {
                if (IS_CLEAR_FAULT(*pf[i]))
                {
                    *pf[i] = SYS_FAULT_NONE;
                    *pp[i] = SYS_FAULT_NONE;
                    UART_SendStatus_Domestic(hu[i], 'C');
                    UART_SendStatus_Domestic(hu[i], 'C');
                }
            }
        }
    }

    /* =========================================================
       MENU COMBO (PB10 + PB13 hold) WITH RELEASE-TO-REARM LATCH
       ========================================================= */
    static uint8_t menu_combo_armed = 1u;  // 1=can trigger, 0=must release first

    /* Re-arm only when BOTH buttons are released */
    if (!pb10_stable && !pb13_stable)
    {
        menu_combo_armed = 1u;
        combo_active = 0u;
    }

    if (menu_combo_armed)
    {
        if (pb10_stable && pb13_stable && !RreceiverInrush && !YreceiverInrush && !BreceiverInrush)
        {
            if (!combo_active)
            {
                combo_active = 1u;
                combo_start_tick = HAL_GetTick();
            }
            else if ((HAL_GetTick() - combo_start_tick) >= SETTINGS_COMBO_HOLD_MS)
            {
                combo_active = 0u;
                pb12_pending_off = 0u;

                /* Disarm until release so it won't re-open immediately */
                menu_combo_armed = 0u;

                /* Stop outputs + request runtime menu */
                runtime_setting_menu();
                enter_settings_req = 1u;
            }

            return; /* block other button actions while combo held */
        }
        else
        {
            combo_active = 0u;
        }
    }

    /* =========================================================
       NORMAL RUN STATE
       ========================================================= */
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET && pb12_press )
    {
        buzzer_muted = 1;
    }


    if (!system_is_off)
    {
        if (pb12_press)
        {
            pb12_pending_off = 1;
            pb12_pending_tick = HAL_GetTick();
        }

        if (pb12_pending_off)
        {
            if (pb10_stable)
            {
                pb12_pending_off = 0;
            }
            else if ((HAL_GetTick() - pb12_pending_tick) >= OFF_GUARD_MS)
            {
                pb12_pending_off = 0;
                recover = 0;
                System_EnterOffState_ISRsafe();
            }
        }
    }
    else
    {

        /* =====================================================
           EXISTING PB11 RECOVERY LOGIC
           ===================================================== */
        if (pb11_press && working_settings.pcb_id == 0)
        {
            recover = 1;
            uart_send_F = 0;
            uart_send_S = 0;
            oc_count = 0;
            hv_count = 0;
            lv_count = 0;
            hv_recover_count = 0;
            lv_recover_count = 0;
            system_fault_active = 0;
            prev_fault_active = 0;
            curr_fault_active = 0;
            recovery_just_completed = 0;

            volt_state = STATE_NORMAL;
            UART_SendStatus('C');
            UART_SendStatus('C');
            USER_OFF = 0;
            system_is_off = 0;
        }
        else if (working_settings.pcb_id == 1 && pb11_press)
        {
        	/* Separate helper:
        	       Domestic Auto partial restart when one/more phases are missing */
        	    if (Domestic_HandlePB11_SelectiveRestart_AllFaults())
        	    {
        	        return;
        	    }
            /* =====================================================
               DOMESTIC MANUAL SPECIAL CASE
               Startup with one/more phases missing:
               - allow PB11 to start only PRESENT boards
               - keep missing board in PHASE OUT / PHASE OUT CLR
               - do NOT affect industrial / auto paths
               ===================================================== */
            if ((working_settings.commericial_set == 'N') &&
                (working_settings.Restart_Mode == 'N') &&
                (!ALL_PHASES_PRESENT()))
            {
                /* Clear only present-phase OFF/CLEAR status if needed.
                   Missing phase fault must remain visible. */

                if (g_phase_present[0])
                {
                    if ((SysFault.R_fault == SYS_FAULT_PHASE_OUT_CLEAR) ||
                        (SysFault.R_fault == SYS_FAULT_SYSTEM_OFF))
                    {
                        SysFault.R_fault = SYS_FAULT_NONE;
                    }

                    if ((R_prev_fault == SYS_FAULT_PHASE_OUT_CLEAR) ||
                        (R_prev_fault == SYS_FAULT_SYSTEM_OFF))
                    {
                        R_prev_fault = SYS_FAULT_NONE;
                    }
                }

                if (g_phase_present[1])
                {
                    if ((SysFault.Y_fault == SYS_FAULT_PHASE_OUT_CLEAR) ||
                        (SysFault.Y_fault == SYS_FAULT_SYSTEM_OFF))
                    {
                        SysFault.Y_fault = SYS_FAULT_NONE;
                    }

                    if ((Y_prev_fault == SYS_FAULT_PHASE_OUT_CLEAR) ||
                        (Y_prev_fault == SYS_FAULT_SYSTEM_OFF))
                    {
                        Y_prev_fault = SYS_FAULT_NONE;
                    }
                }

                if (g_phase_present[2])
                {
                    if ((SysFault.B_fault == SYS_FAULT_PHASE_OUT_CLEAR) ||
                        (SysFault.B_fault == SYS_FAULT_SYSTEM_OFF))
                    {
                        SysFault.B_fault = SYS_FAULT_NONE;
                    }

                    if ((B_prev_fault == SYS_FAULT_PHASE_OUT_CLEAR) ||
                        (B_prev_fault == SYS_FAULT_SYSTEM_OFF))
                    {
                        B_prev_fault = SYS_FAULT_NONE;
                    }
                }

                /* Turn system ON and request sending C only to present boards
                   from main loop (NOT from ISR). */
                system_is_off = 0;
                clear_sent = 1;
                contactor_timer = HAL_GetTick();
                dom_manual_start_req = 1u;
                return;
            }

            /* ---------------- Existing behavior remains for
               - Domestic Manual with all 3 phases present
               - Industrial Auto
               - Industrial Manual
               - Any other existing path
               ---------------- */
            if (!ALL_PHASES_PRESENT())
            {
                return;
            }

            /* Industrial mode:
               If runtime phase-out happened and Phase Sequence check is enabled,
               do not restart from TIM6 context. Just request main-loop handling. */
            if ((working_settings.commericial_set == 'Y') &&
                (working_settings.ph_seq_cut == 'Y') &&
                ind_rt_phase_seq_required)
            {
                ind_manual_restart_req = 1u;
                return;
            }

            SysFault.R_fault = SYS_FAULT_NONE;
            SysFault.Y_fault = SYS_FAULT_NONE;
            SysFault.B_fault = SYS_FAULT_NONE;
            R_prev_fault = SYS_FAULT_NONE;
            Y_prev_fault = SYS_FAULT_NONE;
            B_prev_fault = SYS_FAULT_NONE;

            if (Is3Phase())
            {
                CSeq_Start_3Phase();
            }
            else
            {
                UART_SendStatus('C');
                UART_SendStatus('C');
            }
            system_is_off = 0;
            clear_sent = 2;
            contactor_timer = HAL_GetTick();

        }

    }
}
static inline uint8_t domestic_phase_present_now(uint8_t idx)
{
    switch (idx)
    {
        case 0: return is_phase_present(PHASE_R) ? 1u : 0u;
        case 1: return is_phase_present(PHASE_Y) ? 1u : 0u;
        case 2: return is_phase_present(PHASE_B) ? 1u : 0u;
        default: return 0u;
    }
}
void ThreePhase_Fault_and_Contactor_Task(void)
{
    static uint8_t contactor_state = 0;

    UART_HandleTypeDef *hu[3] = { &huart1, &huart4, &huart3 };   // R, Y, B
    uint16_t cont_pin[3]      = { CONT_1_Pin, CONT_2_Pin, CONT_3_Pin };

    int ready[3] = { RreceiverCont, YreceiverCont, BreceiverCont };
    SYS_FAULT_TYPE f[3] = { SysFault.R_fault, SysFault.Y_fault, SysFault.B_fault };

    uint8_t auto_mode = (working_settings.Restart_Mode == 'Y');

    /* =========================================================
       INRUSH LATCH / CLR-PENDING UPDATE
       ---------------------------------------------------------
       - If inrush is active now -> latch fault
       - If previously latched and now input cleared -> show CLR
       - CLR must wait for PB11 (no auto restart)
       ========================================================= */
    uint8_t inrush_now[3] =
    {
        (uint8_t)RreceiverInrush,
        (uint8_t)YreceiverInrush,
        (uint8_t)BreceiverInrush
    };

    for (int i = 0; i < 3; i++)
    {
        if (inrush_now[i])
        {
            tp_inrush_latched[i] = 1u;
        }
        else
        {
            tp_inrush_latched[i] = 0u;
        }
    }

    uint8_t any_inrush_active =
        (inrush_now[0] || inrush_now[1] || inrush_now[2]) ? 1u : 0u;

    uint8_t any_inrush_pending =
        (inrush_now[0] || inrush_now[1] || inrush_now[2]) ? 1u : 0u;

    /* =========================================================
       INDUSTRIAL MODE (3P-I)
       ---------------------------------------------------------
       Any inrush in any board:
       -> all pulses OFF
       -> all contactors OFF
       -> system OFF
       When inrush clears:
       -> still remain OFF
       -> PB11 only should restart
       ========================================================= */
    if (working_settings.commericial_set == 'Y')
    {
        if (any_inrush_pending)
        {
            system_is_off = 1u;

            HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, CONT_2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, CONT_3_Pin, GPIO_PIN_RESET);

            UART_SendStatus_All('F');   // all slave pulses OFF

            CSeq_Stop_3Phase();
            clear_sent = 0;
            contactor_state = 0;
            contactor_timer = HAL_GetTick();
            return;
        }

        /* ---- YOUR EXISTING INDUSTRIAL CODE CONTINUES UNCHANGED ---- */

        uint8_t any_fault =
            IS_PHASE_FAULT(SysFault.R_fault) ||
            IS_PHASE_FAULT(SysFault.Y_fault) ||
            IS_PHASE_FAULT(SysFault.B_fault);

//        uint8_t all_ready =
//            (RreceiverCont == 1) &&
//            (YreceiverCont == 1) &&
//            (BreceiverCont == 1);Bphasecont
        uint8_t all_ready =
               (Rphasecont == 1) &&
               (Yphasecont == 1) &&
               (Bphasecont == 1);

        if (any_fault)
        {
            system_is_off = 1;

            HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, CONT_2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, CONT_3_Pin, GPIO_PIN_RESET);

            UART_SendStatus_All('F');

            /* IMPORTANT: cancel any running restart pulse sequence */
            CSeq_Stop_3Phase();

            clear_sent = 0;
            contactor_state = 0;
            contactor_timer = HAL_GetTick();
            return;
        }

        if (clear_sent == 0)
        {
            if (working_settings.Restart_Mode == 'Y')
            {
                if (ALL_PHASES_PRESENT())
                {
                    /* Only for industrial runtime phase return when Phase Sequence is enabled */
                    if (!Industrial_RuntimePhaseSeq_ReadyForAutoRestart())
                    {
                        system_is_off = 1;
                        return;
                    }

                    CSeq_Start_3Phase();
                    clear_sent = 2; // sequence in progress
                    system_is_off = 0;
                    contactor_timer = HAL_GetTick();
                }
                else
                {
                    system_is_off = 1;
                    return;
                }
            }
            else
            {
                system_is_off = 1;
                return;
            }
        }


        /* After starting, wait until stagger is finished */
        if (clear_sent == 2)
        {
            if (!CSeq_IsActive())
            {
                clear_sent = 1;                 // completed
                contactor_timer = HAL_GetTick();
            }
        }

        if (system_is_off || (Rphasecont == 0 || Yphasecont ==0 ||Bphasecont == 0 ))
        {
            HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, CONT_2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, CONT_3_Pin, GPIO_PIN_RESET);
            contactor_state = 0;
            return;
        }

        if (all_ready)
        {
            if (contactor_state == 0)
            {
                if (HAL_GetTick() - contactor_timer >= 250)
                {
                    HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(GPIOA, CONT_2_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(GPIOA, CONT_3_Pin, GPIO_PIN_RESET);
                    contactor_state = 1;
                }
            }
        }

        return;
    }

    /* =========================================================
       DOMESTIC MODE (3P-D)
       ---------------------------------------------------------
       Inrush affects only the particular board:
       -> only that board pulse OFF
       -> only that board contactor OFF
       -> others continue
       When inrush clears:
       -> show CLR
       -> stay OFF until PB11
       -> no auto restart for inrush
       ========================================================= */

    uint8_t phase_stable[3] =
        {
            g_phase_present[0],
            g_phase_present[1],
            g_phase_present[2]
        };

    /* -------- If global system OFF -> force all OFF + reset Domestic FSM -------- */
    if (system_is_off)
    {
        HAL_GPIO_WritePin(GPIOA, CONT_1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, CONT_2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, CONT_3_Pin, GPIO_PIN_RESET);

        for (int i = 0; i < 3; i++)
        {
            dom_fsm[i] = DOM_ST_OFF;
            dom_fsm_tmr[i] = 0;
            dom_need_C[i] = 0;

            dom_sent_F[i] = 0;
            dom_last_block_fault[i] = SYS_FAULT_NONE;
        }

        dom_prev_system_off = 1u;
        return;
    }
    else
    {
        /* system just turned ON -> wait each board to become ready by its own pulse */
        if (dom_prev_system_off)
        {
            for (int i = 0; i < 3; i++)
            {
                dom_fsm[i] = DOM_ST_WAIT_HIGH;  // wait for receiverCont==1
                dom_fsm_tmr[i] = 0;
                dom_need_C[i] = 0;
            }
            dom_prev_system_off = 0u;
        }
    }

    /* -------- Determine per-board block + keep per-board F/C behavior -------- */
    uint8_t blocking[3] = {0,0,0};

    uint8_t inrush_block[3] =
    {
        (uint8_t)(inrush_now[0]),
        (uint8_t)(inrush_now[1]),
        (uint8_t)(inrush_now[2])
    };

    for (int i = 0; i < 3; i++)
    {
        uint8_t is_confirm_fault =
            (f[i] == SYS_FAULT_HV) ||
            (f[i] == SYS_FAULT_LV) ||
            (f[i] == SYS_FAULT_OVERLOAD) ||
            (f[i] == SYS_FAULT_HEAT) ||
            (f[i] == SYS_FAULT_HV_ACTIVE) ||
            (f[i] == SYS_FAULT_LV_ACTIVE) ||
            (f[i] == SYS_FAULT_OVERLOAD_ACTIVE) ||
            (f[i] == SYS_FAULT_PHASE_OUT);

        uint8_t is_clear = IS_CLEAR_FAULT(f[i]);

        /* Overload CLR must block even in AUTO */
        uint8_t overload_clr_block = (f[i] == SYS_FAULT_OVERLOAD_CLEAR);
        uint8_t heat_clr_block     = (f[i] == SYS_FAULT_HEAT_CLEAR);


        blocking[i] =
        is_confirm_fault ||
        (!auto_mode && is_clear) ||
        (auto_mode && (overload_clr_block || heat_clr_block)) ||
        inrush_block[i];


        if (blocking[i])
        {
#if 0
            /* First time entering block -> arm flicker protection */
            if (dom_sent_F[i] == 0)
            {
                dom_sent_F[i] = 1;
                dom_last_block_fault[i] = f[i];

                /* Flicker fix: must see ready drop LOW before accepting HIGH */
                dom_fsm[i] = DOM_ST_WAIT_LOW;
                dom_fsm_tmr[i] = 0;

                /* IMPORTANT:
                   Inrush CLR must NEVER auto restart.
                   Only PB11 should restart.
                */
                if (inrush_block[i])
                    dom_need_C[i] = 0u;
                else
                    dom_need_C[i] = auto_mode ? 1u : 0u;
            }
#endif
            if (dom_sent_F[i] == 0)
            {
                dom_sent_F[i] = 1;
                dom_last_block_fault[i] = f[i];
                /* Flicker fix: must see ready drop LOW before accepting HIGH */
                dom_fsm[i] = DOM_ST_WAIT_LOW;
                dom_fsm_tmr[i] = 0;

                /* Since inrush CLR state is removed, allow normal restart behavior */
                dom_need_C[i] = auto_mode ? 1u : 0u;
            }

            UART_SendStatus_Domestic(hu[i], 'F');
            HAL_GPIO_WritePin(GPIOA, cont_pin[i], GPIO_PIN_RESET);
        }
        else
        {
        	if (dom_sent_F[i])
			{
				/* Auto restart pulse only after phase is STABLY present */
				if (auto_mode && dom_need_C[i] && phase_stable[i])
				{
					UART_SendStatus_Domestic(hu[i], 'C'); // one pulse only
					UART_SendStatus_Domestic(hu[i], 'C'); // one pulse only
					dom_need_C[i] = 0u;
				}

				dom_sent_F[i] = 0;
				dom_last_block_fault[i] = SYS_FAULT_NONE;

				/* Always go through WAIT_LOW after unblock
				   to flush any stale ready=1 before accepting ON again */
				dom_fsm[i] = DOM_ST_WAIT_LOW;
				dom_fsm_tmr[i] = 0;
			}
        }
    }

    /* =========================
       Domestic per-board contactor FSM
       ========================= */

    for (int i = 0; i < 3; i++)
    {
        if (blocking[i])
            continue; // already forced OFF

        if (!phase_stable[i])
        {
            HAL_GPIO_WritePin(GPIOA, cont_pin[i], GPIO_PIN_RESET);
            dom_fsm[i] = DOM_ST_WAIT_LOW;
            dom_fsm_tmr[i] = 0;
            continue;
        }

        switch (dom_fsm[i])
        {
            default:
            case DOM_ST_OFF:
                HAL_GPIO_WritePin(GPIOA, cont_pin[i], GPIO_PIN_RESET);
                dom_fsm[i] = DOM_ST_WAIT_HIGH;
                break;

            case DOM_ST_WAIT_LOW:
                /* Wait slave to be truly OFF (ready==0) */
                HAL_GPIO_WritePin(GPIOA, cont_pin[i], GPIO_PIN_RESET);
                if (ready[i] == 0)
                {
                    dom_fsm[i] = DOM_ST_WAIT_HIGH;
                }
                break;

            case DOM_ST_WAIT_HIGH:
                        HAL_GPIO_WritePin(GPIOA, cont_pin[i], GPIO_PIN_RESET);

                        /* Only accept READY if phase is already stably present */
                        if (phase_stable[i] && (ready[i] == 1))
                        {
                            dom_fsm_tmr[i] = HAL_GetTick();
                            dom_fsm[i] = DOM_ST_DELAY_ON;
                        }
                        break;

            case DOM_ST_DELAY_ON:
                        /* If phase becomes unstable OR slave drops ready, restart safely */
                        if ((!phase_stable[i]) || (ready[i] == 0))
                        {
                            HAL_GPIO_WritePin(GPIOA, cont_pin[i], GPIO_PIN_RESET);
                            dom_fsm[i] = DOM_ST_WAIT_LOW;
                            dom_fsm_tmr[i] = 0;
                            break;
                        }

                        if (HAL_GetTick() - dom_fsm_tmr[i] >= 250u)
                        {
                            HAL_GPIO_WritePin(GPIOA, cont_pin[i], GPIO_PIN_SET);
                            dom_fsm[i] = DOM_ST_ON;
                        }
                        else
                        {
                            HAL_GPIO_WritePin(GPIOA, cont_pin[i], GPIO_PIN_RESET);
                        }
                        break;


            case DOM_ST_ON:
                        /* Keep contactor ON only while phase is stable and slave is ready */
                        if ((!phase_stable[i]) || (ready[i] == 0))
                        {
                            HAL_GPIO_WritePin(GPIOA, cont_pin[i], GPIO_PIN_RESET);
                            dom_fsm[i] = DOM_ST_WAIT_LOW;
                            dom_fsm_tmr[i] = 0;
                        }
                        else
                        {
                            HAL_GPIO_WritePin(GPIOA, cont_pin[i], GPIO_PIN_SET);
                        }
                        break;
        }
    }
}

void Buzzer_Task(void)
{
    uint8_t buzzer_on = 0;

    /* If user muted the buzzer */
    if (buzzer_muted)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        return;
    }
    /* ================= SINGLE PHASE ================= */
    if (working_settings.pcb_id == 0)
    {
        if (( IS_FAULT_STATE(volt_state) && volt_state != STATE_HV_RECOVER && volt_state != STATE_LV_RECOVER)|| (IS_CLEAR_STATE(volt_state)  &&  volt_state == STATE_CLEAR_OC_FAULT)||RreceiverInrush )
        {
            buzzer_on = 1;
        }
    }
    else
    {

    /* Check R phase */
    if ((IS_PHASE_FAULT(SysFault.R_fault) &&
        SysFault.R_fault != SYS_FAULT_LV_ACTIVE &&
        SysFault.R_fault != SYS_FAULT_OVERLOAD_ACTIVE &&
        SysFault.R_fault != SYS_FAULT_HV_ACTIVE && SysFault.R_fault != SYS_FAULT_HEAT_CLEAR  && SysFault.R_fault != SYS_FAULT_LV_CLEAR  && SysFault.R_fault != SYS_FAULT_HV_CLEAR  && SysFault.R_fault != SYS_FAULT_PHASE_OUT_CLEAR) || RreceiverInrush)
    {
        buzzer_on = 1;
    }

    /* Check Y phase */
    if ((IS_PHASE_FAULT(SysFault.Y_fault) &&
        SysFault.Y_fault != SYS_FAULT_LV_ACTIVE &&
        SysFault.Y_fault != SYS_FAULT_OVERLOAD_ACTIVE &&
        SysFault.Y_fault != SYS_FAULT_HV_ACTIVE  && SysFault.Y_fault != SYS_FAULT_HEAT_CLEAR  && SysFault.Y_fault != SYS_FAULT_LV_CLEAR  && SysFault.Y_fault != SYS_FAULT_HV_CLEAR  && SysFault.Y_fault != SYS_FAULT_PHASE_OUT_CLEAR) || YreceiverInrush ||  (SysFault.Y_fault == SYS_FAULT_OVERLOAD_CLEAR))
    {
        buzzer_on = 1;
    }

    /* Check B phase */
    if ((IS_PHASE_FAULT(SysFault.B_fault) &&
        SysFault.B_fault != SYS_FAULT_LV_ACTIVE &&
        SysFault.B_fault != SYS_FAULT_OVERLOAD_ACTIVE &&
        SysFault.B_fault != SYS_FAULT_HV_ACTIVE  && SysFault.B_fault != SYS_FAULT_HEAT_CLEAR  && SysFault.B_fault != SYS_FAULT_LV_CLEAR  && SysFault.B_fault != SYS_FAULT_HV_CLEAR  && SysFault.B_fault != SYS_FAULT_PHASE_OUT_CLEAR) || BreceiverInrush||   (SysFault.B_fault == SYS_FAULT_OVERLOAD_CLEAR))
    {
        buzzer_on = 1;
    }
    }
    if (buzzer_on)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}


void HandlePhaseFault(PhaseContext *ctx)
{
    SYS_FAULT_TYPE prev = *(ctx->prev_fault);
    SYS_FAULT_TYPE newFault = prev;

    /* =========================================================
       MANUAL MODE CLEAR LOCK
       If fault is cleared, wait until user reset
       ========================================================= */
    /* =========================================================
       MANUAL MODE CLEAR STATE MONITOR
       Stay in CLEAR only if condition is healthy
       If fault reappears → go back to fault
       ========================================================= */

    /* =========================================================
       MANUAL MODE CLEAR STATE MONITOR
       If fault condition returns → go directly to FAULT
       ========================================================= */
    if (working_settings.Restart_Mode == 'N')
    {
        /* HV CLEAR */
        if (prev == SYS_FAULT_HV_CLEAR)
        {
            if (ctx->phaseInput > working_settings.hv_cut)
            {
                *(ctx->hv_timer) = FAULT_DELAY_HV;   // skip APPEAR
                newFault = SYS_FAULT_HV;
            }
            else
            {
                newFault = SYS_FAULT_HV_CLEAR;
            }

            *(ctx->fault_out) = newFault;
            *(ctx->prev_fault) = newFault;
            return;
        }

        /* LV CLEAR */
        if (prev == SYS_FAULT_LV_CLEAR)
        {
            if (ctx->phaseInput < working_settings.lv_cut)
            {
                *(ctx->lv_timer) = FAULT_DELAY_LV;   // skip APPEAR
                newFault = SYS_FAULT_LV;
            }
            else
            {
                newFault = SYS_FAULT_LV_CLEAR;
            }

            *(ctx->fault_out) = newFault;
            *(ctx->prev_fault) = newFault;
            return;
        }

        /* OVERLOAD CLEAR */
        if (prev == SYS_FAULT_OVERLOAD_CLEAR)
        {
            if (ctx->currLoad > working_settings.adjust_load)
            {
                *(ctx->overload_timer) = FAULT_DELAY_OVERLOAD;
                newFault = SYS_FAULT_OVERLOAD;
            }
            else
            {
                newFault = SYS_FAULT_OVERLOAD_CLEAR;
            }

            *(ctx->fault_out) = newFault;
            *(ctx->prev_fault) = newFault;
            return;
        }

        /* HEAT CLEAR */
        if (prev == SYS_FAULT_HEAT_CLEAR)
        {
            if (ctx->phaseHeat)
            {
                *(ctx->heat_timer) = FAULT_DELAY_HEAT;
                newFault = SYS_FAULT_HEAT;
            }
            else
            {
                newFault = SYS_FAULT_HEAT_CLEAR;
            }

            *(ctx->fault_out) = newFault;
            *(ctx->prev_fault) = newFault;
            return;
        }

        /* PHASE OUT CLEAR */
        if (prev == SYS_FAULT_PHASE_OUT_CLEAR)
        {
            if (!ctx->phasePresent)
            {
                newFault = SYS_FAULT_PHASE_OUT;
            }
            else
            {
                newFault = SYS_FAULT_PHASE_OUT_CLEAR;
            }

            *(ctx->fault_out) = newFault;
            *(ctx->prev_fault) = newFault;
            return;
        }
    }
    else if(working_settings.Restart_Mode == 'Y')
    {
     if ((prev == SYS_FAULT_OVERLOAD_CLEAR) ||
         (prev == SYS_FAULT_HEAT_CLEAR))
     {
     *(ctx->fault_out) = prev;
     *(ctx->prev_fault) = prev;
     return;
     }
    }

    /* =========================================================
       PRIORITY 1 : PHASE OUT
       ========================================================= */

    if (!ctx->phasePresent)
    {
        *(ctx->lv_timer) = 0;
        *(ctx->hv_timer) = 0;
        *(ctx->overload_timer) = 0;
        *(ctx->heat_timer) = 0;
        *(ctx->active_timer) = 0;

        newFault = SYS_FAULT_PHASE_OUT;

        *(ctx->fault_out) = newFault;
        *(ctx->prev_fault) = newFault;
        return;
    }

    /* =========================================================
       PHASE RETURN → PHASE_OUT_CLEAR (MANUAL MODE)
       ========================================================= */

    if (working_settings.Restart_Mode == 'N')
    {
        if (prev == SYS_FAULT_PHASE_OUT)
        {
            newFault = SYS_FAULT_PHASE_OUT_CLEAR;

            *(ctx->fault_out) = newFault;
            *(ctx->prev_fault) = newFault;
            return;
        }
    }

    /* =========================================================
       LOW VOLTAGE
       ========================================================= */

    if (ctx->phaseInput < working_settings.lv_cut)
    {
        *(ctx->active_timer) = 0;
        *(ctx->hv_timer) = 0;
        *(ctx->overload_timer) = 0;
        *(ctx->heat_timer) = 0;

        if (*(ctx->lv_timer) < FAULT_DELAY_LV)
        {
            (*(ctx->lv_timer))++;
            newFault = SYS_FAULT_LV_APPEAR;
        }
        else
        {
            newFault = SYS_FAULT_LV;
        }
    }

    /* =========================================================
       HIGH VOLTAGE
       ========================================================= */

    else if (ctx->phaseInput > working_settings.hv_cut)
    {
        *(ctx->active_timer) = 0;
        *(ctx->lv_timer) = 0;
        *(ctx->overload_timer) = 0;
        *(ctx->heat_timer) = 0;

        if (*(ctx->hv_timer) < FAULT_DELAY_HV)
        {
            (*(ctx->hv_timer))++;
            newFault = SYS_FAULT_HV_APPEAR;
        }
        else
        {
            newFault = SYS_FAULT_HV;
        }
    }

    /* =========================================================
       OVERLOAD
       ========================================================= */

    else if (ctx->currLoad > working_settings.adjust_load)
    {
        *(ctx->active_timer) = 0;
        *(ctx->lv_timer) = 0;
        *(ctx->hv_timer) = 0;
        *(ctx->heat_timer) = 0;

        if (*(ctx->overload_timer) < FAULT_DELAY_OVERLOAD)
        {
            (*(ctx->overload_timer))++;
            newFault = SYS_FAULT_OVERLOAD_APPEAR;
        }
        else
        {
            newFault = SYS_FAULT_OVERLOAD;
        }
    }


    /* =========================================================
       HEAT
       ========================================================= */

    else if (ctx->phaseHeat &&  prev != SYS_FAULT_OVERLOAD)
    {
        *(ctx->active_timer) = 0;
        *(ctx->lv_timer) = 0;
        *(ctx->hv_timer) = 0;
        *(ctx->overload_timer) = 0;

        if (*(ctx->heat_timer) < FAULT_DELAY_HEAT)
        {
            (*(ctx->heat_timer))++;
            newFault = SYS_FAULT_HEAT_APPEAR;
        }
        else
        {
            newFault = SYS_FAULT_HEAT;
        }
    }

    /* =========================================================
           NO FAULT CONDITION
           ========================================================= */

        else
        {
            *(ctx->lv_timer) = 0;
            *(ctx->hv_timer) = 0;
            *(ctx->overload_timer) = 0;
            *(ctx->heat_timer) = 0;

            /* ================= AUTO MODE ================= */
            if (working_settings.Restart_Mode == 'Y')
            {
                /* --- HV: stay in HV until Vin <= hv_cut - V_HYST --- */
                if (prev == SYS_FAULT_HV || prev == SYS_FAULT_HV_ACTIVE)
                {
                    if (ctx->phaseInput <= (int)(working_settings.hv_cut - V_HYST))
                    {
                        if (*(ctx->active_timer) < FAULT_ACTIVE_DISPLAY_TIME)
                        {
                            (*(ctx->active_timer))++;
                            newFault = SYS_FAULT_HV_ACTIVE;
                        }
                        else
                        {
                            *(ctx->active_timer) = 0;
                            newFault = SYS_FAULT_NONE;
                        }
                    }
                    else
                    {
                        /* still too close to threshold: keep it faulted */
                        *(ctx->active_timer) = 0;
                        newFault = SYS_FAULT_HV;
                    }
                }
                /* --- LV: stay in LV until Vin >= lv_cut + V_HYST --- */
                else if (prev == SYS_FAULT_LV || prev == SYS_FAULT_LV_ACTIVE)
                {
                    if (ctx->phaseInput >= (int)(working_settings.lv_cut + V_HYST))
                    {
                        if (*(ctx->active_timer) < FAULT_ACTIVE_DISPLAY_TIME)
                        {
                            (*(ctx->active_timer))++;
                            newFault = SYS_FAULT_LV_ACTIVE;
                        }
                        else
                        {
                            *(ctx->active_timer) = 0;
                            newFault = SYS_FAULT_NONE;
                        }
                    }
                    else
                    {
                        /* still too close to threshold: keep it faulted */
                        *(ctx->active_timer) = 0;
                        newFault = SYS_FAULT_LV;
                    }
                }
                /* --- OVERLOAD: mirrors HV/LV above, per request - ACTIVE
                   for FAULT_ACTIVE_DISPLAY_TIME ticks, then auto-clears to
                   NONE with no PB11 acknowledgment needed, same as HV/LV.
                   NOTE: unlike HV/LV, there's no separate hysteresis re-
                   check here (no equivalent "adjust_load - margin" constant
                   existed) - reaching this branch already means currLoad
                   is back under adjust_load, so it goes straight into the
                   ACTIVE countdown. If it should require staying below a
                   margin first (like HV/LV do against V_HYST), say so and
                   I'll add an equivalent load-hysteresis constant. */
                else if (prev == SYS_FAULT_OVERLOAD || prev == SYS_FAULT_OVERLOAD_ACTIVE)
                {
                    if (*(ctx->active_timer) < FAULT_ACTIVE_DISPLAY_TIME)
                    {
                        (*(ctx->active_timer))++;
                        newFault = SYS_FAULT_OVERLOAD_ACTIVE;
                    }
                    else
                    {
                        *(ctx->active_timer) = 0;
                        newFault = SYS_FAULT_NONE;
                    }
                }
                else if (prev == SYS_FAULT_HEAT)
                   {
                   	newFault = SYS_FAULT_HEAT_CLEAR;
                   }
                else
                {
                    newFault = SYS_FAULT_NONE;
                }
            }

            /* ================= MANUAL MODE ================= */
            else
            {
                switch (prev)
                {
                case SYS_FAULT_HV:
                    /* Only allow HV -> HV_CLEAR when Vin <= hv_cut - V_HYST */
                    if (ctx->phaseInput <= (int)(working_settings.hv_cut - V_HYST))
                        newFault = SYS_FAULT_HV_CLEAR;
                    else
                        newFault = SYS_FAULT_HV; /* hold fault */
                    break;

                case SYS_FAULT_LV:
                    /* Only allow LV -> LV_CLEAR when Vin >= lv_cut + V_HYST */
                    if (ctx->phaseInput >= (int)(working_settings.lv_cut + V_HYST))
                        newFault = SYS_FAULT_LV_CLEAR;
                    else
                        newFault = SYS_FAULT_LV; /* hold fault */
                    break;

                case SYS_FAULT_OVERLOAD:
                    newFault = SYS_FAULT_OVERLOAD_CLEAR;
                    break;

                case SYS_FAULT_HEAT:
                    newFault = SYS_FAULT_HEAT_CLEAR;
                    break;

                default:
                    newFault = SYS_FAULT_NONE;
                    break;
                }
                *(ctx->active_timer) = 0;
            }
        }



    *(ctx->fault_out) = newFault;
    *(ctx->prev_fault) = newFault;
}
volatile uint8_t Srunningtimeflag = 0;
volatile uint8_t Rrunningtimeflag = 0;
volatile uint8_t Yrunningtimeflag=0;
volatile uint8_t Brunningtimeflag=0;

#if (MASTER_MONITOR_MODE == 0)       /* the old application's 1 ms tick: no timer is started in monitor/app mode, and the weak HAL default does nothing */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* If you need dividers, keep them static here */
    /* ================= TIM6 (common) ================= */
    if (htim->Instance == TIM6)
    {
        Button_Task();

        if (afterstartingmenu)
        {
            if (ui_mode == UI_RUN)
                PowerButtons_Task_TIM6();
            if(working_settings.pcb_id == 0)
                   Buzzer_Task();   // ← call here

        }
        else
        {
            StartupButton_Task_TIM6();
        }

        return; // ✅ important: avoid falling into other branches
    }

    /* ================= Mode selection ================= */
    /* ================= Mode selection ================= */
    if (GetMode() == MODE_SINGLE)
    {
        /* ================= SINGLE PHASE ================= */
        if (htim->Instance == TIM7)
        {
            if (!voltage_updated)
                return;

            voltage_updated = 0;

            if (system_is_off && USER_OFF)
            {
                volt_state = STATE_OFF;

                oc_count = 0;
                hv_count = 0;
                lv_count = 0;
                heat_count = 0;
                hv_recover_count = 0;
                lv_recover_count = 0;
                oc_recover_count = 0;

                return;
            }

            recovery_just_completed = 0;

            /* ================= LOW VOLTAGE ================= */
            if (
                (phaseRinput < working_settings.lv_cut) ||
                ((volt_state == STATE_LV_FAULT || volt_state == STATE_LV_RECOVER) &&
                 (phaseRinput < (working_settings.lv_cut + V_HYST)))
               )
            {
                lv_count++;

                if (!IS_FAULT_STATE(volt_state))
                    volt_state = STATE_LV_COUNT;

                if (lv_count >= FAULT_DELAY_SEC)
                {
                    lv_count = FAULT_DELAY_SEC;
                    volt_state = STATE_LV_FAULT;
                }
            }
            else
            {
                lv_count = 0;

                if (volt_state == STATE_LV_COUNT)
                    volt_state = STATE_NORMAL;

                else if (volt_state == STATE_LV_FAULT)
                {
                    /* Require the same RECOVER_DELAY_SEC-confirmed hold in
                     * BOTH restart modes before treating LV as cleared -
                     * previously Manual ('N') cleared on a single
                     * instantaneous sample above lv_cut+V_HYST, with no
                     * time confirmation at all, unlike Auto ('Y'). That let
                     * a brief noise/ripple tick above the threshold trigger
                     * an immediate reclose attempt, which then re-opened
                     * moments later when input dipped back down - extra
                     * contactor actuations for no real recovery. Only what
                     * happens AFTER confirmed recovery still differs by
                     * mode (see the STATE_LV_RECOVER case below). */
                    lv_recover_count = 0;
                    volt_state = STATE_LV_RECOVER;
                }
            }

            /* ================= HIGH VOLTAGE ================= */
            if (
                (phaseRinput > working_settings.hv_cut) ||
                ((volt_state == STATE_HV_FAULT || volt_state == STATE_HV_RECOVER) &&
                 (phaseRinput > (working_settings.hv_cut - V_HYST)))
               )
            {
                hv_count++;

                if (!IS_FAULT_STATE(volt_state))
                    volt_state = STATE_HV_COUNT;

                if (hv_count >= FAULT_DELAY_SEC)
                {
                    hv_count = FAULT_DELAY_SEC;
                    volt_state = STATE_HV_FAULT;
                }
            }
            else
            {
                hv_count = 0;

                if (volt_state == STATE_HV_COUNT)
                    volt_state = STATE_NORMAL;

                else if (volt_state == STATE_HV_FAULT)
                {
                    if (working_settings.Restart_Mode == 'Y')
                    {
                        hv_recover_count = 0;
                        volt_state = STATE_HV_RECOVER;
                    }
                    else
                    {
                        volt_state = STATE_CLEAR_HV_FAULT;
                        uart_send_S = 1;
                    }
                }
            }

            /* ================= OVERCURRENT ================= */
            if (curr_R > working_settings.adjust_load)
            {
                oc_count++;
                oc_recover_count = 0;   /* still faulting - reset any recovery progress */

                if (!IS_FAULT_STATE(volt_state))
                    volt_state = STATE_OC_COUNT;

                if (oc_count >= OC_DELAY_SEC)
                {
                    oc_count = OC_DELAY_SEC;
                    volt_state = STATE_OC_FAULT;
                }
            }
            else
            {
                oc_count = 0;

                if (volt_state == STATE_OC_COUNT)
                    volt_state = STATE_NORMAL;

                else if (volt_state == STATE_OC_FAULT)
                {
                    /* Require curr_R to stay below the threshold for
                     * RECOVER_DELAY_SEC (the same hold already used for
                     * LV/HV recovery) before actually clearing, instead of
                     * clearing on a single below-threshold sample - avoids
                     * an immediate re-trip when curr_R is hovering right at
                     * the threshold. */
                    oc_recover_count++;

                    if (oc_recover_count >= RECOVER_DELAY_SEC)
                    {
                        oc_recover_count = 0;
                        volt_state = STATE_CLEAR_OC_FAULT;
                    }
                }
                else
                {
                    oc_recover_count = 0;
                }
            }

            /* ================= HEAT ================= */
            if (phaseRheat == 1)
            {
                heat_count++;

                if (!IS_FAULT_STATE(volt_state))
                    volt_state = STATE_HEAT_COUNT;

                if (heat_count >= HEAT_DELAY_SEC)
                {
                    heat_count = HEAT_DELAY_SEC;
                    volt_state = STATE_HEAT_FAULT;
                }
            }
            else
            {
                heat_count = 0;

                if (volt_state == STATE_HEAT_COUNT)
                    volt_state = STATE_NORMAL;

                if (volt_state == STATE_HEAT_FAULT)
                {
                    if (working_settings.Restart_Mode == 'Y')
                    {
                        volt_state = STATE_CLEAR_HEAT_FAULT;
//vasanth                recovery_just_completed = 1;
                        uart_send_S = 1;
                    }
                    else
                    {
                        volt_state = STATE_CLEAR_HEAT_FAULT;
                        uart_send_S = 1;
                    }
                }
            }

            /* ================= RECOVERY ================= */
            switch (volt_state)
            {
                case STATE_HV_RECOVER:
                    if (phaseRinput <= (working_settings.hv_cut - V_HYST))
                    {
                        hv_recover_count++;

                        if (hv_recover_count >= RECOVER_DELAY_SEC)
                        {
                            hv_recover_count = 0;
                            volt_state = STATE_NORMAL;
                            recovery_just_completed = 1;
                        }
                    }
                    else
                    {
                        hv_recover_count = 0;
                    }
                    break;

                case STATE_LV_RECOVER:
                    if (phaseRinput >= (working_settings.lv_cut + V_HYST))
                    {
                        lv_recover_count++;

                        if (lv_recover_count >= RECOVER_DELAY_SEC)
                        {
                            lv_recover_count = 0;

                            if (working_settings.Restart_Mode == 'Y')
                            {
                                volt_state = STATE_NORMAL;
                                recovery_just_completed = 1;
                            }
                            else
                            {
                                volt_state = STATE_CLEAR_LV_FAULT;
                                uart_send_S = 1;
                            }
                        }
                    }
                    else
                    {
                        lv_recover_count = 0;
                    }
                    break;

                default:
                    break;
            }

            /* ================= UART ================= */
            {
                uint8_t curr_fault_active_local = IS_FAULT_STATE(volt_state);

                if ((curr_fault_active_local == 1) && (prev_fault_active == 0))
                {
                    uart_send_F = 1;
                    uart_send_S = 0;
                    system_fault_active = 1;
                    contactor_enable_request = 0;
                }
                else if (recovery_just_completed)
                {
                    uart_send_S = 1;
                    uart_send_F = 0;
                    contactor_enable_request = 1;
                    system_fault_active = 0;
                }

                prev_fault_active = curr_fault_active_local;
            }
        }

        return;
    }

    else
    {
        /* ================= THREE PHASE ================= */
        if (htim->Instance == TIM14)   /* R */
        {
        	if(Rrunningtimeflag)
        	{
                /* Reset timers only */
                R_lv_timer = 0;
                R_hv_timer = 0;
                R_overload_timer = 0;
                R_heat_timer = 0;
                R_active_timer = 0;
                return;
        	}
            PhaseContext Rctx =
            {
                .phaseInput = Rphaseinput,
                .phaseHeat  = Rphaseheat,
                .currLoad   = curr_R,
                .phasePresent = g_phase_present[0],

                .lv_timer = &R_lv_timer,
                .hv_timer = &R_hv_timer,
                .overload_timer = &R_overload_timer,
                .heat_timer = &R_heat_timer,
                .active_timer = &R_active_timer,
                .prev_fault = &R_prev_fault,
                .fault_out  = &SysFault.R_fault
            };

            HandlePhaseFault(&Rctx);
            if(working_settings.commericial_set == 'Y')
            {
              Buzzer_Task();
            }

            return;
        }
        else if (htim->Instance == TIM16) /* Y */
        {
        	if(Yrunningtimeflag)
        	{
                Y_lv_timer = 0;
                Y_hv_timer = 0;
                Y_overload_timer = 0;
                Y_heat_timer = 0;
                Y_active_timer = 0;
        		return;
        	}
            PhaseContext Yctx =
            {
                .phaseInput = Yphaseinput,
                .phaseHeat  = Yphaseheat,
                .currLoad   = curr_Y,
                .phasePresent = g_phase_present[1],

                .lv_timer = &Y_lv_timer,
                .hv_timer = &Y_hv_timer,
                .overload_timer = &Y_overload_timer,
                .heat_timer = &Y_heat_timer,
                .active_timer = &Y_active_timer,
                .prev_fault = &Y_prev_fault,
                .fault_out  = &SysFault.Y_fault
            };

            HandlePhaseFault(&Yctx);
            if(working_settings.commericial_set == 'Y')
            {
              Buzzer_Task();
            }

            return;
        }
        else if (htim->Instance == TIM17) /* B */
        {
        	if(Brunningtimeflag)
        	{
                B_lv_timer = 0;
                B_hv_timer = 0;
                B_overload_timer = 0;
                B_heat_timer = 0;
                B_active_timer = 0;
                return;
        	}
            PhaseContext Bctx =
            {
                .phaseInput = Bphaseinput,
                .phaseHeat  = Bphaseheat,
                .currLoad   = curr_B,
                .phasePresent = g_phase_present[2],

                .lv_timer = &B_lv_timer,
                .hv_timer = &B_hv_timer,
                .overload_timer = &B_overload_timer,
                .heat_timer = &B_heat_timer,
                .active_timer = &B_active_timer,
                .prev_fault = &B_prev_fault,
                .fault_out  = &SysFault.B_fault
            };

            HandlePhaseFault(&Bctx);
            if(working_settings.commericial_set == 'Y')
            {
              Buzzer_Task();
            }

            return;
        }

        /* If you had "master report once", do it in ONE timer only
           (ex: in TIM14 after handling Rctx OR use a separate slow tick) */
    }
}
#endif /* MASTER_MONITOR_MODE == 0 */
#if 0
static void Domestic_SendC_To_PresentBoards(void)
{
    uint8_t useR = domestic_phase_present_now(0);
    uint8_t useY = domestic_phase_present_now(1);
    uint8_t useB = domestic_phase_present_now(2);

    /* Update phase presence immediately */
    g_phase_raw_present[0] = useR;
    g_phase_raw_present[1] = useY;
    g_phase_raw_present[2] = useB;

    g_phase_present[0] = useR;
    g_phase_present[1] = useY;
    g_phase_present[2] = useB;

    /* Send C only to available domestic boards */
    if (useR)
    {
        while (huart1.gState != HAL_UART_STATE_READY) { HAL_Delay(1); }
        UART_SendStatus_Domestic(&huart1, 'C');
        UART_SendStatus_Domestic(&huart1, 'C');
        HAL_Delay(100);
    }

    if (useY)
    {
        while (huart4.gState != HAL_UART_STATE_READY) { HAL_Delay(1); }
        UART_SendStatus_Domestic(&huart4, 'C');
        UART_SendStatus_Domestic(&huart1, 'C');
        HAL_Delay(100);
    }

    if (useB)
    {
        while (huart3.gState != HAL_UART_STATE_READY) { HAL_Delay(1); }
        UART_SendStatus_Domestic(&huart3, 'C');
        UART_SendStatus_Domestic(&huart1, 'C');
        HAL_Delay(100);
    }
}
#endif

static void Domestic_SendC_To_PresentBoards(void)
{
    uint8_t useR = domestic_phase_present_now(0);
    uint8_t useY = domestic_phase_present_now(1);
    uint8_t useB = domestic_phase_present_now(2);

    /* IMPORTANT:
       Do NOT overwrite g_phase_present[] / g_phase_raw_present[] here.
       Runtime phase stability must be controlled only by PhaseOut_RuntimeTask().
       This function should only send C to boards that are physically present now.
    */

    /* Send C only to available domestic boards */
    if (useR)
    {
        while (huart1.gState != HAL_UART_STATE_READY) { HAL_Delay(1); }
        UART_SendStatus_Domestic(&huart1, 'C');
        UART_SendStatus_Domestic(&huart1, 'C');
        HAL_Delay(100);
    }

    if (useY)
    {
        while (huart4.gState != HAL_UART_STATE_READY) { HAL_Delay(1); }
        UART_SendStatus_Domestic(&huart4, 'C');
        UART_SendStatus_Domestic(&huart4, 'C');
        HAL_Delay(100);
    }

    if (useB)
    {
        while (huart3.gState != HAL_UART_STATE_READY) { HAL_Delay(1); }
        UART_SendStatus_Domestic(&huart3, 'C');
        UART_SendStatus_Domestic(&huart4, 'C');
        HAL_Delay(100);
    }
}
static void ApplyRestartModeRuntime(uint8_t newMode)
{
    if (newMode == 'Y')   // AUTO
    {
        uart_send_F = 0;
        uart_send_S = 0;

        system_is_off = 0;
        USER_OFF = 0;
        volt_state = STATE_NORMAL;

        oc_count = 0;
        hv_count = 0;
        lv_count = 0;
        heat_count = 0;
        hv_recover_count = 0;
        lv_recover_count = 0;

        /* ✅ Resume slaves depending on platform */
        if (working_settings.pcb_id == 0)
        {
            UART_SendStatus('C');        // 1P
            UART_SendStatus('C');
        }
        else
        {
            /* =========================
               INDUSTRIAL MODE
               keep existing logic
               ========================= */
            if (working_settings.commericial_set == 'Y')
            {
                CSeq_Start_3Phase();
                clear_sent = 2;
            }
            /* =========================
               DOMESTIC MODE
               runtime/startup restart only for present boards
               ========================= */
            else
            {
                clear_sent = 1;   // domestic doesn't use global 3P C sequence here
                Domestic_SendC_To_PresentBoards();
            }
        }
    }
    else                  // MANUAL ('N')
    {
        /* ✅ Force OFF and require PB11 */
        System_EnterOffState_ISRsafe();  // 1P or 3P
    }
}


static void Industrial_PhaseGate_WithMenuReentry(void)
{
    while ((working_settings.pcb_id == 1) &&
           (working_settings.commericial_set == 'Y'))
    {
        phchk_result_t rc;

        if (working_settings.ph_seq_cut == 'Y')
        {
            rc = Phase_Check_Before_Start();
        }
        else
        {
            rc = Phase_Presence_Check_Before_Start();
        }

        if (rc == PHCHK_OK)
        {
            break;
        }

        /* User asked to reopen settings from phase-check screen */
        system_is_off = 1u;
        USER_OFF = 1u;
        ui_mode = UI_MENU;
        g_force_settings_menu = 1u;
        welcome_message();

        LCD_Clear();
        LCD_SetCursor(1,0);
        LCD_Print(" Settings Done ");
        HAL_Delay(1000);

        ui_mode = UI_RUN;

        /* If user disabled phase sequence, clear runtime lock immediately */
        if (working_settings.ph_seq_cut != 'Y')
        {
            ind_rt_phase_seq_required = 0u;
            ind_rt_phase_seq_display_lock = 0u;
            ind_rt_phase_seq_fail_result = 0u;
        }

        /* loop again using updated settings */
    }
}

static uint8_t PhaseCheck_MenuRequest(void)
{
    static uint8_t combo_active_local = 0u;
    static uint8_t combo_armed_local  = 1u;
    static uint32_t combo_start_local = 0u;

    uint8_t pb10 = raw_pressed(GPIOB, GPIO_PIN_10);
    uint8_t pb13 = raw_pressed(GPIOB, GPIO_PIN_13);

    /* Re-arm only after BOTH released */
    if (!pb10 && !pb13)
    {
        combo_armed_local  = 1u;
        combo_active_local = 0u;
    }

    /* If TIM6 already requested settings, consume it once
       and force release before next trigger */
    if (enter_settings_req)
    {
        enter_settings_req = 0u;
        combo_active_local = 0u;
        combo_armed_local  = 0u;
        return 1u;
    }

    if (!combo_armed_local)
    {
        return 0u;
    }

    if (pb10 && pb13)
    {
        if (!combo_active_local)
        {
            combo_active_local = 1u;
            combo_start_local  = HAL_GetTick();
        }
        else if ((HAL_GetTick() - combo_start_local) >= SETTINGS_COMBO_HOLD_MS)
        {
            combo_active_local = 0u;
            combo_armed_local  = 0u;   /* IMPORTANT */
            return 1u;
        }
    }
    else
    {
        combo_active_local = 0u;
    }

    return 0u;
}
static phchk_result_t Phase_Check_Before_Start(void)
{
    uint8_t phase_out;

    while (1)
    {
        /* Allow reopening settings from this screen */
        if (PhaseCheck_MenuRequest())
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            LCD_Clear();
            return PHCHK_OPEN_SETTINGS;
        }

        /* ---------------- PHASE PRESENCE CHECK ---------------- */
        phase_out = detect_phase_out();
        if (phase_out != 0u)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            LCD_Clear();
            LCD_SetCursor(1,0);
            LCD_Print(" PHASE FAULT ");
            LCD_SetCursor(2,0);
            LCD_Print(phase_out_to_string(phase_out));
            HAL_Delay(100);
            continue;
        }

        /* ---------------- PHASE SEQUENCE CHECK ---------------- */
        phase_seq_result = detect_phase_sequence();

        if (phase_seq_result == 1u)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            LCD_Clear();
            LCD_SetCursor(1,0);
            LCD_Print(" PHASE OK ");
            LCD_SetCursor(2,0);
            LCD_Print(" SEQ: R-Y-B ");
            HAL_Delay(1000);
            return PHCHK_OK;
        }
        else if (phase_seq_result == 3u)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
//            LCD_Clear();
            LCD_SetCursor(1,0);
            LCD_Print(" PHASE FAULT ");
            LCD_SetCursor(2,0);
            LCD_Print(" SAME PHASE ");
            HAL_Delay(100);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
//            LCD_Clear();
            LCD_SetCursor(1,0);
            LCD_Print(" REVERSE SEQUENCE ");
            HAL_Delay(100);
        }
    }
}

static phchk_result_t Phase_Presence_Check_Before_Start(void)
{
    uint8_t phase_out;

    while (1)
    {
        /* Allow reopening settings from this screen too */
        if (PhaseCheck_MenuRequest())
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            LCD_Clear();
            return PHCHK_OPEN_SETTINGS;
        }

        phase_out = detect_phase_out();

        if (phase_out != 0u)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
            LCD_Clear();
            LCD_SetCursor(1,0);
            LCD_Print(" PHASE FAULT ");
            LCD_SetCursor(2,0);
            LCD_Print(phase_out_to_string(phase_out));
            HAL_Delay(100);
            continue;
        }

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        LCD_Clear();
        LCD_SetCursor(1,0);
        LCD_Print(" PHASE OK ");
        HAL_Delay(2000);
        return PHCHK_OK;
    }
}

void setting_menu_handshake()
{
	LCD_SetCursor(1, 0);
	LCD_Print(" Settings Done ");
	HAL_Delay(10);

	while(!Recceived) // IF RECEVED P FROM SLAVE, BREAK WHILE LOOP
	{
		HAL_UART_Transmit_DMA(&huart1,(uint8_t*)"[N]\r\n",5);
		HAL_Delay(100);
	}
	while(!uart1_ready) // IF RECEVED D FROM SLAVE, BREAK WHILE LOOP
	{
		send_based_on_pcb_id();
		HAL_Delay(100);
	}

	while(!menureceiveflag && uart1_ready)  // // IF RECEVED A FROM SLAVE, BREAK WHILE LOOP
	{
		send_settings_to_all();
		HAL_Delay(1000);
	}

	HAL_UART_Transmit_DMA(&huart1,(uint8_t*)"[M]\r\n",5);
	HAL_Delay(100);

	// Resetting the handshake flags

	Recceived = 0;
	uart1_ready = 0;
	menureceiveflag = 0;

}


static void domestic_handshake_one_board(UART_HandleTypeDef *huart,
                                         char board_id,
                                         volatile uint8_t *p_received,
                                         volatile uint8_t *p_ready,
                                         volatile uint8_t *p_menu_ack)
{
    char id_msg[5] = { '[', board_id, ']', '\r', '\n' };

    /* Step 1: wait for [P] */
    while (!(*p_received))
    {
        if (huart->gState == HAL_UART_STATE_READY)
        {
            HAL_UART_Transmit_DMA(huart, (uint8_t*)"[N]\r\n", 5);
        }
        HAL_Delay(100);
    }

    /* Step 2: wait for [D] */
    while (!(*p_ready))
    {
        if (huart->gState == HAL_UART_STATE_READY)
        {
            HAL_UART_Transmit_DMA(huart, (uint8_t*)id_msg, 5);
        }
        HAL_Delay(100);
    }

    /* Step 3: wait for [A] */
    while (!(*p_menu_ack))
    {
        sprintf(HVLVOUTtxbuf, "[S]%lu,%lu,%lu\r\n",
                working_settings.hv_cut,
                working_settings.lv_cut,
                working_settings.ac_output_voltage);

        if (huart->gState == HAL_UART_STATE_READY)
        {
            HAL_UART_Transmit_DMA(huart,
                                  (uint8_t*)HVLVOUTtxbuf,
                                  strlen(HVLVOUTtxbuf));
        }
        HAL_Delay(100);
    }

    /* Step 4: complete only this board */
    /* Step 4: complete only this board */
    while (huart->gState != HAL_UART_STATE_READY)
    {
        HAL_Delay(1);
    }
    HAL_UART_Transmit_DMA(huart, (uint8_t*)"[M]\r\n", 5);
    HAL_Delay(100);

    /* =========================================================
       Domestic Manual startup only:
       Immediately force this board OFF after menu exit,
       so no brief pulse appears before first display / PB11.
       This does NOT affect:
       - Industrial Auto
       - Industrial Manual
       - Domestic Auto
       ========================================================= */
    if ((working_settings.commericial_set == 'N') &&
        (working_settings.Restart_Mode == 'N'))
    {
        while (huart->gState != HAL_UART_STATE_READY)
        {
            HAL_Delay(1);
        }

        UART_SendStatus_Domestic(huart, 'F');
        HAL_Delay(100);
    }

    /* Keep total settle time close to existing behavior */
    HAL_Delay(1800);

    /* Reset only this board flags */
    *p_received = 0;
    *p_ready = 0;
    *p_menu_ack = 0;
}

void setting_menu_handshake_threephase(void)
{
	LCD_Clear();
    LCD_SetCursor(1, 0);
    LCD_Print("Processing Parameter");
    HAL_Delay(10);

    /* =========================================================
       INDUSTRIAL MODE (UNCHANGED)
       ========================================================= */
    if (working_settings.commericial_set == 'Y')
    {
        while(!RRecceived || !BRecceived || !YRecceived) // IF RECEVED P FROM SLAVE, BREAK WHILE LOOP
        {
            HAL_UART_Transmit_DMA(&huart1,(uint8_t*)"[N]\r\n",5);
            HAL_Delay(100);
            HAL_UART_Transmit_DMA(&huart4,(uint8_t*)"[N]\r\n",5);
            HAL_Delay(100);
            HAL_UART_Transmit_DMA(&huart3,(uint8_t*)"[N]\r\n",5);
            HAL_Delay(100);
        }

        while(!Ruart1_ready || !Yuart2_ready || !Buart3_ready) // IF RECEVED D FROM SLAVE, BREAK WHILE LOOP
        {
            HAL_UART_Transmit_DMA(&huart1,(uint8_t*)"[R]\r\n",5);
            HAL_Delay(100);
            HAL_UART_Transmit_DMA(&huart4,(uint8_t*)"[Y]\r\n",5);
            HAL_Delay(100);
            HAL_UART_Transmit_DMA(&huart3,(uint8_t*)"[B]\r\n",5);
            HAL_Delay(100);
        }

        while(!Rmenureceiveflag || !Ymenureceiveflag || !Bmenureceiveflag) // IF RECEVED A FROM SLAVE, BREAK WHILE LOOP
        {
            sprintf(HVLVOUTtxbuf,"[S]%lu,%lu,%lu\r\n",
                    working_settings.hv_cut,
                    working_settings.lv_cut,
                    working_settings.ac_output_voltage);

            HAL_UART_Transmit_DMA(&huart1,(uint8_t*)HVLVOUTtxbuf,strlen(HVLVOUTtxbuf));
            HAL_Delay(100);
            HAL_UART_Transmit_DMA(&huart4,(uint8_t*)HVLVOUTtxbuf,strlen(HVLVOUTtxbuf));
            HAL_Delay(100);
            HAL_UART_Transmit_DMA(&huart3,(uint8_t*)HVLVOUTtxbuf,strlen(HVLVOUTtxbuf));
            HAL_Delay(100);
        }

        HAL_UART_Transmit_DMA(&huart1,(uint8_t*)"[M]\r\n",5);
        HAL_Delay(2000);
        HAL_UART_Transmit_DMA(&huart4,(uint8_t*)"[M]\r\n",5);
        HAL_Delay(2000);
        HAL_UART_Transmit_DMA(&huart3,(uint8_t*)"[M]\r\n",5);
        HAL_Delay(100);

        /* Resetting the handshake flags */
        RRecceived = 0;
        YRecceived = 0;
        BRecceived = 0;
        Ruart1_ready = 0;
        Yuart2_ready = 0;
        Buart3_ready = 0;
        Rmenureceiveflag = 0;
        Ymenureceiveflag = 0;
        Bmenureceiveflag = 0;

        Rrunningtimeflag = 0;
        Yrunningtimeflag = 0;
        Brunningtimeflag = 0;
        return;
    }

    /* =========================================================
       DOMESTIC MODE (NEW)
       Handshake only for phases currently present
       ========================================================= */
    uint8_t useR = domestic_phase_present_now(0);
    uint8_t useY = domestic_phase_present_now(1);
    uint8_t useB = domestic_phase_present_now(2);

//    /* Optional: update runtime phase status immediately */
//    g_phase_raw_present[0] = useR;
//    g_phase_raw_present[1] = useY;
//    g_phase_raw_present[2] = useB;
//    g_phase_present[0] = useR;
//    g_phase_present[1] = useY;
//    g_phase_present[2] = useB;

    PhaseOut_RuntimeTask();

    /* If phase is present, do individual handshake for that board only */
    if (useR)
    {
        domestic_handshake_one_board(&huart1, 'R',
                                     &RRecceived,
                                     &Ruart1_ready,
                                     &Rmenureceiveflag);
    }
    else
    {
        RRecceived = 0;
        Ruart1_ready = 0;
        Rmenureceiveflag = 0;
    }

    if (useY)
    {
        domestic_handshake_one_board(&huart4, 'Y',
                                     &YRecceived,
                                     &Yuart2_ready,
                                     &Ymenureceiveflag);
    }
    else
    {
        YRecceived = 0;
        Yuart2_ready = 0;
        Ymenureceiveflag = 0;
    }

    if (useB)
    {
        domestic_handshake_one_board(&huart3, 'B',
                                     &BRecceived,
                                     &Buart3_ready,
                                     &Bmenureceiveflag);
    }
    else
    {
        BRecceived = 0;
        Buart3_ready = 0;
        Bmenureceiveflag = 0;
    }

    /* Reset runtime flags as before */
    Rrunningtimeflag = 0;
    Yrunningtimeflag = 0;
    Brunningtimeflag = 0;
}
static uint8_t Industrial_RuntimePhaseSeq_CheckAndDisplay(void)
{
    phase_seq_result = detect_phase_sequence();

    if (phase_seq_result == 1u)
    {
        LCD_Clear();
        LCD_SetCursor(1,0);
        LCD_Print(" PHASE OK ");
        LCD_SetCursor(2,0);
        LCD_Print(" SEQ: R-Y-B ");
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

        HAL_Delay(2000);

        ind_rt_phase_seq_required = 0u;
        ind_rt_phase_seq_display_lock = 0u;
        ind_rt_phase_seq_fail_result  = 0u;

        return 1u;
    }
    else if (phase_seq_result == 3u)
    {
        LCD_Clear();
        LCD_SetCursor(1,0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

        LCD_Print(" PHASE FAULT ");
        LCD_SetCursor(2,0);
        LCD_Print(" SAME PHASE ");
        HAL_Delay(500);

        ind_rt_phase_seq_display_lock = 1u;
        ind_rt_phase_seq_fail_result  = 3u;

        return 0u;
    }
    else
    {
        LCD_Clear();
        LCD_SetCursor(1,0);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

        LCD_Print(" REVERSE SEQUENCE ");
        HAL_Delay(500);

        ind_rt_phase_seq_display_lock = 1u;
        ind_rt_phase_seq_fail_result  = 2u;

        return 0u;
    }
}

//////////////////////////////////////////////////////////////////
/* ======== ISR-side helpers ======== */
static inline void freq_edge_isr_push(uint16_t dt_us)
{
    uint8_t next = (uint8_t)((freq_head + 1U) & FBUF_MASK);
    if (next != freq_tail) {         // drop oldest if full (or just drop new)
        freq_buf[freq_head] = dt_us;
        freq_head = next;
    }
}


void freq_edge_isr_handler(void)
{
    uint16_t t_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim15);

    if (last_edge_us16 == 0) {     // first edge
            last_edge_us16 = t_now;
            return;
        }

    uint16_t dt    = (uint16_t)(t_now - last_edge_us16);  // 16-bit wrap safe
    last_edge_us16 = t_now;

    /* Guard & sanity bounds */
    if (dt < EDGE_GUARD_US)  return;      // ignore re-triggers/glitches <3ms
    if (dt < PERIOD_MIN_US)  return;      // too fast (noise)
    if (dt > PERIOD_MAX_US)  return;      // too slow (missing edges)

    freq_edge_isr_push(dt);
}
static void MX_GPIO_RPhase_EXTI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Configure R_Phase pin as input with rising-edge interrupt */
    GPIO_InitStruct.Pin  = R_Phase_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;   // or PULLDOWN/UP if your hardware needs
    HAL_GPIO_Init(R_Phase_GPIO_Port, &GPIO_InitStruct);

    /* Enable the proper EXTI IRQ for the R_Phase pin number.
       Replace EXTI4_15 with EXTI0_1 or EXTI2_3 if your pin index is 0..3. */
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

static uint16_t Median_U16(uint16_t *arr, uint8_t n)
{
    /* Insertion sort: efficient enough for small buffers */
    for (uint8_t i = 1; i < n; i++)
    {
        uint16_t key = arr[i];
        int8_t j = (int8_t)i - 1;

        while ((j >= 0) && (arr[j] > key))
        {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }

    /* Return middle element (or average of two middle values if even count) */
    if (n & 1U)
    {
        return arr[n / 2];
    }
    else
    {
        uint16_t a = arr[(n / 2) - 1];
        uint16_t b = arr[n / 2];
        return (uint16_t)((a + b) / 2U);
    }
}


/* ======== Consumer: run in main loop (non-ISR) ======== */
static void Freq_Consumer_Task(void)
{
    uint16_t local[FREQ_FBUF_SIZE];
    uint8_t cnt = 0;

    /* Pull available samples from ISR ring buffer */
    __disable_irq();
    while ((freq_tail != freq_head) && (cnt < FREQ_FBUF_SIZE))
    {
        local[cnt++] = freq_buf[freq_tail];
        freq_tail = (uint8_t)((freq_tail + 1U) & FBUF_MASK);
    }
    __enable_irq();

    if (cnt == 0)
        return;

    /* ----------------------------------------------------
       Step 1: Find median of the batch
       ---------------------------------------------------- */
    uint16_t sort_buf[FREQ_FBUF_SIZE];
    for (uint8_t i = 0; i < cnt; i++)
    {
        sort_buf[i] = local[i];
    }

    uint16_t median_us = Median_U16(sort_buf, cnt);

    /* ----------------------------------------------------
       Step 2: Keep only samples near the median
       Threshold = +/-8% around median
       Adjust 8% if needed (5% to 10% is typical)
       ---------------------------------------------------- */
    uint32_t sum = 0;
    uint8_t valid_cnt = 0;

    uint32_t lower = ((uint32_t)median_us * 92U) / 100U;   // -8%
    uint32_t upper = ((uint32_t)median_us * 108U) / 100U;  // +8%

    for (uint8_t i = 0; i < cnt; i++)
    {
        uint16_t d = local[i];
        if ((d >= lower) && (d <= upper))
        {
            sum += d;
            valid_cnt++;
        }
    }

    /* Fallback: if all rejected, use median directly */
    float avg_us;
    if (valid_cnt > 0)
    {
        avg_us = (float)sum / (float)valid_cnt;
    }
    else
    {
        avg_us = (float)median_us;
    }

    /* ----------------------------------------------------
       Step 3: Convert period to frequency
       f_cHz = (1e6 * 100) / avg_us
       ---------------------------------------------------- */
    uint32_t f_cHz = (uint32_t)(100000000.0f / avg_us);

    __disable_irq();
    mains_freq_centiHz = f_cHz;
    __enable_irq();

    mains_freq_hz_cached = (float)f_cHz / 100.0f;
}




/* Safe getter (if you prefer a float) */
static inline float Get_Mains_Freq_Hz(void)
{
    uint32_t cHz;
    __disable_irq();
    cHz = mains_freq_centiHz;
    __enable_irq();
    return (float)cHz / 100.0f;
}

void UART_RxStart_ByMode(void)
{
    if (GetMode() == MODE_SINGLE)
    {
        // Start only USART1
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

        // Stop others (optional but recommended)
        HAL_UART_AbortReceive(&huart4);
        HAL_UART_AbortReceive(&huart3);
    }
    else
    {
        // Start all 3
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_dma, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart4_rx, DMA_IT_HT);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_dma, RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
}

void TIM_Start_ByMode(void)
{
    /* TIM6 always */
    HAL_TIM_Base_Start_IT(&htim6);

    if (GetMode() == MODE_SINGLE)
    {
        HAL_TIM_Base_Start_IT(&htim7);

        /* Add these for frequency measurement in single-phase */
        HAL_TIM_Base_Start(&htim15);
        MX_GPIO_RPhase_EXTI_Init();

        /* Stop three-phase timers */
        HAL_TIM_Base_Stop_IT(&htim14);
        HAL_TIM_Base_Stop_IT(&htim16);
        HAL_TIM_Base_Stop_IT(&htim17);
    }
    else
    {
        /* Stop single-phase timer */
        HAL_TIM_Base_Stop_IT(&htim7);

        HAL_TIM_Base_Start(&htim15);
        MX_GPIO_RPhase_EXTI_Init();

        /* Start three-phase timers */
        HAL_TIM_Base_Start_IT(&htim14);
        HAL_TIM_Base_Start_IT(&htim16);
        HAL_TIM_Base_Start_IT(&htim17);
    }
}

void Runtime_R_Handshake(void)
{
    static uint8_t state = 0;


    switch(state)
    {
        case 0:
            HAL_UART_Transmit_DMA(&huart1,(uint8_t*)"[N]\r\n",5);
            state = 1;
            break;

        case 1:
            if(RRecceived)
            {
                HAL_UART_Transmit_DMA(&huart1,(uint8_t*)"[R]\r\n",5);
                state = 2;
            }
            break;

        case 2:
            if(Ruart1_ready)
            {
                sprintf(HVLVOUTtxbuf,"[S]%lu,%lu,%lu\r\n",
                        working_settings.hv_cut,
                        working_settings.lv_cut,
                        working_settings.ac_output_voltage);

                HAL_UART_Transmit_DMA(&huart1,
                        (uint8_t*)HVLVOUTtxbuf,
                        strlen(HVLVOUTtxbuf));

                state = 3;
            }
            break;

        case 3:
            if(Rmenureceiveflag)
            {
                HAL_UART_Transmit_DMA(&huart1,(uint8_t*)"[M]\r\n",5);

                // Reset only R flags
                RRecceived = 0;
                Ruart1_ready = 0;
                Rmenureceiveflag = 0;
                Rrunningtimeflag = 0;

                state = 0;
            }
            break;
    }
}
void Runtime_Y_Handshake(void)
{
    static uint8_t state = 0;

    switch(state)
    {
        case 0:
            HAL_UART_Transmit_DMA(&huart4,(uint8_t*)"[N]\r\n",5);
            state = 1;
            break;

        case 1:
            if(YRecceived)
            {
                HAL_UART_Transmit_DMA(&huart4,(uint8_t*)"[Y]\r\n",5);
                state = 2;
            }
            break;

        case 2:
            if(Yuart2_ready)
            {
                sprintf(HVLVOUTtxbuf,"[S]%lu,%lu,%lu\r\n",
                        working_settings.hv_cut,
                        working_settings.lv_cut,
                        working_settings.ac_output_voltage);

                HAL_UART_Transmit_DMA(&huart4,
                        (uint8_t*)HVLVOUTtxbuf,
                        strlen(HVLVOUTtxbuf));

                state = 3;
            }
            break;

        case 3:
            if(Ymenureceiveflag)
            {
                HAL_UART_Transmit_DMA(&huart4,(uint8_t*)"[M]\r\n",5);

                YRecceived = 0;
                Yuart2_ready = 0;
                Ymenureceiveflag = 0;
                Yrunningtimeflag = 0;

                state = 0;
            }
            break;
    }
}
void Runtime_B_Handshake(void)
{
    static uint8_t state = 0;

    switch(state)
    {
        case 0:
            HAL_UART_Transmit_DMA(&huart3,(uint8_t*)"[N]\r\n",5);
            state = 1;
            break;

        case 1:
            if(BRecceived)
            {
                HAL_UART_Transmit_DMA(&huart3,(uint8_t*)"[B]\r\n",5);
                state = 2;
            }
            break;

        case 2:
            if(Buart3_ready)
            {
                sprintf(HVLVOUTtxbuf,"[S]%lu,%lu,%lu\r\n",
                        working_settings.hv_cut,
                        working_settings.lv_cut,
                        working_settings.ac_output_voltage);

                HAL_UART_Transmit_DMA(&huart3,
                        (uint8_t*)HVLVOUTtxbuf,
                        strlen(HVLVOUTtxbuf));

                state = 3;
            }
            break;

        case 3:
            if(Bmenureceiveflag)
            {
                HAL_UART_Transmit_DMA(&huart3,(uint8_t*)"[M]\r\n",5);

                BRecceived = 0;
                Buart3_ready = 0;
                Bmenureceiveflag = 0;
                Brunningtimeflag = 0;

                state = 0;
            }
            break;
    }
}

int main(void)
{

  HAL_Init();
  SystemClock_Config();

  /* Peripheral Initialization */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();

  /* USER INIT */

  LCD_Init();

#if (MASTER_MONITOR_MODE == 1)
  /* Bring-up monitor: shows the new slave link on the LCD and never returns.
     Nothing of the original start-up below runs in this mode (no timers, no
     contactor logic, no wait for the old protocol). Set MASTER_MONITOR_MODE to
     0 in master_monitor.h to get the original firmware back. */
#if (MASTER_APP_MODE == 1)
  MasterApp_Run();
#else
  MasterMonitor_Run();
#endif
#endif

#if (MASTER_MONITOR_MODE == 0)
  Load_Settings_From_Flash();

  HAL_TIM_Base_Start_IT(&htim6);


  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_rx_dma, RX_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, uart4_rx_dma, RX_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart4_rx, DMA_IT_HT);

  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, uart3_rx_dma, RX_BUF_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

  R_prev_fault = SYS_FAULT_NONE;
  Y_prev_fault = SYS_FAULT_NONE;
  B_prev_fault = SYS_FAULT_NONE;

  SysFault.R_fault = SYS_FAULT_NONE;
  SysFault.Y_fault = SYS_FAULT_NONE;
  SysFault.B_fault = SYS_FAULT_NONE;

  welcome_message();

  UART_RxStart_ByMode();


  LCD_Clear();
  LCD_SetCursor(1,0);
  LCD_Print("  Checking...  ");
  HAL_Delay(2000);
  LCD_Clear();


  /* =====================================================
     STARTUP PHASE CHECK
     ===================================================== */
  if (working_settings.pcb_id == 1)
  {
      if (working_settings.commericial_set == 'Y')
      {
          Industrial_PhaseGate_WithMenuReentry();
      }
  }

  if (working_settings.pcb_id == 0 && !Srunningtimeflag)
  {
	  LCD_Clear();
	  while(!Srunningtimeflag)
	  {
          LCD_SetCursor(1,0);
          LCD_Print("LCD Power Failure"); //     fluctuation
          LCD_SetCursor(2,0);
          LCD_Print("Restart the System");
          HAL_Delay(100);
	  }
  }
  else if (working_settings.pcb_id == 1 && working_settings.commericial_set == 'Y' && (!Rrunningtimeflag || !Yrunningtimeflag||!Brunningtimeflag) )
  {
	  LCD_Clear();
	  while(!Rrunningtimeflag || !Yrunningtimeflag||!Brunningtimeflag)
	  {
          LCD_SetCursor(1,0);
          LCD_Print("LCD Power Failure"); //     fluctuation
          LCD_SetCursor(2,0);
          LCD_Print("Restart the System");
          HAL_Delay(100);
	  }
  }

  /* =====================================================
     SETTINGS HANDSHAKE
     ===================================================== */

  if (working_settings.pcb_id == 0)
  {
      setting_menu_handshake();
  }
  else
  {
      setting_menu_handshake_threephase();
  }

  ApplyRestartModeRuntime(working_settings.Restart_Mode);
  TIM_Start_ByMode();

  afterstartingmenu = 1;
  static uint32_t last_freq_consume_tick = 0; // Used for Frequency measurement
  LCD_Clear();

  /* =====================================================
     SINGLE PHASE LOOP
     ===================================================== */

  if (working_settings.pcb_id == 0)
  {
      while (1)
      {

          if (enter_settings_req)
          {
              enter_settings_req = 0;

              system_is_off = 1;
              USER_OFF = 1;
              volt_state = STATE_OFF;

              ui_mode = UI_MENU;
              g_force_settings_menu = 1;
              welcome_message();

              if(working_settings.pcb_id == 1)
              {
            	  LCD_Clear();
            	  while(1)
            	  {
                      LCD_SetCursor(1,0);
                      LCD_Print("Mode Change Detected");
                      LCD_SetCursor(2,0);
                      LCD_Print("Restart the System");
                      HAL_Delay(100);
            	  }
              }



              LCD_Clear();
              LCD_SetCursor(1,0);
              LCD_Print(" Settings Done ");
              HAL_Delay(2000);

              ui_mode = UI_RUN;

              ApplyRestartModeRuntime(working_settings.Restart_Mode);
              UART_RxStart_ByMode();


              LCD_Clear();
              setting_menu_handshake();
          }

          /* Current Read */

          curr_R = current_read(ADC_CHANNEL_0,0);

          /* Contactor */

          Contactor1_Handler();


		phaseRoutput_disp = apply_deadband(phaseRoutput_disp,
										   median_filter_push(&mf_sp_vout, phaseRoutput),
										   1);


          LCD_Display_Status_singlephase();


          if (HAL_GetTick() - last_freq_consume_tick >= 100)
          {
              last_freq_consume_tick = HAL_GetTick();
              Freq_Consumer_Task();
              Mainfrequency = Get_Mains_Freq_Hz();
          }
      }
  }

  /* =====================================================
     THREE PHASE LOOP
     ===================================================== */

  else
  {
	  while (1)
      {
    	  /* Settings Menu */
    	  if (enter_settings_req)
		  {
			enter_settings_req = 0;

			system_is_off = 1;
			USER_OFF = 1;
			//              volt_state = STATE_OFF;
			SysFault.R_fault = SYS_FAULT_SYSTEM_OFF;
			SysFault.Y_fault = SYS_FAULT_SYSTEM_OFF;
			SysFault.B_fault = SYS_FAULT_SYSTEM_OFF;

			R_prev_fault = SYS_FAULT_SYSTEM_OFF;
			Y_prev_fault = SYS_FAULT_SYSTEM_OFF;
			B_prev_fault = SYS_FAULT_SYSTEM_OFF;

			/* FIX: reset industrial contactor state before entering runtime settings */
			if (working_settings.commericial_set == 'Y' &&
			working_settings.Restart_Mode == 'Y')
			{
				ThreePhase_Fault_and_Contactor_Task();   // forces static contactor_state -> 0 through system_is_off path
				CSeq_Stop_3Phase();                      // stop any previous C sequence cleanly
				clear_sent = 0;
				contactor_timer = HAL_GetTick();
			}


			  ui_mode = UI_MENU;
			  g_force_settings_menu = 1;
			  welcome_message();
			  if(working_settings.pcb_id == 0)
			  {
				LCD_Clear();
				  while(1)
				  {
                      LCD_SetCursor(1,0);
                      LCD_Print("Mode Change Detected");
                      LCD_SetCursor(2,0);
                      LCD_Print("Restart the System");
                      HAL_Delay(100);
				  }
			  }

			  LCD_Clear();
			  LCD_SetCursor(1,0);
			  LCD_Print(" Settings Done ");
			  HAL_Delay(2000);

			  /* Keep UI locked in MENU state for the whole runtime-setting transaction.
				 PhaseCheck_MenuRequest() already polls PB10+PB13 directly, so reopen still works. */
			  enter_settings_req = 0u;
			  combo_active = 0u;
			  pb12_pending_off = 0u;

			  if (working_settings.pcb_id == 1)
			  {
				  if (working_settings.commericial_set == 'Y')
				  {
					  Industrial_PhaseGate_WithMenuReentry();
				  }
			  }

			  /* Consume any nested request raised while user was still holding PB10+PB13 */
			  enter_settings_req = 0u;
			  combo_active = 0u;
			  pb12_pending_off = 0u;
			  UART_RxStart_ByMode();

			  setting_menu_handshake_threephase();

			  LCD_Clear();
			  LCD_SetCursor(1,0);
			  LCD_Print(" After Handshake ");
			  HAL_Delay(2000);

			  /* Remove temporary SYSTEM OFF placeholders before restart */
			  if ((working_settings.pcb_id == 1) &&
				  (working_settings.commericial_set == 'Y'))
			  {
				  Clear_SystemOff_Faults_3Phase();
			  }

			  ApplyRestartModeRuntime(working_settings.Restart_Mode);

			  LCD_Clear();
			  LCD_SetCursor(1,0);
			  LCD_Print(" Restart Completed ");
			  HAL_Delay(2000);
			  LCD_Clear();

			  ui_mode = UI_RUN;
		  }


    	  PhaseOut_RuntimeTask();
    	  Industrial_RuntimePhaseSeq_Monitor();
    	  Industrial_ManualRestart_Task();
    	  CSeq_Task_3Phase();


          /* Domestic Manual startup with phase missing:
             send C only to present boards */
    	  /* Domestic selective restart for partial-fault case */
    	  if (dom_partial_start_req)
    	  {
    	      uint8_t mask = dom_partial_start_mask;
    	      dom_partial_start_req = 0u;
    	      Domestic_SendC_To_SelectedBoards(mask);
    	  }

          if (clear_sent == 2 && !CSeq_IsActive())
          {
              clear_sent = 1;
              contactor_timer = HAL_GetTick();
          }

          /* Runtime Handshake */

          if (Rrunningtimeflag)
              Runtime_R_Handshake();

          if (Yrunningtimeflag)
              Runtime_Y_Handshake();

          if (Brunningtimeflag)
              Runtime_B_Handshake();

          /* Current Read */

          curr_R = current_read(ADC_CHANNEL_0,0);
          curr_Y = current_read(ADC_CHANNEL_1,1);
          curr_B = current_read(ADC_CHANNEL_2,2);

          /* Contactor + Fault */

          ThreePhase_Fault_and_Contactor_Task();

          /* LCD */

          /* Display-only filtering */
//          Rphaseoutput_disp = median_filter_push(&mf_r_vout, Rphaseoutput);
//          Yphaseoutput_disp = median_filter_push(&mf_y_vout, Yphaseoutput);
//          Bphaseoutput_disp = median_filter_push(&mf_b_vout, Bphaseoutput);
//
//
//          if(( Rphaseoutput_disp > (working_settings.ac_output_voltage - 2))
//              && ( Rphaseoutput_disp < (working_settings.ac_output_voltage + 2)))
//        	  Rphaseoutput_disp = working_settings.ac_output_voltage;
//
//          if(( Yphaseoutput_disp > (working_settings.ac_output_voltage - 2))
//			  && ( Yphaseoutput_disp < (working_settings.ac_output_voltage + 2)))
//			  Yphaseoutput_disp = working_settings.ac_output_voltage;
//
//          if(( Bphaseoutput_disp > (working_settings.ac_output_voltage - 2))
//			  && ( Bphaseoutput_disp < (working_settings.ac_output_voltage + 2)))
//			  Bphaseoutput_disp = working_settings.ac_output_voltage;

           Rphaseoutput_disp = apply_deadband(Rphaseoutput_disp,
         		  	  	  	  	  	  	  	 median_filter_push(&mf_r_vout, Rphaseoutput),
                                              1);
           Yphaseoutput_disp = apply_deadband(Yphaseoutput_disp,
											 median_filter_push(&mf_y_vout, Yphaseoutput),
										     1);
           Bphaseoutput_disp = apply_deadband(Bphaseoutput_disp,
											 median_filter_push(&mf_b_vout, Bphaseoutput),
											 1);


          LCD_Display_Status_threephase();

          if (HAL_GetTick() - last_freq_consume_tick >= 100)  // every 100 ms
           {
               last_freq_consume_tick = HAL_GetTick();
               Freq_Consumer_Task();  // pulls new samples and updates mains_freq_centiHz
           }
           Mainfrequency = Get_Mains_Freq_Hz();

      }
  }
#endif /* MASTER_MONITOR_MODE == 0: the original start-up and main loop, not built in monitor/app mode (they can never run there, and they filled the flash) */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 64000 -1 ;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 250 -1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/* ================= TIM14 : 2 SECOND TIMER ================= */


static void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;

  /* 64 MHz / 64000 = 1 kHz (1 ms tick)
     Period = 2000 = 2 second */

  htim14.Init.Prescaler = 64000 - 1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 100 - 1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }

}


/* ======== TIM15 @ 1 MHz free-running ======== */
static void MX_TIM15_Init(void)
{
    /* TIM15 is a 16-bit general-purpose timer on STM32G0.
       We configure it to tick at 1 MHz so the counter increments ~1us. */

    htim15.Instance = TIM15;

    /* Prescaler calculation:
       SystemCoreClock / (PSC+1) = 1,000,000 Hz  => PSC = SystemCoreClock/1e6 - 1
       Works for HSI 16 MHz (PSC=15) and other integer multiples. */
    uint32_t psc = (SystemCoreClock / 1000000UL);
    if (psc == 0) psc = 1;
    psc -= 1U;

    htim15.Init.Prescaler         = (uint16_t)psc;
    htim15.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim15.Init.Period            = 0xFFFF;             // 16-bit wraps ~65ms
    htim15.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim15.Init.RepetitionCounter = 0;
    htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
    {
        Error_Handler();
    }
}


/* ================= TIM16 : 10 ms TIMER ================= */

static void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;

  /* 10 ms timer */

  htim16.Init.Prescaler = 64000 - 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100 - 1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }

}


/* ================= TIM17 : 100 ms TIMER ================= */

static void MX_TIM17_Init(void)
{

  htim17.Instance = TIM17;

  /* 100 ms timer */

  htim17.Init.Prescaler = 64000 - 1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 100 - 1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }

}
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = (MASTER_MONITOR_MODE ? LINK_BAUDRATE : 115200UL);   /* monitor mode: new link at 500000 baud */
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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

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
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Ch4_7_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMAMUX1_OVR_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */


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
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : R_Phase_Pin Y_Phase_Pin B_Phase_Pin ON_OFF_Pin */
  GPIO_InitStruct.Pin = R_Phase_Pin|Y_Phase_Pin|B_Phase_Pin|ON_OFF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CONT_1_Pin CONT_2_Pin CONT_3_Pin */
  GPIO_InitStruct.Pin = CONT_1_Pin|CONT_2_Pin|CONT_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
