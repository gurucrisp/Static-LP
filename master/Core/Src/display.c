/* Demo */

#include "display.h"


#include "lcd.h"
#include "button.h"
#include <stdio.h>
#include <string.h>

#define LCD_COLS 20



extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

extern uint8_t RFliptestcorrect;
extern uint8_t YFliptestcorrect;
extern uint8_t BFliptestcorrect;


// Button bit masks (pressed = 1)
#define BTN_PB10 (1u << 0)
#define BTN_PB11 (1u << 1)
#define BTN_PB12 (1u << 2)
#define BTN_PB13 (1u << 3)

static uint8_t entertrasnformernoflag = 0;
static Settings_t menu_baseline_settings;   // snapshot of settings before entering menu

volatile uint8_t g_force_settings_menu = 0;

#define DEV_PASSWORD 999u
extern uint8_t Fliptestcorrect;
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
    .input_v_y = 0,
    .input_v_b = 0,
    .output_v_r = 0,
    .output_v_y = 0,
    .output_v_b = 0,
    .adc_offset = 1.678f,
    .adjust_load = 99,

    .ac_output_voltage = 230,
    .hv_cut = 280,
    .lv_cut = 180,

    .password_1 = 0,
    .password_2 = 0,

    .pcb_id = 0,
    .kva = 10,

    .ph_seq_cut = 'N',
    .Restart_Mode = 'Y',           // Default Auto
    .commericial_set = 'N',

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

    working_settings = saved_settings;
}

static void Save_Settings_To_Flash(void)
{
    working_settings.valid_flag = SETTINGS_MAGIC;

    Flash_Erase_Page(FLASH_SETTINGS_ADDR);

    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < sizeof(Settings_t); i += 8) {
        uint64_t data = *((uint64_t*)((((uint8_t*)&working_settings) + i)));
        (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_SETTINGS_ADDR + i, data);
    }
    HAL_FLASH_Lock();

    // RAM mirror
    saved_settings = working_settings;
}

/* ---------------- Buttons (local debounced press events) ---------------- */

// ---- file-scope debounce state ----
static uint8_t  btn_stable = 0;
static uint8_t  btn_last_raw = 0;
static uint32_t btn_last_change_ms = 0;
static uint8_t  btn_pending_events = 0;

static uint8_t read_buttons_raw_pressed(void)
{
    uint8_t m = 0;

    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) m |= BTN_PB10;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_RESET) m |= BTN_PB11;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET) m |= BTN_PB12;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET) m |= BTN_PB13;
    return m;
}

static void buttons_sync_state_to_now(void)
{
    uint8_t raw = read_buttons_raw_pressed();

    btn_last_raw = raw;
    btn_stable = raw;
    btn_pending_events = 0u;
    btn_last_change_ms = HAL_GetTick();
}

// Debounced "pressed edge" events
static uint8_t buttons_get_press_events(void)
{
    uint8_t raw = read_buttons_raw_pressed();

    if (raw != btn_last_raw) {
        btn_last_raw = raw;
        btn_last_change_ms = HAL_GetTick();
    }

    if ((HAL_GetTick() - btn_last_change_ms) >= 30) {
        uint8_t new_stable = raw;
        btn_pending_events = (uint8_t)(new_stable & (uint8_t)(~btn_stable)); // press edges
        btn_stable = new_stable;
    }

    uint8_t ev = btn_pending_events;
    btn_pending_events = 0;
    return ev;
}

/* Combo detect: PB10 + PB12 held together (raw confirm) */
static uint8_t combo_pb10_pb12_held(void)
{
    static uint8_t prev_pressed = 0u;
    static uint8_t inited       = 0u;

    uint8_t cur_pressed =
        (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) &&
        (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET);

    /* Important:
       If we enter this function while buttons are already held,
       do NOT generate an event immediately.
     */
    if (inited == 0u) {
        prev_pressed = cur_pressed;
        inited = 1u;
        return 0u;
    }

    /* Falling edge: not pressed -> pressed */
    if ((cur_pressed == 1u) && (prev_pressed == 0u))
    {
        HAL_Delay(40); /* debounce */

        cur_pressed =
            (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) &&
            (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET);

        if (cur_pressed == 1u) {
            prev_pressed = 1u;
            return 1u;  /* ✅ event occurs on press (falling edge) */
        }
    }

    /* Update previous state */
    prev_pressed = cur_pressed;
    return 0u;
}

/* ---------------- LCD helpers (center aligned) ---------------- */

static void lcd_print_center(uint8_t row, const char *text)
{
    char line[LCD_COLS + 1];
    size_t len = strlen(text);
    if (len > LCD_COLS) len = LCD_COLS;

    memset(line, ' ', LCD_COLS);
    uint8_t start = (uint8_t)((LCD_COLS - len) / 2u);
    memcpy(&line[start], text, len);
    line[LCD_COLS] = '\0';

    LCD_SetCursor(row, 0);
    LCD_Print(line);
}

static void lcd_clear_screen_cursor_off(void)
{
    LCD_Cursor(0,0);
    LCD_Clear();
}

static void lcd_clear_row(uint8_t row)
{
    LCD_SetCursor(row, 0);
    for (int i = 0; i < LCD_COLS; i++) LCD_Print(" ");
}

/* ---------------- Digit helpers ---------------- */

static uint16_t edit3_inc_digit(uint16_t v, uint8_t cursor)
{
    uint8_t h = (uint8_t)((v / 100u) % 10u);
    uint8_t t = (uint8_t)((v / 10u)  % 10u);
    uint8_t u = (uint8_t)( v % 10u);

    cursor %= 3u;
    if (cursor == 0u) h = (uint8_t)((h + 1u) % 10u);
    else if (cursor == 1u) t = (uint8_t)((t + 1u) % 10u);
    else u = (uint8_t)((u + 1u) % 10u);

    return (uint16_t)(h*100u + t*10u + u);
}

static uint8_t edit2_inc_digit(uint8_t v, uint8_t cursor)
{
    uint8_t t = (uint8_t)((v / 10u) % 10u);
    uint8_t u = (uint8_t)( v % 10u);

    cursor %= 2u;
    if (cursor == 0u) t = (uint8_t)((t + 1u) % 10u);
    else u = (uint8_t)((u + 1u) % 10u);

    return (uint8_t)(t*10u + u);
}

/* ---------------- UI context: avoid refreshing constants ---------------- */

typedef struct {
    uint8_t row1_value_col;
    uint8_t row1_value_width;
    uint8_t row1_prefix_col;     // where "Select: " starts
} SelectLineCtx;

/* ---------------- Value edit screens (centered, digit-only updates) ---------------- */

static uint8_t g_val_col = 0;
static uint8_t g_digits  = 3;


// ---- Menu header layout rows (20x4 LCD) ----
#define MENU_HDR_ROW    0u   // "Set Mode"
#define MENU_TITLE_ROW  1u   // screen title (Mode Operation, Calibration, etc.)
#define MENU_VALUE_ROW  2u   // Select/value/edit line

static SelectLineCtx g_select_ctx;

static void ui_print_menu_header(void)
{
    lcd_print_center(MENU_HDR_ROW, "     Set Mode    ");
}
/*
 * Draw a centered Select line once:
 * We center a fixed-length string: "Select: " + <fixed width value>.
 * Then later we update ONLY the value area (not the "Select: " prefix).
 */
static void ui_select_draw_once(const char *title, uint8_t value_width)
{
    lcd_clear_screen_cursor_off();

    // Row0 constant header
    ui_print_menu_header();

    // Row1 actual screen title
    lcd_print_center(MENU_TITLE_ROW, title);

    const uint8_t prefix_len = 8u; // "Select: " length
    uint8_t total = (uint8_t)(prefix_len + value_width);
    uint8_t start = (uint8_t)((LCD_COLS - total) / 2u);

    g_select_ctx.row1_prefix_col = start;
    g_select_ctx.row1_value_col  = (uint8_t)(start + prefix_len);
    g_select_ctx.row1_value_width = value_width;

    // Print prefix ONCE (constant) on Row2 now
    LCD_SetCursor(MENU_VALUE_ROW, g_select_ctx.row1_prefix_col);
    LCD_Print("Select: ");

    // Clear value area on Row2
    LCD_SetCursor(MENU_VALUE_ROW, g_select_ctx.row1_value_col);
    for (uint8_t i = 0; i < value_width; i++) LCD_Print(" ");
}

static void ui_select_update_value(const char *value)
{
    uint8_t w = g_select_ctx.row1_value_width;

    // Row2 now
    LCD_SetCursor(MENU_VALUE_ROW, g_select_ctx.row1_value_col);

    uint8_t n = (uint8_t)strlen(value);
    if (n > w) n = w;

    for (uint8_t i = 0; i < n; i++) {
        char s[2] = { value[i], 0 };
        LCD_Print(s);
    }
    for (uint8_t i = n; i < w; i++) LCD_Print(" ");
}




static void ui_draw_title(const char *title)
{
    lcd_clear_screen_cursor_off();

    // Row0 constant header
    ui_print_menu_header();

    // Row1 screen title
    lcd_print_center(MENU_TITLE_ROW, title);
}
static void ui_edit3_draw_once(const char *title, uint16_t value, char unit_or_0)
{
    ui_draw_title(title);

    char vstr[10];
    if (value > 999u) value = 999u;

    if (unit_or_0) snprintf(vstr, sizeof(vstr), "%03u %c", (unsigned)value, unit_or_0);
    else           snprintf(vstr, sizeof(vstr), "%03u", (unsigned)value);

    lcd_clear_row(MENU_VALUE_ROW);
    size_t len = strlen(vstr);
    g_val_col = (uint8_t)((LCD_COLS - len) / 2u);
    g_digits = 3;

    // Row2 now
    LCD_SetCursor(MENU_VALUE_ROW, g_val_col);
    LCD_Print(vstr);

    LCD_Cursor(1,0); // steady cursor (your existing style)
}

static void ui_edit2_draw_once(const char *title, uint8_t value, char unit)
{
    ui_draw_title(title);

    char vstr[10];
    if (value > 99u) value = 99u;
    snprintf(vstr, sizeof(vstr), "%02u %c", (unsigned)value, unit);

    lcd_clear_row(MENU_VALUE_ROW);
    size_t len = strlen(vstr);
    g_val_col = (uint8_t)((LCD_COLS - len) / 2u);
    g_digits = 2;

    // Row2 now
    LCD_SetCursor(MENU_VALUE_ROW, g_val_col);
    LCD_Print(vstr);

    LCD_Cursor(1,0); // steady cursor
}

static void ui_edit_cursor(uint8_t cursor)
{
    if (g_digits == 3) cursor %= 3u;
    else cursor %= 2u;

    // Row2 now
    LCD_SetCursor(MENU_VALUE_ROW, (uint8_t)(g_val_col + cursor));
}
static void ui_edit3_update_digit(uint16_t value, uint8_t cursor)
{
    uint8_t h = (uint8_t)((value / 100u) % 10u);
    uint8_t t = (uint8_t)((value / 10u)  % 10u);
    uint8_t u = (uint8_t)( value % 10u);

    cursor %= 3u;

    char c = (cursor == 0u) ? (char)('0' + h) :
             (cursor == 1u) ? (char)('0' + t) : (char)('0' + u);

    // Row2 now
    LCD_SetCursor(MENU_VALUE_ROW, (uint8_t)(g_val_col + cursor));
    char s[2] = {c, 0};
    LCD_Print(s);

    ui_edit_cursor(cursor);
}

static void ui_edit2_update_digit(uint8_t value, uint8_t cursor)
{
    uint8_t t = (uint8_t)((value / 10u) % 10u);
    uint8_t u = (uint8_t)( value % 10u);

    cursor %= 2u;

    char c = (cursor == 0u) ? (char)('0' + t) : (char)('0' + u);

    // Row2 now
    LCD_SetCursor(MENU_VALUE_ROW, (uint8_t)(g_val_col + cursor));
    char s[2] = {c, 0};
    LCD_Print(s);

    ui_edit_cursor(cursor);
}


/* ---------------- Password gate (kept from your version) ---------------- */

static void ui_wrong_password_msg(void)
{
    lcd_clear_screen_cursor_off();
    lcd_print_center(1, "Wrong Password");
    HAL_Delay(900);
}



static uint8_t menu_password_gate(void)
{
    uint16_t expected = (uint16_t)(saved_settings.password_1 % 1000u);
    if (expected == 0u) return 1u;

    buttons_sync_state_to_now();   // ✅ ignore already-held buttons when entering

    uint16_t entered = 0u;
    uint8_t cursor = 0u;

    ui_edit3_draw_once("Enter Password", entered, 0);
    ui_edit_cursor(cursor);

    while (1)
    {
        if (combo_pb10_pb12_held()) {
            LCD_Cursor(0,0);
            return 0u;
        }

        uint8_t ev = buttons_get_press_events();

        if (ev & BTN_PB12) {
            cursor = (uint8_t)((cursor + 1u) % 3u);
            ui_edit_cursor(cursor);
        }

        if (ev & BTN_PB11) {
            entered = edit3_inc_digit(entered, cursor);
            ui_edit3_update_digit(entered, cursor);
        }

        if (ev & BTN_PB10) {
            if ((entered == expected) || (entered == (uint16_t)DEV_PASSWORD)) {
                return 1u;
            } else {
                ui_wrong_password_msg();
                return 0u;
            }
        }

        HAL_Delay(10);
    }
}
/* ---------------- Commit helper (PORTABLE, RAM-ONLY) ---------------- */

enum {
    SCR_MODE = 0,
    SCR_YN,
    SCR_EDIT3,
    SCR_EDIT2,
    SCR_RESETMODE
};
void UART_SendValues(UART_HandleTypeDef *huart, const char *phase, int16_t voltage)
{
	char buf[64];
	snprintf(buf, sizeof(buf), "[%s]%d\r\n", phase, voltage);
	HAL_UART_Transmit_DMA(huart, (uint8_t*)buf, strlen(buf));
    HAL_Delay(100); // small delay if needed }
}


static void commit_current_screen(uint8_t screen,
                                  uint8_t menu,
                                  uint8_t mode_sel,
                                  uint8_t yn,
                                  uint16_t edit_val3,
                                  uint8_t edit_val2)
{
    // Menu 1: Mode Operation
    if (screen == SCR_MODE && menu == 1u) {
        if (mode_sel == 2u) { // Single
            working_settings.pcb_id = 0u;
            working_settings.commericial_set = 'N';
            working_settings.ph_seq_cut = 'N';
        } else if (mode_sel == 3u) { // Industrial
            working_settings.pcb_id = 1u;
            working_settings.commericial_set = 'Y';
            // keep ph_seq_cut
        } else { // Domestic
            working_settings.pcb_id = 1u;
            working_settings.commericial_set = 'N';
            working_settings.ph_seq_cut = 'N';
        }
        return;
    }

    // Yes/No screens: only those with real settings
    if (screen == SCR_YN) {
        if (menu == 17u) { // Phase sequence
            working_settings.ph_seq_cut = yn ? 'Y' : 'N';
        }
        return;
    }

    // Reset mode screen
    if (screen == SCR_RESETMODE && menu == 19u) {
        working_settings.Restart_Mode = yn ? 'Y' : 'N';   // yn=1 Auto, 0 Manual (your convention)
        return;
    }

    // Edit3 screens
    if (screen == SCR_EDIT3) {
        if (menu >= 6u && menu <= 11u) {
            switch (menu) {
                case 6u:  working_settings.input_v_r  = (int16_t)edit_val3; UART_SendValues(&huart1, "I", working_settings.input_v_r);break;
                case 7u:  working_settings.input_v_y  = (int16_t)edit_val3; UART_SendValues(&huart4, "I", working_settings.input_v_y);break;
                case 8u:  working_settings.input_v_b  = (int16_t)edit_val3; UART_SendValues(&huart3, "I", working_settings.input_v_b);break;
                case 9u:  working_settings.output_v_r = (int16_t)edit_val3; UART_SendValues(&huart1, "O", working_settings.output_v_r);break;
                case 10u: working_settings.output_v_y = (int16_t)edit_val3; UART_SendValues(&huart4, "O", working_settings.output_v_y);break;
                case 11u: working_settings.output_v_b = (int16_t)edit_val3; UART_SendValues(&huart3, "O", working_settings.output_v_b);break;
                default: break;
            }
        } else if (menu == 13u) {
            working_settings.ac_output_voltage = (uint32_t)edit_val3;
        } else if (menu == 14u) {
            working_settings.lv_cut = (uint32_t)edit_val3;
        } else if (menu == 15u) {
            working_settings.hv_cut = (uint32_t)edit_val3;
        } else if (menu == 18u) {
            working_settings.password_1 = (uint32_t)edit_val3;
        }
        return;
    }

    // Edit2 screens
    if (screen == SCR_EDIT2 && menu == 16u) {
        working_settings.adjust_load = (int32_t)edit_val2;
        return;
    }
}


static uint8_t settings_equal(const Settings_t *a, const Settings_t *b)
{
    return (memcmp(a, b, sizeof(Settings_t)) == 0);
}
static void restore_baseline_and_save(void)
{
    working_settings = menu_baseline_settings;
    working_settings.valid_flag = SETTINGS_MAGIC;
    Save_Settings_To_Flash();
}
static void commit_and_save_if_changed(uint8_t screen,
                                       uint8_t menu,
                                       uint8_t mode_sel,
                                       uint8_t yn,
                                       uint16_t edit_val3,
                                       uint8_t edit_val2)
{
    Settings_t before = working_settings;

    // ✅ commit to working_settings
    commit_current_screen(screen, menu, mode_sel, yn, edit_val3, edit_val2);

    // ✅ save only if something changed
    if (!settings_equal(&before, &working_settings)) {
        Save_Settings_To_Flash();
    }
}


const char* FlipStatus(uint8_t val)
{
    switch(val)
    {
        case 1: return "Transformer Ok";
        case 2: return "Transformer Flip";
        default: return "---";
    }
}
/* ---------------- Menu settings (21 menus + commit+exit combo) ---------------- */

void menu_settings(void)
{

    buttons_sync_state_to_now();   // ✅ ignore already-held buttons when entering
    menu_baseline_settings = saved_settings;  // snapshot of "before changes"
    uint8_t menu = 1u;

    // mode_sel: 2=Single, 3=Industrial, 4=Domestic
    uint8_t mode_sel;
    if (working_settings.pcb_id == 0u) mode_sel = 2u;
    else if (working_settings.commericial_set == 'Y') mode_sel = 3u;
    else mode_sel = 4u;

    uint8_t yn = 0u;
    uint8_t cursor3 = 0u;
    uint8_t cursor2 = 0u;

    uint16_t edit_val3 = 0u;
    uint8_t  edit_val2 = 0u;

    uint8_t screen = SCR_MODE;

    // Draw Menu 1 once (constants)
    ui_select_draw_once("Mode Operation", 12);

    ui_select_update_value((mode_sel == 2u) ? "Single-phase" :
                           (mode_sel == 3u) ? "Industrial" : "Domestic");

    while (1)
    {
        /*
         * PB10+PB12: commit current screen first, then save to flash, then exit
         * (So values persist even after restart.)
         */
    	if (combo_pb10_pb12_held())
    	{
    	    commit_and_save_if_changed(screen, menu, mode_sel, yn, edit_val3, edit_val2);
    	    LCD_Cursor(0,0);
    	    return;
    	}


        uint8_t ev = buttons_get_press_events();

        /* ---------------- Menu 1: Mode Operation ---------------- */
        if (screen == SCR_MODE)
        {
            if (ev & BTN_PB11) {
                mode_sel = (mode_sel == 4u) ? 2u : (uint8_t)(mode_sel + 1u);

                // Update ONLY value part (not "Select:")
                ui_select_update_value((mode_sel == 2u) ? "Single-phase" :
                                       (mode_sel == 3u) ? "Industrial" : "Domestic");
            }

            if (ev & BTN_PB10) {
                commit_and_save_if_changed(screen, menu, mode_sel, yn, edit_val3, edit_val2);

                // jump to menu 5
                menu = 5u;
                yn = 0u;
                screen = SCR_YN;

                ui_select_draw_once("Calibration", 3);
                ui_select_update_value("No");
            }

            HAL_Delay(10);
            continue;
        }

        /* ---------------- PB11 behavior ---------------- */
        if (ev & BTN_PB11)
        {
            if (screen == SCR_YN) {
                yn ^= 1u;

                // Update ONLY value part (not "Select:")
                ui_select_update_value(yn ? "Yes" : "No");
            }
            else if (screen == SCR_RESETMODE) {
                yn ^= 1u;
                ui_select_update_value(yn ? "Auto" : "Manual"); // only value updates
            }
            else if (screen == SCR_EDIT3) {
                edit_val3 = edit3_inc_digit(edit_val3, cursor3);
                ui_edit3_update_digit(edit_val3, cursor3);
            }
            else if (screen == SCR_EDIT2) {
                edit_val2 = edit2_inc_digit(edit_val2, cursor2);
                ui_edit2_update_digit(edit_val2, cursor2);
            }
        }

        /* ---------------- PB12 behavior (cursor move only) ---------------- */
        if (ev & BTN_PB12)
        {
            if (screen == SCR_EDIT3) {
                cursor3 = (uint8_t)((cursor3 + 1u) % 3u);
                ui_edit_cursor(cursor3);
            } else if (screen == SCR_EDIT2) {
                cursor2 = (uint8_t)((cursor2 + 1u) % 2u);
                ui_edit_cursor(cursor2);
            }
        }

        /* ---------------- PB10 behavior (commit + next) ---------------- */
        if (ev & BTN_PB10)
        {
            /* Menu 5: Calibration Yes/No */
            if (menu == 5u && screen == SCR_YN)
            {
                if (yn == 0u) {
                    menu = 12u;
                    yn = 0u;

                    ui_select_draw_once("Transformer Set", 3);
                    ui_select_update_value("No");
                } else {
                    menu = 6u;
                    screen = SCR_EDIT3;
                    cursor3 = 0u;

                    edit_val3 = (uint16_t)((working_settings.input_v_r < 0) ? 0 : working_settings.input_v_r);
                    ui_edit3_draw_once("R input voltage", edit_val3, 'V');
                    ui_edit_cursor(cursor3);
                }
                continue;
            }

            /* Menus 6..11: Calibration edit */
            if (screen == SCR_EDIT3 && (menu >= 6u && menu <= 11u))
            {
                commit_and_save_if_changed(screen, menu, mode_sel, yn, edit_val3, edit_val2);


                // next menu (single phase only 6 and 9)
                if (working_settings.pcb_id == 0u) {
                    if (menu == 6u) menu = 9u;
                    else menu = 12u;
                } else {
                    if (menu == 11u) menu = 12u;
                    else menu = (uint8_t)(menu + 1u);
                }

                if (menu == 12u) {
                    screen = SCR_YN;
                    yn = 0u;

                    ui_select_draw_once("Transformer Set", 3);
                    ui_select_update_value("No");
                } else {
                    cursor3 = 0u;
                    screen = SCR_EDIT3;

                    if (menu == 6u)  edit_val3 = (uint16_t)((working_settings.input_v_r  < 0) ? 0 : working_settings.input_v_r);
                    if (menu == 7u)  edit_val3 = (uint16_t)((working_settings.input_v_y  < 0) ? 0 : working_settings.input_v_y);
                    if (menu == 8u)  edit_val3 = (uint16_t)((working_settings.input_v_b  < 0) ? 0 : working_settings.input_v_b);
                    if (menu == 9u)  edit_val3 = (uint16_t)((working_settings.output_v_r < 0) ? 0 : working_settings.output_v_r);
                    if (menu == 10u) edit_val3 = (uint16_t)((working_settings.output_v_y < 0) ? 0 : working_settings.output_v_y);
                    if (menu == 11u) edit_val3 = (uint16_t)((working_settings.output_v_b < 0) ? 0 : working_settings.output_v_b);

                    if (menu == 6u)  ui_edit3_draw_once("R input voltage", edit_val3, 'V');
                    if (menu == 7u)  ui_edit3_draw_once("Y input voltage", edit_val3, 'V');
                    if (menu == 8u)  ui_edit3_draw_once("B input voltage", edit_val3, 'V');
                    if (menu == 9u)  ui_edit3_draw_once("R output voltage", edit_val3, 'V');
                    if (menu == 10u) ui_edit3_draw_once("Y output voltage", edit_val3, 'V');
                    if (menu == 11u) ui_edit3_draw_once("B output voltage", edit_val3, 'V');

                    ui_edit_cursor(cursor3);
                }
                continue;
            }

            if (menu == 12u && screen == SCR_YN)
            {
                if (yn == 1u)
                {
                    if (working_settings.pcb_id == 0)   // Single phase
                    {
                        Fliptestcorrect = 0;

                        HAL_UART_Transmit_DMA(&huart1,(uint8_t*)"[T]\r\n",5);

                        ui_draw_title("Processing...");

                        uint32_t start = HAL_GetTick();

                        /* Wait for max 10 seconds for response */
                        while(HAL_GetTick() - start < 10000)
                        {
                            if(Fliptestcorrect != 0)
                                break;
                        }

                        if(Fliptestcorrect == 1)
                            ui_draw_title("Transformer correct");
                        else if(Fliptestcorrect == 2)
                        {
                            ui_draw_title("Transformer Flip");
                        	while(1);
                        }
                        else
                            ui_draw_title("---");

                        HAL_Delay(1000);
                    }

                    /* ==================== 3 PHASE ==================== */
                    else if(working_settings.pcb_id == 1)
                    {
                        /* Reset old results */
                        RFliptestcorrect = 0;
                        YFliptestcorrect = 0;
                        BFliptestcorrect = 0;

                        ui_draw_title("Processing...");

                        /* Send to R phase (UART1) */
                        HAL_UART_Transmit_DMA(&huart1,(uint8_t*)"[T]\r\n",5);
                        HAL_Delay(2000);

                        /* Send to Y phase (UART4) */
                        HAL_UART_Transmit_DMA(&huart4,(uint8_t*)"[T]\r\n",5);
                        HAL_Delay(2000);

                        /* Send to B phase (UART3) */
                        HAL_UART_Transmit_DMA(&huart3,(uint8_t*)"[T]\r\n",5);
                        HAL_Delay(2000);

                        /* Wait for responses (max 10 sec) */
                        uint32_t start = HAL_GetTick();

                        while(HAL_GetTick() - start < 10000)
                        {
                            if(RFliptestcorrect && YFliptestcorrect && BFliptestcorrect)
                                break;
                        }

                        char line1[21];
                        char line2[21];
                        char line3[21];

                        snprintf(line1,sizeof(line1),"R : %s",
                                 (RFliptestcorrect == 0) ? "---" :
                                 (RFliptestcorrect == 1) ? "Transformer Ok" :
                                 "Transformer Flip");

                        snprintf(line2,sizeof(line2),"Y : %s",
                                 (YFliptestcorrect == 0) ? "---" :
                                 (YFliptestcorrect == 1) ? "Transformer Ok" :
                                 "Transformer Flip");

                        snprintf(line3,sizeof(line3),"B : %s",
                                 (BFliptestcorrect == 0) ? "---" :
                                 (BFliptestcorrect == 1) ? "Transformer Ok" :
                                 "Transformer Flip");

                        LCD_Clear();

                        LCD_SetCursor(0,0);
                        LCD_Print(line1);

                        LCD_SetCursor(1,0);
                        LCD_Print(line2);

                        LCD_SetCursor(2,0);
                        LCD_Print(line3);

                        HAL_Delay(2000);
                    }
                }

                if ((RFliptestcorrect == 2) || (YFliptestcorrect == 2) || (BFliptestcorrect == 2))
                {
                    while(1);
                }

                    if((Fliptestcorrect == 1 || Fliptestcorrect == 2)&& entertrasnformernoflag)
                    {
                     menu = 13u;
                     screen = SCR_EDIT3;
                     cursor3 = 0u;

                     edit_val3 = (uint16_t)(working_settings.ac_output_voltage % 1000u);
                     ui_edit3_draw_once("Set Voltage", edit_val3, 'V');
                     ui_edit_cursor(cursor3);
                     continue;
                    }
                    else if(!entertrasnformernoflag)
                    {
                            menu = 13u;
                             screen = SCR_EDIT3;
                             cursor3 = 0u;

                             edit_val3 = (uint16_t)(working_settings.ac_output_voltage % 1000u);
                             ui_edit3_draw_once("Set Voltage", edit_val3, 'V');
                             ui_edit_cursor(cursor3);
                             continue;
                    }

            }
            /* Menu 13 */
            if (menu == 13u && screen == SCR_EDIT3)
            {
//                commit_current_screen(screen, menu, mode_sel, yn, edit_val3, edit_val2);

                commit_and_save_if_changed(screen, menu, mode_sel, yn, edit_val3, edit_val2);

                menu = 14u;
                cursor3 = 0u;

                edit_val3 = (uint16_t)(working_settings.lv_cut % 1000u);
                ui_edit3_draw_once("I/P LV Cut Voltage", edit_val3, 'V');
                ui_edit_cursor(cursor3);
                continue;
            }

            /* Menu 14 */
            if (menu == 14u && screen == SCR_EDIT3)
            {
//                commit_current_screen(screen, menu, mode_sel, yn, edit_val3, edit_val2);
                commit_and_save_if_changed(screen, menu, mode_sel, yn, edit_val3, edit_val2);


                menu = 15u;
                cursor3 = 0u;

                edit_val3 = (uint16_t)(working_settings.hv_cut % 1000u);
                ui_edit3_draw_once("I/P HV Cut Voltage", edit_val3, 'V');
                ui_edit_cursor(cursor3);
                continue;
            }

            /* Menu 15 */
            if (menu == 15u && screen == SCR_EDIT3)
            {
//                commit_current_screen(screen, menu, mode_sel, yn, edit_val3, edit_val2);
                commit_and_save_if_changed(screen, menu, mode_sel, yn, edit_val3, edit_val2);


                menu = 16u;
                screen = SCR_EDIT2;
                cursor2 = 0u;

                int32_t al = working_settings.adjust_load;
                if (al < 0) al = 0;
                if (al > 99) al = 99;
                edit_val2 = (uint8_t)al;

                ui_edit2_draw_once("Overload Current", edit_val2, 'A');
                ui_edit_cursor(cursor2);
                continue;
            }

            /* Menu 16 */
            if (menu == 16u && screen == SCR_EDIT2)
            {
//                commit_current_screen(screen, menu, mode_sel, yn, edit_val3, edit_val2);
                commit_and_save_if_changed(screen, menu, mode_sel, yn, edit_val3, edit_val2);


                if (working_settings.commericial_set == 'Y') {
                    menu = 17u;
                    screen = SCR_YN;
                    yn = (working_settings.ph_seq_cut == 'Y') ? 1u : 0u;

                    ui_select_draw_once("Phase Sequence", 3);
                    ui_select_update_value(yn ? "Yes" : "No");
                } else {
                    menu = 18u;
                    screen = SCR_EDIT3;
                    cursor3 = 0u;

                    edit_val3 = (uint16_t)(working_settings.password_1 % 1000u);
                    ui_edit3_draw_once("Password", edit_val3, 0);
                    ui_edit_cursor(cursor3);
                }
                continue;
            }

            /* Menu 17 */
            if (menu == 17u && screen == SCR_YN)
            {
//                commit_current_screen(screen, menu, mode_sel, yn, edit_val3, edit_val2);
                commit_and_save_if_changed(screen, menu, mode_sel, yn, edit_val3, edit_val2);


                menu = 18u;
                screen = SCR_EDIT3;
                cursor3 = 0u;

                edit_val3 = (uint16_t)(working_settings.password_1 % 1000u);
                ui_edit3_draw_once("Password", edit_val3, 0);
                ui_edit_cursor(cursor3);
                continue;
            }

            /* Menu 18 */
            if (menu == 18u && screen == SCR_EDIT3)
            {
//                commit_current_screen(screen, menu, mode_sel, yn, edit_val3, edit_val2);
                commit_and_save_if_changed(screen, menu, mode_sel, yn, edit_val3, edit_val2);


                menu = 19u;
                screen = SCR_RESETMODE;
                yn = (working_settings.Restart_Mode == 'Y') ? 1u : 0u;

                ui_select_draw_once("Restart Mode", 6);
                ui_select_update_value(yn ? "Auto" : "Manual");
                continue;
            }

            /* Menu 19 */
            if (menu == 19u && screen == SCR_RESETMODE)
            {
//                commit_current_screen(screen, menu, mode_sel, yn, edit_val3, edit_val2);
                commit_and_save_if_changed(screen, menu, mode_sel, yn, edit_val3, edit_val2);


                menu = 20u;
                screen = SCR_YN;
                yn = 0u;

                ui_select_draw_once("Discard Changes", 3);
                ui_select_update_value("No");
                continue;
            }

            /* Menu 20 Discard Changes */
            if (menu == 20u && screen == SCR_YN)
            {
                if (yn == 0u) {
                    // NO = keep whatever is currently set (already saved on each commit) and exit
                    LCD_Cursor(0,0);
                    return;
                } else {
                    // YES = restore baseline (before entering menu) and save it to Flash
                    restore_baseline_and_save();

                    // then go to Factory Reset confirmation
                    menu = 21u;
                    yn = 0u;

                    ui_select_draw_once("Factory Reset", 3);
                    ui_select_update_value("No");
                }
                continue;
            }

            /* Menu 21 Factory Reset */
            if (menu == 21u && screen == SCR_YN)
            {
                if (yn == 1u) {
                    working_settings = Default_Settings;
                    working_settings.valid_flag = SETTINGS_MAGIC;
                    Save_Settings_To_Flash();
                }
                LCD_Cursor(0,0);
                return;
            }
        }

        HAL_Delay(10);
    }

}

/* ---------------- Welcome message (unchanged) ---------------- */

void welcome_message(void)
{
    // ✅ Force entry from RUN mode (PB10+PB12 hold 5 sec detected in master main.c)
    if (g_force_settings_menu)
    {
        g_force_settings_menu = 0;   // consume request

        LCD_Clear();
        LCD_SetCursor(1,0);
        LCD_Print(" Menu Loading...");
        HAL_Delay(1000);

        buttons_sync_state_to_now();

        if (menu_password_gate()) {
            buttons_sync_state_to_now();
            menu_settings();
        }
        return;
    }

    // ----------- Normal welcome animation (startup) -----------
    for (int i = 0; i < 3; i++)
    {
        // Startup combo still works (PB10+PB12) if user presses quickly on boot
        if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) &&
            (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET))
        {
            HAL_Delay(50);
            if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET) &&
                (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET))
            {
                LCD_Clear();
                LCD_SetCursor(1,0);
                LCD_Print(" Menu Loading...");
                HAL_Delay(1000);

                buttons_sync_state_to_now();

                if (menu_password_gate()) {
                    buttons_sync_state_to_now();
                    menu_settings();
                }
                return;
            }
        }

        lcd_print_center(0, "Welcome");
        lcd_print_center(1, "Static Voltage");

        if (working_settings.pcb_id == 0)
            lcd_print_center(2, "Single Phase Mode");
        else
            lcd_print_center(2, "Three Phase Mode");

        if (i == 0) lcd_print_center(3, "Wait.");
        if (i == 1) lcd_print_center(3, "Wait..");
        if (i == 2) lcd_print_center(3, "Wait...");

        HAL_Delay(1000);
    }
}
