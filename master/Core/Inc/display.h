#ifndef DISPLAY_H
#define DISPLAY_H

#include "main.h"

#define SETTINGS_MAGIC 0xA5A5A5A5
#define FLASH_SETTINGS_ADDR 0x0801F800 // corresponds to 128KB
extern volatile uint8_t g_force_settings_menu;

void welcome_message(void);
void Load_Settings_From_Flash(void);
void menu_settings(void);

typedef struct
{
    // Calibrate settings
    int16_t input_v_r;
    int16_t input_v_y;
    int16_t input_v_b;
    int16_t output_v_r;
    int16_t output_v_y;
    int16_t output_v_b;

    float   adc_offset;
    int32_t adjust_load;

    // Parameters
    uint32_t ac_output_voltage;
    uint32_t hv_cut;
    uint32_t lv_cut;
    uint32_t password_1;
    uint32_t password_2;
    uint32_t pcb_id;
    uint32_t kva;

    // Options
    uint8_t  ph_seq_cut;

    /* Menu 19: Reset Mode (Auto/Manual)
       'Y' = Auto (Default)
       'N' = Manual
    */
    uint8_t  Restart_Mode;

    // uint8_t avr_speed;
    uint8_t  commericial_set;

    // Version or flag for integrity
    uint32_t valid_flag;

    float Nbb_tf;
} Settings_t;

extern Settings_t working_settings;
extern Settings_t saved_settings;

#endif
