#ifndef __LCD_H
#define __LCD_H

#include "main.h"

/* ===== PIN DEFINITIONS ===== */
#define LCD_RS_PIN (1<<4) // PB4
#define LCD_EN_PIN (1<<5) // PB5
#define LCD_D4_PIN (1<<0) // PD0
#define LCD_D5_PIN (1<<1) // PD1
#define LCD_D6_PIN (1<<2) // PD2
#define LCD_D7_PIN (1<<3) // PD3

void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(const char *str);

/* V5 additions: allow UI to control cursor/blink and send raw commands */
void LCD_Command(uint8_t cmd);
void LCD_Cursor(uint8_t on, uint8_t blink);

#endif
