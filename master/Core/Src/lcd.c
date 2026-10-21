#include "lcd.h"

#define RS_HIGH() GPIOB->BSRR = LCD_RS_PIN
#define RS_LOW()  GPIOB->BSRR = (LCD_RS_PIN<<16)

#define EN_HIGH() GPIOB->BSRR = LCD_EN_PIN
#define EN_LOW()  GPIOB->BSRR = (LCD_EN_PIN<<16)

#define D4(x) ((x)?(GPIOD->BSRR=LCD_D4_PIN):(GPIOD->BSRR=(LCD_D4_PIN<<16)))
#define D5(x) ((x)?(GPIOD->BSRR=LCD_D5_PIN):(GPIOD->BSRR=(LCD_D5_PIN<<16)))
#define D6(x) ((x)?(GPIOD->BSRR=LCD_D6_PIN):(GPIOD->BSRR=(LCD_D6_PIN<<16)))
#define D7(x) ((x)?(GPIOD->BSRR=LCD_D7_PIN):(GPIOD->BSRR=(LCD_D7_PIN<<16)))

static void lcd_delay(void)
{
    for(volatile int i=0;i<1500;i++);
}

static void LCD_Enable(void)
{
    EN_HIGH();
    lcd_delay();
    EN_LOW();
    lcd_delay();
}

static void LCD_Send4Bits(uint8_t d)
{
    D4(d & 1);
    D5(d & 2);
    D6(d & 4);
    D7(d & 8);
}

static void LCD_Cmd_Internal(uint8_t cmd)
{
    RS_LOW();
    LCD_Send4Bits(cmd >> 4);
    LCD_Enable();
    LCD_Send4Bits(cmd);
    LCD_Enable();
}

static void LCD_Data(uint8_t d)
{
    RS_HIGH();
    LCD_Send4Bits(d >> 4);
    LCD_Enable();
    LCD_Send4Bits(d);
    LCD_Enable();
}

/* V5: exposed raw command */
void LCD_Command(uint8_t cmd)
{
    LCD_Cmd_Internal(cmd);
}

/* V5: cursor control (uses HD44780 display control commands) */
void LCD_Cursor(uint8_t on, uint8_t blink)
{
    if(!on) {
        LCD_Cmd_Internal(0x0C); // display on, cursor off, blink off
    } else if(blink) {
        LCD_Cmd_Internal(0x0F); // display on, cursor on, blink on
    } else {
        LCD_Cmd_Internal(0x0E); // display on, cursor on, blink off
    }
}

void LCD_Init(void)
{
    HAL_Delay(40);

    LCD_Send4Bits(0x03); LCD_Enable();
    HAL_Delay(5);

    LCD_Send4Bits(0x03); LCD_Enable();
    HAL_Delay(1);

    LCD_Send4Bits(0x03); LCD_Enable();

    LCD_Send4Bits(0x02); LCD_Enable();     // 4-bit mode

    LCD_Cmd_Internal(0x28);  // 4-bit, 2-line, 5x8
    LCD_Cmd_Internal(0x0C);  // display on, cursor off
    LCD_Cmd_Internal(0x06);  // entry mode
    LCD_Clear();
}

void LCD_Clear(void)
{
    LCD_Cmd_Internal(0x01);
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t a[] = {0x80,0xC0,0x94,0xD4};
    LCD_Cmd_Internal((uint8_t)(a[row] + col));
}

void LCD_Print(const char *s)
{
    while(*s) LCD_Data((uint8_t)*s++);
}
