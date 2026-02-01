#ifndef _LCD_I2C_H
#define _LCD_I2C_H

#include "stm32f1xx_hal.h"

#define LCD_ADDR (0x27 << 1)

#define PIN_RS    (1 << 0)
#define PIN_RW    (1 << 1)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)


#define LCD_COMMAND_CLEAR_DISPLAY       0x01
#define LCD_COMMAND_RETURN_HOME         0x02
#define LCD_COMMAND_ENTRY_MODE_SET      0x04
#define LCD_COMMAND_DISPLAY_CONTROL     0x08
#define LCD_COMMAND_CURSOR_SHIFT        0x10
#define LCD_COMMAND_FUNCTION_SET        0x20
#define LCD_COMMAND_SET_CGRAM_ADDR      0x40
#define LCD_COMMAND_SET_DDRAM_ADDR      0x80

void LCD_Init(void);
void LCD_Send_Cmd(uint8_t cmd);
void LCD_Send_Data(uint8_t data);
void LCD_Send_String(char *str);
void LCD_Put_Cur(int row, int col);
void LCD_Clear(void);

#endif