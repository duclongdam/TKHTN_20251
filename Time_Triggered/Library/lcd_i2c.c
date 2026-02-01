#include "lcd_i2c.h"

extern I2C_HandleTypeDef hi2c2;
void LCD_Internal_Transmit(uint8_t data) {
    HAL_I2C_Master_Transmit(&hi2c2, LCD_ADDR, &data, 1, 100);
}

void LCD_Write_Nibble(uint8_t nibble, uint8_t rs_bit) {   
    uint8_t data = nibble | rs_bit | BACKLIGHT;
 
    LCD_Internal_Transmit(data | PIN_EN); 
    HAL_Delay(1); 
    LCD_Internal_Transmit(data & ~PIN_EN);
    HAL_Delay(1);
}

void LCD_Send_Byte(uint8_t val, uint8_t rs_bit) {
    LCD_Write_Nibble(val & 0xF0, rs_bit);
    LCD_Write_Nibble((val << 4) & 0xF0, rs_bit);
}

void LCD_Send_Cmd(uint8_t cmd) {
    LCD_Send_Byte(cmd, 0);
}

void LCD_Send_Data(uint8_t data) {
    LCD_Send_Byte(data, PIN_RS);
}

void LCD_Init(void) {
		//1. Wait > 40ms
    HAL_Delay(50);
	
		//2. Function Set: 8-bit Mode
    LCD_Write_Nibble(0x30, 0);
    HAL_Delay(5);
    LCD_Write_Nibble(0x30, 0);
    HAL_Delay(1);
    LCD_Write_Nibble(0x30, 0);
    HAL_Delay(10);
	
		//3. Function Set: 4-bit Mode
    LCD_Write_Nibble(0x20, 0);
    HAL_Delay(10);
		
		//4. 
    LCD_Send_Cmd(0x28); //Configure number of line (2), font size (5x8)
    LCD_Send_Cmd(0x0C); //Display ON
    LCD_Send_Cmd(0x01);	//Display clear
    LCD_Send_Cmd(0x06); //Entry mode set: cursor move right, display shift stop
    HAL_Delay(2);
}

void LCD_Send_String(char *str) {
    while (*str) LCD_Send_Data(*str++);
}

void LCD_Put_Cur(int row, int col) {
    uint8_t pos = (row == 0) ? (0x80 + col) : (0xC0 + col);
    LCD_Send_Cmd(pos);
}

void LCD_Clear(void){
		LCD_Send_Cmd(0x01);
}
