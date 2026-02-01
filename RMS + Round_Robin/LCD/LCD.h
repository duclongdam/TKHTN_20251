#ifndef lcd_h
#define lcd_h 

#include<stdint.h>
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"

#define RS_PIN 		GPIO_PIN_0 
#define EN_PIN 		GPIO_PIN_1 
#define D4_PIN 		GPIO_PIN_4
#define D5_PIN 		GPIO_PIN_5
#define D6_PIN 		GPIO_PIN_6
#define D7_PIN 		GPIO_PIN_7
#define LCD_PORT		 GPIOA 

void LCD_Enabe(); 
void LCD_Send4bit(uint8_t data); 
void LCD_SendCommand(uint8_t cmd); 
void LCD_Init();
void LCD_Gotoxy(unsigned char x, unsigned char y); 
void LCD_Putchar(uint8_t data);
void LCD_Puts(char *s);
void LCD_PutNumber(int x);

#endif 