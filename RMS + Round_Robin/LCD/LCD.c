#include "LCD.h"
//----------------------------------Lcd16x2-------------------------//
								// delay us 
void delay_us_approx(uint32_t us) {
    uint32_t count = us * 7; 
    while(count--) {
        __NOP(); 
    }
}
                //khoi tai Enable cho LCD 
void LCD_Enable(){
     HAL_GPIO_WritePin(LCD_PORT,EN_PIN,GPIO_PIN_SET);
	   delay_us_approx(150);
		 HAL_GPIO_WritePin(LCD_PORT,EN_PIN,GPIO_PIN_RESET);
		 delay_us_approx(150);
}
                // LCD sen 4 bit 
void LCD_Send4bit(uint8_t data){
     HAL_GPIO_WritePin(LCD_PORT,D4_PIN,data&0x01);
     HAL_GPIO_WritePin(LCD_PORT,D5_PIN,(data>>1)&1);
     HAL_GPIO_WritePin(LCD_PORT,D6_PIN,(data>>2)&1);
     HAL_GPIO_WritePin(LCD_PORT,D7_PIN,(data>>3)&1);
	   LCD_Enable(); 
}
void LCD_SendCommand(uint8_t cmd){
		 HAL_GPIO_WritePin(LCD_PORT,RS_PIN,GPIO_PIN_RESET); 
	   LCD_Send4bit((cmd>>4));
	   LCD_Send4bit(cmd&0x0F);
}
               //Ham khoi tao 
void LCD_Init(){
		 osDelay(50);
		 LCD_SendCommand(0x33);
		 osDelay(5);
	   LCD_SendCommand(0x32);
		 osDelay(5);
		 LCD_SendCommand(0x28);
		 osDelay(5);
		 LCD_SendCommand(0x0c);
		 osDelay(5);
		 LCD_SendCommand(0x01);
}

void LCD_Gotoxy(unsigned char x, unsigned char y){
        unsigned char address;
        if(!y)address=(0x80+x);
        else address=(0xc0+x);
        osDelay(1);
        LCD_SendCommand(address);
        osDelay(1);
}
void LCD_PutChar(unsigned char Data){
		 HAL_GPIO_WritePin(LCD_PORT,RS_PIN,GPIO_PIN_SET); 
	   LCD_Send4bit((Data>>4));
	   LCD_Send4bit(Data&0x0F);
}
void LCD_Puts(char *s){
        while (*s){
                LCD_PutChar(*s);
                s++;
        }
}
void LCD_PutNumber(int number)
{
	if(number <= 9)
	{
		LCD_PutChar(number + 48);
	}
	else if(number > 9 && number < 100)
	{
		LCD_PutChar(number/10 + 48);
		LCD_PutChar(number%10 + 48);
	}
	else if(number > 99 && number < 1000)
	{
		LCD_PutChar(number/100 + 48);
		LCD_PutChar(number/10%10 + 48);
		LCD_PutChar(number%10 + 48);
	}
	else if(number > 999 && number < 10000)
	{
		LCD_PutChar(number/1000 + 48);
		LCD_PutChar(number/100%10 + 48);
		LCD_PutChar(number/10%10 + 48);
		LCD_PutChar(number%10 + 48);
	}
	
}