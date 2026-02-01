#ifndef SHT_31_H
#define SHT_31_H

#include <stdint.h>
#include <cmsis_os.h>
#include "stm32f1xx_hal.h"
#include "main.h"

#define PIN_SHT31_SCL   				GPIO_PIN_6
#define PIN_SHT31_SDA						GPIO_PIN_7
#define SHT31_PORT							GPIOB	
#define PIN_I2C_FREEQ_HZ				400000
#define SHT31_ADDR              0x44

//----------------------SHT31 COMMAND--------------------
#define CMD_SOFT_RESET          0x30A2  
#define CMD_CLEAR_STATUS        0x3041
// Mode single shot (Clock stretching disable)
#define CMD_SINGLE_HIGH         0x2400
#define CMD_SINGLE_MEDIUM       0x240B
#define CMD_SINGLE_LOW          0x2416

extern I2C_HandleTypeDef hi2c1;

uint8_t sht31_crc8(const uint8_t *data, int len); 
uint8_t sht31_write_cmd(uint16_t cmd); 
uint8_t sht31_read_raw( uint16_t *raw_t,uint16_t *raw_h); 
void sht31_init(); 
uint8_t sht31_single_shot( float *temp , float *humi); 

#endif
