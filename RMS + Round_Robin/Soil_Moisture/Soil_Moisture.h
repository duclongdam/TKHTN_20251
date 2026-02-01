#ifndef SOIL_MOISTURE_H
#define SOIL_MOISTURE_H

#include "stm32f1xx_hal.h"

#define ADC_VAL_DRY			3000
#define ADC_VAL_WET			1160

extern ADC_HandleTypeDef hadc1;

uint16_t Read_Raw_SM(); 
uint8_t Read_Percen_SM(); 


#endif