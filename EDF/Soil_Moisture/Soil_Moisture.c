#include "Soil_Moisture.h" 

// Read raw value 
uint16_t Read_Raw_SM(){
	HAL_ADC_Start(&hadc1); 
	
	if(HAL_ADC_PollForConversion(&hadc1,10)==HAL_OK){
		uint16_t val = HAL_ADC_GetValue(&hadc1); 
		HAL_ADC_Stop(&hadc1); 
		return val; 
	}
	HAL_ADC_Stop(&hadc1); 
  return 0; 
}
// Read percentage
uint8_t Read_Percen_SM(){
	uint16_t raw=Read_Raw_SM(); 
	
	if(raw>ADC_VAL_DRY) raw=ADC_VAL_DRY; 
	if(raw<ADC_VAL_WET) raw=ADC_VAL_WET; 
	uint32_t percent = 100 - ((uint32_t)(raw - ADC_VAL_WET) * 100) / (ADC_VAL_DRY - ADC_VAL_WET);
	return (uint8_t)percent; 
}