#include "SHT31.h"
// check crc 
uint8_t sht31_crc8(const uint8_t *data, int len){
    uint8_t crc=0xFF; 
    for(int i=0;i<len;i++){
        crc^=data[i];
        for(int j=0; j<8; j++){
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc; 
}
// write command 
uint8_t sht31_write_cmd(uint16_t cmd){
	uint8_t buf[2]; 
	buf[0]=(uint8_t)(cmd>>8);
	buf[1]=(uint8_t)cmd; 
	return HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDR<<1,buf, 2, 100); 
}
// sht31 read raw 
uint8_t sht31_read_raw( uint16_t *raw_t,uint16_t *raw_h){
	uint8_t buf[6]; 
	uint8_t ret=HAL_I2C_Master_Receive(&hi2c1,SHT31_ADDR<<1,buf,6,100); 
	if(ret!=HAL_OK) return ret; 
	if (sht31_crc8(buf, 2) != buf[2] || sht31_crc8(&buf[3], 2) != buf[5]) return HAL_ERROR; 
    *raw_t = (buf[0] << 8) | buf[1];
    *raw_h = (buf[3] << 8) | buf[4];
   return HAL_OK;
}
// sht31 init
void sht31_init(){
	sht31_write_cmd(CMD_SOFT_RESET); 
	osDelay(10); 
}
// sht31 single shot 
uint8_t sht31_single_shot( float *temp , float *humi){
	uint16_t rt,rh; 
	sht31_write_cmd(CMD_SINGLE_HIGH); 
	osDelay(20); 
	uint8_t ret=sht31_read_raw(&rt,&rh); 
	if(ret!=HAL_OK){
		return 0; 
	}
	*temp= -45.0f+(175.0f)*rt/65535.0f;
  *humi= 100.0f * rh / 65535.0f; 
	return 1; 
}












