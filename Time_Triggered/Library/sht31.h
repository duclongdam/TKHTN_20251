#ifndef SHT31_H
#define SHT31_H

#include "main.h"
#include <stdbool.h>

#define SHT31_ADDR_LO 0x44
#define SHT31_ADDR_HI 0x45

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t addr;
} sht31_t;

void sht31_init(sht31_t *dev, I2C_HandleTypeDef *hi2c, uint16_t addr);
bool sht31_read(sht31_t *dev, float *temp, float *hum);
bool sht31_heater(sht31_t *dev, bool enable);
bool sht31_reset(sht31_t *dev);

#endif