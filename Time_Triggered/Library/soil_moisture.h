#ifndef SOIL_MOISTURE_H
#define SOIL_MOISTURE_H

#include "main.h" // T? d?ng include dúng dòng chip (f1, f4...)

#define SM_DEFAULT_AIR   3000 // Giá tr? khô
#define SM_DEFAULT_WATER 1160 // Giá tr? u?t
#define SM_SAMPLES       10   // S? l?n l?y m?u

typedef struct {
    ADC_HandleTypeDef *hadc;
    uint32_t air;
    uint32_t water;
    float    inverse_range;   // T?i uu: Luu 1/(Air - Water) d? nhân thay vì chia
    uint16_t raw;             // Giá tr? thô
    float    percent;         // K?t qu? %
} SoilSensor;

// Kh?i t?o
void Soil_Init(SoilSensor *sensor, ADC_HandleTypeDef *hadc, uint32_t air, uint32_t water);

// Ð?c c?m bi?n (Blocking t?i thi?u)
void Soil_Read(SoilSensor *sensor);

#endif