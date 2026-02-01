#include "soil_moisture.h"

void Soil_Init(SoilSensor *sensor, ADC_HandleTypeDef *hadc, uint32_t air, uint32_t water) {
    if (!sensor) return;

    sensor->hadc = hadc;
    sensor->air   = (air == 0)   ? SM_DEFAULT_AIR   : air;
    sensor->water = (water == 0) ? SM_DEFAULT_WATER : water;
    sensor->raw = 0;
    sensor->percent = 0.0f;

    // T?i uu: Tính tru?c h? s? nhân d? tránh phép chia trong vòng l?p chính
    // Công th?c g?c: (Air - Raw) / (Air - Water)
    // T?i uu: (Air - Raw) * inverse_range
    float range = (float)(sensor->air - sensor->water);
    if (range == 0) range = 1.0f; // Tránh chia cho 0
    sensor->inverse_range = 100.0f / range;
}

void Soil_Read(SoilSensor *sensor) {
    if (!sensor || !sensor->hadc) return;

    uint32_t sum = 0;
    
    // L?y m?u liên t?c (Burst read) - Không dùng Delay d? tránh ch?n CPU
    for (int i = 0; i < SM_SAMPLES; i++) {
        HAL_ADC_Start(sensor->hadc);
        if (HAL_ADC_PollForConversion(sensor->hadc, 2) == HAL_OK) { // Timeout th?p nh?t có th?
            sum += HAL_ADC_GetValue(sensor->hadc);
        }
    }
    
    uint16_t raw = (uint16_t)(sum / SM_SAMPLES);
    sensor->raw = raw;

    // Tính toán %
    if (raw <= sensor->water) {
        sensor->percent = 100.0f;
    } else if (raw >= sensor->air) {
        sensor->percent = 0.0f;
    } else {
        // T?i uu: S? d?ng phép nhân thay vì phép chia
        sensor->percent = (float)(sensor->air - raw) * sensor->inverse_range;
    }
}