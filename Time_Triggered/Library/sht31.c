#include "sht31.h"

#define CMD_MEASURE_HIGH_NO_STRETCH 0x2400
#define CMD_SOFT_RESET              0x30A2
#define CMD_HEATER_ON               0x306D
#define CMD_HEATER_OFF              0x3066

static uint8_t calc_crc(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (size_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0x31;
            else crc <<= 1;
        }
    }
    return crc;
}

static bool send_cmd(sht31_t *dev, uint16_t cmd) {
    uint8_t buf[2] = {cmd >> 8, cmd & 0xFF};
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->addr << 1, buf, 2, 100) == HAL_OK;
}

void sht31_init(sht31_t *dev, I2C_HandleTypeDef *hi2c, uint16_t addr) {
    dev->hi2c = hi2c;
    dev->addr = addr;
    sht31_reset(dev);
    HAL_Delay(2);
}

bool sht31_reset(sht31_t *dev) {
    return send_cmd(dev, CMD_SOFT_RESET);
}

bool sht31_heater(sht31_t *dev, bool enable) {
    return send_cmd(dev, enable ? CMD_HEATER_ON : CMD_HEATER_OFF);
}

bool sht31_read(sht31_t *dev, float *temp, float *hum) {
    if (!send_cmd(dev, CMD_MEASURE_HIGH_NO_STRETCH)) return false;

    HAL_Delay(20);

    uint8_t buf[6];
    if (HAL_I2C_Master_Receive(dev->hi2c, dev->addr << 1, buf, 6, 100) != HAL_OK) return false;

    if (calc_crc(buf, 2) != buf[2] || calc_crc(buf + 3, 2) != buf[5]) return false;

    uint16_t t_raw = (buf[0] << 8) | buf[1];
    uint16_t h_raw = (buf[3] << 8) | buf[4];

    *temp = -45.0f + 175.0f * ((float)t_raw / 65535.0f);
    *hum = 100.0f * ((float)h_raw / 65535.0f);

    if (*hum > 100.0f) *hum = 100.0f;
    if (*hum < 0.0f) *hum = 0.0f;

    return true;
}