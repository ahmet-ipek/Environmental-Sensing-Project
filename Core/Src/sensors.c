/*
 * sensors.c
 *
 *  Created on: Mar 25, 2025
 *      Author: ahmet
 */

#include "sensors.h"


// LM75A temperature Read

HAL_StatusTypeDef LM75A_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;
    if (HAL_I2C_Mem_Read(hi2c, LM75A_ADDR << 1, 0x00, 1, &data, 1, 100) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

float LM75A_ReadTemperature(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2];
    HAL_I2C_Mem_Read(hi2c, LM75A_ADDR << 1, 0x00, 1, data, 2, 100);

    // Combine bytes into 16-bit value
    int16_t raw_temp = (data[0] << 8) | data[1];

    // Extract 9-bit two's complement value
    int16_t temp_9bit = raw_temp >> 7; // Shift to align 9 bits

    // Handle negative temperatures (sign bit = bit 8)
    if (temp_9bit & 0x100) {
        // Compute two's complement for negative values
        temp_9bit = ~(temp_9bit | 0xFE00) + 1; // Mask and invert
        return temp_9bit * -0.5f;
    } else {
        return temp_9bit * 0.5f;
    }
}

// Si7021 Humidity Read

HAL_StatusTypeDef Si7021_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t cmd = 0xFA; // Read Electronic ID (1st byte)
    uint8_t id[2];
    HAL_I2C_Master_Transmit(hi2c, SI7021_ADDR << 1, &cmd, 1, 100);
    HAL_I2C_Master_Receive(hi2c, SI7021_ADDR << 1, id, 2, 100);

    // Check for valid ID (e.g., 0x06 for Si7021)
    if (id[0] != 0x06) return HAL_ERROR;
    return HAL_OK;
}

float Si7021_ReadHumidity(I2C_HandleTypeDef *hi2c) {
    uint8_t cmd = 0xE5; // Measure RH, Hold Master Mode
    uint8_t data[2];
    HAL_I2C_Master_Transmit(hi2c, SI7021_ADDR << 1, &cmd, 1, 100);
    HAL_Delay(20); // Wait for conversion
    HAL_I2C_Master_Receive(hi2c, SI7021_ADDR << 1, data, 2, 100);
    uint16_t raw_humidity = (data[0] << 8) | data[1];
    return ((125.0f * raw_humidity) / 65536.0f) - 6.0f;
}

// LPS25HB Initialization
HAL_StatusTypeDef LPS25HB_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t who_am_i;
    HAL_I2C_Mem_Read(hi2c, LPS25HB_ADDR << 1, 0x0F, 1, &who_am_i, 1, 100);
    if (who_am_i != 0xBD) return HAL_ERROR;

    // Set ODR to 1 Hz (CTRL_REG1)
    uint8_t ctrl_reg1 = 0b10000000; // PD=1, ODR=1 Hz
    HAL_I2C_Mem_Write(hi2c, LPS25HB_ADDR << 1, 0x20, 1, &ctrl_reg1, 1, 100);
    return HAL_OK;
}

// LPS25HB Pressure Read
float LPS25HB_ReadPressure(I2C_HandleTypeDef *hi2c) {
    uint8_t data[3];
    HAL_I2C_Mem_Read(hi2c, LPS25HB_ADDR << 1, 0x28, 1, data, 3, 100);
    int32_t raw_pressure = (data[2] << 16) | (data[1] << 8) | data[0];
    return raw_pressure / 4096.0f; // Convert to hPa
}
