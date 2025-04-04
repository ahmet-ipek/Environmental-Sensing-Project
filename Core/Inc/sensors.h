/*
 * sensors.h
 *
 *  Created on: Mar 25, 2025
 *      Author: ahmet
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "stm32f4xx_hal.h"

#define LM75A_ADDR        0x48
#define SI7021_ADDR       0x40
#define LPS25HB_ADDR      0x5C

// LM75A (Temperature)
HAL_StatusTypeDef LM75A_Init(I2C_HandleTypeDef *hi2c);
float LM75A_ReadTemperature(I2C_HandleTypeDef *hi2c);

// Si7021 (Humidity)
HAL_StatusTypeDef Si7021_Init(I2C_HandleTypeDef *hi2c);
float Si7021_ReadHumidity(I2C_HandleTypeDef *hi2c);

// LPS25HB (Pressure)
HAL_StatusTypeDef LPS25HB_Init(I2C_HandleTypeDef *hi2c);
float LPS25HB_ReadPressure(I2C_HandleTypeDef *hi2c); // read pressure

#endif /* INC_SENSORS_H_ */
