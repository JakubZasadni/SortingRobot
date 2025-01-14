/*
 * ir_sensor.h
 *
 *  Created on: Jan 14, 2025
 *      Author: Laptop
 */

#ifndef INC_IR_SENSOR_H_
#define INC_IR_SENSOR_H_

#include "stm32f7xx_hal.h"

#define IR_SENSOR_PIN GPIO_PIN_0
#define IR_SENSOR_PORT GPIOA

void IR_Sensor_Init(void);
uint8_t IR_Sensor_Detected(void);

#endif // IR_SENSOR_H

