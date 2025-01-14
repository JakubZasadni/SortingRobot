/*
 * ir_sensor.c
 *
 *  Created on: Jan 14, 2025
 *      Author: Laptop
 */

#include "ir_sensor.h"

void IR_Sensor_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = IR_SENSOR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(IR_SENSOR_PORT, &GPIO_InitStruct);
}

uint8_t IR_Sensor_Detected(void) {
    return HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN) == GPIO_PIN_SET;
}

