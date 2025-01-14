/*
 * servo.c
 *
 *  Created on: Jan 14, 2025
 *      Author: Laptop
 */

#include "servo.h"

extern TIM_HandleTypeDef htim2;

void SetServoAngle(uint8_t angle) {
    uint16_t pulse_length = 500 + ((angle * 2000) / 180);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_length);
}

void activate_servo(void) {
    SetServoAngle(90);
    HAL_Delay(500);
    SetServoAngle(0);
}

