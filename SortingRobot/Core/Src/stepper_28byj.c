/*
 * stepper_28byj.c
 *
 *  Created on: Jan 14, 2025
 *      Author: Laptop
 */

#include "stepper_28byj.h"

extern TIM_HandleTypeDef htim3;

uint8_t step_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};
volatile uint8_t step_index = 0;

void HAL_TIM_PeriodElapsedCallback_TIM3(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        HAL_GPIO_WritePin(STEP_GPIO, IN1, step_sequence[step_index][0]);
        HAL_GPIO_WritePin(STEP_GPIO, IN2, step_sequence[step_index][1]);
        HAL_GPIO_WritePin(STEP_GPIO, IN3, step_sequence[step_index][2]);
        HAL_GPIO_WritePin(STEP_GPIO, IN4, step_sequence[step_index][3]);

        step_index = (step_index + 1) % 8;
    }
}

