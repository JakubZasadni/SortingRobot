/*
 * stepper_nema.c
 *
 *  Created on: Jan 14, 2025
 *      Author: Laptop
 */


#include "stepper_nema.h"

extern TIM_HandleTypeDef htim4;

uint8_t step_sequence2[4][4] = {
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 1},
    {1, 0, 0, 1}
};
volatile uint8_t step_index2 = 0;
volatile uint16_t step_count = 0;
volatile uint8_t current_position = 0;
volatile uint8_t target_position = 0;

void Stepper_MoveToStep(uint8_t position) {
    target_position = position;
    step_count = abs(target_position - current_position) * 25;
    HAL_TIM_Base_Start_IT(&htim4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {
        if (step_count > 0) {
            HAL_GPIO_WritePin(GPIO_PORT, STEP_A1, step_sequence2[step_index2][0]);
            HAL_GPIO_WritePin(GPIO_PORT, STEP_A2, step_sequence2[step_index2][1]);
            HAL_GPIO_WritePin(GPIO_PORT, STEP_B1, step_sequence2[step_index2][2]);
            HAL_GPIO_WritePin(GPIO_PORT, STEP_B2, step_sequence2[step_index2][3]);

            step_index2 = (step_index2 + 1) % 4;
            step_count--;
        } else {
            HAL_TIM_Base_Stop_IT(&htim4);
            current_position = target_position;
        }
    }
}
