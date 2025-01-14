/*
 * stepper_nema.h
 *
 *  Created on: Jan 14, 2025
 *      Author: Laptop
 */

#ifndef INC_STEPPER_NEMA_H_
#define INC_STEPPER_NEMA_H_

#include "stm32f7xx_hal.h"

#define STEP_A1 GPIO_PIN_6   // PA6 -> IN1
#define STEP_A2 GPIO_PIN_7   // PA7 -> IN2
#define STEP_B1 GPIO_PIN_8   // PA8 -> IN3
#define STEP_B2 GPIO_PIN_9   // PA9 -> IN4

#define GPIO_PORT GPIOA  // Wszystkie piny są na porcie GPIOA

void Stepper_MoveToStep(uint8_t position);

#endif /* INC_STEPPER_NEMA_H_ */
