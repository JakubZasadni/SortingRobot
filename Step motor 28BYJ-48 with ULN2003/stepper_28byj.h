/*
 * stepper_28byj.h
 *
 *  Created on: Jan 14, 2025
 *      Author: Laptop
 */

#ifndef INC_STEPPER_28BYJ_H_
#define INC_STEPPER_28BYJ_H_

#include "stm32f7xx_hal.h"

#define IN1 GPIO_PIN_2  // Podłączony do IN1 ULN2003
#define IN2 GPIO_PIN_3  // Podłączony do IN2 ULN2003
#define IN3 GPIO_PIN_4  // Podłączony do IN3 ULN2003
#define IN4 GPIO_PIN_5  // Podłączony do IN4 ULN2003
#define STEP_GPIO GPIOA // Port GPIOA

#endif /* INC_STEPPER_28BYJ_H_ */
