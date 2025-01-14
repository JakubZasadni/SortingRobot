/*
 * servo.h
 *
 *  Created on: Jan 14, 2025
 *      Author: Laptop
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f7xx_hal.h"

void SetServoAngle(uint8_t angle);
void activate_servo(void);

#endif /* INC_SERVO_H_ */
