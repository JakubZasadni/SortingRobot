/*
 * tcs34725.h
 *
 *  Created on: Jan 14, 2025
 *      Author: Laptop
 */

#ifndef INC_TCS34725_H_
#define INC_TCS34725_H_

#include "stm32f7xx_hal.h"

// Definicje dla czujnika TCS34725
#define TCS34725_I2C_ADDRESS (0x29 << 1)
#define TCS34725_ENABLE_REG  0x00
#define TCS34725_RGBC_TIMING 0x01
#define TCS34725_CONTROL     0x0F
#define TCS34725_CDATAL      0x14
#define TCS34725_RDATAL      0x16
#define TCS34725_GDATAL      0x18
#define TCS34725_BDATAL      0x1A
#define TCS34725_ENABLE_PON  0x01
#define TCS34725_ENABLE_AEN  0x02

// Definicje kolorów
#define COLOR_UNKNOWN 0
#define COLOR_RED     1
#define COLOR_GREEN   2
#define COLOR_BLUE    3
#define COLOR_YELLOW  4
#define COLOR_ORANGE  5
#define COLOR_PURPLE  6
#define COLOR_WHITE   7
#define COLOR_BLACK   8

void TCS34725_Init(void);
uint8_t TCS34725_GetColor(void);
void TCS34725_GetRawData(uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear);

#endif /* INC_TCS34725_H_ */
