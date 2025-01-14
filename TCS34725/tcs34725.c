/*
 * tcs34725.c
 *
 *  Created on: Jan 14, 2025
 *      Author: Laptop
 */
#include "main.h"
#include "tcs34725.h"
extern I2C_HandleTypeDef hi2c1;  // Używany interfejs I2C

// Funkcja zapisu do rejestru TCS34725
void TCS34725_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg | 0x80, value};  // Ustawienie bitu CMD (0x80)
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_I2C_ADDRESS, data, 2, HAL_MAX_DELAY);
}

// Funkcja odczytu danych z rejestru TCS34725
uint16_t TCS34725_ReadRegister(uint8_t reg) {
    uint8_t data[2] = {0};
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_I2C_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_I2C_ADDRESS, data, 2, HAL_MAX_DELAY);
    return (data[1] << 8) | data[0];
}

// Funkcja inicjalizująca czujnik TCS34725
void TCS34725_Init(void) {
    TCS34725_WriteRegister(TCS34725_ENABLE_REG, TCS34725_ENABLE_PON);
    HAL_Delay(3);
    TCS34725_WriteRegister(TCS34725_ENABLE_REG, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
    TCS34725_WriteRegister(TCS34725_RGBC_TIMING, 0x00);
    TCS34725_WriteRegister(TCS34725_CONTROL, 0x01);
}

// Funkcja do rozpoznawania koloru
uint8_t TCS34725_GetColor(void) {
    uint16_t red, green, blue, clear;
    TCS34725_GetRawData(&red, &green, &blue, &clear);

    if (clear == 0) return COLOR_UNKNOWN;
    float r_ratio = (float)red / clear;
    float g_ratio = (float)green / clear;
    float b_ratio = (float)blue / clear;

    if (r_ratio > 0.4 && g_ratio < 0.3 && b_ratio < 0.3) return COLOR_RED;
    if (g_ratio > 0.4 && r_ratio < 0.3 && b_ratio < 0.3) return COLOR_GREEN;
    if (b_ratio > 0.4 && r_ratio < 0.3 && g_ratio < 0.3) return COLOR_BLUE;
    if (r_ratio > 0.3 && g_ratio > 0.3 && b_ratio < 0.2) return COLOR_YELLOW;
    if (r_ratio > 0.3 && g_ratio > 0.2 && b_ratio > 0.2) return COLOR_ORANGE;
    if (r_ratio > 0.2 && g_ratio < 0.3 && b_ratio > 0.3) return COLOR_PURPLE;
    if (r_ratio > 0.3 && g_ratio > 0.3 && b_ratio > 0.3) return COLOR_WHITE;
    if (r_ratio < 0.2 && g_ratio < 0.2 && b_ratio < 0.2) return COLOR_BLACK;

    return COLOR_UNKNOWN;
}

// Funkcja odczytująca surowe dane kolorów
void TCS34725_GetRawData(uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear) {
    *clear = TCS34725_ReadRegister(TCS34725_CDATAL);
    *red   = TCS34725_ReadRegister(TCS34725_RDATAL);
    *green = TCS34725_ReadRegister(TCS34725_GDATAL);
    *blue  = TCS34725_ReadRegister(TCS34725_BDATAL);
}

