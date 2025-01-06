#define IN1 GPIO_PIN_2  // Podłączony do IN1 ULN2003
#define IN2 GPIO_PIN_3  // Podłączony do IN2 ULN2003
#define IN3 GPIO_PIN_4  // Podłączony do IN3 ULN2003
#define IN4 GPIO_PIN_5  // Podłączony do IN4 ULN2003
#define STEP_GPIO GPIOA       // Port GPIOA

volatile uint8_t step_index = 0;  // Indeks bieżącego kroku


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

/*
uint8_t step_sequence[4][4] = {
    {1, 0, 0, 1},
    {1, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 1}
};
*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {  // Sprawdzenie, czy przerwanie pochodzi od TIM3
        // Sterowanie silnikiem krokowym
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, step_sequence[step_index][0]);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, step_sequence[step_index][1]);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, step_sequence[step_index][2]);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, step_sequence[step_index][3]);

        step_index = (step_index + 1) % 8;  // Zwiększ indeks kroku
    }
}
