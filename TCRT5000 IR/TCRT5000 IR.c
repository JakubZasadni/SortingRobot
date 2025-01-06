void IR_Sensor_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();  // Włącz zegar dla portu GPIOA

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6;  // Pin PA6 dla czujnika IR
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;  // Ustaw jako wejście
    GPIO_InitStruct.Pull = GPIO_PULLUP;  // Włącz pull-up (czujnik podaje 0, gdy wykryje obiekt)
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


/*
Konieczne beda zmiany w przerwaniu dla timera 3 oraz ponizej porzykladowe wywolanie:
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {  // Sprawdzenie, czy przerwanie pochodzi od TIM3
        if (motor_running) {  // Jeśli silnik ma się obracać
            // Sprawdź stan czujnika IR
            if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET) {
                motor_running = 0;  // Zatrzymaj silnik, jeśli wykryto obiekt
                return;
            }

            // Sterowanie cewkami silnika (sekwencja półkrokowa)
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, step_sequence[step_index][0]);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, step_sequence[step_index][1]);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, step_sequence[step_index][2]);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, step_sequence[step_index][3]);

            step_index = (step_index + 1) % 8;  // Przejdź do kolejnego kroku
        }
    }
}

while (1) {
    if (!motor_running) {  // Jeśli silnik jest zatrzymany
        HAL_Delay(1000);  // Odczekaj 1 sekundę
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) {  // Jeśli obiekt zniknął
            motor_running = 1;  // Uruchom ponownie silnik
            step_index = 0;  // Zresetuj indeks sekwencji kroków
            step_count = 0;  // Zresetuj licznik kroków (jeśli używasz licznika)
        }
    }
}
*/