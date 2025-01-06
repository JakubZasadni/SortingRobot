#define STEP_A1 GPIO_PIN_9   // PD9 -> OUT1
#define STEP_A2 GPIO_PIN_2   // PH2 -> OUT2
#define STEP_B1 GPIO_PIN_12  // PG12 -> OUT3
#define STEP_B2 GPIO_PIN_9   // PG9 -> OUT4

#define GPIO_PORT_A1 GPIOD  // Port dla STEP_A1
#define GPIO_PORT_A2 GPIOH  // Port dla STEP_A2
#define GPIO_PORT_B1 GPIOG  // Port dla STEP_B1
#define GPIO_PORT_B2 GPIOG  // Port dla STEP_B2

// Sekwencja pełnokrokowa dla silnika bipolarny
uint8_t step_sequence2[4][4] = {
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 1},
    {1, 0, 0, 1}
};

volatile uint8_t step_index2 = 0;     // Indeks bieżącego kroku
volatile uint16_t step_count2 = 0;    // Licznik kroków dla jednej pozycji
volatile uint16_t steps_per_position = 25;  // Kroki na jedną pozycję
volatile uint8_t current_position = 0;  // Bieżąca pozycja (od 0 do 7)
volatile uint8_t target_position = 0;   // Docelowa pozycja (od 0 do 7)
volatile uint16_t steps_per_position = 25;  // Kroki na jedną pozycję
volatile uint8_t total_positions = 8;   // Liczba pozycji (8 przegrody)


void move_to_position(uint8_t target) {
    int16_t step_difference = (target - current_position + total_positions) % total_positions;
    
    // Wybierz najkrótszą drogę: w przód lub w tył
    if (step_difference > total_positions / 2) {
        step_difference -= total_positions;
    }

    uint16_t steps_to_move = abs(step_difference) * steps_per_position;

    step_count = 0;  // Zresetuj licznik kroków
    target_position = target;  // Ustaw nową docelową pozycję

    // Ustaw kierunek obrotu
    if (step_difference > 0) {
        direction = 1;  // Obrót w prawo (np. zgodnie z ruchem wskazówek zegara)
    } else {
        direction = -1;  // Obrót w lewo
    }

    // Rozpocznij ruch (silnik będzie wykonywał kroki w przerwaniu timera)
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {  // Sprawdzenie, czy przerwanie pochodzi od TIM4
        if (step_count < steps_to_move) {  // Jeśli nie osiągnięto jeszcze docelowej liczby kroków
            // Sterowanie cewkami silnika
            HAL_GPIO_WritePin(GPIO_PORT_A1, STEP_A1, step_sequence[step_index][0]);
            HAL_GPIO_WritePin(GPIO_PORT_A2, STEP_A2, step_sequence[step_index][1]);
            HAL_GPIO_WritePin(GPIO_PORT_B1, STEP_B1, step_sequence[step_index][2]);
            HAL_GPIO_WritePin(GPIO_PORT_B2, STEP_B2, step_sequence[step_index][3]);

            step_index = (step_index + direction + 4) % 4;  // Zwiększ/zmniejsz indeks kroku
            step_count++;  // Zwiększ licznik kroków
        } else {
            // Ruch zakończony – ustaw nową bieżącą pozycję
            current_position = target_position;
        }
    }
}


Dzięki za dodatkowe wyjaśnienie! Skoro silnik ma ustawiać się w jednej z 8 pozycji na podstawie koloru wykrytego przez czujnik TCS, oto jak możemy to zrealizować:

Plan działania:
Czujnik TCS: Odczytujemy kolor wykryty przez czujnik TCS (np. TCS34725 lub podobny).
Analiza koloru: Na podstawie wartości RGB wykrytego koloru przypisujemy odpowiednią pozycję (od 0 do 7).
Obrót silnika:
Silnik obraca się do odpowiedniej pozycji, wykonując odpowiednią liczbę kroków w przód lub w tył, aby osiągnąć najkrótszą drogę do celu.
Zapamiętywanie bieżącej pozycji: Po ustawieniu w danej pozycji zapamiętujemy, w której pozycji znajduje się silnik, aby móc później obliczyć różnicę kroków do kolejnej pozycji.
Zmienne globalne:
Dodaj poniższe zmienne do main.c:

c
Skopiuj kod
volatile uint8_t current_position = 0;  // Bieżąca pozycja (od 0 do 7)
volatile uint8_t target_position = 0;   // Docelowa pozycja (od 0 do 7)
volatile uint16_t steps_per_position = 25;  // Kroki na jedną pozycję
volatile uint8_t total_positions = 8;   // Liczba pozycji (8 przegrody)
Funkcja ustawiająca silnik w docelowej pozycji:
Dodaj funkcję, która obliczy liczbę kroków potrzebnych do osiągnięcia docelowej pozycji:

c
Skopiuj kod
void move_to_position(uint8_t target) {
    int16_t step_difference = (target - current_position + total_positions) % total_positions;
    
    // Wybierz najkrótszą drogę: w przód lub w tył
    if (step_difference > total_positions / 2) {
        step_difference -= total_positions;
    }

    uint16_t steps_to_move = abs(step_difference) * steps_per_position;

    step_count = 0;  // Zresetuj licznik kroków
    target_position = target;  // Ustaw nową docelową pozycję

    // Ustaw kierunek obrotu
    if (step_difference > 0) {
        direction = 1;  // Obrót w prawo (np. zgodnie z ruchem wskazówek zegara)
    } else {
        direction = -1;  // Obrót w lewo
    }

    // Rozpocznij ruch (silnik będzie wykonywał kroki w przerwaniu timera)
}
Modyfikacja przerwania timera:
W przerwaniu timera dodaj obsługę kierunku oraz zatrzymanie po osiągnięciu docelowej liczby kroków:

c
Skopiuj kod
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {  // Sprawdzenie, czy przerwanie pochodzi od TIM4
        if (step_count < steps_to_move) {  // Jeśli nie osiągnięto jeszcze docelowej liczby kroków
            // Sterowanie cewkami silnika
            HAL_GPIO_WritePin(GPIO_PORT_A1, STEP_A1, step_sequence[step_index][0]);
            HAL_GPIO_WritePin(GPIO_PORT_A2, STEP_A2, step_sequence[step_index][1]);
            HAL_GPIO_WritePin(GPIO_PORT_B1, STEP_B1, step_sequence[step_index][2]);
            HAL_GPIO_WritePin(GPIO_PORT_B2, STEP_B2, step_sequence[step_index][3]);

            step_index = (step_index + direction + 4) % 4;  // Zwiększ/zmniejsz indeks kroku
            step_count++;  // Zwiększ licznik kroków
        } else {
            // Ruch zakończony – ustaw nową bieżącą pozycję
            current_position = target_position;
        }
    }
}
Funkcja obsługująca kolor:
Dodaj funkcję, która na podstawie odczytanego koloru wybierze docelową pozycję:

c
Skopiuj kod
uint8_t get_position_from_color(uint8_t r, uint8_t g, uint8_t b) {
    // Przykładowe progi kolorów dla 8 pozycji
    if (r > 200 && g < 100 && b < 100) return 0;  // Czerwony
    if (r > 200 && g > 200 && b < 100) return 1;  // Żółty
    if (r < 100 && g > 200 && b < 100) return 2;  // Zielony
    if (r < 100 && g > 200 && b > 200) return 3;  // Cyjan
    if (r < 100 && g < 100 && b > 200) return 4;  // Niebieski
    if (r > 200 && g < 100 && b > 200) return 5;  // Fioletowy
    if (r > 150 && g > 150 && b > 150) return 6;  // Biały
    return 7;  // Inne kolory lub brak koloru – pozycja 7
}

//Przykładowe wywołanie:


while (1) {
    // Odczytaj wartości kolorów RGB z czujnika TCS
    /*uint8_t r = read_red();
    uint8_t g = read_green();
    uint8_t b = read_blue();
    */

    // Wyznacz docelową pozycję na podstawie koloru
    uint8_t target = get_position_from_color(r, g, b);

    // Przejdź do docelowej pozycji
    move_to_position(target);

    HAL_Delay(1000);  // Przykładowe opóźnienie przed kolejnym odczytem koloru
}