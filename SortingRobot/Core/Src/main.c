/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

extern I2C_HandleTypeDef hi2c1;  // Używany interfejs I2C

// Funkcja zapisu do rejestru TCS34725
static void TCS34725_WriteRegister(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg | 0x80, value};  // Ustawienie bitu CMD (0x80)
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_I2C_ADDRESS, data, 2, HAL_MAX_DELAY);
}

// Funkcja odczytu danych z rejestru TCS34725
static uint16_t TCS34725_ReadRegister(uint8_t reg) {
    uint8_t data[2] = {0};
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_I2C_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_I2C_ADDRESS, data, 2, HAL_MAX_DELAY);
    return (data[1] << 8) | data[0];
}

// Funkcja inicjalizująca czujnik TCS34725
void TCS34725_Init(void) {
    // Włącz zasilanie i ADC
    TCS34725_WriteRegister(TCS34725_ENABLE_REG, TCS34725_ENABLE_PON);
    HAL_Delay(3);
    TCS34725_WriteRegister(TCS34725_ENABLE_REG, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);

    // Ustaw domyślny czas integracji i wzmocnienie
    TCS34725_WriteRegister(TCS34725_RGBC_TIMING, 0x00);
    TCS34725_WriteRegister(TCS34725_CONTROL, 0x01);
}

// Funkcja odczytująca surowe dane kolorów
static void TCS34725_GetRawData(uint16_t *red, uint16_t *green, uint16_t *blue, uint16_t *clear) {
    *clear = TCS34725_ReadRegister(TCS34725_CDATAL);
    *red   = TCS34725_ReadRegister(TCS34725_RDATAL);
    *green = TCS34725_ReadRegister(TCS34725_GDATAL);
    *blue  = TCS34725_ReadRegister(TCS34725_BDATAL);
}

// Funkcja do rozpoznawania koloru na podstawie surowych danych
uint8_t TCS34725_GetColor(void) {
    uint16_t red, green, blue, clear;
    TCS34725_GetRawData(&red, &green, &blue, &clear);

    if (clear == 0) return COLOR_UNKNOWN;

    // Normalizacja wartości
    float r_ratio = (float)red / clear;
    float g_ratio = (float)green / clear;
    float b_ratio = (float)blue / clear;

    // Prosta klasyfikacja kolorów
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

// Definicje pinów TCRT5000 IR
#define IR_SENSOR_PIN GPIO_PIN_0
#define IR_SENSOR_PORT GPIOA

void IR_Sensor_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = IR_SENSOR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(IR_SENSOR_PORT, &GPIO_InitStruct);
}

// Funkcja sprawdzająca, czy koralik został wykryty
uint8_t IR_Sensor_Detected(void) {
    return HAL_GPIO_ReadPin(IR_SENSOR_PORT, IR_SENSOR_PIN) == GPIO_PIN_SET;
}


//Silnik krokowy NEMA17
#define STEP_A1 GPIO_PIN_6   // PA6 -> IN1
#define STEP_A2 GPIO_PIN_7   // PA7 -> IN2
#define STEP_B1 GPIO_PIN_8   // PA8 -> IN3
#define STEP_B2 GPIO_PIN_9   // PA9 -> IN4

#define GPIO_PORT GPIOA  // Wszystkie piny są na porcie GPIOA

uint8_t step_sequence2[4][4] = {
    {1, 0, 1, 0},
    {0, 1, 1, 0},
    {0, 1, 0, 1},
    {1, 0, 0, 1}
};
volatile uint8_t step_index2 = 0;     // Indeks bieżącego kroku
volatile uint16_t step_count = 0;    // Licznik kroków dla jednej pozycji
volatile uint16_t steps_per_position = 25;  // Kroki na jedną pozycję

volatile uint8_t current_position = 0;  // Bieżąca pozycja (od 0 do 7)
volatile uint8_t target_position = 0;   // Docelowa pozycja (od 0 do 7)
volatile uint8_t total_positions = 8;   // Liczba pozycji (8 przegrody)


/*void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM4) {  // Sprawdzenie, czy przerwanie pochodzi od TIM4
        if (step_count < steps_per_position) {  // Sprawdź, czy nie osiągnięto liczby kroków dla pozycji

            HAL_GPIO_WritePin(GPIO_PORT, STEP_A1, step_sequence2[step_index2][0]);
            HAL_GPIO_WritePin(GPIO_PORT, STEP_A2, step_sequence2[step_index2][1]);
            HAL_GPIO_WritePin(GPIO_PORT, STEP_B1, step_sequence2[step_index2][2]);
            HAL_GPIO_WritePin(GPIO_PORT, STEP_B2, step_sequence2[step_index2][3]);

            step_index2 = (step_index2 + 1) % 4;  // Zwiększ indeks kroku
            step_count++;  // Zwiększ licznik kroków
        }
    }
}*/

// Funkcja do ustawienia pozycji silnika NEMA 17
void Stepper_MoveToStep(uint8_t position) {
    if (position < total_positions) {
        target_position = position;
        step_count = 0;  // Zresetuj licznik kroków
        steps_per_position = abs(target_position - current_position) * 25;  // Oblicz kroki do docelowej pozycji
    }
}

//Silnik krokowy 28BYJ-48 z ULN2003
//definicje


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


    if (htim->Instance == TIM4) {  // Sprawdzenie, czy przerwanie pochodzi od TIM4
            if (step_count < steps_per_position) {  // Jeśli nie osiągnięto jeszcze liczby kroków dla pozycji
                // Sterowanie cewkami silnika
                HAL_GPIO_WritePin(GPIO_PORT, STEP_A1, step_sequence2[step_index2][0]);
                HAL_GPIO_WritePin(GPIO_PORT, STEP_A2, step_sequence2[step_index2][1]);
                HAL_GPIO_WritePin(GPIO_PORT, STEP_B1, step_sequence2[step_index2][2]);
                HAL_GPIO_WritePin(GPIO_PORT, STEP_B2, step_sequence2[step_index2][3]);

                step_index2 = (step_index2 + 1) % 4;  // Przejdź do kolejnego kroku w sekwencji półkrokowej
                step_count++;  // Zwiększ licznik kroków
            } else {
                // Przejdź do kolejnej pozycji
                step_count = 0;  // Zresetuj licznik kroków
                current_position = (current_position + 1) % 4;  // Zwiększ pozycję i zacznij od nowa
            }
        }
}

//Serwomechanizm SG90

//Funkcja przeliczania kąta
void SetServoAngle(uint8_t angle) {
    // Przeliczenie kąta na impuls PWM (zakres od 0.5 ms do 2.5 ms)
    uint16_t pulse_length = 500 + ((angle * 2000) / 180);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_length); // Ustawienie wartości PWM
}

void activate_servo(void) {
    // Funkcja aktywująca serwomechanizm w celu wypchnięcia koralika
    SetServoAngle(90);
    HAL_Delay(500);
    SetServoAngle(0);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim3); // Uruchom przerwania dla TIM3
  HAL_TIM_Base_Start_IT(&htim4);  // Uruchom Timer 4 z przerwaniami

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Uruchomienie silnika 28BYJ-48 do podawania koralików
	  	  	  HAL_TIM_Base_Start_IT(&htim3);  // Uruchomienie przerwań timera 3 dla 28BYJ-48

	          // Czekaj, aż czujnik odbiciowy wykryje koralik
	          while (!IR_Sensor_Detected());

	          // Zatrzymaj silnik podający koraliki
	          HAL_TIM_Base_Stop_IT(&htim3);  // Zatrzymanie przerwań timera 3

	          // Odczytaj kolor koralika za pomocą TCS34725
	          uint8_t detected_color = TCS34725_GetColor();

	          // Obrót silnika krokowego NEMA 17 do odpowiedniej pozycji
	          switch (detected_color) {
	              case COLOR_RED:
	                  Stepper_MoveToStep(0);
	                  break;
	              case COLOR_GREEN:
	                  Stepper_MoveToStep(1);
	                  break;
	              case COLOR_BLUE:
	                  Stepper_MoveToStep(2);
	                  break;
	              case COLOR_YELLOW:
	                  Stepper_MoveToStep(3);
	                  break;
	              case COLOR_ORANGE:
	                  Stepper_MoveToStep(4);
	                  break;
	              case COLOR_PURPLE:
	                  Stepper_MoveToStep(5);
	                  break;
	              case COLOR_WHITE:
	                  Stepper_MoveToStep(6);
	                  break;
	              case COLOR_BLACK:
	                  Stepper_MoveToStep(7);
	                  break;
	              default:
	                  // Jeśli kolor nieznany, zignoruj koralik
	                  continue;
	          }

	          // Pozycjonowanie serwomechanizmu w celu wrzucenia koralika do odpowiedniego pojemnika
	          activate_servo();

	          // Krótka przerwa przed ponownym uruchomieniem silnika podającego
	          HAL_Delay(500);
	      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00A0A3F7;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IR_SENSOR_Pin|IN1_Pin|IN2_Pin|IN3_Pin
                          |IN4_Pin|STEP_A1_Pin|STEP_A2_Pin|STEP_B1_Pin
                          |STEP_B2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IR_SENSOR_Pin IN1_Pin IN2_Pin IN3_Pin
                           IN4_Pin STEP_A1_Pin STEP_A2_Pin STEP_B1_Pin
                           STEP_B2_Pin */
  GPIO_InitStruct.Pin = IR_SENSOR_Pin|IN1_Pin|IN2_Pin|IN3_Pin
                          |IN4_Pin|STEP_A1_Pin|STEP_A2_Pin|STEP_B1_Pin
                          |STEP_B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
