//Funkcja przeliczania kąta 
void SetServoAngle(uint8_t angle) {
    // Przeliczenie kąta na impuls PWM (zakres od 0.5 ms do 2.5 ms)
    uint16_t pulse_length = 500 + ((angle * 2000) / 180);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_length); // Ustawienie wartości PWM
}

int main(void)
{
// Ustawienie kąta na 90 stopni
	         SetServoAngle(90);
	         HAL_Delay(2000); // 2 sekundy

	         // Powrót na pozycję początkową (0 stopni)
	         SetServoAngle(0);
	         HAL_Delay(2000); // 2 sekundy

}