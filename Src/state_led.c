#include "state_led.h"

void SetNumLockLED(uint8_t on)
{
	HAL_GPIO_WritePin(NUM_LED_GPIO_Port, NUM_LED_Pin, on == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void SetCapsLockLED(uint8_t on)
{
	HAL_GPIO_WritePin(CAP_LED_GPIO_Port, CAP_LED_Pin, on == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void SetScrollLockLED(uint8_t on)
{
	HAL_GPIO_WritePin(SCR_LED_GPIO_Port, SCR_LED_Pin, on == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void SetModeLED(uint8_t on)
{
	HAL_GPIO_WritePin(MODE_LED_GPIO_Port, MODE_LED_Pin, on == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void SetLogoLED(uint16_t brightVal)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, brightVal);
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}
