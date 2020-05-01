#include "keyboard.h"
#include "usbhid.h"
#include "state_led.h"
#include "matrix_led.h"
//#include <stdio.h>

uint8_t KeyCheck(void)
{
	static uint8_t debounceCount[MAX_COL][MAX_ROW];
	uint16_t portVal = 0;
	uint16_t checkingRow = 0;
	uint8_t keyVal = 0;
	uint8_t *debounce = NULL;
	uint8_t nKeyCount = 2;
	uint8_t pressed = 0;
	uint16_t lastSum = 0;
	uint16_t keySum = 0;
	uint8_t modifyVal = keyBuff[0];
	uint8_t fnPressed = 0;

	//const char *keyName = NULL;
	//char debugBuff[64] = { '0' };


	for (uint8_t i = 0; i < 8; i++)
	{
		lastSum += keyBuff[i];
		keyBuff[i] = 0;
	}

	for (uint8_t x = 0; x < MAX_COL; x++)
	{
		KeyColPrepare(x);
		portVal = ~ReadGpioPort(ROW_GPIO_Port);
		//		if (portVal > 0)
		{
			for (uint8_t y = 0; y < MAX_ROW; y++)
			{
				checkingRow = 0x8000 >> y;
				debounce = &(debounceCount[x][y]);
				if (checkingRow & portVal)
				{
					if (*debounce < 100)
					{
						(*debounce)++;
					}
					if (*debounce >= 5)		//debounce 
					{
						pressed = 1;
						keyVal = KEYBOARD_Value_Map[x][y];

						if (*debounce == 5)
						{
							MatrixOnKeyPressed(x, y, keyVal);
						}

						switch (keyVal)
						{
						case KC_MODIFIER_LEFT_ALT:
							keyBuff[0] |= KC_MODIFIER_LEFT_ALT_VAL;
							break;
						case KC_MODIFIER_LEFT_SHIFT:
							keyBuff[0] |= KC_MODIFIER_LEFT_SHIFT_VAL;
							break;
						case KC_MODIFIER_LEFT_CTRL:
							keyBuff[0] |= KC_MODIFIER_LEFT_CTRL_VAL;
							break;
						case KC_MODIFIER_LEFT_UI:
							keyBuff[0] |= gameMode == 1 ? 0x00 : KC_MODIFIER_LEFT_UI_VAL;
							break;
						case KC_MODIFIER_RIGHT_ALT:
							keyBuff[0] |= KC_MODIFIER_RIGHT_ALT_VAL;
							break;
						case KC_MODIFIER_RIGHT_SHIFT:
							keyBuff[0] |= KC_MODIFIER_RIGHT_SHIFT_VAL;
							break;
						case KC_MODIFIER_RIGHT_CTRL:
							keyBuff[0] |= KC_MODIFIER_RIGHT_CTRL_VAL;
							break;
						case KC_MODIFIER_RIGHT_UI:
							keyBuff[0] |= gameMode == 1 ? 0x00 : KC_MODIFIER_RIGHT_UI_VAL;
							break;
						case KC_GAME:
						{
							if (*debounce == 5)
							{
								GameModeSw();
							}
						}
						break;
						case KC_BACKLIGHT:
						{
							if (*debounce == 5)
							{
								MatrixBrightnessChange();
							}
						}
						break;
						case KC_APPLICATION:		//as fn key
						{
							fnPressed = 1;
						}
						break;
						default:
						{
							if (keyVal == KC_INSERT)
							{
								if (fnPressed > 0)
								{
									if (*debounce == 5)
									{
										InsertEnableSw();
										MatrixOnKeyPressed(x, y, KC_INSERT_SW);
									}
									continue;
								}
								else if (insertEnable < 1 && keyBuff[0] == 0)
								{
									//key insert is disable and none modify key is pressed
									continue;
								}
							}

							for (uint8_t i = 2; i < 8; i++)
							{
								if (keyBuff[i] == 0)
								{
									keyBuff[i] = keyVal;
									break;
								}
							}

							//if(*debounce == 5)
							{
								keyBuff[nKeyCount] = keyVal;
								if (nKeyCount < 7)
								{
									nKeyCount++;
								}
							}
						}
						break;
						}

						//keyName = KEYBOARD_Name_Map[x][y];
						//sprintf(debugBuff, "x=%u,y=%u,,name=%s,val=%x\n", x, y, keyName, keyVal);
						//HAL_UART_Transmit(&huart1, (uint8_t *)debugBuff, 64, 100);
					}
				}
				else
				{
					*debounce = 0;
				}
			}
		}
	}

	for (uint8_t i = 0; i < 8; i++)
	{
		keySum += keyBuff[i];
	}
	if (keySum != lastSum || modifyVal != keyBuff[0])
	{
		keyChange = 1;
	}

	return pressed;
}


uint32_t ReadGpioPort(GPIO_TypeDef* GPIOx)
{
	return GPIOx->IDR;
}

void KeyColPrepare(uint8_t index)
{
	uint16_t gpio_pin = 0x0001 << index;
	volatile uint16_t delay = 5;

	HAL_GPIO_WritePin(COL_GPIO_Port, COL0_Pin | COL1_Pin | COL2_Pin | COL3_Pin
		| COL4_Pin | COL5_Pin | COL6_Pin | COL7_Pin
		| COL8_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(COL_GPIO_Port, gpio_pin, GPIO_PIN_RESET);

	while (delay-- > 0);
}
