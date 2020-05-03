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
	uint8_t modifyVal = keyBuff[1];
	uint8_t fnPressed = 0;
	uint8_t mediaKeyDown = 0;

	//char debugBuff[64] = { '0' };


	for (uint8_t i = 0; i < REPORT_SIZE; i++)
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
						case KC_LALT:
							keyBuff[1] |= KC_LALT_VAL;
							break;
						case KC_LSHIFT:
							keyBuff[1] |= KC_LSHIFT_VAL;
							break;
						case KC_LCTRL:
							keyBuff[1] |= KC_LCTRL_VAL;
							break;
						case KC_LGUI:
							keyBuff[1] |= gameMode == 1 ? 0x00 : KC_LGUI_VAL;
							break;
						case KC_RALT:
							keyBuff[1] |= KC_RALT_VAL;
							break;
						case KC_RSHIFT:
							keyBuff[1] |= KC_RSHIFT_VAL;
							break;
						case KC_RCTRL:
							keyBuff[1] |= KC_RCTRL_VAL;
							break;
						case KC_RGUI:
							keyBuff[1] |= gameMode == 1 ? 0x00 : KC_RGUI_VAL;
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
						case KC_FN:
						{
							fnPressed = 1;
						}
						break;
						case KC_MEDIA_PLAY:
						case KC_MEDIA_STOP:
						case KC_MEDIA_SCAN_NEXT:
						case KC_MEDIA_SCAN_PREV:
						case KC_MEDIA_MUTE:
						//case KC_MEDIA_VOLUME_UP:
						//case KC_MEDIA_VOLUME_DOWN:
						{
							mediaKeyDown = 1;
							if (*debounce == 5)
							{
								mediaKeyState = MK_STATE_DOWN;
								switch (keyVal)
								{
								case KC_MEDIA_PLAY:
									mediaKeyVal = KC_MEDIA_PLAY_VAL;
									break;
								case KC_MEDIA_STOP:
									mediaKeyVal = KC_MEDIA_STOP_VAL;
									break;
								case KC_MEDIA_SCAN_NEXT:
									mediaKeyVal = KC_MEDIA_SCAN_NEXT_VAL;
									break;
								case KC_MEDIA_SCAN_PREV:
									mediaKeyVal = KC_MEDIA_SCAN_PREV_VAL;
									break;
								case KC_MEDIA_MUTE:
									mediaKeyVal = KC_MEDIA_MUTE_VAL;
									break;
								default:
									break;
								}
							}
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

							for (uint8_t i = 3; i < REPORT_SIZE; i++)
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
								if (nKeyCount < 8)
								{
									nKeyCount++;
								}
							}
						}
						break;
						}

						//sprintf(debugBuff, "x=%u,y=%u,val=%x\n", x, y, keyVal);
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

	for (uint8_t i = 0; i < REPORT_SIZE; i++)
	{
		keySum += keyBuff[i];
	}
	if (keySum != lastSum || modifyVal != keyBuff[1])
	{
		keyChange = 1;
	}
	if (mediaKeyState == MK_STATE_REPORTED && mediaKeyDown == 0)
	{
		mediaKeyState = MK_STATE_UP;
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
