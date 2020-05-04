#include "keyboard.h"
#include "usbhid.h"
#include "state_led.h"
#include "matrix_led.h"
//#include <stdio.h>


extern struct kbReportSt kbReport;

uint8_t KeyCheck(void)
{
	static uint8_t debounceCount[MAX_COL][MAX_ROW];
	uint16_t portVal = 0;
	uint16_t checkingRow = 0;
	uint8_t keyVal = 0;
	uint8_t *debounce = NULL;
	uint8_t nKeyCount = 0;
	uint8_t pressed = 0;
	uint16_t lastSum = 0;
	uint16_t keySum = 0;
	uint8_t modifyVal = kbReport.modify;
	uint8_t fnPressed = 0;
	uint8_t mediaKeyDown = 0;

	//char debugBuff[64] = { '0' };

	kbReport.modify = 0;
	for (uint8_t i = 0; i < 6; i++)
	{
		lastSum += kbReport.keys[i];
		kbReport.keys[i] = 0;
	}

	for (uint8_t x = 0; x < MAX_COL; x++)
	{
		KeyColPrepare(x);
		portVal = ~ReadGpioPort(ROW_GPIO_Port);
		//if (portVal > 0)
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
					if (*debounce >= DEBOUNCE_MS)		//debounce
					{
						pressed = 1;
						keyVal = KEYBOARD_Value_Map[x][y];

						if (*debounce == DEBOUNCE_MS)
						{
							MatrixOnKeyPressed(x, y, keyVal);
						}

						switch (keyVal)
						{
						case KC_LALT:
							kbReport.modify |= KC_LALT_VAL;
							break;
						case KC_LSHIFT:
							kbReport.modify |= KC_LSHIFT_VAL;
							break;
						case KC_LCTRL:
							kbReport.modify |= KC_LCTRL_VAL;
							break;
						case KC_LGUI:
							kbReport.modify |= gameMode == 1 ? 0x00 : KC_LGUI_VAL;
							break;
						case KC_RALT:
							kbReport.modify |= KC_RALT_VAL;
							break;
						case KC_RSHIFT:
							kbReport.modify |= KC_RSHIFT_VAL;
							break;
						case KC_RCTRL:
							kbReport.modify |= KC_RCTRL_VAL;
							break;
						case KC_RGUI:
							kbReport.modify |= gameMode == 1 ? 0x00 : KC_RGUI_VAL;
							break;
						case KC_GAME:
						{
							if (*debounce == DEBOUNCE_MS)
							{
								GameModeSw();
							}
						}
						break;
						case KC_BACKLIGHT:
						{
							if (*debounce == DEBOUNCE_MS)
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
							if (*debounce == DEBOUNCE_MS)
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
									if (*debounce == DEBOUNCE_MS)
									{
										InsertEnableSw();
										MatrixOnKeyPressed(x, y, KC_INSERT_SW);
									}
									continue;
								}
								else if (insertEnable < 1 && kbReport.modify == 0)
								{
									//key insert is disable and none modify key is pressed
									continue;
								}
							}

							uint8_t pressing = 0;
							for (uint8_t i = 0; i < 6; i++)
							{
								if (kbReport.keys[i] == 0)
								{
									kbReport.keys[i] = keyVal;
									pressing = 1;
									break;
								}
							}
							if (pressing == 1)
							{
								continue;
							}

							//if(*debounce == DEBOUNCE)
							{
								kbReport.keys[nKeyCount] = keyVal;
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

	for (uint8_t i = 0; i < 6; i++)
	{
		keySum += kbReport.keys[i];
	}
	if (keySum != lastSum || modifyVal != kbReport.modify)
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
