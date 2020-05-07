#include "keyboard.h"
#include "usbhid.h"
#include "state_led.h"
#include "matrix_led.h"


extern struct kbReportSt kbReport;

bool keyFnDown = false;

#define MAX_QUEUE 6

uint8_t queueBuff[MAX_QUEUE];


uint8_t KeyCheck(void)
{
	static uint8_t debounceCount[MAX_COL][MAX_ROW];
	uint16_t portVal = 0;
	uint16_t checkingRow = 0;
	uint8_t keyVal = 0;
	uint8_t *debounce = NULL;
	uint8_t pressed = 0;

	//char debugBuff[64] = { '0' };

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
				keyVal = KEYBOARD_Value_Map[x][y];
				if (checkingRow & portVal)
				{
					pressed = 1;
					if (*debounce < 100)
					{
						(*debounce)++;
					}
					if (*debounce == DEBOUNCE_MS) //debounce		//TODO fixme == only
					{
						//MatrixOnKeyPressed(x, y, keyVal);
						OnKeyDown(x, y, keyVal);
					}
				}
				else
				{
					if (keyVal > 0 && *debounce >= DEBOUNCE_MS)
					{
						OnKeyUp(x, y, keyVal);
					}

					*debounce = 0;
				}
			}
		}
	}

	return pressed;
}

uint32_t ReadGpioPort(GPIO_TypeDef *GPIOx)
{
	return GPIOx->IDR;
}

void KeyColPrepare(uint8_t index)
{
	uint16_t gpio_pin = 0x0001 << index;
	volatile uint16_t delay = 8;

	HAL_GPIO_WritePin(COL_GPIO_Port, COL0_Pin | COL1_Pin | COL2_Pin | COL3_Pin | COL4_Pin | COL5_Pin | COL6_Pin | COL7_Pin | COL8_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(COL_GPIO_Port, gpio_pin, GPIO_PIN_RESET);

	while (delay-- > 0)
	{
	}
}

bool IsModifyKey(uint8_t keyVal)
{
	switch (keyVal)
	{
	case KC_LALT:
	case KC_LSHIFT:
	case KC_LCTRL:
	case KC_LGUI:
	case KC_RALT:
	case KC_RSHIFT:
	case KC_RCTRL:
	case KC_RGUI:
		return true;
	default:
		break;
	}
	return false;
}

void OnKeyDown(uint8_t x, uint8_t y, uint8_t keyVal)
{
	if (IsModifyKey(keyVal))
	{
		keyChange = 1;
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
		GameModeSw();
	}
	break;
	case KC_BACKLIGHT:
	{
		MatrixBrightnessChange();
	}
	break;
	case KC_FN:
	{
		keyFnDown = 1;
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
		break;

	default:
	{
		if (keyVal == KC_INSERT)
		{
			if (keyFnDown == true)
			{
				InsertEnableSw();
				MatrixOnKeyPressed(x, y, KC_INSERT_SW);
				break;
			}
			else if (insertEnable < 1 && kbReport.modify == 0)
			{
				//key insert is disable and none modify key is pressed
				break;
			}
		}

		keyChange = 1;

		bool inReport = false;
		for (uint8_t i = 0; i < 6; i++)
		{
			if (kbReport.keys[i] == keyVal)
			{
				inReport = true;
				break;
			}
			else if (kbReport.keys[i] == 0)
			{
				inReport = true;
				kbReport.keys[i] = keyVal;
				break;
			}
		}
		if (!inReport)
		{
			for (uint8_t i = 0; i < MAX_QUEUE; i++)
			{
				if (queueBuff[i] == 0)
				{
					queueBuff[i] = keyVal;
					break;
				}
			}
		}
	}
	break;
	}
	MatrixOnKeyPressed(x, y, keyVal);
	//sprintf(debugBuff, "x=%u,y=%u,val=%x\n", x, y, keyVal);
	//HAL_UART_Transmit(&huart1, (uint8_t *)debugBuff, 64, 100);
}

void OnKeyUp(uint8_t x, uint8_t y, uint8_t keyVal)
{
	if (IsModifyKey(keyVal))
	{
		keyChange = 1;
	}

	switch (keyVal)
	{
	case KC_LALT:
		kbReport.modify &= ~KC_LALT_VAL;
		break;
	case KC_LSHIFT:
		kbReport.modify &= ~KC_LSHIFT_VAL;
		break;
	case KC_LCTRL:
		kbReport.modify &= ~KC_LCTRL_VAL;
		break;
	case KC_LGUI:
		kbReport.modify &= ~KC_LGUI_VAL;
		break;
	case KC_RALT:
		kbReport.modify &= ~KC_RALT_VAL;
		break;
	case KC_RSHIFT:
		kbReport.modify &= ~KC_RSHIFT_VAL;
		break;
	case KC_RCTRL:
		kbReport.modify &= ~KC_RCTRL_VAL;
		break;
	case KC_RGUI:
		kbReport.modify &= ~KC_RGUI_VAL;
		break;
	case KC_GAME:
	{
	}
	break;
	case KC_BACKLIGHT:
	{
	}
	break;
	case KC_FN:
	{
		keyFnDown = 0;
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
			mediaKeyState = MK_STATE_UP;
		}
		break;

	default:
	{
		keyChange = 1;
		bool queueOut = false;
		for (uint8_t i = 0; i < 6; i++)
		{
			if (kbReport.keys[i] == keyVal)
			{
				keyChange = 1;
				for (uint8_t j = 0; j < MAX_QUEUE; j++)
				{
					if (queueBuff[j] > 0)
					{
						queueOut = true;
						kbReport.keys[i] = queueBuff[j];
						queueBuff[j] = 0;
						break;
					}
				}
				if (!queueOut)
				{
					kbReport.keys[i] = 0;
				}
				break;
			}
		}
		if (!queueOut)
		{
			for (uint8_t i = 0; i < MAX_QUEUE; i++)
			{
				if (queueBuff[i] == keyVal)
				{
					queueBuff[i] = 0;
					break;
				}
			}
		}

	}
	break;
	}
}
