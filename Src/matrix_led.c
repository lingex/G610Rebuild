﻿#include "matrix_led.h"
#include "state_led.h"
#include "usbhid.h"
#include <stdio.h>

static uint8_t gCmdBuff[255] = {0};
static uint8_t* matrixBuff = &gCmdBuff[2]; //led buff

static MatrixTaskTypeDef matrixTask[MAX_MATRIX_TASK] = {0};

/*
	about the STLED524 led driver

	every 2 bytes controls dimming, slope and delay registers of one dot

	Dimming 			Bit7 Bit6 Bit5 Bit4 Bit3 Bit2 Bit1 Bit0
	Add = nnh 							Dn_xx[7:0]
	Default 						value Not defined

	Slope and delay 	Bit7 Bit6 Bit5 Bit4		Bit3 Bit2 		Bit1 Bit0
	Add = (nn+1)h 		  -	   -    -    -   	PnCYCxx[1:0] 	PnDLYxx[1:0]
	Default value 		  -    -    -    -      Not defined     Not defined

*/

const unsigned char MATRIX_LED_Map[MAX_COL][MAX_ROW] =
{
	{0,				0,			0,				0,				0,				0,				0,					0,				MLI_NONE,	MLI_BL,		MLI_PLAY,		MLI_STOP,	MLI_NEXT,	MLI_PREV,			MLI_KP_MINUS,		MLI_MUTE,},
	{MLI_ESC,		MLI_F1,		MLI_F2,		    MLI_F3,			MLI_F4,			MLI_F5,			MLI_F6,				MLI_F7,			MLI_F8,		MLI_F9,		MLI_F10,		MLI_F11,	MLI_F12,	MLI_PSR,			MLI_SCRLOCK,		MLI_PAUSE,},
	{0,				0,			MLI_TILDE,	    MLI_1,			MLI_2,			MLI_3,			MLI_4,				MLI_5,			MLI_6,		MLI_7,		MLI_8,			MLI_9,		MLI_0,		MLI_UNDERSCORE,		MLI_PLUS,			0,},
	{0,				0,			MLI_TAB,		MLI_Q,			MLI_W,			MLI_E,			MLI_R,				MLI_T,			MLI_Y,		MLI_U,		MLI_I,			MLI_O,		MLI_P,		MLI_OPEN_BRACKET,	MLI_CLOSE_BRACKET,	MLI_OPEN_BACKSLASH,},
	{0,				0,			MLI_CAPS_LOCK,	MLI_A,			MLI_S,			MLI_D,			MLI_F,				MLI_G,			MLI_H,		MLI_J,		MLI_K,			MLI_L,		MLI_COLON,	MLI_QUOTE,		    0,					MLI_ENTER,},
	{0,				0,			MLI_LSHIFT,		0,				MLI_Z,			MLI_X,			MLI_C,				MLI_V,			MLI_B,		MLI_N,		MLI_M,			MLI_COMMA,	MLI_DOT,	MLI_SLASH,		    0,					MLI_RSHIFT,},
	{0,				0,			MLI_LCTRL,		MLI_LGUI,		MLI_LALT,		0,				0,					MLI_SPACEBAR,	0,			0,			0,				MLI_RALT,	MLI_RGUI,	MLI_FN,				MLI_RCTRL,			MLI_LEFT,},
	{MLI_BACKSPACE,	MLI_INSERT,	MLI_HOME,		MLI_PAGEUP,		MLI_NUM_LOCK,	MLI_KP_DIVIDE,	MLI_KP_MULTIPLY,	MLI_KP_PLUS,	MLI_DELETE,	MLI_END,	MLI_PAGEDOWN,	MLI_KP_7,	MLI_KP_8,	MLI_KP_9,			MLI_KP_4,			MLI_KP_5,},
	{MLI_RIGHT,		MLI_KP_0,	MLI_KP_DOT,		MLI_KP_ENTER,	MLI_KP_1,		MLI_KP_2,		MLI_KP_6,			MLI_KP_3,		MLI_DOWN,	MLI_UP,		0,				0,			0,			0,					0,					0,},
};

const unsigned char MATRIX_Abs_Map[7][22] =
{
	{MLI_NONE,		MLI_NONE,	MLI_NONE,	MLI_NONE,		MLI_NONE,		MLI_NONE,		MLI_NONE,		MLI_NONE,		MLI_NONE,		MLI_BL,		MLI_NONE,	MLI_NONE,			MLI_NONE,			MLI_NONE,			MLI_NONE,		MLI_MUTE,	MLI_NONE,		MLI_NONE,	MLI_NONE,		MLI_NONE,		MLI_NONE,		MLI_NONE,},
	{MLI_ESC,		MLI_F1,		MLI_F2,		MLI_F3,			MLI_F4,			MLI_F5,			MLI_F6,			MLI_F7,			MLI_F8,			MLI_F9,		MLI_F10,	MLI_F11,			MLI_F12,			MLI_PSR,			MLI_SCRLOCK,	MLI_PAUSE,	MLI_PLAY,		MLI_STOP,	MLI_PREV,		MLI_NEXT,		MLI_NONE,		MLI_NONE,},
	{MLI_TILDE,	    MLI_1,		MLI_2,		MLI_3,			MLI_4,			MLI_5,			MLI_6,			MLI_7,			MLI_8,			MLI_9,		MLI_0,		MLI_UNDERSCORE,		MLI_PLUS,			MLI_BACKSPACE,		MLI_BACKSPACE,	MLI_INSERT,	MLI_HOME,		MLI_PAGEUP,	MLI_NUM_LOCK, 	MLI_KP_DIVIDE,	MLI_KP_MULTIPLY,MLI_KP_MINUS,},
	{MLI_TAB,		MLI_Q,		MLI_W,		MLI_E,			MLI_R,			MLI_T,			MLI_Y,			MLI_U,			MLI_I,			MLI_O,		MLI_P,		MLI_OPEN_BRACKET,	MLI_CLOSE_BRACKET,	MLI_OPEN_BACKSLASH,	MLI_DELETE,		MLI_END,	MLI_PAGEDOWN,	MLI_KP_7,	MLI_KP_8,		MLI_KP_9,		MLI_KP_PLUS,	MLI_NONE,},
	{MLI_CAPS_LOCK,	MLI_A,		MLI_S,		MLI_D,			MLI_F,			MLI_G,			MLI_H,			MLI_J,			MLI_K,			MLI_L,		MLI_COLON,	MLI_QUOTE,		    MLI_ENTER,			MLI_ENTER,			MLI_NONE,		MLI_NONE,	MLI_NONE,		MLI_KP_4,	MLI_KP_5,		MLI_KP_6,		MLI_KP_PLUS,	MLI_NONE,},
	{MLI_LSHIFT,	MLI_LSHIFT,	MLI_Z,		MLI_X,			MLI_C,			MLI_V,			MLI_B,			MLI_N,			MLI_M,			MLI_COMMA,	MLI_DOT,	MLI_SLASH,	    	MLI_RSHIFT,			MLI_RSHIFT,			MLI_NONE,		MLI_UP,		MLI_NONE,		MLI_KP_1,	MLI_KP_2,		MLI_KP_3,		MLI_KP_ENTER,	MLI_NONE,},
	{MLI_LCTRL,		MLI_LGUI,	MLI_LALT,	MLI_SPACEBAR,	MLI_SPACEBAR,	MLI_SPACEBAR,	MLI_SPACEBAR,	MLI_SPACEBAR,	MLI_SPACEBAR,	MLI_RALT,	MLI_RGUI,	MLI_FN,				MLI_RCTRL,			MLI_LEFT,			MLI_DOWN,		MLI_RIGHT,	MLI_KP_0,		MLI_KP_0,	MLI_KP_DOT,		MLI_KP_ENTER,	MLI_NONE,		MLI_NONE,},
};


WaveEffectTypeDef waveTask[MAX_EFFECT_TASK];

void MatrixInit(void)
{
	HAL_GPIO_WritePin(MATRIX_RST_GPIO_Port, MATRIX_RST_Pin, GPIO_PIN_SET);	   //RESET PIN
	HAL_GPIO_WritePin(MATRIX_SYNC_GPIO_Port, MATRIX_SYNC_Pin, GPIO_PIN_RESET); //SYNC
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);	   //SS
	HAL_Delay(1);															   //wait for the chip reset

	//HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);

	//clear all reg
	uint8_t cmdBuff[3];
	cmdBuff[0] = ST524_CMD_WRITE_CTL_REG;
	cmdBuff[1] = 0x00; //from address
	cmdBuff[2] = 0x83; //all reg default

	SpiTransmit(cmdBuff, sizeof(cmdBuff));

	WaveEffectTypeDef* pEffect = NULL;
	for (uint8_t i = 0; i < MAX_EFFECT_TASK; i++)
	{
		pEffect = &waveTask[i];
		pEffect->nextTick = 0;
	}

#if 0
	//clear buff
	for (uint16_t i = 0; i < PATTERN_SIZE; i += 2)
	{
		matrixBuff[i] = DEFAULT_DIMMING; //this value will also control the brightness
	}
	if (insertEnable < 1)
	{
		//turn of insert
		matrixBuff[MLI_INSERT] = 0x00;
	}
	if (gameMode > 0)
	{
		//turn off GUI
		matrixBuff[MLI_LGUI] = 0x00;
		matrixBuff[MLI_RGUI] = 0x00;
	}
#endif

	//MatrixSyncBuff(DISP_P1);
#ifdef _DEBUG_
	uint32_t start = HAL_GetTick();
#endif

	volatile int loop = 3;
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	while (loop-- > 0);

	MatrixSetBrightness(brightness);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	loop = 3;
	while (loop-- > 0);

	MatrixDisplayOn(DISP_P1);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	loop = 3;
	while (loop-- > 0);

#ifdef _DEBUG_
	char debugBuff[64] = {0};
	sprintf(debugBuff, "Matrix init cost: %lu.\n", HAL_GetTick() - start);
	HAL_UART_Transmit(&huart1, (uint8_t *)debugBuff, 64, 100);
#endif
}

void MatrixSetBrightness(uint8_t val)
{
#if 0
	//this is another way to change the brightness (by set the pwm reg), but will cause noise
	uint8_t cmdBuff[4];
	cmdBuff[0] = ST524_CMD_WRITE_CTL_REG;
	cmdBuff[1] = ST524_ADDR_DSP_VISUAL; //from address
	cmdBuff[2] = 0x02;					//swctl en
	cmdBuff[3] = val;					//pwm reg
	SpiTransmit(cmdBuff, 4);
#endif

	for (uint16_t i = 0; i < PATTERN_SIZE; i += 2)
	{
		matrixBuff[i] = val;
	}
	if (insertEnable < 1)
	{
		//turn of insert
		matrixBuff[MLI_INSERT] = 0x00;
	}
	if (gameMode > 0)
	{
		//turn off GUI
		matrixBuff[MLI_LGUI] = 0x00;
		matrixBuff[MLI_RGUI] = 0x00;
	}

	MatrixSyncBuff(DISP_P1);
	SetLogoLED(brightness);
}

void MatrixDisplayOn(uint8_t p)
{
	assert_param(IS_ST524_PATTERN(P));

	uint8_t cmdBuff[4];
	cmdBuff[0] = ST524_CMD_WRITE_CTL_REG;
	cmdBuff[1] = ST524_ADDR_SWCTL; //address
	cmdBuff[2] = 0x01;			   //swctl en
	cmdBuff[3] = p;
	SpiTransmit(cmdBuff, sizeof(cmdBuff));
}

void MatrixDisplayOff(void)
{
	uint8_t cmdBuff[3];
	cmdBuff[0] = ST524_CMD_WRITE_CTL_REG;
	cmdBuff[1] = ST524_ADDR_SWCTL; //address
	cmdBuff[2] = 0x00;			   //swctl en=0

	SpiTransmit(cmdBuff, sizeof(cmdBuff));
}

void MatrixSyncBuff(uint8_t p)
{
	assert_param(IS_ST524_PATTERN(P));

	//write data to pattern
	gCmdBuff[0] = p == DISP_P1 ? ST524_CMD_WRITE_P1_REG : ST524_CMD_WRITE_P2_REG;
	gCmdBuff[1] = 0x00; //address

	SpiTransmit(gCmdBuff, PATTERN_SIZE + 2);
}

void MatrixSyncByte(uint8_t p, uint8_t regAddr, uint8_t val)
{
	//write data to pattern 1
	uint8_t cmdBuff[3];
	cmdBuff[0] = ST524_CMD_WRITE_P1_REG;
	cmdBuff[1] = regAddr; //address
	cmdBuff[2] = val;

	SpiTransmit(cmdBuff, sizeof(cmdBuff));
}

void MatrixOnKeyPressed(uint8_t x, uint8_t y, uint8_t keyVal)
{
	WaveEffectTypeDef* pEffect = NULL;
	uint8_t index = MATRIX_LED_Map[x][y];

	if (smearLight != 0)
	{
		for (uint8_t i = 0; i < MAX_EFFECT_TASK; i++)
		{
			pEffect = &waveTask[i];
			if (pEffect->nextTick == 0)	//first empty task
			{
				pEffect->nextTick = HAL_GetTick() + MAX_EFFECT_INTERVAL;
				pEffect->step = 0;
				pEffect->x = x;
				pEffect->y = y;
				pEffect->dotIndex = index;

				MatrixSyncByte(DISP_P1, index, 0xff);
				break;
			}
			else if (pEffect->dotIndex == index)//press again, restart task
			{
				pEffect->step = 0;
			}
		}
	}

	switch (keyVal)
	{
	case KC_INSERT_SW:
	{
		//uint8_t index = MATRIX_LED_Map[x][y];

		//matrixBuff[index] = insertEnable > 0 ? DEFAULT_DIMMING : 0;
		matrixBuff[MLI_INSERT] = insertEnable > 0 ? brightness : 0;

		MatrixSyncByte(DISP_P1, MLI_INSERT, matrixBuff[MLI_INSERT]);
	}
	break;
	case KC_GAME:
	{
		if (gameMode == 0)
		{
			matrixBuff[MLI_LGUI] = brightness;
			matrixBuff[MLI_RGUI] = brightness;
		}
		else
		{
			matrixBuff[MLI_LGUI] = 0;
			matrixBuff[MLI_RGUI] = 0;
		}
		MatrixSyncByte(DISP_P1, MLI_LGUI, matrixBuff[MLI_LGUI]);
		MatrixSyncByte(DISP_P1, MLI_RGUI, matrixBuff[MLI_RGUI]);
	}
	break;

	default:
		break;
	}
}

void MatrixOnKeyReleased(uint8_t x, uint8_t y, uint8_t keyVal)
{
	UNUSED(x);
	UNUSED(y);
	UNUSED(keyVal);
}

void MatrixBrightnessChange(void)
{
	switch (brightness)
	{
	case BL_VAL_0:
		brightness = BL_VAL_1;
		break;
	case BL_VAL_1:
		brightness = BL_VAL_2;
		break;
	case BL_VAL_2:
		brightness = BL_VAL_3;
		break;
	case BL_VAL_3:
		brightness = BL_VAL_4;
		break;
	case BL_VAL_4:
		brightness = BL_VAL_5;
		break;
	default:
		brightness = BL_VAL_0;
		break;
	}

	MatrixSetBrightness(brightness);
	BrightnessSave();
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
}

void MatrixEffectTimer(uint32_t timeTick)
{
	static uint8_t taskIndex;
	WaveEffectTypeDef* pEffect = &waveTask[taskIndex];

	if (pEffect->nextTick > 0 && timeTick >= pEffect->nextTick)
	{
		MatrixEffectNextMove(pEffect, timeTick);
	}

	if (++taskIndex >= MAX_EFFECT_TASK)
	{
		taskIndex = 0;
	}
}

void MatrixEffectNextMove(WaveEffectTypeDef* pEffect, uint32_t timeTick)
{
	if (pEffect->step >= MAX_EFFECT_STEP)
	{
		//restore
		MatrixSyncByte(DISP_P1, pEffect->dotIndex, matrixBuff[pEffect->dotIndex]);
		pEffect->nextTick = 0;
	}
	else
	{
		pEffect->step++;
		pEffect->nextTick = timeTick + MAX_EFFECT_INTERVAL;
		MatrixSyncByte(DISP_P1, pEffect->dotIndex, (EFFECT_VAL_HI - pEffect->step * EFFECT_STEP_VAL));
	}
}

bool MatrixTaskPush(uint8_t x, uint8_t y, uint8_t keyVal)
{
	MatrixTaskTypeDef* pTask = NULL;

	for (uint8_t i = 0; i < MAX_MATRIX_TASK; i++)
	{
		pTask = &matrixTask[i];
		if (pTask->keyVal == 0)	//first free task
		{
			pTask->x = x;
			pTask->y = y;
			pTask->keyVal = keyVal;
			return true;
			break;
		}
	}
	return false;
}

void MatrixTaskTimer()
{
	static uint8_t taskIndex = 0;
	MatrixTaskTypeDef* pTask = &matrixTask[taskIndex];

	if (pTask->keyVal > 0)
	{
		MatrixOnKeyPressed(pTask->x, pTask->y, pTask->keyVal);
		pTask->keyVal = 0;	//free this task
	}

	if (++taskIndex >= MAX_MATRIX_TASK)
	{
		taskIndex = 0;
	}
}

void SpiTransmit(uint8_t* pData, uint16_t len)
{
	//while (HAL_DMA_GetState(&hdma_spi2_tx) != HAL_DMA_STATE_READY);
	while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);

	ResetSpiSSPin();

#if 1	//using DMA
	HAL_SPI_Transmit_DMA(&hspi2, pData, len);

	uint32_t unused = hspi2.Instance->SR;	//read SR and DR to clear all interrupt flag
	unused = hspi2.Instance->DR;
	UNUSED(unused);
	//__HAL_SPI_CLEAR_CRCERRFLAG(&hspi2);
	//__HAL_SPI_CLEAR_FREFLAG(&hspi2);
	//__HAL_SPI_CLEAR_MODFFLAG(&hspi2);
	//__HAL_SPI_CLEAR_OVRFLAG(&hspi2);
#else
	//HAL_SPI_Transmit_IT(&hspi2, pData, len);	//not test

	HAL_SPI_Transmit(&hspi2, pData, len, SPI_TIMEOUT_PA * len);
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_SET);
#endif

}

inline void SetSpiSSPin(void)
{
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_SET);
}

inline void ResetSpiSSPin(void)
{
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);
}