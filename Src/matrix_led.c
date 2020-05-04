#include "matrix_led.h"
#include "state_led.h"
#include "usbhid.h"
//#include <stdio.h>

void MatrixInit(void)
{
	HAL_GPIO_WritePin(MATRIX_RST_GPIO_Port, MATRIX_RST_Pin, GPIO_PIN_SET);				//RESET PIN
	HAL_GPIO_WritePin(MATRIX_SYNC_GPIO_Port, MATRIX_SYNC_Pin, GPIO_PIN_RESET);		//SYNC
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);				//SS
	HAL_Delay(1);		//wait for the chip reset

	//HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);

	//clear all reg
	len = 3;
	uint8_t cmdBuff[3];
	cmdBuff[0] = ST524_CMD_WRITE_CTL_REG;
	cmdBuff[1] = 0x00;	//from address
	cmdBuff[2] = 0x83;	//all reg default
	HAL_SPI_Transmit(&hspi2, cmdBuff, len, SPI_TIMEOUT_PA * len);
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_SET);

	//clear buff
	for (uint16_t i = 0; i < PATTERN_SIZE; i+=2)
	{
		matrixBuff[i] = DEFAULT_DIMMING;	//this value will also control the brightness
	}
	if (insertEnable < 1)
	{
		//turn of insert
		matrixBuff[INDEX_OF_KEY_INSERT] = 0x00;
	}
	if (gameMode > 0)
	{
		//turn off GUI
		matrixBuff[INDEX_OF_KEY_LGUI] = 0x00;
		matrixBuff[INDEX_OF_KEY_RGUI] = 0x00;
	}

	MatrixSyncBuff(DISP_P1);

	MatrixSetBrightness(brightness);
}

void MatrixSetBrightness(uint8_t val)
{
	//set pwm reg
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);

	len = 4;
	uint8_t cmdBuff[4];
	cmdBuff[0] = ST524_CMD_WRITE_CTL_REG;
	cmdBuff[1] = ST524_ADDR_DSP_VISUAL;	//from address
	cmdBuff[2] = 0x02;	//swctl en
	cmdBuff[3] = val;	//pwm reg
	HAL_SPI_Transmit(&hspi2, cmdBuff, len, SPI_TIMEOUT_PA * len);

	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_SET);

	SetLogoLED(brightness);
}


void MatrixDisplayOn(uint8_t p)
{
	if (p != DISP_P1 && p != DISP_P2)
	{
		return;
	}

	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);

	len = 4;
	uint8_t cmdBuff[4];
	cmdBuff[0] = ST524_CMD_WRITE_CTL_REG;
	cmdBuff[1] = ST524_ADDR_SWCTL;	//from address
	cmdBuff[2] = 0x01;	//swctl en
	cmdBuff[3] = p;
	HAL_SPI_Transmit(&hspi2, cmdBuff, len, SPI_TIMEOUT_PA * len);

	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_SET);
}

void MatrixDisplayOff(void)
{
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);

	len = 3;
	uint8_t cmdBuff[4];
	cmdBuff[0] = ST524_CMD_WRITE_CTL_REG;
	cmdBuff[1] = ST524_ADDR_SWCTL;	//from address
	cmdBuff[2] = 0x00;	//swctl en=0
	HAL_SPI_Transmit(&hspi2, cmdBuff, len, SPI_TIMEOUT_PA * len);

	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_SET);
}

void MatrixSyncBuff(uint8_t p)
{
	if (p != DISP_P1 && p != DISP_P2)
	{
		return;
	}

	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);

	//write data to pattern 1
	len = 2;
	uint8_t cmdBuff[2];
	cmdBuff[0] = p == DISP_P1 ? ST524_CMD_WRITE_P1_REG : ST524_CMD_WRITE_P2_REG;;
	cmdBuff[1] = 0x00; //address
	HAL_SPI_Transmit(&hspi2, cmdBuff, len, SPI_TIMEOUT_PA * len);

	len = PATTERN_SIZE;
	HAL_SPI_Transmit(&hspi2, matrixBuff, len, SPI_TIMEOUT_PA * len);

	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_SET);
}

void MatrixSyncByte(uint8_t p, uint8_t regAddr, uint8_t val)
{
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);

	//write data to pattern 1
	len = 3;
	uint8_t cmdBuff[3];
	cmdBuff[0] = ST524_CMD_WRITE_P1_REG;
	cmdBuff[1] = regAddr; //address
	cmdBuff[2] = val;
	HAL_SPI_Transmit(&hspi2, cmdBuff, len, SPI_TIMEOUT_PA * len);

	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_SET);
}

void MatrixOnKeyPressed(uint8_t x, uint8_t y, uint8_t keyVal)
{
	switch (keyVal)
	{
	case KC_INSERT_SW:
	{
		uint8_t index = KEYBOARD_LED_Map[x][y];

		matrixBuff[index] = insertEnable > 0 ? DEFAULT_DIMMING : 0;

		MatrixSyncByte(DISP_P1, index, matrixBuff[index]);
	}
		break;
	case KC_GAME:
	{
		if (gameMode > 0)
		{
			matrixBuff[INDEX_OF_KEY_LGUI] = DEFAULT_DIMMING;
			matrixBuff[INDEX_OF_KEY_RGUI] = DEFAULT_DIMMING;
		}
		else
		{
			matrixBuff[INDEX_OF_KEY_LGUI] = 0;
			matrixBuff[INDEX_OF_KEY_RGUI] = 0;
		}
		MatrixSyncByte(DISP_P1, INDEX_OF_KEY_LGUI, matrixBuff[INDEX_OF_KEY_LGUI]);
		MatrixSyncByte(DISP_P1, INDEX_OF_KEY_RGUI, matrixBuff[INDEX_OF_KEY_RGUI]);
		//MatrixSyncBuff();
	}
		break;

	default:
		break;
	}
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

	SetLogoLED(brightness);

	MatrixSetBrightness(brightness);

	MatrixSyncBuff(DISP_P1);

	BrightnessSave();
}
