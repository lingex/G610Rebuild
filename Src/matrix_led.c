#include "matrix_led.h"
#include "state_led.h"
#include "usbhid.h"
//#include <stdio.h>

void MatrixInit(void)
{
	HAL_GPIO_WritePin(MATRIX_RST_GPIO_Port, MATRIX_RST_Pin, GPIO_PIN_SET);				//RESET PIN
	HAL_GPIO_WritePin(MATRIX_SYNC_GPIO_Port, MATRIX_SYNC_Pin, GPIO_PIN_RESET);		//SYNC
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);				//SS
	HAL_Delay(5);		//wait for the chip reset

	//TODO fixme check if chip exist

	//HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);


	//clear
	len = 3;
	uint8_t cmdBuff[3];
	cmdBuff[0] = ST524_CMD_WRITE_CTL_REG;
	cmdBuff[1] = 0x00;	//from address
	cmdBuff[2] = 0x83;	//all reg default
	HAL_SPI_Transmit(&hspi2, cmdBuff, len, SPI_TIMEOUT_PA * len);
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_SET);
	//HAL_Delay(5);

	for (uint16_t i = 0; i < PATTERN_SIZE; i++)
	{
		matrixBuff[i] = brightness;
	}
	if (insertEnable < 1)
	{
		//turn of insert
		matrixBuff[INDEX_OF_KEY_INSERT] = 0x00;
	}

	//display on
	//len = 4;
	//matrixBuff[0] = ST524_CMD_WRITE_CTL_REG;
	//matrixBuff[1] = ST524_ADDR_SWCTL;	//from address
	//matrixBuff[2] = 0x01;	//swctl en
	//matrixBuff[3] = 0x01;	//disp p1 en
	//HAL_SPI_Transmit(&hspi2, matrixBuff, len, SPI_TIMEOUT_PA * len);
	//HAL_Delay(1);

	//enable pwm
	//len = 4;
	//tmpSpiBuff[0] = ST524_CMD_WRITE_CTL_REG;
	//tmpSpiBuff[1] = ST524_ADDR_DSP_VISUAL;	//from address
	//tmpSpiBuff[2] = 0x02;	//disp visual ctrl reg
	//tmpSpiBuff[3] = 0x0f;	//pwm ctrl reg
	//HAL_SPI_Transmit(&hspi2, tmpSpiBuff, len, SPI_TIMEOUT_PA * len);
	//HAL_Delay(1);
}

void MatrixSetBrightness(uint8_t val)
{
	len = PATTERN_SIZE + 1;
	for (uint8_t i = 0; i < PATTERN_SIZE; i++)
	{
		matrixBuff[i] = val;
	}
	if (insertEnable < 1)
	{
		//turn of insert
		matrixBuff[INDEX_OF_KEY_INSERT] = 0x00;
	}
}

void MatrixOn(void)
{
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);

	len = 4;
	uint8_t cmdBuff[4];
	cmdBuff[0] = ST524_CMD_WRITE_CTL_REG;
	cmdBuff[1] = ST524_ADDR_SWCTL;	//from address
	cmdBuff[2] = 0x01;	//swctl en
	cmdBuff[3] = 0x01;	//disp p1 en
	HAL_SPI_Transmit(&hspi2, cmdBuff, len, SPI_TIMEOUT_PA * len);

	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_SET);
	SetLogoLED(brightness);
}

void MatrixSyncBuff(void)
{
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);

	//write data to pattern 1
	len = 2;
	uint8_t cmdBuff[2];
	cmdBuff[0] = ST524_CMD_WRITE_P1_REG;
	cmdBuff[1] = 0x00; //address
	HAL_SPI_Transmit(&hspi2, cmdBuff, len, SPI_TIMEOUT_PA * len);

	len = PATTERN_SIZE;
	HAL_SPI_Transmit(&hspi2, matrixBuff, len, SPI_TIMEOUT_PA * len);

	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_SET);
}

void MatrixSyncByte(uint8_t regAddr, uint8_t val)
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
	if (keyVal == KC_INSERT_SW)
	{
		uint8_t index = KEYBOARD_LED_Map[x][y];
		//if (index == INDEX_OF_KEY_INSERT)
		{
			if (insertEnable)
			{
				matrixBuff[index] = brightness;
			}
			else
			{
				matrixBuff[index] = 0;
			}
		}
		MatrixSyncByte(index, matrixBuff[index]);
		//MatrixSyncBuff();
	}
	//TODO something else
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
	MatrixSyncBuff();

	WriteEEPROM(BL_SETTING_ADDR, brightness);
}
