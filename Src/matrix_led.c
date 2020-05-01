#include "matrix_led.h"
#include "state_led.h"

void MatrixInit(void)
{
	HAL_GPIO_WritePin(MATRIX_RST_GPIO_Port, MATRIX_RST_Pin, GPIO_PIN_SET);				//RESET PIN
	HAL_GPIO_WritePin(MATRIX_SYNC_GPIO_Port, MATRIX_SYNC_Pin, GPIO_PIN_RESET);		//SYNC
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);				//SS
	HAL_Delay(5);		//wait for the chip reset

	//TODO fixme check if chip exist

	//HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);


	//clear
	//len = 3;
	//tmpSpiBuff[0] = ST524_CMD_WRITE_CTL_REG;
	//tmpSpiBuff[1] = 0x00;	//from address
	//tmpSpiBuff[2] = 0x80;	//all reg default
	//HAL_SPI_Transmit(&hspi2, tmpSpiBuff, len, 500 * len);
	//HAL_Delay(5);


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
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);
	//HAL_Delay(1);

	//write data to pattern 1
	len = PATTERN_SIZE + 2;
	matrixBuff[0] = ST524_CMD_WRITE_P1_REG;
	matrixBuff[1] = 0x00; //from address
	for (uint16_t i = 2; i <= PATTERN_SIZE; i++)
	{
		matrixBuff[i] = val;
	}

	HAL_SPI_Transmit(&hspi2, matrixBuff, len, SPI_TIMEOUT_PA * len);

	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_SET);
	//	HAL_Delay(1);
}

void MatrixOn(void)
{
	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_RESET);
	//HAL_Delay(1);

	len = 4;
	matrixBuff[0] = ST524_CMD_WRITE_CTL_REG;
	matrixBuff[1] = ST524_ADDR_SWCTL;	//from address
	matrixBuff[2] = 0x01;	//swctl en
	matrixBuff[3] = 0x01;	//disp p1 en
	HAL_SPI_Transmit(&hspi2, matrixBuff, len, SPI_TIMEOUT_PA * len);

	HAL_GPIO_WritePin(MATRIX_SS_GPIO_Port, MATRIX_SS_Pin, GPIO_PIN_SET);
	//	HAL_Delay(1);
	SetLogoLED(backlight);
}

void MatrixBrightnessChange(void)
{
	switch (backlight)
	{
	case BL_VAL_0:
		backlight = BL_VAL_1;
		break;
	case BL_VAL_1:
		backlight = BL_VAL_2;
		break;
	case BL_VAL_2:
		backlight = BL_VAL_3;
		break;
	case BL_VAL_3:
		backlight = BL_VAL_4;
		break;
	default:
		backlight = BL_VAL_0;
		break;
	}

	SetLogoLED(backlight);

	MatrixSetBrightness(backlight);
	WriteEEPROM(BL_SETTING_ADDR, backlight);
}
