/**
  ******************************************************************************
  * @file    matrix_led.h
  * @author  MCD Application Team
  * @brief   Header file for the matrix_led.c file, driver of STLED524,
  *  		 a 5x24 dot matrix LED display driver.
  ******************************************************************************
  *
  * the matrix LED display driver
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MATRIX_LED_H
#define __MATRIX_LED_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define SPI_TIMEOUT_PA 2

#define ST524_ADDR_SWCTL 0x00
#define ST524_ADDR_DSP_CTL 0x01
#define ST524_ADDR_CLK 0x10
#define ST524_ADDR_COL_CTL1 0x11
#define ST524_ADDR_COL_CTL2 0x12
#define ST524_ADDR_COL_CTL 0x13
#define ST524_ADDR_ROW_CTL 0x14
#define ST524_ADDR_BLANKING_TIME 0x15
#define ST524_ADDR_BOOST_CTL 0x16
#define ST524_ADDR_DSP_VISUAL 0x20
#define ST524_ADDR_PWM_CTL 0x21
#define ST524_ADDR_SCROLL_CTL1 0x30
#define ST524_ADDR_SCROLL_CTL2 0x31
#define ST524_ADDR_SCROLL_CTL3 0x32
#define ST524_ADDR_INTERNET_EN 0x40

#define ST524_CMD_WRITE_CTL_REG 0x00
#define ST524_CMD_WRITE_P1_REG 0x02
#define ST524_CMD_WRITE_P2_REG 0x04
#define ST524_CMD_READ_CTL_REG 0x01
#define ST524_CMD_READ_P1_REG 0x03
#define ST524_CMD_READ_P2_REG 0x05

#define PATTERN_SIZE (0xEF + 1)

#define DEFAULT_DIMMING 0xf0

#define INDEX_OF_KEY_INSERT 142
#define INDEX_OF_KEY_LGUI 192
#define INDEX_OF_KEY_RGUI 204

	enum BRIGHTNESS_VALS
	{
		BL_VAL_0 = 0,
		BL_VAL_1 = 50,
		BL_VAL_2 = 85,
		BL_VAL_3 = 125,
		BL_VAL_4 = 185,
		BL_VAL_5 = 250,
	};

	enum DISPLAY_PATTERNS
	{
		DISP_P1 = 1,
		DISP_P2 = 2,
	};

	extern uint8_t brightness;
	extern uint8_t insertEnable;
	extern uint8_t gameMode;

	extern const unsigned char KEYBOARD_LED_Map[MAX_COL][MAX_ROW];

	extern SPI_HandleTypeDef hspi2;
	extern UART_HandleTypeDef huart1;

	extern void BrightnessSave(void);

	void MatrixInit(void);

	void MatrixSetBrightness(uint8_t val);

	void MatrixDisplayOn(uint8_t p);
	void MatrixDisplayOff(void);

	void MatrixSyncBuff(uint8_t p);

	void MatrixSyncByte(uint8_t p, uint8_t regAddr, uint8_t val);

	void MatrixOnKeyPressed(uint8_t x, uint8_t y, uint8_t keyVal);

	void MatrixBrightnessChange(void);

#ifdef __cplusplus
}
#endif

#endif /* __MATRIX_LED_H */
/**
  * @}
  */

/**
	* @}
	*/
