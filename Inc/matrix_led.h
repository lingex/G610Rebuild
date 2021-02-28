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

#define SPI_TIMEOUT_PA 		1

#define MAX_EFFECT_TASK		8
#define MAX_EFFECT_STEP		10
#define MAX_EFFECT_INTERVAL	100
#define EFFECT_STEP_VAL		(EFFECT_VAL_HI - brightness)/MAX_EFFECT_STEP
#define EFFECT_VAL_HI		0xfe
#define EFFECT_VAL_LO		0x0a


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


enum
{
	MLI_NONE = 236,
	MLI_BL = 150,
	MLI_PLAY = 162,
	MLI_STOP = 170,
	MLI_NEXT = 220,
	MLI_PREV = 180,
	MLI_KP_MINUS = 226,
	MLI_MUTE = 160,

	MLI_ESC = 0,
	MLI_F1 = 10,
	MLI_F2 = 20,
	MLI_F3 = 30,
	MLI_F4 = 40,
	MLI_F5 = 50,
	MLI_F6 = 60,
	MLI_F7 = 70,
	MLI_F8 = 80,
	MLI_F9 = 90,
	MLI_F10 = 100,
	MLI_F11 = 110,
	MLI_F12 = 120,
	MLI_PSR = 140,
	MLI_SCRLOCK = 154,
	MLI_PAUSE = 152,

	MLI_TILDE = 2,
	MLI_1 = 12,
	MLI_2 = 22,
	MLI_3 = 32,
	MLI_4 = 42,
	MLI_5 = 52,
	MLI_6 = 62,
	MLI_7 = 72,
	MLI_8 = 82,
	MLI_9 = 92,
	MLI_0 = 102,
	MLI_UNDERSCORE = 112,
	MLI_PLUS = 122,

	MLI_TAB = 4,
	MLI_Q = 14,
	MLI_W = 24,
	MLI_E = 34,
	MLI_R = 44,
	MLI_T = 54,
	MLI_Y = 64,
	MLI_U = 74,
	MLI_I = 84,
	MLI_O = 94,
	MLI_P = 104,
	MLI_OPEN_BRACKET = 114,
	MLI_CLOSE_BRACKET = 124,
	MLI_OPEN_BACKSLASH = 134,

	MLI_CAPS_LOCK = 6,
	MLI_A = 16,
	MLI_S = 26,
	MLI_D = 36,
	MLI_F = 46,
	MLI_G = 56,
	MLI_H = 66,
	MLI_J = 76,
	MLI_K = 86,
	MLI_L = 96,
	MLI_COLON = 106,
	MLI_QUOTE = 116,
	MLI_ENTER = 136,

	MLI_LSHIFT = 8,
	MLI_Z = 28,
	MLI_X = 38,
	MLI_C = 48,
	MLI_V = 58,
	MLI_B = 68,
	MLI_N = 78,
	MLI_M = 88,
	MLI_COMMA = 98,
	MLI_DOT = 108,
	MLI_SLASH = 118,
	MLI_RSHIFT = 138,

	MLI_LCTRL = 190,
	MLI_LGUI = 192,
	MLI_LALT = 194,
	MLI_SPACEBAR = 198,
	MLI_RALT = 202,
	MLI_RGUI = 204,
	MLI_FN = 206,
	MLI_RCTRL = 208,
	MLI_LEFT = 210,

	MLI_BACKSPACE = 132,
	MLI_INSERT = 142,
	MLI_HOME = 146,
	MLI_PAGEUP = 156,
	MLI_NUM_LOCK = 166,
	MLI_KP_DIVIDE = 176,
	MLI_KP_MULTIPLY = 186,
	MLI_KP_PLUS = 224,
	MLI_DELETE = 144,
	MLI_END = 148,
	MLI_PAGEDOWN = 158,
	MLI_KP_7 = 164,
	MLI_KP_8 = 174,
	MLI_KP_9 = 184,
	MLI_KP_4 = 168,
	MLI_KP_5 = 172,

	MLI_RIGHT = 216,
	MLI_KP_0 = 218,
	MLI_KP_DOT = 228,
	MLI_KP_ENTER = 222,
	MLI_KP_1 = 238,
	MLI_KP_2 = 178,
	MLI_KP_6 = 182,
	MLI_KP_3 = 188,
	MLI_DOWN = 214,
	MLI_UP = 212,
};

typedef struct
{
	uint32_t nextTick;
	uint8_t step;
	uint8_t x;
	uint8_t y;
	uint8_t dotIndex;
}WaveEffectTypeDef;

	enum BRIGHTNESS_VALS
	{
		BL_VAL_0 = 0,
		BL_VAL_1 = 20,
		BL_VAL_2 = 55,
		BL_VAL_3 = 115,
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
	extern uint8_t smearLight;

	extern const unsigned char KEYBOARD_LED_Map[MAX_COL][MAX_ROW];

	extern SPI_HandleTypeDef hspi2;
	extern DMA_HandleTypeDef hdma_spi2_tx;
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

	void MatrixEffectTimer(uint32_t timeTick);

	void MatrixEffectNextMove(WaveEffectTypeDef* pEffect, uint32_t timeTick);

	void SpiTransmit(uint8_t* pData, uint16_t len);

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
