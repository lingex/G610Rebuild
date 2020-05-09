/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __KEYBOARD_H
#define __KEYBOARD_H

#ifdef __cplusplus
extern "C"
{
#endif

	/* Includes ------------------------------------------------------------------*/
#include "main.h"
#define DEBOUNCE_MS 10

	extern uint8_t gameMode;
	extern uint8_t insertEnable;
	extern uint8_t keyChange;
	extern uint8_t mediaKeyState;
	extern uint8_t mediaKeyVal;
	extern const char *KEYBOARD_Name_Map[MAX_COL][MAX_ROW];
	extern const unsigned char KEYBOARD_Value_Map[MAX_COL][MAX_ROW];
	extern UART_HandleTypeDef huart1;

	extern void GameModeSw(void);
	extern void InsertEnableSw(void);
	extern void NumLockEnableSw(void);
	extern void MediaKeyDown(uint8_t keyVal);
	extern void DfuMode(void);

	void KeyCheck(void);
	uint32_t ReadGpioPort(GPIO_TypeDef *GPIOx);
	void KeyColPrepare(uint8_t index);

	bool IsModifyKey(uint8_t keyVal);

	void OnKeyDown(uint8_t x, uint8_t y, uint8_t keyVal);
	void OnKeyUp(uint8_t x, uint8_t y, uint8_t keyVal);

#ifdef __cplusplus
}
#endif

#endif /* __KEYBOARD_H */
/**
  * @}
  */

/**
	* @}
	*/
