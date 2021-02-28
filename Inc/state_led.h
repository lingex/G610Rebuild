/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STATE_LED_H
#define __STATE_LED_H

#ifdef __cplusplus
extern "C" {
#endif

	/* Includes ------------------------------------------------------------------*/
#include  "main.h"

	extern TIM_HandleTypeDef htim3;
	extern void NumLockTaskActive(void);
	extern void NumLockTaskDeActive(void);


	void SetNumLockLED(uint8_t on);

	void SetCapsLockLED(uint8_t on);

	void SetScrollLockLED(uint8_t on);

	void SetModeLED(uint8_t on);

	void SetLogoLED(uint16_t brightVal);


#ifdef __cplusplus
}
#endif

#endif  /* __STATE_LED_H */
/**
  * @}
  */

  /**
	* @}
	*/
