/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ROW13_Pin GPIO_PIN_13
#define ROW13_GPIO_Port GPIOC
#define ROW14_Pin GPIO_PIN_14
#define ROW14_GPIO_Port GPIOC
#define ROW15_Pin GPIO_PIN_15
#define ROW15_GPIO_Port GPIOC
#define ROW0_Pin GPIO_PIN_0
#define ROW0_GPIO_Port GPIOC
#define ROW1_Pin GPIO_PIN_1
#define ROW1_GPIO_Port GPIOC
#define ROW2_Pin GPIO_PIN_2
#define ROW2_GPIO_Port GPIOC
#define ROW3_Pin GPIO_PIN_3
#define ROW3_GPIO_Port GPIOC
#define COL0_Pin GPIO_PIN_0
#define COL0_GPIO_Port GPIOA
#define COL1_Pin GPIO_PIN_1
#define COL1_GPIO_Port GPIOA
#define COL2_Pin GPIO_PIN_2
#define COL2_GPIO_Port GPIOA
#define COL3_Pin GPIO_PIN_3
#define COL3_GPIO_Port GPIOA
#define COL4_Pin GPIO_PIN_4
#define COL4_GPIO_Port GPIOA
#define COL5_Pin GPIO_PIN_5
#define COL5_GPIO_Port GPIOA
#define COL6_Pin GPIO_PIN_6
#define COL6_GPIO_Port GPIOA
#define COL7_Pin GPIO_PIN_7
#define COL7_GPIO_Port GPIOA
#define ROW4_Pin GPIO_PIN_4
#define ROW4_GPIO_Port GPIOC
#define ROW5_Pin GPIO_PIN_5
#define ROW5_GPIO_Port GPIOC
#define CAP_LED_Pin GPIO_PIN_0
#define CAP_LED_GPIO_Port GPIOB
#define SCR_LED_Pin GPIO_PIN_1
#define SCR_LED_GPIO_Port GPIOB
#define MATRIX_SS_Pin GPIO_PIN_2
#define MATRIX_SS_GPIO_Port GPIOB
#define NUM_LED_Pin GPIO_PIN_10
#define NUM_LED_GPIO_Port GPIOB
#define MODE_LED_Pin GPIO_PIN_11
#define MODE_LED_GPIO_Port GPIOB
#define MATRIX_SCK_Pin GPIO_PIN_13
#define MATRIX_SCK_GPIO_Port GPIOB
#define MATRIX_MISO_Pin GPIO_PIN_14
#define MATRIX_MISO_GPIO_Port GPIOB
#define MATRIX_MOSI_Pin GPIO_PIN_15
#define MATRIX_MOSI_GPIO_Port GPIOB
#define ROW6_Pin GPIO_PIN_6
#define ROW6_GPIO_Port GPIOC
#define ROW7_Pin GPIO_PIN_7
#define ROW7_GPIO_Port GPIOC
#define ROW8_Pin GPIO_PIN_8
#define ROW8_GPIO_Port GPIOC
#define ROW9_Pin GPIO_PIN_9
#define ROW9_GPIO_Port GPIOC
#define COL8_Pin GPIO_PIN_8
#define COL8_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define USB_EN_Pin GPIO_PIN_15
#define USB_EN_GPIO_Port GPIOA
#define ROW10_Pin GPIO_PIN_10
#define ROW10_GPIO_Port GPIOC
#define ROW11_Pin GPIO_PIN_11
#define ROW11_GPIO_Port GPIOC
#define ROW12_Pin GPIO_PIN_12
#define ROW12_GPIO_Port GPIOC
#define MATRIX_SYNC_Pin GPIO_PIN_2
#define MATRIX_SYNC_GPIO_Port GPIOD
#define MATRIX_RST_Pin GPIO_PIN_3
#define MATRIX_RST_GPIO_Port GPIOB
#define LOGO_LED_Pin GPIO_PIN_5
#define LOGO_LED_GPIO_Port GPIOB
#define PWR_EN_Pin GPIO_PIN_7
#define PWR_EN_GPIO_Port GPIOB
#define EC_B_Pin GPIO_PIN_8
#define EC_B_GPIO_Port GPIOB
#define EC_A_Pin GPIO_PIN_9
#define EC_A_GPIO_Port GPIOB
#define EC_A_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

#define COL_GPIO_Port GPIOA
#define ROW_GPIO_Port GPIOC

#define REPORT_SIZE 9

#define MAX_COL		9
#define MAX_ROW		16

#define APP_ADDR  (FLASH_BASE | 0x3000)

#define BL_SETTING_ADDR				  (FLASH_EEPROM_BASE + 0x00)
#define MODE_SETTING_ADDR			  (FLASH_EEPROM_BASE + 0x04)
#define INSERT_SETTING_ADDR		  (FLASH_EEPROM_BASE + 0x08)


struct kbReportSt
{
  uint8_t id;
  uint8_t modify;
  uint8_t reserved;
  uint8_t keys[6];
};
extern struct kbReportSt kbReport;

enum MEDIA_KEY_STATE
{
	MK_STATE_NONE = 0,
	MK_STATE_DOWN,
	MK_STATE_REPORTED,
	MK_STATE_UP,
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
