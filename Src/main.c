/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
#include "usbhid.h"
#include "state_led.h"
#include "keyboard.h"
#include "matrix_led.h"
#include "ztask.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

const unsigned char KEYBOARD_Value_Map[MAX_COL][MAX_ROW] =
{
  {0           ,0        ,0                   ,0          ,0             ,0           ,0             ,0          ,KC_GAME  ,KC_BACKLIGHT,KC_MEDIA_PLAY ,KC_MEDIA_STOP,KC_MEDIA_SCAN_NEXT,KC_MEDIA_SCAN_PREV,KC_KP_MINUS     ,KC_MEDIA_MUTE,},
  {KC_ESCAPE   ,KC_F1    ,KC_F2               ,KC_F3      ,KC_F4         ,KC_F5       ,KC_F6         ,KC_F7      ,KC_F8    ,KC_F9       ,KC_F10        ,KC_F11       ,KC_F12            ,KC_PRINTSCREEN    ,KC_SCROLL_LOCK  ,KC_PAUSE    ,},
  {0           ,0        ,KC_TILDE            ,KC_1       ,KC_2          ,KC_3        ,KC_4          ,KC_5       ,KC_6     ,KC_7        ,KC_8          ,KC_9         ,KC_0              ,KC_UNDERSCORE     ,KC_PLUS         ,0           ,},
  {0           ,0        ,KC_TAB              ,KC_Q       ,KC_W          ,KC_E        ,KC_R          ,KC_T       ,KC_Y     ,KC_U        ,KC_I          ,KC_O         ,KC_P              ,KC_OPEN_BRACKET   ,KC_CLOSE_BRACKET,KC_BACKSLASH,},
  {0           ,0        ,KC_CAPS_LOCK        ,KC_A       ,KC_S          ,KC_D        ,KC_F          ,KC_G       ,KC_H     ,KC_J        ,KC_K          ,KC_L         ,KC_COLON          ,KC_QUOTE          ,0               ,KC_ENTER    ,},
  {0           ,0        ,KC_LSHIFT           ,0          ,KC_Z          ,KC_X        ,KC_C          ,KC_V       ,KC_B     ,KC_N        ,KC_M          ,KC_COMMA     ,KC_DOT            ,KC_SLASH          ,0               ,KC_RSHIFT   ,},
  {0           ,0        ,KC_LCTRL            ,KC_LGUI    ,KC_LALT       ,0           ,0             ,KC_SPACEBAR,0        ,0           ,0             ,KC_RALT      ,KC_RGUI           ,KC_FN             ,KC_RCTRL        ,KC_LEFT     ,},
  {KC_BACKSPACE,KC_INSERT,KC_HOME             ,KC_PAGEUP  ,KC_KP_NUM_LOCK,KC_KP_DIVIDE,KC_KP_MULTIPLY,KC_KP_PLUS ,KC_DELETE,KC_END      ,KC_PAGEDOWN   ,KC_KP_7      ,KC_KP_8           ,KC_KP_9           ,KC_KP_4         ,KC_KP_5     ,},
  {KC_RIGHT    ,KC_KP_0  ,KC_KP_DOT           ,KC_KP_ENTER,KC_KP_1       ,KC_KP_2     ,KC_KP_6       ,KC_KP_3    ,KC_DOWN  ,KC_UP       ,0             ,0            ,0                 ,0                 ,0               ,0           ,},
};


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t keyChange = 0;

struct kbReportSt kbReport;

uint8_t brightness = 0;
uint8_t gameMode = 0;
uint8_t insertEnable = 0;
volatile bool time_1ms = false;

uint16_t intPin = 0;

volatile int16_t encoderCount = 0;

int zt_bindIdEncoder = 0; //encoder task restore bind id
int zt_bindIdCfgSave = 0;
int zt_bindIdEcDebounce = 0;

typedef void (*pFunction)(void); 
pFunction JumpToApplication;
uint32_t JumpAddress;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

extern void KeyCheck(void);

void WriteEEPROM(uint32_t addr, uint32_t val);

void GameModeSw(void);
void InsertEnableSw(void);
void BrightnessSave(void);
void MediaKeyDown(uint8_t key);
void MediaKeyUp(void);
void VolumeKeyDown(uint8_t key);
void VolumeKeyUp(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void SetConfigSaveTask(void);

void ConfigSave(void);

void EncoderCheck(void);
void EncoderDebounce(void);
void DfuMode(void);
void MatrixTimer(void);
void ReportCheck(void);
void RunOfficialApp(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
		SCB->VTOR = APP_ADDR;
		//char debugBuff[64] = {'0'};

  /* USER CODE END 1 */


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

	//wait connection OK
	while(hUsbDeviceFS.dev_address == 0)
	{
		SetLogoLED(50);
	}

	//sprintf(debugBuff, "x=%u,y=%u,name=%s,val=%x\n", 1, 2, "ax", 9);
	//HAL_UART_Transmit(&huart1, (uint8_t *)debugBuff, 64, 100);

	brightness = *(uint32_t*)BL_SETTING_ADDR;
	gameMode = *(uint32_t*)MODE_SETTING_ADDR;
	insertEnable = *(uint32_t*)INSERT_SETTING_ADDR;

	SetModeLED(gameMode);

	MatrixInit();
	MatrixDisplayOn(1);

	zt_bindIdEncoder =  zt_bind(VolumeKeyUp, 20, 0);		//Volume adj key hold time
	zt_bindIdCfgSave =  zt_bind(ConfigSave, 5000, 0);   //configs save delay when changed
	zt_bindIdEcDebounce =  zt_bind(EncoderDebounce, 1, 0);
	zt_bind(EncoderCheck, 30, 1);
	zt_bind(KeyCheck, 1, 1);
	//zt_bind(ReportCheck, 10, 1);

	zt_bind(MatrixTimer, 10, 1);

  kbReport.id = 1;		//report id

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (keyChange != 0)
		{
			keyChange = 0;
			USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&kbReport, 9);

      //char debugBuff[64];
      //sprintf(debugBuff, "report modify[%u], [%u][%u][%u][%u][%u][%u]\n", kbReport.modify, kbReport.keys[0], kbReport.keys[1], kbReport.keys[2], kbReport.keys[3], kbReport.keys[4], kbReport.keys[5]);
	    //HAL_UART_Transmit(&huart1, (uint8_t *)debugBuff, 64, 100);
		}
    if (time_1ms)
    {
      time_1ms = false;
      zt_poll();    //task execute
    }    
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 31;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */
  HAL_TIM_Base_Start_IT(&htim6);
  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	HAL_HalfDuplex_EnableTransmitter(&huart1);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, COL0_Pin|COL1_Pin|COL2_Pin|COL3_Pin
                          |COL4_Pin|COL5_Pin|COL6_Pin|COL7_Pin
                          |COL8_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CAP_LED_Pin|SCR_LED_Pin|NUM_LED_Pin|MODE_LED_Pin
                          |MATRIX_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MATRIX_SS_Pin|PWR_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_EN_GPIO_Port, USB_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MATRIX_SYNC_GPIO_Port, MATRIX_SYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ROW13_Pin ROW14_Pin ROW15_Pin ROW0_Pin
                           ROW1_Pin ROW2_Pin ROW3_Pin ROW4_Pin
                           ROW5_Pin ROW6_Pin ROW7_Pin ROW8_Pin
                           ROW9_Pin ROW10_Pin ROW11_Pin ROW12_Pin */
  GPIO_InitStruct.Pin = ROW13_Pin|ROW14_Pin|ROW15_Pin|ROW0_Pin
                          |ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin
                          |ROW5_Pin|ROW6_Pin|ROW7_Pin|ROW8_Pin
                          |ROW9_Pin|ROW10_Pin|ROW11_Pin|ROW12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : COL0_Pin COL1_Pin COL2_Pin COL3_Pin
                           COL4_Pin COL5_Pin COL6_Pin COL7_Pin
                           COL8_Pin */
  GPIO_InitStruct.Pin = COL0_Pin|COL1_Pin|COL2_Pin|COL3_Pin
                          |COL4_Pin|COL5_Pin|COL6_Pin|COL7_Pin
                          |COL8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CAP_LED_Pin SCR_LED_Pin MATRIX_SS_Pin NUM_LED_Pin
                           MODE_LED_Pin MATRIX_RST_Pin PWR_EN_Pin */
  GPIO_InitStruct.Pin = CAP_LED_Pin|SCR_LED_Pin|MATRIX_SS_Pin|NUM_LED_Pin
                          |MODE_LED_Pin|MATRIX_RST_Pin|PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_EN_Pin */
  GPIO_InitStruct.Pin = USB_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MATRIX_SYNC_Pin */
  GPIO_InitStruct.Pin = MATRIX_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MATRIX_SYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EC_B_Pin EC_A_Pin */
  GPIO_InitStruct.Pin = EC_B_Pin|EC_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void WriteEEPROM(uint32_t addr, uint32_t val)
{
	HAL_FLASHEx_DATAEEPROM_Unlock();

	HAL_FLASHEx_DATAEEPROM_Erase(FLASH_TYPEPROGRAM_WORD, addr);

	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAM_WORD, addr, val);

	HAL_FLASHEx_DATAEEPROM_Lock();
}

void GameModeSw(void)
{
	gameMode = gameMode > 0 ? 0 : 1;

	SetModeLED(gameMode);
	//WriteEEPROM(MODE_SETTING_ADDR, gameMode);
  SetConfigSaveTask();
}

void InsertEnableSw(void)
{
	insertEnable = insertEnable > 0 ? 0 : 1;

	//WriteEEPROM(INSERT_SETTING_ADDR, insertEnable);
  SetConfigSaveTask();
}

void BrightnessSave(void)
{
  //WriteEEPROM(BL_SETTING_ADDR, brightness);
  SetConfigSaveTask();
}

void EncoderCheck(void)
{
  if (encoderCount != 0 )
  {
    if (encoderCount > 0)
    {
      VolumeKeyDown(KC_MEDIA_VOLUME_UP_VAL);
    }
    else if (encoderCount < 0)
    {
      VolumeKeyDown(KC_MEDIA_VOLUME_DOWN_VAL);
    }
    encoderCount = 0;
    zt_start(zt_bindIdEncoder, 1);
  }
}

void EncoderDebounce(void)
{
  zt_stop(zt_bindIdEcDebounce);

  GPIO_PinState pinA = HAL_GPIO_ReadPin(EC_A_GPIO_Port, EC_A_Pin);
  GPIO_PinState pinB = HAL_GPIO_ReadPin(EC_B_GPIO_Port, EC_B_Pin);

  if (intPin == EC_A_Pin)
  {
    if (pinB == GPIO_PIN_SET)
    {
      encoderCount++;
    }
    else
    {
      encoderCount--;
    }
  }
  if (intPin == EC_B_Pin)
  {
    if (pinA == GPIO_PIN_SET)
    {
      encoderCount++;
    }
    else
    {
      encoderCount--;
    }
  }
  intPin = 0;
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void DfuMode(void)
{
  //soft reset
  HAL_RCC_DeInit();
  HAL_DeInit();
	SCB->VTOR = (FLASH_BASE | 0x0000);
	NVIC_SystemReset();
}

void MatrixTimer(void)
{
  MatrixEffectTimer(HAL_GetTick());
}

void MediaKeyDown(uint8_t key)
{
	uint8_t report[3];
  report[0]= HID_MEDIA_REPORT;
  report[1]= key;
  report[2]= 0x00;
  USBD_HID_SendReport(&hUsbDeviceFS, report, 3);
}

void ReportCheck(void)
{
  //if (keyChange != 0)
	{
		keyChange = 0;
		kbReport.id = 1;		//report id
  	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&kbReport, 9);
    //char debugBuff[64];
    //sprintf(debugBuff, "report modify[%u], [%u][%u][%u][%u][%u][%u]\n", kbReport.modify, kbReport.keys[0], kbReport.keys[1], kbReport.keys[2], kbReport.keys[3], kbReport.keys[4], kbReport.keys[5]);
    //HAL_UART_Transmit(&huart1, (uint8_t *)debugBuff, 64, 100);
	  //HAL_Delay(10); //important TODO fixme
	}
}

void RunOfficialApp(void)
{
#if 0
  JumpAddress = *(__IO uint32_t*)(OFFICIAL_ADDR +4);
	JumpToApplication = (pFunction) JumpAddress;
					
	__set_MSP(*(__IO uint32_t*)OFFICIAL_ADDR);
	JumpToApplication();
  while (1); // never reached
#else
  uint32_t appStack;
	pFunction appEntry;
 
	//__disable_irq();		//won't work when this execute 
  //HAL_NVIC_ClearPendingIRQ(SysTick_IRQn);
 
	// get the application stack pointer (1st entry in the app vector table)
	appStack = (uint32_t)*((__IO uint32_t*)OFFICIAL_ADDR);
 
	// Get the app entry point (2nd entry in the app vector table
	appEntry = (pFunction)*(__IO uint32_t*)(OFFICIAL_ADDR + 4);
 
	HAL_RCC_DeInit();
	HAL_DeInit();
 
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL  = 0;
 
	// Reconfigure vector table offset to match the app location
	SCB->VTOR = OFFICIAL_ADDR;
	__set_MSP(appStack); // Set app stack pointer
	appEntry(); // Start the app
 
	while (1); // never reached
  
#endif
}

void MediaKeyUp(void)
{
	uint8_t report[3];
  report[0]= HID_MEDIA_REPORT;
  report[1]= 0x00;
  report[2]= 0x00;
  USBD_HID_SendReport(&hUsbDeviceFS, report, 3);
}

void VolumeKeyDown(uint8_t key)
{
	MediaKeyDown(key);
	//HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
}

void VolumeKeyUp(void)
{
  MediaKeyUp();
  zt_stop(zt_bindIdEncoder);
  //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void SetConfigSaveTask(void)
{
  zt_start(zt_bindIdCfgSave, 1);
}

void ConfigSave(void)
{
  zt_stop(zt_bindIdCfgSave);
  WriteEEPROM(MODE_SETTING_ADDR, gameMode);
  WriteEEPROM(INSERT_SETTING_ADDR, insertEnable);
  WriteEEPROM(BL_SETTING_ADDR, brightness);
}

void USBD_HID_GetReport(uint8_t * report, int len)
{
  // see from http://www.microchip.com/forums/m433757.aspx
  // report[0] is the report id
  // report[1] is the led bit filed
  if (report[0] == 1)
	{
		// report id 1 is "led" refer to report id in hid report des
    SetNumLockLED(report[1] & 0x01);
		SetCapsLockLED(report[1] & 0x02);
		SetScrollLockLED(report[1] & 0x04);
  }
}

//encoder interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  GPIO_PinState pinA = HAL_GPIO_ReadPin(EC_A_GPIO_Port, EC_A_Pin);
  GPIO_PinState pinB = HAL_GPIO_ReadPin(EC_B_GPIO_Port, EC_B_Pin);

  if (GPIO_Pin == EC_A_Pin && pinA != GPIO_PIN_SET)
  {
    //not rising
    return;
  }
  if (GPIO_Pin == EC_B_Pin && pinB != GPIO_PIN_RESET)
  {
    //not falling
    return;
  }

  intPin = GPIO_Pin;

  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  zt_start(zt_bindIdEcDebounce, 1);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim6.Instance)
  {
      zt_tick();
      time_1ms = true;
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
		/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
		/* User can add his own implementation to report the file name and line number,
		   tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
