/**
  ******************************************************************************
  * @file    usbd_hid.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header file for the usbd_hid_core.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_HID_H
#define __USB_HID_H

#ifdef __cplusplus
extern "C" {
#endif

	/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

  /** @defgroup USBD_HID
	* @brief This file is the Header file for usbd_hid.c
	* @{
	*/

	/** set this to 0/1 if to have led or not
	 * it can be defined in the build configuraton symbol */
#ifndef  HID_LED_SUPPORT
#define HID_LED_SUPPORT 1
#endif


#ifndef  HID_NKRO_SUPPORT
#define HID_NKRO_SUPPORT 1

#define EP_DOUBLE_BUFFER	0x06
#define NKRO_INTERFACE	1
#define NKRO_ENDPOINT_NUM 	1
#define NKRO_ENDPOINT_ADDR 	0x02
#define NKRO_SIZE     	24
#define NKRO_BUFFER   	EP_DOUBLE_BUFFER
#define NKRO_KEYS     	(NKRO_SIZE - 1)

#endif


	 /**  define  VOLUME_REPORT  to the report ID to use for volume or 0 if not support*/
#define  HID_MEDIA_REPORT  2
#ifdef   HID_MEDIA_REPORT
#   define HID_MEDIA_SIZE    25
#else
#   define HID_MEDIA_SIZE   0
#endif

#if HID_MEDIA_REPORT == 1
#     error "volume report can't be 1 already used for stad report"
#endif

/** @defgroup USBD_HID_Exported_Defines
  * @{
  */
#define HID_EPIN_ADDR                 0x81
#define HID_EPIN_SIZE					32

#define HID_EPOUT_ADDR             0x01
#define HID_EPOUT_SIZE                0x08

  //NKRO 报告
#define HID_EPIN_ADDR_2             0x82
#define HID_EPIN_SIZE_2				 26

#define NKRO_REPORT_ID 3

#if  HID_LED_SUPPORT
#   define CFG_ADD_SIZE                  7 /* one ep */
#   define HID_NUM_EP   3
//#   define HID_NUM_EP   1//NKRO
#else
#   define CFG_ADD_SIZE                  0
#   define HID_NUM_EP   1
#endif

#if HID_LED_SUPPORT
#   define HID_LED_SIZE    18
#else
#   define HID_LED_SIZE    0
#endif


#if HID_NKRO_SUPPORT
#   define HID_NKRO_SIZE    (59-18)// - 18		//18=led state in nkro
#   define HID_NKRO_CFG_DESC_SIZE    7
#else
#   define HID_NKRO_SIZE    0
#   define HID_NKRO_CFG_DESC_SIZE    0
#endif



#define USB_HID_CONFIG_DESC_SIZ       (HID_NKRO_CFG_DESC_SIZE + 34 + CFG_ADD_SIZE)
#define USB_HID_DESC_SIZ              9


#define HID_REPORT_DESC_SIZE    (47 + HID_MEDIA_SIZE + HID_LED_SIZE + HID_NKRO_SIZE)


#define HID_DESCRIPTOR_TYPE           0x21
#define HID_REPORT_DESC               0x22

//#define HID_HS_BINTERVAL               0x07
#define HID_HS_BINTERVAL               0x01 //lingex

#ifndef  HID_FS_BINTERVAL
#define HID_FS_BINTERVAL               0x0A

#endif
#define HID_POLLING_INTERVAL           0x0A

#define HID_REQ_SET_PROTOCOL          0x0B
#define HID_REQ_GET_PROTOCOL          0x03

#define HID_REQ_SET_IDLE              0x0A
#define HID_REQ_GET_IDLE              0x02

#define HID_REQ_SET_REPORT            0x09
#define HID_REQ_GET_REPORT            0x01
/**
  * @}
  */


  /** @defgroup USBD_CORE_Exported_TypesDefinitions
	* @{
	*/
	typedef enum
	{
		HID_IDLE = 0,
		HID_BUSY,
	}
	HID_StateTypeDef;


	typedef struct
	{
		uint32_t             Protocol;
		uint32_t             IdleState;
		uint32_t             AltSetting;
		HID_StateTypeDef     state;
	}
	USBD_HID_HandleTypeDef;
	/**
	  * @}
	  */



	  /** @defgroup USBD_CORE_Exported_Macros
		* @{
		*/

		/**
		  * @}
		  */

		  /** @defgroup USBD_CORE_Exported_Variables
			* @{
			*/

	extern USBD_ClassTypeDef  USBD_HID;
#define USBD_HID_CLASS    &USBD_HID
	/**
	  * @}
	  */

	  /** @defgroup USB_CORE_Exported_Functions
		* @{
		*/
	uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev,
		uint8_t *report,
		uint16_t len);

	uint8_t USBD_HID_SendNkroReport(USBD_HandleTypeDef *pdev,
		uint8_t *report,
		uint16_t len);

	uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef *pdev);


	/** user redifinable */
	void USBD_HID_GetReport(uint8_t * OutData, int len);
	/**
	  * @}
	  */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_HID_H */
/**
  * @}
  */

  /**
	* @}
	*/

	/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
