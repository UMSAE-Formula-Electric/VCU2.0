/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IWDG_RELOAD_PERIOD 4094
#define BT_DUMP_TASK_ENABLED 1
#define APPS_PROC_TASK_ENABLED 1
#define IWDG_EVENT_ALL_ALLOWED_BITS 0xFFFFFF
#define ACU_HRTBEAT_TASK_ENABLED 1
#define VCU_STATE_TASK_ENABLED 1
#define DASH_LED_TASK_ENABLED 1
#define DEFAULT_TASK_ENABLED 0
#define MC_CAN_COMMS_TASK_ENABLED 1
#define CAN_TX_TASK_ENABLED 1
#define MC_HRTBEAT_TASK_ENABLED 1
#define CAN_RX_TASK_ENABLED 1
#define WATCH_DOG_TASK_ENABLED 1
#define ACU_CAN_COMMS_TASK_ENABLED 1
#define RTD_LED2_BLUE_Pin GPIO_PIN_2
#define RTD_LED2_BLUE_GPIO_Port GPIOE
#define TSA_LED2_GREEN_Pin GPIO_PIN_3
#define TSA_LED2_GREEN_GPIO_Port GPIOE
#define RTD_LED1_GREEN_Pin GPIO_PIN_4
#define RTD_LED1_GREEN_GPIO_Port GPIOE
#define TSA_LED1_BLUE_Pin GPIO_PIN_5
#define TSA_LED1_BLUE_GPIO_Port GPIOE
#define APPS1_Pin GPIO_PIN_0
#define APPS1_GPIO_Port GPIOC
#define BPS_Pin GPIO_PIN_2
#define BPS_GPIO_Port GPIOC
#define LC_LED_Pin GPIO_PIN_0
#define LC_LED_GPIO_Port GPIOA
#define TC_LED_Pin GPIO_PIN_1
#define TC_LED_GPIO_Port GPIOA
#define APPS2_Pin GPIO_PIN_5
#define APPS2_GPIO_Port GPIOA
#define VBATT_Pin GPIO_PIN_7
#define VBATT_GPIO_Port GPIOA
#define TRACTION_BTN_Pin GPIO_PIN_5
#define TRACTION_BTN_GPIO_Port GPIOC
#define TSA_BTN_Pin GPIO_PIN_0
#define TSA_BTN_GPIO_Port GPIOB
#define RTD_BTN_Pin GPIO_PIN_1
#define RTD_BTN_GPIO_Port GPIOB
#define LAUNCH_BTN_Pin GPIO_PIN_2
#define LAUNCH_BTN_GPIO_Port GPIOB
#define VCU_LED_BLUE_Pin GPIO_PIN_8
#define VCU_LED_BLUE_GPIO_Port GPIOE
#define VCU_LED_GREEN_Pin GPIO_PIN_9
#define VCU_LED_GREEN_GPIO_Port GPIOE
#define VCU_LED_RED_Pin GPIO_PIN_10
#define VCU_LED_RED_GPIO_Port GPIOE
#define SHUTDOWN_CTL_Pin GPIO_PIN_12
#define SHUTDOWN_CTL_GPIO_Port GPIOE
#define BT_RESET_Pin GPIO_PIN_10
#define BT_RESET_GPIO_Port GPIOD
#define BT_SW_BTN_Pin GPIO_PIN_13
#define BT_SW_BTN_GPIO_Port GPIOD
#define BT_P2_4_Pin GPIO_PIN_14
#define BT_P2_4_GPIO_Port GPIOD
#define BT_P2_0_Pin GPIO_PIN_15
#define BT_P2_0_GPIO_Port GPIOD
#define SHUTDOWN_TAP_Pin GPIO_PIN_6
#define SHUTDOWN_TAP_GPIO_Port GPIOD
#define BT_EAN_Pin GPIO_PIN_7
#define BT_EAN_GPIO_Port GPIOD
#define SENSOR_CAN_RX_Pin GPIO_PIN_5
#define SENSOR_CAN_RX_GPIO_Port GPIOB
#define SENSOR_CAN_TX_Pin GPIO_PIN_6
#define SENSOR_CAN_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SAFETY_LOOP_TAP_Pin	SHUTDOWN_TAP_Pin
#define SAFETY_LOOP_TAP_GPIO_Port SHUTDOWN_TAP_GPIO_Port
#define SAFETY_LOOP_CTL_Pin SHUTDOWN_CTL_Pin
#define SAFETY_LOOP_CTL_GPIO_Port SHUTDOWN_CTL_GPIO_Port
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
