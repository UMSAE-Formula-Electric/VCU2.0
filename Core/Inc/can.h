/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "can_utils.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

enum ACU_TO_CAN_MSG{
    CAN_ACB_TSA_ACK = 0,
    CAN_ACB_TSA_NACK,
    CAN_ACB_RTD_ACK,
    CAN_ACB_RTD_NACK,
    CAN_GO_IDLE_REQ,  //Request to go idle
    CAN_NO_SAFETY_LOOP_SET,  //Message to VCU to indicate that the safety loop is open at the VCU. Used when the car is idle
    CAN_NO_SAFETY_LOOP_CLEAR,//Message to VCU to indicate that the safety loop is closed at the VCU. Used when the car is idle
    CAN_AIR_WELD_SET,
    CAN_HEARTBEAT_REQUEST,
    CAN_HEARTBEAT_RESPONSE,
    CAN_BATTERY_VOLTAGE_REQUEST,
    CAN_BATTERY_VOLTAGE_RESPONSE
};

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

