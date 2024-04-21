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

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

#define CAN_MC_QUEUE_LENGTH 		64
#define CAN_AMS_QUEUE_LENGTH 		64
#define CAN_ACB_VCU_QUEUE_LENGTH 	64
#define CAN_QUEUE_LENGTH 			64

#define CAN_QUEUE_ITEM_SIZE sizeof( CanRxMsg )

#define CANTXTIMEOUT 			10 //number of milleseconds to wait for the can packet to send
#define CANTXMBTIMEOUT			10 //number of retires to get a mailbox on send

#define CAN_NO_RTR 				0
#define CAN_RTR 				1

#define CAN_SUB_Q_DELAY_MS 		100

#define CAN_MC_ACTIVE_MESSAGES  0x0C1
#define CAN_MC_RESPONSE_MSG     0x0C2

#define CAN_ACU_CAN_ID			0x69
#define CAN_VCU_CAN_ID			0x88
#define CAN_SCU_CAN_ID			0x89
#define CAN_AMS_CAN_ID          0x70
#define CAN_VCU_LOG_ID			0x71 //ID for sending VCU data to ACB

#define CAN_BMS_BASE_ID         0x10

#define CAN_BMS_OVERALL_ID		CAN_BMS_BASE_ID + 0
#define CAN_BMS_VOLTAGE_ID		CAN_BMS_BASE_ID + 1
#define CAN_BMS_DIAGNOSTIC_ID	CAN_BMS_BASE_ID + 7
#define CAN_BMS_TEMP_ID			CAN_BMS_BASE_ID + 8

#define CAN_VCU_SET_ACB_STATE_ID	0x01

#define CAN_EXT 				1
#define CAN_NO_EXT 				0

/* USER CODE END Private defines */

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

enum ACB_TO_CAN_MSG{
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

uint8_t sendCan(CAN_TypeDef* CANx, uint8_t const * data, int32_t length, uint32_t dest, uint8_t isRTR, uint8_t isExtended);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

