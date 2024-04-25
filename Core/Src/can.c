/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "motor_controller_can.h"
#include "logger.h"
#include "cmsis_os2.h"
#include "iwdg.h"
#include "ACB_comms_handler.h"

uint32_t TxMailbox;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
    CAN_FilterTypeDef sFilterConfig;

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        /* Filter configuration Error */
        Error_Handler();
    }

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void messageReceivedFromControlUnit(const char *unitType) {
    char canMsg[50];
    if (strcmp(unitType, "VCU") == 0) strncpy(canMsg, "VCU received a CAN message from the VCU.\r\n", sizeof(canMsg) - 1);
    else if (strcmp(unitType, "ACU") == 0) strncpy(canMsg, "VCU received a CAN message from the ACU.\r\n", sizeof(canMsg) - 1);
    else if (strcmp(unitType, "SCU") == 0) strncpy(canMsg, "VCU received a CAN message from the SCU.\r\n", sizeof(canMsg) - 1);
    logMessage(canMsg, true);
}

HAL_StatusTypeDef CAN_Polling(void)
{
    uint32_t isCanRxFifoFilled = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
    if (isCanRxFifoFilled < 1)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef isCanMsgReceived = HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
    if (isCanMsgReceived != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void StartCanRxTask(void *argument)
{
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadTerminate(osThreadGetId());
    }

    char canMsg[50];
    HAL_CAN_Start(&hcan1);

    for (;;)
    {
        kickWatchdogBit(osThreadGetId());

        if (CAN_Polling() == HAL_OK)
        {
            if (RxHeader.IDE == CAN_ID_EXT)
            {
                switch (RxHeader.ExtId)
                {}
            }
            if (RxHeader.IDE == CAN_ID_STD)
            {
                switch (RxHeader.StdId)
                {
                    case CAN_VCU_CAN_ID:
                        processVcuCanIdRxData(rxPacket.rxPacketData);
                        break;
                    case CAN_ACU_CAN_ID:
                        messageReceivedFromControlUnit("ACU");
                        break;
                    case CAN_SCU_CAN_ID:
                        messageReceivedFromControlUnit("SCU");
                        break;

                    case CAN_MC_RX_HIGHSPEED: //High speed message, 333Hz
                        mc_process_fast_can(RxData);
                        break;

                        //Motor Controller Messages
                    case CAN_MC_RX_TEMP1_ID: //IGBT temp readings
                        mc_process_temp1_can(RxData);
                        break;

                    case CAN_MC_RX_TEMP2_ID:
                        mc_process_temp2_can(RxData);
                        break;

                    case CAN_MC_RX_ANALOG_INPUTS_VOLTAGE:
                        mc_process_analog_inputs_voltage_can(RxData);
                        break;

                    case CAN_MC_RX_DIGITAL_INPUT_STATUS:
                        mc_process_digital_input_status_can(RxData);
                        break;

                    case CAN_MC_RX_MOTOR_ID:
                        mc_process_motor_can(RxData);
                        break;

                    case CAN_MC_RX_CURRENT_ID:
                        mc_process_current_can(RxData);
                        break;

                    case CAN_MC_RX_VOLT_ID:
                        mc_process_volt_can(RxData);
                        break;

                    case CAN_MC_RX_FAULT_ID:
                        mc_process_fault_can(RxData);
                        break;

                    case CAN_MC_RX_INTERNAL_VOLTAGES:
                        mc_process_internal_volt_can(RxData);
                        break;

                    case CAN_MC_RX_INTERNAL_STATES:
                        mc_process_internal_states_can(RxData);
                        break;

                    case CAN_MC_RX_TORQUE_TIMER_INFO:
                        mc_process_torque_timer_info_can(RxData);
                        break;

                    case CAN_MC_RX_MODULATION_INDEX:
                        mc_process_modulation_index_can(RxData);
                        break;

                    case CAN_MC_RX_FIRMWARE_INFO:
                        mc_process_firmware_info_can(RxData);
                        break;

                    case CAN_MC_RX_DIAGNOSTIC_DATA:
                        mc_process_diagnostic_data_can(RxData);
                        break;

                    case CAN_MC_RX_TORQUE_CAPABILITY:
                        mc_process_torque_capability_can(RxData);
                        break;

                    case CAN_MC_RX_TEMP3_ID: //Motor temp reading
                        mc_process_temp3_can(RxData);
                        break;

                    default:
                        break;
                }
            }
        }
    }
}

void StartCanTxTask(void *argument){
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadTerminate(osThreadGetId());
    }

    char canMsg[50];
    for(;;){
        kickWatchdogBit(osThreadGetId());

        TxHeader.IDE = CAN_ID_STD; // Using Standard ID
        TxHeader.StdId = CAN_SCU_CAN_ID;   // Transmitter's ID (11-bit wide)
        TxHeader.RTR = CAN_RTR_DATA; // Data frame
        TxHeader.DLC = 6; // Length of data bytes
        TxData[0] = 'H'; // ASCII code for 'H'
        TxData[1] = 'i'; // ASCII code for 'i'
        TxData[2] = ' '; // ASCII code for space
        TxData[3] = 'S'; // ASCII code for 'S'
        TxData[4] = 'C'; // ASCII code for 'C'
        TxData[5] = 'U'; // ASCII code for 'U'

        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        	logMessage("VCU couldn't send a message to the CAN Bus.\r\n", true);
        }
        else {
            logMessage("VCU sent a message to the CAN Bus.\r\n", true);
        }
    }
}
/* USER CODE END 1 */
