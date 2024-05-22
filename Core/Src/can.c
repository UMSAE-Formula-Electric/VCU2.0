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
#include "usart.h"
#include "motor_controller_can_utils.h"
#include "logger.h"
#include "cmsis_os2.h"
#include "iwdg.h"
#include "acu_comms_handler.h"
#include "mc_comms_handler.h"

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
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
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

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
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

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxPacketTypeDef rxPacket;

    if (!(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &(rxPacket.rxPacketHeader), rxPacket.rxPacketData) == HAL_OK &&
            osMessageQueuePut(canRxPacketQueueHandle, &rxPacket, 0, 0) == osOK)) {
        uint32_t currQueueSize = osMessageQueueGetCount(canRxPacketQueueHandle);
        uint32_t maxQueueCapacity = osMessageQueueGetCapacity(canRxPacketQueueHandle);

        if (currQueueSize == maxQueueCapacity) {  /* Queue is full */
            sendToUsart("Error adding received message to the CAN Rx queue because the queue is full.\r\n", true);
        }
        else {  /* Error receiving message from CAN */
            sendToUsart("Error receiving message from the CAN Bus and adding it to the Rx queue.\r\n", true);
        }
        Error_Handler();
    }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    sendToUsart("CAN Rx FIFO0 is full.\r\n", true);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t canError = HAL_CAN_GetError(hcan);
    if (canError != HAL_CAN_ERROR_NONE) {
        sendToUsart("CAN ERROR CAN ERROR CAN ERROR!!\r\n", true);
    }
}

void messageReceivedFromControlUnit(const char *unitType) {
    char canMsg[50];
    if (strcmp(unitType, "VCU") == 0) strncpy(canMsg, "VCU received a CAN message from the VCU.\r\n", sizeof(canMsg) - 1);
    else if (strcmp(unitType, "ACU") == 0) strncpy(canMsg, "VCU received a CAN message from the ACU.\r\n", sizeof(canMsg) - 1);
    else if (strcmp(unitType, "SCU") == 0) strncpy(canMsg, "VCU received a CAN message from the SCU.\r\n", sizeof(canMsg) - 1);
    sendToUsart(canMsg, true);
}

void StartCanRxTask(void *argument)
{
    uint8_t isTaskActivated = (int)argument;
    if (isTaskActivated == 0) {
        osThreadTerminate(osThreadGetId());
    }

    if (!(HAL_CAN_Start(&hcan1) == HAL_OK && HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_RX_FIFO0_FULL | CAN_IT_ERROR) == HAL_OK))
    {
        Error_Handler();
    }

    CAN_RxPacketTypeDef rxPacket;
    osStatus_t isMsgTakenFromQueue;

    for (;;)
    {
        kickWatchdogBit(osThreadGetId());

        isMsgTakenFromQueue = osMessageQueueGet(canRxPacketQueueHandle, &rxPacket, 0, 0);
        if (isMsgTakenFromQueue == osOK)
        {
            if (rxPacket.rxPacketHeader.IDE == CAN_ID_EXT)
            {
                switch (rxPacket.rxPacketHeader.ExtId)
                {}
            }
            if (rxPacket.rxPacketHeader.IDE == CAN_ID_STD)
            {
                switch (rxPacket.rxPacketHeader.StdId)
                {
                    case CAN_ACU_TO_VCU_ID:
                        processAcuToVcuCanIdRxData(rxPacket.rxPacketData);
                        break;
                    case CAN_ACU_CAN_ID:
                        messageReceivedFromControlUnit("ACU");
                        break;
                    case CAN_SCU_CAN_ID:
                        messageReceivedFromControlUnit("SCU");
                        break;

                    case CAN_MC_RX_HIGHSPEED: //High speed message, 333Hz
                        notify_mc_heartbeat_task();
                        mc_process_fast_can(rxPacket.rxPacketData);
                        break;

                        //Motor Controller Messages
                    case CAN_MC_RX_TEMP1_ID: //IGBT temp readings
                        mc_process_temp1_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_TEMP2_ID:
                        mc_process_temp2_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_ANALOG_INPUTS_VOLTAGE:
                        mc_process_analog_inputs_voltage_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_DIGITAL_INPUT_STATUS:
                        mc_process_digital_input_status_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_MOTOR_ID:
                        mc_process_motor_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_CURRENT_ID:
                        mc_process_current_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_VOLT_ID:
                        mc_process_volt_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_FAULT_ID:
                        mc_process_fault_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_INTERNAL_VOLTAGES:
                        mc_process_internal_volt_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_INTERNAL_STATES:
                        mc_process_internal_states_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_TORQUE_TIMER_INFO:
                        mc_process_torque_timer_info_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_MODULATION_INDEX:
                        mc_process_modulation_index_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_FIRMWARE_INFO:
                        mc_process_firmware_info_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_DIAGNOSTIC_DATA:
                        mc_process_diagnostic_data_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_TORQUE_CAPABILITY:
                        mc_process_torque_capability_can(rxPacket.rxPacketData);
                        break;

                    case CAN_MC_RX_TEMP3_ID: //Motor temp reading
                        mc_process_temp3_can(rxPacket.rxPacketData);
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

    CAN_TxPacketTypeDef txPacket;
    osStatus_t isMsgTakenFromQueue;

    for(;;){
        kickWatchdogBit(osThreadGetId());

        isMsgTakenFromQueue = osMessageQueueGet(canTxPacketQueueHandle, &txPacket, 0, 0);
        if (isMsgTakenFromQueue == osOK) {
            if (HAL_CAN_AddTxMessage(&hcan1, &(txPacket.txPacketHeader), txPacket.txPacketData, &TxMailbox) != HAL_OK) {
                sendToUsart("VCU couldn't send a message to the CAN Bus.\r\n", true);
            }
            else {
                sendToUsart("VCU sent a message to the CAN Bus.\r\n", true);
            }
        }
    }
}
/* USER CODE END 1 */
