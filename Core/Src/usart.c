/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

extern osThreadId_t USARTRxPacketQueue;
extern osThreadId_t USARTTxPacketQueue;

/* USER CODE END 0 */

USART_HandleTypeDef husart2;

/* USART2 init function */
void MX_USART2_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_TX_RX;
  husart2.Init.CLKPolarity = USART_POLARITY_LOW;
  husart2.Init.CLKPhase = USART_PHASE_1EDGE;
  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_USART_MspInit(USART_HandleTypeDef* usartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(usartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    PA4     ------> USART2_CK
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
}

void HAL_USART_MspDeInit(USART_HandleTypeDef* usartHandle)
{

  if(usartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    PA4     ------> USART2_CK
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);

  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void StartUSARTTxTask(void *argument){
	char * currMsg = NULL;

	for(;;){
		if(uxQueueMessagesWaiting( USARTRxPacketQueue ) != 0){
			xQueueReceive( USARTRxPacketQueue, currMsg, pdMS_TO_TICKS( USART_DELAY ) );
			HAL_USART_Transmit(&husart2, currMsg, strlen(currMsg), pdMS_TO_TICKS( USART_DELAY ) );
		}
	}
}

uint8_t USART2_Transmit(char* msg){
	//Check if usart2 is initialized
	if(husart2.Instance != USART2) return USART_DNE;

	//Check if queue exists
	if(!USARTTxPacketQueue) return USART_DNE;

	//Try Pushing to Queue
	int retval = xQueueSend(USARTTxPacketQueue, msg, pdMS_TO_TICKS(USART_DELAY));

	if(retval != 1) return USART_ERROR;

	return USART_OK;
}


//TODO Bluetooth
//uint8_t usartBTSend( char * Data){
//    BaseType_t retRTOS = 0;
//    uint32_t ulNotifiedValue;
//    uint8_t ret = 0;
//    uint32_t count = 0;
//    retRTOS = xSemaphoreTake( BLUTOOTH_USART.sem,  pdMS_TO_TICKS(USART_TIMEOUT));
//    if(retRTOS == pdPASS){
//        BLUTOOTH_USART.printHandle = xTaskGetCurrentTaskHandle();
//        //USARTx->curPrintingTask = xTaskGetCurrentTaskHandle();
//        bool end = false;
//        while(!end){
//            if(Data[count] == BT_END_FLAG) {
//                end = true;
//            }
//            USART_SendData(BLUTOOTH_USART.usart, Data[count]);
//            count++;
//
//            if(BLUTOOTH_USART.usart == USART3){
//                BLUTOOTH_USART.txEnabled = true;
//            }
//            USART_ITConfig(BLUTOOTH_USART.usart, USART_IT_TXE, ENABLE);
//
//            retRTOS = xTaskNotifyWait(0x00,0x00, &ulNotifiedValue, pdMS_TO_TICKS(USART_TIMEOUT));
//            //check if timeout
//            if(retRTOS != pdTRUE){
//                ret = 1;
//            }
//        }
//        xSemaphoreGive(BLUTOOTH_USART.sem);
//    }
//    else{
//        ret = 1;
//    }
//    return ret;
//}

/* USER CODE END 1 */
