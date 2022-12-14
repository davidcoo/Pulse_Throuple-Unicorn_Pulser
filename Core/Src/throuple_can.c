/*
 * throuple_can.c
 *
 *  Created on: Nov 22, 2022
 *      Author: David Cooper
 */


#include "cmsis_os.h"
#include "throuple_can.h"
#include <string.h>

void can_init();
void can_send();



static void MX_CAN_Init(void);
static void Error_Handler(void);

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PFP */
uint32_t TxMailbox;
uint8_t TxData[8];
uint8_t RxData[8];
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

QueueHandle_t xQueueCANRx;

void can_init(){
	MX_CAN_Init();
	xQueueCANRx = xQueueCreate( 10,sizeof(can_msg_t));
}

// Interrupt Handler for receiving a can message.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
 	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	can_msg_t msg;
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	if (RxHeader.DLC !=0)
	{
		msg.id = RxHeader.StdId;
		memcpy(msg.msg, RxData, 8);
		if (msg.id == 0x100){
			 set_blinkers(msg.msg[3],msg.msg[4],msg.msg[5]);
		}
		BaseType_t res = xQueueSendFromISR(xQueueCANRx,(void * )&msg, &xHigherPriorityTaskWoken );
		if (res == pdTRUE){
			res = 0;
		}
	}
	if( xHigherPriorityTaskWoken )
	{
	        /* Actual macro used here is port specific. */

	}
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0;  // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0x200<<5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0x100<<5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDLIST;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_16BIT;
	canfilterconfig.SlaveStartFilterBank = 14;  // doesn't matter in single can controllers

	if (HAL_CAN_ConfigFilter(&hcan, &canfilterconfig)!= HAL_OK){
		// Filter Config Error
		Error_Handler();
	}

	// Start CAN
	if (HAL_CAN_Start(&hcan) != HAL_OK)
	{
	  /* Start Error */
	  Error_Handler();
	}
	// Activate RX Notification
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)!= HAL_OK){
		// Notification Error
		Error_Handler();
	}

  /* USER CODE END CAN_Init 2 */
}


CAN_TxHeaderTypeDef	TxHeader;
uint32_t TxMailbox;
uint8_t real_data[8];
void throuple_can_tx(uint16_t id, uint8_t * data){

	TxHeader.DLC = 8;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = id;

	HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox);
}


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}


