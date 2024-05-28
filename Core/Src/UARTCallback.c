/*
 * UARTCallback.c
 *
 *  Created on: May 27, 2020
 *      Author: Alejandro Mera
 */

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "main.h"
#include "modbus.h"


modbusRingBuffer_t RingBuffer;


/**
 * @brief
 * This is the callback for HAL interrupts of UART TX used by Modbus library.
 * This callback is shared among all UARTS, if more interrupts are used
 * user should implement the correct control flow and verification to maintain
 * Modbus functionality.
 * @ingroup UartHandle UART HAL handler
 */
//uint8_t Com3[255];
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	/* Modbus RTU TX callback BEGIN */
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//	int i;
//	for (i = 0; i < numberHandlers; i++ )
//	{
//	   	if (mHandlers[i]->port == huart  )
//	   	{
//	   		// notify the end of TX
//	   		xTaskNotifyFromISR(mHandlers[i]->myTaskModbusAHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
//	   		break;
//	   	}
//
//	}
//	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//
//	/* Modbus RTU TX callback END */
//
//	/*
//	 * Here you should implement the callback code for other UARTs not used by Modbus
//	 *
//	 * */
//
//}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Modbus RTU RX callback BEGIN */
    if (UartHandle->Instance == USART1){

		RingAdd(&ctx->Buffer, dataRX);
		HAL_UART_Receive_IT(&huart1, &dataRX, 1);
		osSemaphoreRelease(ctx->mSem);
    }
    if (UartHandle->Instance == USART2){

		RingAdd(&ctx2->Buffer, dataRX2);
		HAL_UART_Receive_IT(&huart2, &dataRX2, 1);
		osSemaphoreRelease(ctx2->mSem);
	}
	/* Modbus RTU RX callback END */

	/*
	 * Here you should implement the callback code for other UARTs not used by Modbus
	 *
	 *
	 * */

}


