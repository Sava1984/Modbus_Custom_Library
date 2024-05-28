/*
 * modbus.c
 *
 *  Created on: May 24, 2024
 *      Author: zip_s
 */
#include "main.h"
#include "modbus.h"
#include "modbus_crc.h"

void RingAdd(modbusRingBuffer_t *xRingBuffer, uint8_t u8Val)
{

	xRingBuffer->uxBuffer[xRingBuffer->u8end] = u8Val;
	xRingBuffer->u8end = (xRingBuffer->u8end + 1) % MAX_BUFFER;
	if (xRingBuffer->u8available == MAX_BUFFER)
	{
		xRingBuffer->overflow = true;
		xRingBuffer->u8start = (xRingBuffer->u8start + 1) % MAX_BUFFER;
	}
	else
	{
		xRingBuffer->overflow = false;
		xRingBuffer->u8available++;
	}

}

uint8_t RingGetAllBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer)
{
	return RingGetNBytes(xRingBuffer, buffer, xRingBuffer->u8available);
}

uint8_t RingGetNBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer, uint8_t uNumber)
{
	uint8_t uCounter;
	if(xRingBuffer->u8available == 0  || uNumber == 0 ) return 0;
	if(uNumber > MAX_BUFFER) return 0;

	for(uCounter = 0; uCounter < uNumber && uCounter< xRingBuffer->u8available ; uCounter++)
	{
		buffer[uCounter] = xRingBuffer->uxBuffer[xRingBuffer->u8start];
		xRingBuffer->u8start = (xRingBuffer->u8start + 1) % MAX_BUFFER;
	}
	xRingBuffer->u8available = xRingBuffer->u8available - uCounter;
	xRingBuffer->overflow = false;
	RingClear(xRingBuffer);

	return uCounter;
}

uint8_t RingCountBytes(modbusRingBuffer_t *xRingBuffer)
{
return xRingBuffer->u8available;
}

void RingClear(modbusRingBuffer_t *xRingBuffer)
{
xRingBuffer->u8start = 0;
xRingBuffer->u8end = 0;
xRingBuffer->u8available = 0;
xRingBuffer->overflow = false;
}
uint8_t WaitBytes(modbus *Handler,uint8_t ByteCount)
{
	osStatus_t stat;
	while (RingCountBytes(&Handler->Buffer)<ByteCount)
	{
		stat = osSemaphoreAcquire(Handler->mSem, Handler->response_timeout);//ждем пока придет нужное количество байт
		if (stat != osOK)
		 {
			RingClear(&Handler->Buffer);
			return 0;
		 }
	}
	return 1;
}
uint8_t BuildResponse(modbus *Handler,uint8_t cmd)
{
	//uint16_t crc;

	switch (cmd)
	{
	case MODBUS_HOLDING:
		Handler->ResponseBuffer[0] = Handler->slaveID;
		Handler->ResponseBuffer[1] = 0x03;
		Handler->ResponseBuffer[2] = Handler->regcount*2;
		int j=3;
		for (int i =0;i<Handler->regcount;i++)			//переворачиваем байты, чтобы на мастере было удобно
		{
			Handler->ResponseBuffer[j++] = Handler->Holding_Regs[i]>>8;
			Handler->ResponseBuffer[j++] = Handler->Holding_Regs[i];

		}
		//memcpy(Handler->ResponseBuffer+3,&Handler->Holding_Regs+Handler->offset,Handler->regcount*2);
		Handler->crc = crc16(Handler->ResponseBuffer,(Handler->regcount*2)+3);
		Handler->ResponseBuffer[(Handler->regcount*2)+3] = ((uint8_t*)&Handler->crc)[1];
		Handler->ResponseBuffer[(Handler->regcount*2)+4] = ((uint8_t*)&Handler->crc)[0];
		HAL_UART_Transmit_DMA(Handler->mbUart, Handler->ResponseBuffer, (Handler->regcount*2+5));
		break;
	case MODBUS_WRITE_HOLDING:
		Handler->ResponseBuffer[0] = Handler->slaveID;
		Handler->ResponseBuffer[1] = 0x06;
		Handler->ResponseBuffer[2] = Handler->offset>>8;
		Handler->ResponseBuffer[3] = Handler->offset;
		Handler->ResponseBuffer[4] = Handler->Holding_Regs[Handler->offset]>>8;
		Handler->ResponseBuffer[5] = Handler->Holding_Regs[Handler->offset];
		Handler->crc = crc16(Handler->ResponseBuffer,6);
		Handler->ResponseBuffer[6] = ((uint8_t*)&Handler->crc)[1];
		Handler->ResponseBuffer[7] = ((uint8_t*)&Handler->crc)[0];
		HAL_UART_Transmit_DMA(Handler->mbUart, Handler->ResponseBuffer, 8);
		break;
	case MODBUS_ERROR://illegal function
		Handler->ResponseBuffer[0] = Handler->slaveID;
		Handler->ResponseBuffer[1] = Handler->func|0x80;
		Handler->ResponseBuffer[2] = 0x01;
		Handler->crc = crc16(Handler->ResponseBuffer,3);
		Handler->ResponseBuffer[3] = ((uint8_t*)&Handler->crc)[1];
		Handler->ResponseBuffer[4] = ((uint8_t*)&Handler->crc)[0];
		HAL_UART_Transmit_DMA(Handler->mbUart, Handler->ResponseBuffer, 5);
		break;
	case MODBUS_ERROR_ADRESS:	//illegal adress
		Handler->ResponseBuffer[0] = Handler->slaveID;
		Handler->ResponseBuffer[1] = Handler->func|0x80;
		Handler->ResponseBuffer[2] = 0x02;
		Handler->crc = crc16(Handler->ResponseBuffer,3);
		Handler->ResponseBuffer[3] = ((uint8_t*)&Handler->crc)[1];
		Handler->ResponseBuffer[4] = ((uint8_t*)&Handler->crc)[0];
		HAL_UART_Transmit_DMA(Handler->mbUart, Handler->ResponseBuffer, 5);
		break;
	}

	//HAL_UART_Transmit_DMA(&huart1, Handler->ResponseBuffer, (Handler->regcount*2+5));
}
uint8_t Parse_Command (modbus *Handler,uint8_t *buf)
{
	 RingGetNBytes(&Handler->Buffer,buf+2,4);
	Handler->offset = buf[3];
	Handler->offset |= buf[2]<<8;
	Handler->regcount = buf[5];
	Handler->regcount |= buf[4];
	if ((Handler->offset+Handler->regcount)>(START_ADRESS+MAX_ADRESS))
		{
			return 0;
		}
	return 1;
}
int32_t Modbus_Recieve (modbus *Handler)
{
	Handler->length_to_read = 2;
	if (!WaitBytes(Handler,Handler->length_to_read))
	{
		return 0;
	}
	RingGetNBytes(&Handler->Buffer,Handler->recv,2);	// начинаем обработку если пришло хотя бы 2 байта
	if (Handler->recv[0] != Handler->slaveID)
		 {
			RingClear(&Handler->Buffer);	//не наш ID
			return MODBUS_ERROR;
		 }
	Handler->length_to_read += 2;					// если пришло для нас, читаем еоманду и параметры
	if (!WaitBytes(Handler,Handler->length_to_read))
		{
			return MODBUS_ERROR;
		}
	if (!Parse_Command(Handler,Handler->recv))	// проверяем, что в запросе не превышен лимит регистров
	{
		Handler->func = Handler->recv[1];
		BuildResponse(Handler,MODBUS_ERROR_ADRESS);
		return MODBUS_ERROR_ADRESS;
	}
	Handler->length_to_read = 2;
	if (!WaitBytes(Handler,Handler->length_to_read))
		{
			return 0;
		}
	RingGetNBytes(&Handler->Buffer,Handler->recv+6,Handler->length_to_read);
	Handler->crc=crc16(Handler->recv,6);
	((uint8_t*)&Handler->Recv_crc)[1] = Handler->recv[6];
	((uint8_t*)&Handler->Recv_crc)[0] = Handler->recv[7];
	if (Handler->crc != Handler->Recv_crc)
	{
		return 0;
	}

	 switch (Handler->recv[1])
	 {
		 case READ_HOLDING:
				BuildResponse(Handler,MODBUS_HOLDING);
			 break;
		 case WRITE_HOLDING:
				((uint8_t*)& Handler->Holding_Regs[Handler->offset])[1] = Handler->recv[5];
				((uint8_t*)& Handler->Holding_Regs[Handler->offset])[0] = Handler->recv[4];
				BuildResponse(Handler,MODBUS_WRITE_HOLDING);
			 break;
		 default:									// если функция не поддерживается, возвращаем ошибку
			 	 Handler->func = Handler->recv[1];
				 BuildResponse(Handler,MODBUS_ERROR);
				 Handler->error = MODBUS_ERROR;
			 break;
	 }


return 	MODBUS_OK;
}
