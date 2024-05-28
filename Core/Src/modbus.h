/*
 * modbus.h
 *
 *  Created on: May 24, 2024
 *      Author: zip_s
 */

#ifndef SRC_MODBUS_H_
#define SRC_MODBUS_H_

#include "cmsis_os.h"

#define MAX_BUFFER	256

#define READ_HOLDING 	0x03
#define WRITE_HOLDING 	0x06

#define MAX_ADRESS	32
#define START_ADRESS	0


#define MODBUS_HOLDING 			0
#define MODBUS_WRITE_HOLDING 	1
#define MODBUS_ERROR			2
#define MODBUS_ERROR_ADRESS		3
#define MODBUS_OK				4

extern UART_HandleTypeDef huart1;
extern uint8_t dataRX;

extern UART_HandleTypeDef huart2;
extern uint8_t dataRX2;

typedef struct
{
uint8_t uxBuffer[MAX_BUFFER];
uint8_t u8start;
uint8_t u8end;
uint8_t u8available;
bool    overflow;

}modbusRingBuffer_t;

struct _modbus {
    uint8_t slaveID;
    uint8_t func;
    uint8_t regcount;
    uint8_t offset;
    uint16_t Holding_Regs[120];
    uint16_t crc;
    uint32_t response_timeout;
    modbusRingBuffer_t Buffer;
    osSemaphoreId_t mSem;
    uint8_t ResponseBuffer[256];
    UART_HandleTypeDef *mbUart;
    uint32_t error;
	uint8_t recv[20],command;
	uint16_t Recv_crc;
	int length_to_read;
};


typedef struct _modbus modbus;

extern modbus *ctx;
extern modbus *ctx2;
// Кольцевой буфер
void RingClear(modbusRingBuffer_t *xRingBuffer);
uint8_t RingCountBytes(modbusRingBuffer_t *xRingBuffer);
uint8_t RingGetNBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer, uint8_t uNumber);
uint8_t RingGetAllBytes(modbusRingBuffer_t *xRingBuffer, uint8_t *buffer);
void RingAdd(modbusRingBuffer_t *xRingBuffer, uint8_t u8Val);

int32_t Modbus_Recieve (modbus *Handler);
uint8_t BuildResponse(modbus *ErrHandler,uint8_t cmd);
#endif /* SRC_MODBUS_H_ */
