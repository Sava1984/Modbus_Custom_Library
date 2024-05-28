/*
 * modbus_crc.h
 *
 *  Created on: May 28, 2024
 *      Author: zip_s
 */

#ifndef SRC_MODBUS_CRC_H_
#define SRC_MODBUS_CRC_H_

#include "main.h"

uint16_t crc16(uint8_t *buffer, uint16_t buffer_length);

#endif /* SRC_MODBUS_CRC_H_ */
