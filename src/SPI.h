/* Copyright (C) 2015 Kristian Sloth Lauszus. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Sloth Lauszus
 Web      :  http://www.lauszus.com
 e-mail   :  lauszus@gmail.com
*/

#if !defined(__spi_h__) && USE_FLOW_SENSOR
#define __spi_h__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void initSPI(void);

void spiWrite(uint8_t regAddr, uint8_t data);
void spiWriteData(uint8_t regAddr, const uint8_t *data, uint32_t length);

uint8_t spiRead(uint8_t regAddr);
void spiReadData(uint8_t regAddr, uint8_t *data, uint32_t length);

void spiTransfer(const uint8_t *header, uint16_t headerLength, const uint8_t *sendData, uint8_t *recvData, uint32_t dataLength);

#ifdef __cplusplus
}
#endif

#endif
