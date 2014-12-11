/* Copyright (C) 2014 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
*/

#ifndef __mpu6500_h__
#define __mpu6500_h__

#ifdef __cplusplus
extern "C" {
#endif

bool dateReadyMPU6500(void);
void getMPU6500Angles(float *roll, float *pitch, float dt);
void getMPU6050Gyro(int16_t *gyroData);

void initMPU6500_i2c(void);
void updateMPU6500(int16_t *accData, int16_t *gyroData);

void i2cWrite(uint8_t addr, uint8_t data);
void i2cWriteData(uint8_t addr, uint8_t *date, uint8_t length);
uint8_t i2cRead(uint8_t addr);
void i2cReadData(uint8_t addr, uint8_t *data, uint8_t length);

void initMPU6500(void);
void spiReadData(uint32_t addr, uint32_t *buffer);
void spiWriteData(uint32_t addr, uint32_t buffer);

#ifdef __cplusplus
}
#endif

#endif
