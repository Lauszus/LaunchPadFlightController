/* Copyright (C) 2015 Kristian Lauszus, TKJ Electronics. All rights reserved.

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

#include <stdbool.h>

#include "Types.h"

// Scale factor for +-2000deg/s and +-8g - see datasheet: http://www.invensense.com/mems/gyro/documents/PS-MPU-6500A-01.pdf at page 9-10
#define MPU6500_GYRO_SCALE_FACTOR   16.4f
#define MPU6500_ACC_SCALE_FACTOR    4096.0f

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    sensorRaw_t acc; // Raw accelerometer readings
    sensorRaw_t gyro; // Raw gyroscope readings
    angle_t gyroRate; // Gyroscope readings in deg/s
} mpu6500_t;

void initMPU6500(void);
bool dataReadyMPU6500(void);
void getMPU6500Data(mpu6500_t *mpu6500);
bool calibrateAcc(void);

#ifdef __cplusplus
}
#endif

#endif
