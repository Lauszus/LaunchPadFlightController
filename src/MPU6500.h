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

#ifndef __mpu6500_h__
#define __mpu6500_h__

#include <stdbool.h>

#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    sensorRaw_t acc, gyro; // Raw accelerometer and gyroscope readings
    sensor_t accBodyFrame; // Magnitude of the acceleration in the body frame
    float accScaleFactor, gyroScaleFactor; // Gyroscope and accelerometer scale factor
    angle_t gyroRate; // Gyroscope readings in deg/s
} mpu6500_t;

void initMPU6500(mpu6500_t *mpu6500);
void mpu6500BoardOrientation(sensorRaw_t *sensorRaw);
bool dataReadyMPU6500(void);
void getMPU6500Data(mpu6500_t *mpu6500);
bool calibrateMPU6500Acc(mpu6500_t *mpu6500);

#ifdef __cplusplus
}
#endif

#endif
