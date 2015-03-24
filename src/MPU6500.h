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

#pragma anon_unions

#ifdef __cplusplus
extern "C" {
#endif

typedef union {
	struct {
		int16_t X, Y, Z;
	} __attribute__((packed));
	int16_t data[3];
} acc_t;

typedef union {
	struct {
		int16_t X, Y, Z;
	} __attribute__((packed));
	int16_t data[3];
} gyro_t;

typedef union {
    struct {
        float X, Y, Z;
    } __attribute__((packed));
    float data[3];
} gyroRate_t;

typedef struct {
    acc_t acc; // Raw accelerometer readings
    gyro_t gyro; // Raw gyroscope readings
    gyroRate_t gyroRate; // Gyroscope readings in deg/s
} mpu6500_t;

void initMPU6500(void);
bool dataReadyMPU6500(void);
void getMPU6500Data(mpu6500_t *mpu6500);
void getMPU6500Angles(mpu6500_t *mpu6500, float *roll, float *pitch, float dt);
bool calibrateAcc(void);

#ifdef __cplusplus
}
#endif

#endif
