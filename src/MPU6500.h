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

#ifdef __cplusplus
extern "C" {
#endif

void initMPU6500(void);

bool dataReadyMPU6500(void);
void getMPU6500Data(int16_t *accData, int16_t *gyroData);
void getMPU6500Angles(int16_t *accData, float *gyroRate, float *roll, float *pitch, float dt);

#ifdef __cplusplus
}
#endif

#endif
