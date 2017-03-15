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

#ifndef __imu_h__
#define __imu_h__

#include "MPU6500.h"

#define DEG_TO_RAD  0.017453292519943295769236907684886f
#define RAD_TO_DEG  57.295779513082320876798154814105f

#ifdef __cplusplus
extern "C" {
#endif

void getAngles(mpu6500_t *mpu6500, sensor_t *mag, angle_t *angle, float dt);
void rotateV(sensor_t *v, const angle_t *angle);

#ifdef __cplusplus
}
#endif

#endif
