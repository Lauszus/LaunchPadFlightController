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

#if !defined(__hmc5883l_h__) && USE_MAG
#define __hmc5883l_h__

#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    sensorRaw_t magRaw; // Raw magnetometer readings
    sensor_t mag; // Magnetometer readings with gain and offset applied
    sensor_t magGain; // Magnetometer gain
} hmc5883l_t;

void intHMC5883L(hmc5883l_t *hmc5883l);
bool dataReadyHMC5883L(void);
void getHMC5883LData(hmc5883l_t *hmc5883l, bool calibrating);
void calibrateMag(hmc5883l_t *hmc5883l);

#ifdef __cplusplus
}
#endif

#endif
