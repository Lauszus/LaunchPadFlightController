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

#if !defined(__sonar_h__) && USE_SONAR
#define __sonar_h__

#ifdef __cplusplus
extern "C" {
#endif

#include "BMP180.h"

void initSonar(void);
bool triggerSonar(void);

#if USE_BARO
int16_t getSonarDistance(angle_t *angle, bmp180_t *bmp180, uint8_t maxTiltAngle);
#else
int16_t getSonarDistance(angle_t *angle, uint8_t maxTiltAngle);
#endif

#ifdef __cplusplus
}
#endif

#endif
