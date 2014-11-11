/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

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

#ifndef __kalmany_h__
#define __kalmany_h__

#ifdef __cplusplus
extern "C" {
#endif

void KalmanYInit(void);
float getAngleY(float newAngle, float newRate, float dt);
void setAngleY(float newAngle);
float getRateY(void);
void setQangleY(float newQ_angle);
void setQbiasY(float newQ_bias);
void setRmeasureY(float newR_measure);

float getQangleY(void);
float getQbiasY(void);
float getRmeasureY(void);

#ifdef __cplusplus
}
#endif

#endif
