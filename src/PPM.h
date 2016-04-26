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

#ifndef __ppm_h__
#define __ppm_h__

#ifdef __cplusplus
extern "C" {
#endif

// Internal motor values are in the range [-100:100]
#define MIN_MOTOR_OUT (-100.0f)
#define MAX_MOTOR_OUT (100.0f)

void initPPM(void);
void writePPMAllOff(void);
void updateMotorsAll(float *values);
void calibrateESCs(bool flag);

float mapf(float x, float in_min, float in_max, float out_min, float out_max);

#ifdef __cplusplus
}
#endif

#endif
