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

#ifndef __ppm_h__
#define __ppm_h__

#ifdef __cplusplus
extern "C" {
#endif

#define PPM_MIN 1064 // From SimonK firmware
#define PPM_MAX 1864 // From SimonK firmware

void initPPM(void);
void writePPMAllOff(void);
void updateMotor(uint8_t motor, float value);
void updateMotorsAll(float *values);
void writePPMUs(uint8_t motor, uint16_t us);
void writePPMWidth(uint8_t motor, uint16_t width);
uint16_t getPeriod(void);
void calibrateESCs(bool flag);

float mapf(float x, float in_min, float in_max, float out_min, float out_max);

#ifdef __cplusplus
}
#endif

#endif
