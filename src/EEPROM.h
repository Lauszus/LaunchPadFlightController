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

#ifndef __eeprom_h__
#define __eeprom_h__

#include "MPU6500.h"
#include "PID.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	// Can be set by the user
    pid_t pidRoll, pidPitch, pidYaw; // PID values
    float angleKp; // Self level mode Kp value
    uint8_t maxAngleInclination; // Max angle in self level mode
    float stickScalingRollPitch, stickScalingYaw; // Stick scaling values
    float Q_angle, Q_bias, R_measure; // Kalman filter coefficients

    // Will be set by the microcontroller
    bool calibrateESCs; // Flag used to tell if it should calibrate ESCs at next power cycle
    acc_t accZero; // Accelerometer calibration values
} config_t;

extern config_t cfg;

void initEEPROM(void);
void updateConfig(void);

#ifdef __cplusplus
}
#endif

#endif
