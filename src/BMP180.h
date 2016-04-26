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

#if !defined(__bmp180_h__) && USE_BARO
#define __bmp180_h__

#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t AC1;
    int16_t AC2;
    int16_t AC3;
    uint16_t AC4;
    uint16_t AC5;
    uint16_t AC6;
    int16_t B1;
    int16_t B2;
    // The MB calibration value is never used
    int16_t MC;
    int16_t MD;
} bmp180_cal_t;

typedef struct {
    int32_t pressure; // Pressure in Pa
    int32_t temperature; // Temperature in 0.1 C
    float absoluteAltitude; // Absolute altitude in cm
    float groundAltitude; // Ground altitude in cm
    uint8_t mode; // The oversampling mode to be used by the BMP180
    bmp180_cal_t cal; // Calibration data
} bmp180_t;

void initBMP180(bmp180_t *bmp180);
bool getBMP180Data(bmp180_t *bmp180);

#ifdef __cplusplus
}
#endif

#endif
