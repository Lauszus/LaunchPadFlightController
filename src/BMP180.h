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

#if !defined(__bmp180_h__) && USE_BARO
#define __bmp180_h__

#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t AC1; // The AC1 calibration from the BMP180
    int16_t AC2; // The AC2 calibration from the BMP180
    int16_t AC3; // The AC3 calibration from the BMP180
    uint16_t AC4; // The AC4 calibration from the BMP180
    uint16_t AC5; // The AC5 calibration from the BMP180
    uint16_t AC6; // The AC6 calibration from the BMP180
    int16_t B1; // The B1 calibration from the BMP180
    int16_t B2; // The B2 calibration from the BMP180
    int16_t MB; // The MB calibration from the BMP180
    int16_t MC; // The MC calibration from the BMP180
    int16_t MD; // The MD calibration from the BMP180
} bmp180_cal_t;

typedef struct {
    int32_t pressure; // Pressure in Pa
    int32_t temperature; // Temperature in 0.1 C
    float absoluteAltitude; // Absolute altitude in cm
    float groundAltitude; // Ground altitude in cm
    uint8_t mode; // The oversampling mode to be used by the BMP180
    bmp180_cal_t cal; // Calibration data
} bmp180_t;

void intBMP180(bmp180_t *bmp180);
void getBMP180Data(bmp180_t *bmp180);

#ifdef __cplusplus
}
#endif

#endif
