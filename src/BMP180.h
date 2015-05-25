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
    float presure;
    uint8_t ui8Mode; // The sampling mode to be used by the BMP180
    uint8_t ui8NewMode; // The new sampling mode, which is used when a register write succeeds
    int16_t i16AC1; // The AC1 calibration from the BMP180
    int16_t i16AC2; // The AC2 calibration from the BMP180
    int16_t i16AC3; // The AC3 calibration from the BMP180
    uint16_t ui16AC4; // The AC4 calibration from the BMP180
    uint16_t ui16AC5; // The AC5 calibration from the BMP180
    uint16_t ui16AC6; // The AC6 calibration from the BMP180
    int16_t i16B1; // The B1 calibration from the BMP180
    int16_t i16B2; // The B2 calibration from the BMP180
    int16_t i16MC; // The MC calibration from the BMP180
    int16_t i16MD; // The MD calibration from the BMP180
} bmp180_t;

void intBMP180(bmp180_t *bmp180);
bool dataReadyBMP180(void);
void getBMP180Data(bmp180_t *bmp180, bool calibrating);
void calibrateBaro(bmp180_t *bmp180);

#ifdef __cplusplus
}
#endif

#endif
