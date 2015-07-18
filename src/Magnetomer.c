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

#include <stdint.h>

#if USE_MAG

#include "HMC5883L.h"
#include "Magnetometer.h"

static hmc5883l_t hmc5883l;

void initMag(void) {
    initHMC5883L(&hmc5883l);
}

void getMagData(sensor_t *mag) {
    if (dataReadyHMC5883L()) // The HMC5883L update rate is very slow (15 Hz), so we have to check if data is ready
        getHMC5883LData(&hmc5883l, false); // Get magnetometer values with zero values subtracted
    *mag = hmc5883l.mag;
}

void calibrateMag(void) {
    calibrateHMC5883L(&hmc5883l);
}

#endif // USE_MAG
