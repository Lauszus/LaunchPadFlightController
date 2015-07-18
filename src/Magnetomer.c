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

#include "AK8963.h"
#include "HMC5883L.h"
#include "Magnetometer.h"

static hmc5883l_t hmc5883l;
static ak8963_t ak8963;

static bool useMagHMC5883L, useMagAK8963;

void initMag(void) {
    useMagHMC5883L = initHMC5883L(&hmc5883l);
    if (useMagHMC5883L)
        return;
    useMagAK8963 = initAK8963();
}

void getMagData(sensor_t *mag) {
    if (useMagHMC5883L) {
        if (dataReadyHMC5883L()) // The HMC5883L update rate is very slow (15 Hz), so we have to check if data is ready
            getHMC5883LData(&hmc5883l, false); // Get magnetometer values with zero values subtracted
        *mag = hmc5883l.mag;
    } else if (useMagAK8963) {
        if (dataReadyAK8963())
            getAK8963Data(&ak8963, false);
        for (uint8_t axis = 0; axis < 3; axis++)
            mag->data[axis] = ak8963.mag.data[axis];
    }
}

void calibrateMag(void) {
    if (useMagHMC5883L)
        calibrateHMC5883L(&hmc5883l);
    else if (useMagAK8963)
        calibrateAK8963(&ak8963);
}

#endif // USE_MAG
