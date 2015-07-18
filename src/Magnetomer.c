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
#include "EEPROM.h"
#include "HMC5883L.h"
#include "Magnetometer.h"
#include "Pins.h"
#include "Time.h"

#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"

static hmc5883l_t hmc5883l;
static ak8963_t ak8963;

static bool useMagHMC5883L, useMagAK8963;

void initMag(void) {
    useMagHMC5883L = initHMC5883L(&hmc5883l);
    if (useMagHMC5883L)
        return;
    useMagAK8963 = initAK8963();
}

void getMagData(sensor_t *mag, bool calibrating) {
    if (useMagHMC5883L) {
        if (dataReadyHMC5883L()) // The HMC5883L update rate is very slow (15 Hz), so we have to check if data is ready
            getHMC5883LData(&hmc5883l, calibrating);
        *mag = hmc5883l.mag;
    } else if (useMagAK8963) { // The AK8963 update rate is set to 100 Hz
        if (dataReadyAK8963())
            getAK8963Data(&ak8963, calibrating);
        for (uint8_t axis = 0; axis < 3; axis++)
            mag->data[axis] = ak8963.mag.data[axis];
    }
}

static bool dataReadyMag(void) {
    if (useMagHMC5883L)
        return dataReadyHMC5883L();
    return dataReadyAK8963();
}

void calibrateMag(void) {
    if (!useMagHMC5883L && !useMagAK8963)
        return; // Return in case no magnetometer was detected

    GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED
    while (!dataReadyMag()) {
        delay(1); // Wait for data to get ready
    }
    sensor_t mag;
    getMagData(&mag, true); // Get magnetometer values without zero values subtracted
    sensor_t magZeroMin = mag, magZeroMax = mag; // Get initial reading

    uint32_t now = millis();
    while ((int32_t)(millis() - now) < 30000) { // Calibrate for 30s
        while (!dataReadyMag()) {
            delay(1); // Wait for data to get ready
        }
        getMagData(&mag, true); // Get magnetometer values without zero values subtracted
        for (uint8_t axis = 0; axis < 3; axis++) {
            if (mag.data[axis] < magZeroMin.data[axis])
                magZeroMin.data[axis] = mag.data[axis];
            if (mag.data[axis] > magZeroMax.data[axis])
                magZeroMax.data[axis] = mag.data[axis];
        }
    }

    for (uint8_t axis = 0; axis < 3; axis++)
        cfg.magZero.data[axis] = (magZeroMax.data[axis] + magZeroMin.data[axis]) / 2.0f;
    updateConfig(); // Save new values in EEPROM
    GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED
}

#endif // USE_MAG
