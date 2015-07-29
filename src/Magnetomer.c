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
#include <stdbool.h>

#if USE_MAG

#include "AK8963.h"
#include "Buzzer.h"
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
    if ((useMagHMC5883L = initHMC5883L(&hmc5883l)) != false)
        return;
    if ((useMagAK8963 = initAK8963()) != false)
        return;

    // No magnetometer was detected
    beepLongBuzzer();
    GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED, so user knows something is up
}

bool getMagData(sensor_t *mag, bool calibrating) {
    bool newData = false;
    if (useMagHMC5883L) {
        if (dataReadyHMC5883L()) { // The HMC5883L update rate is very slow (15 Hz), so we have to check if data is ready
            newData = true;
            getHMC5883LData(&hmc5883l, calibrating);
            *mag = hmc5883l.mag;
        }
    } else if (useMagAK8963) { // The AK8963 update rate is set to 100 Hz
        if (dataReadyAK8963()) {
            newData = true;
            getAK8963Data(&ak8963, calibrating);
            *mag = ak8963.mag;
        }
    }
    return newData;
}

void calibrateMag(void) {
    if (!useMagHMC5883L && !useMagAK8963)
        return; // Return in case no magnetometer was detected

    GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED
    sensor_t mag;
    while (!getMagData(&mag, true)) { // Get magnetometer values without zero values subtracted
        delay(1); // Wait for data to get ready
    }
    sensor_t magZeroMin = mag, magZeroMax = mag; // Get initial reading

    uint32_t now = millis();
    while ((int32_t)(millis() - now) < 30000) { // Calibrate for 30s
        while (!getMagData(&mag, true)) { // Get magnetometer values without zero values subtracted
            delay(1); // Wait for data to get ready
        }
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
