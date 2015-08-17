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

// Function pointer used to call the low-level magnetometer drivers
static bool (*dataReady)(void);
static void (*getData)(sensor_t *mag, bool calibrating);

void initMag(void) {
    if (initHMC5883L()) { // The HMC5883L update rate is very slow (15 Hz)
        dataReady = &dataReadyHMC5883L;
        getData = &getHMC5883LData;
        return;
    } else if (initAK8963()) { // The AK8963 update rate is set to 100 Hz
        dataReady = &dataReadyAK8963;
        getData = &getAK8963Data;
        return;
    }

    // No magnetometer was detected
    beepLongBuzzer();
    GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED, so user knows something is up
}

bool getMagData(sensor_t *mag, bool calibrating) {
    if (dataReady) { // Make sure pointer is not NULL
        if (dataReady()) {
            getData(mag, calibrating);
            return true;
        }
    }
    return false;
}

void calibrateMag(void) {
    if (!dataReady)
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
