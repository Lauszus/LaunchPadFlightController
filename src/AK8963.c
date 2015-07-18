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
#include "EEPROM.h"
#include "I2C.h"
#include "Pins.h"
#include "Time.h"

#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

#define AK8963_ADDRESS                  0x0C

#define AK8963_WIA                      0x00
#define AK8963_WIA_ID                   0x48

#define AK8963_STATUS_1                 0x02
#define AK8963_STATUS_1_DRDY_BIT        (1 << 0)

#define AK8963_HXL                      0x03
#define AK8963_DATA_LENGTH              7 // Length of magnetometer data + ST2 (Status 2) register
#define AK8963_STATUS_2_HOFL_BIT        (1 << 3)

#define AK8963_CONTROL_1                0x0A
#define AK8963_CONTROL_1_CONT_2_BIT     (6 << 0) // Continuous measurement mode 2 (100 Hz)
#define AK8963_CONTROL_1_16_OUTPUT_BIT  (1 << 4) // 16-bit output

#define AK8963_CONTROL_2                0x0B
#define AK8963_CONTROL_2_SRST_BIT       (1 << 0) // Soft reset

bool dataReadyAK8963(void) {
    return i2cRead(AK8963_ADDRESS, AK8963_STATUS_1) & AK8963_STATUS_1_DRDY_BIT; // Read DRDY bit
}

bool initAK8963(void) {
    uint8_t id = i2cRead(AK8963_ADDRESS, AK8963_WIA); // Read "WIA" register
    if (id == AK8963_WIA_ID) {
#if UART_DEBUG
        UARTprintf("AK8963 found\n");
#endif
    } else {
#if UART_DEBUG
        UARTprintf("Could not find AK8963: %2X\n", id);
#endif
        return false;
    }

    i2cWrite(AK8963_ADDRESS, AK8963_CONTROL_2, AK8963_CONTROL_2_SRST_BIT); // Reset device, this resets all internal registers to their default values
    delay(100);
    while (i2cRead(AK8963_ADDRESS, AK8963_CONTROL_2) & AK8963_CONTROL_2_SRST_BIT) {
        // Wait for the bit to clear
    }
    delay(100);

    i2cWrite(AK8963_ADDRESS, AK8963_CONTROL_1, AK8963_CONTROL_1_CONT_2_BIT | AK8963_CONTROL_1_16_OUTPUT_BIT); // Set to continuous measurement mode 2 (100 Hz) and 16-bit output

    return true;
}

// X-axis should be facing forward
// Y-axis should be facing to the left
// Z-axis should be facing upward
static void ak8963BoardOrientation(sensorRaw_t *sensorRaw) {
    // Note that the AK8963 is aligned differently compared to the MPU-9250 - see: http://store.invensense.com/datasheets/invensense/MPU9250REV1.0.pdf page 38
    // x -> y
    // y -> x
    // z-> -z
    sensorRaw_t sensorRawTemp = *sensorRaw;
    sensorRaw->axis.X = sensorRawTemp.axis.X;
    sensorRaw->axis.Y = -sensorRawTemp.axis.Y;
    sensorRaw->axis.Z = -sensorRawTemp.axis.Z;
}

void getAK8963Data(ak8963_t *ak8963, bool calibrating) {
    uint8_t buf[AK8963_DATA_LENGTH];
    i2cReadData(AK8963_ADDRESS, AK8963_HXL, buf, AK8963_DATA_LENGTH); // Get magnetometer values

    ak8963->mag.axis.X = (buf[1] << 8) | buf[0];
    ak8963->mag.axis.Y = (buf[3] << 8) | buf[2];
    ak8963->mag.axis.Z = (buf[5] << 8) | buf[4];

    if (buf[6] & AK8963_STATUS_2_HOFL_BIT) { // ST2 register must be read!
#if UART_DEBUG
        UARTprintf("Magnetic sensor overflow occurred\n");
#endif
    }

    ak8963BoardOrientation(&ak8963->mag); // Apply board orientation

    if (!calibrating) { // If we are not calibrating, then subtract zero values
        for (uint8_t axis = 0; axis < 3; axis++)
            ak8963->mag.data[axis] -= cfg.magZero.data[axis]; // Subtract zero value stored in EEPROM
    }
}

void calibrateAK8963(ak8963_t *ak8963) {
    GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED
    while (!dataReadyAK8963()); // Wait for data to get ready
    getAK8963Data(ak8963, true); // Get magnetometer values without zero values subtracted
    sensorRaw_t magZeroMin = ak8963->mag, magZeroMax = ak8963->mag; // Get initial reading

    uint32_t now = millis();
    while ((int32_t)(millis() - now) < 30000) { // Calibrate for 30s
        while (!dataReadyAK8963()); // Wait for data to get ready
        getAK8963Data(ak8963, true); // Get magnetometer values without zero values subtracted
        for (uint8_t axis = 0; axis < 3; axis++) {
            if (ak8963->mag.data[axis] < magZeroMin.data[axis])
                magZeroMin.data[axis] = ak8963->mag.data[axis];
            if (ak8963->mag.data[axis] > magZeroMax.data[axis])
                magZeroMax.data[axis] = ak8963->mag.data[axis];
        }
    }
    for (uint8_t axis = 0; axis < 3; axis++)
        cfg.magZero.data[axis] = (magZeroMax.data[axis] + magZeroMin.data[axis]) / 2.0f;
    updateConfig(); // Save new values in EEPROM
    GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED
}

#endif // USE_MAG

