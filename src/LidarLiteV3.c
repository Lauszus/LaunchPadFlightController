/* Copyright (C) 2016 Kristian Sloth Lauszus. All rights reserved.

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

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#if USE_LIDAR_LITE

#include "EEPROM.h"
#include "I2C.h"
#include "IMU.h"
#include "LidarLiteV3.h"
#include "Time.h"

#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

#define LIDAR_LITE_ADDRESS              0x62

#define LIDAR_LITE_ACQ_COMMAND          0x00
#define LIDAR_LITE_STATUS               0x01
#define LIDAR_LITE_SIG_COUNT_VAL        0x02
#define LIDAR_LITE_ACQ_CONFIG_REG       0x04
#define LIDAR_LITE_THRESHOLD_BYPASS     0x1C

#define LIDAR_LITE_DATA                 0x8F

// Implemented based on: http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf

void initLidarLite(void) {
    uint8_t status = i2cRead(LIDAR_LITE_ADDRESS, LIDAR_LITE_STATUS);
    if (!(status & (1 << 6)) && (status & (1 << 5))) { // Read process error and health flag
#if UART_DEBUG
        UARTprintf("LIDAR-Lite v3 found\n");
#endif
    } else {
#if UART_DEBUG
        UARTprintf("Could not find LIDAR-Lite v3: %02X\n", status);
#endif
        while (1);
    }

    i2cWrite(LIDAR_LITE_ADDRESS, LIDAR_LITE_ACQ_COMMAND, 0x00); //  Reset FPGA, all registers return to default values
    delay(22); // Wait 22 ms after reset according to datasheet

    // Default configuration
    i2cWrite(LIDAR_LITE_ADDRESS, LIDAR_LITE_SIG_COUNT_VAL, 0x80); // Maximum acquisition count
    i2cWrite(LIDAR_LITE_ADDRESS, LIDAR_LITE_ACQ_CONFIG_REG, 0x08); // Disable measurement quick termination
    i2cWrite(LIDAR_LITE_ADDRESS, LIDAR_LITE_THRESHOLD_BYPASS, 0x00); // Use default valid measurement detection algorithm
}

bool triggerLidarLite(void) {
    static bool busyFlag = false;

    if (!busyFlag) {
        static uint8_t biasCounter = 0;
        if (biasCounter == 0)
            i2cWrite(LIDAR_LITE_ADDRESS, LIDAR_LITE_ACQ_COMMAND, 0x04); // Take acquisition & correlation processing with receiver bias correction
        else
            i2cWrite(LIDAR_LITE_ADDRESS, LIDAR_LITE_ACQ_COMMAND, 0x03); // Take acquisition & correlation processing without receiver bias correction
        if (++biasCounter >= 100) // Receive bias correction every 100th measurement according to the datasheet
            biasCounter = 0;
    }

    busyFlag = i2cRead(LIDAR_LITE_ADDRESS, LIDAR_LITE_STATUS) & 0x01; // Read status register to check busy flag
    return !busyFlag; // Return true if new measurement is ready
}

// Returns the distance in mm. Range is 0-40000 mm or -1 if the value is invalid.
int32_t getLidarLiteDistance(angle_t *angle) {
    if (fmaxf(fabsf(angle->axis.roll), fabsf(angle->axis.pitch)) > cfg.maxAngleInclinationDistSensor) // Return -1 if it is tilted more than the maximum tilt angle
        return -1;

    uint8_t i2cBuffer[2]; // Buffer for I2C data
    i2cReadData(LIDAR_LITE_ADDRESS, LIDAR_LITE_DATA, i2cBuffer, 2); // Read distance measurement

    int32_t distance = (int16_t)((i2cBuffer[0] << 8) | i2cBuffer[1]); // Return the distance in cm
    distance *= 10; // Convert to mm
    distance *= cosf(angle->axis.roll * DEG_TO_RAD) * cosf(angle->axis.pitch * DEG_TO_RAD); // Calculate adjacent side

    return distance;
}

#endif // USE_LIDAR_LITE
