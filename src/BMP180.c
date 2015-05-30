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
#include <math.h>

#if USE_BARO

#include "BMP180.h"
#include "I2C.h"
#include "Time.h"

#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

#define BMP185_DEBUG 0 // Set this to 1 to check temperature and pressure calculation

// Inspired by: https://code.google.com/p/open-headtracker, https://github.com/cleanflight/cleanflight/blob/master/src/main/drivers/compass_hmc5883l.c and https://github.com/adafruit/Adafruit-BMP085-Library/blob/master/Adafruit_BMP085.cpp
#define BMP180_ADDRESS                  0x77 // Address of barometer

#define BMP185_CAL_AC1                  0xAA // Calibration data (16 bits)
#define BMP185_CAL_LENGTH               22

#define BMP185_CHIP_ID                  0xD0
#define BMP185_CHIP_ID_VALUE            0x55

#define BMP185_SOFT_RESET               0xE0
#define BMP185_SOFT_RESET_CMD           0xB6

// Control register
#define BMP185_CONTROL                  0xF4

// Control register - pressure accuracy
#define BMP185_CONTROL_LOW_POWER        0 // Ultra low power mode
#define BMP185_CONTROL_STANDARD_MODE    1 // Standard mode
#define BMP185_CONTROL_HIGH_RES         2 // High resolution mode
#define BMP185_CONTROL_ULTRA_HIGH_RES   3 // Ultra high resolution mode

#define BMP185_MEASUREMENT              0xF6

// Commands used to start the measurements
#define BMP185_READ_TEMP_CMD            0x2E
#define BMP185_READ_PRESSURE_CMD        0x34

enum {
    START_TEMP = 0,
    READ_TEMP,
    READ_PRESSURE,
} bmp180State;

// See the datasheet: http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
bool getBMP180Data(bmp180_t *bmp180) {
    static uint32_t stateTimer; // Used for time keeping
    static uint16_t stateDelay; // Used for delay between measurements
    static int32_t UT; // Temperature measurement
    static int32_t UP; // Pressure measurement

    switch(bmp180State) {
        case START_TEMP:
            i2cWrite(BMP180_ADDRESS, BMP185_CONTROL, BMP185_READ_TEMP_CMD); // Start temperature measurements
            stateTimer = micros();
            bmp180State = READ_TEMP;
            break;

        case READ_TEMP:
            if ((int32_t)(micros() - stateTimer) > 4500) { // Wait 4.5 ms before reading data
                uint8_t buf[2];
                i2cReadData(BMP180_ADDRESS, BMP185_MEASUREMENT, buf, 2); // Get uncompensated temperature value
                UT = (buf[0] << 8) | buf[1];

                i2cWrite(BMP180_ADDRESS, BMP185_CONTROL, BMP185_READ_PRESSURE_CMD + (bmp180->mode << 6)); // Start pressure measurements
                if (bmp180->mode == BMP185_CONTROL_LOW_POWER)
                    stateDelay = 4500; // Wait 4.5 ms between readings when in ultra low power mode
                else if (bmp180->mode == BMP185_CONTROL_STANDARD_MODE)
                    stateDelay = 7500; // Wait 7.5 ms between readings when in standard mode
                else if (bmp180->mode == BMP185_CONTROL_HIGH_RES)
                    stateDelay = 13500; // Wait 13.5 ms between readings when in high resolution mode
                else
                    stateDelay = 25500; // Wait 25.5 ms between readings when in ultra high resolution mode
                stateTimer = micros();
                bmp180State = READ_PRESSURE;
            }
            break;

        case READ_PRESSURE:
            if ((int32_t)(micros() - stateTimer) > stateDelay) {
                uint8_t buf[3];
                i2cReadData(BMP180_ADDRESS, BMP185_MEASUREMENT, buf, 3); // Get uncompensated pressure value
                UP = (((uint32_t)buf[0] << 16) | (buf[1] << 8) | buf[2]) >> (8 - bmp180->mode);

#if BMP185_DEBUG
                // Use datasheet numbers!
                bmp180->cal.AC1 = 408;
                bmp180->cal.AC2 = -72;
                bmp180->cal.AC3 = -14383;
                bmp180->cal.AC4 = 32741;
                bmp180->cal.AC5 = 32757;
                bmp180->cal.AC6 = 23153;

                bmp180->cal.B1 = 6190;
                bmp180->cal.B2 = 4;

                bmp180->cal.MC = -8711;
                bmp180->cal.MD = 2868;

                UT = 27898;
                bmp180->mode = 0;
                UP = 23843;
#endif

                // Calculate true temperature
                int32_t X1 = (UT - (int32_t)bmp180->cal.AC6) * ((int32_t)bmp180->cal.AC5) >> 15;
                int32_t X2 = ((int32_t)bmp180->cal.MC << 11) / (X1 + (int32_t)bmp180->cal.MD);
                int32_t B5 = X1 + X2;
                bmp180->temperature = (B5 + 8) >> 4;

                // Calculate true pressure
                int32_t B6 = B5 - 4000;

                X1 = ((int32_t)bmp180->cal.B2 * (B6 * B6 >> 12)) >> 11;
                X2 = ((int32_t)bmp180->cal.AC2 * B6) >> 11;
                int32_t X3 = X1 + X2;
                int32_t B3 = ((((int32_t)bmp180->cal.AC1 * 4 + X3) << bmp180->mode) + 2) >> 2;

                X1 = ((int32_t)bmp180->cal.AC3 * B6) >> 13;
                X2 = ((int32_t)bmp180->cal.B1 * ((B6 * B6) >> 12)) >> 16;
                X3 = ((X1 + X2) + 2) >> 2;
                uint32_t B4 = ((int32_t)bmp180->cal.AC4 * (uint32_t)(X3 + 0x8000UL)) >> 15;

                uint32_t B7 = ((uint32_t)UP - B3) * (50000UL >> bmp180->mode);
                if (B7 < (1UL << 31))
                    bmp180->pressure = (B7 << 1) / B4;
                else
                    bmp180->pressure = (B7 / B4) << 1;

                X1 = (bmp180->pressure >> 8) * (bmp180->pressure >> 8);
                X1 = (X1 * 3038L) >> 16;
                X2 = (-7357L * bmp180->pressure) >> 16;
                bmp180->pressure += (X1 + X2 + 3791L) >> 4;

                static const int32_t p0 = 101325; // Pressure at sea level
                bmp180->absoluteAltitude = 44330.0f * (1.0f - powf((float)bmp180->pressure / (float)p0, 1.0f / 5.255f)) * 100.0f; // Get altitude in cm

#if BMP185_DEBUG && UART_DEBUG
                UARTprintf("%d == 150\t%d == 69964\t%d == 301666\n", bmp180->temperature, bmp180->pressure, (int32_t)bmp180->absoluteAltitude);
                while (1);
#endif

                bmp180State = START_TEMP;
                return true; // Indicate that new values has been calculated
            }
            break;

        default:
            bmp180State = START_TEMP;
            break;
    }

    return false;
}

void intBMP180(bmp180_t *bmp180) {
    uint8_t buf[BMP185_CAL_LENGTH]; // Buffer for I2C data

    buf[0] = i2cRead(BMP180_ADDRESS, BMP185_CHIP_ID);
    if (buf[0] == BMP185_CHIP_ID_VALUE) { // Read Chip ID
#if UART_DEBUG
        UARTprintf("BMP180 found\n");
#endif
    } else {
#if UART_DEBUG
        UARTprintf("Could not find BMP180: %2X\n", buf[0]);
#endif
        while (1);
    }

    i2cWrite(BMP180_ADDRESS, BMP185_SOFT_RESET, BMP185_SOFT_RESET_CMD); // Reset device, this will perform the same sequence as power on reset.
    delay(100);

    bmp180->mode = BMP185_CONTROL_ULTRA_HIGH_RES; // Set to ultra high resolution mode

    // Read calibration values
    i2cReadData(BMP180_ADDRESS, BMP185_CAL_AC1, buf, BMP185_CAL_LENGTH);
    bmp180->cal.AC1 = (buf[0] << 8) | buf[1];
    bmp180->cal.AC2 = (buf[2] << 8) | buf[3];
    bmp180->cal.AC3 = (buf[4] << 8) | buf[5];
    bmp180->cal.AC4 = (buf[6] << 8) | buf[7];
    bmp180->cal.AC5 = (buf[8] << 8) | buf[9];
    bmp180->cal.AC6 = (buf[10] << 8) | buf[11];

    bmp180->cal.B1  = (buf[12] << 8) | buf[13];
    bmp180->cal.B2  = (buf[14] << 8) | buf[15];

    // The MB calibration value is newer used
    bmp180->cal.MC  = (buf[18] << 8) | buf[19];
    bmp180->cal.MD  = (buf[20] << 8) | buf[21];

    bmp180State = START_TEMP; // Reset state machine

    while (!getBMP180Data(bmp180)) {
        // Wait until measurement is complete
    }
    bmp180->groundAltitude = bmp180->absoluteAltitude; // Get initial altitude
}

#endif // USE_BARO
