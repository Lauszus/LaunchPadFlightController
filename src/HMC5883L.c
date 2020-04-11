/* Copyright (C) 2015 Kristian Sloth Lauszus. All rights reserved.

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
#include <stdlib.h>
#include <math.h>

#if USE_MAG

#include "Config.h"
#include "EEPROM.h"
#include "HMC5883L.h"
#include "I2C.h"
#include "Time.h"
#include "Types.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

#define HMC5883L_ADDRESS 0x1E // Address of magnetometer

#define HMC5883L_CONF_REG_A             0x00
#define HMC5883L_CONF_REG_B             0x01
#define HMC5883L_MODE_REG               0x02
#define HMC5883L_OUTPUT_REG_X_MSB       0x03
#define HMC5883L_OUTPUT_REG_X_LSB       0x04
#define HMC5883L_OUTPUT_REG_Z_MSB       0x05
#define HMC5883L_OUTPUT_REG_Z_LSB       0x06
#define HMC5883L_OUTPUT_REG_Y_MSB       0x07
#define HMC5883L_OUTPUT_REG_Y_LSB       0x08
#define HMC5883L_STATUS_REG_            0x09
#define HMC5883L_ID_REG_A               0x0A
#define HMC5883L_ID_REG_B               0x0B
#define HMC5883L_ID_REG_C               0x0C

// Configuration Register A
#define HMC5883L_CONF_REG_A_MA_8        (3 << 5) // 8 sample averages
#define HMC5883L_CONF_REG_A_D0_15       (4 << 2) // Data output rate to 15 Hz (default)
#define HMC5883L_CONF_REG_A_MS_NORM     (0 << 0) // Normal measurement mode (default)
#define HMC5883L_CONF_REG_A_MS_POS      (1 << 0) // Positive bias mode
#define HMC5883L_CONF_REG_A_MS_NEG      (2 << 0) // Negative bias mode

// Configuration Register B
#define HMC5883L_CONF_REG_B_GN_13       (1 << 5) // 1.3 Ga gain (default)
#define HMC5883L_CONF_REG_B_GN_25       (3 << 5) // 2.5 Ga gain

// Mode Register
#define HMC5883L_MODE_REG_HS            (1 << 7) // I2C high speed mode (400 kHz)
#define HMC5883L_MODE_REG_MD_CONT       (0 << 0) // Continuous measurement mode
#define HMC5883L_MODE_REG_MD_SINGLE     (1 << 0) // Single measurement mode

#define HMC5883L_X_SELF_TEST_GAUSS      1.16f // X axis level when bias current is applied
#define HMC5883L_Y_SELF_TEST_GAUSS      1.16f // Y axis level when bias current is applied
#define HMC5883L_Z_SELF_TEST_GAUSS      1.08f // Z axis level when bias current is applied
#define SELF_TEST_LOW_LIMIT             (243.0f / 390.0f) // Low limit when Gain = 5 (4.7 Ga)
#define SELF_TEST_HIGH_LIMIT            (575.0f / 390.0f) // High limit when Gain = 5 (4.7 Ga)
#define GAIN_13_GA_LSB_GAIN             1090.0f // Gain (LSB/Gauss) for 1.3 Ga gain (default)
#define GAIN_25_GA_LSB_GAIN             660.0f  // Gain (LSB/Gauss) for 2.5 Ga gain

static struct hmc5883l_t {
    sensorRaw_t magRaw; // Raw magnetometer readings
    sensor_t magGain; // Magnetometer gain
} hmc5883l;

static volatile bool dataReady;

// Returns true when data is ready to be read
bool dataReadyHMC5883L(void) {
    bool dataReadyTemp = dataReady;
    dataReady = false; // Clear flag
    return dataReadyTemp;
}

static void drdyHandler(void) {
    GPIOIntClear(GPIO_HMC5883L_DRDY_BASE, GPIO_HMC5883L_DRDY_PIN); // Clear interrupt source
    dataReady = true;
}

// X-axis should be facing forward
// Y-axis should be facing to the right
// Z-axis should be facing downward
static void hmc5883lBoardOrientation(sensor_t *sensor) {
    sensor_t sensorTemp = *sensor;
    sensor->axis.X = sensorTemp.axis.Y;
    sensor->axis.Y = -sensorTemp.axis.X;
    sensor->axis.Z = sensorTemp.axis.Z;
}

static void getHMC5883LDataRaw(sensorRaw_t *magRaw) {
    uint8_t buf[6];
    i2cReadData(HMC5883L_ADDRESS, HMC5883L_OUTPUT_REG_X_MSB, buf, 6); // Get magnetometer values

    magRaw->axis.X = (int16_t)((buf[0] << 8) | buf[1]);
    magRaw->axis.Z = (int16_t)((buf[2] << 8) | buf[3]);
    magRaw->axis.Y = (int16_t)((buf[4] << 8) | buf[5]);
}

void getHMC5883LData(sensor_t *mag, bool calibrating) {
    getHMC5883LDataRaw(&hmc5883l.magRaw); // Get raw reading
    for (uint8_t axis = 0; axis < 3; axis++)
        mag->data[axis] = (float)hmc5883l.magRaw.data[axis] * hmc5883l.magGain.data[axis]; // Apply gain

    hmc5883lBoardOrientation(mag); // Apply board orientation
/*
    // The value should be positive when pointing at north
    UARTprintf("%d\t%d\t%d\n", hmc5883l.magRaw.axis.X, hmc5883l.magRaw.axis.Y, hmc5883l.magRaw.axis.Z);
    UARTFlushTx(false);
*/
    if (!calibrating) { // If we are not calibrating, then subtract zero values
        for (uint8_t axis = 0; axis < 3; axis++)
            mag->data[axis] -= cfg.magZero.data[axis]; // Subtract zero value stored in EEPROM
    }

#if 0 && UART_DEBUG
    UARTprintf("%d.%03u\t%d.%03u\t%d.%03u\n",
                                            (int16_t)mag->axis.X, (uint16_t)(abs(mag->axis.X * 1000.0f) % 1000),
                                            (int16_t)mag->axis.Y, (uint16_t)(abs(mag->axis.Y * 1000.0f) % 1000),
                                            (int16_t)mag->axis.Z, (uint16_t)(abs(mag->axis.Z * 1000.0f) % 1000));
    UARTFlushTx(false);
#endif
}

static bool checkLimit(sensorRaw_t sensorRaw, int16_t low, int16_t high) {
    bool status = true;
    for (uint8_t axis = 0; axis < 3; axis++) {
        if (sensorRaw.data[axis] < low || sensorRaw.data[axis] > high) {
            status = false;
            break;
        }
    }
    return status;
}

// Inspired by: https://code.google.com/p/open-headtracker and https://github.com/cleanflight/cleanflight/blob/master/src/main/drivers/compass_hmc5883l.c
bool initHMC5883L(void) {
    uint8_t buf[3]; // Buffer for I2C data
    i2cReadData(HMC5883L_ADDRESS, HMC5883L_ID_REG_A, buf, 3);
    if (buf[0] != 'H' || buf[1] != '4' || buf[2] != '3') { // Read identification registers
#if 0 && UART_DEBUG
        UARTprintf("Could not find HMC5883L: %c%c%c\n", buf[0], buf[1], buf[2]);
#endif
        return false;
    }

    // Enable interrupt for DRDY (Data Ready Interrupt Pin)
    SysCtlPeripheralEnable(GPIO_HMC5883L_DRDY_PERIPH); // Enable GPIO peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    GPIOPinTypeGPIOInput(GPIO_HMC5883L_DRDY_BASE, GPIO_HMC5883L_DRDY_PIN); // Set DRDY pin as input
    GPIOIntTypeSet(GPIO_HMC5883L_DRDY_BASE, GPIO_HMC5883L_DRDY_PIN, GPIO_FALLING_EDGE); // Enable interrupt on falling

    IntPrioritySet(INT_GPIOF, 2); // Set interrupt priority to 2
    GPIOIntEnable(GPIO_HMC5883L_DRDY_BASE, GPIO_HMC5883L_DRDY_PIN); // Enable interrupt
    GPIOIntRegister(GPIO_HMC5883L_DRDY_BASE, drdyHandler); // Register interrupt handler

    // Self test according to datasheet: http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5883L_3-Axis_Digital_Compass_IC.pdf page 19
    // Only difference is that I use 2.5 Ga
    static const int16_t LOW_LIMIT = SELF_TEST_LOW_LIMIT * GAIN_25_GA_LSB_GAIN;
    static const int16_t HIGH_LIMIT = SELF_TEST_HIGH_LIMIT * GAIN_25_GA_LSB_GAIN;

    i2cWrite(HMC5883L_ADDRESS, HMC5883L_CONF_REG_A, HMC5883L_CONF_REG_A_MA_8 | HMC5883L_CONF_REG_A_D0_15 | HMC5883L_CONF_REG_A_MS_POS); // Set to 8 samples, update rate to 15 Hz (default) and positive bias configuration
    i2cWrite(HMC5883L_ADDRESS, HMC5883L_CONF_REG_B, HMC5883L_CONF_REG_B_GN_25); // Set gain to 2.5 Ga
    i2cWrite(HMC5883L_ADDRESS, HMC5883L_MODE_REG, HMC5883L_MODE_REG_HS | HMC5883L_MODE_REG_MD_CONT); // Configure device for high speed I2C mode (400 kHz) in continuous mode

    while (!dataReadyHMC5883L()); // Wait for data
    getHMC5883LDataRaw(&hmc5883l.magRaw); // First values are discarded, as it is from previous setting
    delay(100); // This delays seems to prevent it from failing from time to time
    while (!dataReadyHMC5883L()); // Wait for data
    getHMC5883LDataRaw(&hmc5883l.magRaw); // Read positive bias values

    if (!checkLimit(hmc5883l.magRaw, LOW_LIMIT, HIGH_LIMIT)) {
#if UART_DEBUG
        UARTprintf("HMC5883L self test high limit failed: %d < %d %d %d < %d\n", LOW_LIMIT, hmc5883l.magRaw.axis.X, hmc5883l.magRaw.axis.Y, hmc5883l.magRaw.axis.Z, HIGH_LIMIT);
#endif
        while (1);
    }

    sensor_t mag_total = { .data = { 0, 0, 0 } };
    mag_total.axis.X += hmc5883l.magRaw.axis.X;
    mag_total.axis.Y += hmc5883l.magRaw.axis.Y;
    mag_total.axis.Z += hmc5883l.magRaw.axis.Z;

    i2cWrite(HMC5883L_ADDRESS, HMC5883L_CONF_REG_A, HMC5883L_CONF_REG_A_MA_8 | HMC5883L_CONF_REG_A_D0_15 | HMC5883L_CONF_REG_A_MS_NEG); // Set to 8 samples, update rate to 15 Hz (default) and negative bias configuration
    while (!dataReadyHMC5883L()); // Wait for data
    getHMC5883LDataRaw(&hmc5883l.magRaw); // First values are discarded, as it is from previous setting

    while (!dataReadyHMC5883L()); // Wait for data
    getHMC5883LDataRaw(&hmc5883l.magRaw); // Read negative bias values

    if (!checkLimit(hmc5883l.magRaw, -HIGH_LIMIT, -LOW_LIMIT)) {
#if UART_DEBUG
        UARTprintf("HMC5883L self test low limit failed: %d < %d %d %d < %d\n", -HIGH_LIMIT, hmc5883l.magRaw.axis.X, hmc5883l.magRaw.axis.Y, hmc5883l.magRaw.axis.Z, -LOW_LIMIT);
#endif
        while (1);
    }

    mag_total.axis.X -= hmc5883l.magRaw.axis.X;
    mag_total.axis.Y -= hmc5883l.magRaw.axis.Y;
    mag_total.axis.Z -= hmc5883l.magRaw.axis.Z;

    // Calculate gain
    hmc5883l.magGain.axis.X = GAIN_25_GA_LSB_GAIN * HMC5883L_X_SELF_TEST_GAUSS / (mag_total.axis.X / 2.0f);
    hmc5883l.magGain.axis.Y = GAIN_25_GA_LSB_GAIN * HMC5883L_Y_SELF_TEST_GAUSS / (mag_total.axis.Y / 2.0f);
    hmc5883l.magGain.axis.Z = GAIN_25_GA_LSB_GAIN * HMC5883L_Z_SELF_TEST_GAUSS / (mag_total.axis.Z / 2.0f);

    i2cWrite(HMC5883L_ADDRESS, HMC5883L_CONF_REG_A, HMC5883L_CONF_REG_A_MA_8 | HMC5883L_CONF_REG_A_D0_15 | HMC5883L_CONF_REG_A_MS_NORM); // Set to 8 samples, update rate to 15 Hz (default) and normal measurement configuration
    i2cWrite(HMC5883L_ADDRESS, HMC5883L_CONF_REG_B, HMC5883L_CONF_REG_B_GN_13); // Set gain to 1.3 Ga (default)
    while (!dataReadyHMC5883L()); // Wait for data
    getHMC5883LDataRaw(&hmc5883l.magRaw); // First values are discarded, as it is from previous gain

#if 0 && UART_DEBUG
    UARTprintf("Mag cal: %d\t%d\t%d\n", (int16_t)mag_total.axis.X, (int16_t)mag_total.axis.Y, (int16_t)mag_total.axis.Z);
    UARTprintf("Mag gain: %d.%03u\t%d.%03u\t%d.%03u\n",
                                                    (int16_t)hmc5883l.magGain.axis.X, (uint16_t)(abs(hmc5883l.magGain.axis.X * 1000.0f) % 1000),
                                                    (int16_t)hmc5883l.magGain.axis.Y, (uint16_t)(abs(hmc5883l.magGain.axis.Y * 1000.0f) % 1000),
                                                    (int16_t)hmc5883l.magGain.axis.Z, (uint16_t)(abs(hmc5883l.magGain.axis.Z * 1000.0f) % 1000));
    UARTFlushTx(false);
#endif

    return true;
}

#endif // USE_MAG
