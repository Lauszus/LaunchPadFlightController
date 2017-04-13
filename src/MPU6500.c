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

#include "Config.h"
#include "EEPROM.h"
#include "I2C.h"
#include "MPU6500.h"
#include "Time.h"

#include "driverlib/gpio.h"
#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

#define MPU6500_SMPLRT_DIV                  0x19
#define MPU6500_INT_PIN_CFG                 0x37
#define MPU6500_ACCEL_XOUT_H                0x3B
#define MPU6500_GYRO_XOUT_H                 0x43
#define MPU6500_PWR_MGMT_1                  0x6B
#define MPU6500_WHO_AM_I                    0x75

#define MPU6500_ADDRESS                     0x68
#define MPU6500_WHO_AM_I_ID                 0x70
#define MPU9250_WHO_AM_I_ID                 0x71

// Scale factor for +-2000deg/s and +-8g - see datasheet: http://www.invensense.com/mems/gyro/documents/PS-MPU-6500A-01.pdf at page 9-10
#define MPU6500_GYRO_SCALE_FACTOR_2000      16.4f
#define MPU6500_ACC_SCALE_FACTOR_8          4096.0f

static sensorRaw_t gyroZero; // Gyroscope zero values are found at every power on

// Returns true when data is ready to be read
bool dataReadyMPU6500(void) {
    return GPIOPinRead(GPIO_MPU_INT_BASE, GPIO_MPU_INT_PIN);
}

// X-axis should be facing forward
// Y-axis should be facing to the right
// Z-axis should be facing downward
void mpu6500BoardOrientation(sensorRaw_t *sensorRaw) {
    sensorRaw_t sensorRawTemp = *sensorRaw;
    sensorRaw->axis.X = -sensorRawTemp.axis.Y;
    sensorRaw->axis.Y = -sensorRawTemp.axis.X;
    sensorRaw->axis.Z = -sensorRawTemp.axis.Z;
}

// Returns accelerometer and gyro data with zero values subtracted
void getMPU6500Data(mpu6500_t *mpu6500) {
    uint8_t buf[14];
    i2cReadData(MPU6500_ADDRESS, MPU6500_ACCEL_XOUT_H, buf, 14); // Note that we can't write directly into mpu6500_t, because of endian conflict. So it has to be done manually

    mpu6500->acc.axis.X = -((int16_t)((buf[0] << 8) | buf[1])); // The MPU-6500/MPU-9250 assumes that it reads +1g when pointing away from gravity, so we have to invert the values
    mpu6500->acc.axis.Y = -((int16_t)((buf[2] << 8) | buf[3]));
    mpu6500->acc.axis.Z = -((int16_t)((buf[4] << 8) | buf[5]));

    mpu6500->gyro.axis.X = (int16_t)((buf[8] << 8) | buf[9]);
    mpu6500->gyro.axis.Y = (int16_t)((buf[10] << 8) | buf[11]);
    mpu6500->gyro.axis.Z = (int16_t)((buf[12] << 8) | buf[13]);

    mpu6500BoardOrientation(&mpu6500->acc); // Apply board orientation
    mpu6500BoardOrientation(&mpu6500->gyro);

    for (uint8_t axis = 0; axis < 3; axis++) {
        mpu6500->acc.data[axis] -= cfg.accZero.data[axis]; // Subtract accelerometer zero values
        mpu6500->gyro.data[axis] -= gyroZero.data[axis]; // Subtract gyro zero values
        mpu6500->gyroRate.data[axis] = (float)mpu6500->gyro.data[axis] / mpu6500->gyroScaleFactor; // Convert to deg/s
    }
/*
    // Acceleration should be positive when aligned with the gravity vector and gyro should follow the right hand rule
    UARTprintf("%d\t%d\t%d\t", mpu6500->acc.axis.X, mpu6500->acc.axis.Y, mpu6500->acc.axis.Z);
    UARTprintf("%d\t%d\t%d\n", mpu6500->gyro.axis.X, mpu6500->gyro.axis.Y, mpu6500->gyro.axis.Z);
    UARTFlushTx(false);
*/
}

static bool checkMinMax(int32_t *array, uint8_t length, int16_t maxDifference) { // Used to check that the flight controller is not moved while calibrating
    int32_t min = array[0], max = array[0];
    for (uint8_t i = 1; i < length; i++) {
        if (array[i] < min)
            min = array[i];
        else if (array[i] > max)
            max = array[i];
    }
    return max - min < maxDifference;
}

static bool calibrateSensor(sensorRaw_t *zeroValues, uint8_t regAddr, int16_t maxDifference) {
    static int32_t sensorBuffer[3][25];
    static const uint8_t bufLength = 25;
    uint8_t buf[6];

    for (uint8_t i = 0; i < bufLength; i++) {
        while (!dataReadyMPU6500()) {
            // Wait until new date is ready
        }
        i2cReadData(MPU6500_ADDRESS, regAddr, buf, 6);
        sensorBuffer[0][i] = (int16_t)((buf[0] << 8) | buf[1]); // X
        sensorBuffer[1][i] = (int16_t)((buf[2] << 8) | buf[3]); // Y
        sensorBuffer[2][i] = (int16_t)((buf[4] << 8) | buf[5]); // Z
        delay(10);
    }

    for (uint8_t axis = 0; axis < 3; axis++) {
        if (!checkMinMax(sensorBuffer[axis], bufLength, maxDifference))
            return 1; // Return error
    }

    for (uint8_t axis = 0; axis < 3; axis++) {
        for (uint8_t i = 1; i < bufLength; i++)
            sensorBuffer[axis][0] += sensorBuffer[axis][i]; // Sum up all readings
        zeroValues->data[axis] = sensorBuffer[axis][0] / bufLength; // Get average
    }

    mpu6500BoardOrientation(zeroValues); // Apply board orientation

    return 0; // No error
}

static bool calibrateMPU6500Gyro(void) {
    bool rcode = calibrateSensor(&gyroZero, MPU6500_GYRO_XOUT_H, 100); // 100 / 16.4 ~= 6.10 deg/s

#if UART_DEBUG
    if (!rcode)
        UARTprintf("Gyro zero values: %d\t%d\t%d\n", gyroZero.axis.X, gyroZero.axis.Y, gyroZero.axis.Z);
    else
        UARTprintf("Gyro calibration error\n");
#endif

    return rcode;
}

bool calibrateMPU6500Acc(mpu6500_t *mpu6500) {
    bool rcode = calibrateSensor(&cfg.accZero, MPU6500_ACCEL_XOUT_H, 100); // 100 / 4096 ~= 0.02g
    if (!rcode) {
        // The MPU-6500/MPU-9250 assumes that it reads +1g when pointing away from gravity, so we have to invert the values
        cfg.accZero.axis.X = -cfg.accZero.axis.X;
        cfg.accZero.axis.Y = -cfg.accZero.axis.Y;
        cfg.accZero.axis.Z = -cfg.accZero.axis.Z;
        cfg.accZero.axis.Z -= mpu6500->accScaleFactor; // Z-axis should be reading +1g when horizontal, so we subtract 1g from the value
#if UART_DEBUG
        UARTprintf("Accelerometer zero values: %d\t%d\t%d\n", cfg.accZero.axis.X, cfg.accZero.axis.Y, cfg.accZero.axis.Z);
#endif
        updateConfig(); // Write new values to EEPROM
    }
#if UART_DEBUG
    else
        UARTprintf("Accelerometer calibration error\n");
#endif

    return rcode; // No error
}

void initMPU6500(mpu6500_t *mpu6500) {
    uint8_t i2cBuffer[5]; // Buffer for I2C data

    i2cBuffer[0] = i2cRead(MPU6500_ADDRESS, MPU6500_WHO_AM_I);
    if (i2cBuffer[0] == MPU6500_WHO_AM_I_ID) { // Read "WHO_AM_I" register
#if UART_DEBUG
        UARTprintf("MPU-6500 found\n");
#endif
    } else if (i2cBuffer[0] == MPU9250_WHO_AM_I_ID) {
#if UART_DEBUG
        UARTprintf("MPU-9250 found\n");
#endif
    } else {
#if UART_DEBUG
        UARTprintf("Could not find MPU-6500 or MPU-9250: %02X\n", i2cBuffer[0]);
#endif
        while (1);
    }

    i2cWrite(MPU6500_ADDRESS, MPU6500_PWR_MGMT_1, (1 << 7)); // Reset device, this resets all internal registers to their default values
    delay(100);
    while (i2cRead(MPU6500_ADDRESS, MPU6500_PWR_MGMT_1) & (1 << 7)) {
        // Wait for the bit to clear
    };
    delay(100);
    i2cWrite(MPU6500_ADDRESS, MPU6500_PWR_MGMT_1, (1 << 3) | (1 << 0)); // Disable sleep mode, disable temperature sensor and use PLL as clock reference

    i2cBuffer[0] = 0; // Set the sample rate to 1kHz - 1kHz/(1+0) = 1kHz
    i2cBuffer[1] = 0x03; // Disable FSYNC and set 41 Hz Gyro filtering, 1 KHz sampling
    i2cBuffer[2] = 3 << 3; // Set Gyro Full Scale Range to +-2000deg/s
    i2cBuffer[3] = 2 << 3; // Set Accelerometer Full Scale Range to +-8g
    i2cBuffer[4] = 0x03; // 41 Hz Acc filtering
    i2cWriteData(MPU6500_ADDRESS, MPU6500_SMPLRT_DIV, i2cBuffer, 5); // Write to all five registers at once

    // Set accelerometer and gyroscope scale factor from datasheet
    mpu6500->gyroScaleFactor = MPU6500_GYRO_SCALE_FACTOR_2000;
    mpu6500->accScaleFactor = MPU6500_ACC_SCALE_FACTOR_8;

    /* Enable Raw Data Ready Interrupt on INT pin and enable bypass/passthrough mode */
    i2cBuffer[0] = (1 << 5) | (1 << 4) | (1 << 1); // Enable LATCH_INT_EN, INT_ANYRD_2CLEAR and BYPASS_EN
                                                   // When this bit is equal to 1, the INT pin is held high until the interrupt is cleared
                                                   // When this bit is equal to 1, interrupt status is cleared if any read operation is performed
                                                   // When asserted, the I2C_MASTER interface pins (ES_CL and ES_DA) will go into 'bypass mode' when the I2C master interface is disabled
    i2cBuffer[1] = (1 << 0);                       // Enable RAW_RDY_EN - When set to 1, Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin
    i2cWriteData(MPU6500_ADDRESS, MPU6500_INT_PIN_CFG, i2cBuffer, 2); // Write to both registers at once

    // Set INT input pin
    SysCtlPeripheralEnable(GPIO_MPU_INT_PERIPH); // Enable GPIO peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinTypeGPIOInput(GPIO_MPU_INT_BASE, GPIO_MPU_INT_PIN); // Set as input

    delay(100); // Wait for sensor to stabilize

    while (calibrateMPU6500Gyro()) { // Get gyro zero values
        // Loop until calibration is successful
    }
}
