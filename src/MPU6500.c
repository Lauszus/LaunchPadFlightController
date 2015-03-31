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

#include "EEPROM.h"
#include "I2C.h"
#include "MPU6500.h"
#include "Time.h"
#include "Kalman.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor
#endif

#define MPU6500_SMPLRT_DIV          0x19
#define MPU6500_INT_PIN_CFG         0x37
#define MPU6500_ACCEL_XOUT_H        0x3B
#define MPU6500_GYRO_XOUT_H         0x43
#define MPU6500_PWR_MGMT_1          0x6B
#define MPU6500_WHO_AM_I            0x75

#define MPU6500_ADDRESS             0x68
#define MPU6500_WHO_AM_I_ID         0x70

#define MPU6500_GYRO_SCALE_FACTOR   16.4f // Scale factor for +-2000deg/s - see datasheet: http://www.invensense.com/mems/gyro/documents/PS-MPU-6500A-01.pdf at page 9

#define PI                          3.1415926535897932384626433832795f
#define RAD_TO_DEG                  57.295779513082320876798154814105f

#define GPIO_MPU_INT_PERIPH         SYSCTL_PERIPH_GPIOE
#define GPIO_MPU_INT_BASE           GPIO_PORTE_BASE
#define GPIO_MPU_INT_PIN            GPIO_PIN_3

static gyro_t gyroZero; // Gyroscope zero values are found at every power on

// Returns true when data is ready to be read
bool dataReadyMPU6500(void) {
    return GPIOPinRead(GPIO_MPU_INT_BASE, GPIO_MPU_INT_PIN);
}

// Returns accelerometer and gyro data with zero values subtracted
void getMPU6500Data(mpu6500_t *mpu6500) {
    uint8_t buf[14];
    i2cReadData(MPU6500_ADDRESS, MPU6500_ACCEL_XOUT_H, buf, 14); // Note that we can't write directly into mpu6500_t, because of endian conflict. So it has to be done manually

    mpu6500->acc.X = (buf[0] << 8) | buf[1];
    mpu6500->acc.Y = (buf[2] << 8) | buf[3];
    mpu6500->acc.Z = (buf[4] << 8) | buf[5];

    mpu6500->gyro.X = (buf[8] << 8) | buf[9];
    mpu6500->gyro.Y = (buf[10] << 8) | buf[11];
    mpu6500->gyro.Z = (buf[12] << 8) | buf[13];

    for (uint8_t axis = 0; axis < 3; axis++) {
        mpu6500->acc.data[axis] -= cfg.accZero.data[axis]; // Subtract accelerometer zero values
        mpu6500->gyro.data[axis] -= gyroZero.data[axis]; // Subtract gyro zero values
        mpu6500->gyroRate.data[axis] = ((float)mpu6500->gyro.data[axis]) / MPU6500_GYRO_SCALE_FACTOR; // Convert to deg/s
    }
}

// Accelerometer readings can be in any scale, but gyro rate needs to be in deg/s
void getMPU6500Angles(mpu6500_t *mpu6500, kalman_t *kalmanRoll, kalman_t *kalmanPitch, float dt) {
    const float accz_lpf_cutoff = 5.0f; // Source: https://github.com/cleanflight/cleanflight
    const float fc_acc = 0.5f / (PI * accz_lpf_cutoff); // Calculate RC time constant used in the accZ lpf
    static float accz_smooth = 0;
    accz_smooth += (dt / (fc_acc + dt)) * (mpu6500->acc.Z - accz_smooth); // Low pass filter

    // Pitch should increase when pitching quadcopter downward
    // and roll should increase when tilting quadcopter clockwise

    /*static float gyroAngle[3] = { 0, 0, 0 };
    for (uint8_t axis = 0; axis < 3; axis++)
        gyroAngle[axis] += mpu6500->gyroRate.data[axis] * dt; // Gyro angle is only used for debugging*/

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
    float rollAcc = atanf(mpu6500->acc.X / sqrtf(mpu6500->acc.Y * mpu6500->acc.Y + accz_smooth * accz_smooth)) * RAD_TO_DEG;
    float pitchAcc  = atan2f(-mpu6500->acc.Y, -accz_smooth) * RAD_TO_DEG;

    getAngle(kalmanRoll, rollAcc, mpu6500->gyroRate.Y, dt);
    getAngle(kalmanPitch, pitchAcc, mpu6500->gyroRate.X, dt);
#if 0
    static float compAngleRoll, compAnglePitch;
    compAngleRoll = 0.93f * (compAngleRoll + mpu6500->gyroRate.Y * dt) + 0.07f * rollAcc; // Calculate the angle using a Complimentary filter
    compAnglePitch = 0.93f * (compAnglePitch + mpu6500->gyroRate.X * dt) + 0.07f * pitchAcc;

    UARTprintf("%d\t%d\t%d\t\t", (int16_t)rollAcc, (int16_t)gyroAngle[1], (int16_t)*roll);
    delay(1);
    UARTprintf("%d\t%d\t%d\n", (int16_t)pitchAcc, (int16_t)gyroAngle[0], (int16_t)*pitch);
    UARTFlushTx(false);
#endif
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

static bool calibrateSensor(int16_t *zeroValues, uint8_t regAddr, int16_t maxDifference) {
    const uint8_t bufLength = 25;
    static int32_t sensorBuffer[3][bufLength];
    uint8_t buf[6];

    for (uint8_t i = 0; i < bufLength; i++) {
        while (!dataReadyMPU6500()) {
            // Wait until new date is ready
        }
        i2cReadData(MPU6500_ADDRESS, regAddr, buf, 6);
        sensorBuffer[0][i] = (buf[0] << 8) | buf[1]; // X
        sensorBuffer[1][i] = (buf[2] << 8) | buf[3]; // Y
        sensorBuffer[2][i] = (buf[4] << 8) | buf[5]; // Z
        delay(10);
    }

    for (uint8_t axis = 0; axis < 3; axis++) {
        if (!checkMinMax(sensorBuffer[axis], bufLength, maxDifference))
            return 1; // Return error
    }

    for (uint8_t axis = 0; axis < 3; axis++) {
        for (uint8_t i = 1; i < bufLength; i++)
            sensorBuffer[axis][0] += sensorBuffer[axis][i]; // Sum up all readings
        zeroValues[axis] = sensorBuffer[axis][0] / bufLength; // Get average
    }

    return 0; // No error
}

static bool calibrateGyro(void) {
    bool rcode = calibrateSensor(gyroZero.data, MPU6500_GYRO_XOUT_H, 100); // 100 / 16.4 ~= 6.10 deg/s

    if (!rcode) {
#if UART_DEBUG
        UARTprintf("Gyro zero values: %d\t%d\t%d\n", gyroZero.X, gyroZero.Y, gyroZero.Z);
#endif
    } else {
#if UART_DEBUG
        UARTprintf("Gyro calibration error\n");
#endif
        // TODO: Turn on buzzer
    }

    return rcode;
}

bool calibrateAcc(void) {
    bool rcode = calibrateSensor(cfg.accZero.data, MPU6500_ACCEL_XOUT_H, 100); // 100 / 4096 ~= 0.02g
    cfg.accZero.Z += 4096; // Z-axis is reading -1g when horizontal, so we add 1g to the value found

    if (!rcode) {
#if UART_DEBUG
        UARTprintf("Accelerometer zero values: %d\t%d\t%d\n", cfg.accZero.X, cfg.accZero.Y, cfg.accZero.Z);
#endif
        updateConfig(); // Write new values to EEPROM
    } else {
#if UART_DEBUG
        UARTprintf("Accelerometer calibration error\n");
#endif
        // TODO: Turn on buzzer
    }

    return rcode; // No error
}

void initMPU6500(void) {
    uint8_t i2cBuffer[5]; // Buffer for I2C data

    i2cBuffer[0] = i2cRead(MPU6500_ADDRESS, MPU6500_WHO_AM_I);
    if (i2cBuffer[0] == MPU6500_WHO_AM_I_ID) { // Read "WHO_AM_I" register
#if UART_DEBUG
        UARTprintf("MPU-6500 found\n");
#endif
    } else {
#if UART_DEBUG
        UARTprintf("Could not find MPU-6500: %2X\n", i2cBuffer[0]);
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

    /* Enable Raw Data Ready Interrupt on INT pin */
    i2cBuffer[0] = (1 << 5) | (1 << 4); // Enable LATCH_INT_EN and INT_ANYRD_2CLEAR
                                        // When this bit is equal to 1, the INT pin is held high until the interrupt is cleared
                                        // When this bit is equal to 1, interrupt status is cleared if any read operation is performed
    i2cBuffer[1] = (1 << 0);            // Enable RAW_RDY_EN - When set to 1, Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin
    i2cWriteData(MPU6500_ADDRESS, MPU6500_INT_PIN_CFG, i2cBuffer, 2); // Write to both registers at once

    // Set INT input pin
    SysCtlPeripheralEnable(GPIO_MPU_INT_PERIPH); // Enable GPIO peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinTypeGPIOInput(GPIO_MPU_INT_BASE, GPIO_MPU_INT_PIN); // Set as input

    delay(100); // Wait for sensor to stabilize

    //printMPU6050Debug();

    while (calibrateGyro()) { // Get gyro zero values
        // Loop until calibration is succesful
    }
}
