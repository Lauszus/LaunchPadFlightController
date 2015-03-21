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

#include "I2C.h"
#include "MPU6500.h"
#include "time.h"
#include "KalmanX.h"
#include "KalmanY.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

#define MPU6500_ADDRESS 0x68
#define PI              3.1415926535897932384626433832795f
#define RAD_TO_DEG      57.295779513082320876798154814105f

#define GPIO_MPU_INT_PERIPH     SYSCTL_PERIPH_GPIOE
#define GPIO_MPU_INT_BASE       GPIO_PORTE_BASE
#define GPIO_MPU_INT_PIN        GPIO_PIN_3

static int16_t gyroZero[3];

// Returns true when data is ready to be read
bool dataReadyMPU6500(void) {
    return GPIOPinRead(GPIO_MPU_INT_BASE, GPIO_MPU_INT_PIN);
}

// Returns raw accelerometer data and gyro data with zero values subtracted
void getMPU6500Data(int16_t *accData, int16_t *gyroData) {
    uint8_t buf[14];

    i2cReadData(MPU6500_ADDRESS, 0x3B, buf, 14);

    accData[0] = (buf[0] << 8) | buf[1]; // X
    accData[1] = (buf[2] << 8) | buf[3]; // Y
    accData[2] = (buf[4] << 8) | buf[5]; // Z

    gyroData[0] = (buf[8] << 8) | buf[9]; // X
    gyroData[1] = (buf[10] << 8) | buf[11]; // Y
    gyroData[2] = (buf[12] << 8) | buf[13]; // Z

    for (uint8_t axis = 0; axis < 3; axis++)
        gyroData[axis] -= gyroZero[axis]; // Subtract gyro zero values

    // TODO: Subtract accelerometer zero values as well
}

// Accelerometer readings can be in any scale, but gyro date needs to be in deg/s
void getMPU6500Angles(int16_t *accData, float *gyroRate, float *roll, float *pitch, float dt) {
    const float accz_lpf_cutoff = 5.0f; // Source: https://github.com/cleanflight/cleanflight
    const float fc_acc = 0.5f / (PI * accz_lpf_cutoff); // Calculate RC time constant used in the accZ lpf
    static float accz_smooth = 0;
    accz_smooth += (dt / (fc_acc + dt)) * (accData[2] - accz_smooth); // Low pass filter

    // Pitch should increase when pitching quadcopter downward
    // and roll should increase when tilting quadcopter clockwise

    /*static float gyroAngle[3] = { 0, 0, 0 };
    for (uint8_t axis = 0; axis < 3; axis++)
        gyroAngle[axis] += gyroRate[axis] * dt; // Gyro angle is only used for debugging*/

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees
    float rollAcc = atanf(accData[0] / sqrtf(accData[1] * accData[1] + accz_smooth * accz_smooth)) * RAD_TO_DEG;
    float pitchAcc  = atan2f(-accData[1], -accz_smooth) * RAD_TO_DEG;

    *roll = getAngleX(rollAcc, gyroRate[1], dt);
    *pitch = getAngleY(pitchAcc, gyroRate[0], dt);
/*
    UARTprintf("%d\t%d\t%d\t\t", (int16_t)rollAcc, (int16_t)gyroAngle[1], (int16_t)*roll);
    delay(1);
    UARTprintf("%d\t%d\t%d\n", (int16_t)pitchAcc, (int16_t)gyroAngle[0], (int16_t)*pitch);
    UARTFlushTx(false);
*/
/*
    static float compAngleX, compAngleY;
    compAngleX = 0.93f * (compAngleX + gyroRate[0] * dt) + 0.07f * rollAcc; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93f * (compAngleY + gyroRate[1] * dt) + 0.07f * pitchAcc;
*/
}

/*
void printMPU6050Debug(void) {
    while (1) {
#if 0
        UARTprintf("%d\t%d\t\t", (int16_t)KalmanX, (int16_t)KalmanY);
        UARTprintf("%d\t%d\t\t", (int16_t)compAngleX, (int16_t)compAngleY);
        UARTprintf("%d\t%d\t\t", (int16_t)roll, (int16_t)pitch);
        UARTprintf("%d\t%d\t%d\n", (int16_t)gyroAngle[0], (int16_t)gyroAngle[1], (int16_t)gyroAngle[2]);
#else
        UARTprintf("%d\t%d\t", (int16_t)roll, (int16_t)gyroAngle[0]);
        delay(1);
        UARTprintf("%d\t%d\t\t", (int16_t)compAngleX, (int16_t)KalmanX);
        delay(1);
        UARTprintf("%d\t%d\t", (int16_t)pitch, (int16_t)gyroAngle[1]);
        delay(1);
        UARTprintf("%d\t%d\t\n", (int16_t)compAngleY, (int16_t)KalmanY);
#endif
        delay(10);
    }
}*/

void initMPU6500(void) {
    uint8_t i2cBuffer[5]; // Buffer for I2C data

    i2cBuffer[0] = i2cRead(MPU6500_ADDRESS, 0x75);
    if (i2cBuffer[0] == 0x70) // Read "WHO_AM_I" register
        UARTprintf("MPU-6500 found\n");
    else {
        UARTprintf("Could not find MPU-6500: %2X\n", i2cBuffer[0]);
        while (1);
    }

    i2cWrite(MPU6500_ADDRESS, 0x6B, (1 << 7)); // Reset device, this resets all internal registers to their default values
    delay(100);
    while (i2cRead(MPU6500_ADDRESS, 0x6B) & (1 << 7)) {
        // Wait for the bit to clear
    };
    delay(100);
    i2cWrite(MPU6500_ADDRESS, 0x6B, (1 << 3) | (1 << 0)); // Disable sleep mode, disable temperature sensor and use PLL as clock reference

    i2cBuffer[0] = 0; // Set the sample rate to 1kHz - 1kHz/(1+0) = 1kHz
    i2cBuffer[1] = 0x03; // Disable FSYNC and set 41 Hz Gyro filtering, 1 KHz sampling
    i2cBuffer[2] = 3 << 3; // Set Gyro Full Scale Range to +-2000deg/s
    i2cBuffer[3] = 2 << 3; // Set Accelerometer Full Scale Range to +-8g
    i2cBuffer[4] = 0x03; // 41 Hz Acc filtering
    i2cWriteData(MPU6500_ADDRESS, 0x19, i2cBuffer, 5); // Write to all five registers at once

    /* Enable Raw Data Ready Interrupt on INT pin */
    i2cBuffer[0] = (1 << 5) | (1 << 4); // Enable LATCH_INT_EN and INT_ANYRD_2CLEAR
                                        // When this bit is equal to 1, the INT pin is held high until the interrupt is cleared
                                        // When this bit is equal to 1, interrupt status is cleared if any read operation is performed
    i2cBuffer[1] = (1 << 0); // Enable RAW_RDY_EN - When set to 1, Enable Raw Sensor Data Ready interrupt to propagate to interrupt pin
    i2cWriteData(MPU6500_ADDRESS, 0x37, i2cBuffer, 2); // Write to both registers at once

    // Set INT input pin
    SysCtlPeripheralEnable(GPIO_MPU_INT_PERIPH); // Enable GPIO peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinTypeGPIOInput(GPIO_MPU_INT_BASE, GPIO_MPU_INT_PIN); // Set as input

    delay(100); // Wait for sensor to stabilize

    //printMPU6050Debug();

    while (!dataReadyMPU6500()) {
        // Wait until date is ready
    }

    // TOOD: Read gyro values multiple times and check if it's moved while doing so
    // Get gyro zero values
    uint8_t buf[6];
    i2cReadData(MPU6500_ADDRESS, 0x43, buf, 6);
    gyroZero[0] = (buf[0] << 8) | buf[1]; // X
    gyroZero[1] = (buf[2] << 8) | buf[3]; // Y
    gyroZero[2] = (buf[4] << 8) | buf[5]; // Z

    KalmanXInit();
    KalmanYInit();

    setAngleX(0.0f); // Set starting angle
    setAngleY(0.0f);
}
