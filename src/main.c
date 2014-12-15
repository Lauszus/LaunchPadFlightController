/* Copyright (C) 2014 Kristian Lauszus, TKJ Electronics. All rights reserved.

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
#include <stdlib.h>

#include "RX.h"
#include "UART.h"
#include "time.h"
#include "PPM.h"
#include "PID.h"
#include "MPU6500.h"

#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

// Only acro mode is actually working for now
#define ACRO_MODE 1

#define SYSCTL_PERIPH_LED SYSCTL_PERIPH_GPIOF
#define GPIO_LED_BASE     GPIO_PORTF_BASE
#define GPIO_RED_LED      GPIO_PIN_1
#define GPIO_GREEN_LED    GPIO_PIN_3

int main(void) {
    // Set the clocking to run directly from the external crystal/oscillator.
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set clock to 80MHz (400MHz(PLL) / 2 / 2.5 = 80 MHz)

    initPPM();
    initUART();
    delay(100);
    UARTprintf("Started\n");
    initTime();
    initRX();

    //initMPU6500();
    initMPU6500_i2c();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_LED); // Enable GPIOF peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinTypeGPIOOutput(GPIO_LED_BASE, GPIO_RED_LED | GPIO_GREEN_LED); // Set red and blue LEDs as outputs

    IntMasterEnable();

    delay(100); // Wait a little for everything to initialize

    UARTprintf("CLK %d\n", SysCtlClockGet());
    UARTprintf("min: %d, max: %d, period: %d\n", PPM_MIN, PPM_MAX, getPeriod());

#if !ACRO_MODE
    const float restAngleRoll = 1.67f, restAnglePitch = -2.55f; // TODO: Make a calibration routine for these values
#endif

#if ACRO_MODE
    // 0.0200.0.1000.0.0000
    pidRoll.Kp = 0.016f;
    pidRoll.Ki = 0.050f;
    pidRoll.Kd = 0.0f;
#else
    pidRoll.Kp = 1.75f;
    pidRoll.Ki = 1.0f;//2.3f;
    pidRoll.Kd = 0.0f;
#endif
    pidRoll.integratedError = 0.0f;
    pidRoll.lastError = 0.0f;

    pidPitch = pidRoll; // Use same PID values for both pitch and roll

    // x2 the values work pretty well - TODO: Fine-tune these
    pidYaw = pidRoll;
    pidYaw.Kp *= 2.0f;
    pidYaw.Ki *= 2.8f; // I increased this in order for it to stop yawing slowly
    pidYaw.Kd *= 2.0f;

    printPIDValues();

#if ACRO_MODE
    static int16_t gyroData[3];
#else
    float roll, pitch;
#endif
    static uint32_t imuTimer = 0, pidTimer = 0;

    // Motor 0 is bottom right, motor 1 is top right, motor 2 is bottom left and motor 3 is top left
    static float motors[4] = { -100.0f, -100.0f, -100.0f, -100.0f };

    static bool armed = false;

    while (!validRXData || rxChannel[RX_AUX1_CHAN] > 1000) {
        // Wait until we have valid data and we are unarmed
    }

    while (1) {
        checkUARTData();

        // Make sure there is valid data, AUX channel is armed and that throttle is applied
        // The throttle check can be removed if one prefer the motors to spin once it is armed
        // TODO: Arm using throttle low and yaw right
        if (!validRXData || rxChannel[RX_AUX1_CHAN] < 1000 || rxChannel[RX_THROTTLE_CHAN] < RX_MIN_INPUT + 25) {
            writePPMAllOff();
            pidRoll.integratedError = pidRoll.lastError = 0.0f;
            pidPitch.integratedError = pidPitch.lastError = 0.0f;
            pidYaw.integratedError = pidYaw.lastError = 0.0f;
            armed = false;
        } else
            armed = true;

        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_RED_LED | GPIO_GREEN_LED, !validRXData || rxChannel[RX_AUX1_CHAN] < 1000 ? GPIO_GREEN_LED : GPIO_RED_LED); // Turn on red led if there is valid data and AUX channel is in armed position otherwise turn on green LED

        if (dataReadyMPU6500()) {
            float dt = (float)(micros() - imuTimer) / 1000000.0f;
            //UARTprintf("%d\n", micros() - imuTimer);
            imuTimer = micros();

#if ACRO_MODE
            getMPU6500Gyro(gyroData);

            /*UARTprintf("%d\t%d\t%d\n", gyroData[0], gyroData[1], gyroData[2]);
            UARTFlushTx(false);*/
#else
            getMPU6500Angles(&roll, &pitch, dt);

            /*UARTprintf("%d.%02d\t%d.%02d\n", (int16_t)roll, (int16_t)abs(roll * 100.0f) % 100, (int16_t)pitch, (int16_t)abs(pitch * 100.0f) % 100);
            UARTFlushTx(false);*/
#endif
        }

        float dt = (float)(micros() - pidTimer);
        if (armed && dt > 2500) { // Limit to 2.5ms (400 Hz)
            //UARTprintf("%d\n", micros() - pidTimer);
            pidTimer = micros();
            dt /= 1000000.0f; // Convert to seconds

#if ACRO_MODE
            float rollOut = updatePID(&pidRoll, 0, gyroData[1], dt);
            float pitchOut = updatePID(&pidPitch, 0, gyroData[0], dt);
#else
            float rollOut = updatePID(&pidRoll, restAngleRoll, roll, dt);
            float pitchOut = updatePID(&pidPitch, restAnglePitch, pitch, dt);
#endif
            float yawOut = updatePID(&pidYaw, 0, gyroData[2], dt);

            float throttle = map(rxChannel[RX_THROTTLE_CHAN], RX_MIN_INPUT, RX_MAX_INPUT, -100.0f, 100.0f);
            for (uint8_t i = 0; i < 4; i++)
                motors[i] = throttle;

            motors[0] -= rollOut;
            motors[1] -= rollOut;
            motors[2] += rollOut;
            motors[3] += rollOut;

            motors[0] += pitchOut;
            motors[1] -= pitchOut;
            motors[2] += pitchOut;
            motors[3] -= pitchOut;

            motors[0] -= yawOut;
            motors[1] += yawOut;
            motors[2] += yawOut;
            motors[3] -= yawOut;

            for (uint8_t i = 0; i < 4; i++)
                motors[i] = constrain(motors[i], -50.0f, 50.0f);

            // Pitch Control
            float elevator = map(rxChannel[RX_ELEVATOR_CHAN], RX_MIN_INPUT, RX_MAX_INPUT, -100.0f, 100.0f);
            motors[0] += elevator / 2.0f;
            motors[1] -= elevator / 2.0f;
            motors[2] += elevator / 2.0f;
            motors[3] -= elevator / 2.0f;

            // Roll Control
            float aileron = map(rxChannel[RX_AILERON_CHAN], RX_MIN_INPUT, RX_MAX_INPUT, -100.0f, 100.0f);
            motors[0] -= aileron / 2.0f;
            motors[1] -= aileron / 2.0f;
            motors[2] += aileron / 2.0f;
            motors[3] += aileron / 2.0f;

            // Rudder Control
            float rudder = map(rxChannel[RX_RUDDER_CHAN], RX_MIN_INPUT, RX_MAX_INPUT, -100.0f, 100.0f);
            motors[0] -= rudder;
            motors[1] += rudder;
            motors[2] += rudder;
            motors[3] -= rudder;

            updateMotorsAll(motors);

            //UARTprintf("%d\t%d\n", (int16_t)elevator, (int16_t)aileron);
#if 0
            UARTprintf("%d\t%d\t\t", (int16_t)roll, (int16_t)pitch);
            UARTprintf("%d\t%d\t\t", (int16_t)rollOut, (int16_t)pitchOut);
            UARTprintf("%d\t%d\t%d\t%d\n", (int16_t)motors[0], (int16_t)motors[1], (int16_t)motors[2], (int16_t)motors[3]);
            UARTFlushTx(false);
#endif
        }
    }
}

// TODO:
    // Save PID values in EEPROM
    // Adjust PID values using pots on transmitter
    // Only enable peripheral clock once
    // Tune yaw PID values separately
    // Make limit of integrated error adjustable
    // Get self-level working - enable DLPF for accelerometer
    // Use SPI instead of I2C
