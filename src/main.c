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
#include <stdlib.h>

#include "I2C.h"
#include "RX.h"
#include "UART.h"
#include "Time.h"
#include "PPM.h"
#include "PID.h"
#include "MPU6500.h"
#include "Sonar.h"

#include "inc/hw_memmap.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

#define SYSCTL_PERIPH_LED SYSCTL_PERIPH_GPIOF
#define GPIO_LED_BASE     GPIO_PORTF_BASE
#define GPIO_RED_LED      GPIO_PIN_1
#define GPIO_BLUE_LED     GPIO_PIN_2
#define GPIO_GREEN_LED    GPIO_PIN_3

static const float angleKp = 4.0f;
static const float stickScalingRollPitch = 2.0f, stickScalingYaw = 2.0f;
static const float restAngleRoll = 3.62f, restAnglePitch = 0.20f;
static const uint8_t maxAngleInclination = 50.0f; // Max angle in self level mode

static float gyroRate[3], roll, pitch; // Gyro rate in deg/s and roll and pitch calculated using Kalman filter
static uint32_t imuTimer = 0, pidTimer = 0; // Used to keep track of the time

// Motor 0 is bottom right, motor 1 is top right, motor 2 is bottom left and motor 3 is top left
static float motors[4] = { -100.0f, -100.0f, -100.0f, -100.0f };
static bool armed = false;

void pidResetError(void) {
    pidRoll.integratedError = pidRoll.lastError = 0.0f;
    pidPitch.integratedError = pidPitch.lastError = 0.0f;
    pidYaw.integratedError = pidYaw.lastError = 0.0f;
}

void initPIDValues(void) {
    pidRoll.Kp = 0.2f;
    pidRoll.Ki = 0.8f;
    pidRoll.Kd = 0.0f;

    pidRoll.integrationLimit = 0.6f; // Prevent windup

    pidPitch = pidRoll; // Use same PID values for both pitch and roll

    // x2 the values work pretty well - TODO: Fine-tune these
    pidYaw = pidRoll;
    pidYaw.Kp *= 3.0f;
    pidYaw.Ki *= 3.5f; // I increased this in order for it to stop yawing slowly
    pidYaw.Kd *= 2.0f;

    pidResetError();
}

int main(void) {
    // Set the clocking to run directly from the external crystal/oscillator.
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set clock to 80MHz (400MHz(PLL) / 2 / 2.5 = 80 MHz)

    initPPM();
    initUART();
    delay(100); // It needs a little delay after UART has been enabled
    UARTprintf("Started\n");
    initTime();
    initRX();
    initSonar();

    initI2C();
    delay(100); // Wait a little bit for I2C to stabilize
    initMPU6500();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_LED); // Enable GPIOF peripheral
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
    GPIOPinTypeGPIOOutput(GPIO_LED_BASE, GPIO_RED_LED | GPIO_BLUE_LED | GPIO_GREEN_LED); // Set red, blue and green LEDs as outputs

    IntMasterEnable();

    delay(100); // Wait a little for everything to initialize

    UARTprintf("CLK %d\n", SysCtlClockGet());
    UARTprintf("min: %d, max: %d, period: %d\n", PPM_MIN, PPM_MAX, getPeriod());

    initPIDValues();
    printPIDValues();

    while (!validRXData || getRXChannel(RX_AUX2_CHAN) > 0) {
        // Wait until we have valid data and safety aux channel is in safe position
    }

    while (1) {
        checkUARTData();

        // Make sure there is valid data and safety channel is in armed position
        if (validRXData && getRXChannel(RX_AUX2_CHAN) > 0) {
            if (!armed && getRXChannel(RX_THROTTLE_CHAN) < -95 && getRXChannel(RX_RUDDER_CHAN) > 95) // Arm using throttle low and yaw right
                armed = true;
            else if (armed && getRXChannel(RX_THROTTLE_CHAN) < -95 && getRXChannel(RX_RUDDER_CHAN) < -95) // Disarm using throttle low and yaw left
                armed = false;
        } else
            armed = false;

        // Turn on red led if armed otherwise turn on green LED
        GPIOPinWrite(GPIO_LED_BASE, GPIO_RED_LED | GPIO_GREEN_LED, !armed ? GPIO_GREEN_LED : GPIO_RED_LED);

        uint32_t now = micros();
        if (dataReadyMPU6500()) {
            float dt = (float)(now - imuTimer) / 1000000.0f;
            //UARTprintf("%d\n", now - imuTimer);
            imuTimer = now;

            int16_t accData[3], gyroData[3];
            getMPU6500Data(accData, gyroData); // Get accelerometer and gyroscope values
            for (uint8_t axis = 0; axis < 3; axis++)
                gyroRate[axis] = ((float)gyroData[axis]) / 16.4f; // Convert to deg/s
            getMPU6500Angles(accData, gyroRate, &roll, &pitch, dt); // Calculate pitch and roll

            roll -= restAngleRoll; // Apply angle trim
            pitch -= restAnglePitch;

            /*UARTprintf("%d\t%d\t%d\n", gyroData[0], gyroData[1], gyroData[2]);
            UARTFlushTx(false);*/
            /*UARTprintf("%d.%02d\t%d.%02d\n", (int16_t)roll, (int16_t)abs(roll * 100.0f) % 100, (int16_t)pitch, (int16_t)abs(pitch * 100.0f) % 100);
            UARTFlushTx(false);*/
        }

        bool angleMode;
        if (rxChannel[RX_AUX1_CHAN] > RX_MID_INPUT) {
            angleMode = true;
            GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED if in angle mode
        } else {
            angleMode = false;
            GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED if in acro mode
        }

        now = micros();
        float dt = (float)(now - pidTimer);
        if (armed && dt > 2500) { // Limit to 2.5ms (400 Hz)
            //UARTprintf("%d\n", now - pidTimer);
            pidTimer = now;
            dt /= 1000000.0f; // Convert to seconds

            float aileron = getRXChannel(RX_AILERON_CHAN);
            float elevator = getRXChannel(RX_ELEVATOR_CHAN);
            float rudder = getRXChannel(RX_RUDDER_CHAN);
            //UARTprintf("%d\t%d\t%d\n", (int16_t)aileron, (int16_t)elevator, (int16_t)rudder);

            float setPoint[2];
            if (angleMode) { // Angle mode
                setPoint[0] = constrain(aileron, -maxAngleInclination, maxAngleInclination) - roll;
                setPoint[1] = constrain(elevator, -maxAngleInclination, maxAngleInclination) - pitch;
                setPoint[0] *= angleKp;
                setPoint[1] *= angleKp;
            } else { // Acro mode
                setPoint[0] = aileron * stickScalingRollPitch;
                setPoint[1] = elevator * stickScalingRollPitch;
            }

            /*UARTprintf("%d\t%d\n", (int16_t)setPoint[0], (int16_t)setPoint[1]);
            UARTFlushTx(false);*/

            // Roll and pitch control can both be gyro or accelerometer based
            float rollOut = updatePID(&pidRoll, setPoint[0], gyroRate[1], dt);
            float pitchOut = updatePID(&pidPitch, setPoint[1], gyroRate[0], dt);

            // Yaw is always gyro controlled
            float yawOut = updatePID(&pidYaw, rudder * stickScalingYaw, gyroRate[2], dt);

            float throttle = getRXChannel(RX_THROTTLE_CHAN);
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

            updateMotorsAll(motors);

            //UARTprintf("%d\t%d\n", (int16_t)elevator, (int16_t)aileron);
#if 0
            UARTprintf("%d\t%d\t\t", (int16_t)roll, (int16_t)pitch);
            UARTprintf("%d\t%d\t\t", (int16_t)rollOut, (int16_t)pitchOut);
            UARTprintf("%d\t%d\t%d\t%d\n", (int16_t)motors[0], (int16_t)motors[1], (int16_t)motors[2], (int16_t)motors[3]);
            UARTFlushTx(false);
#endif
        }

        triggerSonar(); // Trigger sonar
    }
}

// TODO:
    // Save PID values in EEPROM
    // Adjust PID values using pots on transmitter
    // Only enable peripheral clock once
    // Tune yaw PID values separately
    // Use SPI instead of I2C for MPU-6500
    // Set Kd as well
    // Scope PWM output and check that it is in sync with control loop
    // Define all pins in a pins.h
    // Use sonar distance for something usefull - see: https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/altitudehold.c
        // https://github.com/cleanflight/cleanflight/blob/master/src/main/sensors/sonar.c#L90-L99
    // Limit other motors if one reaches maximum: https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/mixer.c#L677-L684
    // Make acc calibration routine - apply zero values before Kalman routine
    // Retune PID again and tune stickscaling
    // Only have one Kalman.c file. Use struct as argument instead
    // Takes average of three readings in DTerm: https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/pid.c#L721-L732
    // Create Android App
    // Add buzzer. Beep on startup, turn on at gyro calibration error, connection loss etc.
    // Add disarm timer
    // Remove safety AUX channel once 100% stable
