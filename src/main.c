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

#include "EEPROM.h"
#include "I2C.h"
#include "MPU6500.h"
#include "PPM.h"
#include "PID.h"
#include "RX.h"
#include "Sonar.h"
#include "Time.h"
#include "UART.h"

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor

#define SYSCTL_PERIPH_LED SYSCTL_PERIPH_GPIOF
#define GPIO_LED_BASE     GPIO_PORTF_BASE
#define GPIO_RED_LED      GPIO_PIN_1
#define GPIO_BLUE_LED     GPIO_PIN_2
#define GPIO_GREEN_LED    GPIO_PIN_3

static float gyroRate[3], roll, pitch; // Gyro rate in deg/s and roll and pitch calculated using Kalman filter
static uint32_t imuTimer = 0, pidTimer = 0; // Used to keep track of the time

// Motor 0 is bottom right, motor 1 is top right, motor 2 is bottom left and motor 3 is top left
static float motors[4] = { -100.0f, -100.0f, -100.0f, -100.0f };
static bool armed = false;

int main(void) {
    // Set the clocking to run directly from the external crystal/oscillator.
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // Set clock to 80MHz (400MHz(PLL) / 2 / 2.5 = 80 MHz)

    initPPM();
    initUART();
    delay(100); // It needs a little delay after UART has been enabled
    UARTprintf("Started\n");
    initEEPROM();
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

    setDefaultPIDValues();
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
        GPIOPinWrite(GPIO_LED_BASE, GPIO_RED_LED | GPIO_GREEN_LED, armed ? GPIO_RED_LED : GPIO_GREEN_LED);

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

            /*UARTprintf("%d\t%d\t%d\n", gyroData[0], gyroData[1], gyroData[2]);
            UARTFlushTx(false);*/
            /*UARTprintf("%d.%02d\t%d.%02d\n", (int16_t)roll, (int16_t)abs(roll * 100.0f) % 100, (int16_t)pitch, (int16_t)abs(pitch * 100.0f) % 100);
            UARTFlushTx(false);*/
        }

        static bool angleMode = false;
        if (!angleMode && getRXChannel(RX_AUX1_CHAN) > -10) {
            angleMode = true;
            GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, GPIO_BLUE_LED); // Turn on blue LED if in angle mode
        } else if (angleMode) {
            angleMode = false;
            GPIOPinWrite(GPIO_LED_BASE, GPIO_BLUE_LED, 0); // Turn off blue LED if in acro mode
        }

        // Don't spin motors if the throttle is low
        bool runMotors = false;
        if (armed && getRXChannel(RX_THROTTLE_CHAN) > -95)
            runMotors = true;
        else {
            writePPMAllOff();
            resetPIDError();
        }

        if (runMotors) {
            now = micros();
            float dt = (float)(now - pidTimer);
            if (runMotors && dt > 2500) { // Limit to 2.5ms (400 Hz)
                //UARTprintf("%d\n", now - pidTimer);
                pidTimer = now;
                dt /= 1000000.0f; // Convert to seconds

                float aileron = getRXChannel(RX_AILERON_CHAN);
                float elevator = getRXChannel(RX_ELEVATOR_CHAN);
                float rudder = getRXChannel(RX_RUDDER_CHAN);
                //UARTprintf("%d\t%d\t%d\n", (int16_t)aileron, (int16_t)elevator, (int16_t)rudder);

                float setPoint[2];
                if (angleMode) { // Angle mode
                    setPoint[0] = constrain(aileron, -cfg.maxAngleInclination, cfg.maxAngleInclination) - roll;
                    setPoint[1] = constrain(elevator, -cfg.maxAngleInclination, cfg.maxAngleInclination) - pitch;
                    setPoint[0] *= cfg.angleKp;
                    setPoint[1] *= cfg.angleKp;
                } else { // Acro mode
                    setPoint[0] = aileron * cfg.stickScalingRollPitch;
                    setPoint[1] = elevator * cfg.stickScalingRollPitch;
                }

                /*UARTprintf("%d\t%d\n", (int16_t)setPoint[0], (int16_t)setPoint[1]);
                UARTFlushTx(false);*/

                // Roll and pitch control can both be gyro or accelerometer based
                float rollOut = updatePID(&cfg.pidRoll, setPoint[0], gyroRate[1], dt);
                float pitchOut = updatePID(&cfg.pidPitch, setPoint[1], gyroRate[0], dt);

                // Yaw is always gyro controlled
                float yawOut = updatePID(&cfg.pidYaw, rudder * cfg.stickScalingYaw, gyroRate[2], dt);

                float throttle = getRXChannel(RX_THROTTLE_CHAN);
                for (uint8_t i = 0; i < 4; i++)
                    motors[i] = throttle;

                // Apply mix for quadcopter in x-configuration
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
        }

        triggerSonar(); // Trigger sonar
    }
}

// TODO:
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
    // Retune PID again and tune stickscaling
    // Only have one Kalman.c file. Use struct as argument instead
    // Takes average of three readings in DTerm: https://github.com/cleanflight/cleanflight/blob/master/src/main/flight/pid.c#L721-L732
    // Create Android App
    // Add buzzer. Beep on startup, arm changed, turn on at gyro/acc calibration error, connection loss etc.
    // Add disarm timer
    // Remove safety AUX channel once 100% stable
